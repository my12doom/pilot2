#include "Apcap.h"
#include <unistd.h>
#include <string.h>
#include "radiotap.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>		/* for ioctl() */

using namespace std;
using namespace HAL;

static uint8_t uint8_taRadiotapHeader[] = {			// radiotap TX header
	0x00, 0x00, // <-- radiotap version
	0x0c, 0x00, // <- radiotap header length
	0x00, 0x80, 0x08, 0x00, 	// <-- bitmap: tx flag(15), mcs(19)
	0x08, 0x00,							// tx flags, no retry
	0x1f, 0x10|0x08|0x04 | 0x01, 0,		// MCS2, 20Mhz BW, short GI, LDPC, HT greenfield
} ;

static uint8_t radiotap_channel[] = {			// radiotap TX header
	0x00, 0x00, // <-- radiotap version
	0x0c, 0x00, // <- radiotap header length
	0x08, 0x00, 0x00, 0x00, 	// <-- bitmap: channel(3)
	0x7C, 0x15, 0x08, 0x00,			// channel 5500Mhz, 5Mhz bw, no offset
} ;

typedef struct  {
	int m_nChannel;
	int m_nChannelFlags;
	int m_nRate;
	int m_nAntenna;
	int m_nRadiotapFlags;
	int m_rssi;
} __attribute__((packed)) PENUMBRA_RADIOTAP_DATA;

typedef union {
	struct {
		unsigned FragmentNumber: 4;
		unsigned  SequenceNumber: 12;
	};
	uint16_t usValue;
} DOT11_SEQUENCE_CONTROL;

static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}
/* Penumbra IEEE80211 header */
static uint8_t uint8_taIeeeHeader[] = {
	//0xD0, 0x01, 						// FC: frame control
	0x08, 0x01,
	0x00, 0x00,							// DID: duration or ID
	0x00, 0x13, 0xef, 0xf1, 0x00, 0x06,	// address1 (RX)
	//0x13, 0x22, 0x33, 0x44, 0x55, 0x66,
	//0x70, 0xf1, 0x1c, 0x2e, 0x51, 0x4a,
	0x70, 0xf1, 0x1c, 0x2e, 0x51, 0x50,
	//0x13, 0x22, 0x33, 0x44, 0x55, 0x66,	// address2 (TX)
	//0x00, 0x13, 0xef, 0xf1, 0x00, 0x06,	// address3 (DEST)
	0x13, 0x22, 0x33, 0x44, 0x55, 0x66,
	0x10, 0x86,							// SC: c
};

namespace androidUAV
{

APCAP_RX::APCAP_RX(const char*interface /*= INADDR_ANY*/, int port /*= 0xbbb*/)
{
	pthread_mutex_init(&cs, NULL);

	init_ok = false;
	byte_counter = 0;

	char szErrbuf[PCAP_ERRBUF_SIZE] = {0};
	ppcap = pcap_open_live(interface, MAX_PACKET_LENGTH, 1, -1, szErrbuf);
	if (ppcap == NULL)
		goto fail;

	if (pcap_datalink(ppcap) == DLT_IEEE802_11_RADIO)
	{
		n80211HeaderLength = sizeof(uint8_taIeeeHeader);
		char szErrbuf[PCAP_ERRBUF_SIZE];
		char szProgram[512];
		struct bpf_program bpfprogram;
		sprintf(szProgram, "ether[0x0a:4]==0x13223344 && ether[0x0e:2] == 0x55%.2x", 0x66);
		pcap_compile(ppcap, &bpfprogram, szProgram, 1, 0);
		//pcap_setfilter(ppcap, &bpfprogram);
		pcap_freecode(&bpfprogram);
	}
	else if (pcap_datalink(ppcap) == DLT_PRISM_HEADER)
	{
		strcpy(szErrbuf, "no DLT_PRISM_HEADER encapsulation support yet\n");
		goto fail;
	}

	else
	{
		strcpy(szErrbuf, "unsupported encapsulation\n");
		goto fail;
	}

	pcap_setnonblock(ppcap, 0, szErrbuf);
	selectable_fd = pcap_get_selectable_fd(ppcap);

	init_ok = true;
	worker_run = true;
	pthread_create(&worker_thread, NULL, worker_entry, this);
	fprintf(stderr, "APCAP_RX: init OK\n");
	
	// TX part
	{
	uint16_t *psize = (uint16_t*)(uint8_taRadiotapHeader + 2);
	*psize = sizeof(uint8_taRadiotapHeader);
	init_ok = false;
	memcpy(packet_transmit_buffer, uint8_taRadiotapHeader, sizeof(uint8_taRadiotapHeader));
	memcpy(packet_transmit_buffer+sizeof(uint8_taRadiotapHeader), uint8_taIeeeHeader, sizeof(uint8_taIeeeHeader));

	printf("header=%d+%d bytes\n", sizeof(uint8_taRadiotapHeader), sizeof(uint8_taIeeeHeader));

	int buf_size = 0;
	setsockopt(pcap_fileno(ppcap), SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));
	}

	init_ok = true;
	strcpy(ifname, interface);
	return;

fail:
	init_ok = false;
	fprintf(stderr, "APCAP_RX: init failed, error=%s\n", szErrbuf);
}

APCAP_RX::~APCAP_RX()
{
	clearup();
	pthread_mutex_destroy(&cs);
}

int APCAP_RX::clearup()
{
	// close socket and signal the worker thread to exit
	worker_run = false;
	pcap_close(ppcap);
	
	// wait for thread to exit
	if (NULL != worker_thread)
		pthread_join(worker_thread, NULL);
	worker_thread = NULL;

	return 0;
}

// read a block from rx queue, remove block from the queue if remove == true.
// returns num bytes read, negative values for error.
int APCAP_RX::read(void *buf, int max_block_size, bool remove /*= true*/)
{
	pthread_mutex_lock(&cs);

	if (packets.size() == 0)
	{
		pthread_mutex_unlock(&cs);
		return error_no_more_blocks;
	}

	packet p = packets.front();
	if (remove)
		packets.pop_front();
	int size = min(max_block_size, p.size);
	memcpy(buf, p.data, size);
	pthread_mutex_unlock(&cs);

	return size;
}

// query num available blocks in rx queue, negative values for error.
int APCAP_RX::available()
{
	pthread_mutex_lock(&cs);

	int count = packets.size();

	pthread_mutex_unlock(&cs);

	return count;
}

void* APCAP_RX::worker()
{
	while(worker_run)
	{
		// read from pcap

		// timeout = 10ms
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 1e5;

		// use select fd for timeouts
		fd_set readset;
		FD_ZERO(&readset);
		FD_SET(selectable_fd, &readset);

		select(30, &readset, NULL, NULL, &timeout);

		// got one packet?
		if(FD_ISSET(selectable_fd, &readset))
		{

			// read it
			uint8_t payloadBuffer[MAX_PACKET_LENGTH];
			uint8_t *puint8_tPayload = payloadBuffer;
			struct pcap_pkthdr * ppcapPacketHeader = NULL;
			int retval = pcap_next_ex(ppcap, &ppcapPacketHeader, (const u_char**)&puint8_tPayload);


			// dump for debugging purpose
			/*
			printf("payload:");
			for(int i=0; i<20; i++)
				printf("%02x,", puint8_tPayload[i]);
			printf("\n");
			*/

			// check for errors
			if (retval < 0 || retval != 1)
			{
				printf("Socket broken: %s\n", pcap_geterr(ppcap));
				continue;
			}

			if(ppcapPacketHeader->caplen != ppcapPacketHeader->len)
			{
				printf("cap len%d len %d\033[0m\n",ppcapPacketHeader->caplen, ppcapPacketHeader->len);
				continue;
			}

			uint16_t uint16_tHeaderLen = (puint8_tPayload[2] + (puint8_tPayload[3] << 8)); //radio tap header length field
			if (ppcapPacketHeader->len < (uint16_tHeaderLen + n80211HeaderLength))
			{
				//printf("ppcapPacketHeader has less than expected header length\n");
				continue;
			}

			// retrive radiotap header fields
			PENUMBRA_RADIOTAP_DATA prd;
			struct ieee80211_radiotap_iterator rti;
			if (ieee80211_radiotap_iterator_init(&rti,(struct ieee80211_radiotap_header *)puint8_tPayload,ppcapPacketHeader->len) < 0)
				continue;

			while (ieee80211_radiotap_iterator_next(&rti) == 0) {

				switch (rti.this_arg_index) {
				case IEEE80211_RADIOTAP_RATE:
					prd.m_nRate = (*rti.this_arg);
					break;

				case IEEE80211_RADIOTAP_CHANNEL:
					prd.m_nChannel =
						le16_to_cpu(*((uint16_t *)rti.this_arg));
					prd.m_nChannelFlags =
						le16_to_cpu(*((uint16_t *)(rti.this_arg + 2)));
					break;

				case IEEE80211_RADIOTAP_ANTENNA:
					prd.m_nAntenna = (*rti.this_arg) + 1;
					break;

				case IEEE80211_RADIOTAP_FLAGS:
					//printf("flag:%d\n", *rti.this_arg);
					prd.m_nRadiotapFlags = *rti.this_arg;
					break;

				case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
					latest_rssi = (int8_t)(*rti.this_arg);
					break;
				}
			}

			int byte_count = ppcapPacketHeader->len - uint16_tHeaderLen - n80211HeaderLength;
			if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
				byte_count -= 4;

			bool checksum_correct = (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_BADFCS) == 0;

			if (byte_count <= 0)
				continue;

			packet p;
			p.size = byte_count;
			memcpy(p.data, puint8_tPayload + uint16_tHeaderLen + n80211HeaderLength, byte_count);

			pthread_mutex_lock(&cs);

			if (p.size > 0)
			{
				if (packets.size() > 1024)
					packets.erase(packets.begin());
				packets.push_back(p);
			}

			pthread_mutex_unlock(&cs);

			static int i = 0;
			//printf("\rRX:%d%s", i++, checksum_correct?"OK":"NK\n");
			//if (!checksum_correct)
			//	printf("NK\n");


			byte_counter += byte_count;
			static int64_t last_bandwidth = getus();
			if(getus() - last_bandwidth > 1000000)
			{
				last_bandwidth = getus();
				//printf("%d bytes/s\n", byte_counter);
				byte_counter = 0;
			}
		}
	}

	return 0;
}

// write a block
// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
// returns num bytes written, negative values for error.
int APCAP_RX::write(const void *buf, int block_size)
{
	if (!init_ok)
		return error_unsupported;

	// update sequence_number
	DOT11_SEQUENCE_CONTROL *sc = (DOT11_SEQUENCE_CONTROL *)(packet_transmit_buffer + sizeof(uint8_taRadiotapHeader) + 22);
	sc->SequenceNumber ++;	

	memcpy(packet_transmit_buffer + sizeof(uint8_taRadiotapHeader) + sizeof(uint8_taIeeeHeader), buf, block_size);
	int plen = sizeof(uint8_taRadiotapHeader) + sizeof(uint8_taIeeeHeader) + block_size;
	int r = pcap_inject(ppcap, packet_transmit_buffer, plen);
    if (r != plen) {
        pcap_perror(ppcap, "Trouble injecting packet");
        return -2;
    }

	uint8_t *data = (uint8_t*)buf;
	//printf("data:%d/%d\n", data[0], data[1]);

	return 0;
}

int APCAP_RX::set_mcs_bw(int mcs, int bw)
{
	packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-1] = mcs;
	packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-2] &= ~0x3;

	if (bw == 40)
		packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-2] |= 1;

	//packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-3] |= 0x20; // STBC known
	packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-2] &= ~0x60;
	packet_transmit_buffer[sizeof(uint8_taRadiotapHeader)-2] |= 2 << 5;	// STBC = 2

	return 0;
}

int APCAP_RX::set_rf(int frequency, uint8_t offset, uint8_t bw)
{
	uint8_t buffer[100];

	radiotap_channel[10] = (offset&3) | ((bw&3)<<2);
	radiotap_channel[11] = 0;

	uint16_t *psize = (uint16_t*)(radiotap_channel + 2);
	*psize = sizeof(radiotap_channel);

	*(uint16_t*)(radiotap_channel + 8) = frequency;

	printf("*psize =%d, flags=%02x,%02x\n", *psize, radiotap_channel[10], radiotap_channel[11]);

	memcpy(buffer, radiotap_channel, sizeof(radiotap_channel));
	memcpy(buffer+sizeof(radiotap_channel), uint8_taIeeeHeader, sizeof(uint8_taIeeeHeader));

	pcap_inject(ppcap, buffer, sizeof(buffer));
	usleep(10000);

	return 0;
}
#include <linux/wireless.h>	/* for "struct iwreq" et al */

int wlan_ioctl_mp(
		int skfd,
		char *ifname,
		void *pBuffer,
		unsigned int BufferSize)
{
	int err;
	struct ifreq ifr;
	

	union iwreq_data u;
	err = 0;

	memset(&u, 0, sizeof(union iwreq_data));
	u.data.pointer = pBuffer;
	u.data.length = (unsigned short)BufferSize;

	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, ifname, strlen(ifname));
	ifr.ifr_ifru.ifru_data = &u;

	err = ioctl(skfd, SIOCDEVPRIVATE+2, &ifr);

	if (u.data.length == 0)
		*(char*)pBuffer = 0;
			 
	return err;
}

int APCAP_RX::set_rf2(int frequency, uint8_t offset, uint8_t bw, int8_t ant)
{
	uint8_t buffer[100];
	if (bw == 0)
		bw = 1;
	else if (bw == 1)
		bw = 6;
	else if (bw == 2)
		bw = 5;
		

	struct 
	{
		uint16_t frequency;
		uint8_t offset;
		uint8_t bw;
		int8_t ant_tx;
		int8_t ant_rx;
	} set_rf_para = {frequency, offset, bw, ant, -1};


	//int ret = ::ioctl(pcap_fileno(ppcap), SIOCDEVPRIVATE+2, &set_rf_para);
	//pcap_inject(ppcap, buffer, sizeof(buffer));
	int ret = wlan_ioctl_mp(pcap_fileno(ppcap), ifname, &set_rf_para, sizeof(set_rf_para));
	//usleep(10000);

	return ret;
}

}
