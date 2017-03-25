#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#include <pcap/pcap.h>

/*
//prism header
#define DNAMELEN 16 // Device name length

#define PRISM_MSGCODE 0x0041 // Monitor Frame
#define PRISM_DID_HOSTTIME 0x1041 // Host time element
#define PRISM_DID_MACTIME 0x2041 // Mac time element
#define PRISM_DID_CHANNEL 0x3041 // Channel element
#define PRISM_DID_RSSI 0x4041 // RSSI element
#define PRISM_DID_SQ 0x5041 // SQ element
#define PRISM_DID_SIGNAL 0x6041 // Signal element
#define PRISM_DID_NOISE 0x7041 // Noise element
#define PRISM_DID_RATE 0x8041 // Rate element
#define PRISM_DID_ISTX 0x9041 // Is Tx frame
#define PRISM_DID_FRMLEN 0xA041 // Frame length

#define PRISM_STATUS_OK 0 // Prism Status: the associated prism_value is supplied
#define PRISM_STATUS_NO_VALUE 1 // Prism Status: the associated prism_value is NOT supplied


struct prism_value
{
 uint32_t did; // This has a different ID for each parameter
 uint16_t status; // 0 = set;  1 = not set (yes - not what you expected)
 uint16_t len; // length of the data (u32) used 0-4
 uint32_t data; // The data value
} __attribute__ ((packed));

struct prism_header
{
 uint32_t msgcode;             // = PRISM_MSGCODE
 uint32_t msglen;     // The length of the entire header - usually 144 bytes = 0x90
 char devname[DNAMELEN];       // The name of the device that captured this packet
 struct prism_value hosttime;  // This is measured in jiffies - I think
 struct prism_value mactime;   // This is a truncated microsecond timer,
                                  // we get the lower 32 bits of a 64 bit value
 struct prism_value channel;
 struct prism_value rssi;
 struct prism_value sq;
 struct prism_value signal;
 struct prism_value noise;
 struct prism_value rate;
 struct prism_value istx;
 struct prism_value frmlen;
 char   dot_11_header[];
} __attribute__ ((packed));
*/
//this sits at the payload of the wifi packet (outside of FEC)
typedef struct {
    uint32_t sequence_number;
} __attribute__((packed)) wifi_packet_header_t;

#define MAX_PACKET_LENGTH 4192
#define MAX_USER_PACKET_LENGTH 1450

/* this is the template radiotap header we send packets out with */

static const uint8_t uint8_taRadiotapHeader[] = {

	0x00, 0x00, // <-- radiotap version
	0x0c, 0x00, // <- radiotap header lengt
	0x04, 0x80, 0x00, 0x00, // <-- bitmap
	0x22, 		// rate
	0x0, 		// txpower
	0x18,		// rtx_retries
	0x00,		// data_retries
};

/* Penumbra IEEE80211 header */

//the last byte of the mac address is recycled as a port number
#define SRC_MAC_LASTBYTE 15
#define DST_MAC_LASTBYTE 21

static uint8_t uint8_taIeeeHeader[] = {
	0x08, 0x01, 		// FC: frame control
	0x00, 0x00,			// DID: duration or ID
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	// address1
	0x13, 0x22, 0x33, 0x44, 0x55, 0x66,	// address2
	0x13, 0x22, 0x33, 0x44, 0x55, 0x66,	// address3
	0x10, 0x86,			// SC: Sequence control
};



int flagHelp = 0;

void set_port_no(uint8_t *pu, uint8_t port) {
	//dirty hack: the last byte of the mac address is the port number. this makes it easy to filter out specific ports via wireshark
	pu[sizeof(uint8_taRadiotapHeader) + SRC_MAC_LASTBYTE] = port;
	pu[sizeof(uint8_taRadiotapHeader) + DST_MAC_LASTBYTE] = port;
}


int packet_header_init(uint8_t *packet_header) {
	uint8_t *puint8_t = packet_header;
	memcpy(packet_header, uint8_taRadiotapHeader, sizeof(uint8_taRadiotapHeader));
	puint8_t += sizeof(uint8_taRadiotapHeader);
	memcpy(puint8_t, uint8_taIeeeHeader, sizeof (uint8_taIeeeHeader));
	puint8_t += sizeof (uint8_taIeeeHeader);
			
	//determine the length of the header
	return puint8_t - packet_header;
}

void pb_transmit_packet(pcap_t *ppcap, int seq_nr, uint8_t *packet_transmit_buffer, int packet_header_len, const uint8_t *packet_data, int packet_length) {
    //add header outside of FEC
    wifi_packet_header_t *wph = (wifi_packet_header_t*)(packet_transmit_buffer + packet_header_len);
    wph->sequence_number = seq_nr;

    //copy data
    memcpy(packet_transmit_buffer + packet_header_len + sizeof(wifi_packet_header_t), packet_data, packet_length);

    int plen = packet_length + packet_header_len + sizeof(wifi_packet_header_t);
    int r = pcap_inject(ppcap, packet_transmit_buffer, plen);
    if (r != plen) {
        pcap_perror(ppcap, "Trouble injecting packet");
        return;
    }
}

static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}


extern "C" int
tx(int argc, char *argv[])
{
	char szErrbuf[PCAP_ERRBUF_SIZE];
	int i;
	pcap_t *ppcap = NULL;
	char fBrokenSocket = 0;
	int pcnt = 0;
	time_t start_time;
    uint8_t packet_transmit_buffer[MAX_PACKET_LENGTH];
	size_t packet_header_length = 0;


    packet_header_length = packet_header_init(packet_transmit_buffer);

	// open the interface in pcap
	szErrbuf[0] = '\0';
	ppcap = pcap_open_live(argv[optind], 800, 1, 20, szErrbuf);
	if (ppcap == NULL) {
		fprintf(stderr, "Unable to open interface %s in pcap: %s\n",
		    argv[optind], szErrbuf);
		return (1);
	}

	int nLinkEncap = pcap_datalink(ppcap);

	switch (nLinkEncap) {
		case DLT_PRISM_HEADER:
			fprintf(stderr, "DLT_PRISM_HEADER Encap\n");			
			break;

		case DLT_IEEE802_11_RADIO:
			fprintf(stderr, "DLT_IEEE802_11_RADIO Encap\n");
			break;

		default:
			fprintf(stderr, "!!! unknown encapsulation on !\n");

	}
	pcap_setnonblock(ppcap, 0, szErrbuf);


	i=0;
	int64_t t = getus();
	while(1)
	{
		uint8_t data[1024];
		for(int i=0; i<sizeof(data); i++)
			data[i] = rand();
		pb_transmit_packet(ppcap, 0, packet_transmit_buffer, packet_header_length, data, sizeof(data));
		usleep(100);
		i++;
		if (getus() - t > 1000000)
		{
			printf("payload %dKBytes(%d Kbps)/s\n", i, i*8);
			i = 0;
			t = getus();
		}
	}

	return (0);
}

//////////////////////////
//////      RX SIDE
//////////////////////////

#include "ieee80211_radiotap.h"
extern "C" 
{
	#include "wifibroadcast/radiotap.h"
}
typedef struct {
	pcap_t *ppcap;
	int selectable_fd;
	int n80211HeaderLength;
} monitor_interface_t;

// this is where we store a summary of the
// information from the radiotap header
typedef struct  {
	int m_nChannel;
	int m_nChannelFlags;
	int m_nRate;
	int m_nAntenna;
	int m_nRadiotapFlags;
} __attribute__((packed)) PENUMBRA_RADIOTAP_DATA;


// BYTE ORDER
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define	le16_to_cpu(x) (x)
#define	le32_to_cpu(x) (x)
#else
#define	le16_to_cpu(x) ((((x)&0xff)<<8)|(((x)&0xff00)>>8))
#define	le32_to_cpu(x) \
((((x)&0xff)<<24)|(((x)&0xff00)<<8)|(((x)&0xff0000)>>8)|(((x)&0xff000000)>>24))
#endif
#define	unlikely(x) (x)


int open_and_configure_interface(const char *name, int port, monitor_interface_t *interface) {
	struct bpf_program bpfprogram;
	char szProgram[512];
	char szErrbuf[PCAP_ERRBUF_SIZE];
		// open the interface in pcap

	szErrbuf[0] = '\0';
	interface->ppcap = pcap_open_live(name, 2048, 1, -1, szErrbuf);
	if (interface->ppcap == NULL) {
		fprintf(stderr, "Unable to open interface %s in pcap: %s\n",
		    name, szErrbuf);
		return -1;
	}
	

	if(pcap_setnonblock(interface->ppcap, 1, szErrbuf) < 0) {
		fprintf(stderr, "Error setting %s to nonblocking mode: %s\n", name, szErrbuf);
	}

	int nLinkEncap = pcap_datalink(interface->ppcap);

	switch (nLinkEncap) {

		case DLT_PRISM_HEADER:
			fprintf(stderr, "DLT_PRISM_HEADER Encap\n");
			interface->n80211HeaderLength = 0x20; // ieee80211 comes after this
			sprintf(szProgram, "radio[0x4a:4]==0x13223344 && radio[0x4e:2] == 0x55%.2x", port);
			break;

		case DLT_IEEE802_11_RADIO:
			fprintf(stderr, "DLT_IEEE802_11_RADIO Encap\n");
			interface->n80211HeaderLength = 0x18; // ieee80211 comes after this
			sprintf(szProgram, "ether[0x0a:4]==0x13223344 && ether[0x0e:2] == 0x55%.2x", port);
			break;

		default:
			fprintf(stderr, "!!! unknown encapsulation on %s !\n", name);
			return -2;
	}

	if (pcap_compile(interface->ppcap, &bpfprogram, szProgram, 1, 0) == -1) {
		puts(szProgram);
		puts(pcap_geterr(interface->ppcap));
		return -3;
	} else {
		if (pcap_setfilter(interface->ppcap, &bpfprogram) == -1) {
			fprintf(stderr, "%s\n", szProgram);
			fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
		} else {
		}
		pcap_freecode(&bpfprogram);
	}

	interface->selectable_fd = pcap_get_selectable_fd(interface->ppcap);

	return 0;
}

void process_packet(monitor_interface_t *interface) {
	struct pcap_pkthdr * ppcapPacketHeader = NULL;
	struct ieee80211_radiotap_iterator rti;
	PENUMBRA_RADIOTAP_DATA prd;
	uint8_t payloadBuffer[MAX_PACKET_LENGTH];
	uint8_t *puint8_tPayload = payloadBuffer;
	int bytes;
	int n;
	int retval;
	int uint16_tHeaderLen;
	static int c = 0;

	// receive

	retval = pcap_next_ex(interface->ppcap, &ppcapPacketHeader,
		(const u_char**)&puint8_tPayload);

	if (retval < 0) {
		fprintf(stderr, "Socket broken\n");
		fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
		return;
	}
	if(ppcapPacketHeader->caplen != ppcapPacketHeader->len)
	{
		fprintf(stderr, "cap len%d len %d\033[0m\n",ppcapPacketHeader->caplen, ppcapPacketHeader->len);
	}
	//if(retval == 0)
	//	fprintf(stderr, "retval = 0\n");

	if (retval != 1)
		return;


	uint16_tHeaderLen = (puint8_tPayload[2] + (puint8_tPayload[3] << 8)); //uint16_tHeaderLen == 21
	if (ppcapPacketHeader->len <
		(uint16_tHeaderLen + interface->n80211HeaderLength))
		return;
	//len : packet length -- caplen: capture length
	bytes = ppcapPacketHeader->len -
		(uint16_tHeaderLen + interface->n80211HeaderLength);
	if (bytes < 0)
		return;

	if (ieee80211_radiotap_iterator_init(&rti,
		(struct ieee80211_radiotap_header *)puint8_tPayload,
		ppcapPacketHeader->len) < 0)
	{
		return;
	}
	while ((n = ieee80211_radiotap_iterator_next(&rti)) == 0) {

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
			prd.m_nRadiotapFlags = *rti.this_arg;
			break;

		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
//				rx_status->adapter[adapter_no].current_signal_dbm = (int8_t)(*rti.this_arg);
			break;

		}
	}
	puint8_tPayload += uint16_tHeaderLen + interface->n80211HeaderLength;

	if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
	{
		bytes -= 4;
	}
	int checksum_correct = (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_BADFCS) == 0;

	static int64_t t = getus();
	static int size = 0;
	size += bytes;
	if (getus() - t > 1000000)
	{
		printf("\r  %d bytes/s   \n", size);
		t = getus();
		size = 0;
	}


	//calculate error bytes here
	//countError(puint8_tPayload+8,bytes-8,CHECKVAL);
	//process_payload(puint8_tPayload, bytes, checksum_correct, block_buffer_list, adapter_no);
	printf("\r %dth packet, %d bytes, crc %s", c++, bytes, checksum_correct ? "OK" : "FAIL");
	if (!checksum_correct)
		printf("\n");
}

extern "C" int rx(int argc, char *argv[])
{
	monitor_interface_t minterface;

	open_and_configure_interface(argv[1], 0x66, &minterface);
	
	for(;;) { 
		fd_set readset;
		struct timeval to;

		to.tv_sec = 0;
		to.tv_usec = 1e5;
	
		FD_ZERO(&readset);
		FD_SET(minterface.selectable_fd, &readset);

		int n = select(30, &readset, NULL, NULL, &to);

		if(FD_ISSET(minterface.selectable_fd, &readset))
		{
			process_packet(&minterface);
		}
	}

	return (0);
}


/////////////////////
/////////////////////

int test_pcap_block_device();

extern "C" int
main(int argc, char *argv[])
{
	usleep(10000000);
	printf("start ifconfig config\n");
	system("ifconfig wlan0 down");
	system("iwconfig wlan0 mode monitor");
	system("ifconfig wlan0 up");
	system("iwconfig wlan0 rate 12M");
	system("iwconfig wlan0 channel 9");
	printf("end ifconfig config\n");

	printf("main2\n");
	return test_pcap_block_device();
	for(int i=0; i<argc; i++)
	{
		if (strcmp("-tx", argv[i]) == 0)
		{
			printf("main2: TX\n");
			return tx(argc, argv);
		}
		if (strcmp("-rx", argv[i]) == 0)
		{
			printf("main2: RX\n");
			return rx(argc, argv);
		}
		if (strcmp("-arx", argv[i]) == 0)
		{
			printf("main2: ARX\n");
			return test_pcap_block_device();
		}
	}
	printf("main2: main\n");

	return 0;
}