// (c)2015 befinitiv

/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not, write to the Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include "fec.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <sys/mman.h>

#include "lib.h"
#include "wifibroadcast.h"
#include "radiotap.h"

extern uint32_t errorBytes;
extern uint32_t recvBytes; 
#define MAX_PACKET_LENGTH 4192
#define MAX_USER_PACKET_LENGTH 1450
#define MAX_DATA_OR_FEC_PACKETS_PER_BLOCK 32

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)



#define COL(x)  "\033[;" #x "m"
#define RED     COL(31)
#define GREEN   COL(32)
#define YELLOW  COL(33)
#define BLUE    COL(34)
#define MAGENTA COL(35)
#define CYAN    COL(36)
#define WHITE   COL(0)
#define GRAY    "\033[0m"

// this is where we store a summary of the
// information from the radiotap header

typedef struct  {
	int m_nChannel;
	int m_nChannelFlags;
	int m_nRate;
	int m_nAntenna;
	int m_nRadiotapFlags;
} __attribute__((packed)) PENUMBRA_RADIOTAP_DATA;



int flagHelp = 0;
int param_port = 0;
int param_data_packets_per_block = 8;
int param_fec_packets_per_block = 4;
int param_block_buffers = 1;
int param_packet_length = MAX_USER_PACKET_LENGTH;
int max_block_num = -1;

void
usage(void)
{
	printf(
	    "(c)2015 befinitiv. Based on packetspammer by Andy Green.  Licensed under GPL2\n"
	    "\n"
	    "Usage: rx [options] <interfaces>\n\nOptions\n"
			"-p <port> Port number 0-255 (default 0)\n"
			"-b <count> Number of data packets in a block (default 8). Needs to match with tx.\n"
	    "-r <count> Number of FEC packets per block (default 4). Needs to match with tx.\n\n"
	    "-f <bytes> Number of bytes per packet (default %d. max %d). This is also the FEC block size. Needs to match with tx\n"
			"-d <blocks> Number of transmissions blocks that are buffered (default 1). This is needed in case of diversity if one adapter delivers data faster than the other. Note that this increases latency\n"
	    "Example:\n"
	    "  iwconfig wlan0 down\n"
	    "  iw dev wlan0 set monitor otherbss fcsfail\n"
	    "  ifconfig wlan0 up\n"
			"  iwconfig wlan0 channel 13\n"
	    "  rx wlan0        Receive raw packets on wlan0 and output the payload to stdout\n"
	    "\n", MAX_USER_PACKET_LENGTH, MAX_USER_PACKET_LENGTH);
	exit(1);
}

typedef struct {
	pcap_t *ppcap;
	int selectable_fd;
	int n80211HeaderLength;
} monitor_interface_t;

typedef struct {
	int block_num;
	packet_buffer_t *packet_buffer_list;
} block_buffer_t;

void open_and_configure_interface(const char *name, int port, monitor_interface_t *interface) {
	struct bpf_program bpfprogram;
	char szProgram[512];
	char szErrbuf[PCAP_ERRBUF_SIZE];
		// open the interface in pcap

	szErrbuf[0] = '\0';
	interface->ppcap = pcap_open_live(name, 2048, 1, -1, szErrbuf);
	if (interface->ppcap == NULL) {
		fprintf(stderr, "Unable to open interface %s in pcap: %s\n",
		    name, szErrbuf);
		exit(1);
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
			exit(1);

	}

	if (pcap_compile(interface->ppcap, &bpfprogram, szProgram, 1, 0) == -1) {
		puts(szProgram);
		puts(pcap_geterr(interface->ppcap));
		exit(1);
	} else {
		if (pcap_setfilter(interface->ppcap, &bpfprogram) == -1) {
			fprintf(stderr, "%s\n", szProgram);
			fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
		} else {
		}
		pcap_freecode(&bpfprogram);
	}

	interface->selectable_fd = pcap_get_selectable_fd(interface->ppcap);
}

void process_packet(monitor_interface_t *interface, block_buffer_t *block_buffer_list, int adapter_no) {
		struct pcap_pkthdr * ppcapPacketHeader = NULL;
		struct ieee80211_radiotap_iterator rti;
		PENUMBRA_RADIOTAP_DATA prd;
		u8 payloadBuffer[MAX_PACKET_LENGTH];
		u8 *pu8Payload = payloadBuffer;
		int bytes;
		int n;
		int retval;
		int u16HeaderLen;
		static int c = 0;

		// receive

		retval = pcap_next_ex(interface->ppcap, &ppcapPacketHeader,
		    (const u_char**)&pu8Payload);

		if (retval < 0) {
			fprintf(stderr, "Socket broken\n");
			fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
			exit(1);
		}
		if(ppcapPacketHeader->caplen != ppcapPacketHeader->len)
		{
			fprintf(stderr,BLUE"cap len%d len %d\033[0m\n",ppcapPacketHeader->caplen, ppcapPacketHeader->len);
		}
		//if(retval == 0)
		//	fprintf(stderr, "retval = 0\n");

		if (retval != 1)
			return;


		u16HeaderLen = (pu8Payload[2] + (pu8Payload[3] << 8)); //u16HeaderLen == 21
		if (ppcapPacketHeader->len <
		    (u16HeaderLen + interface->n80211HeaderLength))
			return;
		//len : packet length -- caplen: capture length
		bytes = ppcapPacketHeader->len -
			(u16HeaderLen + interface->n80211HeaderLength);
		if (bytes < 0)
			return;

		if (ieee80211_radiotap_iterator_init(&rti,
		    (struct ieee80211_radiotap_header *)pu8Payload,
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
				    le16_to_cpu(*((u16 *)rti.this_arg));
				prd.m_nChannelFlags =
				    le16_to_cpu(*((u16 *)(rti.this_arg + 2)));
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
		pu8Payload += u16HeaderLen + interface->n80211HeaderLength;

		if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
		{
			bytes -= 4;
		}
        int checksum_correct = (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_BADFCS) == 0;

		//calculate error bytes here
		//countError(pu8Payload+8,bytes-8,CHECKVAL);
        //process_payload(pu8Payload, bytes, checksum_correct, block_buffer_list, adapter_no);
		printf("\r %dth packet, %d bytes, crc %s", c++, bytes, checksum_correct ? "OK" : "FAIL");
		if (!checksum_correct)
			printf("\n");
}

int
main2(int argc, char *argv[])
{
	monitor_interface_t interfaces[MAX_PENUMBRA_INTERFACES];
	int num_interfaces = 0;
	int i;
    block_buffer_t *block_buffer_list;

	
	while (1) {
		int nOptionIndex;
		static const struct option optiona[] = {
			{ "help", no_argument, &flagHelp, 1 },
			{ 0, 0, 0, 0 }
		};
		int c = getopt_long(argc, argv, "hp:b:d:r:f:",
			optiona, &nOptionIndex);

		if (c == -1)
			break;
		switch (c) {
		case 0: // long option
			break;

		case 'h': // help
			usage();

		case 'p': //port
			param_port = atoi(optarg);
			break;
		
		case 'b': 
			param_data_packets_per_block = atoi(optarg);
			break;

		case 'r': 
			param_fec_packets_per_block = atoi(optarg);
			break;
		
		case 'd':
            param_block_buffers = atoi(optarg);
			break;
		
		case 'f': // MTU
			param_packet_length = atoi(optarg);
			break;

		default:
			fprintf(stderr, "unknown switch %c\n", c);
			usage();
			break;
		}
	}

	if (optind >= argc)
		usage();
	
	
	if(param_packet_length > MAX_USER_PACKET_LENGTH) {
		printf("Packet length is limited to %d bytes (you requested %d bytes)\n", MAX_USER_PACKET_LENGTH, param_packet_length);
		return (1);
	}


	fec_init();

	int x = optind;
	while(x < argc && num_interfaces < MAX_PENUMBRA_INTERFACES) {
		open_and_configure_interface(argv[x], param_port, interfaces + num_interfaces);
		++num_interfaces;
		++x;
	}
	
    //block buffers contain both the block_num as well as packet buffers for a block.
    block_buffer_list = malloc(sizeof(block_buffer_t) * param_block_buffers);
    for(i=0; i<param_block_buffers; ++i)
	{
        block_buffer_list[i].block_num = -1;
        block_buffer_list[i].packet_buffer_list = lib_alloc_packet_buffer_list(param_data_packets_per_block+param_fec_packets_per_block, MAX_PACKET_LENGTH);
	}


	for(;;) { 
		fd_set readset;
		struct timeval to;

		to.tv_sec = 0;
		to.tv_usec = 1e5;
	
		FD_ZERO(&readset);
		for(i=0; i<num_interfaces; ++i)
			FD_SET(interfaces[i].selectable_fd, &readset);

		int n = select(30, &readset, NULL, NULL, &to);

		for(i=0; i<num_interfaces; ++i) 
		{
			if(n == 0)
				break;
			if(FD_ISSET(interfaces[i].selectable_fd, &readset)) 
			{
                process_packet(interfaces + i, block_buffer_list, i);
			}
		}

	}

	return (0);
}
