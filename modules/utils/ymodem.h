#pragma once
#include "IUART.h"

enum ymodem_state
{
	ymodem_wait_file_header,
	ymodem_wait_packets,
	ymodem_transition_ended,
	ymodem_state_error,
};

enum ymodem_event
{
	ymodem_file_header,
	ymodem_file_data,
	ymodem_bad_packet,
	ymodem_end_of_transition,
	ymodem_error,
};

class ymodem_receiver
{
public:
	virtual int on_event(void *data, int datasize, ymodem_event event){return 0;}

	ymodem_receiver(HAL::IUART *uart);
	~ymodem_receiver();

	int reset();
	int run();
	int set_timeout(int timeout){return this->timeout = timeout;}

protected:
	int timeout;
	HAL::IUART *uart;
	int state;
	int64_t Ctimeout;
	uint8_t data[1030];
	int error_count;
	uint8_t blk_number;
};