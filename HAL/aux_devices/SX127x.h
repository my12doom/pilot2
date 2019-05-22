#pragma once

#include <stdint.h>

#include <HAL/Interface/IBlockDevice.h>
#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>
#include <HAL/Interface/IInterrupt.h>
#include <HAL/Interface/ITimer.h>
#include <modules/utils/fifo.h>
#include <modules/utils/AES.h>


#define SX127xManager_TX_QUEUE 2
#define SX127xManager_RX_QUEUE 5
#define GFSK_THRESHOLD 48


class SX127x
{
public:
	SX127x();
	~SX127x();

	// optional txen & rxen
	int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *txen = 0, HAL::IGPIO *rxen = 0);

	// shared
	int write_reg(uint8_t reg, uint8_t v);
	uint8_t read_reg(uint8_t reg);
	int config_DIO(int DIO_index, int source);		// see table 18 in datasheet for detailed mapping
	int set_frequency(float MHz, float crystal=32.0f);
	uint8_t get_mode();
	void set_mode(uint8_t mode);
	int set_tx_power(int dbm);							// for PA_BOOST only(2~17dbm or 20dbm) and OCP setting

	// FSK/LORA switching
	int set_lora_mode(bool lora);

	// LORA 
	int set_rate(int BW, int CR, int SF);				// BW, CR, SF
	void self_check();

	// FSK
	int set_rate(int bitrate, float crystal = 32.0f);

	
	int get_rssi();										// RSSI in dbm
	bool has_pending_rx();
	bool tx_done();
	void clear_tx_done_flag();
	void dio1_int(bool rx_mode);

	// IBlockDevice
	virtual int write(const void *buf, int block_size);						// write FIFO!
	virtual int read(void *buf, int max_block_size, bool remove = true);	// read FIFO
	virtual int available();

protected:

	HAL::ISPI *spi;
	HAL::IGPIO *cs;
	HAL::IGPIO *txen;
	HAL::IGPIO *rxen;


	bool lora_mode;
	uint8_t regs[256];

	int write_fifo(const void *buf, int size);
	int read_fifo(void *buf, int size);
	void _set_mode(uint8_t mode);

	uint8_t gfsk_fifo[256];
	int gfsk_fifo_size;
};

enum sx127x_mode
{
	mode_sleep = 0,
	mode_standby = 1,
	mode_prepare_tx = 2,
	mode_tx = 3,
	mode_prepare_rx = 4,
	mode_rx = 5,
	mode_rx_single = 6,
	mode_CAD = 7,
};

typedef struct
{
	int size;		// data size
	uint8_t data[255];
	float frequency;
	int16_t power;	// 2-17,20, or 0("don't change") for TX, RSSI for rx
	int16_t retries_left;
	int64_t next_tx;
	uint8_t mcs;	// not used yet
} sx127x_packet;

class SX127xManager
{
public:
	SX127xManager();
	~SX127xManager();

	int init(SX127x *x, HAL::IInterrupt * interrupt, HAL::ITimer *timer, HAL::IInterrupt * DIO1);
	int write(sx127x_packet p, int priority);
	int txqueue_space(int priority);		// remaining free space of TX queue
	int txqueue_total(int priority);		// total space of TX queue
	int rxqueue_count();					// RX available packet count
	int flush();
	int read(sx127x_packet *p);
	int cancel_current_packet();		// not supported by sx1278
	int set_aes(uint8_t *key, int keysize){aes.set_key(key, keysize*8); return 0;}
	int set_lora_mode(bool lora_mode);
	void set_tx_interval(int new_tx_interval){tx_interval = new_tx_interval;}
	bool ready_for_next_tx();

	int set_frequency(float tx_frequency, float rx_frequency);

protected:
	CircularQueue<sx127x_packet, SX127xManager_TX_QUEUE> tx_queue[2];		// [0 ~ 1] : priority
	CircularQueue<sx127x_packet, SX127xManager_RX_QUEUE> rx_queue;
	SX127x *x;
	HAL::ITimer *timer;
	HAL::IInterrupt *interrupt;
	HAL::IInterrupt *DIO1;
	int tx_interval;
	int64_t last_tx_done_time;

	static void int_entry(void *parameter, int flags){((SX127xManager*)parameter)->_int(flags);}
	static void int_entry_DIO1(void *parameter, int flags){((SX127xManager*)parameter)->_int_DIO1(flags);}
	static void timer_entry(void *p){((SX127xManager*)p)->tim();}
	void _int(int flags);
	void _int_DIO1(int flags);
	void tim();
	void state_maching_go();

	int stuck;
	bool fromint;

	bool use_aes;// = false;
	AESCryptor2 aes;

	uint8_t mode;	// cached SX127x mode register
	float tx_frequency;
	float rx_frequency;
};
