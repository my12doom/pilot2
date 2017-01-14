// NRF24L01 / BK5811 reciever
// resource requirement:
// 1 SPI, 3 GPIO (CSN, CE, IRQ), 1 EXTI(IRQ), 1 timer for frequency hooping
// realtime performance tolerance is ~400us for NRF24L01, ~half of that for BK5811 

#pragma once
#include <HAL/Interface/Interfaces.h>
#include <math/randomizer.h>
#include <HAL/aux_devices/NRF24L01.h>

namespace sensors
{
	class NRFIn : public HAL::IRCIN
	{
	public:
		NRFIn();
		~NRFIn();

		int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce, HAL::IInterrupt *irq, HAL::ITimer *timer);

		// set randomizer seed for frequency hooping.
		// default seed is 0x1234567890345678
		int set_seed(int64_t seed);

		// set AES key for encryption, pass NULL to disable encryption.
		// encryption is disabled by default.
		int set_key(void *key, int keylen);	

		// get max latency of last one second
		int get_latency();

		// total channel count
		virtual int get_channel_count();

		// return num channel written to out pointer
		virtual int get_channel_data(int16_t *out, int start_channel, int max_count);

		// return num channel written to out pointer
		virtual int get_channel_update_time(int64_t *out, int start_channel, int max_count);
			
		// statistics functions is mainly for RC calibration purpose.
		virtual int get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count);
		virtual int reset_statistics();

		virtual HAL::RCIN_State state();
	protected:

		static void nrf_irq_entry(void *parameter, int flags){((NRFIn*)parameter)->nrf_irq_handler();}
		static void timer_entry(void *user_data){((NRFIn*)user_data)->timer_handler();}
		
		void nrf_irq_handler();
		void timer_handler();
		int hoop_to(int next_hoop_id);

		
		int64_t last_packet_time;
		int16_t rc_static[2][6];
		int16_t valid_data[6];

		randomizer<128, 512> rando;
		devices::NRF24L01 nrf;
		int hoop_interval;
		int hoop_id;
		int miss;
		int maxmiss;
		
		//HAL::IGPIO *cs;
		HAL::IGPIO *ce;
		//HAL::IGPIO *irq;
		//HAL::IGPIO *dbg;
		//HAL::IGPIO *dbg2;
		HAL::IInterrupt *interrupt;
		HAL::ITimer *timer;
		
		int o;		
		int rdp;
	};
}
