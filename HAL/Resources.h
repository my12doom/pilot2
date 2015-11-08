#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4Interfaces.h>

#define MAX_ACCELEROMETER_COUNT 5
#define MAX_GPS_COUNT 3
#define LED_NUM 5
#define UART_NUM 4
#define TIMER_NUM 4
#define BATTERYVOLTAGE_NUM 4
#define MAX_FLOW_COUNT 2
#define MAX_DEVICE_COUNT 20

class Manager
{
	//Manager construct:
	public :
		Manager();
		~Manager(){};
			
		typedef struct{
			char name[10];
			void *p;
		} device_entry;
			
		typedef struct{
			char name[10];//name 
			devices::ILED *pLED;   //pointer
		}LED_table;
		typedef struct{
			char name[10];//name 
			devices::IRGBLED *pLED;   //pointer
		}RGBLED_table;
		typedef struct{
			char name[10];
			HAL::IUART *pUart; 
		}UART_table;
		typedef struct{
			char name[10];
			uint8_t num;
			HAL::ITimer *pTimer; 
		}Timer_table;
		typedef struct{
			char name[18];
			uint8_t num;
			devices::IBatteryVoltage *pIBatteryVoltage; 
		}BatteryVoltage_table;
	
	
	private:
		HAL::IRCIN *rcin;
		HAL::IRCOUT *rcout;
		LED_table led_table[LED_NUM];
		int led_num;
		RGBLED_table rgbled_table[LED_NUM];
		int rgbled_num;
		UART_table uart_table[UART_NUM];
		int uart_num;
		Timer_table timer_table[TIMER_NUM];
		int timer_num;
		BatteryVoltage_table batteryvoltage_table[BATTERYVOLTAGE_NUM];
		int batteryvoltage_num;
		int accelerometer_count;
		devices::IAccelerometer * accelerometers[MAX_ACCELEROMETER_COUNT];
		int gyroscope_count;
		devices::IGyro * gyroscopes[MAX_ACCELEROMETER_COUNT];	
		int barometer_count;
		devices::IBarometer * barometers[MAX_ACCELEROMETER_COUNT];	
		int magnetometer_count;
		devices::IMagnetometer * magnetometers[MAX_ACCELEROMETER_COUNT];
		int gps_count;
		devices::IGPS * GPSs[MAX_GPS_COUNT];
		HAL::IAsyncWorker * async_worker;
		int flow_count;
		sensors::IFlow * Flows[MAX_FLOW_COUNT];

		int device_count;
		device_entry devices[MAX_DEVICE_COUNT];
		
	
	public :
		int get_RGBLED_Num();
		int get_LED_Num();
		int get_UART_Num();
		int get_Timer_Num();
		int get_gyroscope_count();
		int get_accelerometer_count();
		int get_magnetometer_count();
		int get_flow_count();
		int get_barometer_count();
		int get_GPS_count();
		int get_device_count();
		
		//register function:
		int register_LED(const char *name,devices::ILED *pLED);
		int register_RGBLED(const char *name,devices::IRGBLED *pLED);
		int register_gyroscope(devices::IGyro *gyro);
		int register_accelerometer(devices::IAccelerometer *accel);
		int register_barometer(devices::IBarometer *baro);
		int register_magnetometer(devices::IMagnetometer *mag);
		int register_GPS(devices::IGPS *gps);
		int register_UART(const char *name,HAL::IUART *pUart);
		int register_Timer(const char *name,HAL::ITimer *pTimer);
		int register_BatteryVoltage(const char *name,devices::IBatteryVoltage *pIBatteryVoltage);
		int register_RCIN(HAL::IRCIN *rcin);
		int register_flow(sensors::IFlow *worker);
		int register_RCOUT(HAL::IRCOUT *rcout);
		int register_asyncworker(HAL::IAsyncWorker *worker);
		int register_device(const char *name, void *device);

		//getDevice function:
		void *get_device(const char *name);
		devices::ILED* getLED(const char *name);
		devices::IRGBLED* getRGBLED(const char *name);
		HAL::IUART *getUART(const char *name);
		HAL::ITimer *getTimer(const char *name);
		devices::IBatteryVoltage *getBatteryVoltage(const char *name);
		devices::IAccelerometer * get_accelerometer(int index);
		devices::IGyro * get_gyroscope(int index);
		devices::IMagnetometer * get_magnetometer(int index);
		devices::IBarometer * get_barometer(int index);
		devices::IGPS * get_GPS(int index);
		HAL::IRCIN * get_RCIN();
		HAL::IRCOUT * get_RCOUT();
		HAL::IAsyncWorker *get_asyncworker();
		sensors::IFlow *get_flow(int index);
};

//Declear manager as global:
extern Manager manager;

// board bsp implements and starts from this function
extern int bsp_init_all();

// board bsp implements this
extern void reset_system();