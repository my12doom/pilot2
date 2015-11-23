#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <HAL/Resources.h>
#include <utils/param.h>
#include <FileSystem/ff.h>
#include <utils/log.h>

using namespace HAL;
using namespace devices;
using namespace sensors;

// constants
#define THROTTLE_STOP 1100
#define THROTTLE_MAX 1900
static param THROTTLE_IDLE("idle", 1240);
static const float PI = acos(-1.0);
static param rc_setting[8][4] = 
{
	{param("rc00", 1000), param("rc01", 1520), param("rc02", 2000), param("rc03", 0),},
	{param("rc10", 1000), param("rc11", 1520), param("rc12", 2000), param("rc13", 0),},
	{param("rc20", 1000), param("rc21", 1520), param("rc22", 2000), param("rc23", 0),},
	{param("rc30", 1000), param("rc31", 1520), param("rc32", 2000), param("rc33", 0),},
	{param("rc40", 1000), param("rc41", 1520), param("rc42", 2000), param("rc43", 0),},
	{param("rc50", 1000), param("rc51", 1520), param("rc52", 2000), param("rc53", 0),},
	{param("rc60", 1000), param("rc61", 1520), param("rc62", 2000), param("rc63", 0),},
	{param("rc70", 1000), param("rc71", 1520), param("rc72", 2000), param("rc73", 0),},
};

class yet_another_pilot;
extern yet_another_pilot yap;

class yet_another_pilot
{
public:
	yet_another_pilot();
	~yet_another_pilot(){}

	// HAL / sensors
	ILED *state_led;
	ILED *SD_led;
	ILED *flashlight;
	IRCIN *rcin;
	IRCOUT *rcout;
	IRGBLED *rgb;
	HAL::IUART *vcp;// = NULL;

	// sensor data
	float rc[6];
	px4flow_frame frame;
	gyro_data gdata;
	accelerometer_data adata;
	mag_data mdata;
	baro_data bdata;
	bool new_baro_data;
	bool new_gps_data;
	float voltage;
	float current;

	// setup and loop functions
	int setup();
	void main_loop();
	void sdcard_logging_loop();
	static void main_loop_entry(){yap.main_loop();}
	static void sdcard_logging_loop_entry(){yap.sdcard_logging_loop();}

	// main loop sub routines.
	int estimate_states();
	int read_rc();
	int run_controllers();
	int output();
	int read_sensors();
	int read_flow();
	
	// helper variables
	float dt;
	int64_t last_tick;
	
	// helper functions
	float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert);
	int calculate_baro_altitude();
	float constrain(float v, float low, float high);
} yap;

yet_another_pilot::yet_another_pilot()
:voltage(0)
,current(0)
,last_tick(0)
{
}
// helper functions
float yet_another_pilot::constrain(float v, float low, float high)	
{
	if (v < low)
		return low;
	if (v > high)
		return high;
	return v;
}

int yet_another_pilot::output()
{
	int16_t ppm_output[4];
	for(int i=0; i<4; i++)
	{
		ppm_output[i] = THROTTLE_STOP;
	}
	return rcout->write(ppm_output, 0,  4);
}


int yet_another_pilot::estimate_states()
{
	// TODO ~~
	
	return 0;
}

int yet_another_pilot::run_controllers()
{
	// TODO ~~
	
	return 0;
}

int yet_another_pilot::read_flow()
{
	if (manager.get_flow_count())
	{
		sensors::IFlow *flow = manager.get_flow(0);

		if (flow->read_flow(&frame) < 0)
			return -1;
	}

	return 0;
}

int yet_another_pilot::read_sensors()
{
	// read gyros
	if (manager.get_gyroscope_count() < 1 || !manager.get_gyroscope(0) || manager.get_gyroscope(0)->read(&gdata) < 0)
		return -1;

	// read accelerometers
	if (manager.get_accelerometer_count() < 1 || !manager.get_accelerometer(0) || manager.get_accelerometer(0)->read(&adata) < 0)
		return -2;
	
	if (manager.get_magnetometer_count() < 1 || !manager.get_magnetometer(0) || manager.get_magnetometer(0)->read(&mdata) < 0)
		return -3;
	
	// read barometers
	if (manager.get_barometer_count() >= 1 && manager.get_barometer(0))
	{
		IBarometer *baro = manager.get_barometer(0);
		if (!baro->healthy())
			return -4;
		new_baro_data = 0 == baro->read(&bdata);
	}


	// read GPSs
	if (manager.get_GPS_count() >= 1 && manager.get_GPS(0))
	{
		IGPS *gps = manager.get_GPS(0);
		if (!gps->healthy())
			return -5;
	}


	// voltage and current sensors
	float alpha = dt / (dt + 1.0f/(2*PI * 2.0f));		// 2hz low pass filter
	if (manager.getBatteryVoltage("BatteryVoltage"))
		voltage = voltage * (1-alpha) + alpha * manager.getBatteryVoltage("BatteryVoltage")->read();
	if (manager.getBatteryVoltage("BatteryCurrent"))
		current = current * (1-alpha) + alpha * manager.getBatteryVoltage("BatteryCurrent")->read();	
	
	return 0;
}


float yet_another_pilot::ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = constrain(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}


int yet_another_pilot::read_rc()
{
	int16_t pwm_input[6];
	int64_t pwm_input_update[6];
	
	rcin->get_channel_data(pwm_input, 0, 6);
	rcin->get_channel_update_time(pwm_input_update, 0, 6);
	for(int i=0; i<6; i++)
	{
		rc[i] = ppm2rc(pwm_input[i], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);
	}

	rc[2] = (rc[2]+1)/2;	
		
	return 0;
}

void yet_another_pilot::main_loop(void)
{	
	// calculate systime dt
	static int64_t tic = 0;
	int64_t round_start_tick = systimer->gettime();
	dt = (round_start_tick-last_tick)/1000000.0f;
	last_tick = round_start_tick;
	
	// rc inputs
	read_rc();
		
	// read sensors
	read_sensors();
	read_flow();
	
	// all state estimating, AHRS, position, altitude, etc
	estimate_states();
	
	// all controlling
	run_controllers();
	
	// final output to motors
	output();
}

void yet_another_pilot::sdcard_logging_loop(void)
{
	log_flush();
}
int yet_another_pilot::setup(void)
{
	state_led = manager.getLED("state");
	SD_led = manager.getLED("SD");
	flashlight = manager.getLED("flashlight");
	rcin = manager.get_RCIN();
	rcout = manager.get_RCOUT();
	rgb = manager.getRGBLED("rgb");
	vcp = manager.getUART("VCP");
	
	// make sure all sensors are ready
	

	// get two timers, one for main loop and one for SDCARD logging loop(with different priority).
	manager.getTimer("mainloop")->set_period(1000);
	manager.getTimer("mainloop")->set_callback(main_loop_entry);
	manager.getTimer("log")->set_period(10000);
	manager.getTimer("log")->set_callback(sdcard_logging_loop_entry);
	
	return 0;
}

int main()
{
	bsp_init_all();
		
	yap.setup();
	while(1)
	{
	}
}

