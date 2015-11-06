#pragma once

#include <HAL/Interface/Interfaces.h>
#include <HAL/Resources.h>

#include <Algorithm/ahrs.h>
#include <Algorithm/attitude_controller.h>
#include <Algorithm/altitude_estimator.h>
#include <Algorithm/altitude_estimatorCF.h>
#include <Algorithm/altitude_controller.h>
#include <Algorithm/pos_estimator.h>
#include <Algorithm/pos_controll.h>
#include <Algorithm/of_controller.h>
#include <Algorithm/mag_calibration.h>
#include <Algorithm/motion_detector.h>

#include <math/LowPassFilter2p.h>

class yet_another_pilot;
extern yet_another_pilot yap;

class yet_another_pilot
{
public:

	devices::ILED *state_led;
	devices::ILED *SD_led;
	devices::ILED *flashlight;
	HAL::IRCIN *rcin;
	HAL::IRCOUT *rcout;
	devices::IRGBLED *rgb;
	devices::IRangeFinder * range_finder;// = NULL;
	mag_calibration mag_calibrator;
	HAL::IUART *vcp;// = NULL;


	int64_t g_pwm_input_update[16];//
	int16_t g_ppm_output[16];//
	int16_t g_pwm_input[16];//
	
	
	// states
	int mag_calibration_state;// = 0;			// 0: not running, 1: collecting data, 2: calibrating
	int last_mag_calibration_result;// = 0xff;	// 0xff: not calibrated at all, other values from mag calibration.
	int usb_data_publish;// = 0;
	int lowpower;// = 0;		// lowpower == 0:power good, 1:low power, no action taken, 2:low power, action taken.
	bool acc_cal_requested;// = false;
	bool acc_cal_done;// = false;
	bool motor_saturated;// = false;
	bool rc_fail;// = false;
	int round_running_time;// = 0;
	devices::gps_data gps;
	bool new_gps_data;// = false;
	int critical_errors;// = 0;
	int cycle_counter;// = 0;
	float ground_pressure;// = 0;
	float ground_temperature;// = 0;
	float rc[8];// = {0};			// ailerron : full left -1, elevator : full down -1, throttle: full down 0, rudder, full left -1
	float rc_mobile[4];// = {0};	// rc from mobile devices
	float accelz;// = 0;
	bool airborne;// = false;
	float takeoff_ground_altitude;// = 0;
	bool armed;// = false;
	copter_mode submode;// = basic;
	int64_t collision_detected;// = 0;	// remember to clear it before arming
	int64_t tilt_us;// = 0;	// remember to clear it before arming
	math::LowPassFilter2p gyro_lpf2p[3];// = {LowPassFilter2p(333.3, 40), LowPassFilter2p(333.3, 40), LowPassFilter2p(333.3, 40)};	// 2nd order low pass filter for gyro.
	vector gyro_reading;			// gyro reading with temperature compensation and LPF, without AHRS bias estimating
	vector body_rate;				// body rate, with all compensation applied
	vector accel;// = {NAN, NAN, NAN};
	vector mag;
	vector gyro_uncalibrated;
	vector accel_uncalibrated;
	vector mag_uncalibrated;
	vector accel_earth_frame;

	int64_t last_tick;// = 0;
	int64_t last_gps_tick;// = 0;
	pos_estimator estimator;
	pos_controller controller;
	attitude_controller attitude_controller;
	altitude_estimator alt_estimator;
	altitude_estimatorCF alt_estimatorCF;
	altitude_controller alt_controller;
	OpticalFlowController of_controller;
	float ground_speed_north;		// unit: m/s
	float ground_speed_east;		// unit: m/s
	float ground_accel_north;		// unit: m/s/s
	float ground_accel_east;		// unit: m/s/s
	float voltage;// = 0;
	float current;// = 0;
	float interval;// = 0;
	float yaw_launch;
	float a_raw_pressure;// = 0;
	float a_raw_temperature;// = 0;
	float a_raw_altitude;// = 0;
	bool new_baro_data;// = false;
	float throttle_real;// = 0;
	float throttle_result;// = 0;
	float mah_consumed;// = 0;
	float wh_consumed;// = 0;
	float sonar_distance;// = NAN;
	float bluetooth_roll;// = 0;
	float bluetooth_pitch;// = 0;
	int64_t bluetooth_last_update;// = 0;
	int64_t mobile_last_update;// = 0;
	vector gyro_temp_k;// = {0};		// gyro temperature compensating curve (linear)
	vector gyro_temp_a;// = {0};
	float temperature0;// = 0;
	float mpu6050_temperature;
	vector imu_statics[3][4];// = {0};		//	[accel, gyro, mag][min, current, max, avg]
	int avg_count;// = 0;
	sensors::px4flow_frame frame;
	int loop_hz;// = 0;
	vector acc_calibrator[6];						// accelerometer calibration data array.
	int acc_avg_count[6];// = {0};						// accelerometer calibration average counter.
	motion_detector motion_acc;						// motion detector for accelerometer calibration.
	bool islanding;// = false ;

	// constructor
	yet_another_pilot();
	~yet_another_pilot(){}
	
	// setup and loop functions
	int setup();
	void main_loop();
	void sdcard_logging_loop();
	static void mag_calibrating_worker_entry(int parameter){((yet_another_pilot*)parameter)->mag_calibrating_worker();}
	static void main_loop_entry(){yap.main_loop();}
	static void sdcard_logging_loop_entry(){yap.sdcard_logging_loop();}
	void mag_calibrating_worker();
	
	// mag and accelerometer calibration
	int sensor_calibration();
	int finish_accel_cal();
	void reset_accel_cal();
	void reset_mag_cal();
	
	// automated functions
	int start_taking_off();
	
	// main loop sub routines.
	int read_rc();
	int light_words();
	int lowpower_handling();
	int crash_detector();
	int land_detector();
	int check_stick();
	int disarm();
	int arm(bool arm = true);
	int set_submode(copter_mode newmode);
	int calculate_state();
	int read_sensors();
	int run_controllers();
	int output();
	void handle_takeoff();
	int save_logs();
	copter_mode submode_from_stick();
	
	// UART functions
	int handle_cli(HAL::IUART *uart);
	int handle_uart4_controll();
	int handle_wifi_controll();
		
	// helper functions
	float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert);	
	static int min(int a, int b){return a>b?b:a;}
	static int max(int a, int b){return a<b?b:a;}
	static float fmin(float a, float b){return a > b ? b : a;}
	static float fmax(float a, float b){return a > b ? a : b;}
	int send_package(const void *data, uint16_t size, uint8_t type, HAL::IUART*uart);
	void output_rc();
	void STOP_ALL_MOTORS();
	int calculate_baro_altitude();
	
//protected:
};

