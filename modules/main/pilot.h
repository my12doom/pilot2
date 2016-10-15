#pragma once

#include <HAL/Interface/Interfaces.h>
#include <HAL/Resources.h>

#include <Algorithm/ahrs.h>
#include <Algorithm/attitude_controller.h>
#include <Algorithm/altitude_estimator.h>
#include <Algorithm/altitude_estimator2.h>
#include <Algorithm/altitude_controller.h>
#include <Algorithm/pos_estimator.h>
#include <Algorithm/pos_controll.h>
#include <Algorithm/pos_controll_old.h>
#include <Algorithm/of_controller.h>
#include <Algorithm/of_controller2.h>
#include <Algorithm/mag_calibration.h>
#include <Algorithm/motion_detector.h>
#include <Algorithm/battery_estimator.h>
#include <Algorithm/ekf_estimator.h>
#include <Algorithm/pos_estimator2.h>
#include <math/LowPassFilter2p.h>
#include <utils/fifo2.h>
#include <utils/ymodem.h>

#include "mode_basic.h"
#include "mode_althold.h"
#include "mode_of_loiter.h"
#include "mode_poshold.h"
#include "mode_RTL.h"


class yet_another_pilot;
extern yet_another_pilot yap;

enum pos_estimator_state
{
	none = 0,
	velocity_and_local = 1,	// optical flow or something
	transiting = 2,			// transiting between different position data source, a "braking" or velocity controll is recommended since velocity is available.
	fully_ready = 3,		// GPS
};

static const char * pos_estimator_state_str[] = 
{
	"none",
	"velocity_and_local",
	"transiting",
	"fully_ready",
};

enum acrobatic_moves_state
{
	acrobatic_none,
	acrobatic_flip_rising,
	acrobatic_flip_rotating,
};

enum acrobatic_moves
{
	acrobatic_move_flip,
};

class ymodem_firmware_writter : public ymodem_receiver
{
public:
	ymodem_firmware_writter(HAL::IUART *uart)
	:ymodem_receiver(uart)
	{
		last_event = systimer->gettime();
		firmware_size = 0;
	}

	~ymodem_firmware_writter()
	{

	}
	virtual int on_event(void *data, int datasize, ymodem_event event);
	int64_t last_event;
	int firmware_size;
};

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
	int rc_fail;	// 0 : RF controlling, 1: mobile controlling, 2: mobile overriding, -1: total failure
	float rc_fail_tick;
	int round_running_time;// = 0;
	devices::gps_data gps;
	bool new_gps_data;// = false;
	int critical_errors;// = 0;
	int cycle_counter;// = 0;
	int imu_counter;
	float ground_pressure;// = 0;
	float ground_temperature;// = 0;
	float rc[8];// = {0};			// ailerron : full left -1, elevator : full down -1, throttle: full down 0, rudder, full left -1
	float rc_mobile[4];// = {0};	// rc from mobile devices
	bool airborne;// = false;
	float takeoff_ground_altitude;// = 0;
	bool armed;// = false;
	int64_t last_arming_time;

	copter_mode flight_mode;// = basic;

	flight_mode_basic mode_basic;
	flight_mode_althold mode_althold;
	flight_mode_of_loiter mode_of_loiter;
	flight_mode_poshold mode_poshold;
	flight_mode_RTL mode_RTL;
	IFlightMode *modes[mode_MAX];


	int64_t collision_detected;// = 0;	// remember to clear it before arming
	int64_t tilt_us;// = 0;	// remember to clear it before arming
	math::LowPassFilter2p gyro_lpf2p[3];// = {LowPassFilter2p(1000, 40), LowPassFilter2p(1000, 40), LowPassFilter2p(1000, 40)};	// 2nd order low pass filter for gyro.
	math::LowPassFilter2p accel_lpf2p[3];// = {LowPassFilter2p(1000, 40), LowPassFilter2p(1000, 40), LowPassFilter2p(1000, 40)};	// 2nd order low pass filter for gyro.
	vector gyro_reading;			// gyro reading with temperature compensation and LPF, without AHRS bias estimating
	vector body_rate;				// body rate, with all compensation applied
	vector accel;// = {NAN, NAN, NAN};
	vector mag;
	vector gyro_uncalibrated;
	vector accel_uncalibrated;
	vector mag_uncalibrated;

	int64_t last_tick;// = 0;
	int64_t last_gps_tick;// = 0;
	pos_estimator estimator;
	pos_estimator2 estimator2;
	pos_controller_base *pos_control;
	pos_controller pos_control_hybird;
	pos_controller_old pos_control_loiter;
	attitude_controller attitude_controll;
	altitude_estimator alt_estimator;
	altitude_estimator2 alt_estimator2;
	altitude_controller alt_controller;
	OpticalFlowController2 of_controller;
	ekf_estimator ekf_est;
	battery_estimator batt;
	
	float ground_speed_north;		// unit: m/s
	float ground_speed_east;		// unit: m/s
	float ground_accel_north;		// unit: m/s/s
	float ground_accel_east;		// unit: m/s/s
	float climb_rate_gps;
	float gps_attitude_timeout;
	float voltage;// = 0;
	float current;// = 0;
	float interval;// = 0;
	float yaw_launch;
	float a_raw_pressure;// = 0;
	float b_raw_pressure;// = 0;
	float a_raw_temperature;// = 0;
	float a_raw_altitude;// = 0;
	float b_raw_altitude;// = 0;
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
	float v_flow_ned[3];//ned flow velocity
	int loop_hz;// = 0;
	vector acc_calibrator[6];						// accelerometer calibration data array.
	int acc_avg_count[6];// = {0};						// accelerometer calibration average counter.
	motion_detector motion_acc;						// motion detector for accelerometer calibration.
	bool islanding;// = false ;
	bool land_possible;
	bool imu_data_lock;
	int event_count;
	int events[10];
	int events_args[10];
	float home[2];
	float home_set;
	FIFO<2048> raw_imu_buffer;
	bool firmware_loading;		// loading firmware from uart
	bool mag_reset_requested;
	motion_detector detect_acc;
	motion_detector detect_gyro;
	acrobatic_moves_state acrobatic;
	float acrobatic_timer;
	float acrobatic_number;		// helper variable for acrobatic state tracking.
	int arcrobatic_arg;
	float m_rf_ok_ticker;

	// constructor
	yet_another_pilot();
	~yet_another_pilot(){}
	
	// setup and loop functions
	int setup();
	void main_loop();
	void sdcard_loop();
	void mag_calibrating_worker();
	static void mag_calibrating_worker_entry(int parameter){((yet_another_pilot*)parameter)->mag_calibrating_worker();}
	static void main_loop_entry(){yap.main_loop();}
	static void sdcard_loop_entry(){yap.sdcard_loop();}
	static void imu_reading_entry(){;}
	
	// mag and accelerometer calibration
	int sensor_calibration();
	int finish_accel_cal();
	void reset_accel_cal();
	void reset_mag_cal();
	void cancel_mag_cal();
	
	// automated functions
	int prepare_for_throw() { pending_throwgo = true; return 0;}
	int start_taking_off();
	int start_acrobatic(acrobatic_moves move, int arg);
	
	// main loop sub routines.
	int read_rc();
	int light_words();
	int lowpower_handling();
	int crash_detector();
	int land_detector();
	int check_stick_action();
	int disarm();
	int arm(bool arm = true, bool forced = false);
	int set_mode(copter_mode newmode);
	int calculate_state();
	int read_sensors();
	int read_imu_and_filter();		// should be called from a seperate thread(timer) with higher than read_sensors()'s priority
	int run_controllers();
	int output();
	void handle_takeoff();
	int save_logs();
	int decide_mode_switching();
	int execute_mode_switching();		// should and only should be called by handle_mode_switching() or arm(), when armed, airborne state, position estimator state, or mode switch changed.
	int handle_acrobatic();
	copter_mode mode_from_stick();

	copter_mode last_mode_from_switch;
	bool last_airborne;
	bool pending_throwgo;
	pos_estimator_state last_position_state;

	// event handling
	int new_event(int event, int arg);
	int handle_events();
	
	// UART functions
	int handle_cli(HAL::IUART *uart);
	int handle_uart4_controll();
	int handle_wifi_controll(HAL::IUART *uart);
	int stupid_joystick();
		
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
	int default_alt_controlling();
	int get_pos_velocity_ned(float *pos, float *velocity);
	int get_home(float *home_pos);
	int set_home(const float *new_home);
	int set_home_LLH(const float *LLH);
	pos_estimator_state get_estimator_state();

	
//protected:
};

enum yap_events
{
	event_arm,
	event_disarm,
	event_airborne,
	event_touchdown,
	event_pos_ready,
	event_pos_bad,
	event_home_set,
	event_mode_switch_changed,
};

