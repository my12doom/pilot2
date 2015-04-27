#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <Protocol/RFData.h>
#include <Protocol/common.h>
#include <utils/vector.h>
#include <utils/param.h>
#include <utils/log.h>
#include <utils/console.h>
#include <utils/gauss_newton.h>

#include <Algorithm/ahrs.h>
#include <Algorithm/ahrs2.h>
#include <Algorithm/altitude_estimator.h>
#include <Algorithm/altitude_estimatorCF.h>
#include <Algorithm/altitude_controller.h>
#include <Algorithm/pos_estimator.h>
#include <Algorithm/pos_controll.h>
#include <Algorithm/of_controller.h>
#include <Algorithm/mag_calibration.h>

#include <HAL/Interface/Interfaces.h>
#include <BSP/Resources.h>
using namespace HAL;
using namespace devices;
#include <FileSystem/ff.h>

#define THROTTLE_STOP (max((int)(rc_setting[2][0]-20),1000))
#define THROTTLE_MAX (min((int)(rc_setting[2][2]-20),2000))
#define SAFE_ON(x) if(x) (x)->on()
#define SAFE_OFF(x) if(x) (x)->off()

ILED *state_led;
ILED *SD_led;
ILED *flashlight;
IRCIN *rcin;
IRCOUT *rcout;
IRGBLED *rgb;
int mag_calibration_state = 0;		// 0: not running, 1: collecting data, 2: calibrating
mag_calibration mag_calibrator;

extern "C"
{

	#include <HAL/STM32F4/usb_comF4/cdc/usbd_cdc_core.h>
	#include <HAL/STM32F4/usb_comF4/core/usbd_usr.h>
	#include <HAL/STM32F4/usb_comF4/usb_conf/usbd_desc.h>
	#include <HAL/STM32F4/usb_comF4/usb_conf/usb_conf.h>

	#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
	#if defined ( __ICCARM__ ) //!< IAR Compiler 
	#pragma data_alignment=4
	#endif
	#endif // USB_OTG_HS_INTERNAL_DMA_ENABLED

	__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
}


// parameters

static param crash_protect("prot", 0);		// crash protection

static param simple_mode("simp", 0.0f) ;
static param pid_factor[3][4] = 			// pid_factor[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("rP1",0.60), param("rI1",0.80), param("rD1",0.03),param("rM1",PI)},
	{param("rP2",0.60), param("rI2",0.80), param("rD2",0.03),param("rM2",PI)},
	{param("rP3",1.75), param("rI3",0.25), param("rD3",0.01),param("rM3",PI)},
};
static param pid_factor2[3][4] = 			// pid_factor2[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("sP1", 6), param("sI1", 0), param("sD1", 0),param("sM1", PI/45)},
	{param("sP2", 6), param("sI2", 0), param("sD2", 0),param("sM2", PI/45)},
	{param("sP3", 8), param("sI3", 0), param("sD3", 0),param("sM3", PI/45)},
};
static param quadcopter_max_climb_rate("maxC",5);
static param quadcopter_max_descend_rate("maxD", 2);
static param quadcopter_trim[3] = 
{
	param("trmR", 0 * PI / 18),				// roll
	param("trmP", 0 * PI / 180),			// pitch
	param("trmY", 0.0f),					// yaw
};
static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

static param _gyro_bias[2][4] =	//[p1,p2][temperature,g0,g1,g2]
{
	{param("gbt1", NAN), param("gb11", 0), param("gb21", 0), param("gb31", 0),},
	{param("gbt2", NAN), param("gb12", 0), param("gb22", 0), param("gb32", 0),},
};

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

static param QUADCOPTER_MAX_YAW_OFFSET("offy", PI/4);
static param QUADCOPTER_ACRO_YAW_RATE("raty", PI);
static param hall_sensor_sensitivity("hall", 0.0666f);
static param motor_matrix("mat", 0);
static param THROTTLE_IDLE("idle", 1176);
static param acc_bias[3] = 
{
	param("abix", 0), param("abiy", 0), param("abiz", 0),
};
static param acc_scale[3] = 
{
	param("ascx", 1), param("ascy", 1), param("ascz", 1),
};
static param mag_bias[3] = 
{
	param("mbx", 0), param("mby", 0), param("mbz", 0),
};
static param mag_scale[3] = 
{
	param("mgx", 1), param("mgy", 1), param("mgz", 1),
};
#define MAX_MOTOR_COUNT 8
static float quadcopter_mixing_matrix[2][MAX_MOTOR_COUNT][3] = // the motor mixing matrix, [motor number] [roll, pitch, yaw]
{
	{							// + mode
		{0, -1, +1},			// rear, CCW
		{-1, 0, -1},			// right, CW
		{0, +1, +1},			// front, CCW
		{+1, 0, -1},			// left, CW
	},
	{							// X mode
		{-0.707f,-0.707f,+0.707f},				//REAR_R, CCW
		{-0.707f,+0.707f,-0.707f},				//FRONT_R, CW
		{+0.707f,+0.707f,+0.707f},				//FRONT_L, CCW
		{+0.707f,-0.707f,-0.707f},				//REAR_L, CW
	}
};

float PI180 = 180/PI;

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}
static int16_t max(int16_t a, int16_t b)
{
	return a<b?b:a;
}
static float f_min(float a, float b)
{
	return a > b ? b : a;
}
static float f_max(float a, float b)
{
	return a > b ? a : b;
}

void led_all_on()
{
	SAFE_ON(state_led);
	SAFE_ON(SD_led);
}

void led_all_off()
{
	SAFE_OFF(state_led);
	SAFE_OFF(SD_led);
}

int64_t g_pwm_input_update[16] = {0};
int16_t g_ppm_output[16] = {0};
int16_t g_pwm_input[16] = {0};

void output_rc()
{
	rcout->write(g_ppm_output, 0,  min(rcout->get_channel_count(), countof(g_ppm_output)));
}

void STOP_ALL_MOTORS()
{
	for(int i=0; i<16; i++)
		g_ppm_output[i] = THROTTLE_STOP;
	output_rc();
}
int handle_uart4_cli();
int handle_uart4_controll();

// states
devices::gps_data gps;
bool new_gps_data = false;
int critical_errors = 0;
float angle_pos[3] = {0};
float angle_target_unrotated[3] = {0};	// for quadcopter only currently, for fixed-wing, pos is also angle_pos
float angle_target[3] = {0};	// for quadcopter only currently, for fixed-wing, pos is also angle_pos
float angle_error[3] = {0};
float angle_errorD[3] = {0};
float angle_errorI[3] = {0};
float pos[3] = {0};
float target[3] = {0};		// target [roll, pitch, yaw] (pid controller target, can be angle or angle rate)
int cycle_counter = 0;
float ground_pressure = 0;
float ground_temperature = 0;
float rc[8] = {0};			// ailerron : full left -1, elevator : full down -1, throttle: full down 0, rudder, full left -1
float rc_mobile[4] = {0};	// rc from mobile devices
float accelz = 0;
bool airborne = false;
float takeoff_ground_altitude = 0;
fly_mode mode = initializing;
copter_mode submode = basic;
int64_t collision_detected = 0;	// remember to clear it before arming
int64_t tilt_us = 0;	// remember to clear it before arming
bool gyro_bias_estimating_end = false;
vector gyro_radian;
vector accel = {NAN, NAN, NAN};
vector mag;
vector accel_uncalibrated;
vector mag_uncalibrated;
float mag_radius = -999;
vector accel_earth_frame_mwc;
vector accel_earth_frame;
vector mag_earth_frame;
bool new_baro_data = false;
baro_data baro_reading;
int64_t time;
float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]
const int lpf_order = 5;
float errorD_lpf[lpf_order][3] = {0};			// variable for high order low pass filter, [order][roll, pitch, yaw]

int64_t last_tick = 0;
int64_t last_gps_tick = 0;
static unsigned short gps_id = 0;
pos_estimator estimator;
pos_controller controller;
altitude_estimator alt_estimator;
altitude_estimatorCF alt_estimatorCF;
altitude_controller alt_controller;
OpticalFlowController of_controller;
float ground_speed_north;		// unit: m/s
float ground_speed_east;		// unit: m/s
float airspeed_sensor_data;
float voltage = 0;
float current = 0;
float interval = 0;

int64_t last_rc_work = 0;
float yaw_launch;
float pid_result[3] = {0}; // total pid for roll, pitch, yaw

float a_raw_pressure = 0;
float a_raw_temperature = 0;
float a_raw_altitude = 0;



float throttle_real = 0;
float throttle_result = 0;

float mah_consumed = 0;
float wh_consumed = 0;

float sonar_distance = NAN;
int64_t last_sonar_time = 0;

short adxrs453_value = 0;
short mpu9250_value[7] = {0};
bool has_5th_channel = true;
bool has_6th_channel = true;

float bluetooth_roll = 0;
float bluetooth_pitch = 0;
int64_t bluetooth_last_update = 0;
int64_t mobile_last_update = 0;


vector gyro_temp_k = {0};		// gyro temperature compensating curve (linear)
vector gyro_temp_a = {0};
float temperature0 = 0;
float mpu6050_temperature;

volatile vector imu_statics[3][4] = {0};		//	[accel, gyro, mag][min, current, max, avg]
int avg_count = 0;
sensors::px4flow_frame frame;
int loop_hz = 0;

bool islanding = false ;
int calculate_baro_altitude()
{
	// raw altitude
	double scaling = (double)a_raw_pressure / ground_pressure;
	float temp = ((float)ground_temperature) + 273.15f;
	a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
	if (fabs(a_raw_altitude) < 5.0f)
		ground_temperature = a_raw_temperature;
	else
		printf("WTF?");

	return 0;
}

int prepare_pid()
{
	// calculate current core pid position

	float new_angle_pos[3] = {euler[0], euler[1], euler[2]};

	// the quadcopter's main pid lock on angle rate
	for(int i=0; i<3; i++)
	{
		pos[i] = ::gyro_radian.array[i];
		angle_pos[i] = new_angle_pos[i];
	}

	TRACE("\r%.2f,%.2f,%.2f", pos[0]*PI180, pos[1]*PI180, pos[2]*PI180);

	switch (mode)
	{
	case quadcopter:
		{

			// airborne or armed and throttle up
			bool after_unlock_action = airborne || rc[2] > 0.2f;

			// throttle
			if (submode == althold || submode == poshold || submode == bluetooth || submode == optical_flow)
			{
				float v = rc[2] - 0.5f;
				float user_rate;
				if (fabs(v)<0.05f)
					user_rate = 0;
				else if (v>= 0.05f)
				{
					user_rate = (v-0.05f)/0.45f;
					user_rate = user_rate * user_rate * quadcopter_max_climb_rate;
				}
				else
				{
					user_rate = (v+0.05f)/0.45f;
					user_rate = -user_rate * user_rate * quadcopter_max_descend_rate;
				}

				float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
				alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
				
				//attention : this is for landing mode
				if(true==islanding)	alt_controller.update(interval,user_rate-0.5f);
				else alt_controller.update(interval, user_rate);
				
				throttle_result = alt_controller.get_result();

				TRACE("\rthr=%f/%f", throttle_result, alt_controller.get_result());
			}
			else
			{
				throttle_result = rc[2];
			}

			// lean angle
			if (submode == basic || submode == althold)
			{
				if (after_unlock_action)	// airborne or armed and throttle up
				{
					// roll & pitch, RC trim is accepted.
					for(int i=0; i<2; i++)
					{
						float limit_l = angle_target_unrotated[i] - PI*2 * interval;
						float limit_r = angle_target_unrotated[i] + PI*2 * interval;
						angle_target_unrotated[i] = rc[i] * quadcopter_range[i] * (i==1?-1:1);	// pitch stick and coordinate are reversed 
						angle_target_unrotated[i] = limit(angle_target_unrotated[i], limit_l, limit_r);
						angle_target[i] = angle_target_unrotated[i];
					}

					if (simple_mode > 0.1f)
					{
						float diff = yaw_launch - euler[2];
						float cosdiff = cos(diff);
						float sindiff = sin(diff);
						angle_target[0] = angle_target_unrotated[0] * cosdiff - angle_target_unrotated[1] * sindiff;
						angle_target[1] = angle_target_unrotated[0] * sindiff + angle_target_unrotated[1] * cosdiff;
					}
					
				}
				else
				{
					angle_target[0] = 0;
					angle_target[1] = 0;
					angle_target[2] = angle_pos[2];
				}
			}

			else if (submode == optical_flow)
			{
				if (after_unlock_action)	// airborne or armed and throttle up
				{
					float stick_roll = rc[0] * quadcopter_range[0];
					float stick_pitch = -rc[1] * quadcopter_range[1];	// pitch stick and coordinate are reversed

					float flow_roll = frame.flow_comp_m_x/1000.0f;
					float flow_pitch = frame.flow_comp_m_y/1000.0f;
					of_controller.update_controller(flow_roll, flow_pitch, stick_roll, stick_pitch, interval);
					of_controller.get_result(&angle_target[0], &angle_target[1]);
				}
				else
				{
					angle_target[0] = 0;
					angle_target[1] = 0;
					angle_target[2] = angle_pos[2];
				}

				// TODO : handle yaw flow

			}

			else if (submode == bluetooth)
			{
				angle_target[0] = bluetooth_roll;
				angle_target[1] = bluetooth_pitch;
			}
			else if (submode == poshold)
			{
				// 10hz pos controller rate
				static int64_t last_pos_controll_time = 0;
				float dt = (systimer->gettime() - last_pos_controll_time) / 1000000.0f;
				if (dt > 0.1f)
				{
					last_pos_controll_time = systimer->gettime();
					if (dt < 1.0f)
					{
						position_meter meter = estimator.get_estimation_meter();


						float ne_pos[2] = {meter.latitude, meter.longtitude};
						float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
						float desired_velocity[2] = {rc[1] * 5, rc[0] * 5};
						if (abs(desired_velocity[0]) < 0.4f)
							desired_velocity[0] = 0;
						if (abs(desired_velocity[1]) < 0.4f)
							desired_velocity[1] = 0;

						controller.provide_attitue_position(euler, ne_pos, ne_velocity);
						controller.set_desired_velocity(desired_velocity);
						controller.update_controller(dt);

						controller.get_target_angles(angle_target);
					}
				}
			}
			// yaw:
			float delta_yaw = ((fabs(rc[3]) < RC_DEAD_ZONE) ? 0 : rc[3]) * interval * QUADCOPTER_ACRO_YAW_RATE;

			float new_target = radian_add(angle_target[2], delta_yaw);
			float new_error = abs(radian_sub(new_target, angle_pos[2]));
			if (new_error > (airborne?QUADCOPTER_MAX_YAW_OFFSET:(QUADCOPTER_MAX_YAW_OFFSET/5)) && new_error > abs(angle_error[2]))
				;
			else
				angle_target[2] = new_target;

			if (!after_unlock_action)
				angle_target[2] = angle_pos[2];

			// now calculate target angle rate
			// based on a PID stablizer
			for(int i=0; i<3; i++)
			{
				float new_angle_error = radian_sub(angle_target[i], angle_pos[i]);	// use radian_sub mainly for yaw


				// 5hz low pass filter for D, you won't be that crazy, right?
				static const float lpf_RC = 1.0f/(2*PI * 20.0f);
				float alpha = interval / (interval + lpf_RC);

				if (airborne)
				{
					angle_errorI[i] += new_angle_error * interval;
					angle_errorI[i] = limit(angle_errorI[i], -pid_factor2[i][3], pid_factor2[i][3]);
				}
				angle_errorD[i] = (1-alpha) * angle_errorD[i] + alpha * (new_angle_error - angle_error[i]) / interval;
				angle_error[i] = new_angle_error;

				// apply angle pid
				target[i] = angle_error[i] * pid_factor2[i][0] + angle_errorI[i] * pid_factor2[i][1] + angle_errorD[i] * pid_factor2[i][2];

				// max target rate: 180 degree/second
				target[i] = limit(target[i], -PI, PI);
			}
			TRACE(",roll=%f,%f", angle_pos[0] * PI180, angle_target[0] * PI180, airborne ? "true" : "false");
			TRACE("angle pos,target=%f,%f, air=%s\r\n", angle_pos[2] * PI180, angle_target[2] * PI180, airborne ? "true" : "false");

			// check takeoff
			if ( (alt_estimator.state[0] > takeoff_ground_altitude + 1.0f) ||
				(alt_estimator.state[0] > takeoff_ground_altitude && throttle_result > alt_controller.throttle_hover) ||
				(throttle_result > alt_controller.throttle_hover + QUADCOPTER_THROTTLE_RESERVE))
			{
				airborne = true;
				gyro_bias_estimating_end = true;
			}
		}
		break;
	}

	return 0;
}

int pid()
{
	for(int i=0; i<3; i++)
	{
		float new_p = (target[i]-pos[i]);

		// I
		if (airborne)		// only integrate after takeoff
		error_pid[i][1] += new_p * interval;
		error_pid[i][1] = limit(error_pid[i][1], -pid_factor[i][3], pid_factor[i][3]);

		// D, with 40hz 4th order low pass filter
		static const float lpf_RC = 1.0f/(2*PI * 40.0f);
		float alpha = interval / (interval + lpf_RC);
		float derivative = (new_p - error_pid[i][0] )/interval;

		for(int j=0; j<lpf_order; j++)
			errorD_lpf[j][i] = errorD_lpf[j][i] * (1-alpha) + alpha * (j==0?derivative:errorD_lpf[j-1][i]);

		error_pid[i][2] = errorD_lpf[lpf_order-1][i];

		// P
		error_pid[i][0] = new_p;

		// sum
		pid_result[i] = 0;
		float p_rc = limit(rc[5]+1, 0, 2);
		for(int j=0; j<3; j++)
		{
			pid_result[i] += error_pid[i][j] * pid_factor[i][j];
		}
	}
	TRACE(", pid=%.2f, %.2f, %.2f\n", pid_result[0], pid_result[1], pid_result[2]);

	return 0;
}

int output()
{
	if (mode == quadcopter || (mode == rc_fail) )
	{
		//pid[2] = -pid[2];
		throttle_real = 0;
		int matrix = (float)motor_matrix;

		int motor_count = MAX_MOTOR_COUNT;

		for(int i=0; i<MAX_MOTOR_COUNT; i++)
		{
			if (quadcopter_mixing_matrix[matrix][i][0] == quadcopter_mixing_matrix[matrix][i][1] && 
				quadcopter_mixing_matrix[matrix][i][1] == quadcopter_mixing_matrix[matrix][i][2] && 
				fabs((float)quadcopter_mixing_matrix[matrix][i][2]) < 0.01f)
			{
				motor_count = i;
				break;
			}

			if (mode == rc_fail)
			{
				int16_t data[16];
				int count = min(rcout->get_channel_count(), 16);
				for(int i=0; i<count; i++)
					data[i] = THROTTLE_STOP;
				rcout->write(data, 0,  count);
			}
			else
			{
				float mix = throttle_result;

				TRACE("\r%.2f, mode=%d", mix, submode);

				for(int j=0; j<3; j++)
					mix += quadcopter_mixing_matrix[matrix][i][j] * pid_result[j] * QUADCOPTER_THROTTLE_RESERVE;

				g_ppm_output[i] = limit(THROTTLE_IDLE + mix*(THROTTLE_MAX-THROTTLE_IDLE), THROTTLE_IDLE, THROTTLE_MAX);

				TRACE("\rpid[x] = %f, %f, %f", pid[0], pid[1], pid[2]);
				throttle_real += mix;
			}
		}
		throttle_real /= motor_count;
	}

	if (mode == _shutdown || mode == initializing)
	{
		STOP_ALL_MOTORS();
	}


	output_rc();
	return 0;
}



// called by main loop, only copy logs to a memory buffer, should be very fast
int save_logs()
{
	if (LOG_LEVEL == LOG_SDCARD	&& !log_ready)
		return 0;

	// if the saving task is transferring logs into 2nd buffer
	if (log_pending !=0)
		return -1;

	// send/store debug data
	time = systimer->gettime();
	sensor_data sensor =
	{
		{mag_uncalibrated.array[0] * 10, mag_uncalibrated.array[1] * 10, mag_uncalibrated.array[2] * 10},
		{accel_uncalibrated.array[0] * 100, accel_uncalibrated.array[1] * 100, accel_uncalibrated.array[2] * 100},
		mpu6050_temperature * 100 - 10000,
		{gyro_radian.array[0] * 18000/PI, gyro_radian.array[1] * 18000/PI, gyro_radian.array[2] * 18000/PI},
		voltage * 1000,
		current * 1000,
	};
	log(&sensor, TAG_SENSOR_DATA, time);

	imu_data imu = 
	{
		a_raw_pressure,
		a_raw_temperature,
		{estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2]},
		{estGyro.array[0], estGyro.array[1], estGyro.array[2]},
		{estMagGyro.array[0], estMagGyro.array[1], estMagGyro.array[2]},
	};
	log(&imu, TAG_IMU_DATA, time);
	log(&frame, TAG_PX4FLOW_DATA, time);
	
	position p = estimator.get_estimation();
	ned_data ned = 
	{
		0,
		{accel_earth_frame.array[0] * 1000, accel_earth_frame.array[1] * 1000, accel_earth_frame.array[2] * 1000},
		p.latitude * double(10000000.0/COORDTIMES), 
		p.longtitude * double(10000000.0/COORDTIMES), 
		error_lat : estimator.abias_lat,
		error_lon : estimator.abias_lon,
	};

	log(&ned, TAG_NED_DATA, time);


	pilot_data pilot = 
	{
		alt_estimator.state[0] * 100,
		airspeed_sensor_data * 1000,
		{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
		{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
		mode,
		mah_consumed,
	};

	log(&pilot, TAG_PILOT_DATA, time);

	pilot_data2 pilot2 = 
	{
		{error_pid[0][1]*180*100/PI, error_pid[1][1]*180*100/PI, error_pid[2][1]*180*100/PI},
		{error_pid[0][2]*180*100/PI, error_pid[1][2]*180*100/PI, error_pid[2][2]*180*100/PI},
	};

	log(&pilot2, TAG_PILOT_DATA2, time);

	ppm_data ppm = 
	{
		{g_pwm_input[0], g_pwm_input[1], g_pwm_input[2], g_pwm_input[3], g_pwm_input[4], g_pwm_input[5]},
		{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
	};

	log(&ppm, TAG_PPM_DATA, time);

	quadcopter_data quad = 
	{
		angle_pos[0] * 18000/PI, angle_pos[1] * 18000/PI, angle_pos[2] * 18000/PI,
		angle_target[0] * 18000/PI, angle_target[1] * 18000/PI, angle_target[2] * 18000/PI,
		pos[0] * 18000/PI, pos[1] * 18000/PI, pos[2] * 18000/PI,
		target[0] * 18000/PI,  target[1] * 18000/PI, target[2] * 18000/PI, 
	};

	log(&quad, TAG_QUADCOPTER_DATA, time);


	quadcopter_data2 quad2 = 
	{
		alt_estimator.state[1] * 100,
		airborne,
		submode,
		alt_estimator.state[0] * 100,
		alt_estimator.state[2] * 100,
		a_raw_altitude * 100,
		accelz_mwc * 100,
		loop_hz,
		THROTTLE_IDLE + throttle_result * (THROTTLE_MAX-THROTTLE_IDLE),
		kalman_accel_bias : alt_estimator.state[3] * 1000,
		{gyro_bias[0] * 1800000/PI, gyro_bias[1] * 1800000/PI, gyro_bias[2] * 1800000/PI,}
	};

	log(&quad2, TAG_QUADCOPTER_DATA2, time);

	quadcopter_data4 quad4 = 
	{
		isnan(alt_controller.m_sonar_target) ? 0 : alt_controller.m_sonar_target*100,
	};
	log(&quad4, TAG_QUADCOPTER_DATA4, time);

	quadcopter_data3 quad3 = 
	{
		alt_controller.baro_target * 100,
		alt_estimatorCF.state[0] * 100,
		alt_controller.target_climb_rate * 100,
		alt_estimatorCF.state[1] * 100,
		alt_controller.target_accel * 100,
		(accelz + alt_estimatorCF.state[3]) * 100,
		throttle_result*1000,
		yaw_launch * 18000 / PI,
		euler[2] * 18000 / PI,
		alt_controller.throttle_hover*1000,
		0,//sonar_result(),
		alt_controller.accel_error_pid[1]*1000,
	};

	log(&quad3, TAG_QUADCOPTER_DATA3, time);

	// pos controller data1
	pos_controller_data pc = 
	{
		controller.setpoint[0],
		controller.setpoint[1],
		controller.pos[0],
		controller.pos[1],
		controller.target_velocity[0]*1000,
		controller.target_velocity[1]*1000,
		controller.velocity[0]*1000,
		controller.velocity[1]*1000,
	};
	log(&pc, TAG_POS_CONTROLLER_DATA1, time);


	// pos controller data2
	pos_controller_data2 pc2 = 
	{
		controller.target_accel[0]*1000,
		controller.target_accel[1]*1000,
		{
			{controller.pid[0][0]*100, controller.pid[0][1]*100, controller.pid[0][2]*100,},
			{controller.pid[1][0]*100, controller.pid[1][1]*100, controller.pid[1][2]*100,},
		}
	};

	log(&pc2, TAG_POS_CONTROLLER_DATA2, time);

	if (last_gps_tick > systimer->gettime() - 2000000)
	{
		::gps_data data = 
		{
			{gps.DOP[0], gps.DOP[1], gps.DOP[2]},
			gps.speed*100,
			gps.longitude * 10000000, gps.latitude * 10000000, gps.altitude,
			gps.satelite_in_view, gps.satelite_in_use,
			gps.sig, gps.fix,
			gps_id & 0xf,
			gps.direction,
		};

		log(&data, TAG_GPS_DATA, time);
	}
	
	return 0;
}

int read_sensors()
{
	int64_t reading_start = systimer->gettime();
	vector acc = {0};
	vector gyro = {0};
	vector mag = {0};
	
	if (manager.get_flow_count())
	{
		sensors::IFlow *flow = manager.get_flow(0);

		if (flow->read_flow(&frame) < 0)
			sonar_distance = NAN;
		else
			sonar_distance = frame.ground_distance <= 0.30f ? NAN : frame.ground_distance / 1000.0f;
	}

	// read usart source
	handle_uart4_cli();

	

	// read gyros
	int healthy_gyro_count = 0;
	for(int i=0; i<manager.get_gyroscope_count(); i++)
	{
		IGyro* gyroscope = manager.get_gyroscope(i);
		
		gyro_data data;
		if (!gyroscope->healthy() || gyroscope->read(&data) < 0)
			continue;
		
		mpu6050_temperature = data.temperature;
		gyro.V.x += data.x;
		gyro.V.y += data.y;
		gyro.V.z += data.z;

		healthy_gyro_count ++;
	}
	if (healthy_gyro_count == 0)
		critical_errors |= error_gyro;
	gyro.V.x /= healthy_gyro_count;
	gyro.V.y /= healthy_gyro_count;
	gyro.V.z /= healthy_gyro_count;

	// read accelerometers
	int healthy_acc_count = 0;
	for(int i=0; i<manager.get_accelerometer_count(); i++)
	{
		IAccelerometer* accelerometer = manager.get_accelerometer(i);
		
		accelerometer_data data;
		if (!accelerometer->healthy() || accelerometer->read(&data) < 0)
			continue;

		acc.V.x += data.x;
		acc.V.y += data.y;
		acc.V.z += data.z;

		healthy_acc_count ++;
	}
	if (healthy_acc_count == 0)
		critical_errors |= error_accelerometer;
	acc.V.x /= healthy_acc_count;
	acc.V.y /= healthy_acc_count;
	acc.V.z /= healthy_acc_count;

	// read magnetometers
	int healthy_mag_count = 0;
	for(int i=0; i<manager.get_magnetometer_count(); i++)
	{
		IMagnetometer* magnetometer = manager.get_magnetometer(i);
		
		mag_data data;
		if (!magnetometer->healthy() || magnetometer->read(&data) < 0)
			continue;
		
		mag.V.x += data.x;
		mag.V.y += data.y;
		mag.V.z += data.z;

		healthy_mag_count ++ ;
	}
	if (healthy_mag_count == 0)
		critical_errors |= error_magnet;
	mag.V.x /= healthy_mag_count;
	mag.V.y /= healthy_mag_count;
	mag.V.z /= healthy_mag_count;
	
	// read barometers
	int healthy_baro_count = 0;
	for(int i=0; i<manager.get_barometer_count(); i++)
	{
		IBarometer* barometer = manager.get_barometer(i);
		
		baro_data data;
		if (!barometer->healthy())
			continue;
		int res = barometer->read(&data);
		if (res < 0)
			continue;

		new_baro_data = res == 0;
		a_raw_temperature = data.temperature;
		a_raw_pressure = data.pressure;

		healthy_baro_count ++;
	}
	if (healthy_baro_count == 0)
		critical_errors |= error_baro;

	// read GPSs
	int lowest_hdop = 100000;
	for(int i=0; i<manager.get_GPS_count(); i++)
	{
		IGPS *gps = manager.get_GPS(i);
		if (!gps->healthy())
			continue;

		devices::gps_data data;
		int res = gps->read(&data);

		// TODO: select best GPS correctly
		if (res == 0 && data.DOP[1] > 0 && data.DOP[1] < lowest_hdop)
		{
			lowest_hdop = data.DOP[1];
			::gps = data;
			new_gps_data = (res == 0);
			last_gps_tick = systimer->gettime();
		}
	}	

	// bias and scale calibrating
	accel_uncalibrated = acc;
	mag_uncalibrated = mag;
	for(int i=0; i<3; i++)
	{
		acc.array[i] += acc_bias[i];
		acc.array[i] *= acc_scale[i];
		mag.array[i] += mag_bias[i];
		mag.array[i] *= mag_scale[i];
	}
	acc.array[2] += -1.8f;
	
	float mag_size = sqrt(mag.array[0]*mag.array[0]+mag.array[1]*mag.array[1]+mag.array[2]*mag.array[2]);
	TRACE("\rmag_size:%.3f, %.0f, %.0f, %.0f    ", mag_size, mag.array[0], mag.array[1], mag.array[2]);
	
	
	::mag = mag;

	// TODO: apply a high order LPF to gyro readings
	::gyro_radian = gyro;

	// apply a 5hz LPF to accelerometer readings
	const float RC20 = 1.0f/(2*3.1415926 * 20.0f);
	float alpha20 = interval / (interval + RC20);

	if (isnan(accel.array[0]) || interval == 0)
		::accel = acc;
	else
	{
		vector_multiply(&::accel, 1-alpha20);
		vector_multiply(&acc, alpha20);
		vector_add(&::accel, &acc);
	}

	// statics
	for(int i=0; i<3; i++)
	{
		imu_statics[0][0].array[i] = f_min(accel.array[i], imu_statics[0][0].array[i]);
		imu_statics[0][1].array[i] = accel.array[i];
		imu_statics[0][2].array[i] = f_max(accel.array[i], imu_statics[0][2].array[i]);
		imu_statics[0][3].array[i] = accel.array[i] + imu_statics[0][3].array[i];

		imu_statics[1][0].array[i] = f_min(gyro.array[i], imu_statics[1][0].array[i]);
		imu_statics[1][1].array[i] = gyro.array[i];
		imu_statics[1][2].array[i] = f_max(gyro.array[i], imu_statics[1][2].array[i]);
		imu_statics[1][3].array[i] = gyro.array[i] + imu_statics[1][3].array[i];
	}
	avg_count ++;

	// voltage and current sensors	
	float alpha = interval / (interval + 1.0f/(2*PI * 2.0f));		// 2hz low pass filter
	if (manager.getBatteryVoltage("BatteryVoltage"))
		voltage = voltage * (1-alpha) + alpha * manager.getBatteryVoltage("BatteryVoltage")->read();
	if (manager.getBatteryVoltage("BatteryCurrent"))
		current = current * (1-alpha) + alpha * manager.getBatteryVoltage("BatteryCurrent")->read();	
	
	mah_consumed += fabs(current) * interval / 3.6f;	// 3.6 mah = 1As

	return 0;
}

int calculate_state()
{
	if (interval <=0 || interval > 0.2f)
		return -1;

	NonlinearSO3AHRSupdate(
		accel.array[0], accel.array[1], accel.array[2], 
		mag.array[0], mag.array[1], mag.array[2],
		gyro_radian.array[0], gyro_radian.array[1], gyro_radian.array[2],
		0.15f, 0.0015f, 0.15f, 0.0015f, interval);
// 	MadgwickAHRSupdateIMU(gyro.array[0] /*+ (systimer->gettime() > 15000000 ? PI*5.0f/180.0f : 0)*/, gyro.array[1], -gyro.array[2], -acc_norm.V.y, acc_norm.V.x, acc_norm.V.z, 1.5f, interval);


	euler[0] = radian_add(euler[0], quadcopter_trim[0]);
	euler[1] = radian_add(euler[1], quadcopter_trim[1]);
	euler[2] = radian_add(euler[2], quadcopter_trim[2]);

	TRACE("euler:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, time:%f, bias:%.2f/%.2f/%.2f, pressure=%.2f \n ", euler[0]*PI180, euler[1]*PI180, euler[2]*PI180, roll*PI180, pitch*PI180, yaw_mag*PI180, systimer->gettime()/1000000.0f, gyro_bias[0]*PI180, gyro_bias[1]*PI180, gyro_bias[2]*PI180, a_raw_pressure);

	for(int i=0; i<3; i++)
		accel_earth_frame.array[i] = acc_ned[i];

// 	LOGE("angle target:%.2f,%.2f,%.2f\n", angle_target[0]*PI180, angle_target[1]*PI180, angle_target[2]*PI180);


	vector mwc_acc = {accel.V.y, -accel.V.x, -accel.V.z};
	vector mwc_gyro = {gyro_radian.array[0], gyro_radian.array[1], -gyro_radian.array[2]};
	vector mwc_mag = {mag.V.y, -mag.V.x, -mag.V.z};
	vector_multiply(&mwc_acc, 1.0f/G_in_ms2);

	ahrs_mwc_update(mwc_gyro, mwc_acc, mwc_mag, interval);
	roll = radian_add(roll, quadcopter_trim[0]);
	pitch = radian_add(pitch, quadcopter_trim[1]);
	yaw_mag = radian_add(yaw_mag, quadcopter_trim[2]);
	yaw_gyro = radian_add(yaw_gyro, quadcopter_trim[2]);




	// calculate altitude
	if (new_baro_data)
		calculate_baro_altitude();

	accelz_mwc = accelz;
	accelz = acc_ned[2];

	alt_estimator.set_land_effect(mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - takeoff_ground_altitude) < 1.0f));
	alt_estimator.update(accelz, new_baro_data ? a_raw_altitude : NAN, interval);
	alt_estimatorCF.set_land_effect(mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - takeoff_ground_altitude) < 1.0f));
	alt_estimatorCF.update(accelz, new_baro_data ? a_raw_altitude : NAN, interval);
	estimator.update_accel(accel_earth_frame.array[0], accel_earth_frame.array[1], systimer->gettime());

	if (new_gps_data)
	{
		// fuse gps data
		gps_id++;

		if (gps.DOP[1] > 0 && (gps.DOP[1]/100.0f) < (estimator.healthy ? 3.5f : 2.5f) && gps.fix>=3)
		{
			estimator.update_gps(COORDTIMES * gps.latitude, COORDTIMES * gps.longitude, gps.DOP[1]/100.0f, systimer->gettime());
		}
// 		else
		{
			//estimator.reset();
			//estimator2.reset();
		}

		float yaw_gps = gps.direction * PI / 180;
		if (yaw_gps > PI)
			yaw_gps -= 2 * PI;

		ground_speed_east = sin(yaw_gps) * gps.speed;
		ground_speed_north = cos(yaw_gps) * gps.speed;

		// TODO: compute acceleration and use it to compensate roll and pitch
	}

	return 0;
}

int sensor_calibration()
{
	// update gyro temperature compensation
	if (!isnan((float)_gyro_bias[0][0]) && !isnan((float)_gyro_bias[1][0]))
	{
		float dt = _gyro_bias[1][0] - _gyro_bias[0][0];
		if (dt > 1.0f)
		{
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = _gyro_bias[0][i+1];
				gyro_temp_k.array[i] = (_gyro_bias[1][i+1] - _gyro_bias[0][i+1]) / dt;
			}
			temperature0 = _gyro_bias[0][0];
		}
		else
		{
			// treat as one point
			int group = !isnan(_gyro_bias[0][0]) ? 0 : (!isnan(_gyro_bias[1][0]) ? 1: -1);
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = group >= 0 ? _gyro_bias[group][i+1] : 0;
				gyro_temp_k.array[i] = 0;
			}
			temperature0 = group != 0 ? _gyro_bias[group][0] : 0;
		}
	}
	else
	{
		int group = !isnan(_gyro_bias[0][0]) ? 0 : (!isnan(_gyro_bias[1][0]) ? 1: -1);
		for(int i=0; i<3; i++)
		{
			gyro_temp_a.array[i] = group >= 0 ? _gyro_bias[group][i+1] : 0;
			gyro_temp_k.array[i] = 0;
		}
		temperature0 = group != 0 ? _gyro_bias[group][0] : 0;
	}

	// static base value detection
	vector mag_avg = {0};
	vector accel_avg = {0};
	vector gyro_avg = {0};

	int baro_counter = 0;
	ground_pressure = 0;
	int calibrating_count = 1500;
	ground_temperature = 0;
	for(int i=0; i<calibrating_count; i++)
	{
		long us = systimer->gettime();

		LOGE("\r%d/%d", i, calibrating_count);

		read_sensors();
		if (new_baro_data)
		{
			baro_counter ++;
			ground_pressure += a_raw_pressure;
			ground_temperature += a_raw_temperature;
		}

		//mpu6050_temperature += p->temperature1  / 340.0f + 36.53f;

		vector_add(&accel_avg, &accel);
		vector_add(&mag_avg, &mag);
		vector_add(&gyro_avg, &gyro_radian);

		if (i>calibrating_count/10 && 
			  ((fabs(gyro_radian.array[0]*PI180)>5.0f || fabs(gyro_radian.array[1]*PI180)>5.0f || fabs(gyro_radian.array[2]*PI180)>5.0f))
			)
		{
			LOGE("wtf %f,%f,%f,%f,%f,%f, %d\n", fabs(gyro_radian.array[0]*PI180), fabs(gyro_radian.array[1]*PI180), fabs(gyro_radian.array[2]*PI180), 
				 fabs(gyro_radian.array[0]*PI180), fabs(gyro_radian.array[1]*PI180), fabs(gyro_radian.array[2]*PI180), i);
			return -1;
		}

		if (critical_errors)
			return -2;

		if ((systimer->gettime()/1000)%50 > 25)
			led_all_on();
		else
			led_all_off();

		while(systimer->gettime() - us < 3000)
			;
	}

	vector_divide(&accel_avg, calibrating_count);
	vector_divide(&mag_avg, calibrating_count);
	vector_divide(&gyro_avg, calibrating_count);
//	mpu6050_temperature /= calibrating_count;
	ground_pressure /= baro_counter;
	ground_temperature /= baro_counter;

	LOGE("base value measured\n");

	// init ahrs
	vector mwc_acc = {accel_avg.V.y, -accel_avg.V.x, -accel_avg.V.z};
	float mwc_gyro[3] = {gyro_avg.array[0], gyro_avg.array[1], -gyro_avg.array[2]};
	float mwc_mag[3] = {mag_avg.V.y, -mag_avg.V.x, -mag_avg.V.z};
	vector_multiply(&mwc_acc, 1.0f/G_in_ms2);

	ahrs_mwc_init(gyro_avg, accel_avg, mag_avg);

	NonlinearSO3AHRSinit(accel_avg.V.x, accel_avg.V.y, accel_avg.V.z, 
		mag_avg.V.x, mag_avg.V.y, mag_avg.V.z, 
		gyro_avg.V.x, gyro_avg.V.y, gyro_avg.V.z);

	return 0;
}

extern "C" int Mal_Accessed(void);

int usb_lock()
{
	if (Mal_Accessed())
	{
		STOP_ALL_MOTORS();
	}

	return -1;
}

int set_submode(copter_mode newmode)
{
	if (mode != quadcopter)
		newmode = invalid;
	if (newmode == submode)
		return 0;

	bool has_pos_controller = submode == poshold;
	bool to_use_pos_controller = newmode == poshold;
	bool has_alt_controller = submode == poshold || submode == althold || submode == bluetooth || submode == optical_flow;
	bool to_use_alt_controller = newmode == poshold || newmode == althold || newmode == bluetooth || newmode == optical_flow;
	
	if (!has_pos_controller && to_use_pos_controller)
	{
		// reset pos controller
		position_meter meter = estimator.get_estimation_meter();
		float ne_pos[2] = {meter.latitude, meter.longtitude};
		float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
		float desired_velocity[2] = {0, 0};

		controller.provide_attitue_position(euler, ne_pos, ne_velocity);
		controller.set_desired_velocity(desired_velocity);
		controller.get_target_angles(angle_target);
		controller.reset();
	}

	if (!has_alt_controller && to_use_alt_controller)
	{
		float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
		alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
		alt_controller.reset();
	}

	if (newmode == optical_flow && submode != optical_flow)
	{
		of_controller.reset();
	}
	
	submode = newmode;

	return 0;
}

int set_mode(fly_mode newmode)
{
	if (newmode == mode)
		return 0;

	target[0] = pos[0];
	target[1] = pos[1];
	target[2] = pos[2];

	takeoff_ground_altitude = alt_estimator.state[0];
	yaw_launch = euler[2];
	collision_detected = 0;
	tilt_us = 0;
	throttle_result = 0;
	airborne = false;
	islanding = false;
	float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
	alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
	alt_controller.reset();


	for(int i=0; i<3; i++)
	{
		angle_target[i] = angle_pos[i];
		error_pid[i][1] = 0;	//reset integration
		angle_errorI[i] = 0;
	}
	
	mode = newmode;
	return 0;
}


int check_mode()
{
	// sub mode swtich
	if (g_pwm_input_update[5] > systimer->gettime() - 500000)
	{
		copter_mode newmode = submode;
		if(!has_6th_channel)
			newmode = althold;
		else if (rc[5] < -0.6f)
			newmode = basic;
		else if (rc[5] > 0.6f)
// 			newmode = airborne ? optical_flow : althold;
// 			newmode = (bluetooth_last_update > systimer->gettime() - 500000) ? bluetooth : althold;
			newmode = (estimator.healthy && airborne) ? poshold : althold;
		else if (rc[5] > -0.5f && rc[5] < 0.5f)
			newmode = althold;

		set_submode(newmode);
	}

	if (g_pwm_input_update[4] > systimer->gettime() - 500000 || !has_5th_channel)
	{
		if (mode == initializing)
			set_mode(_shutdown);

		
		// emergency switch
		// magnetometer calibration starts if flip emergency switch 10 times, interval time between each flip should be less than 1 second.
		static float last_ch4 = 0;
		if (fabs(rc[4]-last_ch4) > 0.20f)
		{
			set_mode(_shutdown);
			last_ch4 = rc[4];
			LOGE("shutdown!\n");
			
			static int flip_count = 0;
			static int64_t last_flip_time = 0;
			
			if (systimer->gettime() - last_flip_time < 1000000)
				flip_count ++;
			else
				flip_count = 1;
			last_flip_time = systimer->gettime();
			
			if (flip_count >10 && mag_calibration_state == 0)
			{
				LOGE("start magnetometer calibrating");
				mag_calibrator.reset();
				mag_calibration_state = 1;
			}
		}

		// arm action check: RC first four channel active, throttle minimum, elevator stick down, rudder max or min, aileron max or min, for 0.5second
		static int64_t arm_start_tick = 0;
		bool rc_fail = false;
		for(int i=0; i<4; i++)
			if (g_pwm_input_update[i] < systimer->gettime() - 500000)
				rc_fail = true;
		bool arm_action = !rc_fail && rc[2] < 0.1f  && fabs(rc[0]) > 0.85f
						&& fabs(rc[1]) > 0.85f && fabs(rc[3]) > 0.85f;
		if (!arm_action)
		{
			arm_start_tick = 0;
		}
		else
		{
			if (arm_start_tick > 0)
			{
				if (systimer->gettime() - arm_start_tick > 500000)
				{
					set_mode(quadcopter);
					LOGE("armed!\n");
				}
			}
			else
			{
				arm_start_tick = systimer->gettime();
			}
		}
	}
	else
	{
		TRACE("warning: RC out of controll");
		set_mode(rc_fail);
	}

	return 0;
}
void check_takeoff_OR_landing()
{
	static float last_ch7 = NAN;
	static float read_time;
	static bool iswait=false;
	float time_now;
	//first start
	if(isnan(last_ch7))last_ch7=rc[7];
	if (fabs(rc[7]-last_ch7) > 0.20f)
	{
		last_ch7 = rc[7];
		static int flip_count = 0;
		static int64_t last_flip_time = 0;
		
		if (systimer->gettime() - last_flip_time < 1000000)
			flip_count ++;
		else
			flip_count = 0;
		last_flip_time = systimer->gettime();
			
		if(airborne==true && flip_count >=1)
		{
			flip_count=0;
			LOGE("\nauto landing!\n");
			islanding=true;
		}
		else if(iswait==false&&mode==_shutdown && flip_count >=1)
		{
			flip_count=0;
			LOGE("\nauto take off \n");
			set_mode(quadcopter);	
			LOGE("\narmed!\n");	
			//1s=1000000us		
			read_time=systimer->gettime();
			iswait=true;
		}
	}
	if(iswait==true)
	{
		time_now=systimer->gettime();
		if(time_now>=read_time+2000000)
		{
			iswait=false;
			alt_controller.set_altitude_target(alt_controller.get_altitude_target()+2.0f);
			LOGE("\nalread take off\n");			
		}
	}
}

int64_t land_detect_us = 0;
int land_detector()
{
	if ((throttle_result < 0.2f)				// low throttle
		&& fabs(alt_estimator.state[1]) < (quadcopter_max_descend_rate/4.0f)			// low climb rate : 25% of max descend rate should be reached in such low throttle, or ground was touched
// 		&& fabs(alt_estimator.state[2] + alt_estimator.state[3]) < 0.5f			// low acceleration
	)
	{
		land_detect_us = land_detect_us == 0 ? systimer->gettime() : land_detect_us;

		if (systimer->gettime() - land_detect_us > (airborne ? 1000000 : 3000000))		// 2 seconds for before take off, 1 senconds for landing
		{
			set_mode(_shutdown);
			LOGE("landing detected");
		}
	}
	else
	{
		land_detect_us = 0;
	}
	
	return 0;
}

int crash_detector()
{
	// always detect high G force
	vector ground = {0,0,-2000};		// not very precise, but should be enough
	vector accel_delta;
	for(int i=0; i<3; i++)
		accel_delta.array[i] = accel.array[i] - estAccGyro.array[i];

	float gforce = vector_length(&accel_delta);
	if (gforce > 1.75f)
	{
		TRACE("high G force (%.2f) detected\n", gforce);
		collision_detected = systimer->gettime();
	}

	// forced shutdown if >3g external force
	if (gforce > 3.0f)
	{
		TRACE("very high G force (%.2f) detected (%.0f,%.0f,%.0f)\n", gforce, accel.array[0], accel.array[1], accel.array[2]);
		//set_mode(_shutdown);
	}

	int prot = (float)::crash_protect;

	// tilt detection
	if (rc[2] < 0.1f || prot & CRASH_TILT_IMMEDIATE)
	{
		float tilt = sqrt(euler[0]*euler[0] + euler[1]*euler[1]);
		if (vector_angle(&ground, &estAccGyro) < 0.33f)		// around 70 degree
			tilt_us = tilt_us > 0 ? tilt_us : systimer->gettime();
		else
			tilt_us = 0;
	}

	if (((collision_detected > 0 && systimer->gettime() - collision_detected < 5000000) && (rc[2] < 0.1f || prot & CRASH_COLLISION_IMMEDIATE)) 
		|| (tilt_us> 0 && systimer->gettime()-tilt_us > 1000000))	// more than 1 second
	{
		TRACE("crash landing detected(%s)\n", (collision_detected > 0 && systimer->gettime() - collision_detected < 5000000) ? "collision" : "tilt");

		set_mode(_shutdown);
	}

	return 0;
}

float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = limit(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}

int handle_uart4_cli()
{
	char line[1024];
	char out[1024];
	IUART *uart = manager.getUART("UART2");
	int byte_count = uart->read(line, sizeof(line));
	if (byte_count <= 0)
		return 0;
	
	line[byte_count-1] = 0;
	
	int out_count = parse_command_line(line, out);
	if (out_count>0)
	{
		out[out_count] = 0;
		printf("%s,%s\n", line, out);
		uart->write(out, out_count);
	}

	return 0;
}

int handle_uart4_controll()
{
#ifdef STM32F4
	char line[1024];
	int byte_count = UART4_ReadPacket(line, sizeof(line));
	if (byte_count <= 0)
		return 0;
	
	LOGE(line);

	int len = strlen(line);
	const char * keyword = ",blue\n";
	const char * keyword2 = ",stick\n";

	if (strstr(line, keyword) == (line+len-strlen(keyword)))
	{
		char * p = (char*)strstr(line, ",");
		if (!p)
			return -1;

		*p = NULL;
		p++;
		
		bluetooth_roll = atof(line) * PI / 180;
		bluetooth_pitch = atof(p) * PI / 180;

		bluetooth_roll = limit(bluetooth_roll, -25*PI/180, 25*PI/180);
		bluetooth_pitch = limit(bluetooth_pitch, -25*PI/180, 25*PI/180);

		TRACE("%f,%f\n", bluetooth_roll * PI180, bluetooth_pitch * PI180);
		
		bluetooth_last_update = systimer->gettime();
	}
	else if (strstr(line, keyword2) == (line+len-strlen(keyword2)))
	{
		if (sscanf(line, "%f,%f,%f,%f", &rc_mobile[0], &rc_mobile[1], &rc_mobile[2], &rc_mobile[3] ) == 4)
		{
			mobile_last_update = systimer->gettime();
			TRACE("stick:%f,%f,%f,%f\n", rc_mobile[0], rc_mobile[1], rc_mobile[2], rc_mobile[3]);
		}
	}

	else if (strcmp(line, "arm\n") == 0)
	{
		LOGE("mobile arm\n");
		set_mode(quadcopter);

		const char *armed = "armed\n";
		UART4_SendPacket(armed, strlen(armed));
	}
	else if (strcmp(line, "arm\r\n") == 0)
	{
		LOGE("mobile arm\n");
		set_mode(quadcopter);

		const char *armed = "armed\n";
		UART4_SendPacket(armed, strlen(armed));
	}

	else if (strcmp(line, "disarm\n") == 0)
	{
		LOGE("mobile disarm\n");

		set_mode(_shutdown);
		const char *disarmed = "disarmed\n";
		UART4_SendPacket(disarmed, strlen(disarmed));
	}
	
	else if (strcmp(line, "takeoff\n") == 0)
	{
		LOGE("mobile takeoff\n");

		const char *tak = "taking off\n";
		UART4_SendPacket(tak, strlen(tak));

	}

	else if (strcmp(line, "land\n") == 0)
	{
		LOGE("mobile land\n");
		const char *land = "landing\n";
		UART4_SendPacket(land, strlen(land));

	}
	
	else if (strcmp(line, "RTL\n") == 0)
	{
		LOGE("mobile RTL\n");
		const char *rtl = "RTLing\n";
		UART4_SendPacket(rtl, strlen(rtl));
	}

	else
	{
		// invalid packet
	}
#endif

	return 0;
}

int read_rc()
{
	rcin->get_channel_data(g_pwm_input, 0, 8);
	rcin->get_channel_update_time(g_pwm_input_update, 0, 8);
	TRACE("\rRC");
	for(int i=0; i<8; i++)
	{
		rc[i] = ppm2rc(g_pwm_input[i], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);
		TRACE("%.2f,", rc[i]);
	}

	rc[2] = (rc[2]+1)/2;

	return 0;
}

void mag_calibrating_worker(int parameter)
{
	mag_calibrator.do_calibration();
	mag_calibration_result result;
	int res = mag_calibrator.get_result(&result);	
	LOGE("result:%d, bias:%f,%f,%f, scale:%f,%f,%f\n, residual:%f/%f", res, result.bias[0], result.bias[1], result.bias[2], result.scale[0], result.scale[1], result.scale[2], result.residual_average, result.residual_max);
	mag_calibration_state = 0;
	
	// do checks and flash RGB LED if error occured
	if (res == 0)
	{
		LOGE("mag calibration success\n");
		
		for(int i=0; i<3; i++)
		{
			mag_bias[i] = result.bias[i];
			mag_scale[i] = result.scale[i];
			mag_bias[i].save();
			mag_scale[i].save();
		}		
		
		// flash RGB LED to indicate a successs
		for(int i=0; i<10; i++)
		{
			if (rgb)
			{
				systimer->delayms(300);				
				rgb->write(0,1,0);
				systimer->delayms(300);
				rgb->write(0,0,0);
			}
		}

		// TODO: update ahrs state
	}
	else
	{
		// flash RGB LED to indicate a failure
		LOGE("mag calibration failed\n");
		for(int i=0; i<10; i++)
		{
			if (rgb)
			{
				systimer->delayms(300);				
				rgb->write(1,0,0);
				systimer->delayms(300);
				rgb->write(0,0,0);
			}
		}
	}
}

void test(int i)
{
	systimer->delayms(300);
	rgb->write(0,1,0);
	systimer->delayms(300);
	rgb->write(0,0,0);	
}

void main_loop(void)
{
	// calculate time interval
	static int64_t tic = 0;
	int64_t round_start_tick = systimer->gettime();
	interval = (round_start_tick-last_tick)/1000000.0f;
	last_tick = round_start_tick;
	
	// rc inputs
	read_rc();

	// usb
	usb_lock();	// lock the system if usb transfer occurred

	led_all_off();
	
	// performance counter
	cycle_counter++;
	if (systimer->gettime() - tic > 1000000)
	{
		tic = systimer->gettime();
		LOGE("speed: %d, time:%.2f\r\n", cycle_counter, systimer->gettime()/1000000.0f);
		loop_hz = cycle_counter;
		cycle_counter = 0;
	}

	// flashlight, tripple flash if SDCARD running, double flash if SDCARD failed
	time = systimer->gettime();
	int time_mod_1500 = (time%1500000)/1000;
	if (time_mod_1500 < 20 || (time_mod_1500 > 200 && time_mod_1500 < 220) || (time_mod_1500 > 400 && time_mod_1500 < 420 && log_ready))
	{
		//if (rgb)
		//	rgb->write(0,1,0);
		SAFE_ON(flashlight);
	}
	else
	{
		//if(rgb)
		//	rgb->write(0,0,0);
		SAFE_OFF(flashlight);
	}

	// RC modes and RC fail detection
	check_mode();
	check_takeoff_OR_landing();
	// read sensors
	read_sensors();
	
	// provide mag calibration with data
	if (mag_calibration_state == 1)
	{
		// update RGB LED for user interaction
		if (rgb)
		{
			if (mag_calibrator.get_stage() == stage_horizontal)
				rgb->write(1,0,0);
			else if (mag_calibrator.get_stage() == stage_vertical)
				rgb->write(0,1,0);
			else
				rgb->write(0,0,0);
		}

		
		// data
		mag_calibrator.provide_data(mag_uncalibrated.array, euler, gyro_radian.array, interval);
	
		// make a async call if data ready and change mag_calibration_state to calibrating
		if (mag_calibrator.get_stage() == stage_ready_to_calibrate)
		{
			mag_calibration_state = 2;
			manager.get_asyncworker()->add_work(mag_calibrating_worker, 0);
		}
	}

	// all state estimating, AHRS, position, altitude, etc
	calculate_state();

	prepare_pid();
	pid();
	output();
	save_logs();

	if (mode == quadcopter)
	{
		land_detector();
// 		crash_detector();
	}
	else
		land_detect_us = 0;

	led_all_on();

	if (log_ready)
	{		
		// flash one of the LED(A4) at 10hz
		int t = int(systimer->gettime() % 100000);
		if (t < 50000)
		{
			SAFE_ON(SD_led);
		}
		else
		{
			SAFE_OFF(SD_led);
		}
	}
}


void sdcard_logging_loop(void)
{
	static int64_t tick = systimer->gettime();
	int64_t t = systimer->gettime();
	int dt = t-tick;
	if (dt > 15000)
		TRACE("long log interval:%d\n", dt);


	int64_t starttick = systimer->gettime();
	int res = log_flush();
	starttick = systimer->gettime() - starttick;

	if (starttick > 10000)
		TRACE("long log time:%d\n", int(starttick));

	if (res == 0)
		tick = t;
}

int main(void)
{
	bsp_init_all();
	motor_matrix = 1;
	motor_matrix.save();
	
	/*
	pid_factor[0][0] = 0.3f;
	pid_factor[0][1] = 0.4f;
	pid_factor[0][2] = 0.012f;
	pid_factor[1][0] = 0.45f;
	pid_factor[1][1] = 0.5f;
	pid_factor[1][2] = 0.02f;
	//pid_factor2[0][0] = 4.5f;
	//pid_factor2[1][0] = 4.5f;
	
	while(1)
	{
		float t = systimer->gettime()/5000000.0f * 2 * PI;
		float t2 = systimer->gettime()/15000000.0f * 2 * PI;
		t2 = sin(t2)/2+0.6f;
		float r = t2*(sin(t)/2+0.5f);
		float g = t2*(sin(t+PI*2/3)/2+0.5f);
		float b = t2*(sin(t+PI*4/3)/2+0.5f);
		
		manager.getRGBLED("rgb")->write(r,g,b);
	}
	*/
	
	state_led = manager.getLED("state");
	SD_led = manager.getLED("SD");
	flashlight = manager.getLED("flashlight");
	rcin = manager.get_RCIN();
	rcout = manager.get_RCOUT();
	rgb = manager.getRGBLED("rgb");
	
	STOP_ALL_MOTORS();
	
	// USB
	USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
		USB_OTG_HS_CORE_ID,
#else
		USB_OTG_FS_CORE_ID,
#endif
		&USR_desc,
		&USBD_CDC_cb,
		&USR_cb);

	log_init();
	estimator.set_gps_latency(0);
	SAFE_ON(flashlight);


	int res;
	do
	{
		res = sensor_calibration();
		if (res == -2)
			break;
	}while (res < 0);

	read_rc();
	has_5th_channel = g_pwm_input_update[4] > systimer->gettime()-500000;
	has_6th_channel = g_pwm_input_update[5] > systimer->gettime()-500000;

	// check critical errors
	if (critical_errors != 0)
	{
		LOGE("critical_errors : %x (", critical_errors);
		for(int i=0; (1<<i)<error_MAX; i++)
		{
			if ((1<<i) & critical_errors)
				LOGE("%s %s", i==0?"":" | ", critical_error_desc[i]);
		}
		LOGE(" )\n");

		led_all_off();
		SAFE_OFF(flashlight);

		while (1)
		{
			// blink error code!
			for(int i=1; i<error_MAX; i<<=1)
			{
				led_all_on();
				SAFE_ON(flashlight);

				handle_uart4_cli();

				if (critical_errors & i)
					systimer->delayms(500);
				else
					systimer->delayms(150);

				led_all_off();
				SAFE_OFF(flashlight);
				systimer->delayms(150);
			}

			systimer->delayms(1500);
		}
	}
	
	// get two timers, one for main loop and one for SDCARD logging loop
	manager.getTimer("mainloop")->set_period(3000);
	manager.getTimer("mainloop")->set_callback(main_loop);
	manager.getTimer("log")->set_period(10000);
	manager.getTimer("log")->set_callback(sdcard_logging_loop);

	while(1)
	{

	}

}
