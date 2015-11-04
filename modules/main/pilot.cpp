#include "pilot.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <Protocol/RFData.h>
#include <Protocol/common.h>
#include <Protocol/usb_data_publish.h>
#include <utils/vector.h>
#include <utils/param.h>
#include <utils/log.h>
#include <utils/console.h>
#include <utils/gauss_newton.h>

#include <FileSystem/ff.h>

using namespace HAL;
using namespace devices;
using namespace math;

// constants
#define THROTTLE_STOP (max((int)(isnan(pwm_override_min)? (rc_setting[2][0]-20):pwm_override_min),1000))
#define THROTTLE_MAX (min((int)(isnan(pwm_override_max)? (rc_setting[2][2]-20) : pwm_override_max),2000))
#define SAFE_ON(x) if(x) (x)->on()
#define SAFE_OFF(x) if(x) (x)->off()
#define MAX_MOTOR_COUNT 8
#define SONAR_MIN 0.0f
#define SONAR_MAX 4.5f
const float PI180 = 180/PI;

// parameters
static param crash_protect("prot", 0);		// crash protection
static param pwm_override_max("tmax", NAN);
static param pwm_override_min("tmin", NAN);
static param quadcopter_max_climb_rate("maxC",5);
static param quadcopter_max_descend_rate("maxD", 2);
static param quadcopter_auto_landing_rate_fast("flrt", 1.5f);		// absolute value of fast automated landing speed in meter/s, 
static param quadcopter_auto_landing_rate_final("lrat", 0.5f);		// absolute value of final approach speed in meter/s
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

// constructor and destructor, initializer
yet_another_pilot::yet_another_pilot()
:state_led(NULL)
,SD_led(NULL)
,flashlight(NULL)
,rcin(NULL)
,rcout(NULL)
,rgb(NULL)
,range_finder(NULL)
,vcp(NULL)
,mag_calibration_state(0)			// 0: not running, 1: collecting data, 2: calibrating
,last_mag_calibration_result(0xff)	// 0xff: not calibrated at all, other values from mag calibration.
,usb_data_publish(0)
,lowpower(0)		// lowpower == 0:power good, 1:low power, no action taken, 2:low power, action taken.
,acc_cal_requested(false)
,acc_cal_done(false)
,motor_saturated(false)
,rc_fail(false)
,round_running_time(0)
,new_gps_data(false)
,critical_errors(0)
,cycle_counter(0)
,ground_pressure(0)
,ground_temperature(0)
,accelz(0)
,airborne(false)
,takeoff_ground_altitude(0)
,armed(false)
,submode(basic)
,collision_detected(0)	// remember to clear it before arming
,tilt_us(0)	// remember to clear it before arming
//,gyro_lpf2p({LowPassFilter2p(333.3, 40),LowPassFilter2p(333.3, 40),LowPassFilter2p(333.3, 40)})	// 2nd order low pass filter for gyro.

,last_tick(0)
,last_gps_tick(0)
,voltage(0)
,current(0)
,interval(0)
,new_baro_data(false)
,mah_consumed(0)
,wh_consumed(0)
,sonar_distance(NAN)
,bluetooth_last_update(0)
,mobile_last_update(0)
,avg_count(0)
,loop_hz(0)
,islanding(false)
{
	memset(g_pwm_input_update, 0, sizeof(g_pwm_input_update));
	memset(g_ppm_output, 0, sizeof(g_ppm_output));
	memset(g_pwm_input, 0, sizeof(g_pwm_input));
	memset(acc_avg_count, 0, sizeof(acc_avg_count));
	memset(&imu_statics[0][0], 0, sizeof(imu_statics));
	memset(rc_mobile, 0, sizeof(rc_mobile));
	memset(&gyro_reading, 0, sizeof(gyro_reading));
	memset(&body_rate, 0, sizeof(body_rate));
	memset(&accel, 0, sizeof(accel));
	
	for(int i=0; i<3; i++)
		gyro_lpf2p[i].set_cutoff_frequency(333.33, 40);
}

// helper functions
int yet_another_pilot::send_package(const void *data, uint16_t size, uint8_t type, IUART*uart)
{
	uint8_t start_code[2] = {0x85, 0xa3};
	uart->write(start_code, 2);
	uart->write(&size, 2);
	uart->write(&type, 1);
	uart->write(data, size);

	return size;
}

void yet_another_pilot::output_rc()
{
	rcout->write(g_ppm_output, 0,  min(rcout->get_channel_count(), countof(g_ppm_output)));
}

void yet_another_pilot::STOP_ALL_MOTORS()
{
	for(int i=0; i<16; i++)
		g_ppm_output[i] = THROTTLE_STOP;
	output_rc();
}

int yet_another_pilot::calculate_baro_altitude()
{
	// raw altitude
	double scaling = (double)a_raw_pressure / ground_pressure;
	float temp = ((float)ground_temperature) + 273.15f;
	a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
	if (fabs(a_raw_altitude) < 5.0f)
		ground_temperature = a_raw_temperature;

	return 0;
}



int yet_another_pilot::run_controllers()
{
	attitude_controller.provide_states(euler, body_rate.array, motor_saturated ? LIMIT_ALL : LIMIT_NONE, airborne);
	
	// airborne or armed and throttle up
	bool after_unlock_action = airborne || rc[2] > 0.55f;

	// throttle
	if (submode == althold || submode == poshold || submode == bluetooth || submode == optical_flow)
	{
		float landing_rate = ((alt_estimator.state[0] > takeoff_ground_altitude + 10.0f) && !alt_controller.sonar_actived()) ? quadcopter_auto_landing_rate_fast : quadcopter_auto_landing_rate_final;
		float max_climb_rate = islanding ? (landing_rate + quadcopter_auto_landing_rate_final) : quadcopter_max_climb_rate;	// very low climbe rate even if max throttle in landing state
		
		float v = rc[2] - 0.5f;
		float user_rate;
		if (fabs(v)<0.05f)
			user_rate = 0;
		else if (v>= 0.05f)
		{
			user_rate = (v-0.05f)/0.45f;
			user_rate = user_rate * user_rate * max_climb_rate;
		}
		else
		{
			user_rate = (v+0.05f)/0.45f;
			user_rate = -user_rate * user_rate * quadcopter_max_descend_rate;
		}

		float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
		alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, LIMIT_NONE, airborne);
		
		// landing?
		if(islanding)
			user_rate -= landing_rate;

		alt_controller.update(interval, user_rate);
		
		throttle_result = alt_controller.get_result();

		TRACE("\rthr=%f/%f", throttle_result, alt_controller.get_result());
	}
	else
	{
		throttle_result = rc[2];
	}

	// attitude
	if (submode == basic || submode == althold)
	{
		if (after_unlock_action)	// airborne or armed and throttle up
		{
			float stick[3] = {rc[0], rc[1], NAN};
			attitude_controller.update_target_from_stick(stick, interval);
		}
		else
		{
			float euler_target[3] = {0,0, NAN};
			attitude_controller.set_euler_target(euler_target);
		}
	}

	else if (submode == optical_flow)
	{
		if (after_unlock_action)	// airborne or armed and throttle up
		{
			float stick_roll = rc[0] * quadcopter_range[0];
			float stick_pitch = -rc[1] * quadcopter_range[1];	// pitch stick and coordinate are reversed

			float flow_roll = -frame.flow_comp_m_x/1000.0f;
			float flow_pitch = -frame.flow_comp_m_y/1000.0f;
			of_controller.update_controller(flow_roll, flow_pitch, stick_roll, stick_pitch, interval);
			float euler_target[3] = {0,0, NAN};
			of_controller.get_result(&euler_target[0], &euler_target[1]);
			attitude_controller.set_euler_target(euler_target);
		}
		else
		{
			float euler_target[3] = {0,0, euler[2]};
			attitude_controller.set_euler_target(euler_target);
		}

		// TODO : handle yaw flow

	}

	else if (submode == bluetooth)
	{
		float euler_target[3] = {0,0, NAN};
		euler_target[0] = bluetooth_roll;
		euler_target[1] = bluetooth_pitch;
		attitude_controller.set_euler_target(euler_target);
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

				float euler_target[3];
				controller.provide_attitue_position(euler, ne_pos, ne_velocity);
				controller.set_desired_velocity(desired_velocity);
				controller.update_controller(dt);
				controller.get_target_angles(euler_target);
				euler_target[2] = NAN;
				attitude_controller.set_euler_target(euler_target);
			}
		}
	}

	// yaw:
	if (after_unlock_action)	// airborne or armed and throttle up
	{
		float yaw_array[3] = {NAN, NAN, rc[3]};
		attitude_controller.update_target_from_stick(yaw_array, interval);
	}
	else
	{
		float yaw_array[3] = {NAN, NAN, euler[2]};
		attitude_controller.set_euler_target(yaw_array);
	}

	// check takeoff
	if ( (alt_estimator.state[0] > takeoff_ground_altitude + 1.0f) ||
		(alt_estimator.state[0] > takeoff_ground_altitude && throttle_result > alt_controller.throttle_hover) ||
		(throttle_result > alt_controller.throttle_hover + QUADCOPTER_THROTTLE_RESERVE))
	{
		airborne = true;
	}
	
	attitude_controller.update(interval);

	return 0;
}

int yet_another_pilot::output()
{
	float pid_result[3];
	motor_saturated = false;
	attitude_controller.get_result(pid_result);
	if (armed)
	{
		int matrix = (float)motor_matrix;
		float motor_output[MAX_MOTOR_COUNT] = {0};

		
		// how many motor exists in this motor matrix?
		static int motor_count = 0;
		if(0==motor_count)
		for(int i=0; i<MAX_MOTOR_COUNT; i++)
		{
			if (quadcopter_mixing_matrix[matrix][i][0] == quadcopter_mixing_matrix[matrix][i][1] && 
				quadcopter_mixing_matrix[matrix][i][1] == quadcopter_mixing_matrix[matrix][i][2] && 
				fabs((float)quadcopter_mixing_matrix[matrix][i][2]) < 0.01f)
			{
				motor_count = i;
				break;
			}
		}

		// find how much motor power roll and pitch command requires
		float min_roll_pitch = 1.0f;
		float max_roll_pitch = 0.0f;
		for(int i=0; i<motor_count; i++)
		{
			motor_output[i] = 0;
			for(int j=0; j<2; j++)
				motor_output[i] += quadcopter_mixing_matrix[matrix][i][j] * pid_result[j] * QUADCOPTER_THROTTLE_RESERVE;
			
			min_roll_pitch = fmin(min_roll_pitch, motor_output[i]);
			max_roll_pitch = fmax(max_roll_pitch, motor_output[i]);
		}
		
		// handle saturation caused by roll and pitch alone
		float roll_pitch_factor = 1.0f;
		min_roll_pitch = 1.0f;
		max_roll_pitch = 0.0f;
		if (max_roll_pitch - min_roll_pitch > 1.0f)
			roll_pitch_factor = limit(roll_pitch_factor, 0.5f, 1.0f/(max_roll_pitch - min_roll_pitch));
		for(int i=0; i<motor_count; i++)
		{
			motor_output[i] = 0;
			for(int j=0; j<2; j++)
				motor_output[i] += quadcopter_mixing_matrix[matrix][i][j] * (roll_pitch_factor * pid_result[j]) * QUADCOPTER_THROTTLE_RESERVE;
			
			min_roll_pitch = fmin(min_roll_pitch, motor_output[i]);
			max_roll_pitch = fmax(max_roll_pitch, motor_output[i]);
		}
		
		// add throttle 
		float min_throttle = limit(-min_roll_pitch, 0, 1);
		float max_throttle = limit(1-max_roll_pitch, 0, 1);
		float throttle = limit(throttle_result, min_throttle, max_throttle);
		for(int i=0; i<motor_count; i++)
			motor_output[i] += throttle;
		
		// try ouput max possible yaw command
		float yaw_factor = 1.0f;
		for(int i=0; i<motor_count; i++)
		{
			float yaw_power = quadcopter_mixing_matrix[matrix][i][2] * pid_result[2] * QUADCOPTER_THROTTLE_RESERVE;
			
			if (yaw_power < 0)
			{
				yaw_factor = fmin(yaw_factor, fmax(0, motor_output[i]) / -yaw_power);
			}
			else
			{
				yaw_factor = fmin(yaw_factor, (1-motor_output[i]) / yaw_power);
			}
		}
		for(int i=0; i<motor_count; i++)
		{
			motor_output[i] += quadcopter_mixing_matrix[matrix][i][2] * yaw_factor * pid_result[2] * QUADCOPTER_THROTTLE_RESERVE;
			g_ppm_output[i] = limit(THROTTLE_IDLE + motor_output[i]*(THROTTLE_MAX-THROTTLE_IDLE), THROTTLE_IDLE, THROTTLE_MAX);
		}
		
		// statics
		throttle_real = 0;
		for(int i=0; i<motor_count; i++)
			throttle_real += motor_output[i];
		throttle_real /= motor_count;
		
		// placebo motor saturation detector.
		for(int i=0; i<motor_count; i++)
		{
			if (g_ppm_output[i] <= THROTTLE_IDLE+20 || g_ppm_output[i] >= THROTTLE_MAX-20)
				motor_saturated = true;
		}

	}

	else
	{
		STOP_ALL_MOTORS();
	}

	output_rc();
	return 0;
}



// called by main loop, only copy logs to a memory buffer, should be very fast
int yet_another_pilot::save_logs()
{
	if (LOG_LEVEL == LOG_SDCARD	&& !log_ready)
		return 0;

	// if the saving task is transferring logs into 2nd buffer
	if (log_pending !=0)
		return -1;

	// send/store debug data
	systime = systimer->gettime();
	sensor_data sensor =
	{
		{mag_uncalibrated.array[0] * 10, mag_uncalibrated.array[1] * 10, mag_uncalibrated.array[2] * 10},
		{accel_uncalibrated.array[0] * 100, accel_uncalibrated.array[1] * 100, accel_uncalibrated.array[2] * 100},
		mpu6050_temperature * 100 - 10000,
		{gyro_uncalibrated.array[0] * 18000/PI, gyro_uncalibrated.array[1] * 18000/PI, gyro_uncalibrated.array[2] * 18000/PI},
		voltage * 1000,
		current * 1000,
	};
	log(&sensor, TAG_SENSOR_DATA, systime);

	imu_data imu = 
	{
		a_raw_pressure,
		a_raw_temperature,
	
		{body_rate.array[0] * 18000/PI, body_rate.array[1] * 18000/PI, body_rate.array[2] * 18000/PI},
		{accel.array[0] * 100, accel.array[1] * 100, accel.array[2] * 100},
		{mag.array[0] * 10, mag.array[1] * 10, mag.array[2] * 10},
	};
	log(&imu, TAG_IMU_DATA, systime);
	log(&frame, TAG_PX4FLOW_DATA, systime);
	
	position p = estimator.get_estimation();
	position_meter pmeter = estimator.get_estimation_meter();
	ned_data ned = 
	{
		0,
		{accel_earth_frame.array[0] * 1000, accel_earth_frame.array[1] * 1000, accel_earth_frame.array[2] * 1000},
		p.latitude * double(10000000.0/COORDTIMES), 
		p.longtitude * double(10000000.0/COORDTIMES), 
		error_lat : pmeter.vlatitude*100,
		error_lon : pmeter.vlongtitude*100,
	};

	log(&ned, TAG_NED_DATA, systime);


	pilot_data pilot = 
	{
		alt_estimator.state[0] * 100,
		0,//airspeed_sensor_data * 1000,
		{attitude_controller.pid[0][0]*180*100/PI, attitude_controller.pid[1][0]*180*100/PI, attitude_controller.pid[2][0]*180*100/PI},
		{attitude_controller.body_rate_sp[0]*180*100/PI, attitude_controller.body_rate_sp[1]*180*100/PI, attitude_controller.body_rate_sp[2]*180*100/PI},
		armed ? 1 : 0,
		mah_consumed,
	};

	log(&pilot, TAG_PILOT_DATA, systime);

	pilot_data2 pilot2 = 
	{
		{attitude_controller.pid[0][1]*180*100/PI, attitude_controller.pid[1][1]*180*100/PI, attitude_controller.pid[2][1]*180*100/PI},
		{attitude_controller.pid[0][2]*180*100/PI, attitude_controller.pid[1][2]*180*100/PI, attitude_controller.pid[2][2]*180*100/PI},
	};

	log(&pilot2, TAG_PILOT_DATA2, systime);

	ppm_data ppm = 
	{
		{g_pwm_input[0], g_pwm_input[1], g_pwm_input[2], g_pwm_input[3], g_pwm_input[4], g_pwm_input[5]},
		{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
	};

	log(&ppm, TAG_PPM_DATA, systime);

	quadcopter_data quad = 
	{
		euler[0] * 18000/PI, euler[1] * 18000/PI, euler[2] * 18000/PI,
		attitude_controller.euler_sp[0] * 18000/PI, attitude_controller.euler_sp[1] * 18000/PI, attitude_controller.euler_sp[2] * 18000/PI,
		body_rate.array[0] * 18000/PI, body_rate.array[1] * 18000/PI, body_rate.array[2] * 18000/PI,
		attitude_controller.body_rate_sp[0] * 18000/PI,  attitude_controller.body_rate_sp[1] * 18000/PI, attitude_controller.body_rate_sp[2] * 18000/PI, 
	};

	log(&quad, TAG_QUADCOPTER_DATA, systime);


	quadcopter_data2 quad2 = 
	{
		alt_estimator.state[1] * 100,
		airborne,
		submode,
		alt_estimator.state[0] * 100,
		alt_estimator.state[2] * 100,
		a_raw_altitude * 100,
		0,//accelz_mwc * 100,
		loop_hz,
		THROTTLE_IDLE + throttle_result * (THROTTLE_MAX-THROTTLE_IDLE),
		kalman_accel_bias : alt_estimator.state[3] * 1000,
		{gyro_bias[0] * 1800000/PI, gyro_bias[1] * 1800000/PI, gyro_bias[2] * 1800000/PI,}
	};

	log(&quad2, TAG_QUADCOPTER_DATA2, systime);

	float mag_size = sqrt(mag.array[0]*mag.array[0]+mag.array[1]*mag.array[1]+mag.array[2]*mag.array[2]);
	float erra = err_a[0] * err_a[0] + err_a[1] * err_a[1] + err_a[2] * err_a[2];
	float errm = err_m[0] * err_m[0] + err_m[1] * err_m[1] + err_m[2] * err_m[2];
	quadcopter_data4 quad4 = 
	{
		isnan(alt_controller.m_sonar_target) ? 0 : alt_controller.m_sonar_target*100,
		mag_size,
		halfvx*2000,
		halfvy*2000,
		halfvz*2000,
		raw_yaw * 18000 / PI,
		acc_horizontal[0] * 1000,
		acc_horizontal[1] * 1000,
		{erra * 18000/PI, errm * 18000/PI, err_a[2] * 18000/PI},
	};
	log(&quad4, TAG_QUADCOPTER_DATA4, systime);

	extern uint32_t lost1;
	extern uint32_t lost2;
	quadcopter_data5 quad5 = 
	{
		{q0,q1,q2,q3},
		ground_speed_north*100,
		ground_speed_east*100,
		round_running_time,
		motor_saturated,
	};
	log(&quad5, TAG_QUADCOPTER_DATA5, systime);

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

	log(&quad3, TAG_QUADCOPTER_DATA3, systime);

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
	log(&pc, TAG_POS_CONTROLLER_DATA1, systime);


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

	log(&pc2, TAG_POS_CONTROLLER_DATA2, systime);

	if (last_gps_tick > systimer->gettime() - 2000000)
	{
		static unsigned short gps_id = 0;
		::gps_data data = 
		{
			{gps.DOP[0], gps.DOP[1], gps.DOP[2]},
			gps.speed*100,
			gps.longitude * 10000000, gps.latitude * 10000000, gps.altitude,
			gps.satelite_in_view, gps.satelite_in_use,
			gps.sig, gps.fix,
			gps_id++ & 0xf,
			gps.direction,
		};

		log(&data, TAG_GPS_DATA, systime);
	}

	rc_mobile_data mobile = 
	{
		{rc_mobile[0] * 1000, rc_mobile[0] * 1000, rc_mobile[0] * 1000, rc_mobile[0] * 1000,},
		min((systimer->gettime() - mobile_last_update)/1000, 65535),
	};
	log(&mobile, TAG_MOBILE_DATA, systime);
	
	return 0;
}

int yet_another_pilot::read_sensors()
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
		{
			static int last_frame_count = 0;
			if (last_frame_count != frame.frame_count && vcp && (usb_data_publish & data_publish_flow))
			{
				last_frame_count = frame.frame_count;

				if (usb_data_publish & data_publish_binary)
				{
					usb_flow_data data = 
					{
						systimer->gettime(),
						frame.pixel_flow_x_sum,
						frame.pixel_flow_y_sum,
						frame.ground_distance,
					};

					send_package(&data, sizeof(data), data_publish_flow, vcp);
				}
				else
				{
					char tmp[100];
					sprintf(tmp, "flow:%.3f,%d,%d,%f\n", systimer->gettime()/1000000.0f, frame.pixel_flow_x_sum, frame.pixel_flow_y_sum, frame.ground_distance/1000.0f);
					vcp->write(tmp, strlen(tmp));
				}
			}

			sonar_distance = frame.ground_distance / 1000.0f;
			if (sonar_distance <= SONAR_MIN || sonar_distance >= SONAR_MAX)
				sonar_distance = NAN;
			
			//float flowx = frame.pixel_flow_x_sum - body_rate.array[0] * 18000/PI * 0.006f;
			//float floay = frame.pixel_flow_y_sum - body_rate.array[1] * 18000/PI * 0.006f;

			//frame.gyro_x_rate = flowx;
			//frame.gyro_y_rate = floay;
		}
	}
	
	if (range_finder)
	{
		range_finder->trigger();
		float distance = 0;
		if (0 == range_finder->read(&distance))
		{
			if (distance <= SONAR_MIN || distance >= SONAR_MAX)
				sonar_distance = NAN;
			else
				sonar_distance = distance;
		}
		
		if (isnan(sonar_distance))
			printf("\rNAN     ");
		else
			printf("\r%.3f", sonar_distance);
	}

	// read usart source
	handle_cli(vcp);

	

	// read gyros
	int healthy_gyro_count = 0;
	for(int i=0; i<min(manager.get_gyroscope_count(),1); i++)
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
	for(int i=0; i<min(manager.get_accelerometer_count(),1); i++)
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
	
	// apply a 2nd order LPF to gyro readings
	for(int i=0; i<3; i++)
		this->gyro_reading.array[i] = gyro_lpf2p[i].apply(gyro.array[i]);

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

	if (vcp && (usb_data_publish & data_publish_imu))
	{
		gyro_data gyro2 = {0};
		accelerometer_data acc2 = {0};

		if (manager.get_accelerometer_count()>=2)
			manager.get_accelerometer(1)->read(&acc2);
		if (manager.get_gyroscope_count()>=2)
			manager.get_gyroscope(1)->read(&gyro2);

		if (usb_data_publish & data_publish_binary)
		{
			usb_imu_data data = 
			{
				systimer->gettime(),
				{acc.V.x * 1000, acc.V.y * 1000, acc.V.z * 1000,},
				{gyro.V.x * 18000 / PI, gyro.V.y * 18000 / PI, gyro.V.z * 18000 / PI,},
				{acc2.x * 1000, acc2.y * 1000, acc2.z * 1000,},
				{gyro2.x * 18000 / PI, gyro2.y * 18000 / PI, gyro2.z * 18000 / PI,},
				{mag.V.x, mag.V.y, mag.V.z,},
			};

			send_package(&data, sizeof(data), data_publish_imu, vcp);
		}
		else
		{
			char tmp[200];
			sprintf(tmp, 
				"imu:%.4f"
				",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f"
				",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", systimer->gettime()/1000000.0f,
			acc.V.x, acc.V.y, acc.V.z, gyro.V.x, gyro.V.y, gyro.V.z, mag.V.x, mag.V.y, mag.V.z, mpu6050_temperature,
			acc2.x, acc2.y, acc2.z, gyro2.x, gyro2.y, gyro2.z);
			
			vcp->write(tmp, strlen(tmp));
		}
	}


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

	if (vcp && (usb_data_publish & data_publish_baro) && new_baro_data)
	{
		if (usb_data_publish & data_publish_binary)
		{
			usb_baro_data data =
			{
				systimer->gettime(),
				a_raw_pressure,
				a_raw_temperature*100
			};

			send_package(&data, sizeof(data), data_publish_baro, vcp);
		}
		else
		{
			char tmp[200];
			sprintf(tmp, "baro:%.3f,%.0f,%.3f\n", systimer->gettime()/1000000.0f, a_raw_pressure, a_raw_temperature);
			vcp->write(tmp, strlen(tmp));
		}
	}

	// read GPSs
	int lowest_hdop = 100000;
	new_gps_data = false;
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
			this->gps = data;
			new_gps_data = (res == 0);
			last_gps_tick = systimer->gettime();
		}
	}
	if (manager.get_GPS_count() == 0)
		critical_errors |= error_GPS;

	if (vcp && (usb_data_publish & data_publish_gps) && new_gps_data)
	{
		if (usb_data_publish & data_publish_binary)
		{
			usb_gps_data data = 
			{
				systimer->gettime(),
				gps.latitude,
				gps.longitude,
				gps.altitude*100,
				gps.DOP[1],
				gps.speed,
				gps.direction,
			};

			send_package(&data, sizeof(data), data_publish_gps, vcp);
		}
		else
		{
			char tmp[200];
			sprintf(tmp, "gps:%.3f,%.6f,%.6f,%.2f,%.2f,%.2f,%.0f\n", systimer->gettime()/1000000.0f,
			gps.latitude, gps.longitude, gps.altitude, gps.DOP[1]/100.0f, gps.speed, gps.direction);
			vcp->write(tmp, strlen(tmp));
		}
	}


	// bias and scale calibrating
	float temperature_delta = mpu6050_temperature - temperature0;
	float gyro_bias[3] = 
	{
		-(temperature_delta * gyro_temp_k.array[0] + gyro_temp_a.array[0]),
		-(temperature_delta * gyro_temp_k.array[1] + gyro_temp_a.array[1]),
		-(temperature_delta * gyro_temp_k.array[2] + gyro_temp_a.array[2]),
	};
	accel_uncalibrated = acc;
	mag_uncalibrated = mag;
	gyro_uncalibrated = gyro_reading;
	
	for(int i=0; i<3; i++)
	{
		acc.array[i] += acc_bias[i];
		acc.array[i] *= acc_scale[i];
		mag.array[i] += mag_bias[i];
		mag.array[i] *= mag_scale[i];
		gyro_reading.array[i] += gyro_bias[i];
	}
	
	this->mag = mag;
	

	// apply a 5hz LPF to accelerometer readings
	const float RC20 = 1.0f/(2*3.1415926 * 20.0f);
	float alpha20 = interval / (interval + RC20);

	if (isnan(accel.array[0]) || interval == 0)
		this->accel = acc;
	else
	{
		vector_multiply(&this->accel, 1-alpha20);
		vector_multiply(&acc, alpha20);
		vector_add(&this->accel, &acc);
	}

	// accelerometer motion detector and calibration.
	motion_acc.new_data(accel_uncalibrated);
	if (motion_acc.get_average(NULL) > 100)
	{
		vector avg;
		int avg_counter = motion_acc.get_average(&avg);
		int i = -1;
		if (avg.array[0] < -8.5f)			// 3 times of datasheet "typical" zero G output, +-150mg for XY, +-280mg for Z.
			i = 0;
		else if (avg.array[0] > 8.5f)
			i = 1;
		else if (avg.array[1] < -8.5f)
			i = 2;
		else if (avg.array[1] > 8.5f)
			i = 3;
		else if (avg.array[2] < -7.2f)
			i = 4;
		else if (avg.array[2] > 7.2f)
			i = 5;

		if (i>=0 && i<6)
		{
			if (avg_counter > acc_avg_count[i])
			{
				acc_avg_count[i] = avg_counter;
				acc_calibrator[i] = avg;
			}
		}
	}
	finish_accel_cal();

	// statics
	for(int i=0; i<3; i++)
	{
		imu_statics[0][0].array[i] = fmin(accel.array[i], imu_statics[0][0].array[i]);
		imu_statics[0][1].array[i] = accel.array[i];
		imu_statics[0][2].array[i] = fmax(accel.array[i], imu_statics[0][2].array[i]);
		imu_statics[0][3].array[i] = accel.array[i] + imu_statics[0][3].array[i];

		imu_statics[1][0].array[i] = fmin(gyro.array[i], imu_statics[1][0].array[i]);
		imu_statics[1][1].array[i] = gyro.array[i];
		imu_statics[1][2].array[i] = fmax(gyro.array[i], imu_statics[1][2].array[i]);
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

int yet_another_pilot::calculate_state()
{
	if (interval <=0 || interval > 0.2f)
		return -1;

	float factor = 1.0f;
	float factor_mag = 1.0f;
	
	NonlinearSO3AHRSupdate(
		accel.array[0], accel.array[1], accel.array[2], 
		mag.array[0], mag.array[1], mag.array[2],
		gyro_reading.array[0], gyro_reading.array[1], gyro_reading.array[2],
		0.15f*factor, 0.0015f, 0.15f*factor_mag, 0.0015f, interval);

	euler[0] = radian_add(euler[0], quadcopter_trim[0]);
	euler[1] = radian_add(euler[1], quadcopter_trim[1]);
	euler[2] = radian_add(euler[2], quadcopter_trim[2]);
	
	body_rate.array[0] = gyro_reading.array[0] + gyro_bias[0];
	body_rate.array[1] = gyro_reading.array[1] + gyro_bias[1];
	body_rate.array[2] = gyro_reading.array[2] + gyro_bias[2];

	float mag_size = sqrt(mag.array[0]*mag.array[0]+mag.array[1]*mag.array[1]+mag.array[2]*mag.array[2]);
	TRACE("mag_size:%.3f, %.0f, %.0f, %.0f    \n", mag_size, mag.array[0], mag.array[1], mag.array[2]);
	TRACE("euler:%.2f,%.2f,%.2f, systime:%f, bias:%.2f/%.2f/%.2f, pressure=%.2f \n ", euler[0]*PI180, euler[1]*PI180, euler[2]*PI180, systimer->gettime()/1000000.0f, gyro_bias[0]*PI180, gyro_bias[1]*PI180, gyro_bias[2]*PI180, a_raw_pressure);

	for(int i=0; i<3; i++)
		accel_earth_frame.array[i] = acc_ned[i];

	// calculate altitude
	if (new_baro_data)
		calculate_baro_altitude();

	accelz = acc_ned[2];

	alt_estimator.set_land_effect(armed && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - takeoff_ground_altitude) < 1.0f));
	alt_estimator.update(accelz, a_raw_altitude, interval);
	alt_estimator.set_static_mode(!armed);
	alt_estimatorCF.set_land_effect(armed && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - takeoff_ground_altitude) < 1.0f));
	alt_estimatorCF.update(accelz, new_baro_data ? a_raw_altitude : NAN, interval);
	estimator.update_accel(accel_earth_frame.array[0], accel_earth_frame.array[1], systimer->gettime());

	if (new_gps_data)
	{
		// fuse gps data
		if (gps.fix > 2)
		{
			estimator.update_gps(COORDTIMES * gps.latitude, COORDTIMES * gps.longitude, gps.DOP[1]/100.0f, systimer->gettime());

			log_set_time(gps.timestamp);
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

int yet_another_pilot::sensor_calibration()
{
	for(int i=0; i<3; i++)
	{
		if (isnan(mag_bias[i]))
			mag_bias[i] = 0;
		if (isnan(mag_scale[i]))
			mag_scale[i] = 1;
	}

	
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
	motion_detector detect_acc;
	motion_detector detect_gyro;
	vector mag_avg = {0};
	//vector accel_avg = {0};
	//vector gyro_avg = {0};
	
	motion_acc.set_threshold(1);
	detect_acc.set_threshold(1);
	detect_gyro.set_threshold(5 * PI / 180);


	int baro_counter = 0;
	ground_pressure = 0;
	int calibrating_count = 900;
	ground_temperature = 0;
	
	while(detect_acc.get_average(NULL) < calibrating_count && detect_gyro.get_average(NULL) < calibrating_count)
	{
		read_sensors();
		if (critical_errors)
			return -2;
		if (detect_gyro.new_data(gyro_reading))
			return -1;
		if (detect_acc.new_data(accel))
			return -1;
		systimer->delayus(3000);

		LOGE("\r%d/%d", detect_acc.get_average(NULL), calibrating_count);

		vector_add(&mag_avg, &mag);
		if (new_baro_data)
		{
			baro_counter ++;
			ground_pressure += a_raw_pressure;
			ground_temperature += a_raw_temperature;
		}
		
		if ((systimer->gettime()/1000)%50 > 25)
		{
			SAFE_ON(state_led);
			SAFE_ON(SD_led);
		}
		else
		{
			SAFE_OFF(state_led);
			SAFE_OFF(SD_led);
		}

		if (rgb)
		{
			float color[5][3] = 
			{
				{1,0,0},
				{0,1,0},
				{0,0,1},
				{0.2,0,1},
				{0.2,1,0},
			};

			int i = (systimer->gettime() / 150000) % 5;
			rgb->write(color[i][0], color[i][01], color[i][2]);
		}
	}
	
	vector_divide(&mag_avg, calibrating_count);
	ground_pressure /= baro_counter;
	ground_temperature /= baro_counter;

	LOGE("base value measured\n");
	
	vector accel_avg;
	vector gyro_avg;
	detect_gyro.get_average(&gyro_avg);
	detect_acc.get_average(&accel_avg);

	NonlinearSO3AHRSinit(accel_avg.V.x, accel_avg.V.y, accel_avg.V.z, 
		mag_avg.V.x, mag_avg.V.y, mag_avg.V.z, 
		gyro_avg.V.x, gyro_avg.V.y, gyro_avg.V.z);

	return 0;
}

int yet_another_pilot::set_submode(copter_mode newmode)
{
	if (!armed)
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
		float euler_target[3];

		controller.provide_attitue_position(euler, ne_pos, ne_velocity);
		controller.set_desired_velocity(desired_velocity);
		controller.get_target_angles(euler_target);
		controller.reset();
		euler_target[2] = NAN;
		attitude_controller.set_euler_target(euler_target);
	}

	if (!has_alt_controller && to_use_alt_controller)
	{
		float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
		alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, LIMIT_NONE, airborne);
		alt_controller.reset();
	}

	if (newmode == optical_flow && submode != optical_flow)
	{
		of_controller.reset();
	}
	
	submode = newmode;

	return 0;
}

int yet_another_pilot::arm(bool arm /*= true*/)
{
	if (arm == armed)
		return 0;

	attitude_controller.provide_states(euler, body_rate.array, motor_saturated ? LIMIT_ALL : LIMIT_NONE, airborne);
	attitude_controller.reset();
	

	takeoff_ground_altitude = alt_estimator.state[0];
	yaw_launch = euler[2];
	collision_detected = 0;
	tilt_us = 0;
	throttle_result = 0;
	airborne = false;
	islanding = false;
	float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
	alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, LIMIT_NONE, airborne);
	alt_controller.reset();
	
	armed = arm;
	return 0;
}

int yet_another_pilot::disarm()
{
	return arm(false);
}

void yet_another_pilot::reset_mag_cal()
{
	LOGE("start magnetometer calibrating");
	mag_calibrator.reset();
	mag_calibration_state = 1;
}

void yet_another_pilot::reset_accel_cal()
{
	memset(acc_avg_count, 0, sizeof(acc_avg_count));
	acc_cal_requested = true;
	acc_cal_done = false;
}

int yet_another_pilot::finish_accel_cal()
{
	if (!acc_cal_requested || acc_cal_done)
		return 0;

	for(int i=0; i<6; i++)
		if (acc_avg_count[i] < 100)
			return -(i+1);

	// calculate
	gauss_newton_sphere_fitting fitter;
	float data[6*3];
	for(int i=0; i<6; i++)
	{
		data[i*3+0] = acc_calibrator[i].array[0];
		data[i*3+1] = acc_calibrator[i].array[1];
		data[i*3+2] = acc_calibrator[i].array[2];
	}

	fitter.calculate(data, 6);
	fitter.get_result(data);	// bias[0~2], scale[0~2]

	data[3] *= G_in_ms2;
	data[4] *= G_in_ms2;
	data[5] *= G_in_ms2;
	
	// apply and save
	acc_bias[0] = data[0];
	acc_bias[1] = data[1];
	acc_bias[2] = data[2];
	acc_scale[0] = data[3];
	acc_scale[1] = data[4];
	acc_scale[2] = data[5];

	acc_bias[0].save();
	acc_bias[1].save();
	acc_bias[2].save();
	acc_scale[0].save();
	acc_scale[1].save();
	acc_scale[2].save();

	LOGE("acc bias: %.3f, %.3f, %.3f, scale: %.3f, %.3f, %.3f\n", data[0], data[1], data[2], data[3], data[4], data[5]);
	acc_cal_done = true;

	return 0;
}

int yet_another_pilot::check_stick()
{
	if (critical_errors)
	{
		disarm();
		return -1;
	}

	// sub mode swtich
	if (g_pwm_input_update[5] > systimer->gettime() - 500000)
	{
		copter_mode newmode = submode;
		if (rc[5] < -0.6f)
			newmode = basic;
		else if (rc[5] > 0.6f)
// 			newmode = airborne ? optical_flow : althold;
// 			newmode = (bluetooth_last_update > systimer->gettime() - 500000) ? bluetooth : althold;
			newmode = airborne ? (estimator.healthy() ? poshold : optical_flow) : althold;
		else if (rc[5] > -0.5f && rc[5] < 0.5f)
 			newmode = airborne ? optical_flow : althold;
//			newmode = althold;

		set_submode(newmode);
	}

	if (g_pwm_input_update[4] > systimer->gettime() - 500000)
	{		
		// emergency switch
		// magnetometer calibration starts if flip emergency switch 10 times, interval systime between each flip should be less than 1 second.
		static float last_ch4 = 0;
		if (fabs(rc[4]-last_ch4) > 0.20f)
		{
			disarm();
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
				reset_mag_cal();
			}
		}

		// arm action check: RC first four channel active, throttle minimum, elevator stick down, rudder max or min, aileron max or min, for 0.5second
		static int64_t arm_start_tick = 0;
		static int64_t arm_command_sent = false;
		bool rc_fail = false;
		for(int i=0; i<4; i++)
			if (g_pwm_input_update[i] < systimer->gettime() - 500000)
				rc_fail = true;
		bool arm_action = !rc_fail && rc[2] < 0.1f  && fabs(rc[0]) > 0.85f
						&& fabs(rc[1]) > 0.85f && fabs(rc[3]) > 0.85f;
		if (!arm_action)
		{
			arm_start_tick = 0;
			arm_command_sent = false;
		}
		else
		{
			if (arm_start_tick > 0 && !arm_command_sent)
			{
				if (systimer->gettime() - arm_start_tick > 500000)
				{
					arm_command_sent = true;
					
					if (armed)
					{
						disarm();
						LOGE("disarmed!\n");
					}
					else
					{
						arm();
						LOGE("armed!\n");
					}
				}
			}
			else
			{
				arm_start_tick = systimer->gettime();
			}
		}
	}

	return 0;
}

static float takeoff_arming_time = 0;
static bool is_taking_off=false;
void yet_another_pilot::check_takeoff_OR_landing()
{
	static float last_ch7 = NAN;
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
		else if(is_taking_off==false&&!armed && flip_count >=1)
		{
			flip_count=0;
			LOGE("\nauto take off \n");
			arm();
			check_stick();	// to set submode and reset all controller
			alt_controller.set_altitude_target(alt_controller.get_altitude_target()-2.0f);
			LOGE("\narmed!\n");
			//1s=1000000us		
			takeoff_arming_time=systimer->gettime();
			is_taking_off=true;
		}
	}
	if(is_taking_off==true)
	{
		time_now=systimer->gettime();
		if(time_now>=takeoff_arming_time+2000000)
		{
			is_taking_off=false;
			alt_controller.set_altitude_target(alt_controller.get_altitude_target()+2.0f);
			LOGE("\nalread take off\n");
		}
	}
}

int64_t land_detect_us = 0;
int yet_another_pilot::land_detector()
{
	if ((rc[2] < 0.1f || (islanding && throttle_result < 0.2f))					// landing and low throttle output, or just throttle stick down
		&& fabs(alt_estimator.state[1]) < (quadcopter_max_descend_rate/4.0f)	// low climb rate : 25% of max descend rate should be reached in such low throttle, or ground was touched
// 		&& fabs(alt_estimator.state[2] + alt_estimator.state[3]) < 0.5f			// low acceleration
	)
	{
		land_detect_us = land_detect_us == 0 ? systimer->gettime() : land_detect_us;

		if (systimer->gettime() - land_detect_us > (airborne ? 1000000 : 30000000))		// 30 seconds for before take off, 1 senconds for landing
		{
			disarm();
			LOGE("landing detected");
		}
	}
	else
	{
		land_detect_us = 0;
	}
	
	return 0;
}

int yet_another_pilot::crash_detector()
{
	// always detect high G force
	vector accel_delta;
	
	accel_delta.array[0] = accel.array[0]/G_in_ms2 - 2 * halfvx;
	accel_delta.array[1] = accel.array[1]/G_in_ms2 - 2 * halfvy;
	accel_delta.array[2] = accel.array[2]/G_in_ms2 - 2 * halfvz;

	float gforce = vector_length(&accel_delta);
	
	static float gforce_sum = 0;
	static float gforce_sum2 = 0;
	static int gforce_count = 0;
	float gforce_avg = 1.0f;
	float gforce_variance = 1.0f;
	if (airborne)
	{
		gforce_count ++;
		gforce_sum += gforce;
		gforce_sum2 += gforce * gforce;
	}
	if (gforce_count > 0)
	{
		gforce_avg =  gforce_sum / gforce_count;
		gforce_variance = gforce_sum2 / gforce_count - gforce_avg * gforce_avg;
	}
	
	if (gforce > 2.75f)
	{
		TRACE("high G force (%.2f) detected\n", gforce);
		collision_detected = systimer->gettime();
	}

	// forced shutdown if >7g external force
	if (gforce > 5.5f)
	{
		LOGE("very high G force (%.2f) detected (%.0f,%.0f,%.0f)\n", gforce, accel.array[0], accel.array[1], accel.array[2]);
		disarm();
	}

	int prot = (float)::crash_protect;

	bool landing_requested = rc[2] < 0.1f || (islanding && throttle_result < 0.2f);
		
	// tilt detection
	float cos0 = cos(euler[0]);
	float cos1 = cos(euler[1]);
	float tilt = sqrt(cos0*cos0 + cos1*cos1);
	if (tilt < 0.33f || cos0 < 0 || cos1 < 0)		// around 70 degree
		tilt_us = tilt_us > 0 ? tilt_us : systimer->gettime();
	else
		tilt_us = 0;

	if ((collision_detected > 0 && systimer->gettime() - collision_detected < 100000) && (landing_requested || prot & CRASH_COLLISION_IMMEDIATE)) 
	{
		LOGE("landing impact detected(%s)\n", (collision_detected > 0 && systimer->gettime() - collision_detected < 5000000) ? "collision" : "tilt");

		disarm();
	}

	if (tilt_us> 0 && systimer->gettime()-tilt_us > 2000000)	// flip over more than 2 second
	{
		LOGE("flip over for more than 1 senconds");

		disarm();
	}

	return 0;
}

float yet_another_pilot::ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = limit(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}

int yet_another_pilot::handle_cli(IUART *uart)
{
	if (!uart)
		return -1;
	
	char line[1024];
	char out[1024];
	int byte_count = uart->readline(line, sizeof(line));
	if (byte_count <= 0)
		return 0;
	
	line[byte_count-1] = 0;
	
	int out_count = parse_command_line(line, out);
	if (out_count>0)
	{
		out[out_count] = 0;
		TRACE("%s,%s\n", line, out);
		uart->write(out, out_count);
	}

	return 0;
}

int yet_another_pilot::handle_wifi_controll()
{
	char line[1024];
	char out[1024];
	IUART *uart = manager.getUART("AUX");
	if (!uart)
		return -1;
	
	int byte_count = uart->readline(line, sizeof(line));
	if (byte_count <= 0)
		return 0;
	
	line[byte_count] = 0;
	int len = strlen(line);
	const char * keyword2 = ",stick\n";
	
	TRACE("%s\n", line);

	if (strstr(line, keyword2) == (line+len-strlen(keyword2)))
	{
		if (sscanf(line, "%f,%f,%f,%f", &rc_mobile[0], &rc_mobile[1], &rc_mobile[2], &rc_mobile[3] ) == 4)
		{
			mobile_last_update = systimer->gettime();
			TRACE("stick:%f,%f,%f,%f\n", rc_mobile[0], rc_mobile[1], rc_mobile[2], rc_mobile[3]);
			
			rc_mobile[0] *= 1.33f;
			rc_mobile[1] *= 1.33f;
			rc_mobile[2] = (rc_mobile[2] - 0.5f) * 1.35f + 0.5f;
			rc_mobile[3] *= 1.33f;
		}
	}

	else if (strcmp(line, "arm\n") == 0)
	{
		LOGE("mobile arm\n");
		arm();
		check_stick();	// to set submode and reset all controller
		alt_controller.set_altitude_target(alt_controller.get_altitude_target()-2.0f);

		const char *armed = "armed\n";
		uart->write(armed, strlen(armed));
	}

	else if (strcmp(line, "disarm\n") == 0)
	{
		LOGE("mobile disarm\n");

		disarm();
		const char *disarmed = "disarmed\n";
		uart->write(disarmed, strlen(disarmed));
	}
	
	else if (strcmp(line, "takeoff\n") == 0)
	{
		const char *tak = "taking off\n";
		uart->write(tak, strlen(tak));

		LOGE("\nauto take off \n");
		arm();
		check_stick();	// to set submode and reset all controller
		alt_controller.set_altitude_target(alt_controller.get_altitude_target()-2.0f);
		LOGE("\narmed!\n");
		//1s=1000000us		
		takeoff_arming_time=systimer->gettime();
		is_taking_off=true;
	}

	else if (strcmp(line, "land\n") == 0)
	{
		LOGE("mobile land\n");
		const char *land = "landing\n";
		uart->write(land, strlen(land));

		islanding = true;
	}
	
	else if (strcmp(line, "RTL\n") == 0)
	{
		LOGE("mobile RTL\n");
		const char *rtl = "RTLing\n";
		uart->write(rtl, strlen(rtl));
	}

	else
	{
		// invalid packet
	}

	return 0;
}

int yet_another_pilot::handle_uart4_controll()
{
#if 0
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

int yet_another_pilot::read_rc()
{
	rcin->get_channel_data(g_pwm_input, 0, 8);
	rcin->get_channel_update_time(g_pwm_input_update, 0, 8);
	TRACE("\rRC");
	int rc2_update_time = systimer->gettime() - g_pwm_input_update[2];
	for(int i=0; i<8; i++)
	{
		rc[i] = ppm2rc(g_pwm_input[i], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);
		TRACE("%.2f,", rc[i]);
	}

	rc[2] = (rc[2]+1)/2;
	
	// rc no signel for 0.5 seconds, or -10% or more throttle
	if (rc2_update_time > 500000 || g_pwm_input[2] < (rc_setting[2][0] - (rc_setting[2][2] - rc_setting[2][0])/10))
		rc_fail = true;
	
	// wifi controll override
	if (systimer->gettime() - mobile_last_update < 400000 && rc[6] < -0.5f)
	for(int i=0; i<4; i++)
	{
		rc[i] = rc_mobile[i];
		TRACE("%.2f,", rc_mobile[i]);
	}
	
	TRACE("                      ");
	
	return 0;
}

int yet_another_pilot::lowpower_handling()
{
	if (interval > 0.1f)
		return -1;
	
	static float lowpower1 = 0;
	static float lowpower2 = 0;

	if (voltage > 6 && voltage < 10.2f)
	{
		lowpower2 += interval;
	}
	
	else if (voltage > 6 && voltage<10.8f)
	{
		lowpower1 += interval;
		lowpower2 = 0;
	}

	else
	{
		lowpower1 = 0;
		lowpower2 = 0;
	}

	if (lowpower1 > 3)
		lowpower = max(lowpower, 1);

	if (lowpower2 > 3)
		lowpower = max(lowpower, 2);

	if (lowpower >= 2)
		islanding = true;

	return 0;
}

int yet_another_pilot::light_words()
{
	// critical errors
	if (critical_errors != 0)
	{
		static int last_critical_errors = 0;
		if (last_critical_errors != critical_errors)
		{
			last_critical_errors = critical_errors;
			LOGE("critical_errors : %x (", critical_errors);
			for(int i=0; (1<<i)<error_MAX; i++)
			{
				if ((1<<i) & critical_errors)
					LOGE("%s %s", i==0?"":" | ", critical_error_desc[i]);
			}
			LOGE(" )\n");
		}

		int error_code_count = 0;
		for(int i=0; (1<<i)<error_MAX; i++)
			error_code_count++;

		static int64_t t0 = 0;
		if (t0 == 0)
			t0 = systimer->gettime();
		int64_t t = (systimer->gettime()-t0) / 1000;
		int light = (t / 500) % (error_code_count+3);
		if (rgb)
		{
			if (t % 500 < 250 || light >=  error_code_count)
			{
				rgb->write(0,0,0);
			}
			else
			{
				if ((1<<light) & critical_errors)
				{
					rgb->write(1,0,0);
				}
				else
				{
					rgb->write(0,1,0);
				}
			}
		}
	}

	else if (lowpower)			// low voltage warning
	{
		// fast red flash (10hz)
		int dt = lowpower == 2 ? 50000 : 500000;
		systime = systimer->gettime();
		if (rgb && mag_calibration_state == 0)
		if (systime % (dt*2) < dt)
			rgb->write(1,0,0);
		else
			rgb->write(0,0,0);
	}
	else		// normal flashlight, double flash if SDCARD running, single flash if SDCARD failed
	{
		systime = systimer->gettime();
		int time_mod_1500 = (systime%1500000)/1000;
		if (time_mod_1500 < 20 || (time_mod_1500 > 200 && time_mod_1500 < 220 && log_ready))
		{
			if (rgb && mag_calibration_state == 0)
				rgb->write(estimator.healthy() ? 0 : 0.8,1,0);
			SAFE_ON(flashlight);
		}
		else
		{
			if(rgb && mag_calibration_state == 0)
				rgb->write(0,0,0);
			SAFE_OFF(flashlight);
		}
	}
	
	return 0;
}

void yet_another_pilot::mag_calibrating_worker()
{
	mag_calibrator.do_calibration();
	mag_calibration_result result;
	last_mag_calibration_result = mag_calibrator.get_result(&result);
	LOGE("result:%d, bias:%f,%f,%f, scale:%f,%f,%f\n, residual:%f/%f", last_mag_calibration_result, result.bias[0], result.bias[1], result.bias[2], result.scale[0], result.scale[1], result.scale[2], result.residual_average, result.residual_max);
	
	// do checks and flash RGB LED if error occured
	if (last_mag_calibration_result == 0)
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

	// log
	mag_calibration_data d =
	{
		{result.bias[0] * 10, result.bias[1] * 10, result.bias[2] * 10},
		{result.scale[0] * 1000, result.scale[1] * 1000, result.scale[2] * 1000},
		result.residual_average * 1000,
		result.residual_max * 1000,
		result.residual_min * 1000,
		last_mag_calibration_result,
		result.num_points_collected,
	};

	// assume blocking async worker won't do any harm
	int64_t timestamp = systimer->gettime();
	while (log_ready && log(&d, TAG_MAG_CALIBRATION_DATA, timestamp) != 0)
		LOGE("writing log failed");// retry

	// ends
	mag_calibration_state = 0;
}

void yet_another_pilot::main_loop(void)
{	
	// calculate systime interval
	static int64_t tic = 0;
	int64_t round_start_tick = systimer->gettime();
	interval = (round_start_tick-last_tick)/1000000.0f;
	last_tick = round_start_tick;
	
	// rc inputs
	read_rc();

	SAFE_OFF(state_led);
	
	// performance counter
	cycle_counter++;
	if (systimer->gettime() - tic > 1000000)
	{
		tic = systimer->gettime();
		LOGE("speed: %d, systime:%.2f\r\n", cycle_counter, systimer->gettime()/1000000.0f);
		loop_hz = cycle_counter;
		cycle_counter = 0;
	}
	
	// RC modes and RC fail detection, or alternative RC source
	check_stick();
	check_takeoff_OR_landing();
	handle_wifi_controll();

	// read sensors
	int64_t read_sensor_cost = systimer->gettime();
	read_sensors();
	read_sensor_cost = systimer->gettime() - read_sensor_cost;
	
	// provide mag calibration with data
	if (mag_calibration_state == 1)
	{
		// update RGB LED for user interaction
		if (rgb)
		{
			if (mag_calibrator.get_stage() == stage_horizontal)
				rgb->write(1,0,0);
			else if (mag_calibrator.get_stage() == stage_vertical_pitch)
				rgb->write(0,1,0);
			else if (mag_calibrator.get_stage() == stage_vertical_roll)
				rgb->write(0,0,1);
			else
				rgb->write(0,0,0);
		}

		
		// data
		mag_calibrator.provide_data(mag_uncalibrated.array, euler, body_rate.array, interval);
	
		// make a async call if data ready and change mag_calibration_state to calibrating
		if (mag_calibrator.get_stage() == stage_ready_to_calibrate)
		{
			mag_calibration_state = 2;
			manager.get_asyncworker()->add_work(mag_calibrating_worker_entry, (int)this);
		}
	}

	lowpower_handling();

	// light words
	light_words();

	// all state estimating, AHRS, position, altitude, etc
	calculate_state();

	run_controllers();
	output();
	save_logs();

	if (armed)
	{
		land_detector();
 		crash_detector();
	}
	else
		land_detect_us = 0;

	SAFE_ON(state_led);

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

	round_running_time = systimer->gettime() - round_start_tick;
	TRACE("\r%d/%d", int(read_sensor_cost), round_running_time);
}


void yet_another_pilot::sdcard_logging_loop(void)
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
		TRACE("long log systime:%d\n", int(starttick));

	if (res == 0)
		tick = t;
}

int yet_another_pilot::setup(void)
{	
	range_finder = (IRangeFinder *)manager.get_device("sonar");
	state_led = manager.getLED("state");
	SD_led = manager.getLED("SD");
	flashlight = manager.getLED("flashlight");
	rcin = manager.get_RCIN();
	rcout = manager.get_RCOUT();
	rgb = manager.getRGBLED("rgb");
	vcp = manager.getUART("VCP");

	for(int i=0; i<16; i++)
		g_ppm_output[i] = THROTTLE_MAX;
	output_rc();
	
	systimer->delayms(10);
	STOP_ALL_MOTORS();
	
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

	// get two timers, one for main loop and one for SDCARD logging loop
	manager.getTimer("mainloop")->set_period(3000);
	manager.getTimer("mainloop")->set_callback(main_loop_entry);
	manager.getTimer("log")->set_period(10000);
	manager.getTimer("log")->set_callback(sdcard_logging_loop_entry);
	
	return 0;
}

int main()
{
	bsp_init_all();
	
	while(0)
	{
		float t = systimer->gettime()/5000000.0f * 2 * PI;
		float t2 = systimer->gettime()/15000000.0f * 2 * PI;
		t2 = sin(t2)/2+0.6f;
		float r = t2*(sin(t)/2+0.5f);
		float g = t2*(sin(t+PI*2/3)/2+0.5f);
		float b = t2*(sin(t+PI*4/3)/2+0.5f);
		
		manager.getRGBLED("rgb")->write(r,g,b);
	}
	
	yap.setup();
	while(1)
	{
	}
}

yet_another_pilot yap;