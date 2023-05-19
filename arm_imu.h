#ifndef IMU_H
#define IMU_H

#include "arm_hardi2c.h"


typedef struct
{
	void (*current_function)(void *ptr);
	float mag_x;
	float mag_y;
	float mag_z;
	int mag_x_axis;
	int mag_y_axis;
	int mag_z_axis;
	int mag_x_sign;
	int mag_y_sign;
	int mag_z_sign;
	int mag_x_max;
	int mag_y_max;
	int mag_z_max;
	int mag_x_min;
	int mag_y_min;
	int mag_z_min;
	float mag_bandwidth;
	int mag_timeout;
// make the IMU start another conversion
	int want_mag;
// got a mag reading
	int got_mag;
	int calibrate_mag;
// dump raw values
	int calibrate_imu;
	int dump_theta;


	float accel_x;
	float accel_y;
	float accel_z;
	int accel_x_axis;
	int accel_y_axis;
	int accel_z_axis;
	int accel_x_sign;
	int accel_y_sign;
	int accel_z_sign;
	float accel_bandwidth;
	int got_accel;

// uncentered, lowpassed values
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float gyro_x_centered;
	float gyro_y_centered;
	float gyro_z_centered;
	int gyro_x_axis;
	int gyro_y_axis;
	int gyro_z_axis;
	int gyro_x_sign;
	int gyro_y_sign;
	int gyro_z_sign;
	float gyro_bandwidth;

	int compass_sign;
	int compass_offset;
	int attitude_blend;
	int blend_counter;
//	int angle_to_gyro;

	float gyro_x_center;
	float gyro_y_center;
	float gyro_z_center;
	float prev_gyro_x_center;
	float prev_gyro_y_center;
	float prev_gyro_z_center;
	float gyro_x_accum;
	float gyro_y_accum;
	float gyro_z_accum;
	int gyro_center_count;
	int gyro_x_min;
	int gyro_y_min;
	int gyro_z_min;
	int gyro_x_max;
	int gyro_y_max;
	int gyro_z_max;

	float abs_roll;
	float abs_pitch;
	float abs_heading;

// final output
	float current_roll;
	float current_pitch;
	float current_heading;
	int got_ahrs;
	
	int total_accel;
	int total_mag;
	int total_gyro;
	int total_temp;

	int gyro_count;
	
	i2c_t i2c;
} imu_t;

void init_imu(imu_t *imu);

#define HANDLE_IMU(imu) \
{ \
	handle_hardi2c(&imu.i2c); \
	if(hardi2c_ready(&imu.i2c)) \
	{ \
		imu.current_function(&imu); \
	} \
}


#endif






