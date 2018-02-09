#include "settings.h"
#include "linux.h"
#include "arm_imu.h"
#include "arm_truck.h"
#include <math.h>
#include "arm_math.h"
#include "uart.h"
#include "stm32f4xx_gpio.h"

// MPU6000 + AK8975 driver

#define GYRO_CENTER_TOTAL (NAV_HZ * 5)
#define IMU_ADDRESS (0x69 << 1)
#define MAG_ADDRESS (0xc << 1)

// gyro readings for each accel probe
#define GYRO_RATIO 8
#define BLEND_DOWNSAMPLE (NAV_HZ / 10)
#define CALIBRATE_IMU_DOWNSAMPLE (NAV_HZ / 10)
#define MAG_TIMEOUT_MAX 100

#define DEBUG_PIN GPIO_Pin_4
#define DEBUG_GPIO GPIOB

static int debug_counter = 0;

static void imu_status1(void *ptr);

static void mag_read(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;

	int x_offset = imu->mag_x_axis * 2;
	int y_offset = imu->mag_y_axis * 2;
	int z_offset = imu->mag_z_axis * 2;


	int mag_x = imu->mag_x_sign * (int16_t)((imu->i2c.burst[x_offset + 1] << 8) | (imu->i2c.burst[x_offset]));
	int mag_y = imu->mag_y_sign * (int16_t)((imu->i2c.burst[y_offset + 1] << 8) | (imu->i2c.burst[y_offset]));
	int mag_z = imu->mag_z_sign * (int16_t)((imu->i2c.burst[z_offset + 1] << 8) | (imu->i2c.burst[z_offset]));
	imu->mag_x = (imu->mag_x * (1.0f - imu->mag_bandwidth) + (float)mag_x * imu->mag_bandwidth);
	imu->mag_y = (imu->mag_y * (1.0f - imu->mag_bandwidth) + (float)mag_y * imu->mag_bandwidth);
	imu->mag_z = (imu->mag_z * (1.0f - imu->mag_bandwidth) + (float)mag_z * imu->mag_bandwidth);

	if(0)
	{
		TRACE2
		print_number_nospace(imu->mag_x);
		send_uart("\t", 1);
		print_number_nospace(imu->mag_y);
		send_uart("\t", 1);
		print_number_nospace(imu->mag_z);
	}

// update limits
	if(mag_x > imu->mag_x_max)
	{
		imu->mag_x_max = mag_x;
	}
	if(mag_x < imu->mag_x_min)
	{
		imu->mag_x_min = mag_x;
	}
	if(mag_y > imu->mag_y_max)
	{
		imu->mag_y_max = mag_y;
	}
	if(mag_y < imu->mag_y_min)
	{
		imu->mag_y_min = mag_y;
	}
	if(mag_z > imu->mag_z_max)
	{
		imu->mag_z_max = mag_z;
	}
	if(mag_z < imu->mag_z_min)
	{
		imu->mag_z_min = mag_z;
	}

	imu->total_mag++;

	imu_status1(imu);
}


static void mag_status1(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
// get data
	if((imu->i2c.value & 0x01))
	{
		imu->want_mag = 0;
		
		hardi2c_read_burst(&imu->i2c, MAG_ADDRESS, 0x03, 6);
		imu->current_function = mag_read;
		
		
	}
	else
// give up & go to a movie
	if(imu->mag_timeout >= MAG_TIMEOUT_MAX)
	{
		imu->want_mag = 0;
		imu_status1(imu);
	}
	else
// ping it again
	{
		imu_status1(imu);
	}
}


static void imu_read_gyros(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);


	int offset_x = 8 + imu->gyro_x_axis * 2;
	int offset_y = 8 + imu->gyro_y_axis * 2;
	int offset_z = 8 + imu->gyro_z_axis * 2;
	int gyro_x = imu->gyro_x_sign * (int16_t)((imu->i2c.burst[offset_x] << 8) | (imu->i2c.burst[offset_x + 1]));
	int gyro_y = imu->gyro_y_sign * (int16_t)((imu->i2c.burst[offset_y] << 8) | (imu->i2c.burst[offset_y + 1]));
	int gyro_z = imu->gyro_z_sign * (int16_t)((imu->i2c.burst[offset_z] << 8) | (imu->i2c.burst[offset_z + 1]));
	offset_x = imu->accel_x_axis * 2;
	offset_y = imu->accel_y_axis * 2;
	offset_z = imu->accel_z_axis * 2;
	int accel_x = imu->accel_x_sign * (int16_t)((imu->i2c.burst[offset_x] << 8) | (imu->i2c.burst[offset_x + 1]));
	int accel_y = imu->accel_y_sign * (int16_t)((imu->i2c.burst[offset_y] << 8) | (imu->i2c.burst[offset_y + 1]));
	int accel_z = imu->accel_z_sign * (int16_t)((imu->i2c.burst[offset_z] << 8) | (imu->i2c.burst[offset_z + 1]));


	imu->total_accel++;
	imu->accel_x = imu->accel_x * (1.0f - imu->accel_bandwidth) + (float)accel_x * imu->accel_bandwidth;
	imu->accel_y = imu->accel_y * (1.0f - imu->accel_bandwidth) + (float)accel_y * imu->accel_bandwidth;
	imu->accel_z = imu->accel_z * (1.0f - imu->accel_bandwidth) + (float)accel_z * imu->accel_bandwidth;


	imu->total_gyro++;
	imu->gyro_x = imu->gyro_x * (1.0f - imu->gyro_bandwidth) + (float)gyro_x * imu->gyro_bandwidth;
	imu->gyro_y = imu->gyro_y * (1.0f - imu->gyro_bandwidth) + (float)gyro_y * imu->gyro_bandwidth;
	imu->gyro_z = imu->gyro_z * (1.0f - imu->gyro_bandwidth) + (float)gyro_z * imu->gyro_bandwidth;


	debug_counter++;
//	if(!(debug_counter % 100))
	if(0)
	{
		TRACE2
		print_number_nospace(imu->accel_x);
		send_uart("\t", 1);
		print_number_nospace(imu->accel_y);
		send_uart("\t", 1);
		print_number_nospace(imu->accel_z);
		send_uart("\t", 1);
		print_number_nospace(imu->gyro_x);
		send_uart("\t", 1);
		print_number_nospace(imu->gyro_y);
		send_uart("\t", 1);
		print_number_nospace(imu->gyro_z);
	}


	if(truck.need_gyro_center && !truck.have_gyro_center)
	{
		imu_led_flash();
		
		imu->gyro_x_accum += gyro_x;
		imu->gyro_y_accum += gyro_y;
		imu->gyro_z_accum += gyro_z;
		imu->gyro_x_min = MIN(gyro_x, imu->gyro_x_min);
		imu->gyro_y_min = MIN(gyro_y, imu->gyro_y_min);
		imu->gyro_z_min = MIN(gyro_z, imu->gyro_z_min);
		imu->gyro_x_max = MAX(gyro_x, imu->gyro_x_max);
		imu->gyro_y_max = MAX(gyro_y, imu->gyro_y_max);
		imu->gyro_z_max = MAX(gyro_z, imu->gyro_z_max);
		imu->gyro_center_count++;

		if(ABS(imu->gyro_x_max - imu->gyro_x_min) > truck.gyro_center_max ||
			ABS(imu->gyro_y_max - imu->gyro_y_min) > truck.gyro_center_max ||
			ABS(imu->gyro_z_max - imu->gyro_z_min) > truck.gyro_center_max)
		{
			TRACE2
			print_text("center too big ");
			print_number(ABS(imu->gyro_x_max - imu->gyro_x_min));
			print_number(ABS(imu->gyro_y_max - imu->gyro_y_min));
			print_number(ABS(imu->gyro_z_max - imu->gyro_z_min));

			imu->gyro_center_count = 0;
			imu->gyro_x_accum = 0;
			imu->gyro_y_accum = 0;
			imu->gyro_z_accum = 0;


			imu->gyro_x_min = 65535;
			imu->gyro_y_min = 65535;
			imu->gyro_z_min = 65535;
			imu->gyro_x_max = -65535;
			imu->gyro_y_max = -65535;
			imu->gyro_z_max = -65535;
		}

		if(imu->gyro_center_count >= GYRO_CENTER_TOTAL)
		{
			imu->prev_gyro_x_center = imu->gyro_x_center;
			imu->prev_gyro_y_center = imu->gyro_y_center;
			imu->prev_gyro_z_center = imu->gyro_z_center;
			
 			imu->gyro_x_center = (float)imu->gyro_x_accum / imu->gyro_center_count;
			imu->gyro_y_center = (float)imu->gyro_y_accum / imu->gyro_center_count;
			imu->gyro_z_center = (float)imu->gyro_z_accum / imu->gyro_center_count;

// test if calculation didn't drift
			TRACE2
			print_text("spread=");
			print_float(imu->gyro_x_max - imu->gyro_x_min);
			print_float(imu->gyro_y_max - imu->gyro_y_min);
			print_float(imu->gyro_z_max - imu->gyro_z_min);
			print_text("center=");
			print_float(imu->gyro_x_center);
			print_float(imu->gyro_y_center);
			print_float(imu->gyro_z_center);
			print_text("drift=");
			print_float(imu->prev_gyro_x_center - imu->gyro_x_center);
			print_float(imu->prev_gyro_y_center - imu->gyro_y_center);
			print_float(imu->prev_gyro_z_center - imu->gyro_z_center);

			if(fabs(imu->prev_gyro_x_center) > 0.001 && 
				fabs(imu->prev_gyro_y_center) > 0.001 && 
				fabs(imu->prev_gyro_z_center) > 0.001 && 
				fabs(imu->prev_gyro_x_center - imu->gyro_x_center) < (float)truck.max_gyro_drift &&
				fabs(imu->prev_gyro_y_center - imu->gyro_y_center) < (float)truck.max_gyro_drift &&
				fabs(imu->prev_gyro_z_center - imu->gyro_z_center) < (float)truck.max_gyro_drift)
			{
				TRACE2
				print_text("got center\n");
				truck.have_gyro_center = 1;
				truck.need_gyro_center = 0;
			}
			else
// try again
			{
				imu->gyro_center_count = 0;
				imu->gyro_x_accum = 0;
				imu->gyro_y_accum = 0;
				imu->gyro_z_accum = 0;


				imu->gyro_x_min = 65535;
				imu->gyro_y_min = 65535;
				imu->gyro_z_min = 65535;
				imu->gyro_x_max = -65535;
				imu->gyro_y_max = -65535;
				imu->gyro_z_max = -65535;
			}
		}
	}

// absolute angles
	if(ABS(imu->accel_z) < 1.0f)
	{
		if(imu->accel_x > 0) 
			imu->abs_roll = -M_PI / 2;
		else
			imu->abs_roll = M_PI / 2;
		if(imu->accel_y > 0) 
			imu->abs_pitch = M_PI / 2;
		else
			imu->abs_pitch = -M_PI / 2;
	}
	else
	{
		if(imu->accel_z < 0)
		{

			imu->abs_roll = -M_PI + atan2f(imu->accel_x, -imu->accel_z);
			imu->abs_pitch = -M_PI + atan2f(-imu->accel_y, -imu->accel_z);

		}
		else
		{
			imu->abs_roll = -atan2f(imu->accel_x, imu->accel_z);
			imu->abs_pitch = -atan2f(-imu->accel_y, imu->accel_z);

		}

		imu->abs_roll = fix_angle(imu->abs_roll);
		imu->abs_pitch = fix_angle(imu->abs_pitch);
/*
 * if(!(debug_counter % 100))
 * {
 * 	TRACE2
 * 	print_float(TO_DEG(imu->abs_roll));
 * 	print_float(TO_DEG(imu->abs_pitch));
 * }
 */

	}

// predict angles
	if(!truck.have_gyro_center)
	{
		imu->current_roll = imu->abs_roll;
		imu->current_pitch = imu->abs_pitch;
	}

// calculate heading
	if(truck.enable_mag)
	{
		imu->abs_heading = compass_heading(imu->mag_x,
			imu->mag_y, 
			imu->mag_z,
			imu->mag_x_min,
			imu->mag_y_min,
			imu->mag_z_min,
			imu->mag_x_max,
			imu->mag_y_max,
			imu->mag_z_max,
			imu->current_roll,
			imu->current_pitch - TO_RAD(5),
			imu->compass_sign,
			imu->compass_offset);
		imu->abs_heading = fix_angle(imu->abs_heading);

/*
 * if(!(debug_counter % 100))
 * {
 * TRACE2
 * print_float(TO_DEG(imu->abs_heading));
 * }
 */

	}
	
	if(!truck.have_gyro_center)
	{
		imu->current_heading = imu->abs_heading;
	}

// accumulate angles
	if(truck.have_gyro_center)
	{
		imu->gyro_x_centered = imu->gyro_x - 
			imu->gyro_x_center;
		imu->gyro_y_centered = imu->gyro_y - 
			imu->gyro_y_center;
		imu->gyro_z_centered = imu->gyro_z - 
			imu->gyro_z_center;
	
	
		DISABLE_INTERRUPTS
// accumulate heading for enable_mag = 0
		truck.current_heading += imu->gyro_z_centered / truck.angle_to_gyro / NAV_HZ;
		truck.current_heading = fix_angle(truck.current_heading);

// accmulate for mag 
		imu->current_roll += imu->gyro_x_centered / truck.angle_to_gyro / NAV_HZ;
		imu->current_pitch += imu->gyro_y_centered / truck.angle_to_gyro / NAV_HZ;
		imu->current_heading += imu->gyro_z_centered / truck.angle_to_gyro / NAV_HZ;
		imu->current_roll = fix_angle(imu->current_roll);
		imu->current_pitch = fix_angle(imu->current_pitch);
		imu->current_heading = fix_angle(imu->current_heading);

		imu->blend_counter++;
		if(imu->blend_counter >= BLEND_DOWNSAMPLE)
		{
 			imu->blend_counter = 0;
			float error = get_angle_change(imu->current_roll, 
				imu->abs_roll);
			imu->current_roll += error /
				imu->attitude_blend;

			error = get_angle_change(imu->current_pitch, 
				imu->abs_pitch);
			imu->current_pitch += error /
				imu->attitude_blend;

			if(truck.enable_mag)
			{
				error = get_angle_change(imu->current_heading, 
					imu->abs_heading);
				imu->current_heading += error /
					imu->attitude_blend;
			}
		}
		
		
		ENABLE_INTERRUPTS
	}


//	if(!(debug_counter % 100))
	if(0)
	{
		TRACE2
		print_float(TO_DEG(imu->current_roll));
		print_float(TO_DEG(imu->current_pitch));
		print_float(TO_DEG(imu->current_heading));
	}


// start mag conversion
	if(!(imu->total_gyro % (NAV_HZ / 10)) && truck.enable_mag)
	{
		imu->want_mag = 1;
		imu->mag_timeout = 0;
		hardi2c_write_device(&imu->i2c, MAG_ADDRESS, 0x0a, 0x01);
		imu->current_function = imu_status1;
	}
	else
	if(imu->want_mag)
	{
// start mag status read
		hardi2c_read_device(&imu->i2c, MAG_ADDRESS, 0x02);
		imu->current_function = mag_status1;
	}
	else
// start gyro status read
	{
		imu_status1(imu);
	}

	imu->gyro_count++;
	if(imu->gyro_count >= GYRO_RATIO)
		imu->gyro_count = 0;
}


static void imu_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;

	if(imu->i2c.value & 0x01)
	{
		hardi2c_read_burst(&imu->i2c,
			IMU_ADDRESS, 
			0x3b,
			14);
		imu->current_function = imu_read_gyros;
	}
 	else
	{
		imu_status1(ptr);
	}
}

static void imu_status1(void *ptr)
{
// get gyro status
	imu_t *imu = (imu_t*)ptr;
	hardi2c_read_device(&imu->i2c, IMU_ADDRESS, 0x3a);
// device ID
//	hardi2c_read_device(&imu->i2c, IMU_ADDRESS, 0x75);
	imu->current_function = imu_status2;
}


static void imu_config4(void *ptr)
{
// accel full scale range
// 2g
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1c, 0x0);
	imu->current_function = imu_status1;
}

static void imu_config3(void *ptr)
{
// gyro full scale range
// 500 deg/sec
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1b, 0x08);
	imu->current_function = imu_config4;
	
}

static void imu_config2(void *ptr)
{
// digital low pass filter
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1a, 0x0);
	imu->current_function = imu_config3;
}


static void imu_config1(void *ptr)
{
// sample rate divider
// high enough to keep i2c from dropping samples
	imu_t *imu = (imu_t*)ptr;
//	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x19, 6);
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x19, 18);
	imu->current_function = imu_config2;
	
}

static void imu_config0(void *ptr)
{
// wake up.  Use gyro as clock source.
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x6b, 0x1);
	imu->current_function = imu_config1;
	
}


static void imu_test1(void *ptr);

// Needs a few read requests to warm up, for some reason
static void imu_test2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;


	if(imu->i2c.value == 104)
	{
		imu_config0(ptr);
	}
 	else
	{
		imu_test1(ptr);
	}
}

static void imu_test1(void *ptr)
{
// get gyro status
	imu_t *imu = (imu_t*)ptr;
// device ID
	hardi2c_read_device(&imu->i2c, IMU_ADDRESS, 0x75);
	imu->current_function = imu_test2;
}

void calibrate_imu(imu_t *imu)
{
	TRACE2
	print_text("starting calibration\n");

	DISABLE_INTERRUPTS
	truck.need_gyro_center = 1;
	truck.have_gyro_center = 0;
	imu->gyro_center_count = 0;
	imu->gyro_x_center = 0;
	imu->gyro_y_center = 0;
	imu->gyro_z_center = 0;
	imu->gyro_x_min = 65535;
	imu->gyro_y_min = 65535;
	imu->gyro_z_min = 65535;
	imu->gyro_x_max = -65535;
	imu->gyro_y_max = -65535;
	imu->gyro_z_max = -65535;
	imu->gyro_x_accum = 0;
	imu->gyro_y_accum = 0;
	imu->gyro_z_accum = 0;
	truck.current_heading = 0;
	ENABLE_INTERRUPTS
}

void calibrate_mag(imu_t *imu)
{
	imu->calibrate_mag = 1;
	imu->mag_x_max = -65536;
	imu->mag_y_max = -65536;
	imu->mag_z_max = -65536;
	imu->mag_x_min = 65536;
	imu->mag_y_min = 65536;
	imu->mag_z_min = 65536;
}

void init_imu(imu_t *imu)
{
	init_hardi2c(&imu->i2c, I2C1);
	imu->calibrate_imu = 0;
	imu->dump_theta = 1;


	imu->mag_x_axis = 1;
	imu->mag_y_axis = 2;
	imu->mag_z_axis = 0;
	imu->mag_x_sign = -1;
	imu->mag_y_sign = -1;
	imu->mag_z_sign = 1;
	imu->mag_bandwidth = 0.1f;

	imu->accel_x_axis = 0;
	imu->accel_y_axis = 1;
	imu->accel_z_axis = 2;
	imu->accel_x_sign = -1;
	imu->accel_y_sign = -1;
	imu->accel_z_sign = 1;
	imu->accel_bandwidth = 0.1f;

	imu->gyro_x_axis = 1;
	imu->gyro_y_axis = 0;
	imu->gyro_z_axis = 2;
	imu->gyro_x_sign = -1;
	imu->gyro_y_sign = -1;
	imu->gyro_z_sign = -1;
	imu->gyro_bandwidth = 1.0f;


	imu->compass_sign = 1;
	imu->attitude_blend = 64;
//	imu->angle_to_gyro = 4000;
	imu->gyro_x_min = 65535;
	imu->gyro_y_min = 65535;
	imu->gyro_z_min = 65535;
	imu->gyro_x_max = -65535;
	imu->gyro_y_max = -65535;
	imu->gyro_z_max = -65535;
	imu->current_function = imu_test1;
	debug_counter = 0;
}





