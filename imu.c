#include "settings.h"
#include "linux.h"
#include "imu.h"
#include <math.h>
#include "arm_math.h"
#include "uart.h"
#include "stm32f4xx_gpio.h"


#define GYRO_CENTER_TOTAL 4096
#define GYRO_ADDRESS (0x69 << 1)
#define ACCEL_ADDRESS (0xf << 1)

// gyro readings for each accel probe
#define GYRO_RATIO 8
#define BLEND_DOWNSAMPLE (NAV_HZ / 10)
#define CALIBRATE_IMU_DOWNSAMPLE (NAV_HZ / 10)

static int debug_counter = 0;
imu_t imu;

static void imu_gyro_status1(void *ptr);

static void imu_send_results(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
// DEBUG
//	TOGGLE_PIN(GPIOB, GPIO_Pin_4);

	imu->current_function = imu_gyro_status1;

	if(imu->got_accel)
	{
		imu->got_accel = 0;
		imu->total_accel++;

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
		}
	}

	if(imu->need_gyro_center &&
		!imu->have_gyro_center && 
		imu->gyro_center_count >= GYRO_CENTER_TOTAL)
	{

 		imu->gyro_x_center = imu->gyro_x_accum / imu->gyro_center_count;
		imu->gyro_y_center = imu->gyro_y_accum / imu->gyro_center_count;
		imu->gyro_z_center = imu->gyro_z_accum / imu->gyro_center_count;



TRACE2
print_text("magnitudes: ");
print_float(ABS(imu->gyro_x_max - imu->gyro_x_min));
print_float(ABS(imu->gyro_y_max - imu->gyro_y_min));
print_float(ABS(imu->gyro_z_max - imu->gyro_z_min));
print_text("centers: ");
print_float(imu->gyro_x_center);
print_float(imu->gyro_y_center);
print_float(imu->gyro_z_center);


		imu->have_gyro_center = 1;
		imu->need_gyro_center = 0;

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

	if(imu->have_gyro_center)
	{
		imu->gyro_x_centered = imu->gyro_x - 
			imu->gyro_x_center;
		imu->gyro_y_centered = imu->gyro_y - 
			imu->gyro_y_center;
		imu->gyro_z_centered = imu->gyro_z - 
			imu->gyro_z_center;

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

//			error = get_angle_change_fixed(imu->gyro_heading / imu->angle_to_gyro, 
//				imu->abs_heading);
//			imu->gyro_heading += error * imu->angle_to_gyro / imu->attitude_blend;
		}
	

		imu->current_roll += imu->gyro_x_centered / imu->angle_to_gyro / NAV_HZ;
		imu->current_pitch += imu->gyro_y_centered / imu->angle_to_gyro / NAV_HZ;
		imu->current_heading += imu->gyro_z_centered / imu->angle_to_gyro / NAV_HZ;
		imu->current_roll = fix_angle(imu->current_roll);
		imu->current_pitch = fix_angle(imu->current_pitch);
		imu->current_heading = fix_angle(imu->current_heading);

		imu->got_ahrs = 1;
	}
	else
	{
// predict gyro accumulation
		imu->current_roll = imu->abs_roll;
		imu->current_pitch = imu->abs_pitch;
		imu->current_heading = 0;
	}

	
	if(imu->calibrate_imu)
	{
		imu->debug_counter++;
		if(imu->debug_counter >= CALIBRATE_IMU_DOWNSAMPLE)
		{
			imu->debug_counter = 0;
			TRACE2
//			print_buffer(imu->packet, 14);
//			print_number_nospace(imu->temp_center);
//			send_uart("\t", 1);
//			print_number_nospace(imu->temp);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_x);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_y);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_z);
//			send_uart("\t", 1);
//			print_number_nospace(imu->accel_x);
//			send_uart("\t", 1);
//			print_number_nospace(imu->accel_y);
//			send_uart("\t", 1);
//			print_number_nospace(imu->accel_z);
//			send_uart("\t", 1);
			print_number_nospace(imu->gyro_x);
			send_uart("\t", 1);
			print_number_nospace(imu->gyro_y);
			send_uart("\t", 1);
			print_number(imu->gyro_z);
		}
	}

	if(imu->dump_theta)
	{
		imu->debug_counter++;
		if(imu->debug_counter >= CALIBRATE_IMU_DOWNSAMPLE)
		{
			imu->debug_counter = 0;
//			TRACE2
//			print_float(TO_DEG(imu->current_roll));
//			send_uart("\t", 1);
//			print_float(TO_DEG(imu->current_pitch));
//			send_uart("\t", 1);
//			print_float(TO_DEG(imu->abs_roll));
//			send_uart("\t", 1);
//			print_float(TO_DEG(imu->abs_pitch));
//			send_uart("\t", 1);
//			print_float(TO_DEG(imu->current_heading));
		}
	}
}



void imu_update_accel(imu_t *imu, int accel_x, int accel_y, int accel_z)
{
	imu->accel_x = imu->accel_x * (1.0f - imu->accel_bandwidth) + accel_x * imu->accel_bandwidth;
	imu->accel_y = imu->accel_y * (1.0f - imu->accel_bandwidth) + accel_y * imu->accel_bandwidth;
	imu->accel_z = imu->accel_z * (1.0f - imu->accel_bandwidth) + accel_z * imu->accel_bandwidth;
	imu->got_accel = 1;
}

void imu_update_gyro(imu_t *imu, int gyro_x, int gyro_y, int gyro_z)
{
	if(gyro_x >= 32767) gyro_x = 0;
	if(gyro_y >= 32767) gyro_y = 0;
	if(gyro_z >= 32767) gyro_z = 0;

	imu->gyro_x = imu->gyro_x * (1.0f - imu->gyro_bandwidth) + gyro_x * imu->gyro_bandwidth;
	imu->gyro_y = imu->gyro_y * (1.0f - imu->gyro_bandwidth) + gyro_y * imu->gyro_bandwidth;
	imu->gyro_z = imu->gyro_z * (1.0f - imu->gyro_bandwidth) + gyro_z * imu->gyro_bandwidth;

	imu->total_gyro++;
	if(imu->need_gyro_center && !imu->have_gyro_center)
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
		
		if(ABS(imu->gyro_x_max - imu->gyro_x_min) > imu->gyro_center_max ||
			ABS(imu->gyro_y_max - imu->gyro_y_min) > imu->gyro_center_max ||
			ABS(imu->gyro_z_max - imu->gyro_z_min) > imu->gyro_center_max)
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


static void send_results(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
// 1100 Hz
// DEBUG
//	TOGGLE_PIN(GPIOB, GPIO_Pin_4);

	imu_send_results(imu);
	imu->current_function = imu_gyro_status1;


/*
 * debug_counter++;
 * if(!(debug_counter % DEBUG_DOWNSAMPLE))
 * {
 * TRACE2
 * print_fixed_nospace(imu->current_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->current_pitch);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_pitch);
 * }
 */

}



static void imu_read_accel(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int offset_x = imu->accel_x_axis * 2;
	int offset_y = imu->accel_y_axis * 2;
	int offset_z = imu->accel_z_axis * 2;
	int accel_x = imu->accel_x_sign * (int16_t)((imu->i2c.burst[offset_x + 1] << 8) | (imu->i2c.burst[offset_x]));
	int accel_y = imu->accel_y_sign * (int16_t)((imu->i2c.burst[offset_y + 1] << 8) | (imu->i2c.burst[offset_y]));
	int accel_z = imu->accel_z_sign * (int16_t)((imu->i2c.burst[offset_z + 1] << 8) | (imu->i2c.burst[offset_z]));

	imu_update_accel(imu, accel_x, accel_y, accel_z);
	
	send_results(ptr);
}

static void imu_accel_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->i2c.value & 0x10)
	{
		hardi2c_read_burst(&imu->i2c,
			ACCEL_ADDRESS, 
			0x6,
			6);
		imu->current_function = imu_read_accel;

	}
 	else
	{
		send_results(ptr);
	}
}


static void imu_accel_status1(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int offset_x = imu->gyro_x_axis * 2;
	int offset_y = imu->gyro_y_axis * 2;
	int offset_z = imu->gyro_z_axis * 2;
	int gyro_x = imu->gyro_x_sign * (int16_t)((imu->i2c.burst[offset_x] << 8) | (imu->i2c.burst[offset_x + 1]));
	int gyro_y = imu->gyro_y_sign * (int16_t)((imu->i2c.burst[offset_y] << 8) | (imu->i2c.burst[offset_y + 1]));
	int gyro_z = imu->gyro_z_sign * (int16_t)((imu->i2c.burst[offset_z] << 8) | (imu->i2c.burst[offset_z + 1]));

	imu_update_gyro(imu, gyro_x, gyro_y, gyro_z);


// accel status
	if(imu->gyro_count == 0)
	{
		hardi2c_read_device(&imu->i2c, ACCEL_ADDRESS, 0x18);
		imu->current_function = imu_accel_status2;
	}
	else
	{
		send_results(ptr);
	}

	imu->gyro_count++;
	if(imu->gyro_count >= GYRO_RATIO)
		imu->gyro_count = 0;
}



static void imu_gyro_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->i2c.value & 0x01)
	{
		hardi2c_read_burst(&imu->i2c,
			GYRO_ADDRESS, 
			0x1d,
			6);
		imu->current_function = imu_accel_status1;
	}
 	else
	{
		imu_gyro_status1(ptr);
	}
}

static void imu_gyro_status1(void *ptr)
{
// gyro status
	imu_t *imu = (imu_t*)ptr;
	hardi2c_read_device(&imu->i2c, GYRO_ADDRESS, 0x1a);
	imu->current_function = imu_gyro_status2;
}

static void imu_config4(void *ptr)
{
// accel CTRL_REG1
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x1b, 0xe0);
	imu->current_function = imu_gyro_status1;

}

static void imu_config3b(void *ptr)
{
// DATA_CTRL_REG 
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x21, 0x03);
	imu->current_function = imu_config4;
}

static void imu_config3(void *ptr)
{
// accel CTRL_REG1
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x1b, 0x00);
	imu->current_function = imu_config3b;
	
}

static void imu_config2(void *ptr)
{
// DLPF
// full scale is fixed on the ITG3200
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, GYRO_ADDRESS, 0x16, 0x18);
	imu->current_function = imu_config3;
	
}


static void imu_config1(void *ptr)
{
// sample rate divider
// high enough to keep i2c from dropping samples
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, GYRO_ADDRESS, 0x15, 0x6);
	imu->current_function = imu_config2;
	
}

void calibrate_imu(imu_t *imu)
{
	imu->need_gyro_center = 1;
	imu->gyro_center_count = 0;
	imu->have_gyro_center = 0;
	imu->gyro_x_min = 65535;
	imu->gyro_y_min = 65535;
	imu->gyro_z_min = 65535;
	imu->gyro_x_max = -65535;
	imu->gyro_y_max = -65535;
	imu->gyro_z_max = -65535;
	imu->gyro_x_accum = 0;
	imu->gyro_y_accum = 0;
	imu->gyro_z_accum = 0;
}

void init_imu(imu_t *imu)
{
	bzero(imu, sizeof(imu_t));
	init_hardi2c(&imu->i2c, I2C3);
	imu->current_function = imu_config1;
	imu->calibrate_imu = 0;
	imu->dump_theta = 1;


	imu->accel_x_axis = 0;
	imu->accel_y_axis = 2;
	imu->accel_z_axis = 1;
	imu->accel_x_sign = -1;
	imu->accel_y_sign = -1;
	imu->accel_z_sign = -1;
	imu->accel_bandwidth = 0.1f;
		
	imu->gyro_x_axis = 0;
	imu->gyro_y_axis = 1;
	imu->gyro_z_axis = 2;
	imu->gyro_x_sign = 1;
	imu->gyro_y_sign = 1;
	imu->gyro_z_sign = -1;
	imu->gyro_bandwidth = 1.0f;
	
	
	imu->compass_sign = 1;
	imu->attitude_blend = 64;
	imu->angle_to_gyro = 700;
	imu->gyro_center_max = 100;
	imu->gyro_x_min = 65535;
	imu->gyro_y_min = 65535;
	imu->gyro_z_min = 65535;
	imu->gyro_x_max = -65535;
	imu->gyro_y_max = -65535;
	imu->gyro_z_max = -65535;
	imu->current_function = imu_config1;
	debug_counter = 0;
}





