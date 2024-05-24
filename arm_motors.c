/*
 * STM32 Controller for direct drive truck
 * Copyright (C) 2012-2020 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// motor control
// motors are now calibrated at runtime & stored in the FS

// the old method:
// calibrate the motor sensors by enabling TEST_MOTORS in arm_truck2.h
// capture the output from the serial port.  Send it to 
// cat motor_data | ./motor_table > arm_motors.h
// make clean;make truck2.bin;./uart_programmer truck2.bin
// graph the output to verify the sensors work

#include <stdint.h>
#include "arm_fs.h"
#include "arm_math2.h"
#include "uart.h"
//#include "arm_motors.h"
#include "arm_truck2.h"
#include "linux.h"


//#define HALL0_TABLE_SIZE (sizeof(hall0_table) / sizeof(uint16_t))
//#define HALL1_TABLE_SIZE (sizeof(hall1_table) / sizeof(uint16_t))
//#define HALL2_TABLE_SIZE (sizeof(hall2_table) / sizeof(uint16_t))
//#define HALL3_TABLE_SIZE (sizeof(hall3_table) / sizeof(uint16_t))

extern truck_t truck;

// memory resident copies for field calibration
#define MAX_TABLE_SIZE 64
uint16_t hall0_table_ptr[MAX_TABLE_SIZE];
uint16_t hall1_table_ptr[MAX_TABLE_SIZE];
uint16_t hall2_table_ptr[MAX_TABLE_SIZE];
uint16_t hall3_table_ptr[MAX_TABLE_SIZE];
uint8_t motor0_table_ptr[MAX_TABLE_SIZE * MAX_TABLE_SIZE];
uint8_t motor1_table_ptr[MAX_TABLE_SIZE * MAX_TABLE_SIZE];
int hall0_size = 0;
int hall1_size = 0;
int hall2_size = 0;
int hall3_size = 0;
// magic number for motor tables
#define MOTORS_MAGIC 0x6f6648ea

int get_hall_index(int value, const uint16_t *table, int table_size)
{
    int i;
    int distance = -1;
    int result = -1;
    for(i = 0; i < table_size; i++)
    {
        int new_distance = ABS(value - table[i]);
        if(distance < 0 || new_distance < distance)
        {
            distance = new_distance;
            result = i;
        }
        else
        {
            break;
        }
    }
    
    return result;
}


// get angle of motors in degrees
int get_motor_angle(int motor, int hall)
{
    int row, column;
    int rows, columns;
    const uint16_t *row_table;
    const uint16_t *column_table;
    const uint8_t *motor_table;

    int hall0 = truck.halls[hall + 0].value;
    int hall1 = truck.halls[hall + 1].value;

    switch(motor)
    {
        case LEFT_MOTOR:
            rows = hall0_size;
            columns = hall1_size;
            row_table = hall0_table_ptr;
            column_table = hall1_table_ptr;
            motor_table = motor0_table_ptr;
            break;
        case RIGHT_MOTOR:
            rows = hall2_size;
            columns = hall3_size;
            row_table = hall2_table_ptr;
            column_table = hall3_table_ptr;
            motor_table = motor1_table_ptr;
            break;
    }
    
    row = get_hall_index(hall0, row_table, rows);
    column = get_hall_index(hall1, column_table, columns);

    int angle = motor_table[row * columns + column];
    return angle * ANGLE_STEP / 7;
}

void init_motor_tables()
{
// copy flash tables to RAM during startup
    uint32_t address = next_address(MOTORS_MAGIC);
    flush_uart();
	if(address != 0xffffffff)
    {
// skip header
        address += 8;

        const uint8_t *buffer = (unsigned char*)address;
		int offset = 0;
        hall0_size = *(int*)(buffer + offset);
        offset += 4;
        hall1_size = *(int*)(buffer + offset);
        offset += 4;
        hall2_size = *(int*)(buffer + offset);
        offset += 4;
        hall3_size = *(int*)(buffer + offset);
        offset += 4;
        
        TRACE2
        print_text("hall0_size=");
        print_number(hall0_size);
        print_text("hall1_size=");
        print_number(hall1_size);
        print_text("hall2_size=");
        print_number(hall2_size);
        print_text("hall3_size=");
        print_number(hall3_size);
        print_lf();
        flush_uart();

        if(hall0_size <= 0 || hall0_size > MAX_TABLE_SIZE)
        {
            TRACE2
            print_text("Invalid hall0_size\n");
            return;
        }

        if(hall1_size <= 0 || hall1_size > MAX_TABLE_SIZE)
        {
            TRACE2
            print_text("Invalid hall1_size\n");
            return;
        }

        if(hall2_size <= 0 || hall2_size > MAX_TABLE_SIZE)
        {
            TRACE2
            print_text("Invalid hall2_size\n");
            return;
        }

        if(hall3_size <= 0 || hall3_size > MAX_TABLE_SIZE)
        {
            TRACE2
            print_text("Invalid hall3_size\n");
            return;
        }

        memcpy(hall0_table_ptr, buffer + offset, sizeof(uint16_t) * hall0_size);
        offset += sizeof(uint16_t) * hall0_size;
        memcpy(hall1_table_ptr, buffer + offset, sizeof(uint16_t) * hall1_size);
        offset += sizeof(uint16_t) * hall1_size;
        memcpy(hall2_table_ptr, buffer + offset, sizeof(uint16_t) * hall2_size);
        offset += sizeof(uint16_t) * hall2_size;
        memcpy(hall3_table_ptr, buffer + offset, sizeof(uint16_t) * hall3_size);
        offset += sizeof(uint16_t) * hall3_size;

        memcpy(motor0_table_ptr, buffer + offset, hall0_size * hall1_size);
        offset += hall0_size * hall1_size;
        memcpy(motor1_table_ptr, buffer + offset, hall2_size * hall3_size);

        int i;
        TRACE2
        print_lf();
        for(i = 0; i < MAX_TABLE_SIZE; i++)
        {
            print_text("MOTORS: ");
            print_number(i * ANGLE_STEP);
            if(i < hall0_size) 
                print_number(hall0_table_ptr[i]);
            else
                print_number(-1);
            if(i < hall1_size) 
                print_number(hall1_table_ptr[i]);
            else
                print_number(-1);
            if(i < hall2_size) 
                print_number(hall2_table_ptr[i]);
            else
                print_number(-1);
            if(i < hall3_size) 
                print_number(hall3_table_ptr[i]);
            else
                print_number(-1);

            print_lf();
            flush_uart();
        }

	}
    else
    {
        TRACE2
        print_text("motor table not found\n");
    }
}




// memory resident table creation
// copied from motor_table.c
uint16_t motor_lines[LINES][4];


// copy unique values in ascending order
int sort_sensors(uint16_t *output, int column)
{
    int i, j;
    int last_value = -1;
    int total = 0;


    for(i = 0; i < LINES; i++)
    {
        int got_it = 0;
        int lowest = 65536;
        for(j = 0; j < LINES; j++)
        {
            int value = motor_lines[j][column];
            if(value > last_value &&
                value < lowest)
            {
                lowest = value;
                output[i] = value;
                got_it = 1;
            }
        }
        
        if(!got_it)
        {
            break;
        }
        else
        {
            last_value = output[i];
            total++;
        }
    }
    
    return total;
}



// translate hall0 & hall1 values to rows & columns
void tabulate_sensors(uint8_t *motor_table_ptr,
    uint16_t *hall0_table, 
    uint16_t *hall1_table, 
    int *size0, 
    int *size1, 
    int hall0, 
    int hall1)
{
    TRACE2
    flush_uart();

    *size0 = sort_sensors(hall0_table, hall0);
    *size1 = sort_sensors(hall1_table, hall1);
    int i, j, k;
    int dst = 0;


// rows
    for(i = 0; i < *size0; i++)
    {
        int want_hall0 = hall0_table[i];
// columns
        for(j = 0; j < *size1; j++)
        {
            int want_hall1 = hall1_table[j];
// search for nearest sensor values
            int angle = -1;
            int nearest_distance = -1;
            for(k = 0; k < LINES; k++)
            {
                int distance = hypot(motor_lines[k][hall0] - want_hall0,
                    motor_lines[k][hall1] - want_hall1);
                if(distance < nearest_distance ||
                    nearest_distance < 0)
                {
                    angle = k * ANGLE_STEP;
                    nearest_distance = distance;
                }
                PET_WATCHDOG
            }
// print_text("dst=");
// print_number(dst);
// TRACE
            motor_table_ptr[dst++] = angle / 40;
//TRACE
        }
    }
    TRACE2
    flush_uart();
}

// create memory resident motor tables
void do_motor_table()
{
    tabulate_sensors(motor0_table_ptr, 
        hall0_table_ptr, 
        hall1_table_ptr, 
        &hall0_size, 
        &hall1_size, 
        0, 
        1);
    tabulate_sensors(motor1_table_ptr,
        hall2_table_ptr, 
        hall3_table_ptr, 
        &hall2_size, 
        &hall3_size, 
        2, 
        3);

// write the tables to flash
    int bytes = 2 * (hall0_size + 
            hall1_size + 
            hall2_size + 
            hall3_size) +
        hall0_size * hall1_size +
        hall2_size * hall3_size +
        16;
    TRACE2
    print_text("writing motor tables size=");
    print_number(bytes);
    uint8_t *buffer = kmalloc(bytes, 0);
    int offset = 0;
    *(int*)(buffer + offset) = hall0_size;
    offset += 4;
    *(int*)(buffer + offset) = hall1_size;
    offset += 4;
    *(int*)(buffer + offset) = hall2_size;
    offset += 4;
    *(int*)(buffer + offset) = hall3_size;
    offset += 4;

    memcpy(buffer + offset, hall0_table_ptr, sizeof(uint16_t) * hall0_size);
    offset += sizeof(uint16_t) * hall0_size;
    memcpy(buffer + offset, hall1_table_ptr, sizeof(uint16_t) * hall1_size);
    offset += sizeof(uint16_t) * hall1_size;
    memcpy(buffer + offset, hall2_table_ptr, sizeof(uint16_t) * hall2_size);
    offset += sizeof(uint16_t) * hall2_size;
    memcpy(buffer + offset, hall3_table_ptr, sizeof(uint16_t) * hall3_size);
    offset += sizeof(uint16_t) * hall3_size;

    memcpy(buffer + offset, motor0_table_ptr, hall0_size * hall1_size);
    offset += hall0_size * hall1_size;
    memcpy(buffer + offset, motor1_table_ptr, hall2_size * hall3_size);
    
    int bytes_rounded = bytes + (4 - (bytes % 4));
    save_file(MOTORS_MAGIC, buffer, bytes_rounded);
    kfree(buffer);
}




