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
// calibrate the motor sensors by enabling TEST_MOTORS in arm_truck2.h
// capture the output from the serial port.  Send it to 
// cat motor_data | ./motor_table > arm_motors.h
// graph the output to verify the sensors work

#include <stdint.h>
#include "arm_math.h"
#include "uart.h"
#include "arm_motors.h"
#include "arm_truck2.h"


#define HALL0_TABLE_SIZE (sizeof(hall0_table) / sizeof(uint16_t))
#define HALL1_TABLE_SIZE (sizeof(hall1_table) / sizeof(uint16_t))
#define HALL2_TABLE_SIZE (sizeof(hall2_table) / sizeof(uint16_t))
#define HALL3_TABLE_SIZE (sizeof(hall3_table) / sizeof(uint16_t))

extern truck_t truck;


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
            rows = HALL0_TABLE_SIZE;
            columns = HALL1_TABLE_SIZE;
            row_table = hall0_table;
            column_table = hall1_table;
            motor_table = motor0_table;
            break;
        case RIGHT_MOTOR:
            rows = HALL2_TABLE_SIZE;
            columns = HALL3_TABLE_SIZE;
            row_table = hall2_table;
            column_table = hall3_table;
            motor_table = motor1_table;
            break;
    }
    
    row = get_hall_index(hall0, row_table, rows);
    column = get_hall_index(hall1, column_table, columns);

    int angle = motor_table[row * columns + column];
    return angle * ANGLE_STEP / 7;
}





