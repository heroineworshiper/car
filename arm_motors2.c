/*
 * STM32 Controller for direct drive truck
 * Copyright (C) 2012-2021 Adam Williams <broadcast at earthling dot net>
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
#include "arm_motors2.h"
#include "arm_truck2.h"

extern truck_t truck;
#define COLUMNS 4
#define TABLE_SIZE (sizeof(motor_data) / sizeof(uint16_t) / COLUMNS)

int get_motor_angle(int motor, int hall)
{
    int column0 = 0;
    int column1 = 1;
    int hall0 = truck.halls[hall + 0].value;
    int hall1 = truck.halls[hall + 1].value;

    if(motor == RIGHT_MOTOR)
    {
        column0 = 2;
        column1 = 3;
    }
    
    int i;
    int distance = -1;
    int result = -1;
    for(i = 0; i < TABLE_SIZE; i++)
    {
        int hall0_diff = motor_data[i * COLUMNS + column0] - hall0;
        int hall1_diff = motor_data[i * COLUMNS + column1] - hall1;
        int distance2 = sqrt(hall0_diff * hall0_diff + hall1_diff * hall1_diff);
        if(distance2 < distance || result < 0)
        {
            distance = distance2;
            result = i;
        }
    }
    
    return result * 40 / 7;
}






