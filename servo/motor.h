/*
 * STEPPER CONTROL FOR BRUSHLESS SERVO
 * Copyright (C) 2022 Adam Williams <broadcast at earthling dot net>
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

#define GROUND_MOTOR 0xfe
#define COAST_MOTOR 0xff

#define SYSTEMCLOCK       24500000         // SYSCLK frequency in Hz
#define MAX_PWM (SYSTEMCLOCK / 24000)
#define PCA_RELOAD (-MAX_PWM)
// must be divisable by 3
// update arm_cam.c if you change this
#define SIN_TOTAL 252

