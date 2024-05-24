/*
 * Smart leash
 * Copyright (C) 2022-2024 Adam Williams <broadcast at earthling dot net>
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


// parse serial port & print leash output on the console
// gcc -O2 -o leash_test leash_test.c



#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/serial.h>
#include <math.h>

// byte stuffing for UART
#define START_CODE 0xff
#define ESC_CODE 0xfe
#define BUFFER_SIZE 1024
#define TEXT_SIZE 5
uint8_t buffer[BUFFER_SIZE];
uint8_t text[BUFFER_SIZE];
int got_esc = 0;
int text_offset = 0;

static int init_serial(char *path, int baud_code)
{
	struct termios term;

	printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

 
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, baud_code);
	cfsetospeed(&term, baud_code);
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}

#define TO_RAD(x) (((float)(x)) * 2 * M_PI / 360)
#define TO_DEG(x) ((x) * 360 / 2 / M_PI)

int min_angle_adc = 190;
int max_angle_adc = 65;
int center_angle_adc = 128;
float min_angle = TO_RAD(-70);
float max_angle = TO_RAD(70);
void handle_text(uint8_t c)
{
    if(text_offset < TEXT_SIZE)
    {
        text[text_offset++] = c;
        if(text_offset >= TEXT_SIZE)
        {
            int length = (int16_t)(text[0] | (text[1] << 8));
            int angle_adc = text[2];
            int encoder0 = text[3];
            int encoder1 = text[4];
            float angle;
            if(angle_adc >= center_angle_adc)
                angle = (float)(angle_adc - center_angle_adc) * 
                    (0 - min_angle) /
                    (center_angle_adc - min_angle_adc);
            else
                angle = (float)(center_angle_adc - angle_adc) * 
                    (max_angle - 0) /
                    (center_angle_adc - max_angle_adc);
            
            printf("%d %d %d %d %f\n", 
                length, 
                angle_adc, 
                encoder0, 
                encoder1, 
                TO_DEG(angle));
        }
    }
}

int main(int argc, char *argv[])
{
	int baud_code = B115200;
    int fd = init_serial("/dev/ttyUSB0", baud_code);

    while(1)
    {
        int bytes_read = read(fd, buffer, BUFFER_SIZE);
        int i;
        for(i = 0; i < bytes_read; i++)
        {
            if(!got_esc)
            {
                if(buffer[i] == START_CODE)
                {
                    text_offset = 0;
                }
                else
                if(buffer[i] == ESC_CODE)
                {
                    got_esc = 1;
                }
                else
                {
                    handle_text(buffer[i]);
                }
            }
            else
            {
                got_esc = 0;
                handle_text(buffer[i] ^ 0xff);
            }
        }
    }
}


















