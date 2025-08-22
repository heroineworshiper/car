/*
 * UWB tester for 2 board phase delay failure
 * Copyright (C) 2017-2025  Adam Williams <broadcast at earthling dot net>
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



// gcc -O2 -o uwb_phase uwb_phase.c



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
#include <signal.h>
#ifndef __clang__
#include <linux/serial.h>
#endif 
#include <math.h>

#define LOG_FILE "uwb.cap"
#define TXTLEN 1024
#define FILTER 0.01

//#define ENABLE_PDOA
//#define ENABLE_DS

int local_echo = 1;
int send_cr = 1;
int serial_fd = -1;
FILE *log_fd = 0;

typedef struct
{
    int32_t round;
    int32_t reply_delay;
    int32_t clock_offset;
} ranging_t;

void quit()
{
    printf("quit %d: Giving up & going to a movie\n", __LINE__);
    fclose(log_fd);
    exit(0);
}

// Copy of pic chksum routine
static uint16_t get_chksum(unsigned char *buffer, int size)
{
	int i;
	uint16_t result = 0;

	size /= 2;
	for(i = 0; i < size; i++)
	{
		uint16_t prev_result = result;
// Not sure if word aligned
		uint16_t value = (buffer[0]) | (buffer[1] << 8);
		result += value;
// Carry bit
		if(result < prev_result) result++;
		buffer += 2;
	}

	uint16_t result2 = (result & 0xff) << 8;
	result2 |= (result & 0xff00) >> 8;
	return result2;
}

// Returns the FD of the serial port
static int init_serial(char *path, int baud, int custom_baud)
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


#ifndef __clang__
// Try to set kernel to custom baud and low latency
	if(custom_baud)
	{
		struct serial_struct serial_struct;
		if(ioctl(fd, TIOCGSERIAL, &serial_struct) < 0)
		{
			printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		}

		serial_struct.flags |= ASYNC_LOW_LATENCY;
		serial_struct.flags &= ~ASYNC_SPD_CUST;
		if(custom_baud)
		{
			serial_struct.flags |= ASYNC_SPD_CUST;
			serial_struct.custom_divisor = (int)((float)serial_struct.baud_base / 
				(float)custom_baud + 0.5);
			baud = B38400;
		}  
/*
 * printf("init_serial: %d serial_struct.baud_base=%d serial_struct.custom_divisor=%d\n", 
 * __LINE__,
 * serial_struct.baud_base,
 * serial_struct.custom_divisor);
 */


// Do setserial on the command line to ensure it actually worked.
		if(ioctl(fd, TIOCSSERIAL, &serial_struct) < 0)
		{
			printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		}
	}
#endif // !__clang__
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
 
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, baud);
	cfsetospeed(&term, baud);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
//    term.c_cflag |= CRTSCTS; // flow control
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;

// printf("init_serial: %d path=%s iflag=0x%08x lflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
// __LINE__, 
// path, 
// term.c_iflag, 
// term.c_lflag, 
// term.c_oflag, 
// term.c_cflag);

	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}

// Read a character
unsigned char read_char(int fd)
{
	unsigned char c;
	int result;

//printf("read_char %d\n", __LINE__);
	do
	{
		result = read(fd, &c, 1);
//printf("read_char %d %d %d\n", __LINE__, result, c);

		if(result <= 0)
		{
			printf("Unplugged\n");
			quit();
		}

	} while(result <= 0);
	return c;
}

// Send a character
void write_char(int fd, unsigned char c)
{
	int result;
	do
	{
		result = write(fd, &c, 1);
	} while(!result);
}


// trap SIGINT
static void sig_catch(int sig)
{
//    printf("sig_catch %d: sig=%d\n", __LINE__, sig);
}

static void help()
{
    printf("Usage: uwb_test [tty path]\n");
    exit(0);
}

void write_log(char *string)
{
	if(log_fd)
	{
		int result = fwrite(string, 1, strlen(string), log_fd);
        if(result < strlen(string))
            printf("Write to log file failed %s\n", strerror(errno));
		fflush(log_fd);
	}
}

int main(int argc, char *argv[])
{
	int baud = 115200;
	int baud_enum;
	int custom_baud = 0;
	char *path = 0;
	int i;

	if(argc > 1)
	{
		for(i = 1; i < argc; i++)
		{
            if(!strcmp(argv[i], "-h"))
            {
                help();
            }
            else
			{
				path = argv[i];
			}
		}
	}

	baud_enum = B115200;

    signal(SIGINT, sig_catch);
    signal(SIGTSTP, sig_catch);

	log_fd = fopen(LOG_FILE, "w");
	if(!log_fd)
	{
		printf("Couldn't open %s\n", LOG_FILE);
	}

	serial_fd = -1;
	if(path) serial_fd = init_serial(path, baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB0", baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB1", baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB2", baud_enum, custom_baud);

	int test_state = 0;
    ranging_t buffer;
    int offset = 0;
    int range1 = 0;
    int range2 = 0;
    int range1_accum = 0;
    int range2_accum = 0;
    int counter1 = 0;
    int counter2 = 0;
    double clock_offset1 = 0;
    int clock_offset_init1 = 100;
    double clock_offset2 = 0;
    int clock_offset_init2 = 100;

	struct timeval start_time;
	struct timeval current_time;
	gettimeofday(&start_time, 0);


	fd_set rfds;

	while(1)
	{
		FD_ZERO(&rfds);
		FD_SET(serial_fd, &rfds);
		FD_SET(0, &rfds);
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		int result = select(serial_fd + 1, 
			&rfds, 
			0, 
			0, 
			&timeout);
//printf("main %d serial_fd=%d result=%d\n", __LINE__, serial_fd, result);

        if(result < 0) 
        {
            quit();
        }

// data from serial port
		if(FD_ISSET(serial_fd, &rfds))
		{
			unsigned char c = read_char(serial_fd);
            int print_it = 0;

            switch(test_state)
            {
                case 0:
                    if(c == 0xff)
                        test_state++;
                    else
                        print_it = 1;
                    break;
                case 1:
                    if(c == 0xd2)
                        test_state = 2;
                    else
                    if(c == 0xd3)
                        test_state = 3;
                    else
                    if(c == 0xff)
                        continue;
                    else
                        print_it = 1;
                    break;
// BASE board
                case 2:
                {
                    uint8_t *ptr = (uint8_t*)&buffer;
                    ptr[offset++] = c;
                    if(offset >= 12)
                    {
                        offset = 0;
                        test_state = 0;
// clock_offset is the biggest source of noise so must be filtered more than range
                        if(clock_offset_init1 > 0)
                        {
                            clock_offset_init1--;
                            clock_offset1 = buffer.clock_offset;
                        }
                        else
                            clock_offset1 = clock_offset1 * (1.0 - FILTER) +
                                (double)buffer.clock_offset * FILTER;

                        char string[TXTLEN];
                        range1_accum += buffer.round - 
                            buffer.reply_delay +
                            (int)(clock_offset1 * 3700 / 17000);
                        counter1++;
                        if(counter1 >= 10)
                        {
                            range1_accum /= counter1;
                            range1 = range1_accum;
//                             sprintf(string, 
//                                 "RANGE1=%d CLOCK_OFFSET1=%.0f\n", 
//                                 range1,
//                                 clock_offset1);
//                             printf("%s", string);
//  			                write_log(string);
                            counter1 = 0;
                            range1_accum = 0;
                        }
                    }
                    break;
                }
                
// BASE2 board
                case 3:
                {
                    uint8_t *ptr = (uint8_t*)&buffer;
                    ptr[offset++] = c;
                    if(offset >= 12)
                    {
                        offset = 0;
                        test_state = 0;
// clock_offset is the biggest source of noise so must be filtered more than range
                        if(clock_offset_init2 > 0)
                        {
                            clock_offset_init2--;
                            clock_offset2 = buffer.clock_offset;
                        }
                        else
                            clock_offset2 = clock_offset2 * (1.0 - FILTER) +
                                (double)buffer.clock_offset * FILTER;

                        char string[TXTLEN];
                        range2_accum += buffer.round - 
                            buffer.reply_delay +
                            (int)(clock_offset2 * 3700 / 17000);
                        counter2++;
                        if(counter2 >= 10)
                        {
                            range2_accum /= counter2;
                            range2 = range2_accum;
                            sprintf(string, 
                                "DIFF=%d CLOCK_OFFSET1=%.0f CLOCK_OFFSET2=%.0f\n", 
                                range1 - range2,
                                clock_offset1,
                                clock_offset2);
                            printf("%s", string);
 			                write_log(string);
                            counter2 = 0;
                            range2_accum = 0;
                        }
                    }
                    break;
                }
            }

			if(print_it)
			{
				printf("%c", c);
				fflush(stdout);

			    if(log_fd)
			    {
				    fputc(c, log_fd);
				    fflush(log_fd);
			    }
			}

		}

// data from console
		if(FD_ISSET(0, &rfds))
		{
			int i;
            uint8_t test_buffer[8];
			int bytes = read(0, test_buffer, sizeof(test_buffer));

			for(i = 0; i < bytes; i++)
			{
				char c = test_buffer[i];
//printf("main %d: i=%d c=%d\n", __LINE__, i, c);


				if(local_echo)
				{
					if(c < 0xa)
						printf("0x%02x ", c);
					else
						printf("%c", c);

					fflush(stdout);
				}

				if(c == 0xa)
				{
					if(send_cr)
					{
						write_char(serial_fd, 0xd);
					}
					
					write_char(serial_fd, 0xa);
//printf("main %d: i=%d c=%d\n", __LINE__, i, c);
				}
				else
				{
					write_char(serial_fd, c);
//printf("main %d: i=%d c=%d\n", __LINE__, i, c);
				}
			}
		}


	}
}







