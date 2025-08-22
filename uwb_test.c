/*
 * UWB tester
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



// gcc -O2 -o uwb_test uwb_test.c



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

#define ENABLE_PDOA
//#define ENABLE_DS

int local_echo = 1;
int send_cr = 1;
int serial_fd = -1;
FILE *log_fd = 0;

typedef struct
{
    int32_t roundA;
#ifdef ENABLE_DS
    int32_t replyA;
    int32_t roundB;
    int32_t replyB;
#else
    int32_t reply_delay;
#endif
    int32_t clock_offset;
#ifdef ENABLE_PDOA
    uint32_t pdoa;
#endif
//    uint8_t base_temp;
//    uint8_t roamer_temp;
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
    int range = 0;
#ifdef ENABLE_PDOA
    int pdoa_accum = 0;
    int pdoa_count = 0;
#endif
    int roamer_temp = 0;
    int base_temp = 0;
    double clock_offset = 0;
    int clock_offset_init = 100;
    int counter = 0;

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
                        test_state++;
                    else
                    if(c == 0xff)
                        continue;
                    else
                        print_it = 1;
                    break;
                case 2:
                {
                    uint8_t *ptr = (uint8_t*)&buffer;
                    ptr[offset++] = c;
//printf("main %d: %d\n", __LINE__, (int)sizeof(ranging_t));
                    int bufsize = 8;
#ifdef ENABLE_DS
                    bufsize += 12;
#else
                    bufsize += 4; // reply_delay
#endif
#ifdef ENABLE_PDOA
                    bufsize += 4;
#endif
                    if(offset >= bufsize)
                    {
                        offset = 0;
                        test_state = 0;
// clock_offset is the biggest source of noise so must be filtered more than range
#define FILTER 0.01
                        if(clock_offset_init > 0)
                        {
                            clock_offset_init--;
                            clock_offset = buffer.clock_offset;
                        }
                        else
                            clock_offset = clock_offset * (1.0 - FILTER) +
                                (double)buffer.clock_offset * FILTER;

#ifdef ENABLE_DS
                        int reply_diff = buffer.replyA - buffer.replyB;
                        int first_rt = buffer.roundA - buffer.replyB;
                        int second_rt = buffer.roundB - buffer.replyA;
#ifdef ENABLE_PDOA
                        int pdoa_qual = buffer.pdoa >> 16;
                        int pdoa = buffer.pdoa & 0x3fff;

                        int fp_th_md = (buffer.pdoa >> 14) & 0x1;
                        int sts_toast = (buffer.pdoa >> 15) & 0x1;
                        if(pdoa & (0x1 << 13)) pdoa |= 0xffffc000;
                        int angle = (int)((double)pdoa / (1 << 11) * 180 / M_PI);
printf("main %d: sts_toast=%d\n", __LINE__, sts_toast);
                        if(!sts_toast)
                        {
                            pdoa_accum += pdoa;
                            pdoa_count++;
                        }
#endif // ENABLE_PDOA
//                        int range = first_rt + second_rt;

                        char string[TXTLEN];
//                         sprintf(string, "DATA: %d %d %d %d %d\n", 
//                             buffer.roundA,
//                             buffer.replyA,
//                             buffer.roundB,
//                             buffer.replyB,
//                             buffer.clock_offset);
//                         sprintf(string, "PDOA: %d %d %d %d\n", pdoa_qual, fp_th_md, pdoa, angle);
//                         printf("%s", string);
// 			            write_log(string);

                        range += (int)((double)(first_rt + second_rt) - 
                            (int)(clock_offset * (double)1200 / 30000));
//                        base_temp += buffer.base_temp;
//                        roamer_temp += buffer.roamer_temp;
                        counter++;
                        if(counter >= 10)
                        {
                            range /= counter;
#ifdef ENABLE_PDOA
                            if(pdoa_count > 0)
                            {
                                pdoa_accum /= pdoa_count;
                            }
                            int angle = (int)((double)pdoa_accum / (1 << 11) * 180 / M_PI);
#endif
//                            base_temp = base_temp * 10 / counter;
//                            roamer_temp = roamer_temp * 10 / counter;

                            sprintf(string, 
                                "PDOA=%d\tANGLE=%d\n", 
                                pdoa_accum,
                                angle);
//                             sprintf(string, 
//                                 "RAW=%d RANGE=%.3fm CLOCK_OFFSET=%.0f\n", 
//                                 range,
//                                 (double)(range - -400) / 700 * .5,
//                                 clock_offset);
                            printf("%s", string);
 			                write_log(string);

                            counter = 0;
                            range = 0;
#ifdef ENABLE_PDOA
                            pdoa_accum = 0;
                            pdoa_count = 0;
#endif

//                            roamer_temp = 0;
//                            base_temp = 0;
                        }
#else // ENABLE_DS

                            char string[TXTLEN];
//                             sprintf(string, "DATA: %08x %08x %08x\n", 
//                                 buffer.roundA,
//                                 buffer.reply_delay,
//                                 buffer.clock_offset);
//                             printf("%s", string);
// 			                write_log(string);
                            range += buffer.roundA - 
                                buffer.reply_delay +
                                (int)(clock_offset * 3700 / 17000);

#ifdef ENABLE_PDOA
                            int pdoa_qual = buffer.pdoa >> 16;
                            int pdoa = buffer.pdoa & 0x3fff;

                            int fp_th_md = (buffer.pdoa >> 14) & 0x1;
                            int sts_toast = (buffer.pdoa >> 15) & 0x1;
                            if(pdoa & (0x1 << 13)) pdoa |= 0xffffc000;
                            int angle = (int)((double)pdoa / (1 << 11) * 180 / M_PI);
//printf("main %d: sts_toast=%d\n", __LINE__, sts_toast);
                            if(!sts_toast)
                            {
                                pdoa_accum += pdoa;
                                pdoa_count++;
                            }
#endif // ENABLE_PDOA

                            counter++;
                            if(counter >= 10)
                            {
                                range /= counter;

#ifdef ENABLE_PDOA
                                if(pdoa_count > 0)
                                {
                                    pdoa_accum /= pdoa_count;
                                }
                                int angle = (int)((double)pdoa_accum / (1 << 11) * 180 / M_PI);
#endif

//                                 sprintf(string, 
//                                     "RANGE=%d CLOCK_OFFSET=%.0f\n", 
//                                     range,
//                                     clock_offset);
//                                 sprintf(string, 
//                                     "RANGE=%.3f CLOCK_OFFSET=%.0f\n", 
//                                     .5 * (range - 16300) / (16600 - 16300),
//                                     clock_offset);
                                sprintf(string, 
                                    "PDOA=%d\tPDOA_COUNT=%d\tANGLE=%d\n", 
                                    pdoa_accum,
                                    pdoa_count,
                                    angle);
                                printf("%s", string);
 			                    write_log(string);
                                counter = 0;
                                range = 0;
#ifdef ENABLE_PDOA
                                pdoa_accum = 0;
                                pdoa_count = 0;
#endif
                            }
#endif // !ENABLE_DS
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







