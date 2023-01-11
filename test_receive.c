// A better alternative to minicom
// Serial terminal with custom baud

// gcc -o test_receive test_receive.c


// sample XBee commands:
// atbd13900
// atbd6


//#define TEST_MPU9150
//#define TEST_GYRO
//#define TEST_RADIO
//#define TEST_ADC
//#define TEST_IR
//#define ADD_NEWLINES


// delay between characters required for programming bluetooth
int output_delay = 1000;

#define LOG_FILE "terminal.cap"

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
#ifndef __clang__
#include <linux/serial.h>
#endif 


int hex_output = 1;
int ascii_output = 1;
int local_echo = 0;
int send_cr = 1;

void quit()
{
// reset the console
	struct termios info;
	tcgetattr(fileno(stdin), &info);
	info.c_lflag |= ICANON;
	info.c_lflag |= ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &info);
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
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
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


int main(int argc, char *argv[])
{
	int baud = 100000;
	int baud_enum;
	int custom_baud = 0;
	char *path = 0;
	int i;
	if(argc > 1)
	{
		for(i = 1; i < argc; i++)
		{
			if(!strcmp(argv[i], "-e"))
			{
				local_echo = 1;
			}
			else
			if(!strcmp(argv[i], "-b"))
			{
				baud = atoi(argv[i + 1]);
				i++;
			}
			else
			if(!strcmp(argv[i], "-x"))
			{
				hex_output = 1;
				ascii_output = 0;
			}
			else
			{
				path = argv[i];
			}
		}
	}

	switch(baud)
	{
		case 300: baud_enum = B300; break;
		case 1200: baud_enum = B1200; break;
		case 2400: baud_enum = B2400; break;
		case 9600: baud_enum = B9600; break;
		case 19200: baud_enum = B19200; break;
		case 38400: baud_enum = B38400; break;
		case 57600: baud_enum = B57600; break;
		case 115200: baud_enum = B115200; break;
		case 230400: baud_enum = B230400; break;
#ifndef __clang__
		case 460800: baud_enum = B460800; break;
		case 500000: baud_enum = B500000; break;
		case 921600: baud_enum = B921600; break;
		case 1000000: baud_enum = B1000000; break;
		case 1152000: baud_enum = B1152000; break;
		case 2000000: baud_enum = B2000000; break;
#endif // !__clang__
		default:
			baud_enum = B115200;
			custom_baud = baud;
			break;
	}

	printf("Receiver test.  Baud=%d hex_output=%d ascii_output=%d local_echo=%d send_cr=%d output_delay=%d\n", 
		baud,
		hex_output,
		ascii_output,
		local_echo,
        send_cr,
        output_delay);
	if(path) printf("path=%s\n", path);

	FILE *fd = fopen(LOG_FILE, "w");
	if(!fd)
	{
		printf("Couldn't open %s\n", LOG_FILE);
	}

	int serial_fd = -1;
	if(path) serial_fd = init_serial(path, baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB0", baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB1", baud_enum, custom_baud);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB2", baud_enum, custom_baud);

	int test_state = 0;
	unsigned char test_buffer[32];
	int distance[4];
	int counter = 0;
	int counter2 = 0;
	int result2 = 0;
	int result3 = 0;
	double gyro_accum = 0;
	double temp_accum = 0;
	int sync_code_state = 0;


	struct timeval start_time;
	struct timeval current_time;
	gettimeofday(&start_time, 0);

	struct termios info;
	tcgetattr(fileno(stdin), &info);
	info.c_lflag &= ~ICANON;
	info.c_lflag &= ~ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &info);

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

		if(FD_ISSET(serial_fd, &rfds))
		{


			unsigned char c = read_char(serial_fd);

			if(fd)
			{
				fputc(c, fd);
				fflush(fd);
			}

			if(hex_output)
			{
				printf("%02x ", (unsigned char)c);
// 0xffe7 causes linefeed
				switch(sync_code_state)
				{
					case 1:
						if(c == 0xe7) 
						{
							printf("\n");
                            sync_code_state = 0;
						}
						else
                        if(c == 0xff)
                        {
                            sync_code_state = 1;
                        }
                        else
						{
							sync_code_state = 0;
						}
						break;

					default:
						if(c == 0xff) sync_code_state = 1;
						break;
				}
				fflush(stdout);
			}




		}

// send input from console
		if(FD_ISSET(0, &rfds))
		{
			int i;
			int bytes = read(0, test_buffer, sizeof(test_buffer));
			
			for(i = 0; i < bytes; i++)
			{
				char c = test_buffer[i];
				
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
					
// delay to avoid overflowing a 9600 passthrough
// doesn't print without local echo
					usleep(output_delay);
					write_char(serial_fd, 0xa);
				}
				else
				{
				
					write_char(serial_fd, c);
				}

// delay to avoid overflowing a 9600 passthrough
				usleep(output_delay);
			}
		}


	}
}







