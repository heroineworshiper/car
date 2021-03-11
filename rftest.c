// transmit & receive continuously with some 2.4Ghz xbees.  
// Measure the signal strength.
// gcc -o rftest rftest.c


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

#define BUFSIZE 256
void read_uart_text(int fd)
{
    uint8_t buffer[BUFSIZE];
    int result = read(fd, buffer, BUFSIZE);
    if(result <= 0)
    {
        printf("read_uart_text: Unplugged\n");
        exit(0);
    }

    int i;
    for(i = 0; i < result; i++)
    {
        printf("%c", buffer[i]);
    }
    printf("\n");

}


void start_code(uint8_t c);
void (*uart_state)(uint8_t) = start_code;
int uart_length;
int uart_counter = 0;
uint8_t uart_buffer[BUFSIZE];

void get_chksum(uint8_t c)
{
    uart_state = start_code;
    
    
//     int i;
//     for(i = 0; i < uart_length; i++)
//     {
//         printf("%02x ", uart_buffer[i]);
//     }
//     printf("\n");
    printf("RSSI: -%d\n", uart_buffer[9]);
}

void get_data(uint8_t c)
{
    uart_buffer[uart_counter++] = c;
    if(uart_counter >= uart_length)
    {
        uart_state = get_chksum;
    }
}

void get_len2(uint8_t c)
{
    uart_length |= c;
    uart_counter = 0;
    uart_state = get_data;
}

void get_len1(uint8_t c)
{
    uart_length = ((int)c) << 8;
    uart_state = get_len2;
}

void start_code(uint8_t c)
{
    if(c == 0x7e)
    {
        uart_state = get_len1;
    }
}

void read_uart(int fd)
{
    uint8_t c;
    int result = read(fd, &c, 1);
    if(result <= 0)
    {
        printf("read_uart: Unplugged\n");
        exit(0);
    }

    uart_state(c);
}


void main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("Usage:\nrftest r /dev/ttyUSB0\n");
        printf("rftest w /dev/ttyUSB1\n");
        exit(1);
    }

    char *mode = argv[1];
    char *uart_path = argv[2];

// open the serial port
    int fd = init_serial(uart_path, B115200, 0);

// enter API mode
    printf("Initializing XBEE.\n");
    sleep(1);
    
    const char *text1 = "+++";
    write(fd, text1, strlen(text1));
    sleep(1);
    read_uart_text(fd);
    
    const char *text2 = "ATAP01\r\n";
    write(fd, text2, strlen(text2));
    read_uart_text(fd);
    
    const char *text3 = "ATCN\r\n";
    write(fd, text3, strlen(text3));
    read_uart_text(fd);

    if(!strcmp(mode, "r"))
    {
        printf("Reading\n");
        while(1)
        {
            read_uart(fd);
        }
    }
    else
    if(!strcmp(mode, "w"))
    {
        printf("Writing\n");
        while(1)
        {
            int length = 19;
            int total_length = length + 4;
            uint8_t test_packet[] = {
                0x7e, // delimiter
                0x00, length, // length
                0x00, // transmit to 64 bit address
                0x00, // frame ID
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, // address
                0x01, // disable ACK
                0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // data
                0x00
            };

            int i;
            int chksum = 0;
            for(i = 3; i < 3 + length; i++)
            {
                chksum += test_packet[i];
            }
            test_packet[3 + length] = 0xff - (chksum & 0xff);

            printf("Writing ");
            for(i = 0; i < total_length; i++)
            {
                printf("%02x ", test_packet[i]);
            }
            printf("\n");
            write(fd, test_packet, total_length);
//            read_uart(fd);

            usleep(100000);
        }
    }
}









