/*
 * Truck vision
 * Copyright (C) 2007-2015  Adam Williams <broadcast at earthling dot net>
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


// Lane following, using USB cam



#include "vision.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <sys/wait.h>         /*  for waitpid()             */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <linux/spi/spidev.h>
#include "jpeglib.h"
#include <setjmp.h>

vision_t vision;
void detect_path();
void init_yuv();

#define SQR(x) ((x) * (x))
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define TEXTLEN 1024




// Camera type
#define LOGITEC
//#define ZSTAR
//#define SUYIN






#ifdef ZSTAR
#define BRIGHTNESS 255
#define CONTRAST 0
#define SHARPNESS 0
#endif




#ifdef LOGITEC
// Lower brightness just clamps the white level below 255 & increases the framerate
#define BRIGHTNESS 128
#define CONTRAST 32

// Set to 0 to increase framerate
//#define SATURATION 0
//#define SATURATION 34
#define SATURATION 64


#define GAIN 0
#endif // LOGITEC




#ifdef SUYIN
#define BRIGHTNESS 0
#define CONTRAST 32
#define SATURATION 32
#define SHARPNESS 0
#endif


// Save unprocessed frames to files.  Only good with JPEG input
//#define RECORD_INPUT
// Read JPEG from files instead of video device.
#define PLAYBACK
// Record the processed images.
#define RECORD_OUTPUT


// quality of recorded JPEGs
#define JPEG_QUALITY 90
// Starting frame for playback
#define STARTING_FRAME 0
// Device to read
#define DEVICE_PATH "/dev/video0"

#define CONFIG_PATH "vision.conf"

// space for compressed output frame
#define PICTURE_DATA_SIZE 0x1000000

#define LED_GPIO 2





#define DHT_SIZE 420
#define HEADERFRAME1 0xaf


static unsigned char dht_data[DHT_SIZE] = {
  0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
  0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x01, 0x00, 0x03,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
  0x0a, 0x0b, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
  0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04,
  0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
  0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15,
  0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95,
  0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
  0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
  0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5,
  0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
  0xfa, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05,
  0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
  0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22,
  0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
  0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
  0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94,
  0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
  0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
  0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
  0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};





#ifndef X86

// http://elinux.org/RPi_Low-level_peripherals#C
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map = 0;

// I/O access
volatile unsigned *gpio = 0;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


//
// Set up a memory regions to access GPIO
//
void init_gpio()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      return;
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %p\n", gpio_map);//errno also set!
      return;
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;

// call INP before OUT
	INP_GPIO(LED_GPIO); 
	OUT_GPIO(LED_GPIO);

} // setup_io



void toggle_led()
{
	vision.led_on = !vision.led_on;
	
	if(vision.led_on)
	{
		GPIO_SET = 1 << LED_GPIO;
	}
	else
	{
		GPIO_CLR = 1 << LED_GPIO;
	}
}


/*
 * unsigned char spi_buffer[64];
 * int spi_size = 0;
 * int spi_offset = 0;
 * uint16_t spi_shift = 0;
 * int spi_counter = 0;
 * void handle_spi()
 * {
 * 	if(spi_counter <= 0)
 * 	{
 * 		if(spi_index < spi_offset)
 * 		{
 * 			spi_shift = spi_buffer[spi_index++];
 * 			spi_counter = 8;
 * 		}
 * 	}
 * 	
 * 	if(spi_counter > 0)
 * 	{
 * 		
 * 	}
 * }
 */


int spi_fd;
#define SPI_PATH "/dev/spidev0.1"
// minimum is 4096
#define SPI_SPEED 4096
#define SPI_BITS 8


void* spi_thread(void *ptr)
{
	while(1)
	{
//printf("spi_thread %d\n", __LINE__);
		sem_wait(&vision.spi_send_lock);
//printf("spi_thread %d\n", __LINE__);
	
		struct spi_ioc_transfer tr;
		
		
		tr.tx_buf = (unsigned long)vision.spi_tx_data;
		tr.rx_buf = (unsigned long)vision.spi_rx_data;
		tr.len = vision.spi_tx_size;
		tr.delay_usecs = 0;
		tr.speed_hz = SPI_SPEED;
		tr.bits_per_word = SPI_BITS;
		

		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
//printf("spi_thread %d\n", __LINE__);
		
		sem_post(&vision.spi_complete_lock);
//printf("spi_thread %d\n", __LINE__);
	}

/*
 * 	printf("spi_thread %d: ", __LINE__);
 * 	int i;
 * 	for(i = 0; i < sizeof(tx_data); i++)
 * 	{
 * 		printf("%02x ", rx_data[i]);
 * 	}
 * 	printf("\n");
 * 		
 */
}


void init_spi()
{
	spi_fd = open(SPI_PATH, O_RDWR);
	if(spi_fd < 0)
	{
		printf("init_spi %d: couldn't open %s\n", __LINE__, SPI_PATH);
		return;
	}
	
	uint8_t mode = 0;
	ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	uint8_t bits = SPI_BITS;
	ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	uint32_t speed = SPI_SPEED;
	ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	
	sem_init(&vision.spi_send_lock, 0, 0);
	sem_init(&vision.spi_complete_lock, 0, 1);

	pthread_t tid;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_create(&tid, &attr, spi_thread, 0);

	
}

void write_spi(unsigned char *data, int size)
{
//printf("write_spi %d\n", __LINE__);
//	sem_wait(&vision.spi_complete_lock);
//printf("write_spi %d\n", __LINE__);
	memcpy(vision.spi_tx_data, data, size);
	vision.spi_tx_size = size;
//printf("write_spi %d\n", __LINE__);
	sem_post(&vision.spi_send_lock);
//printf("write_spi %d\n", __LINE__);
}

#endif // !X86



FILE *tar_fd = 0;


void append_tar(char *filename, unsigned char *data, int size)
{
	if(!tar_fd) tar_fd = fopen("vision.tar", "w");
	unsigned char header[512];
	const unsigned char user_id[] = { 0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xfe };
	const unsigned char blank_checksum[] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
	bzero(header, 512);
	strcpy((char*)header, filename);
	sprintf((char*)header + 100, "0000644");
	memcpy(header + 108, user_id, 8);
	memcpy(header + 116, user_id, 8);
	sprintf((char*)header + 124, "%011o", size);
	sprintf((char*)header + 136, "%011o", 0);
	memcpy(header + 148, blank_checksum, 8);
	header[156] = 0x30;
	sprintf((char*)header + 257, "ustar  ");
	sprintf((char*)header + 265, "nfsnobody");
	sprintf((char*)header + 297, "nfsnobody");
	int chksum = 0;
	int i;
	for(i = 0; i < 512; i++) chksum += header[i];
	sprintf((char*)header + 148, "%o", chksum);
	
	fwrite(header, 512, 1, tar_fd);
	fwrite(data, size, 1, tar_fd);
	
	if(size % 512)
	{
		for(i = 0; i < 512 - (size % 512); i++) fputc(0, tar_fd);
	}
	fflush(tar_fd);
}


void append_file(unsigned char *data, int size)
{
#ifdef RECORD_OUTPUT
	if(!tar_fd) tar_fd = fopen(vision.write_path, "w");
	if(!tar_fd)
	{
		printf("Couldn't open %s\n", vision.write_path);
		exit(1);
	}
	
	fwrite(data, size, 1, tar_fd);
	fflush(tar_fd);
	vision.frames_written++;
#endif // RECORD_OUTPUT
}

int init_input(char *path)
{
	int i;
	int error = 0;
	int fd;
	int w = vision.cam_w;
	int h = vision.cam_h;
	
	if(w == 640 && h == 240)
	{
		h = 480;
	}

	printf("init_input %d: opening video device %s\n", __LINE__, path);
	fd = open(path, O_RDWR);
	if(fd == -1)
	{
	
		perror("init_input open");
		return -1;
	}



// Set up frame rate
	struct v4l2_streamparm v4l2_parm;
	v4l2_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(fd, VIDIOC_G_PARM, &v4l2_parm) < 0)
		perror("init_input VIDIOC_G_PARM");

/*
 * 	printf("init_input %d %d time per frame=%d/%d\n", __LINE__,
 * 		v4l2_parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME,
 * 		v4l2_parm.parm.capture.timeperframe.numerator,
 * 		v4l2_parm.parm.capture.timeperframe.denominator);
 */

	v4l2_parm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
	v4l2_parm.parm.capture.timeperframe.numerator = 1;
	v4l2_parm.parm.capture.timeperframe.denominator = 100;

	if(ioctl(fd, VIDIOC_S_PARM, &v4l2_parm) < 0)
		perror("init_input VIDIOC_S_PARM");

	if(ioctl(fd, VIDIOC_G_PARM, &v4l2_parm) < 0)
		perror("init_input VIDIOC_G_PARM");



// Brightness & contrast
	struct v4l2_control ctrl_arg;
	struct v4l2_queryctrl arg;

// User class control
//	printf("init_input %d brightness=0x%x\n", __LINE__, V4L2_CID_BRIGHTNESS);
   	for(i = V4L2_CID_BASE; i < V4L2_CID_LASTP1; i++)
   	{
   		bzero(&arg, sizeof(arg));
   		arg.id = i;
   		if(!ioctl(fd, VIDIOC_QUERYCTRL, &arg))
   		{
   			printf("init_input %d control=0x%x %s value=%d\n", 
   				__LINE__, 
   				i, 
   				arg.name,
   				arg.default_value);
   		}
   	}
	
// Camera class conrol
   	for(i = V4L2_CID_CAMERA_CLASS_BASE; i < V4L2_CID_CAMERA_CLASS_BASE + 32; i++)
   	{
   		bzero(&arg, sizeof(arg));
   		arg.id = i;
   		if(!ioctl(fd, VIDIOC_QUERYCTRL, &arg))
   		{
   			printf("init_input %d control=0x%x %s value=%d\n", 
   				__LINE__, 
   				i, 
   				arg.name,
   				arg.default_value);
   		}
   	}

#ifdef LOGITEC
	ctrl_arg.id = V4L2_CID_BRIGHTNESS;
	ctrl_arg.value = BRIGHTNESS;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}

	ctrl_arg.id = V4L2_CID_SATURATION;
	ctrl_arg.value = SATURATION;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SATURATION");
	}


// fixed exposure
	ctrl_arg.id = V4L2_CID_GAIN;
	ctrl_arg.value = GAIN;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_GAIN");
	}

// 0 is enabled
	ctrl_arg.id = V4L2_CID_EXPOSURE_AUTO;
	ctrl_arg.value = 0;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_EXPOSURE_AUTO");
	}

// 	ctrl_arg.id = V4L2_CID_EXPOSURE_ABSOLUTE;
// // Edmond optical exposure
// 	ctrl_arg.value = settings->shutter_speed_code;
// 
// 	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
// 	{
// 		perror("init_input VIDIOC_S_CTRL V4L2_CID_EXPOSURE_ABSOLUTE");
// 	}

/*
 * 	ctrl_arg.id = V4L2_CID_AUTO_WHITE_BALANCE;
 * 	ctrl_arg.value = 0;
 * 
 * 	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
 * 	{
 * 		perror("init_input VIDIOC_S_CTRL V4L2_CID_AUTO_WHITE_BALANCE");
 * 	}
 * 
 * 	ctrl_arg.id = V4L2_CID_RED_BALANCE;
 * 	ctrl_arg.value = 0;
 * 
 * 	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
 * 	{
 * 		perror("init_input VIDIOC_S_CTRL V4L2_CID_RED_BALANCE");
 * 	}
 * 
 * 	ctrl_arg.id = V4L2_CID_BLUE_BALANCE;
 * 	ctrl_arg.value = 0;
 * 
 * 	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
 * 	{
 * 		perror("init_input VIDIOC_S_CTRL V4L2_CID_BLUE_BALANCE");
 * 	}
 */


#endif // LOGITEC









#ifdef SUYIN
	ctrl_arg.id = V4L2_CID_BRIGHTNESS;
	ctrl_arg.value = BRIGHTNESS;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}

	ctrl_arg.id = V4L2_CID_SATURATION;
	ctrl_arg.value = SATURATION;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SATURATION");
	}

	ctrl_arg.id = V4L2_CID_SHARPNESS;
	ctrl_arg.value = SHARPNESS;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_SHARPNESS");
	}

	ctrl_arg.id = V4L2_CID_AUTO_WHITE_BALANCE;
	ctrl_arg.value = 0;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_AUTO_WHITE_BALANCE");
	}

#endif // SUYIN




#ifdef ZSTAR
	ctrl_arg.id = V4L2_CID_BRIGHTNESS;
	ctrl_arg.value = BRIGHTNESS;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}
	

	ctrl_arg.id = V4L2_CID_SHARPNESS;
	ctrl_arg.value = SHARPNESS;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SHARPNESS");
	}
	

	ctrl_arg.id = V4L2_CID_AUTOGAIN;
	ctrl_arg.value = 0;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input V4L2_CID_AUTOGAIN");
	}

#endif // ZSTAR

	

	
	

// Set up data format
	
	vision.v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(fd, VIDIOC_G_FMT, &vision.v4l2_params) < 0)
		perror("init_input VIDIOC_G_FMT");
	vision.v4l2_params.fmt.pix.width = w;
	vision.v4l2_params.fmt.pix.height = h;


// Probe the compression format
// Show formats
#if 1
	int pixel_formats[] =
	{
/* RGB formats */
		V4L2_PIX_FMT_RGB332,
		V4L2_PIX_FMT_RGB444,
		V4L2_PIX_FMT_RGB555,
		V4L2_PIX_FMT_RGB565,
		V4L2_PIX_FMT_RGB555X,
		V4L2_PIX_FMT_RGB565X,
		V4L2_PIX_FMT_BGR24  ,
		V4L2_PIX_FMT_RGB24  ,
		V4L2_PIX_FMT_BGR32  ,
		V4L2_PIX_FMT_RGB32  ,

/* Grey formats */
		V4L2_PIX_FMT_GREY ,
		V4L2_PIX_FMT_Y10  ,
		V4L2_PIX_FMT_Y16  ,

/* Palette formats */
		V4L2_PIX_FMT_PAL8 ,

/* Luminance+Chrominance formats */
		V4L2_PIX_FMT_YVU410 ,
		V4L2_PIX_FMT_YVU420 ,
		V4L2_PIX_FMT_YUYV   ,
		V4L2_PIX_FMT_YYUV   ,
		V4L2_PIX_FMT_YVYU   ,
		V4L2_PIX_FMT_UYVY   ,
		V4L2_PIX_FMT_VYUY   ,
		V4L2_PIX_FMT_YUV422P,
		V4L2_PIX_FMT_YUV411P,
		V4L2_PIX_FMT_Y41P   ,
		V4L2_PIX_FMT_YUV444 ,
		V4L2_PIX_FMT_YUV555 ,
		V4L2_PIX_FMT_YUV565 ,
		V4L2_PIX_FMT_YUV32  ,
		V4L2_PIX_FMT_YUV410 ,
		V4L2_PIX_FMT_YUV420 ,
		V4L2_PIX_FMT_HI240  ,
		V4L2_PIX_FMT_HM12   ,

/* two planes -- one Y, one Cr + Cb interleaved  */
		V4L2_PIX_FMT_NV12 ,
		V4L2_PIX_FMT_NV21 ,
		V4L2_PIX_FMT_NV16 ,
		V4L2_PIX_FMT_NV61 ,

/* Bayer formats - see http://www.siliconimaging.com/RGB%20Bayer.htm */
		V4L2_PIX_FMT_SBGGR8,
		V4L2_PIX_FMT_SGBRG8,
		V4L2_PIX_FMT_SGRBG8,
		V4L2_PIX_FMT_SRGGB8,
		V4L2_PIX_FMT_SBGGR10,
		V4L2_PIX_FMT_SGBRG10,
		V4L2_PIX_FMT_SGRBG10,
		V4L2_PIX_FMT_SRGGB10,
/* 10bit raw bayer DPCM compressed to 8 bits */
		V4L2_PIX_FMT_SGRBG10DPCM8,
/*
 * 10bit raw bayer, expanded to 16 bits
 * xxxxrrrrrrrrrrxxxxgggggggggg xxxxggggggggggxxxxbbbbbbbbbb...
 */
		V4L2_PIX_FMT_SBGGR16,

/* compressed formats */
		V4L2_PIX_FMT_MJPEG,
		V4L2_PIX_FMT_JPEG ,
		V4L2_PIX_FMT_DV   ,
		V4L2_PIX_FMT_MPEG ,

/*  Vendor-specific formats   */
		V4L2_PIX_FMT_CPIA1  ,
		V4L2_PIX_FMT_WNVA   ,
		V4L2_PIX_FMT_SN9C10X,
		V4L2_PIX_FMT_SN9C20X_I420,
		V4L2_PIX_FMT_PWC1    ,
		V4L2_PIX_FMT_PWC2    ,
		V4L2_PIX_FMT_ET61X251,
		V4L2_PIX_FMT_SPCA501 ,
		V4L2_PIX_FMT_SPCA505 ,
		V4L2_PIX_FMT_SPCA508 ,
		V4L2_PIX_FMT_SPCA561 ,
		V4L2_PIX_FMT_PAC207  ,
		V4L2_PIX_FMT_MR97310A,
		V4L2_PIX_FMT_SN9C2028,
		V4L2_PIX_FMT_SQ905C  ,
		V4L2_PIX_FMT_PJPG    ,
		V4L2_PIX_FMT_OV511   ,
		V4L2_PIX_FMT_OV518   ,
		V4L2_PIX_FMT_STV0680

	};

   	for(i = 0; i < sizeof(pixel_formats) / sizeof(int); i++)
   	{
   		vision.v4l2_params.fmt.pix.pixelformat = pixel_formats[i];
   		if(ioctl(fd, VIDIOC_S_FMT, &vision.v4l2_params) >= 0)
   		{
			if(ioctl(fd, VIDIOC_G_FMT, &vision.v4l2_params) >= 0)
			{
				if(vision.v4l2_params.fmt.pix.pixelformat == pixel_formats[i])
				{
   					printf("init_input %d format %d %c%c%c%c is good\n", 
   						__LINE__, 
   						i,
   						((unsigned char*)&pixel_formats[i])[0],
   						((unsigned char*)&pixel_formats[i])[1],
   						((unsigned char*)&pixel_formats[i])[2],
   						((unsigned char*)&pixel_formats[i])[3]);
				}
			}
   		}
   	}
#endif // 0


#ifdef LOGITEC
//	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif
#ifdef ZSTAR
	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
#ifdef SUYIN
	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif

	if(ioctl(fd, VIDIOC_S_FMT, &vision.v4l2_params) < 0)
		perror("init_input VIDIOC_S_FMT");
	if(ioctl(fd, VIDIOC_G_FMT, &vision.v4l2_params) < 0)
		perror("init_input VIDIOC_G_FMT");





// Allocate buffers
	struct v4l2_requestbuffers requestbuffers;
	requestbuffers.count = DEVICE_BUFFERS;
	requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	requestbuffers.memory = V4L2_MEMORY_MMAP;
	if(ioctl(fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
	{
		perror("init_input VIDIOC_REQBUFS");
		error = 1;
	}

/*
 * 	printf("init_input %d requested %d buffers got %d\n", 
 * 		__LINE__,
 * 		DEVICE_BUFFERS,
 * 		requestbuffers.count);
 */

	if(!error)
	{
		for(i = 0; i < DEVICE_BUFFERS; i++)
		{
			struct v4l2_buffer buffer;
			buffer.type = requestbuffers.type;
			buffer.index = i;
			if(ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0)
			{
				perror("init_input VIDIOC_QUERYBUF");
				error = 1;
				break;
			}

/*
 * 			printf("init_input %d mmap size=%d offset=%d\n", 
 * 				__LINE__,
 * 				buffer.length,
 * 				buffer.m.offset);
 */
 			vision.buffer_size = buffer.length;
			vision.frame_buffer[i] = (unsigned char*)mmap(NULL,
				buffer.length,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				fd,
				buffer.m.offset);
		}
	}
	
// Start capturing
	if(!error)
	{
		for(i = 0; i < DEVICE_BUFFERS; i++)
		{
			struct v4l2_buffer buffer;
			buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buffer.memory = V4L2_MEMORY_MMAP;
			buffer.index = i;
			if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
			{
				perror("init_input VIDIOC_QBUF");
				error = 1;
				break;
			}
		}
		
		int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if(ioctl(fd, VIDIOC_STREAMON, &streamon_arg) < 0)
			perror("init_input VIDIOC_STREAMON");
	}


	
	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutex_init(&vision.latest_lock, &mutex_attr);
	


	return fd;
}


void close_input(int fd)
{
	int i;

	for(i = 0; i < DEVICE_BUFFERS; i++)
	{
		munmap(vision.frame_buffer[i], vision.buffer_size);
	}

	int streamoff_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(fd, VIDIOC_STREAMOFF, &streamoff_arg) < 0)
		perror("close_input VIDIOC_STREAMOFF");
	close(fd);
}


//typedef struct 
//{
//	struct jpeg_source_mgr pub;	/* public fields */
//} jpeg_source_mgr;
typedef jpeg_source_mgr* jpeg_src_ptr;



struct my_jpeg_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */
  jmp_buf setjmp_buffer;	/* for return to caller */
};
typedef struct my_jpeg_error_mgr* my_jpeg_error_ptr;
struct my_jpeg_error_mgr my_jpeg_error;

METHODDEF(void) init_source(j_decompress_ptr cinfo)
{
    jpeg_src_ptr src = (jpeg_src_ptr) cinfo->src;
}

// space for compressed input frame
#define JPEG_INPUT_ALLOC 0x1000000
#define   M_EOI     0xd9
uint8_t *jpeg_input_buffer = 0;
int jpeg_input_offset = 0;
int jpeg_input_size = 0;
METHODDEF(boolean) fill_input_buffer(j_decompress_ptr cinfo)
{
	jpeg_src_ptr src = (jpeg_src_ptr) cinfo->src;
printf("fill_input_buffer %d '%02x'\n", __LINE__, jpeg_input_buffer[0]);
	jpeg_input_buffer[0] = (JOCTET)0xFF;
	jpeg_input_buffer[1] = (JOCTET)M_EOI;
	src->next_input_byte = jpeg_input_buffer;
	src->bytes_in_buffer = 2;

	return TRUE;
}


METHODDEF(void) skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo->src;

	src->next_input_byte += num_bytes;
	src->bytes_in_buffer -= num_bytes;
}


METHODDEF(void) term_source(j_decompress_ptr cinfo)
{
}


METHODDEF(void) my_jpeg_error_exit (j_common_ptr cinfo)
{
/* cinfo->err really points to a mjpeg_error_mgr struct, so coerce pointer */
  	my_jpeg_error_ptr mjpegerr = (my_jpeg_error_ptr) cinfo->err;

/* Always display the message. */
/* We could postpone this until after returning, if we chose. */
  	(*cinfo->err->output_message) (cinfo);

/* Return control to the setjmp point */
  	longjmp(mjpegerr->setjmp_buffer, 1);
}


int read_file_frame(FILE *fd, uint8_t *y_buffer, uint8_t *u_buffer, uint8_t *v_buffer)
{
	int picture_size = 0;
	int i, j;

	static const uint8_t start_code[] = 
	{
//		0xff, 0xd8, 0xff, 0xe0, 0x00, 0x21, 0x41, 0x56, 0x49, 0x31
		0xff, 0xd8, 0xff, 0xe0
	};
#define START_CODE_SIZE sizeof(start_code)
	if(jpeg_input_buffer == 0) jpeg_input_buffer = (uint8_t*)malloc(JPEG_INPUT_ALLOC);
	jpeg_input_offset = 0;
	jpeg_input_size = 0;

// Find next start code
	int state = 0;
	while(!feof(fd))
	{
		uint8_t c = fgetc(fd);
		if(c == start_code[state])
		{
			jpeg_input_buffer[jpeg_input_size++] = c;
			state++;
			if(state >= START_CODE_SIZE)
			{
// got it
// read rest of frame
//printf("read_file_frame %d offset=%ld\n", __LINE__, ftell(fd) - state);
				state = 0;
				while(!feof(fd) &&
					jpeg_input_size < JPEG_INPUT_ALLOC)
				{
					uint8_t c = fgetc(fd);
					jpeg_input_buffer[jpeg_input_size++] = c;
					if(c == start_code[state])
					{
						state++;
					}
					else
					{
						state = 0;
					}

// next frame found
					if(state >= START_CODE_SIZE)
					{
// rewind
						fseek(fd, -START_CODE_SIZE, SEEK_CUR);
						jpeg_input_size -= START_CODE_SIZE;
						picture_size = jpeg_input_size;
						break;
					}
				}


				if(setjmp(my_jpeg_error.setjmp_buffer))
				{
/* If we get here, the JPEG code has signaled an error. */
					printf("read_file_frame %d: JPEG error\n", __LINE__);
					return 0;
				}

				struct jpeg_decompress_struct cinfo;
				cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
				my_jpeg_error.pub.error_exit = my_jpeg_error_exit;
				jpeg_create_decompress(&cinfo);
				cinfo.src = (struct jpeg_source_mgr*)
    				(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
        			JPOOL_PERMANENT,
					sizeof(jpeg_source_mgr));
				jpeg_src_ptr src = (jpeg_src_ptr)cinfo.src;
				src->init_source = init_source;
				src->fill_input_buffer = fill_input_buffer;
				src->skip_input_data = skip_input_data;
				src->resync_to_restart = jpeg_resync_to_restart; /* use default method */
				src->term_source = term_source;
				src->next_input_byte = jpeg_input_buffer;
				src->bytes_in_buffer = jpeg_input_size;
				jpeg_read_header(&cinfo, 1);


//printf("read_file_frame %d %dx%d\n", __LINE__, cinfo.input_width, cinfo.input_height);
				cinfo.raw_data_out = TRUE;
				cinfo.err->num_warnings = 1;
				

				jpeg_start_decompress(&cinfo);
				unsigned char **mcu_rows[3];
				for(i = 0; i < 3; i++)
				{
					mcu_rows[i] = (unsigned char**)malloc(16 * sizeof(unsigned char*));
				}


				while(cinfo.output_scanline < vision.cam_h)
				{
//printf("read_file_frame %d scanline=%d\n", __LINE__, cinfo.output_scanline);
					for(j = 0; j < 16; j++)
					{
						int output_y = cinfo.output_scanline + j;
						mcu_rows[0][j] = vision.picture_data +
							output_y * vision.cam_w;
						if(!(j % 2))
						{
							mcu_rows[1][j / 2] = vision.picture_data +
								vision.cam_w * vision.cam_h +
								(output_y / 2) * (vision.cam_w / 2);
							mcu_rows[2][j / 2] = vision.picture_data +
								vision.cam_w * vision.cam_h +
								vision.cam_w * vision.cam_h / 4 +
								(output_y / 2) * (vision.cam_w / 2);
						}
					}

//printf("read_file_frame %d\n", __LINE__);
					jpeg_read_raw_data(&cinfo, 
						mcu_rows, 
						vision.cam_h);

//printf("read_file_frame %d\n", __LINE__);
				}


//printf("read_file_frame %d\n", __LINE__);
				jpeg_finish_decompress(&cinfo);
				jpeg_destroy_decompress(&cinfo);
//printf("read_file_frame %d\n", __LINE__);

				for(i = 0; i < 3; i++)
				{
					free(mcu_rows[i]);
				}

// move data into working buffer
				if(vision.cam_h == vision.working_h)
				{
					for(i = 0; i < vision.working_h; i += 2)
					{
						unsigned char *out_y1 = y_buffer + i * vision.working_w;
						unsigned char *out_y2 = y_buffer + (i + 1) * vision.working_w;
						unsigned char *out_u1 = u_buffer + i * vision.working_w;
						unsigned char *out_u2 = u_buffer + (i + 1) * vision.working_w;
						unsigned char *out_v1 = v_buffer + i * vision.working_w;
						unsigned char *out_v2 = v_buffer + (i + 1) * vision.working_w;
						unsigned char *in_y1 = vision.picture_data + i * vision.cam_w;
						unsigned char *in_y2 = vision.picture_data + (i + 1) * vision.cam_w;
						unsigned char *in_u = vision.picture_data + 
							vision.cam_w * vision.cam_h +
							(i / 2) * (vision.cam_w / 2);
						unsigned char *in_v = vision.picture_data +
							vision.cam_w * vision.cam_h +
							vision.cam_w * vision.cam_h / 4 +
							(i / 2) * (vision.cam_w / 2);
						for(j = 0; j < vision.working_w; j += 2)
						{
							*out_y1++ = *in_y1++;
							*out_y1++ = *in_y1++;
							*out_y2++ = *in_y2++;
							*out_y2++ = *in_y2++;
							*out_u1++ = *in_u;
							*out_u1++ = *in_u;
							*out_u2++ = *in_u;
							*out_u2++ = *in_u++;
							*out_v1++ = *in_v;
							*out_v1++ = *in_v;
							*out_v2++ = *in_v;
							*out_v2++ = *in_v++;
						}
					}
				}
				else
				if(vision.cam_h == vision.working_h * 2)
				{
// shrink input to working size
					for(i = 0; i < vision.working_h; i++)
					{
						unsigned char *in_y1 = vision.picture_data + (i * 2) * vision.cam_w;
						unsigned char *in_y2 = vision.picture_data + (i * 2 + 1) * vision.cam_w;
						unsigned char *in_u = vision.picture_data + 
							vision.cam_w * vision.cam_h + 
							i * (vision.cam_w / 2);
						unsigned char *in_v = vision.picture_data +
							vision.cam_w * vision.cam_h +
							vision.cam_w * vision.cam_h / 4 +
							i * (vision.cam_w / 2);
						unsigned char *out_y = y_buffer + i * vision.working_w;
						unsigned char *out_u = u_buffer + i * vision.working_w;
						unsigned char *out_v = v_buffer + i * vision.working_w;
						for(j = 0; j < vision.working_w; j++)
						{
							*out_y++ = ((uint32_t)*in_y1++ + 
								(uint32_t)*in_y1++ + 
								(uint32_t)*in_y2++ +
								(uint32_t)*in_y2++) / 4;
							*out_u++ = *in_u++;
							*out_v++ = *in_v++;
//							*out_u++ = 0x80;
//							*out_v++ = 0x80;
						}
					}
				}
				else
				if(vision.cam_h == vision.working_h * 4)
				{
// shrink input to working size
					for(i = 0; i < vision.working_h; i++)
					{
						unsigned char *in_y1 = vision.picture_data + (i * 4 + 0) * vision.cam_w;
						unsigned char *in_y2 = vision.picture_data + (i * 4 + 1) * vision.cam_w;
						unsigned char *in_y3 = vision.picture_data + (i * 4 + 2) * vision.cam_w;
						unsigned char *in_y4 = vision.picture_data + (i * 4 + 3) * vision.cam_w;
						unsigned char *in_u1 = vision.picture_data + 
							vision.cam_w * vision.cam_h + 
							i * 2 * (vision.cam_w / 2);
						unsigned char *in_u2 = vision.picture_data + 
							vision.cam_w * vision.cam_h + 
							(i * 2 + 1) * (vision.cam_w / 2);
						unsigned char *in_v1 = vision.picture_data +
							vision.cam_w * vision.cam_h +
							vision.cam_w * vision.cam_h / 4 +
							i * 2 * (vision.cam_w / 2);
						unsigned char *in_v2 = vision.picture_data +
							vision.cam_w * vision.cam_h +
							vision.cam_w * vision.cam_h / 4 +
							(i * 2 + 1) * (vision.cam_w / 2);
						unsigned char *out_y = y_buffer + i * vision.working_w;
						unsigned char *out_u = u_buffer + i * vision.working_w;
						unsigned char *out_v = v_buffer + i * vision.working_w;
						for(j = 0; j < vision.working_w; j++)
						{
							*out_y++ = ((uint32_t)*in_y1++ + 
								(uint32_t)*in_y1++ + 
								(uint32_t)*in_y1++ + 
								(uint32_t)*in_y1++ + 
								(uint32_t)*in_y2++ +
								(uint32_t)*in_y2++ +
								(uint32_t)*in_y2++ +
								(uint32_t)*in_y2++ + 
								(uint32_t)*in_y3++ +
								(uint32_t)*in_y3++ +
								(uint32_t)*in_y3++ +
								(uint32_t)*in_y3++ + 
								(uint32_t)*in_y4++ +
								(uint32_t)*in_y4++ +
								(uint32_t)*in_y4++ +
								(uint32_t)*in_y4++) / 16;
							*out_u++ = ((uint32_t)*in_u1++ + 
								(uint32_t)*in_u1++ +
								(uint32_t)*in_u2++ + 
								(uint32_t)*in_u2++) / 4;
							*out_v++ = ((uint32_t)*in_v1++ + 
								(uint32_t)*in_v1++ +
								(uint32_t)*in_v2++ + 
								(uint32_t)*in_v2++) / 4;
//							*out_u++ = 0x80;
//							*out_v++ = 0x80;
						}
					}
				}
				return picture_size;
			}
		}
		else
		{
			jpeg_input_size = 0;
			state = 0;
		}
	}

	return picture_size;
}

int read_frame()
{
	int i, j;
	int picture_size = 0;
	int result = 0;

#ifdef PLAYBACK
	picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
/*
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 * 	if(picture_size > 0) picture_size = read_file_frame(vision.playback_fd, vision.y_buffer, vision.u_buffer, vision.v_buffer);
 */

#else // PLAYBACK

	struct v4l2_buffer buffer;
	bzero(&buffer, sizeof(buffer));
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	if(ioctl(vision.fd, VIDIOC_DQBUF, &buffer) < 0)
	{
		perror("read_frame VIDIOC_DQBUF");
		return -1;
	}

/*
 * printf("read_frame %d index=%d bytesused=%d pixelformat=%c%c%c%c\n", 
 * __LINE__, 
 * buffer.index, 
 * buffer.bytesused,
 * ((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[0],
 * ((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[1],
 * ((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[2],
 * ((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[3]);
 */

	if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
	{
// Massage into a JPEG image
#ifdef ZSTAR
		memcpy (picture_data, frame_buffer[buffer.index], buffer.bytesused);
		picture_size = buffer.bytesused;
#else // ZSTAR

    	memcpy (vision.picture_data, vision.frame_buffer[buffer.index], HEADERFRAME1);
    	memcpy (vision.picture_data + HEADERFRAME1, dht_data, DHT_SIZE);
    	memcpy (vision.picture_data + HEADERFRAME1 + DHT_SIZE,
	    	vision.frame_buffer[buffer.index] + HEADERFRAME1,
	    	(buffer.bytesused - HEADERFRAME1));
		picture_size = vision.picture_size = buffer.bytesused + DHT_SIZE;
#endif // !ZSTAR
	}
	else
// Capture YUYV
	if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
	{
		if(vision.cam_h == vision.working_h)
		{
			for(i = 0; i < vision.working_h; i++)
			{
				unsigned char *input_row = vision.frame_buffer[buffer.index] +
					i * vision.cam_w * 2;
				unsigned char *output_y = vision.y_buffer + i * vision.working_w;
				unsigned char *output_u = vision.u_buffer + i * vision.working_w;
				unsigned char *output_v = vision.v_buffer + i * vision.working_w;

				for(j = 0; j < vision.working_w; j++)
				{
					*output_y++ = input_row[0];
					*output_y++ = input_row[2];
					*output_u++ = input_row[1];
					*output_u++ = input_row[1];
					*output_v++ = input_row[3];
					*output_v++ = input_row[3];

					input_row += 4;
				}
			}
		}
		else
		{
			for(i = 0; i < vision.working_h; i++)
			{
				unsigned char *input_row1 = vision.frame_buffer[buffer.index] +
					(i * 2) * vision.cam_w * 2;
				unsigned char *input_row2 = vision.frame_buffer[buffer.index] +
					(i * 2 + 1) * vision.cam_w * 2;
				unsigned char *output_y = vision.y_buffer + i * vision.working_w;
				unsigned char *output_u = vision.u_buffer + i * vision.working_w;
				unsigned char *output_v = vision.v_buffer + i * vision.working_w;

				for(j = 0; j < vision.working_w; j++)
				{
					*output_y++ = ((uint32_t)input_row1[0] + 
						(uint32_t)input_row1[2] + 
						(uint32_t)input_row2[0] +
						(uint32_t)input_row2[2]) / 4;
					*output_u++ = ((uint32_t)input_row1[1] + 
						(uint32_t)input_row2[1]) / 2;
					*output_v++ = ((uint32_t)input_row1[3] +
						(uint32_t)input_row2[3]) / 2;

					input_row1 += 4;
					input_row2 += 4;
				}
			}
		}

		picture_size = vision.picture_size = buffer.bytesused;
	}
	else
	if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
	{
printf("read_frame %d index=%d bytesused=%d\n", __LINE__, buffer.index, buffer.bytesused);
		memcpy (vision.picture_data, vision.frame_buffer[buffer.index], buffer.bytesused);
		picture_size = vision.picture_size = buffer.bytesused;
	}
	else
	{
		printf("read_frame %d: unknown picture format: %c%c%c%c\n", 
			__LINE__,
			((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[0],
			((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[1],
			((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[2],
			((unsigned char*)&vision.v4l2_params.fmt.pix.pixelformat)[3]);
	}


// Requeue buffer
	if(ioctl(vision.fd, VIDIOC_QBUF, &buffer) < 0)
	{
		perror("read_frame VIDIOC_QBUF");
	}


/*
 * 	printf("read_frame %d buffer=%d length=%d %02x %02x %02x %02x %02x %02x %02x %02x\n", 
 * 		__LINE__, 
 * 		buffer.index,
 * 		buffer.length,
 * 		frame_buffer[buffer.index][0],
 * 		frame_buffer[buffer.index][1],
 * 		frame_buffer[buffer.index][2],
 * 		frame_buffer[buffer.index][3],
 * 		frame_buffer[buffer.index][4],
 * 		frame_buffer[buffer.index][5], 
 * 		frame_buffer[buffer.index][6],
 * 		frame_buffer[buffer.index][7]);
 */
#endif // !PLAYBACK

	vision.total_frames++;


	return picture_size;

}


typedef struct 
{
	struct jpeg_destination_mgr pub; /* public fields */

	JOCTET *buffer;		/* Pointer to buffer */
} mjpeg_destination_mgr;

typedef mjpeg_destination_mgr *mjpeg_dest_ptr;


METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
  	mjpeg_dest_ptr dest = (mjpeg_dest_ptr)cinfo->dest;

/* Set the pointer to the preallocated buffer */
  	dest->buffer = vision.picture_data;
  	dest->pub.next_output_byte = vision.picture_data;
  	dest->pub.free_in_buffer = PICTURE_DATA_SIZE;
}

METHODDEF(boolean) empty_output_buffer(j_compress_ptr cinfo)
{
	printf("empty_output_buffer %d called\n", __LINE__);

	return TRUE;
}

METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
/* Just get the length */
	mjpeg_dest_ptr dest = (mjpeg_dest_ptr)cinfo->dest;
	vision.picture_size = PICTURE_DATA_SIZE - 
		dest->pub.free_in_buffer;
}

GLOBAL(void) jpeg_buffer_dest(j_compress_ptr cinfo)
{
  	mjpeg_dest_ptr dest;

/* The destination object is made permanent so that multiple JPEG images
 * can be written to the same file without re-executing jpeg_stdio_dest.
 * This makes it dangerous to use this manager and a different destination
 * manager serially with the same JPEG object, because their private object
 * sizes may be different.  Caveat programmer.
 */
	if(cinfo->dest == NULL) 
	{	
/* first time for this JPEG object? */
      	cinfo->dest = (struct jpeg_destination_mgr *)
    		(*cinfo->mem->alloc_small)((j_common_ptr)cinfo, 
				JPOOL_PERMANENT,
				sizeof(mjpeg_destination_mgr));
	}

	dest = (mjpeg_dest_ptr)cinfo->dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_output_buffer;
	dest->pub.term_destination = term_destination;
}


void print_buffer(unsigned char *data, int size)
{
	int i;
	for(i = 0; i < size; i++)
	{
		printf("%02x ", data[i]);
		if(!((i + 1) % 16)) printf("\n");
	}
	printf("\n");
}

void compress_jpeg()
{
	struct jpeg_compress_struct jpeg_compress;
 	struct jpeg_error_mgr jerr;
	jpeg_compress.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&jpeg_compress);
	jpeg_compress.image_width = vision.output_w;
	jpeg_compress.image_height = vision.output_h;
	jpeg_compress.input_components = 3;
	jpeg_compress.in_color_space = JCS_RGB;
	jpeg_set_defaults(&jpeg_compress);
	jpeg_set_quality(&jpeg_compress, JPEG_QUALITY, 0);
	jpeg_compress.dct_method = JDCT_IFAST;
	jpeg_compress.comp_info[0].h_samp_factor = 2;
	jpeg_compress.comp_info[0].v_samp_factor = 2;
	jpeg_compress.comp_info[1].h_samp_factor = 1;
	jpeg_compress.comp_info[1].v_samp_factor = 1;
	jpeg_compress.comp_info[2].h_samp_factor = 1;
	jpeg_compress.comp_info[2].v_samp_factor = 1;
	jpeg_buffer_dest(&jpeg_compress);

	int i, j;
	unsigned char **mcu_rows[3];
	for(i = 0; i < 3; i++)
	{
		mcu_rows[i] = (unsigned char**)malloc(16 * sizeof(unsigned char*));
	}

	jpeg_compress.raw_data_in = TRUE;
	jpeg_start_compress(&jpeg_compress, TRUE);
	while(jpeg_compress.next_scanline < jpeg_compress.image_height)
	{
		for(i = 0; 
			i < 16 && i + jpeg_compress.next_scanline < vision.output_h; 
			i++)
		{
			mcu_rows[0][i] = vision.out_y + 
				(jpeg_compress.next_scanline + i) * vision.output_w;
			if(i < 8)
			{
				unsigned char *u_row = mcu_rows[1][i] = vision.out_u +
					(jpeg_compress.next_scanline + i * 2) * vision.output_w;
				unsigned char *v_row = mcu_rows[2][i] = vision.out_v +
					(jpeg_compress.next_scanline + i * 2) * vision.output_w;
// pack UV data for this line
				for(j = 0; j < vision.output_w / 2; j++)
				{
					u_row[j] = u_row[j * 2];
					v_row[j] = v_row[j * 2];
				}
			}
		}

		jpeg_write_raw_data(&jpeg_compress, 
			mcu_rows, 
			vision.output_h);
	}
	
	
	
	jpeg_finish_compress(&jpeg_compress);
	jpeg_destroy((j_common_ptr)&jpeg_compress);


	for(i = 0; i < 3; i++)
	{
		free(mcu_rows[i]);
	}
	
//	printf("compress_jpeg %d\n", __LINE__);
//	print_buffer(vision.picture_data, vision.picture_size);

	append_file(vision.picture_data, vision.picture_size);
}






 



void init_vision()
{
	int i, j;
	
	bzero(&vision, sizeof(vision_t));

// load config file
	
	FILE *in = fopen(CONFIG_PATH, "r");
	if(!in)
	{
		printf("Failed to open settings file %s\n", CONFIG_PATH);
		exit(1);
	}
	
	vision.cam_w = 640;
	vision.cam_h = 480;
	vision.working_w = 320;
	vision.working_h = 240;
	strcpy(vision.read_path, "vision.in");
	strcpy(vision.write_path, "vision.out");
	strcpy(vision.ref_path, "downhill1.mov");


	char key[TEXTLEN];
	char value[TEXTLEN];
	while(!feof(in))
	{
		key[0] = 0;
		value[0] = 0;
		int temp = fscanf(in, "%s %s", key, value);
		if(key[0])
		{
			char *ptr = key;
			while(*ptr != 0 && 
				(*ptr == ' ' || *ptr == '\t')) ptr++;
			if(*ptr != '#')
			{
//printf("init_vision %d '%s' '%s'\n", __LINE__, key, value);
				if(!strcasecmp(ptr, "TOP_X1")) vision.top_x1 = atoi(value);
				else
				if(!strcasecmp(ptr, "TOP_X2")) vision.top_x2 = atoi(value);
				else
				if(!strcasecmp(ptr, "TOP_Y")) vision.top_y = atoi(value);
				else
				if(!strcasecmp(ptr, "BOTTOM_X1")) vision.bottom_x1 = atoi(value);
				else
				if(!strcasecmp(ptr, "BOTTOM_X2")) vision.bottom_x2 = atoi(value);
				else
				if(!strcasecmp(ptr, "BOTTOM_Y")) vision.bottom_y = atoi(value);
				else
				if(!strcasecmp(ptr, "SIDE_W")) vision.side_w = atoi(value);
				else
				if(!strcasecmp(ptr, "SEARCH_STEP")) vision.search_step = atoi(value);
				else
				if(!strcasecmp(ptr, "CAM_W")) vision.cam_w = atoi(value);
				else
				if(!strcasecmp(ptr, "CAM_H")) vision.cam_h = atoi(value);
				else
				if(!strcasecmp(ptr, "WORKING_W")) vision.working_w = atoi(value);
				else
				if(!strcasecmp(ptr, "WORKING_H")) vision.working_h = atoi(value);
				else
				if(!strcasecmp(ptr, "OUTPUT_W")) vision.output_w = atoi(value);
				else
				if(!strcasecmp(ptr, "OUTPUT_H")) vision.output_h = atoi(value);
				else
				if(!strcasecmp(ptr, "READ_PATH")) strcpy(vision.read_path, value);
				else
				if(!strcasecmp(ptr, "WRITE_PATH")) strcpy(vision.write_path, value);
				else
				if(!strcasecmp(ptr, "REF_PATH")) strcpy(vision.ref_path, value);
			}
			else
			{
// find end of line
				while(!feof(in) && fgetc(in) != '\n')
				{
				}
			}
		}
	}

	printf("init_vision %d: \n", __LINE__);
	printf("top_x1=%d\n", vision.top_x1);
	printf("top_x2=%d\n", vision.top_x2);
	printf("top_y=%d\n", vision.top_y);
	printf("bottom_x1=%d\n", vision.bottom_x1);
	printf("bottom_x2=%d\n", vision.bottom_x2);
	printf("bottom_y=%d\n", vision.bottom_y);
	printf("side_w=%d\n", vision.side_w);
	printf("search_step=%d\n", vision.search_step);
	printf("cam_w=%d\n", vision.cam_w);
	printf("cam_h=%d\n", vision.cam_h);
	printf("working_w=%d\n", vision.working_w);
	printf("working_h=%d\n", vision.working_h);
	printf("output_w=%d\n", vision.output_w);
	printf("output_h=%d\n", vision.output_h);
	printf("read_path=%s\n", vision.read_path);
	printf("ref_path=%s\n", vision.ref_path);
	printf("write_path=%s\n", vision.write_path);
	
	vision.y_buffer = (unsigned char*)malloc(vision.working_w * vision.working_h);
	vision.u_buffer = (unsigned char*)malloc(vision.working_w * vision.working_h);
	vision.v_buffer = (unsigned char*)malloc(vision.working_w * vision.working_h);
// 	if(vision.output_w == vision.working_w &&
// 		vision.output_h == vision.working_h)
// 	{
// 		vision.out_y = vision.y_buffer;
// 		vision.out_u = vision.u_buffer;
// 		vision.out_v = vision.v_buffer;
// 	}
// 	else
	{
		vision.out_y = (unsigned char*)malloc(vision.output_w * vision.output_h);
		vision.out_u = (unsigned char*)malloc(vision.output_w * vision.output_h);
		vision.out_v = (unsigned char*)malloc(vision.output_w * vision.output_h);
	}
	
	vision.picture_data = (unsigned char*)malloc(PICTURE_DATA_SIZE);
	vision.latest_image = (unsigned char*)malloc(PICTURE_DATA_SIZE);
	vision.accum = (int*)malloc(vision.working_w * vision.working_h * sizeof(int));
	vision.mask = (unsigned char*)malloc(vision.working_w * vision.working_h);

#ifndef PLAYBACK
	vision.fd = init_input((char*)DEVICE_PATH);
	if(vision.fd < 0) exit(1);
#else
	vision.playback_fd = fopen(vision.read_path, "r");
	if(!vision.playback_fd) 
	{
		printf("Couldn't open %s\n", vision.read_path);
		exit(1);
	}
#endif

}




void reset_timer(cartimer_t *ptr)
{
	gettimeofday(&ptr->start_time, 0);
}

int get_timer_difference(cartimer_t *ptr)
{
	struct timeval current_time;
	gettimeofday(&current_time, 0);
	int result = current_time.tv_sec * 1000 + current_time.tv_usec / 1000 -
		ptr->start_time.tv_sec * 1000 - ptr->start_time.tv_usec / 1000;
	return result;
}



void error_report(int conn, char *code, char *title, char *msg)
{
	char buffer[TEXTLEN];
	sprintf(buffer, 
		"HTTP/1.0 %s %s\r\n" 
        "\r\n" 
        "<!DOCTYPE HTML PUBLIC \"-//IETF//DTD HTML 2.0//EN\">\r\n" 
        "<TITLE>%s %s</TITLE>\r\n" 
        "</HEAD><BODY>\r\n" 
        "<H1>%s</H1>\r\n%s<P>\r\n" 
        "<HR><ADDRESS>Truck web server\r\n" 
        "</BODY></HTML>\r\n",
		code,
		title,
		code,
		title,
		title,
		msg);
	int result = write(conn, buffer, strlen(buffer));
}

int send_response(int conn, 
	unsigned char *buffer, 
	int buffer_size, 
	char *content_type)
{
	char string[TEXTLEN];
	sprintf(string, "HTTP/1.0 200 OK\r\n" 
     	   "Content-Type: %s\r\n" 
     	   "Server: Truck web server\r\n\r\n",
		   content_type);
	int result = write(conn, string, strlen(string));
	result = write(conn, buffer, buffer_size);
}


int listener;
void* httpd_thread(void *ptr)
{
	int pid;
	while(1)
	{
		int conn = accept(listener, NULL, NULL);

		if ( (pid = fork()) == 0 ) 
		{
// child process
			close(listener);
			
			unsigned char buffer[TEXTLEN];
			int bytes_read = read(conn, buffer, TEXTLEN);
			buffer[bytes_read] = 0;
//			printf("init_httpd %d: '%s'\n", __LINE__, buffer);

// search for GET
			char *end = (char*)buffer + strlen((char*)buffer);
			char *ptr = strstr((char*)buffer, "GET");
			if(ptr)
			{
				ptr += 4;
// get path
				if(ptr < end)
				{
					char *ptr2 = strchr(ptr, ' ');
					if(ptr2 && ptr2 < end)
					{
						*ptr2 = 0;
//						printf("init_httpd %d: '%s'\n", __LINE__, ptr);

// strip arguments
						ptr2 = strchr(ptr, '?');
						if(ptr2)
						{
							*ptr2 = 0;
						}

// create an image
						if(!strcmp(ptr, "/latest.jpg"))
						{
// 							printf("init_httpd %d: latest_size=%d latest_image=%p\n", 
// 								__LINE__, 
// 								vision.latest_size,
// 								vision.latest_image);

							pthread_mutex_lock(&vision.latest_lock);
							unsigned char *buffer2 = (unsigned char*)malloc(vision.latest_size);
							int buffer2_size = vision.latest_size;
							memcpy(buffer2, vision.latest_image, vision.latest_size);
							pthread_mutex_unlock(&vision.latest_lock);
							
							send_response(conn, buffer2, buffer2_size, (char*)"image/jpeg");
							free(buffer2);
						}
						else
						if(!strcmp(ptr, "/total_frames.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.frames_written);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/fps.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.fps);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/path_x.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.path_x);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
// file in the html directory
						if(ptr[0] = '/')
						{
							char string[TEXTLEN];
							if(!strcmp(ptr, "/"))
								sprintf(string, "html/index.html");
							else
								sprintf(string, "html%s", ptr);
							
//							printf("init_httpd %d: '%s'\n", __LINE__, string);
							FILE *fd = fopen(string, "r");
							if(fd)
							{
								fseek(fd, 0, SEEK_END);
								int size = ftell(fd);
								fseek(fd, 0, SEEK_SET);

// need to pad the buffer or fclose locks up
								unsigned char *buffer = (unsigned char*)malloc(size + 16);
								int result = fread(buffer, size, 1, fd);
								buffer[size + 1] = 0;
								fclose(fd);
								
								send_response(conn, buffer, size + 1, (char*)"text/html");
								free(buffer);
							}
							else
							{
								char string2[TEXTLEN];
								sprintf(string2, 
									"The requested file '%s' was not found on this server.",
									string);
								error_report(conn, 
									(char*)"404", 
									(char*)"Not Found",
			                    	string2);
							}
							
						}
						else
						{
							error_report(conn, 
								(char*)"404", 
								(char*)"Not Found",
			                    (char*)"The requested URL was not found on this server.");
						}
					}
				}
			}
			
			close(conn);
			exit(EXIT_SUCCESS);
			
		}
		else
		{
// parent process
			close(conn);
			waitpid(-1, NULL, WNOHANG);
		}
	}
}

// http://www.paulgriffiths.net/program/c/srcs/webservsrc.html
void init_httpd()
{
    listener = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	
	int port = 8080;
	for(port = 8080; port < 8100; port++)
	{
    	servaddr.sin_port = htons(port);
    	if ( bind(listener, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 )
		{
			printf("init_httpd %d: couldn't bind to port %d.\n", __LINE__, port);
			if(port == 99) return;
		}
		else
		{
			printf("init_httpd %d: got port %d\n", __LINE__, port);
			break;
		}
	}
	
	listen(listener, TEXTLEN);
	
	pthread_t tid;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_create(&tid, &attr, httpd_thread, 0);
}

void draw_pixel(int x, int y)
{
	if(!(x >= 0 && y >= 0 && x < vision.output_w && y < vision.output_h)) return;
	int offset = y * vision.output_w + x;
	vision.out_y[offset] = 0xff - vision.out_y[offset];
	vision.out_u[offset] = 0x80 - vision.out_u[offset];
	vision.out_v[offset] = 0x80 - vision.out_v[offset];
}


void draw_line(int x1, int y1, int x2, int y2)
{
	int w = labs(x2 - x1);
	int h = labs(y2 - y1);
//printf("FindObjectMain::draw_line 1 %d %d %d %d\n", x1, y1, x2, y2);

	if(!w && !h)
	{
		draw_pixel(x1, y1);
	}
	else
	if(w > h)
	{
// Flip coordinates so x1 < x2
		if(x2 < x1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = y2 - y1;
		int denominator = x2 - x1;
		int i;
		for(i = x1; i <= x2; i++)
		{
			int y = y1 + (int64_t)(i - x1) * (int64_t)numerator / (int64_t)denominator;
			draw_pixel(i, y);
		}
	}
	else
	{
// Flip coordinates so y1 < y2
		if(y2 < y1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = x2 - x1;
		int denominator = y2 - y1;
		int i;
		for(i = y1; i <= y2; i++)
		{
			int x = x1 + (int64_t)(i - y1) * (int64_t)numerator / (int64_t)denominator;
			draw_pixel(x, i);
		}
	}
//printf("FindObjectMain::draw_line 2\n");
}

void draw_rect(int x1, int y1, int x2, int y2)
{
	draw_line(x1, y1, x2, y1);
	draw_line(x1, y1, x1, y2);
	draw_line(x2, y1, x2, y2);
	draw_line(x1, y2, x2, y2);
}

#define SUM_PIXEL(x, y) \
{ \
	if(x >= 0 && y >= 0 && x < vision.working_w && y < vision.working_h) \
	{ \
		int offset = y * vision.working_w + x; \
		accum_r += vision.y_buffer[offset]; \
/* 		accum_g += vision.u_buffer[offset]; */ \
/*		accum_b += vision.v_buffer[offset]; */ \
		count++; \
	} \
}


int line_color(int x1, int y1, int x2, int y2)
{
	int w = labs(x2 - x1);
	int h = labs(y2 - y1);
//printf("FindObjectMain::draw_line 1 %d %d %d %d\n", x1, y1, x2, y2);

	int64_t accum_r = 0, accum_g = 0, accum_b = 0;
	int count = 0;

	if(!w && !h)
	{
		SUM_PIXEL(x1, y1);
	}
	else
	if(w > h)
	{
// Flip coordinates so x1 < x2
		if(x2 < x1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = y2 - y1;
		int denominator = x2 - x1;
		int i;
		for(i = x1; i <= x2; i++)
		{
			int y = y1 + (int64_t)(i - x1) * (int64_t)numerator / (int64_t)denominator;
			SUM_PIXEL(i, y);
		}
	}
	else
	{
// Flip coordinates so y1 < y2
		if(y2 < y1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = x2 - x1;
		int denominator = y2 - y1;
		int i;
		for(i = y1; i <= y2; i++)
		{
			int x = x1 + (int64_t)(i - y1) * (int64_t)numerator / (int64_t)denominator;
			SUM_PIXEL(x, i);
		}
	}
	
//	return sqrt(SQR(accum_r / count) + SQR(accum_g / count) + SQR(accum_b / count));
	return accum_r / count;
}

void push_to_server()
{
	if(vision.picture_size > 0)
	{
		pthread_mutex_lock(&vision.latest_lock);
//printf("push_to_server %d %d\n", __LINE__, vision.picture_size);
		memcpy(vision.latest_image, vision.picture_data, vision.picture_size);
		vision.latest_size = vision.picture_size;
		pthread_mutex_unlock(&vision.latest_lock);
	}
}

int main()
{
#ifndef X86
	init_spi();
//	init_gpio();
#endif

	init_vision();
	init_yuv();
	
	init_httpd();

	
	
	
	int prev_total = vision.total_frames;
	reset_timer(&vision.timer);
	reset_timer(&vision.timer2);
	while(1)
	{
		if(read_frame() <= 0)
		{
#ifdef PLAYBACK
			break;
#endif
		}


		detect_path();



#ifndef PLAYBACK
#ifdef RECORD_OUTPUT
// compress all frames if recording
		if(1)
#else
// compress 1 frame/sec if only serving web page
		if(get_timer_difference(&vision.timer) > 1000)
#endif
		{
			reset_timer(&vision.timer);


			if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
			{
				append_file(vision.picture_data, vision.picture_size);
			}
			else
			if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
			{
			}
			else
			if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
			{
				compress_jpeg();
			}

// Copy JPEG to web server
			push_to_server();
			
//			printf("main %d: wrote %d %d bytes", __LINE__, vision.frames_written, vision.picture_size);


		}

#else // !PLAYBACK

// compress all frames if playing back
		compress_jpeg();
// Copy JPEG to web server			
		push_to_server();

#endif // PLAYBACK


// Show FPS
		if(get_timer_difference(&vision.timer2) > 1000)
		{
			vision.fps = vision.total_frames - prev_total;
			printf("main %d: fps=%d wrote %d\n", 
				__LINE__, 
				vision.fps,
				vision.frames_written);
			prev_total = vision.total_frames;
			reset_timer(&vision.timer2);
		}

// DEBUG
// process only 1 frame
//if(vision.total_frames >= 1) exit(0);
	}
}







