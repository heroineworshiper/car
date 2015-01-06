/*
 * Truck
 * Copyright (C) 2007-2014  Adam Williams <broadcast at earthling dot net>
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

vision_t vision;





//#define SHOW_FRAMERATE


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
#define SATURATION 34


#define GAIN 0
#endif // LOGITEC




#ifdef SUYIN
#define BRIGHTNESS 0
#define CONTRAST 32
#define SATURATION 32
#define SHARPNESS 0
#endif


// Save unprocessed frames to files.  Only good with JPEG
#define RECORD_INPUT
// Read JPEG from files instead of video device.  Disables RECORD_VIDEO
//#define PLAY_VIDEO
// Record the processed images.  Makes display synchronous.
//#define RECORD_OUTPUT
// Starting frame for playback
#define STARTING_FRAME 0
// Prefix of input files to record
#define RECORD_PATH "/root/vision"
// Prefix of output files to play
#define OUTPUT_PATH "/root/output"

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





// http://elinux.org/RPi_Low-level_peripherals#C
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;
unsigned gpio_temp[64];

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
	gpio = gpio_temp;

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


} // setup_io



FILE *tar_fd = 0;


void append_tar(char *filename, unsigned char *data, int size)
{
	if(!tar_fd) tar_fd = fopen("vision.tar", "w");
	unsigned char header[512];
	const unsigned char user_id[] = { 0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xfe };
	const unsigned char blank_checksum[] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
	bzero(header, 512);
	strcpy(header, filename);
	sprintf(header + 100, "0000644");
	memcpy(header + 108, user_id, 8);
	memcpy(header + 116, user_id, 8);
	sprintf(header + 124, "%011o", size);
	sprintf(header + 136, "%011o", 0);
	memcpy(header + 148, blank_checksum, 8);
	header[156] = 0x30;
	sprintf(header + 257, "ustar  ");
	sprintf(header + 265, "nfsnobody");
	sprintf(header + 297, "nfsnobody");
	int chksum = 0;
	int i;
	for(i = 0; i < 512; i++) chksum += header[i];
	sprintf(header + 148, "%o", chksum);
	
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
	if(!tar_fd) tar_fd = fopen("vision.out", "w");
	fwrite(data, size, 1, tar_fd);
	fflush(tar_fd);
}

int init_input(char *path)
{
	int i;
	int error = 0;
	int fd;
	int w = vision.image_w;
	int h = vision.image_h;
	
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
	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
//	vision.v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
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


	vision.picture_data = malloc(vision.image_w * vision.image_h * 3);
	vision.y_buffer = malloc(vision.image_w * vision.image_h);
	vision.u_buffer = malloc(vision.image_w * vision.image_h);
	vision.v_buffer = malloc(vision.image_w * vision.image_h);
	
	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutex_init(&vision.latest_lock, &mutex_attr);
	vision.latest_image = malloc(vision.image_w * vision.image_h * 3);
	


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




int read_frame()
{
	int i, j;

	int picture_size = 0;

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
		for(i = 0; i < vision.image_h; i++)
		{
			unsigned char *input_row = vision.frame_buffer[buffer.index] +
				i * vision.image_w * 2;
			unsigned char *output_y = vision.y_buffer + i * vision.image_w;
			unsigned char *output_u = vision.u_buffer + (i / 2) * (vision.image_w / 2);
			unsigned char *output_v = vision.v_buffer + (i / 2) * (vision.image_w / 2);

			for(j = 0; j < vision.image_w / 2; j++)
			{
				*output_y++ = *input_row++;
				if(!(i % 2)) 
					*output_u++ = *input_row++;
				else
					input_row++;
				*output_y++ = *input_row++;
				if(!(i % 2)) 
					*output_v++ = *input_row++;
				else
					input_row++;
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


	vision.total_frames++;


	return picture_size;

}








 



void init_vision()
{
	int i, j;
	
	bzero(&vision, sizeof(vision_t));


	vision.image_w = 640;
	vision.image_h = 480;

	vision.fd = init_input("/dev/video0");
	if(vision.fd < 0) exit(1);
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

void error_report(int conn, char *code, char *title, char *msg)
{
	char buffer[1024];
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
	write(conn, buffer, strlen(buffer));
}

int send_response(int conn, 
	unsigned char *buffer, 
	int buffer_size, 
	char *content_type)
{
	char string[1024];
	sprintf(string, "HTTP/1.0 200 OK\r\n" 
     	   "Content-Type: %s\r\n" 
     	   "Server: Truck web server\r\n\r\n",
		   content_type);
	write(conn, string, strlen(string));
	write(conn, buffer, buffer_size);
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
			
			unsigned char buffer[1024];
			int bytes_read = read(conn, buffer, 1024);
			buffer[bytes_read] = 0;
//			printf("init_httpd %d: '%s'\n", __LINE__, buffer);

// search for GET
			char *end = buffer + strlen(buffer);
			char *ptr = strstr(buffer, "GET");
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
//							printf("init_httpd %d: '%s'\n", __LINE__, ptr);

							pthread_mutex_lock(&vision.latest_lock);
							unsigned char *buffer2 = malloc(vision.latest_size);
							int buffer2_size = vision.latest_size;
							memcpy(buffer2, vision.latest_image, vision.latest_size);
							pthread_mutex_unlock(&vision.latest_lock);
							
							send_response(conn, buffer2, buffer2_size, "image/jpeg");
							free(buffer2);
						}
						else
						if(!strcmp(ptr, "/total_frames.txt"))
						{
							char string[1024];
							sprintf(string, "%d", vision.frames_written);
							send_response(conn, string, strlen(string) + 1, "text/html");
						}
						else
						if(!strcmp(ptr, "/fps.txt"))
						{
							char string[1024];
							sprintf(string, "%d", vision.fps);
							send_response(conn, string, strlen(string) + 1, "text/html");
						}
						else
// file in the html directory
						if(ptr[0] = '/')
						{
							char string[1024];
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
								unsigned char *buffer = malloc(size + 16);
								fread(buffer, size, 1, fd);
								buffer[size + 1] = 0;
								fclose(fd);
								
								send_response(conn, buffer, size + 1, "text/html");
								free(buffer);
							}
							else
							{
								char string2[1024];
								sprintf(string2, 
									"The requested file '%s' was not found on this server.",
									string);
								error_report(conn, 
									"404", 
									"Not Found",
			                    	string2);
							}
							
						}
						else
						{
							error_report(conn, 
								"404", 
								"Not Found",
			                    "The requested URL was not found on this server.");
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
	
	listen(listener, 1024);
	
	pthread_t tid;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_create(&tid, &attr, httpd_thread, 0);
}


int main()
{
	init_vision();
	init_gpio();
	init_httpd();

// call INP before OUT
	INP_GPIO(LED_GPIO); 
	OUT_GPIO(LED_GPIO);
	
	
	
	int prev_total = vision.total_frames;
	reset_timer(&vision.timer);
	reset_timer(&vision.timer2);
	while(1)
	{
		if(read_frame() >= 0)
		{
/*
 * 			printf("main %d: total_frames=%d size=%d\n", 
 * 				__LINE__, 
 * 				vision.total_frames,
 * 				vision.picture_size);
 */
		}

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

		if(get_timer_difference(&vision.timer) > 100)
		{
			reset_timer(&vision.timer);
			
			pthread_mutex_lock(&vision.latest_lock);
			memcpy(vision.latest_image, vision.picture_data, vision.picture_size);
			vision.latest_size = vision.picture_size;
			pthread_mutex_unlock(&vision.latest_lock);

#ifdef RECORD_INPUT
			char string[1024];
			string[0] = 0;
			
			if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
			{
				sprintf(string, "frame%06d.jpg", vision.frames_written++);
				append_file(vision.picture_data, vision.picture_size);
//				append_tar(string, vision.picture_data, vision.picture_size);
/*
 * 				FILE *out = fopen(string, "w");
 * 				fwrite(vision.picture_data, vision.picture_size, 1, out);
 * 				fclose(out);
 */
			}
			else
			if(vision.v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
			{
/*
 * 				sprintf(string, "frame%06d.ppm", vision.frames_written++);
 * 				FILE *out = fopen(string, "w");
 * 				fwrite(vision.picture_data, vision.picture_size, 1, out);
 * 				fclose(out);
 */
			}
			else
			{
			}
			
			
//			printf("main %d: wrote %d %d bytes", __LINE__, vision.frames_written, vision.picture_size);
#endif
//			printf("\n");
			toggle_led();
		}
	}
}







