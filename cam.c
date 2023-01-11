// camera library



#include "cam.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>


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




int init_input(char *path, cam_t *cam)
{
	int i;
	int error = 0;
	int w = cam->cam_w;
	int h = cam->cam_h;
	
	if(w == 640 && h == 240)
	{
		h = 480;
	}

	printf("init_input %d: opening video device %s\n", __LINE__, path);
	cam->fd = open(path, O_RDWR);
	if(cam->fd == -1)
	{
	
		perror("init_input open");
		return -1;
	}



// Set up frame rate
	struct v4l2_streamparm v4l2_parm;
	v4l2_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(cam->fd, VIDIOC_G_PARM, &v4l2_parm) < 0)
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

	if(ioctl(cam->fd, VIDIOC_S_PARM, &v4l2_parm) < 0)
		perror("init_input VIDIOC_S_PARM");

	if(ioctl(cam->fd, VIDIOC_G_PARM, &v4l2_parm) < 0)
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
   		if(!ioctl(cam->fd, VIDIOC_QUERYCTRL, &arg))
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
   		if(!ioctl(cam->fd, VIDIOC_QUERYCTRL, &arg))
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

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}

	ctrl_arg.id = V4L2_CID_SATURATION;
	ctrl_arg.value = SATURATION;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SATURATION");
	}


// fixed exposure
	ctrl_arg.id = V4L2_CID_GAIN;
	ctrl_arg.value = GAIN;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_GAIN");
	}

// 0 is enabled
	ctrl_arg.id = V4L2_CID_EXPOSURE_AUTO;
	ctrl_arg.value = 0;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
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

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}

	ctrl_arg.id = V4L2_CID_SATURATION;
	ctrl_arg.value = SATURATION;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SATURATION");
	}

	ctrl_arg.id = V4L2_CID_SHARPNESS;
	ctrl_arg.value = SHARPNESS;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_SHARPNESS");
	}

	ctrl_arg.id = V4L2_CID_AUTO_WHITE_BALANCE;
	ctrl_arg.value = 0;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_AUTO_WHITE_BALANCE");
	}

#endif // SUYIN




#ifdef ZSTAR
	ctrl_arg.id = V4L2_CID_BRIGHTNESS;
	ctrl_arg.value = BRIGHTNESS;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL BRIGHTNESS");
	}


	ctrl_arg.id = V4L2_CID_CONTRAST;
	ctrl_arg.value = CONTRAST;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL CONTRAST");
	}
	

	ctrl_arg.id = V4L2_CID_SHARPNESS;
	ctrl_arg.value = SHARPNESS;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL SHARPNESS");
	}
	

	ctrl_arg.id = V4L2_CID_AUTOGAIN;
	ctrl_arg.value = 0;

	if(ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input V4L2_CID_AUTOGAIN");
	}

#endif // ZSTAR

	

	
	

// Set up data format
	
	cam->v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(cam->fd, VIDIOC_G_FMT, &cam->v4l2_params) < 0)
		perror("init_input VIDIOC_G_FMT");
	cam->v4l2_params.fmt.pix.width = w;
	cam->v4l2_params.fmt.pix.height = h;


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
   		cam->v4l2_params.fmt.pix.pixelformat = pixel_formats[i];
   		if(ioctl(cam->fd, VIDIOC_S_FMT, &cam->v4l2_params) >= 0)
   		{
			if(ioctl(cam->fd, VIDIOC_G_FMT, &cam->v4l2_params) >= 0)
			{
				if(cam->v4l2_params.fmt.pix.pixelformat == pixel_formats[i])
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
//	cam->v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	cam->v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif
#ifdef ZSTAR
	cam->v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
#ifdef SUYIN
	cam->v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif

	if(ioctl(cam->fd, VIDIOC_S_FMT, &cam->v4l2_params) < 0)
		perror("init_input VIDIOC_S_FMT");
	if(ioctl(cam->fd, VIDIOC_G_FMT, &cam->v4l2_params) < 0)
		perror("init_input VIDIOC_G_FMT");





// Allocate buffers
	struct v4l2_requestbuffers requestbuffers;
	requestbuffers.count = DEVICE_BUFFERS;
	requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	requestbuffers.memory = V4L2_MEMORY_MMAP;
	if(ioctl(cam->fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
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
			if(ioctl(cam->fd, VIDIOC_QUERYBUF, &buffer) < 0)
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
 			cam->buffer_size = buffer.length;
			cam->frame_buffer[i] = (unsigned char*)mmap(NULL,
				buffer.length,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				cam->fd,
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
			if(ioctl(cam->fd, VIDIOC_QBUF, &buffer) < 0)
			{
				perror("init_input VIDIOC_QBUF");
				error = 1;
				break;
			}
		}
		
		int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if(ioctl(cam->fd, VIDIOC_STREAMON, &streamon_arg) < 0)
			perror("init_input VIDIOC_STREAMON");
	}


	


	return cam->fd;
}



void close_input(cam_t *cam)
{
	int i;

	for(i = 0; i < DEVICE_BUFFERS; i++)
	{
		munmap(cam->frame_buffer[i], cam->buffer_size);
	}

	int streamoff_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(cam->fd, VIDIOC_STREAMOFF, &streamoff_arg) < 0)
		perror("close_input VIDIOC_STREAMOFF");
	close(cam->fd);
}


int read_frame(cam_t *cam,
	unsigned char *output_y_arg,
	unsigned char *output_u_arg,
	unsigned char *output_v_arg,
	int output_w)
{
	int i, j;
	int picture_size = 0;
	int result = 0;

#ifdef PLAYBACK
	picture_size = read_file_frame(cam->playback_fd, 
		output_y_arg, 
		output_u_arg, 
		output_v_arg);


#else // PLAYBACK

// printf("read_frame %d\n", 
// __LINE__);
	struct v4l2_buffer buffer;
	bzero(&buffer, sizeof(buffer));
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	if(ioctl(cam->fd, VIDIOC_DQBUF, &buffer) < 0)
	{
		perror("read_frame VIDIOC_DQBUF");
		return -1;
	}

// printf("read_frame %d index=%d bytesused=%d pixelformat=%c%c%c%c\n", 
// __LINE__, 
// buffer.index, 
// buffer.bytesused,
// ((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[0],
// ((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[1],
// ((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[2],
// ((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[3]);

	if(cam->v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
	{
// Massage into a JPEG image
#ifdef ZSTAR
		memcpy (picture_data, frame_buffer[buffer.index], buffer.bytesused);
		picture_size = buffer.bytesused;
#else // ZSTAR

    	memcpy (cam->picture_data, cam->frame_buffer[buffer.index], HEADERFRAME1);
    	memcpy (cam->picture_data + HEADERFRAME1, dht_data, DHT_SIZE);
    	memcpy (cam->picture_data + HEADERFRAME1 + DHT_SIZE,
	    	cam->frame_buffer[buffer.index] + HEADERFRAME1,
	    	(buffer.bytesused - HEADERFRAME1));
		picture_size = cam->picture_size = buffer.bytesused + DHT_SIZE;
#endif // !ZSTAR

#ifdef RECORD_INPUT
		append_file(cam->picture_data, picture_size);
#endif
	}
	else
// Capture YUYV
	if(cam->v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
	{

#ifdef RECORD_INPUT
		if(get_timer_difference(&cam->record_input_timer) > cam->record_period)
		{
			reset_timer(&cam->record_input_timer);
			compress_input(cam->frame_buffer[buffer.index], 
				cam->cam_w, 
				cam->cam_h);
			append_file(cam->picture_in, cam->in_size);
		}
#endif


		if(cam->cam_h == cam->working_h)
		{
			for(i = 0; i < cam->working_h; i++)
			{
				unsigned char *input_row = cam->frame_buffer[buffer.index] +
					i * cam->cam_w * 2;
				unsigned char *output_y = output_y_arg + i * output_w;
				unsigned char *output_u = output_u_arg + i * output_w;
				unsigned char *output_v = output_v_arg + i * output_w;

				for(j = 0; j < cam->working_w / 2; j++)
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
		if(cam->cam_h == cam->working_h * 2)
		{
			for(i = 0; i < cam->working_h; i++)
			{
				unsigned char *input_row1 = cam->frame_buffer[buffer.index] +
					(i * 2) * cam->cam_w * 2;
				unsigned char *input_row2 = cam->frame_buffer[buffer.index] +
					(i * 2 + 1) * cam->cam_w * 2;
				unsigned char *output_y = output_y_arg + i * output_w;
				unsigned char *output_u = output_u_arg + i * output_w;
				unsigned char *output_v = output_v_arg + i * output_w;

				for(j = 0; j < cam->working_w; j++)
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
		else
		if(cam->cam_h == cam->working_h * 4)
		{
			for(i = 0; i < cam->working_h; i++)
			{
				unsigned char *input_row1 = cam->frame_buffer[buffer.index] +
					(i * 4) * cam->cam_w * 2;
				unsigned char *input_row2 = cam->frame_buffer[buffer.index] +
					(i * 4 + 1) * cam->cam_w * 2;
				unsigned char *input_row3 = cam->frame_buffer[buffer.index] +
					(i * 4 + 2) * cam->cam_w * 2;
				unsigned char *input_row4 = cam->frame_buffer[buffer.index] +
					(i * 4 + 3) * cam->cam_w * 2;
				unsigned char *output_y = output_y_arg + i * output_w;
				unsigned char *output_u = output_u_arg + i * output_w;
				unsigned char *output_v = output_v_arg + i * output_w;

				for(j = 0; j < cam->working_w; j++)
				{
					*output_y++ = ((uint32_t)input_row1[0] + 
						(uint32_t)input_row1[2] + 
						(uint32_t)input_row1[4] + 
						(uint32_t)input_row1[6] + 
						(uint32_t)input_row2[0] +
						(uint32_t)input_row2[2] +
						(uint32_t)input_row2[4] +
						(uint32_t)input_row2[6] +
						(uint32_t)input_row3[0] + 
						(uint32_t)input_row3[2] + 
						(uint32_t)input_row3[4] + 
						(uint32_t)input_row3[6] + 
						(uint32_t)input_row4[0] +
						(uint32_t)input_row4[2] +
						(uint32_t)input_row4[4] +
						(uint32_t)input_row4[6]) / 16;
					*output_u++ = ((uint32_t)input_row1[1] + 
						(uint32_t)input_row2[1] +
						(uint32_t)input_row3[1] + 
						(uint32_t)input_row4[1] +
						(uint32_t)input_row1[5] + 
						(uint32_t)input_row2[5] +
						(uint32_t)input_row3[5] + 
						(uint32_t)input_row4[5]) / 8;
					*output_v++ = ((uint32_t)input_row1[3] +
						(uint32_t)input_row2[3] +
						(uint32_t)input_row3[3] +
						(uint32_t)input_row4[3] +
						(uint32_t)input_row1[7] +
						(uint32_t)input_row2[7] +
						(uint32_t)input_row3[7] +
						(uint32_t)input_row4[7]) / 8;

					input_row1 += 8;
					input_row2 += 8;
					input_row3 += 8;
					input_row4 += 8;
				}
			}
		}

		picture_size = cam->picture_size = buffer.bytesused;
	}
	else
	if(cam->v4l2_params.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
	{
printf("read_frame %d index=%d bytesused=%d\n", __LINE__, buffer.index, buffer.bytesused);
		memcpy (cam->picture_data, cam->frame_buffer[buffer.index], buffer.bytesused);
		picture_size = cam->picture_size = buffer.bytesused;
	}
	else
	{
		printf("read_frame %d: unknown picture format: %c%c%c%c\n", 
			__LINE__,
			((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[0],
			((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[1],
			((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[2],
			((unsigned char*)&cam->v4l2_params.fmt.pix.pixelformat)[3]);
	}


// Requeue buffer
	if(ioctl(cam->fd, VIDIOC_QBUF, &buffer) < 0)
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

	cam->total_frames++;


	return picture_size;

}



