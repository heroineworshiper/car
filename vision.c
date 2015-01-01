/*
 * VICACOPTER
 * Copyright (C) 2007-2013  Adam Williams <broadcast at earthling dot net>
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


// Vision position sensing in the dark, with a webcam
// Webcam drivers taken from Spca5xx Grabber & uvccapture



#include "settings.h"
#include "copter.h"
#include "arm/config.h"
#include "../flight/flight_math.h"
#include "util.h"

#include "vision.h"
#include "vision_common.h"
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




#ifdef USE_VISION



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
#define SATURATION 0
//#define SATURATION 34


#define GAIN 0
#endif // LOGITEC




#ifdef SUYIN
#define BRIGHTNESS 0
#define CONTRAST 32
#define SATURATION 32
#define SHARPNESS 0
#endif


// What image to show
// Only available with Video4linux 1
//#define SHOW_ORIGINAL


// Processing
//#define NORMALIZE



// Save unprocessed frames to files.  Only good with JPEG
//#define RECORD_INPUT
// Read JPEG from files instead of video device.  Disables RECORD_VIDEO
//#define PLAY_VIDEO
// Record the processed images.  Makes display synchronous.
//#define RECORD_OUTPUT
// Starting frame for playback
#define STARTING_FRAME 0
// Prefix of input files to record
#define RECORD_PATH "/tmp/vision"
// Prefix of output files to play
#define OUTPUT_PATH "/tmp/output"




// Pixels object needs to be off center for the turret to move
//#define SERVO_THRESHOLD (vision->image_h / 8)
#define SERVO_THRESHOLD 0

#define PEAK_SEARCH_RADIUS (vision->image_w * 10 / 100)
// At least as large as PEAK_SEARCH_RADIUS to compensate for peak glitches
#define BORDER_SEARCH_RADIUS (vision->image_w * 10 / 100)
// Depends on frame rate
#define TILT_FEEDBACK_X (settings->xtilt_feedback * 30 / settings->gps_hz)
#define TILT_FEEDBACK_Y (settings->ytilt_feedback * 30 / settings->gps_hz)
#define MAX_FEEDBACK 0.1


#define POSITION_BANDWIDTH 1.0
#define ATTITUDE_BANDWIDTH 1.0
#define MAX_ATTITUDE_CHANGE (TO_RAD(5.0) / settings->gps_hz)





// velocity delay
//#define V_HISTORY 1




#define CHROMA_W (vision->frame_w / 2)
#define CHROMA_H (vision->frame_h / 2)
#define SCAN_H (vision->mask_h - 1)
// size of a compressed buffer
#define MAX_COMPRESSED_SIZE (vision->image_w * vision->image_h * 2)

// Detect using blob detection instead of bottom right peak
//#define USE_BLOBS


// Amount / 10 to count as object luminance
#define BORDER_THRESHOLD 8



#ifdef MARCY2

// Rads per pixel (atan(1/2 object size in meters / distance to object) / 1/2 size of blob in mask pixels)
#define PIXEL_TO_ANGLE (atan((0.38 / 2) / 0.76) / (174.0 / 2))



#endif





// angle created by servo movement in degrees
#define TILT_TO_ANGLE_X 90.0
#define TILT_TO_ANGLE_Y 90.0

void process_frame();


void record_frame(vision_t *ptr, unsigned char *data, int size)
{
#ifdef RECORD_VIDEO
#ifndef RECORD_OUTPUT
#ifndef PLAY_VIDEO
	int i;
	char string[1024];
	sprintf(string, "%s%06d.jpg", RECORD_PATH, ptr->total_frames);
	FILE *out = fopen(string, "w");
	if(out)
	{
// Figure out the frame size by finding the last nonzero byte
// Only used for Video4linux 1
/*
 * 		for(i = size - 1; i >= 0; i--)
 * 		{
 * 			if(data[i] != 0)
 * 			{
 * 				size = i + 1;
 * 				break;
 * 			}
 * 		}
 */
		fwrite(data, size, 1, out);
		fclose(out);
	}
//	ptr->total_frames++;
#endif
#endif // !RECORD_OUTPUT
#endif
}





int read_file(vision_t *ptr)
{
	int i;
	char string[TEXTLEN];
	sprintf(string, "%s%06d.jpg", RECORD_PATH, ptr->left.total_frames);

	printf("read_file %d %d\n", __LINE__, ptr->left.total_frames);
	FILE *in = fopen(string, "r");
	int picture_size = 0;
	if(in)
	{
// Get the frame size
		fseek(in, 0, SEEK_END);
		picture_size = ftell(in);
		fseek(in, 0, SEEK_SET);
		
		int temp = fread(ptr->left.picture_data, picture_size, 1, in);
		fclose(in);
	}
	else
	{
// End of movie
		printf("read_file %d: end of movie\n", __LINE__);
		exit(1);
	}

	ptr->left.total_frames++;
	
	return picture_size;
}


#include "jpeglib.h"
#include <setjmp.h>


typedef struct 
{
	struct jpeg_source_mgr pub;	/* public fields */

	JOCTET * buffer;		/* start of buffer */
	int bytes;             /* total size of buffer */
} jpeg_source_mgr;
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

METHODDEF(boolean) fill_input_buffer(j_decompress_ptr cinfo)
{
	jpeg_src_ptr src = (jpeg_src_ptr) cinfo->src;
#define   M_EOI     0xd9

	src->buffer[0] = (JOCTET)0xFF;
	src->buffer[1] = (JOCTET)M_EOI;
	src->pub.next_input_byte = src->buffer;
	src->pub.bytes_in_buffer = 2;

	return TRUE;
}


METHODDEF(void) skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo->src;

	src->pub.next_input_byte += (size_t)num_bytes;
	src->pub.bytes_in_buffer -= (size_t)num_bytes;
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



// Decompress straight to YUV
void decompress_jpeg(vision_t *vision, 
	int picture_size,
	unsigned char *picture_data,
	unsigned char *y_buffer,
	unsigned char *u_buffer,
	unsigned char *v_buffer)
{
	int i, j;
	int w = vision->image_w;
	int h = vision->image_h;
	
	if(w == 640 && h == 240)
	{
		h = 480;
	}

	if(setjmp(my_jpeg_error.setjmp_buffer))
	{
/* If we get here, the JPEG code has signaled an error. */
		printf("decompress_jpeg %d: JPEG error\n", __LINE__);
		return;
	}


	struct jpeg_decompress_struct cinfo;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
	my_jpeg_error.pub.error_exit = my_jpeg_error_exit;
	jpeg_create_decompress(&cinfo);


// Create buffer source
	cinfo.src = (struct jpeg_source_mgr*)
    	(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
        JPOOL_PERMANENT,
		sizeof(jpeg_source_mgr));
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo.src;
	src->pub.init_source = init_source;
	src->pub.fill_input_buffer = fill_input_buffer;
	src->pub.skip_input_data = skip_input_data;
	src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
	src->pub.term_source = term_source;
	src->pub.bytes_in_buffer = picture_size;
	src->pub.next_input_byte = picture_data;
	src->buffer = picture_data;
	src->bytes = picture_size;
	jpeg_read_header(&cinfo, 1);
	cinfo.raw_data_out = TRUE;
// Stop the unexpected data warnings
	cinfo.err->num_warnings = 1;
	jpeg_start_decompress(&cinfo);

	unsigned char **mcu_rows[3];
	for(i = 0; i < 3; i++)
	{
		mcu_rows[i] = malloc(16 * sizeof(unsigned char*));
	}



	vision->min = 0xff;
	vision->max = 0x0;
	while(cinfo.output_scanline < h)
	{
		int scanline1 = cinfo.output_scanline;
		for(j = 0; j < 16; j++)
		{
			mcu_rows[0][j] = y_buffer +
				(cinfo.output_scanline + j) * w;
			mcu_rows[1][j] = u_buffer +
				((cinfo.output_scanline + j) / 2) * w / 2;
			mcu_rows[2][j] = v_buffer +
				((cinfo.output_scanline + j) / 2) * w / 2;
		}

		jpeg_read_raw_data(&cinfo, 
			mcu_rows, 
			h);

#ifdef NORMALIZE
		int scanline2 = cinfo.output_scanline;
// Get min & max
		for(i = scanline1; i < scanline2; i++)
		{
			unsigned char *row = y_buffer + i * w;
			for(j = 0; j < w; j++)
			{
				if(*row > vision->max) vision->max = *row;
				if(*row < vision->min) vision->min = *row;
				row++;
			}
		}
#endif

	}
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	for(i = 0; i < 3; i++)
	{
		free(mcu_rows[i]);
	}


//printf("decompress_jpeg %d ", __LINE__);
//for(i = 0; i < 32; i++) printf("%02x ", y_buffer[i]);
//printf("\n");
}




void record_output(vision_t *ptr)
{
#ifdef RECORD_OUTPUT
	int i, j;
	char string[TEXTLEN];
	sprintf(string, "%s%06d.jpg", OUTPUT_PATH, ptr->total_output_frames);
	ptr->total_output_frames++;
	FILE *fd = fopen(string, "w");
	int w = ptr->mask_w;
	int h = ptr->mask_h;
	int bytes_per_line = w;
	
	unsigned char *left_src = ptr->left.mask;
	unsigned char *right_src = ptr->right.mask;
	

//printf("record_output %d %s %p\n", __LINE__, string, fd);
	if(fd)
	{
//#ifdef MARCY1
#if 0
		struct jpeg_compress_struct jpeg_compress;
 		struct jpeg_error_mgr jerr;
		jpeg_compress.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&jpeg_compress);
		jpeg_compress.image_width = w;
		jpeg_compress.image_height = h;
		jpeg_compress.input_components = 3;
		jpeg_compress.in_color_space = JCS_RGB;
		jpeg_set_defaults(&jpeg_compress);
		jpeg_set_quality(&jpeg_compress, 80, 0);
		jpeg_compress.dct_method = JDCT_IFAST;
		jpeg_compress.raw_data_in = TRUE;
		jpeg_stdio_dest(&jpeg_compress, fd);


		unsigned char **mcu_rows[3];
		for(i = 0; i < 3; i++)
		{
			mcu_rows[i] = malloc(16 * sizeof(unsigned char*));

	// Discard UV data
			if(i > 0)
			{
				for(j = 0; j < 16; j++)
				{
					mcu_rows[i][j] = malloc(w);
					memset(mcu_rows[i][j], 0x80, w);
				}
			}
		}


		jpeg_start_compress(&jpeg_compress, TRUE);
		while(jpeg_compress.next_scanline < jpeg_compress.image_height)
		{
			for(j = 0; j < 16; j++)
			{
				mcu_rows[0][j] = src +
					(jpeg_compress.next_scanline + j) * w;
			}

			jpeg_write_raw_data(&jpeg_compress, 
				mcu_rows, 
				h);
		}
		jpeg_finish_compress(&jpeg_compress);


		jpeg_destroy((j_common_ptr)&jpeg_compress);
		for(i = 0; i < 3; i++)
		{
			if(i > 0)
			{
				for(j = 0; j < 16; j++)
					free(mcu_rows[i][j]);
			}

			free(mcu_rows[i]);
		}
#endif // MARCY1

#ifdef MARCY2
		struct jpeg_compress_struct jpeg_compress;
 		struct jpeg_error_mgr jerr;
		jpeg_compress.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&jpeg_compress);
		jpeg_compress.image_width = w;
		jpeg_compress.image_height = h;
		jpeg_compress.input_components = 3;
		jpeg_compress.in_color_space = JCS_RGB;
		jpeg_set_defaults(&jpeg_compress);
		jpeg_set_quality(&jpeg_compress, 100, 0);
		jpeg_compress.dct_method = JDCT_IFAST;
		jpeg_stdio_dest(&jpeg_compress, fd);


		unsigned char *rows[1];
		rows[0] = malloc(w * 3);
		jpeg_start_compress(&jpeg_compress, TRUE);
		while(jpeg_compress.next_scanline < jpeg_compress.image_height)
		{
			unsigned char *src_row = src + jpeg_compress.next_scanline * bytes_per_line;
			for(i = 0; i < w; i++)
			{
				rows[0][i * 3] = src_row[i * 4 + 2];
				rows[0][i * 3 + 1] = src_row[i * 4 + 1];
				rows[0][i * 3 + 2] = src_row[i * 4 + 0];
			}
			jpeg_write_scanlines(&jpeg_compress, rows, 1);
		}
		jpeg_finish_compress(&jpeg_compress);
		jpeg_destroy((j_common_ptr)&jpeg_compress);
		free(rows[0]);
#endif // MARCY2


#if defined(MARCY3) || defined(MARCY1)



		struct jpeg_compress_struct jpeg_compress;
 		struct jpeg_error_mgr jerr;
		jpeg_compress.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&jpeg_compress);
		jpeg_compress.image_width = w;
		jpeg_compress.image_height = h;
		jpeg_compress.input_components = 3;
		jpeg_compress.in_color_space = JCS_RGB;
		jpeg_set_defaults(&jpeg_compress);
		jpeg_set_quality(&jpeg_compress, 100, 0);
		jpeg_compress.dct_method = JDCT_IFAST;
		jpeg_stdio_dest(&jpeg_compress, fd);


		unsigned char *rows[1];
		rows[0] = malloc(w * 3);
		jpeg_start_compress(&jpeg_compress, TRUE);
		while(jpeg_compress.next_scanline < jpeg_compress.image_height)
		{
			unsigned char *left_row = ptr->left.mask + jpeg_compress.next_scanline * bytes_per_line;
			unsigned char *right_row = ptr->right.mask + jpeg_compress.next_scanline * bytes_per_line;
			for(i = 0; i < w; i++)
			{
				rows[0][i * 3 + 0] = left_row[i] ? 0xff : 0;
				rows[0][i * 3 + 1] = right_row[i] ? 0xff : 0;
				rows[0][i * 3 + 2] = right_row[i] ? 0xff : 0;
			}
			jpeg_write_scanlines(&jpeg_compress, rows, 1);
		}
		jpeg_finish_compress(&jpeg_compress);
		jpeg_destroy((j_common_ptr)&jpeg_compress);
		free(rows[0]);




#endif // defined(MARCY3) || defined(MARCY1)

		fclose(fd);
	}
	
#endif // RECORD_OUTPUT
	
}












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





int init_input(vision_t *vision, char *path, unsigned char **frame_buffer)
{
	int i;
	int error = 0;
	int fd;
	int w = vision->image_w;
	int h = vision->image_h;
	settings_t *settings = &copter.settings;
	
	if(w == 640 && h == 240)
	{
		h = 480;
	}

#ifndef PLAY_VIDEO
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
/*
 *    			printf("init_input %d control=0x%x %s value=%d\n", 
 *    				__LINE__, 
 *    				i, 
 *    				arg.name,
 *    				arg.default_value);
 */
   		}
   	}
	
// Camera class conrol
   	for(i = V4L2_CID_CAMERA_CLASS_BASE; i < V4L2_CID_CAMERA_CLASS_BASE + 32; i++)
   	{
   		bzero(&arg, sizeof(arg));
   		arg.id = i;
   		if(!ioctl(fd, VIDIOC_QUERYCTRL, &arg))
   		{
/*
 *    			printf("init_input %d control=0x%x %s value=%d\n", 
 *    				__LINE__, 
 *    				i, 
 *    				arg.name,
 *    				arg.default_value);
 */
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
//#if (ROTORS == 1 && SERVOS == 3)
	ctrl_arg.id = V4L2_CID_GAIN;
	ctrl_arg.value = GAIN;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_GAIN");
	}

	ctrl_arg.id = V4L2_CID_EXPOSURE_AUTO;
	ctrl_arg.value = 1;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_EXPOSURE_AUTO");
	}

	ctrl_arg.id = V4L2_CID_EXPOSURE_ABSOLUTE;
// floppy disk exposure
//	ctrl_arg.value = 512;
// Edmond optical exposure
	ctrl_arg.value = settings->shutter_speed_code;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl_arg) < 0)
	{
		perror("init_input VIDIOC_S_CTRL V4L2_CID_EXPOSURE_ABSOLUTE");
	}

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
//#endif // MARCY3


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
	struct v4l2_format v4l2_params;
	v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(fd, VIDIOC_G_FMT, &v4l2_params) < 0)
		perror("init_input VIDIOC_G_FMT");
	v4l2_params.fmt.pix.width = w;
	v4l2_params.fmt.pix.height = h;


// Probe the compression format
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

// Show formats
#if 0
   	for(i = 0; i < sizeof(pixel_formats) / sizeof(int); i++)
   	{
   		v4l2_params.fmt.pix.pixelformat = pixel_formats[i];
   		if(ioctl(fd, VIDIOC_S_FMT, &v4l2_params) >= 0)
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
#endif


#if defined(LOGITEC) || defined(ZSTAR)
	v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
#ifdef SUYIN
	v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif
	if(ioctl(fd, VIDIOC_S_FMT, &v4l2_params) < 0)
		perror("init_input VIDIOC_S_FMT");
	if(ioctl(fd, VIDIOC_G_FMT, &v4l2_params) < 0)
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
 			vision->buffer_size = buffer.length;
			frame_buffer[i] = (unsigned char*)mmap(NULL,
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

#endif // !PLAY_VIDEO

	return fd;
}


void close_input(int fd, unsigned char **frame_buffer)
{
	vision_t *vision = &copter.vision;
	int i;

	for(i = 0; i < DEVICE_BUFFERS; i++)
	{
		munmap(frame_buffer[i], vision->buffer_size);
	}

	int streamoff_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(fd, VIDIOC_STREAMOFF, &streamoff_arg) < 0)
		perror("close_input VIDIOC_STREAMOFF");
	close(fd);
}




int read_frame(vision_t *vision, 
	unsigned char *picture_data, 
	int fd, 
	unsigned char **frame_buffer,
	unsigned char *y_buffer,
	unsigned char *u_buffer,
	unsigned char *v_buffer)
{
	int i, j;

	int picture_size = 0;

#ifndef PLAY_VIDEO
	struct v4l2_buffer buffer;
	bzero(&buffer, sizeof(buffer));
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	if(ioctl(fd, VIDIOC_DQBUF, &buffer) < 0)
	{
		perror("read_frame VIDIOC_DQBUF");
		return;
	}

//printf("read_frame %d index=%d bytesused=%d\n", __LINE__, buffer.index, buffer.bytesused);

#ifdef LOGITEC
// Massage into a JPEG image
    memcpy (picture_data, frame_buffer[buffer.index], HEADERFRAME1);
    memcpy (picture_data + HEADERFRAME1, dht_data, DHT_SIZE);
    memcpy (picture_data + HEADERFRAME1 + DHT_SIZE,
	    frame_buffer[buffer.index] + HEADERFRAME1,
	    (buffer.bytesused - HEADERFRAME1));
	picture_size = buffer.bytesused + DHT_SIZE;
//printf("read_frame %d\n", __LINE__);
#endif // LOGITEC

#ifdef ZSTAR
	memcpy (picture_data, frame_buffer[buffer.index], buffer.bytesused);
	picture_size = buffer.bytesused;
#endif // ZSTAR

#ifdef SUYIN

// Capture YUYV

	for(i = 0; i < vision->image_h; i++)
	{
		unsigned char *input_row = frame_buffer[buffer.index] +
			i * w * 2;
		unsigned char *output_y = y_buffer + i * w;
		unsigned char *output_u = u_buffer + (i / 2) * (w / 2);
		unsigned char *output_v = v_buffer + (i / 2) * (w / 2);

		for(j = 0; j < w / 2; j++)
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


#endif // SUYIN


// Requeue buffer
	if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
	{
		perror("read_frame VIDIOC_QBUF");
	}

#else // !PLAY_VIDEO


	picture_size = read_file(vision);

#endif // PLAY_VIDEO

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

#if defined(LOGITEC) || defined(ZSTAR)
	decompress_jpeg(vision, 
		picture_size, 
		picture_data,
		y_buffer,
		u_buffer,
		v_buffer);
#endif

	if(fd == vision->left.fd) vision->left.total_frames++;
	else
	if(fd == vision->right.fd) vision->right.total_frames++;


	return picture_size;

}


void write_vision(unsigned char *data, int size, int is_left)
{
	vision_t *vision = &copter.vision;
	unsigned char *image_data = data + SEGMENT_HEADER_SIZE;
	int bytes_read = size - SEGMENT_HEADER_SIZE;
	int end_of_frame = 0;
	int i;
//printf("write_vision %d %d\n", __LINE__, bytes_read);

// Test for end of frame code
	int codes_read = bytes_read / 2;
	uint16_t *ptr = (uint16_t*)image_data;
	for(i = 0; i < codes_read; i++)
	{
		if(*ptr == 0xffff)
		{
			end_of_frame = 1;
			bytes_read = (i + 1) * 2;
			break;
		}
		ptr++;
	}

// Append to buffer
	if(is_left)
	{
		if(vision->left.picture_size + bytes_read <= MAX_COMPRESSED_SIZE)
		{
			memcpy(vision->left.picture_data + vision->left.picture_size,
				image_data,
				bytes_read);
			vision->left.picture_size += bytes_read;
		}
		else
		{
			end_of_frame = 1;
		}
	}
	else
	{
		if(vision->right.picture_size + bytes_read <= MAX_COMPRESSED_SIZE)
		{
			memcpy(vision->right.picture_data + vision->right.picture_size,
				image_data,
				bytes_read);
			vision->right.picture_size += bytes_read;
//printf("write_vision %d %d\n", __LINE__, bytes_read);
		}
		else
		{
			end_of_frame = 1;
		}
	}


	if(end_of_frame)
	{
// decompress it
		uint16_t *ptr ;
		uint16_t *buffer_end;
		unsigned char *pixel;
		
		if(is_left)
		{
			ptr = (uint16_t*)(vision->left.picture_data + 4);
			buffer_end = ptr + (vision->left.picture_size - 4) / 2;
			pixel = vision->left.mask;
		}
		else
		{
			ptr = (uint16_t*)(vision->right.picture_data + 4);
			buffer_end = ptr + (vision->right.picture_size - 4) / 2;
			pixel = vision->right.mask;
		}
		
		unsigned char *pixel_end = pixel + vision->mask_w * vision->mask_h;
		int i;


/*
 * printf("write_frame %d %d\n", __LINE__, vision->picture_size);
 * for(i = 0; i < (vision->picture_size - 4) / 2; i++)
 * printf("%04x ", ptr[i]);
 * printf("\n");
 */



		while(ptr < buffer_end && pixel < pixel_end)
		{
			if(*ptr == 0xffff)
				break;

/* set 0's */
			if(pixel + *ptr <= pixel_end)
				memset(pixel, 0x0, *ptr);
			pixel += *ptr;
			ptr++;

/* set 1's */
			if(pixel + *ptr <= pixel_end)
				memset(pixel, MASK_VALUE, *ptr);
			pixel += *ptr;
			ptr++;
		}

 		if(pixel != pixel_end || (ptr + 1) != buffer_end)
 		{
 			printf("write_vision %d %d bad frame %d %d\n", 
 				__LINE__,
				is_left,
 				(int)(pixel_end - pixel),
 				(int)(buffer_end - ptr - 1));
 		}
		else
		{
			if(is_left)
			{
				vision->left.total_frames++;
				vision->got_left = 1;
			}
			else
			{
				vision->right.total_frames++;
				vision->got_right = 1;
			}
			process_frame();
		}




		if(is_left)
		{
			vision->left.picture_size = 0;
		}
		else
		{
			vision->right.picture_size = 0;
		}
	}
}


static void handle_sync_code(unsigned char data);
static void handle_packet(unsigned char data)
{
	vision_t *vision = &copter.vision;
	vision->left_buffer[vision->left_bytes] = data;
	vision->left_bytes++;

	if(vision->left_bytes >= PICTURE_FRAGMENT_SIZE + SEGMENT_HEADER_SIZE)
	{
//printf("handle_packet %d\n", __LINE__);
		write_vision(vision->left_buffer, 
			PICTURE_FRAGMENT_SIZE + SEGMENT_HEADER_SIZE, 
			1);
		vision->current_function = handle_sync_code;
		vision->left_bytes = 0;
	}
}

static void handle_packet_type(unsigned char data)
{
	vision_t *vision = &copter.vision;
	int packet_type = (data & 0xf0);
	if(packet_type == PACKET_VIDEO)
	{
		vision->left_buffer[1] = data;
		vision->left_bytes++;
		vision->current_function = handle_packet;
	}
	else
	{
		vision->current_function = handle_sync_code;
		vision->left_bytes = 0;
	}
}

static void handle_sync_code(unsigned char data)
{
	if(data == SYNC_CODE)
	{
		vision_t *vision = &copter.vision;
		vision->current_function = handle_packet_type;
		vision->left_buffer[0] = SYNC_CODE;
		vision->left_bytes = 1;
//printf("handle_sync_code %d\n", __LINE__);
	}
}

void handle_left_byte(unsigned char data)
{
	if(copter.vision.current_function != 0)
		copter.vision.current_function(data);
}





// Get distance from x1, y1 to max value on line
// Search from outside in
int raytrace(vision_t *ptr, 
	unsigned char *buffer, 
	int x1, 
	int y1, 
	int x2, 
	int y2)
{
	int x, y, step;
	int max = 0;
	int max_x, max_y;
	
	if(abs(y2 - y1) < abs(x2 - x1))
	{
//printf("raytrace %d %d\n", __LINE__, x2 - x1);
		if(x2 > x1) 
			step = 1;
		else
			step = -1;
		for(x = x2; x != x1; x -= step)
		{
			y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
			if(x >= 0 &&
				x < ptr->image_w &&
				y >= 0 &&
				y < ptr->image_w)
			{
//				buffer[y * ptr->image_w + x] = 0xff;
				if(buffer[y * ptr->image_w + x] > max)
				{
					max = buffer[y * ptr->image_w + x];
					max_x = x;
					max_y = y;
				}
			}
		}
	}
	else
	{
//printf("raytrace %d %d\n", __LINE__, y2 - y1);
		if(y2 > y1)
			step = 1;
		else
			step = -1;
		for(y = y2; y != y1; y -= step)
		{
			x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
			if(x >= 0 &&
				x < ptr->image_w &&
				y >= 0 &&
				y < ptr->image_w)
			{
//				buffer[y * ptr->image_w + x] = 0xff;
				if(buffer[y * ptr->image_w + x] > max)
				{
					max = buffer[y * ptr->image_w + x];
					max_x = x;
					max_y = y;
				}
			}
		}
	}
	
	return sqrt(SQR(max_x - x1) + SQR(max_y - y1));
}


#if 0
// Get maximum on line
int test_line(vision_t *ptr, 
	unsigned char *buffer, 
	int x1, 
	int y1, 
	int x2, 
	int y2,
	int *total)
{
	int x, y, step;
	int max = 0;
	const int debug = 0;
	
	if(abs(y2 - y1) <= abs(x2 - x1))
	{
		if(x2 > x1) 
			step = 1;
		else
			step = -1;
		for(x = x2; x != x1; x -= step)
		{
			if(x >= 0 && x < ptr->mask_w)
			{
				y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
				if(y >= 0 &&
					y < ptr->mask_h)
				{
					if(debug) buffer[y * ptr->mask_w + x] ^= 0xff;
					*total += ptr->accum[y * ptr->mask_w + x];
					if(ptr->accum[y * ptr->mask_w + x] > max)
					{
						max = ptr->accum[y * ptr->mask_w + x];
					}
				}
			}
		}
	}
	else
	{
		if(y2 > y1)
			step = 1;
		else
			step = -1;
		for(y = y2; y != y1; y -= step)
		{
			if(y >= 0 && y < ptr->mask_h)
			{
				x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
				if(x >= 0 &&
					x < ptr->mask_w)
				{
					if(debug) buffer[y * ptr->mask_w + x] ^= 0xff;
					*total += ptr->accum[y * ptr->mask_w + x];
					if(ptr->accum[y * ptr->mask_w + x] > max)
					{
						max = ptr->accum[y * ptr->mask_w + x];
					}
				}
			}
		}
	}
	
	return max;
}



#define TEST_RAY \
	unsigned char *value = &vision->accum[y * vision->mask_w + x]; \
	if(!got_red) \
	{ \
		if(*value == RMASK_VALUE) \
		{ \
			got_red = 1; \
		} \
	} \
	else \
	if(*value != RMASK_VALUE) \
	{ \
		*value = MASK_VALUE; \
		return; \
	}


// Get transition from red to blue or blank
void test_line2(vision_t *vision, 
	int x2, 
	int y2, 
	int x1, 
	int y1)
{
	int x, y, step;
	int got_red = 0;
	const int debug = 0;
	
	if(abs(y2 - y1) <= abs(x2 - x1))
	{
		if(x2 > x1) 
			step = 1;
		else
			step = -1;
		for(x = x2; x != x1; x -= step)
		{
			if(x >= 0 && x < vision->mask_w)
			{
				y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
				if(y >= 0 &&
					y < vision->mask_h)
				{
					TEST_RAY
				}
			}
		}
	}
	else
	{
		if(y2 > y1)
			step = 1;
		else
			step = -1;
		for(y = y2; y != y1; y -= step)
		{
			if(y >= 0 && y < vision->mask_h)
			{
				x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
				if(x >= 0 &&
					x < vision->mask_w)
				{
					TEST_RAY
				}
			}
		}
	}
}






// Find boundaries of brightest object
void brightness_tracking(vision_t *vision)
{
	int i, j;
// Brightest part of image
	int peak = 240;
	int peak_x = vision->peak_x;
	int peak_y = vision->peak_y;
	int peak_x1 = vision->peak_x;
	int peak_y1 = vision->peak_y;
	int peak_x2 = vision->peak_x;
	int peak_y2 = vision->peak_y;
	int top_border = 0;
	int bottom_border = 0;
	int left_border = 0;
	int right_border = 0;

	int x1 = 0;
	int x2 = 0;
	int y1 = 0;
	int y2 = 0;



#ifndef USE_BLOBS
// 	x1 = vision->peak_x - PEAK_SEARCH_RADIUS;
// 	x2 = vision->peak_x + PEAK_SEARCH_RADIUS;
// 	y1 = vision->peak_y - PEAK_SEARCH_RADIUS;
// 	y2 = vision->peak_y + PEAK_SEARCH_RADIUS;
	x1 = 0;
	x2 = vision->image_w;
	y1 = 0;
	y2 = vision->image_h;


// Make search radius at least as large as object
	if(x1 > vision->peak_x + vision->left_border)
		x1 = vision->peak_x + vision->left_border;
	if(y1 > vision->peak_y + vision->top_border)
		y1 = vision->peak_y + vision->top_border;
	if(x2 < vision->peak_x + vision->right_border)
		x2 = vision->peak_x + vision->right_border;
	if(y2 < vision->peak_y + vision->bottom_border)
		y2 = vision->peak_y + vision->bottom_border;
	if(x1 <  0) x1 = 0;
	if(x2 > vision->image_w) x2 = vision->image_w;
	if(y1 <  0) y1 = 0;
	if(y2 > vision->image_h) y2 = vision->image_h;
	for(i = y1; i < y2; i++)
	{
		unsigned char *row = vision->accum + i * vision->image_w;
		for(j = x1; j < x2; j++)
		{
			if(row[j] > peak)
			{
				peak_x1 = j;
				peak_y1 = i;
				peak_x2 = j;
				peak_y2 = i;
				peak = row[j];
			}
			else
			if(row[j] == peak)
			{
				peak_x2 = j;
				peak_y2 = i;
			}
		}
	}

	peak_x = (peak_x1 + peak_x2) / 2;
	peak_y = (peak_y1 + peak_y2) / 2;

//printf("brightness_tracking %d peak=%d\n", __LINE__, peak);

// Scan for dropoff
// right border
	x1 = peak_x + vision->right_border - BORDER_SEARCH_RADIUS;
	x2 = peak_x + vision->right_border + BORDER_SEARCH_RADIUS;
	y1 = peak_y + vision->top_border - BORDER_SEARCH_RADIUS;
	y2 = peak_y + vision->bottom_border + BORDER_SEARCH_RADIUS;
	if(x1 <= peak_x) x1 = peak_x + 1;
	if(x2 > vision->image_w) x2 = vision->image_w;
	if(y1 < 0) y1 = 0;
	if(y2 > vision->image_h) y2 = vision->image_h;
	right_border = x1 - peak_x;
	for(i = x1; i < x2; i++)
	{
		int max = 0;
		for(j = y1; j < y2; j++)
		{
			if(vision->accum[j * vision->image_w + i] > max) 
				max = vision->accum[j * vision->image_w + i];
		}

		if(max >= peak * BORDER_THRESHOLD / 10)
		{
			right_border = i - peak_x;
		}
		else
			break;
	}

// left border
	x1 = peak_x + vision->left_border - BORDER_SEARCH_RADIUS;
	x2 = peak_x + vision->left_border + BORDER_SEARCH_RADIUS;
	y1 = peak_y + vision->top_border - BORDER_SEARCH_RADIUS;
	y2 = peak_y + vision->bottom_border + BORDER_SEARCH_RADIUS;
	if(x1 < 0) x1 = 0;
	if(x2 >= peak_x) x2 = peak_x - 1;
	if(y1 < 0) y1 = 0;
	if(y2 > vision->image_h) y2 = vision->image_h;
	left_border = x2 - peak_x;
	for(i = x2; i >= x1; i--)
	{
		int max = 0;
		for(j = y1; j < y2; j++)
		{
			if(vision->accum[j * vision->image_w + i] > max) 
				max = vision->accum[j * vision->image_w + i];
		}

		if(max >= peak * BORDER_THRESHOLD / 10)
		{
			left_border = i - peak_x;
		}
		else
			break;
	}

// top border
	x1 = peak_x + vision->left_border - BORDER_SEARCH_RADIUS;
	x2 = peak_x + vision->right_border + BORDER_SEARCH_RADIUS;
	y1 = peak_y + vision->top_border - BORDER_SEARCH_RADIUS;
	y2 = peak_y + vision->top_border + BORDER_SEARCH_RADIUS;
	if(x1 < 0) x1 = 0;
	if(x2 > vision->image_w) x2 = vision->image_w;
	if(y1 < 0) y1 = 0;
	if(y2 >= peak_y) y2 = peak_y - 1;
	top_border = y2 - peak_y;
	for(i = y2; i >= y1; i--)
	{
		int max = 0;
		for(j = x1; j < x2; j++)
		{
			if(vision->accum[i * vision->image_w + j] > max) 
				max = vision->accum[i * vision->image_w + j];
		}

		if(max >= peak * BORDER_THRESHOLD / 10)
		{
			top_border = i - peak_y;
		}
		else
			break;
	}

// bottom border
	x1 = peak_x + vision->left_border - BORDER_SEARCH_RADIUS;
	x2 = peak_x + vision->right_border + BORDER_SEARCH_RADIUS;
	y1 = peak_y + vision->bottom_border - BORDER_SEARCH_RADIUS;
	y2 = peak_y + vision->bottom_border + BORDER_SEARCH_RADIUS;
	if(x1 < 0) x1 = 0;
	if(x2 > vision->image_w) x2 = vision->image_w;
	if(y1 <= peak_y) y1 = peak_y + 1;
	if(y2 > vision->image_h) y2 = vision->image_h;
	bottom_border = y1 - peak_y;
	for(i = y1; i < y2; i++)
	{
		int max = 0;
		for(j = x1; j < x2; j++)
		{
			if(vision->accum[i * vision->image_w + j] > max)
				max = vision->accum[i * vision->image_w + j];
		}

		if(max >= peak * BORDER_THRESHOLD / 10)
		{
			bottom_border = i - peak_y;
		}
		else
			break;
	}

#else // !USE_BLOBS

// Find brightest point in image
	for(i = 0; i < vision->image_h; i++)
	{
		unsigned char *row = vision->accum + i * vision->image_w;
		for(j = 0; j < vision->image_w; j++)
		{
			if(row[j] > peak)
			{
				peak = row[j];
			}
		}
	}

// Mask all brightest points
	int threshold = peak * BORDER_THRESHOLD / 10;
	for(i = 0; i < vision->image_h; i++)
	{
		unsigned char *in_row = vision->accum + i * vision->image_w;
		unsigned char *out_row = vision->mask + i * vision->image_w;
		for(j = 0; j < vision->image_w; j++)
		{
			if(in_row[j] >= threshold)
			{
				*out_row = MASK_VALUE;
			}
			else
			{
				*out_row = 0;
			}

			out_row++;
		}
	}

printf("brightness_tracking %d\n", __LINE__);
// Find all blobs
	reset_blobs(&vision->blobs);
	for(i = 0; i < SCAN_H; i++)
	{
		unsigned char *mask_row = vision->mask + i * vision->mask_w;

		for(j = 0; j < vision->mask_w; j++)
		{
// got a blob.  Count the number of contiguous pixels.
			if(*mask_row == MASK_VALUE)
			{
				blob_t *blob = vision->blobs.blobs + vision->blobs.total_blobs;
				flood_fill(vision->mask, 
					&vision->blobs, 
					blob, 
					j, 
					i,
					MASK_VALUE);
				process_blob(vision->mask, 
					&vision->blobs, 
					blob, 
					MASK_PREV_BLOB);
				if(vision->blobs.total_blobs < MAX_BLOBS - 1) 
					vision->blobs.total_blobs++;
			}
			
			mask_row++;
		}
	}
	
printf("brightness_tracking %d %d\n", __LINE__, vision->blobs.total_blobs);


// take dimensions of largest blob
	if(vision->blobs.max_blob)
	{
		blob_t *blob = vision->blobs.max_blob;
		peak_x = (blob->x1 + blob->x2) / 2;
		peak_y = (blob->y1 + blob->y2) / 2;
		top_border = blob->y1 - peak_y;
		bottom_border = blob->y2 - peak_y;
		left_border = blob->x1 - peak_x;
		right_border = blob->x2 - peak_x;
	}


#endif // USE_BLOBS





	vision->peak_x = peak_x;
	vision->peak_y = peak_y;
	vision->top_border = top_border;
	vision->bottom_border = bottom_border;
	vision->left_border = left_border;
	vision->right_border = right_border;

	vision->center_x_main = ((peak_x + right_border) + (peak_x + left_border)) / 2;
	vision->center_y_main = ((peak_y + bottom_border) + (peak_y + top_border)) / 2;


// Find brightest point on left side
	int test_column = peak_x + left_border;
	y1 = peak_y + top_border;
	y2 = peak_y + bottom_border;
	CLAMP(test_column, 0, vision->image_w - 1);
	CLAMP(y1, 0, vision->image_h - 1);
	CLAMP(y2, 0, vision->image_h - 1);

	int max = 0;
	int peak1 = 0;
	int peak2 = 0;
	for(i = y1; i < y2; i++)
	{
		if(vision->accum[i * vision->image_w + test_column] > max)
		{
			max = vision->accum[i * vision->image_w + test_column];
			peak1 = i - peak_y;
		}
	}

	max = 0;
	for(i = y2 - 1; i >= 0; i--)
	{
		if(vision->accum[i * vision->image_w + test_column] > max)
		{
			max = vision->accum[i * vision->image_w + test_column];
			peak2 = i - peak_y;
		}
	}
	vision->left_peak = (peak1 + peak2) / 2;

// Find brightest point on right side

	test_column = peak_x + right_border;
	CLAMP(test_column, 0, vision->image_w - 1);
	max = 0;
	peak1 = 0;
	peak2 = 0;
	for(i = y1; i < y2; i++)
	{
		if(vision->accum[i * vision->image_w + test_column] > max)
		{
			max = vision->accum[i * vision->image_w + test_column];
			peak1 = i - peak_y;
		}
	}

	max = 0;
	for(i = y2 - 1; i >= 0; i--)
	{
		if(vision->accum[i * vision->image_w + test_column] > max)
		{
			max = vision->accum[i * vision->image_w + test_column];
			peak2 = i - peak_y;
		}
	}
	vision->right_peak = (peak1 + peak2) / 2;

//return;

//	unsigned char *dst = vision->y_buffer;
	unsigned char *dst = vision->accum;

// Get minor axis by sliding major axis up & down
	int left_y = peak_y + vision->left_peak;
	int right_y = peak_y + vision->right_peak;
	int test_distance = MAX(vision->left_peak - vision->top_border,
		vision->right_peak - vision->top_border);
	int top_total = 0, bottom_total = 0;
	max = 0;
	vision->top_peak = 0;

/*
 * printf("brightness_tracking %d %d %d %d %d\n", 
 * __LINE__, 
 * peak_x, 
 * peak_y,
 * vision->left_peak,
 * vision->right_peak);
 */

	for(i = test_distance; i >= 0; i--)
	{
		int value = test_line(vision, 
			dst, 
			peak_x + left_border, 
			peak_y + vision->left_peak - i,
			peak_x + right_border,
			peak_y + vision->right_peak - i,
			&top_total);
		if(value > max)
		{
			max = value;
			vision->top_peak = -i;
		}
	}

	test_distance = MAX(vision->bottom_border - vision->left_peak,
		vision->bottom_border - vision->right_peak);
	max = 0;
	vision->bottom_peak = 0;
	for(i = test_distance; i >= 0; i--)
	{
		int value = test_line(vision, 
			dst, 
			peak_x + left_border, 
			peak_y + vision->left_peak + i,
			peak_x + right_border,
			peak_y + vision->right_peak + i,
			&bottom_total);
		if(value > max)
		{
			max = value;
			vision->bottom_peak = i;
		}
	}

	vision->closer = 0;
	if(bottom_total > top_total) vision->closer = 1;







// Draw border on output
	draw_rect(dst, 
		peak_x + left_border, 
		peak_y + top_border, 
		peak_x + right_border, 
		peak_y + bottom_border,
		vision->image_w,
		vision->image_h,
		1);

// Peaks on borders
	draw_line(dst, 
		peak_x + vision->left_border, 
		peak_y + vision->left_peak, 
		peak_x + vision->right_border, 
		peak_y + vision->right_peak,
		vision->image_w,
		vision->image_h,
		1);


// Top & bottom
	draw_line(dst, 
		peak_x + vision->left_border, 
		peak_y + vision->left_peak + vision->top_peak, 
		peak_x + vision->right_border, 
		peak_y + vision->right_peak + vision->top_peak,
		vision->image_w,
		vision->image_h,
		1);

	if(!vision->closer)
	{
		draw_line(dst, 
			peak_x + vision->left_border, 
			peak_y + vision->left_peak + vision->top_peak + 1, 
			peak_x + vision->right_border, 
			peak_y + vision->right_peak + vision->top_peak + 1,
			vision->image_w,
			vision->image_h,
		1);
	}
	else
	{
		draw_line(dst, 
			peak_x + vision->left_border, 
			peak_y + vision->left_peak + vision->bottom_peak + 1, 
			peak_x + vision->right_border, 
			peak_y + vision->right_peak + vision->bottom_peak + 1,
			vision->image_w,
			vision->image_h,
		1);
	}
	
	draw_line(dst, 
		peak_x + vision->left_border, 
		peak_y + vision->left_peak + vision->bottom_peak, 
		peak_x + vision->right_border, 
		peak_y + vision->right_peak + vision->bottom_peak,
		vision->image_w,
		vision->image_h,
		1);


/*
 * 	draw_line(vision, 
 * 		dst, 
 * 		peak_x + vision->top_peak, 
 * 		peak_y + vision->top_border, 
 * 		peak_x + vision->bottom_peak, 
 * 		peak_y + vision->bottom_border);
 */

		
/*
 * 	draw_line(vision, 
 * 		dst, 
 * 		peak_x + vision->left_border - 5, 
 * 		peak_y + vision->left_peak, 
 * 		peak_x + vision->left_border + 5, 
 * 		peak_y + vision->left_peak);
 * 
 * 	draw_line(vision, 
 * 		dst, 
 * 		peak_x + vision->right_border - 5, 
 * 		peak_y + vision->right_peak, 
 * 		peak_x + vision->right_border + 5, 
 * 		peak_y + vision->right_peak);
 */

// Draw brightest part of image
// 	draw_cross(vision, 
// 		dst,
// 		peak_x,
// 		peak_y);

// Draw center
/*
 * 	draw_cross(vision, 
 * 		dst,
 * 		vision->center_x_main,
 * 		vision->center_y_main);
 */

#ifdef TEST_VISION
//	getc(stdin);
#endif
}




void difference(vision_t *ptr)
{
// Brightness is difference of all history
	int i, j;
	int image_size = ptr->image_w * ptr->image_h;
	settings_t *settings = &copter.settings;
	for(i = 0; i < image_size; i++)
	{
		unsigned char *dst = ptr->accum + i;
		int diff = 0;
		for(j = 1; j < settings->accum_size; j++)
		{
#ifndef USE_COLOR
			diff += abs(ptr->history[j][i] - ptr->history[j - 1][i]);
#endif
		}
		
		diff *= 1;
		if(diff > 0xff) diff = 0xff;
		ptr->accum[i] = diff;
	}
}

void difference_key(vision_t *ptr)
{
// Pass through only what changes
// Accumulate brightest
	int i, j;
	int image_size = ptr->image_w * ptr->image_h;
	const int threshold = 20 * 255 / 100;
	settings_t *settings = &copter.settings;

	bzero(ptr->accum, image_size);
	unsigned char *accum = ptr->accum;

//printf("difference_key %d\n", __LINE__);
	for(i = 0; i < image_size; i++, accum++)
	{
		for(j = 1; j < settings->accum_size; j++)
		{
			int diff = abs(ptr->history[j][i] - ptr->history[j - 1][i]);
			if(diff > threshold)
			{
				if(ptr->history[j][i] > *accum)
					*accum = ptr->history[j][i];
			}
			
		}
	}
//printf("difference_key %d\n", __LINE__);
}





void accumulate(vision_t *ptr)
{
// Get max of all history & calculate accum buffer
	int i, j, k;
	settings_t *settings = &copter.settings;
	
#ifndef USE_COLOR
	int image_size = ptr->image_w * ptr->image_h;
#else
	int image_size = ptr->image_w * ptr->image_h * 3;
#endif


	bzero(ptr->accum, image_size);
	for(i = 0; i < settings->accum_size; i++)
	{
#ifndef USE_COLOR

		int image_size = ptr->image_w * ptr->image_h;
		unsigned char *history = ptr->history[i];
		unsigned char *accum = ptr->accum;
		for(j = 0; j < image_size; j++)
		{
			if(*history > *accum)
				*accum = *history;
			accum++;
			history++;
		}


#endif // !USE_COLOR
	}

}




// Store frame in history
void store_history(vision_t *ptr)
{
	int i, j, k;
	settings_t *settings = &copter.settings;
// convert to mask & store
	





#ifndef USE_COLOR








// Store in history
	memcpy(ptr->history[ptr->history_slot], 
		ptr->y_buffer, 
		ptr->image_w * ptr->image_h);










#else // !USE_COLOR


// YUV -> RGB
	for(i = 0; i < ptr->image_h; i++)
	{
		unsigned char *y_row = ptr->y_buffer + i * ptr->image_w;
		unsigned char *u_row = ptr->u_buffer + i / 2 * ptr->image_w / 2;
		unsigned char *v_row = ptr->v_buffer + i / 2 * ptr->image_w / 2;
		unsigned char *dst = ptr->history_rgb[ptr->history_slot] + 
			i * ptr->image_w * 3;
		int diff;

		for(j = 0; j < ptr->image_w; j++)
		{
			int y = y_row[j];
			int u = u_row[j / 2] - 0x80;
			int v = v_row[j / 2] - 0x80;
y = 0x80;
			int r = y + V_TO_R * v;
			int g = y + U_TO_G * u + V_TO_G * v;
			int b = y + U_TO_B * u;
			CLAMP(r, 0, 255);
			CLAMP(g, 0, 255);
			CLAMP(b, 0, 255);
			*dst++ = r;
			*dst++ = g;
			*dst++ = b;
		}
	}

printf("store_history %d %d\n", __LINE__, ptr->history_slot);
#endif // USE_COLOR






	ptr->history_slot++;
	if(ptr->history_slot >= settings->accum_size) ptr->history_slot = 0;
}

#endif // 0



void process_eye(unsigned char *image, unsigned char *mask, blobs_t *blobs)
{
// threshold
	vision_t *vision = &copter.vision;
	int image_size = vision->mask_w * vision->mask_h;
	int i, j;
	int w = vision->mask_w;
	int h = vision->mask_h;
	settings_t *settings = &copter.settings;
	const int threshold = settings->threshold;

// threshold the webcams
#ifndef USE_TCM8230
	for(i = 0; i < h; i++)
	{
		unsigned char *in_row;
		unsigned char *out_row = mask + w * i;
		if(w == 640 && h == 240)
		{
			in_row = image + w * i * 2;
		}
		else
		{
			in_row = image + w * i;
		}
		
		for(j = 0; j < w; j++)
		{
			if(*in_row++ > threshold) 
				*out_row++ = MASK_VALUE;
			else
				*out_row++ = 0;
		}
	}
#else // !USE_TCM8230
// black out the noisy borders
	
	unsigned char *row1 = mask;
	unsigned char *row2 = mask + w * (h - 1);
	for(i = 0; i < w; i++)
	{
		row1[i] = 0;
		row2[i] = 0;
	}
	
	for(i = 0; i < h; i++)
	{
		mask[i * w] = 0;
		mask[i * w + (w - 1)] = 0;
	}

#endif // USE_TCM8230




// Have to take all pixels if POV is active, or it goes for the POV image
#ifdef PIC_USE_POV
	if(pov_active())
	{
		scan_single_blob(blobs, mask, RMASK_VALUE);
	}
	else
#endif
	{
//printf("process_eye %d\n", __LINE__);
		scan_blobs(blobs, mask, RMASK_VALUE);
	}
}






void process_monocopter(vision_eye_t *data)
{

#if (ROTORS == 1 && SERVOS == 0)
	unsigned char *image = data->y_buffer;
	unsigned char *mask = data->mask;
	blobs_t *blobs = &data->blobs;
	pic_t *pic = &copter.pic;
	vision_t *vision = &copter.vision;
	settings_t *settings = &copter.settings;
	ins_t *ins = &copter.ins;
	int i, j;

	
	process_eye(image, mask, blobs);
// store largest blob in accumulator
	blob_t *max_blob = blobs->max_blob;
	if(max_blob)
	{
		blob_t *dst_blob = data->blob_history + data->history_slot;

//printf("process_monocopter %d %d ", __LINE__, blobs->total_blobs);
#ifdef PIC_USE_POV
		if(pov_active())
		{
			bzero(dst_blob, sizeof(blob_t));
			dst_blob->x1 = vision->mask_w;
			dst_blob->y1 = vision->mask_h;
			dst_blob->x2 = 0;
			dst_blob->y2 = 0;
// combine all blobs greater than a minimum size
			for(i = 0; i < blobs->total_blobs; i++)
			{
				blob_t *src_blob = blobs->blobs + i;
				if(src_blob->total_pixels > MIN_BLOB_PIXELS)
				{
					memcpy(data->fill_x_history[data->history_slot] + dst_blob->fill2,
						blobs->fill_x + src_blob->fill1,
						sizeof(int) * src_blob->total_pixels);
					memcpy(data->fill_y_history[data->history_slot] + dst_blob->fill2,
						blobs->fill_y + src_blob->fill1,
						sizeof(int) * src_blob->total_pixels);
					dst_blob->fill2 += src_blob->total_pixels;
					dst_blob->total_pixels += src_blob->total_pixels;

					if(src_blob->x1 < dst_blob->x1) dst_blob->x1 = src_blob->x1;
					if(src_blob->x2 > dst_blob->x2) dst_blob->x2 = src_blob->x2;
					if(src_blob->y1 < dst_blob->y1) dst_blob->y1 = src_blob->y1;
					if(src_blob->y2 > dst_blob->y2) dst_blob->y2 = src_blob->y2;
//printf("process_monocopter %d %d %d\n", __LINE__, i, dst_blob->x1);
				}
			}
//printf("%d ", dst_blob->x1);
		}
		else
#endif // PIC_USE_POV
		{
			memcpy(dst_blob,
				max_blob,
				sizeof(blob_t));
			memcpy(data->fill_x_history[data->history_slot],
				blobs->fill_x + max_blob->fill1,
				sizeof(int) * max_blob->total_pixels);
			memcpy(data->fill_y_history[data->history_slot],
				blobs->fill_y + max_blob->fill1,
				sizeof(int) * max_blob->total_pixels);
			dst_blob->fill1 = 0;
			dst_blob->fill2 = max_blob->total_pixels;
		}
//printf("\n");
	}
	else
	{
// no blob for this slot
		bzero(data->blob_history + data->history_slot,
			sizeof(blob_t));
	}

	data->history_slot++;
	if(data->history_slot >= settings->accum_size) 
		data->history_slot = 0;

// accumulate blobs
	data->accum_blob.x1 = vision->mask_w;
	data->accum_blob.y1 = vision->mask_h;
	data->accum_blob.x2 = 0;
	data->accum_blob.y2 = 0;
	int total_pixels = 0;
	data->got_it = 0;
	data->in_frame = 0;
	int *total_fill_x = data->total_fill_x;
	int *total_fill_y = data->total_fill_y;



// get accumulated pixels & dimensions
	double x1 = vision->mask_w;
	double y1 = vision->mask_h;
	double x2 = 0;
	double y2 = 0;
//printf("process_monocopter %d ", __LINE__);
	for(i = 0; i < settings->accum_size; i++)
	{
		blob_t *blob = &data->blob_history[i];
		if(blob->total_pixels > MIN_BLOB_PIXELS)
		{
//printf("%d ", blob->x1);
			if(blob->x1 < x1) x1 = blob->x1;
			if(blob->x2 > x2) x2 = blob->x2;
			if(blob->y1 < y1) y1 = blob->y1;
			if(blob->y2 > y2) y2 = blob->y2;

// accumulate all pixels
			int *fill_x = data->fill_x_history[i];
			int *fill_y = data->fill_y_history[i];
			int fill1 = blob->fill1;
			int fill2 = blob->fill2;

/*
 * printf("process_monocopter %d i=%d %p %p %d %d\n", 
 * __LINE__, 
 * i, 
 * data->fill_x_history[0], 
 * data->total_fill_x, 
 * total_pixels, 
 * fill2 - fill1);
 */

			memcpy(total_fill_x + total_pixels,
				fill_x,
				sizeof(int) * (fill2 - fill1));
			memcpy(total_fill_y + total_pixels,
				fill_y,
				sizeof(int) * (fill2 - fill1));
			total_pixels += fill2 - fill1;
		}
	}

//printf("\n");
//printf("process_monocopter %d %f %f %f %f\n", __LINE__, x1, y1, x2, y2);


	data->center_x = (x1 + x2) / 2;
	data->center_y = (y1 + y2) / 2;
	double max_x1 = data->center_x;
	double max_y1 = data->center_y;
	double max_x2 = data->center_x;
	double max_y2 = data->center_y;
	double major_axis_distance = 0;

// get pixels farthest from center in 2 sides
//printf("process_monocopter %d %d\n", __LINE__, total_pixels);
	if(total_pixels > MIN_BLOB_PIXELS * 2)
	{
		double center_x = data->center_x;
		double center_y = data->center_y;
		data->got_it = 1;
		double max_distance1 = 0;
		double max_distance2 = 0;
		for(i = 0; i < total_pixels; i++)
		{
			double x = total_fill_x[i];
			double y = total_fill_y[i];
			double distance = sqrt(SQR(x - center_x) + SQR(y - center_y));
			if(x < center_x)
			{
				if(distance > max_distance1)
				{
					max_distance1 = distance;
					max_x1 = x;
					max_y1 = y;
				}
			}
			else
			if(x >= center_x)
			{
				if(distance > max_distance2)
				{
					max_distance2 = distance;
					max_x2 = x;
					max_y2 = y;
				}
			}
		}

		major_axis_distance = sqrt(SQR(max_x2 - max_x1) + SQR(max_y2 - max_y1));
	
// new center from major axis


/*
 * printf("process_monocopter %d %f\n", 
 * __LINE__,
 * major_axis_distance);
 */

		data->center_x = (max_x2 + max_x1) / 2;
		data->center_y = (max_y2 + max_y1) / 2;
	}



	if(data->got_it && vision->vision_ready)
	{

		draw_line(mask,
			max_x1,
			max_y1,
			max_x2,
			max_y2,
			vision->mask_w,
			vision->mask_h,
			1);

		draw_cross(mask, 
			data->center_x,
			data->center_y,
			vision->mask_w,
			vision->mask_h,
			1);
	}


// Blob entirely within frame
	if(
/*
 * 		max_x1 > 0 && 
 * 		max_x1 < vision->mask_w &&
 * 		max_y1 > 0 &&
 * 		max_y1 < vision->mask_h &&
 * 		max_x2 > 0 &&
 * 		max_x2 < vision->mask_w &&
 * 		max_y2 > 0 &&
 * 		max_y2 < vision->mask_h &&
 */
		data->got_it &&
		vision->vision_ready)
	{
		data->in_frame = 1;



//#ifndef USE_STEREO
// This didn't work because diameter changes with altitude
// Convert to right ascention & declination
		float image_ra = -(data->center_x - vision->mask_w / 2) * 
			PIXEL_TO_ANGLE_X;
		float image_d = (vision->mask_h / 2 - data->center_y) * 
			PIXEL_TO_ANGLE_Y;

		float x_sign = settings->xtilt_feedback < 0 ? -1 : 1;
		float y_sign = settings->ytilt_feedback < 0 ? -1 : 1;
		float cam_ra = x_sign * TO_RAD(vision->tilt_x * TILT_TO_ANGLE_X);
		float cam_d = y_sign * TO_RAD(vision->tilt_y * TILT_TO_ANGLE_Y);

// calculate distance
		double major_axis_angle = major_axis_distance * 
			PIXEL_TO_ANGLE_X;
		double distance = settings->object_w / tan(major_axis_angle);



// convert to XYZ
		angles_to_xyz(
			&data->x,
			&data->y,
			&data->z,
			image_ra + cam_ra, 
			image_d + cam_d, 
			distance);
//		send_gps(vision->x,
//			vision->y,
//			vision->z,
//			&vision->dx,
//			&vision->dy,
//			&vision->dz);
//printf("process_monocopter %d %f\n", __LINE__, vision->z);
//#endif // !USE_STEREO



// Get attitude for takeoff
		data->roll = -atan2(max_y2 - max_y1, max_x2 - max_x1);
		if(data->roll < -M_PI / 2) data->roll += M_PI;
		if(data->roll > M_PI / 2) data->roll -= M_PI;
		
		
// Get blobs above & below line & farthest from line
		blob_t *top_blob = 0;
		double top_distance = 0;
		blob_t *bottom_blob = 0;
		double bottom_distance = 0;
		for(i = 0; i < settings->accum_size; i++)
		{
			blob_t *blob = &data->blob_history[i];
			if(blob->total_pixels > MIN_BLOB_PIXELS)
			{
// Test if above line
				double blob_center_x = (double)(blob->x1 + blob->x2) / 2;
				double blob_center_y = (double)(blob->y1 + blob->y2) / 2;
				double line_y = (blob_center_x - max_x1) * 
					(max_y2 - max_y1) / (max_x2 - max_x1) +
					max_y1;
				double distance = point_to_line(max_x1, 
					max_y1, 
					max_x2, 
					max_y2, 
					blob_center_x, 
					blob_center_y);
// above line
				if(blob_center_y < line_y)
				{
					if(distance > top_distance || !top_blob)
					{
						top_blob = blob;
						top_distance = distance;
					}
				}
				else
// below line
				{
					if(distance > bottom_distance || !bottom_blob)
					{
						bottom_blob = blob;
						bottom_distance = distance;
					}
				}
			}
		}
		
		blob_t *biggest_blob = 0;
		int is_top = 0;
		double minor_axis_distance = 0;
		if(top_blob && bottom_blob)
		{
			if(top_blob->total_pixels > bottom_blob->total_pixels)
			{
				biggest_blob = top_blob;
				minor_axis_distance = top_distance;
				is_top = 1;
			}
			else
			{
				biggest_blob = bottom_blob;
				minor_axis_distance = bottom_distance;
				is_top = 0;
			}
		}
		else
		if(top_blob)
		{
			biggest_blob = top_blob;
			minor_axis_distance = top_distance;
			is_top = 1;
		}
		else
		if(bottom_blob)
		{
			biggest_blob = bottom_blob;
			minor_axis_distance = bottom_distance;
			is_top = 0;
		}
		
		if(biggest_blob)
		{
			double blob_center_x = (double)(biggest_blob->x1 + biggest_blob->x2) / 2;
			double blob_center_y = (double)(biggest_blob->y1 + biggest_blob->y2) / 2;
			
			draw_line(mask,
				(int)blob_center_x,
				(int)blob_center_y,
				(int)data->center_x,
				(int)data->center_y,
				vision->mask_w,
				vision->mask_h,
				1);

			
			if(minor_axis_distance < major_axis_distance)
			{
				data->pitch = asin(minor_axis_distance / (major_axis_distance / 2));
			}
			else
			{
				data->pitch = 0;
			}
			CLAMP(data->pitch, -M_PI / 2, M_PI / 2);
// If bottom edge is closer to camera, negate the angle
			if(!is_top) data->pitch *= -1;

//printf("process_monocopter %d %f %f\n", __LINE__, FROM_RAD(data->roll), FROM_RAD(data->pitch));



		}
	}
	





#endif // (ROTORS == 1 && SERVOS == 0)

}


void solve_monocopter()
{
	settings_t *settings = &copter.settings;
	pic_t *pic = &copter.pic;
	vision_t *vision = &copter.vision;
	ins_t *ins = &copter.ins;

// Need flight RPM for valid position data
	int engine_switch = get_engine_on();
	if(pic->collect_out < settings->vision_throttle || !engine_switch)
	{
		reset_timer(&vision->throttle_time);
		vision->vision_ready = 0;
/*
 * 		printf("solve_monocopter %d %f %f %d\n", 
 * 			__LINE__,
 * 			pic->collect_out,
 * 			settings->vision_throttle,
 * 			engine_switch);
 */
	}
	else
	if(get_timer_difference(&vision->throttle_time) > settings->vision_throttle_time)
	{
//		printf("solve_monocopter %d\n", __LINE__);
		vision->vision_ready = 1;
	}
	
/*
 * printf("solve_monocopter %d %d %d %d %d %d\n", 
 * __LINE__,
 * vision->left.got_it, 
 * vision->right.got_it, 
 * vision->left.in_frame, 
 * vision->right.in_frame,
 * vision->vision_ready);
 */

	if(vision->vision_ready &&
		vision->right.in_frame &&
		vision->right.got_it
#ifdef USE_STEREO
		 && 
		vision->left.in_frame &&
		vision->left.got_it
#endif
		)
	{
// camera tilt
#ifdef USE_STEREO
/*
 * 		double tilt_x = -TO_RAD(vision->tilt_x * TILT_TO_ANGLE_X);
 * 		double tilt_y = -TO_RAD(vision->tilt_y * TILT_TO_ANGLE_Y);
 * 
 * 
 * 		stereo_position_calculation(
 * 			&vision->x,
 * 			&vision->y,
 * 			&vision->z,
 * 			vision->left.center_x, 
 * 			vision->left.center_y, 
 * 			vision->right.center_x, 
 * 			vision->right.center_y,
 * 			tilt_x,
 * 			tilt_y,
 * 			vision->mask_w,
 * 			vision->mask_h);
 * 
 * 
 * 
 * 
 * 		send_gps(vision->x,
 * 			vision->y,
 * 			vision->z,
 * 			&vision->dx,
 * 			&vision->dy,
 * 			&vision->dz);
 */
#endif // USE_STEREO



/*
 * printf("solve_monocopter %d %f %f %f\n", 
 * __LINE__,
 * vision->x,
 * vision->y,
 * vision->z);
 */


		double roll = 0;
		double pitch = 0;
		double x = 0, y = 0, z = 0;
		int total_angles = 0;
		int total_positions = 0;

#ifdef USE_STEREO
		if(!isnan(vision->left.roll) && 
			!isnan(vision->left.pitch) && 
			!isinf(vision->left.roll) && 
			!isinf(vision->left.pitch))
		{
			roll += vision->left.roll;
			pitch += vision->left.pitch;
			total_angles++;
		}
		
		x += vision->left.x;
		y += vision->left.y;
		z += vision->left.z;
		total_positions++;
		
#endif // USE_STEREO


		if(!isnan(vision->right.roll) && 
			!isnan(vision->right.pitch) && 
			!isinf(vision->right.roll) && 
			!isinf(vision->right.pitch))
		{
			roll += vision->right.roll;
			pitch += vision->right.pitch;
			total_angles++;
		}
		
		x += vision->right.x;
		y += vision->right.y;
		z += vision->right.z;
		total_positions++;

		vision->x = x / total_positions;
		vision->y = y / total_positions;
		vision->z = z / total_positions;
		
  		send_gps(vision->x,
 			vision->y,
  			vision->z,
  			&vision->dx,
  			&vision->dy,
  			&vision->dz);

		if(total_angles > 0)
		{
			roll /= total_angles;
			pitch /= total_angles;

			double diff = roll - ins->theta.x;
			CLAMP(diff, -MAX_ATTITUDE_CHANGE, MAX_ATTITUDE_CHANGE);
			ins->theta.x = ins->theta.x * (1.0 - ATTITUDE_BANDWIDTH) +
				(ins->theta.x + diff) * ATTITUDE_BANDWIDTH;

			diff = pitch - ins->theta.y;
			CLAMP(diff, -MAX_ATTITUDE_CHANGE, MAX_ATTITUDE_CHANGE);
			ins->theta.y = ins->theta.y * (1.0 - ATTITUDE_BANDWIDTH) +
				(ins->theta.y + diff) * ATTITUDE_BANDWIDTH;


// Calculate turn rates
			update_derivative(&vision->roll_rate, ins->theta.x);
			update_derivative(&vision->pitch_rate, ins->theta.y);
			ins->pqr.x = get_derivative(&vision->roll_rate) * 
				settings->gps_hz / 
				derivative_size(&vision->roll_rate);

			ins->pqr.y = get_derivative(&vision->pitch_rate) * 
				settings->gps_hz / 
				derivative_size(&vision->pitch_rate);




			unsigned char buffer[9 * 4];
			*(float*)(buffer + 0) = ins->theta.x;
			*(float*)(buffer + 4) = ins->theta.y;
			*(float*)(buffer + 8) = 0;
			*(float*)(buffer + 12) = 0;
			*(float*)(buffer + 16) = 0;
			*(float*)(buffer + 20) = 0;
			*(float*)(buffer + 24) = ins->pqr.x;
			*(float*)(buffer + 28) = ins->pqr.y;
			*(float*)(buffer + 32) = 0;
			write_record(THETA_TAG, buffer, 9 * 4);
		}

	}
	else
	{
// Stuff without flight RPM, so it can start spinning
// Send as GPS position
		pthread_mutex_lock(&pic->lock);
		pic->gps_in.x = 0;
		pic->gps_in.y = 0;
		pic->gps_in.z = 0;
		pic->gps_heading = 0;
		pic->gps_velocity = 0;
		pic->gps_climb_rate = 0;
		ins->theta.x = 0;
		ins->theta.y = 0;
		ins->pqr.x = 0;
		ins->pqr.y = 0;
		reset_derivative(&vision->roll_rate);
		reset_derivative(&vision->pitch_rate);
		reset_derivative(&vision->dx);
		reset_derivative(&vision->dy);
		reset_derivative(&vision->dz);
		pthread_mutex_unlock(&pic->lock);

// Don't aim servo
		if(!vision->left.in_frame) 
		{
			vision->left.center_x = vision->mask_w / 2;
			vision->left.center_y = vision->mask_h / 2;
		}
		
		if(!vision->right.in_frame) 
		{
			vision->right.center_x = vision->mask_w / 2;
			vision->right.center_y = vision->mask_h / 2;
		}
	}
	
	pic->satellites = MIN_SATELLITES;
	pic->got_gps = 1;


	

}



void process_copter()
{
	vision_t *vision = &copter.vision;

	int i;
//printf("process_copter %d %d %d\n", __LINE__, vision->got_left, vision->got_right);

// Assume always stereo
	if(vision->got_right)
	{
		vision_eye_t *data = &vision->right;
		int min_x = vision->mask_w;
		int min_y = vision->mask_h;
		int max_x = 0;
		int max_y = 0;
		int total_right = 0;
		double right_x = 0;
		double right_y = 0;
		process_eye(data->y_buffer, data->mask, &data->blobs);
		sort_blobs(&data->blobs);

// center of right eye
		for(i = 0; i < data->blobs.total_blobs && i < 4; i++)
		{
//printf("process_copter %d %d %d\n", __LINE__, i, data->blobs.blobs[i].total_pixels);
			total_right++;
			if(data->blobs.blobs[i].x1 < min_x) min_x = data->blobs.blobs[i].x1;
			if(data->blobs.blobs[i].x2 > max_x) max_x = data->blobs.blobs[i].x2;
			if(data->blobs.blobs[i].y1 < min_y) min_y = data->blobs.blobs[i].y1;
			if(data->blobs.blobs[i].y2 > max_y) max_y = data->blobs.blobs[i].y2;
		}
		data->center_x = right_x = (double)(max_x + min_x) / 2;
		data->center_y = right_y = (double)(max_y + min_y) / 2;

		if(total_right)
		{
			draw_cross(data->mask, 
  				right_x, 
  				right_y,
  				vision->mask_w,
  				vision->mask_h,
  				1);
		}
		vision->have_right = total_right;
		data->in_frame = 1;
	}
	
	if(vision->got_left) 
	{
		vision_eye_t *data = &vision->left;
		double left_x = 0;
		double left_y = 0;
		int total_left = 0;
		process_eye(data->y_buffer, data->mask, &data->blobs);
		sort_blobs(&data->blobs);


		int min_x = vision->mask_w;
		int min_y = vision->mask_h;
		int max_x = 0;
		int max_y = 0;

		for(i = 0; i < data->blobs.total_blobs && i < 4; i++)
		{
			total_left++;
			if(data->blobs.blobs[i].x1 < min_x) min_x = data->blobs.blobs[i].x1;
			if(data->blobs.blobs[i].x2 > max_x) max_x = data->blobs.blobs[i].x2;
			if(data->blobs.blobs[i].y1 < min_y) min_y = data->blobs.blobs[i].y1;
			if(data->blobs.blobs[i].y2 > max_y) max_y = data->blobs.blobs[i].y2;
		}
		data->center_x = left_x = (double)(max_x + min_x) / 2;
		data->center_y = left_y = (double)(max_y + min_y) / 2;

//printf("process_copter %d %f %f\n", __LINE__, data->center_x, data->center_y);


		if(total_left)
		{
			draw_cross(data->mask, 
  				left_x, 
  				left_y,
  				vision->mask_w,
  				vision->mask_h,
  				1);
		}
		vision->have_left = total_left;
		data->in_frame = 1;
	}


/*
 * printf("process_copter %d ", __LINE__);
 * for(i = 0; i < vision->blobs.total_blobs; i++)
 * printf("%d ", vision->blobs.blobs[i].total_pixels);
 * printf("\n");
 */
 
 

	
/*
 * 	if(vision->blobs.max_blob)
 * 	{
 * 		blob_t *max_blob = vision->blobs.max_blob;
 * 		left_x = (max_blob->x1 + max_blob->x2) / 2;
 * 		left_y = (max_blob->y1 + max_blob->y2) / 2;
 * 		draw_cross(vision->mask, 
 * 			left_x, 
 * 			left_y,
 * 			vision->mask_w,
 * 			vision->mask_h,
 * 			1);
 * 	}
 * 
 * 	if(vision->blobs2.max_blob)
 * 	{
 * 		blob_t *max_blob = vision->blobs2.max_blob;
 * 		right_x = (max_blob->x1 + max_blob->x2) / 2;
 * 		right_y = (max_blob->y1 + max_blob->y2) / 2;
 * 		draw_cross(vision->mask2, 
 * 			right_x, 
 * 			right_y,
 * 			vision->mask_w,
 * 			vision->mask_h,
 * 			1);
 * 	}
 */


	if(vision->have_left && vision->have_right)
	{
		vision->vision_ready = 1;

// camera tilt
		double tilt_x = -TO_RAD(vision->tilt_x * TILT_TO_ANGLE_X);
		double tilt_y = -TO_RAD(vision->tilt_y * TILT_TO_ANGLE_Y);

/*
 * printf("process_copter %d %f %f %f %f\n", 
 * __LINE__, 
 * FROM_RAD(tilt_x_left),
 * FROM_RAD(tilt_y_left),
 * FROM_RAD(tilt_x_main),
 * FROM_RAD(tilt_y_main));
 */
		stereo_position_calculation(
			&vision->x,
			&vision->y,
			&vision->z,
			vision->left.center_x, 
			vision->left.center_y, 
			vision->right.center_x, 
			vision->right.center_y,
			tilt_x,
			tilt_y,
			vision->mask_w,
			vision->mask_h);

		send_gps(vision->x,
			vision->y,
			vision->z,
			&vision->dx,
			&vision->dy,
			&vision->dz);
	}
	else
	{
		vision->vision_ready = 0;
		
		if(!vision->have_left)
		{
			vision->left.center_x = vision->mask_w / 2;
			vision->left.center_y = vision->mask_h / 2;
		}
		
		if(!vision->have_right)
		{
			vision->right.center_x = vision->mask_w / 2;
			vision->right.center_y = vision->mask_h / 2;
		}
	}
}








void reset_turret()
{
	vision_t *vision = &copter.vision;
	settings_t *settings = &copter.settings;
	vision->tilt_x = 0;
	vision->tilt_y = 0;
	vision->x_pwm = settings->tilt_pwm_mid + settings->xtilt_offset;
	vision->y_pwm = settings->tilt_pwm_mid + settings->ytilt_offset;
}


// this is carried over from a failed attempt to use separate turrets for each camera
static void update_eye_turret(double center_x, 
	double center_y,
	double *tilt_x,
	double *tilt_y,
	int *pwm_x_out,
	int *pwm_y_out)
{
	vision_t *vision = &copter.vision;
	settings_t *settings = &copter.settings;
	float x_error = (float)(center_x - vision->mask_w / 2) / vision->mask_w;
	float y_error = (float)(center_y - vision->mask_h / 2) / vision->mask_h;
	double prev_tilt_x = *tilt_x;
	double prev_tilt_y = *tilt_y;
	float x_sign = settings->xtilt_feedback < 0 ? -1 : 1;
	float y_sign = settings->ytilt_feedback < 0 ? -1 : 1;

	if((abs(x_error * vision->mask_w) > SERVO_THRESHOLD ||
		abs(y_error * vision->mask_h) > SERVO_THRESHOLD) &&
		vision->vision_ready)
	{
		float x_feedback = -x_error * TILT_FEEDBACK_X;
		float y_feedback = -y_error * TILT_FEEDBACK_Y;
		CLAMP(x_feedback, -MAX_FEEDBACK, MAX_FEEDBACK);
		CLAMP(y_feedback, -MAX_FEEDBACK, MAX_FEEDBACK);


		*tilt_x += x_feedback;
		*tilt_y += y_feedback;

/*
 * printf("update_eye_turret %d %f %f %f %f %f %f\n", 
 * __LINE__, 
 * center_x,
 * center_y,
 * x_error, 
 * y_error, 
 * x_feedback, 
 * y_feedback);
 */

// alt/az
		CLAMP(*tilt_x, settings->xtilt_min, settings->xtilt_max);
		CLAMP(*tilt_y, settings->ytilt_min, settings->ytilt_max);
		
		
		
#if (ROTORS == 1) && (SERVOS == 0)
// shift accumulation buffer
		int shift_x = (int)(TO_RAD(x_sign * (*tilt_x - prev_tilt_x) * TILT_TO_ANGLE_X) / PIXEL_TO_ANGLE_X);
		int shift_y = (int)(TO_RAD(y_sign * (*tilt_y - prev_tilt_y) * TILT_TO_ANGLE_Y) / PIXEL_TO_ANGLE_Y);
//printf("update_eye_turret %d %f %f %d %d\n", __LINE__, *tilt_x - prev_tilt_x, *tilt_y - prev_tilt_y, shift_x, shift_y);
		vision_eye_t *data = &vision->right;
		int i, j;
		for(i = 0; i < settings->accum_size; i++)
		{
			blob_t *dst_blob = data->blob_history + i;
			int *fill_x = data->fill_x_history[i];
			int *fill_y = data->fill_y_history[i];
			for(j = 0; j < dst_blob->total_pixels; j++)
			{
				fill_x[j] += shift_x;
				fill_y[j] += shift_y;
			}
			dst_blob->x1 += shift_x;
			dst_blob->x2 += shift_x;
			dst_blob->y1 += shift_y;
			dst_blob->y2 += shift_y;
		}

#endif // (ROTORS == 1) && (SERVOS == 0)
		
		
	}



// DEBUG
// allow the user to manually aim the camera
//		vision->tilt_x = -copter.pic.roll_out;
//		vision->tilt_y = -copter.pic.pitch_out;

//vision->tilt_x = 0;
//vision->tilt_y = 0;

	int pwm_max = settings->tilt_pwm_mid + settings->tilt_pwm_mag;
	int pwm_min = settings->tilt_pwm_mid - settings->tilt_pwm_mag;
	int x_pwm = (int)((-(*tilt_x) + 1.0) / 2.0 * 
		(pwm_max - pwm_min) + pwm_min) + 
		settings->xtilt_offset;
	int y_pwm = (int)((-(*tilt_y) + 1.0) / 2.0 * 
		(pwm_max - pwm_min) + pwm_min) + 
		settings->ytilt_offset;

//printf("update_eye_turret %d %d\n", __LINE__, x_pwm);

	pthread_mutex_lock(&vision->turret_lock);
	*pwm_x_out = x_pwm;
	*pwm_y_out = y_pwm;
	pthread_mutex_unlock(&vision->turret_lock);

}
	

static void update_turret(vision_t *vision)
{
#ifdef USE_TURRET
	vision->initialize_count++;
	if(vision->initialize_count > 0)
	{
		vision->initialize_count--;

// Feedback
		double center_x = 0;
		double center_y = 0;
		int total = 0;
		if(vision->right.in_frame)
		{
			center_x = vision->right.center_x;
			center_y = vision->right.center_y;
			total++;
		}

#ifdef USE_STEREO
		if(vision->left.in_frame)
		{
			center_x += vision->left.center_x;
			center_y += vision->left.center_y;
			total++;
		}
#endif

		if(total > 0)
		{
			center_x /= total;
			center_y /= total;
		}

		update_eye_turret(
			center_x, 
			center_y,
			&vision->tilt_x,
			&vision->tilt_y,
			&vision->x_pwm,
			&vision->y_pwm);

/*
 * printf("update_turret %d %f %f %d %d\n", 
 * __LINE__, 
 * center_x, 
 * center_y,
 * vision->tilt_x,
 * vision->tilt_y,
 * vision->x_pwm,
 * vision->y_pwm);
 */




	}
#endif // USE_TURRET
}


void display_image(vision_t *vision)
{
	settings_t *settings = &copter.settings;
// Downsample to save clockcycles
	int want_gui_frame = (vision->left.total_frames + vision->right.total_frames) * 
		settings->gui_freq / 2 / 
		settings->gps_hz;
//printf("display_image %d %d %d\n", __LINE__, want_gui_frame, vision->total_gui_frames);
	if(want_gui_frame <= vision->total_gui_frames) return;
	vision->total_gui_frames++;


// only send preview if client is viewing
//	if(copter.server.total_clients)
	if(copter.server.client_port != 0)
	{

	// write RLE compressed frame to network
		if(!vision->preview_data)
		{
			vision->preview_allocated = 4 * vision->mask_w * vision->mask_h;
			vision->preview_data = malloc(vision->preview_allocated);
		}




		int total_pixels = vision->mask_w * vision->mask_h;
		int i, j;
		unsigned char *pixel = vision->right.mask;
		unsigned char *end_pixel = vision->right.mask + total_pixels;
		int counter = 0;
		unsigned char *preview_data = vision->preview_data;
		int preview_size = 0;

	// start of frame code
		preview_data[preview_size++] = 0xff;
		preview_data[preview_size++] = 0xd8;
		preview_data[preview_size++] = 0xff;
		preview_data[preview_size++] = 0xda;
		
		
		preview_data[preview_size++] = settings->accum_size;
#ifdef USE_STEREO
		preview_data[preview_size++] = 1;
#else
		preview_data[preview_size++] = 0;
#endif
		preview_data[preview_size++] = vision->mask_w & 0xff;
		preview_data[preview_size++] = (vision->mask_w >> 8) & 0xff;
		preview_data[preview_size++] = vision->mask_h & 0xff;
		preview_data[preview_size++] = (vision->mask_h >> 8) & 0xff;


/*
 * bzero(vision->mask2, total_pixels);
 * for(i = 32; i < 33; i++)
 * {
 *  memset(vision->mask2 + i * vision->mask_w + 32, 0xff, 32);
 * }
 */



	#define COMPRESS_MASK \
		while(pixel < end_pixel && preview_size < vision->preview_allocated - 8) \
		{ \
	/* count 0's */ \
			counter = 0; \
			while(counter < 65534 && pixel < end_pixel && *pixel == 0) \
			{ \
				counter++; \
				pixel++; \
			} \
			preview_data[preview_size++] = counter & 0xff; \
			preview_data[preview_size++] = counter >> 8; \
	 \
	/* count 1's */ \
			counter = 0; \
			while(counter < 65534 && pixel < end_pixel && *pixel != 0) \
			{ \
				counter++; \
				pixel++; \
			} \
			preview_data[preview_size++] = counter & 0xff; \
			preview_data[preview_size++] = counter >> 8; \
	 \
		}

		COMPRESS_MASK


	#ifdef USE_STEREO
		pixel = vision->left.mask;
		end_pixel = vision->left.mask + total_pixels;

		COMPRESS_MASK
	#endif







		preview_data[preview_size++] = 0xff;
		preview_data[preview_size++] = 0xff;
		vision->preview_size = preview_size;


	// send to GUI as packets
		coptertimer_t timer;
		reset_timer(&timer);
		bzero(preview_data + preview_size, 
			1024 - (preview_size % 1024));
		for(i = 0; i < preview_size; i += PICTURE_FRAGMENT_SIZE)
		{
			write_record(PICTURE_TAG, preview_data + i, PICTURE_FRAGMENT_SIZE);
		}
	//	printf("display_image %d %d bytes %dms\n", __LINE__, preview_size, get_timer_difference(&timer));
	}


}


void process_frame()
{
	vision_t *vision = &copter.vision;

	
	coptertimer_t timer;
	reset_timer(&timer);


#if (ROTORS == 1) && (SERVOS == 0)
	if(vision->got_right) process_monocopter(&vision->right);


	#ifdef USE_STEREO
	if(vision->got_left) process_monocopter(&vision->left);
	#endif

	solve_monocopter();

#else
	process_copter();
#endif


#ifdef USE_STEREO
	if(vision->vision_ready) 
#endif
		update_turret(vision);

// record processed frame
#ifdef USE_STEREO
	if(vision->got_right)
#endif
	{
		record_output(vision);


//		printf("process_frame %d %dms\n", __LINE__, get_timer_difference(&timer));
	}

	display_image(vision);


	vision->fps_counter++;

#ifndef USE_SIM
	int ms = get_timer_difference(&vision->fps_timer);

	if(ms >= 1000)
	{
		vision->fps = vision->fps_counter;
		reset_timer(&vision->fps_timer);
		pthread_mutex_lock(&copter.profile.lock);
		copter.profile.ground_fps = vision->fps;
		pthread_mutex_unlock(&copter.profile.lock);
		vision->fps_counter = 0;
	}

// Show framerate
#ifdef SHOW_FRAMERATE
	printf("process_frame %d %dfps\n", 
		__LINE__,
		vision->fps);
#endif
#endif // !USE_SIM

//printf("process_frame %d %d %d\n", __LINE__, vision->got_left, vision->got_right);
	vision->got_right = 0;
	vision->got_left = 0;

}




void* vision_thread(void *ptr2)
{
	vision_t *vision = (vision_t*)ptr2;
	
	while(1)
	{
		int picture_size = 0;
		picture_size = read_frame(vision, 
			vision->right.picture_data,
			vision->right.fd,
			vision->right.frame_buffer,
			vision->right.y_buffer,
			vision->right.u_buffer,
			vision->right.v_buffer);
// record unprocessed JPEG
		record_frame(vision, 
			vision->right.picture_data, 
			picture_size);


#ifdef USE_STEREO
		picture_size = read_frame(vision, 
			vision->left.picture_data,
			vision->left.fd,
			vision->left.frame_buffer,
			vision->left.y_buffer,
			vision->left.u_buffer,
			vision->left.v_buffer);
// record unprocessed JPEG
		record_frame(vision, 
			vision->left.picture_data, 
			picture_size);
#endif // USE_STEREO


		vision->got_left = vision->got_right = 1;
		process_frame();
	}
}



void update_camera_angle()
{
	vision_t *vision = &copter.vision;
	settings_t *settings = &copter.settings;
	pic_t *pic = &copter.pic;

	double new_pitch = (double)(pic->ground_imu0 - settings->ground_imu_miny) /
		(settings->ground_imu_maxy - settings->ground_imu_miny) * M_PI / 2;
	double new_heading = -((double)(pic->ground_imu1 - settings->ground_imu_minz) /
		(settings->ground_imu_maxz - settings->ground_imu_minz) * M_PI) +
		M_PI / 4;
	
	
	LOWPASS(vision->camera_angle.y, new_pitch, 0.05);
	LOWPASS(vision->camera_angle.z, new_heading, 0.05);

/*
 * printf("update_camera_angle %d\t%.02f\t%.02f\n", 
 * __LINE__,
 * FROM_RAD(vision->camera_angle.y),
 * FROM_RAD(vision->camera_angle.z));
 */


/*
 * 	NEW_VECTOR(accel_double, 3);
 * 	NEW_VECTOR(mag_double, 3);
 * 	intvector_to_double(&accel_double, &pic->ground_accel_in);
 * 
 * 	accel_double.x = settings->ground_accel_sign.x * 
 * 		(accel_double.x - settings->ground_accel_center.x) * 
 * 		settings->ground_accel_scale.x;
 * 	accel_double.y = settings->ground_accel_sign.y * 
 * 		(accel_double.y - settings->ground_accel_center.y) * 
 * 		settings->ground_accel_scale.y;
 * 	accel_double.z = settings->ground_accel_sign.z * 
 * 		(accel_double.z - settings->ground_accel_center.z) * 
 * 		settings->ground_accel_scale.z;
 * 
 * 	mag_double.x = (double)(pic->ground_mag_in.x - 
 * 		settings->ground_mag_center.x) / 
 * 		settings->ground_mag_scale.x;
 * 	mag_double.y = (double)(pic->ground_mag_in.y - 
 * 		settings->ground_mag_center.y) / 
 * 		settings->ground_mag_scale.y;
 * 	mag_double.z = (double)(pic->ground_mag_in.z - 
 * 		settings->ground_mag_center.z) / 
 * 		settings->ground_mag_scale.z;
 * 
 * 
 * 	accel_to_euler(&copter, &vision->camera_angle, &accel_double);
 * 	vision->camera_angle.z = compass_heading(&mag_double, 
 * 		&vision->camera_angle,
 * 		settings->ground_compass_offset);
 * 
 * 	if(settings->calibrate_ground_heading)
 * 	{
 * 		printf("update_camera_angle %d\nGROUND_COMPASS_OFFSET %d\n",
 * 			__LINE__,
 * 			-(int)FROM_RAD(vision->camera_angle.z));
 * 	}
 */



// printf("update_camera_angle %d\t%.02f\t%.02f\t%.02f\n", 
// __LINE__,
// accel_double.x,
// accel_double.y,
// accel_double.z);

// printf("update_camera_angle %d\t%.0f\t%.0f\t%.0f\n", 
// __LINE__,
// FROM_RAD(vision->camera_angle.x),
// FROM_RAD(vision->camera_angle.y),
// FROM_RAD(vision->camera_angle.z));
}


void init_vision()
{
	int i, j;
	settings_t *settings = &copter.settings;
	vision_t *vision = &copter.vision;
	
	bzero(vision, sizeof(vision_t));


	INIT_VECTOR(vision->camera_angle, 3);

	vision->image_w = settings->vision_w;
	vision->image_h = settings->vision_h;

	if(settings->use_chroma)
	{
		vision->mask_w = settings->vision_w / 2;
		vision->mask_h = settings->vision_h / 2;
	}
	else
	{
		vision->mask_w = settings->vision_w;
		vision->mask_h = settings->vision_h;
	}



	vision->fps = settings->gps_hz;
// Skip frames to read or write
#ifdef PLAY_VIDEO
	vision->total_frames = STARTING_FRAME;
#endif
	vision->total_output_frames = 0;
	vision->peak_x = settings->vision_w / 2;
	vision->peak_y = settings->vision_h / 2;
	vision->left_border = -1;
	vision->right_border = 1;
	vision->top_border = -1;
	vision->bottom_border = 1;

	reset_turret();
//	vision->dx_history = calloc(sizeof(double), V_HISTORY);
//	vision->dy_history = calloc(sizeof(double), V_HISTORY);

	init_blobs(&vision->left.blobs, 
		vision->mask_w, 
		vision->mask_h);
	init_blobs(&vision->right.blobs, 
		vision->mask_w, 
		vision->mask_h);


	reset_timer(&vision->throttle_time);
	reset_timer(&vision->fps_timer);
	init_derivative(&vision->dx, settings->vision_vhistory);
	init_derivative(&vision->dy, settings->vision_vhistory);
	init_derivative(&vision->dz, settings->vision_vhistory);

	init_derivative(&vision->roll_rate, settings->gps_hz);
	init_derivative(&vision->pitch_rate, settings->gps_hz);


// Initialize history & accumulator
// #ifndef USE_COLOR
// 	vision->history = malloc(sizeof(unsigned char*) * settings->accum_size);
// 	vision->accum = calloc(vision->mask_w, vision->mask_h);
// #else
// 	vision->history_rgb = malloc(sizeof(unsigned char*) * settings->accum_size);
// 	vision->accum = calloc(3 * vision->mask_w, vision->mask_h);
// #endif
// 
// 	for(i = 0; i < settings->accum_size; i++)
// 	{
// #ifndef USE_COLOR
// 		vision->history[i] = calloc(vision->image_w, vision->image_h);
// #else
// 		vision->history_rgb[i] = calloc(1, vision->image_w * vision->image_h * 3);
// #endif
// 	}

	vision->left.mask = calloc(1, vision->mask_w * vision->mask_h);
	vision->left.blob_history = calloc(sizeof(blob_t), settings->accum_size);
	vision->left.fill_x_history = calloc(sizeof(int*), settings->accum_size);
	vision->left.fill_y_history = calloc(sizeof(int*), settings->accum_size);
	for(i = 0; i < settings->accum_size; i++)
	{
		vision->left.fill_x_history[i] = calloc(sizeof(int), vision->mask_w * vision->mask_h);
		vision->left.fill_y_history[i] = calloc(sizeof(int), vision->mask_w * vision->mask_h);
	}
	vision->left.total_fill_x = calloc(sizeof(int), vision->mask_w * vision->mask_h * settings->accum_size);
	vision->left.total_fill_y = calloc(sizeof(int), vision->mask_w * vision->mask_h * settings->accum_size);



	vision->right.mask = calloc(1, vision->mask_w * vision->mask_h);
	vision->right.blob_history = calloc(sizeof(blob_t), settings->accum_size);
	vision->right.fill_x_history = calloc(sizeof(int*), settings->accum_size);
	vision->right.fill_y_history = calloc(sizeof(int*), settings->accum_size);
	for(i = 0; i < settings->accum_size; i++)
	{
		vision->right.fill_x_history[i] = calloc(sizeof(int), vision->mask_w * vision->mask_h);
		vision->right.fill_y_history[i] = calloc(sizeof(int), vision->mask_w * vision->mask_h);
	}
	vision->right.total_fill_x = calloc(sizeof(int), vision->mask_w * vision->mask_h * settings->accum_size);
	vision->right.total_fill_y = calloc(sizeof(int), vision->mask_w * vision->mask_h * settings->accum_size);

/*
 * printf("init_vision %d %p %p\n", 
 * __LINE__, 	
 * vision->fill_x_history[0], 
 * vision->total_fill_x);
 */





	int w = vision->image_w;
	int h = vision->image_h;
	if(w == 640 && h == 240)
	{
		h = 480;
	}


#ifndef USE_TCM8230
	
#ifdef USE_STEREO
// initialize video4linux twice to get the settings taken up
	vision->left.fd = init_input(vision, 
		settings->vision_path, 
		vision->left.frame_buffer);
	if(vision->left.fd < 0) return;

	close_input(vision->left.fd, vision->left.frame_buffer);
	vision->left.fd = init_input(vision, 
		settings->vision_path, 
		vision->left.frame_buffer);
	if(vision->left.fd < 0) return;

	if(!vision->left.y_buffer) vision->left.y_buffer = malloc(w * h);
	if(!vision->left.u_buffer) vision->left.u_buffer = malloc(w * h / 4);
	if(!vision->left.v_buffer) vision->left.v_buffer = malloc(w * h / 4);

#endif // USE_STEREO



	vision->right.fd = init_input(vision, 
		settings->vision_path2, 
		vision->right.frame_buffer);
	if(vision->right.fd < 0) return;

	close_input(vision->right.fd, vision->right.frame_buffer);
	vision->right.fd = init_input(vision, 
		settings->vision_path2, 
		vision->right.frame_buffer);
	if(vision->right.fd < 0) return;

	if(!vision->right.y_buffer) vision->right.y_buffer = malloc(w * h);
	if(!vision->right.u_buffer) vision->right.u_buffer = malloc(w * h / 4);
	if(!vision->right.v_buffer) vision->right.v_buffer = malloc(w * h / 4);


#endif // !USE_TCM8230



#ifdef USE_STEREO

	vision->left_buffer = malloc(FIFO_SIZE);

#endif // USE_STEREO


	vision->left.picture_data = malloc(MAX_COMPRESSED_SIZE);
	vision->right.picture_data = malloc(MAX_COMPRESSED_SIZE);
	
// parsing function for left camera
	vision->current_function = handle_sync_code;


	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutex_init(&vision->display_lock, &mutex_attr);
	pthread_mutex_init(&vision->display_lock2, &mutex_attr);
	pthread_mutex_init(&vision->turret_lock, &mutex_attr);
	pthread_mutex_lock(&vision->display_lock);
	pthread_mutex_lock(&vision->display_lock2);


	

// create capture thread for webcams
#ifndef USE_TCM8230
	pthread_t tid;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_create(&tid, &attr, vision_thread, vision);
#endif // !USE_TCM8230


}









#ifdef TEST_VISION

int main()
{
	vision_t vision;
#define PATH "/dev/video0"
	init_vision(&vision, 
		PATH, 
		"Vision test", 
		640, 
		480,
		0,
		0,
		640,
		480);
}




#endif
#endif // USE_VISION



