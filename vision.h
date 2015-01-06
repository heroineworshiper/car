/*
 * VICACOPTER
 * Copyright (C) 2012  Adam Williams <broadcast at earthling dot net>
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



#ifndef VISION_H
#define VISION_H




#include <linux/videodev2.h>
#include <pthread.h>

// makes no difference in latency because we always consume faster than it 
// produces
//#define DEVICE_BUFFERS 8
#define DEVICE_BUFFERS 2
//#define USE_COLOR



// General purpose timer
typedef struct
{
	struct timeval start_time;
} cartimer_t;


typedef struct 
{
// device read buffers
	int buffer_size;
	unsigned char *frame_buffer[DEVICE_BUFFERS];

	unsigned char **jpeg_rows;
	unsigned char *jpeg_bitmap;
	int jpeg_shmid;
// original image size
	int image_w, image_h;
// processed size (may be chroma only)
	int mask_w, mask_h;

	int max;
	int min;

// compressed image for GUI
	unsigned char *preview_data;
	int preview_size;
	int preview_allocated;


	int fd;
	struct v4l2_format v4l2_params;

// compressed image from camera
	unsigned char *picture_data;
	int picture_size;
// image for web server
	unsigned char *latest_image;
	int latest_size;
	pthread_mutex_t latest_lock;


	unsigned char *y_buffer;
	unsigned char *u_buffer;
	unsigned char *v_buffer;
	unsigned char *mask;

// Total frames read or written
	int total_frames;
	int fps;
	int frames_written;
	cartimer_t timer;
	cartimer_t timer2;
	int led_on;
} vision_t;


void init_vision();
// Reset the timer.  This is not reentrant.
void reset_timer(cartimer_t *ptr);
// Get difference since last reset in ms.  This is reentrant.
int get_timer_difference(cartimer_t *ptr);


#endif // VISION_H





