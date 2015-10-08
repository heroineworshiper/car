/*
 * Truck vision
 * Copyright (C) 2014-2015  Adam Williams <broadcast at earthling dot net>
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



#include <stdio.h>
#include <stdint.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>

// makes no difference in latency because we always consume faster than it 
// produces
//#define DEVICE_BUFFERS 8
#define DEVICE_BUFFERS 2
#define TOTAL_PACKAGES 4
#define TOTAL_CPUS 4
//#define USE_COLOR
#define USE_OPENCV

#define TEXTLEN 1024
#define SQR(x) ((x) * (x))

// General purpose timer
typedef struct
{
	struct timeval start_time;
} cartimer_t;




typedef struct
{
// scaled working image
	unsigned char *in_y;
	unsigned char *in_u;
	unsigned char *in_v;
// output image for debugging
	unsigned char *out_y;
	unsigned char *out_u;
	unsigned char *out_v;
// the mask
	unsigned char *mask;
	int *accum;
	
	sem_t complete_lock;
} vision_package_t;




typedef struct
{
	int done;
	sem_t input_lock;
	sem_t complete_lock;
} vision_engine_t;




typedef struct 
{
// device read buffers
	int buffer_size;
	unsigned char *frame_buffer[DEVICE_BUFFERS];

	char read_path[TEXTLEN];
	char write_path[TEXTLEN];
	char ref_path[TEXTLEN];

	unsigned char **jpeg_rows;
	unsigned char *jpeg_bitmap;
	int jpeg_shmid;
// imported image size
	int cam_w, cam_h;
// working image size
	int working_w, working_h;
// output image size for debugging
	int output_w, output_h;
// coordinates from config file
// range of top X for the line 0 - 100
	int top_x1;
	int top_x2;
	int top_y;
// range of bottom X for the line 0 - 100
	int bottom_x1;
	int bottom_x2;
	int bottom_y;
// pixels on each side of the line 0 - 100
	int side_w;
	int search_step;
	
	int threshold;
	int search_h;
	float max_dx;
	int edge_size;

// the path
	int path_x;
// the vanishing points of the center line in range of 0 - 1.0
	float bottom_x, top_x;
	float geometry_bandwidth;


// compressed image for GUI
	unsigned char *preview_data;
	int preview_size;
	int preview_allocated;


	int fd;
	FILE *playback_fd;
	struct v4l2_format v4l2_params;

// compressed image from camera
	unsigned char *picture_data;
	int picture_size;
// image for web server
	unsigned char *latest_image;
	int latest_size;
	pthread_mutex_t latest_lock;
	sem_t spi_send_lock;
	sem_t spi_complete_lock;
	unsigned char spi_tx_data[1024];
	unsigned char spi_rx_data[1024];
	int spi_tx_size;


// Total frames read or written
	int total_frames;
	int fps;
	int frames_written;
	cartimer_t timer;
	cartimer_t timer2;
	int led_on;

	vision_engine_t engine[TOTAL_CPUS];
	vision_package_t packages[TOTAL_PACKAGES];
	sem_t package_lock;
// next array index to process
	int input_package;
// next array index to store
	int output_package;
} vision_t;



extern vision_t vision;

void init_vision();
// Reset the timer.  This is not reentrant.
void reset_timer(cartimer_t *ptr);
// Get difference since last reset in ms.  This is reentrant.
int get_timer_difference(cartimer_t *ptr);
void draw_line(vision_engine_t *engine, int x1, int y1, int x2, int y2);
void draw_pixel(vision_engine_t *engine, int x, int y);
void draw_rect(vision_engine_t *engine, int x1, int y1, int x2, int y2);
int read_file_frame(FILE *fd, uint8_t *y_buffer, uint8_t *u_buffer, uint8_t *v_buffer);
void compress_jpeg(vision_engine_t *engine);


#endif // VISION_H





