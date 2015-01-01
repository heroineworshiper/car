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



#include "settings.h"
#include "util.h"

#include "vision_common.h"




#ifdef USE_VISION


#include <linux/videodev2.h>

// makes no difference in latency because we always consume faster than it 
// produces
//#define DEVICE_BUFFERS 8
#define DEVICE_BUFFERS 2
//#define USE_COLOR



typedef struct
{
	int fd;

// device read buffers
	unsigned char *frame_buffer[DEVICE_BUFFERS];
// compressed image from camera
	unsigned char *picture_data;
	int picture_size;


	unsigned char *y_buffer;
	unsigned char *u_buffer;
	unsigned char *v_buffer;
	unsigned char *mask;
	blobs_t blobs;

// History
// dimensions of largest blobs
	blob_t *blob_history;
	int **fill_x_history;
	int **fill_y_history;
	int *total_fill_x;
	int *total_fill_y;

// accumulation buffer
//	unsigned char *accum;
// accumulated blob size
	blob_t accum_blob;
// Next slot to fill
	int history_slot;

// Position of object in frame
	double center_x;
	double center_y;

// attitude result
	double roll;
	double pitch;

// position result
	double x, y, z;

// Total frames read or written
	int total_frames;
// last frame had object
	int got_it;
// last frame had object entirely in frame
	int in_frame;
} vision_eye_t;



typedef struct 
{
	int buffer_size;

// packets from left camera
	unsigned char *left_buffer;
	int left_bytes;
// current function for parsing left camera data
	void (*current_function)(unsigned char data);


	unsigned char **jpeg_rows;
	unsigned char *jpeg_bitmap;
	int jpeg_shmid;
// original image size
	int image_w, image_h;
// processed size (may be chroma only)
	int mask_w, mask_h;
	int window_x, window_y, window_w, window_h;
	pthread_mutex_t display_lock;
	pthread_mutex_t display_lock2;


	int max;
	int min;

// compressed image for GUI
	unsigned char *preview_data;
	int preview_size;
	int preview_allocated;

// Copy of mask to display asynchronously
//	unsigned char *display_mask;
// ready to display next frame
//	int display_ready;


	vision_eye_t left;
	vision_eye_t right;




// Total output frames written
	int total_output_frames;
// Total frames sent to GUI
	int total_gui_frames;


	int peak_x;
	int peak_y;
// Relative to peak
// Negative if above or left of peak
	int top_border;
	int bottom_border;
	int left_border;
	int right_border;
// Vertical position on sides of peak
	int left_peak;
	int right_peak;
// Minor axis, relative to left & right peak
	int top_peak;
	int bottom_peak;
// 0 if top is closer
	int closer;
	int fps;



	int turret_fd;
	int initialize_count;
	double tilt_x;
	double tilt_y;
	int x_pwm;
	int y_pwm;
	pthread_mutex_t turret_lock;


// Spherical coordinates
	float x_angle;
	float y_angle;
	float dist;

// Final result
	derivative_t dx;
	derivative_t dy;
	derivative_t dz;
	
	
//	double *dx_history;
//	double *dy_history;
//	int v_slot;
	
	
	double x, y, z;

	derivative_t roll_rate;
	derivative_t pitch_rate;

// Euler camera angle from ground IMU
	vector_t camera_angle;

// Amount of time throttle has been above minimum
	coptertimer_t throttle_time;
	coptertimer_t fps_timer;
	int fps_counter;
// Have enough revolutions for the position information to be useful
	int vision_ready;
// Got frame for each eye
	int got_left;
	int got_right;
// Found object in each eye
	int have_left;
	int have_right;
} vision_t;



void reset_turret();

void init_vision();

void write_vision(unsigned char *data, int size, int is_left);
void handle_left_byte(unsigned char data);


#endif // USE_VISION


#endif // VISION_H





