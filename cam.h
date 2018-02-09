// camera library


#ifndef CAM_H_
#define CAM_H_

#include <linux/videodev2.h>
#include <stdio.h>
#include <pthread.h>

// Save unprocessed frames to files.
//#define RECORD_INPUT
// Read JPEG from files instead of video device.
//#define PLAYBACK
// Record the processed images.
#define RECORD_OUTPUT


// space for compressed output frame
#define PICTURE_DATA_SIZE 0x1000000

// Device to read
#define DEVICE_PATH "/dev/video0"

// makes no difference in latency because we always consume faster than it 
// produces
//#define DEVICE_BUFFERS 8
#define DEVICE_BUFFERS 2

typedef struct
{
// webcam FD
	int fd;
	FILE *playback_fd;
// imported image size
	int cam_w, cam_h;
// working image size
	int working_w, working_h;
	struct v4l2_format v4l2_params;
// device read buffers
	int buffer_size;
	unsigned char *frame_buffer[DEVICE_BUFFERS];
	pthread_mutex_t latest_lock;
// raw image read from camera
	unsigned char *picture_data;
	int picture_size;
// Total frames read
	int total_frames;
} cam_t;


int init_input(char *path, cam_t *cam);
void close_input(cam_t *cam);
int read_frame(cam_t *cam,
	unsigned char *output_y_arg,
	unsigned char *output_u_arg,
	unsigned char *output_v_arg,
	int output_w);



#endif // CAM_H_


