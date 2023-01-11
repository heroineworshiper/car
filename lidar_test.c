#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <jpeglib.h>
#include <setjmp.h>

void (*current_function)(unsigned char);
#define PACKET_SIZE 22
#define TOTAL_RANGES 360
unsigned char packet[PACKET_SIZE];
int distance[TOTAL_RANGES];
int packet_size = 0;
#define PICTURE_W 512
#define PICTURE_H 512
unsigned char *picture;
unsigned char **rows;
#define MAX_DISTANCE 16
#define OVERSAMPLE 1
int total_errors = 0;
Display *display;
Window win;
int screen;
Window rootwin;
Visual *vis;
int default_depth;
XImage *ximage;
unsigned char *x_bitmap;
GC gc;
// Output file to record
#define RECORD_PATH "lidar.out"
unsigned char *compressed_data;
int compressed_size;
int compressed_allocated;
int total_frames = 0;

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
  	dest->buffer = compressed_data;
  	dest->pub.next_output_byte = compressed_data;
  	dest->pub.free_in_buffer = PICTURE_W * PICTURE_H * 3;
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
	compressed_size = 
		compressed_allocated - 
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


void compress_jpeg()
{
	if(!compressed_data)
	{
		compressed_allocated = PICTURE_W * PICTURE_H * 3;
		compressed_data = malloc(compressed_allocated);
	}
	
	struct jpeg_compress_struct jpeg_compress;
 	struct jpeg_error_mgr jerr;
	jpeg_compress.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&jpeg_compress);
	jpeg_compress.image_width = PICTURE_W;
	jpeg_compress.image_height = PICTURE_H;
	jpeg_compress.input_components = 3;
	jpeg_compress.in_color_space = JCS_RGB;
	jpeg_set_defaults(&jpeg_compress);
	jpeg_set_quality(&jpeg_compress, 80, 0);
	jpeg_compress.dct_method = JDCT_IFAST;
	jpeg_compress.comp_info[0].h_samp_factor = 2;
	jpeg_compress.comp_info[0].v_samp_factor = 2;
	jpeg_compress.comp_info[1].h_samp_factor = 1;
	jpeg_compress.comp_info[1].v_samp_factor = 1;
	jpeg_compress.comp_info[2].h_samp_factor = 1;
	jpeg_compress.comp_info[2].v_samp_factor = 1;
	jpeg_buffer_dest(&jpeg_compress);

	int i, j;
	jpeg_start_compress(&jpeg_compress, TRUE);
	while(jpeg_compress.next_scanline < jpeg_compress.image_height)
	{
		jpeg_write_scanlines(&jpeg_compress, 
			rows + jpeg_compress.next_scanline, 
			1);
	}
	
	
	
	jpeg_finish_compress(&jpeg_compress);
	jpeg_destroy((j_common_ptr)&jpeg_compress);


	char string[1024];
	sprintf(string, "%s%06d.jpg", RECORD_PATH, total_frames);
	FILE *out = fopen(string, "w");
	if(out)
	{
		fwrite(compressed_data, compressed_size, 1, out);
		fclose(out);
	}
	else
	{
		printf("Couldn't write %s\n", string);
	}
	
	total_frames++;
}

void get_sync(unsigned char c);

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



unsigned char read_char(int fd)
{
	unsigned char c;
	int result = read(fd, &c, 1);
	if(result <= 0)
	{
		printf("Unplugged\n");
		exit(1);
	}
	
	return c;
}

void draw_dot(int x, int y)
{
	int i, j;
	for(i = y - 1; i < y + 1; i++)
	{
		if(i >= 0 && i < PICTURE_H)
		{
			unsigned char *row = rows[i];
			for(j = x - 1; j < x + 1; j++)
			{
				if(j >= 0 && j < PICTURE_W)
				{
					row[j * 3 + 0] = 0;
					row[j * 3 + 1] = 0;
					row[j * 3 + 2] = 0;
				}
			}
		}
	}
}

void get_packet(unsigned char c)
{
	packet[packet_size++] = c;
	if(packet_size >= PACKET_SIZE)
	{
		current_function = get_sync;
		int rpm = (packet[2] | (packet[3] << 8)) / 64;
		int id = packet[1] - 0xa0;
		int error[4];
		int i, j;

/*
 * 		for(i = 0; i < PACKET_SIZE; i++)
 * 		{
 * 			printf("%02x ", packet[i]);
 * 		}
 * 		printf("\n");
 */

		if(id >= 0 && id < 90)
		{
			for(i = 0; i < 4; i++)
			{
				unsigned char *ptr = packet + 4 + i * 4;
				error[i] = (ptr[1] & 0x80) ? 1 : 0;
				if(!error[i])
				{
					distance[id * 4 + i] = ptr[0] |
						((ptr[1] & 0x3f) << 8);
				}
				else
				{
					total_errors++;
					distance[id * 4 + i] = -1;
				}
			}

			if(id == 89)
			{
				printf("rpm=%d errors=%d\n", 
					rpm,
					total_errors);
				total_errors = 0;

//				memset(picture, 0xff, PICTURE_W * PICTURE_H * 3);
				for(i = 0; i < PICTURE_H; i++)
				{
					unsigned char *ptr = rows[i];
					for(j = 0; j < PICTURE_W; j++)
					{
						if(ptr[0] < 0xff)
						{
							int value = ptr[0] + 256 / OVERSAMPLE;
							if(value > 0xff) value = 0xff;
							ptr[0] = value;
							ptr[1] = value;
							ptr[2] = value;
						}
						
						
						ptr += 3;
					}
				}


				for(i = 0; i < 360; i++)
				{
					if(distance[i] >= 0)
					{
	//printf("i=%d distance=%d\n", i, distance[i]);
						int x = (int)(cos(i * 2 * M_PI / 360) * 
								distance[i] / MAX_DISTANCE) + 
							PICTURE_W / 2;
						int y = (int)(-sin(i * 2 * M_PI / 360) * 
								distance[i] / MAX_DISTANCE) + 
							PICTURE_H / 2;
	//printf("i=%d distance=%d x=%d y=%d\n", i, distance[i], x, y);
						if(x >= 0 && x < PICTURE_W && y >= 0 && y < PICTURE_H)
						{
							draw_dot(x, y);
						}
					}
				}

				for(i = 0; i < PICTURE_H; i++)
				{
					unsigned char *dst_ptr = x_bitmap + i * ximage->bytes_per_line;
					unsigned char *src_ptr = rows[i];
					for(j = 0; j < PICTURE_W; j++)
					{
						*dst_ptr++ = *src_ptr++;
						*dst_ptr++ = *src_ptr++;
						*dst_ptr++ = *src_ptr++;
						*dst_ptr++ = 0;
					}
				}

				XPutImage(display, 
					win, 
					gc, 
					ximage, 
					0, 
					0, 
					0, 
					0, 
					PICTURE_W, 
					PICTURE_H);
				XFlush(display);
				
				compress_jpeg();
			}
		}
		
/*
 * 		printf("rpm=%d id=%d error=%d%d%d%d\n", 
 * 			rpm,
 * 			packet[1] - 0xa0,
 * 			error[0],
 * 			error[1],
 * 			error[2],
 * 			error[3]);
 */
	}
}

void get_sync(unsigned char c)
{
	if(c == 0xfa)
	{
		current_function = get_packet;
		packet_size = 1;
		packet[0] = c;
	}
}


void main()
{
	int serial_fd = init_serial("/dev/ttyUSB0", B115200, 0);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB1", B115200, 0);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB2", B115200, 0);
	if(serial_fd < 0)
	{
		printf("Couldn't open the serial port\n");
		exit(1);
	}

	current_function = get_sync;
	picture = malloc(PICTURE_W * PICTURE_H * 3);
	rows = malloc(PICTURE_H * sizeof(unsigned char*));
	int i;
	for(i = 0; i < PICTURE_H; i++)
	{
		rows[i] = picture + i * PICTURE_W * 3;
	}


	display = XOpenDisplay(NULL);
	screen = DefaultScreen(display);
	rootwin = RootWindow(display, screen);
	vis = DefaultVisual(display, screen);
	default_depth = DefaultDepth(display, screen);
	unsigned long mask = CWEventMask | 
				CWBackPixel | 
				CWColormap | 
				CWCursor;
	XSetWindowAttributes attr;
	mask = 0;
	win = XCreateWindow(display, 
			rootwin, 
			0, 
			0, 
			PICTURE_W, 
			PICTURE_H, 
			0, 
			default_depth, 
			InputOutput, 
			vis, 
			mask, 
			&attr);
	unsigned long gcmask = 0;
	XGCValues gcvalues;
	gc = XCreateGC(display, rootwin, gcmask, &gcvalues);
	XSizeHints size_hints;
	XGetNormalHints(display, win, &size_hints);
	size_hints.flags = PSize;
	size_hints.width = PICTURE_W;
	size_hints.height = PICTURE_H;
	XSetStandardProperties(display, 
		win, 
		"Lidar Test", 
		"Lidar Test", 
		None, 
		0, 
		0, 
		&size_hints);
	XMapWindow(display, win); 
	XFlush(display);
	x_bitmap = 0;
	ximage = XCreateImage(display, 
		vis, 
		default_depth, 
		ZPixmap, 
		0, 
		(char*)x_bitmap, 
		PICTURE_W, 
		PICTURE_H, 
		8, 
		0);
	x_bitmap = malloc(PICTURE_H * ximage->bytes_per_line);
	XDestroyImage(ximage);
	ximage = XCreateImage(display, 
		vis, 
		default_depth, 
		ZPixmap, 
		0, 
		(char*)x_bitmap, 
		PICTURE_W, 
		PICTURE_H, 
		8, 
		0);

	while(1)
	{
		unsigned char c = read_char(serial_fd);
		current_function(c);
	}
}












