// library for reading & writing frame files


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <jpeglib.h>

// compressed output from compress_output
static unsigned char *picture_out = 0;
static int out_size;
static int out_allocated;


// quality of recorded JPEGs
#define JPEG_QUALITY 80



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
  	dest->buffer = picture_out;
  	dest->pub.next_output_byte = picture_out;
  	dest->pub.free_in_buffer = out_allocated;
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
	out_size = out_allocated - 
		dest->pub.free_in_buffer;
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

int compress_output(unsigned char *buffer,
	int allocated,
	unsigned char *out_y, 
	unsigned char *out_u,
	unsigned char *out_v,
	int w,
	int h)
{
	picture_out = buffer;
	out_allocated = allocated;




	struct jpeg_compress_struct jpeg_compress;
 	struct jpeg_error_mgr jerr;
	jpeg_compress.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&jpeg_compress);
	jpeg_compress.image_width = w;
	jpeg_compress.image_height = h;
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
 
 
 	mjpeg_dest_ptr dest;

/* The destination object is made permanent so that multiple JPEG images
 * can be written to the same file without re-executing jpeg_stdio_dest.
 * This makes it dangerous to use this manager and a different destination
 * manager serially with the same JPEG object, because their private object
 * sizes may be different.  Caveat programmer.
 */
	if(jpeg_compress.dest == NULL) 
	{	
/* first time for this JPEG object? */
      	jpeg_compress.dest = (struct jpeg_destination_mgr *)
    		(*jpeg_compress.mem->alloc_small)((j_common_ptr)&jpeg_compress, 
				JPOOL_PERMANENT,
				sizeof(mjpeg_destination_mgr));
	}

	dest = (mjpeg_dest_ptr)jpeg_compress.dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_output_buffer;
	dest->pub.term_destination = term_destination;

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
			i < 16 && i + jpeg_compress.next_scanline < h; 
			i++)
		{
			mcu_rows[0][i] = out_y + 
				(jpeg_compress.next_scanline + i) * w;
			if(i < 8)
			{
				unsigned char *u_row = mcu_rows[1][i] = out_u +
					(jpeg_compress.next_scanline + i * 2) * w;
				unsigned char *v_row = mcu_rows[2][i] = out_v +
					(jpeg_compress.next_scanline + i * 2) * w;
// pack UV data for this line
				for(j = 0; j < w / 2; j++)
				{
					u_row[j] = u_row[j * 2];
					v_row[j] = v_row[j * 2];
				}
			}
		}

		jpeg_write_raw_data(&jpeg_compress, 
			mcu_rows, 
			h);
	}
	
	
	
	jpeg_finish_compress(&jpeg_compress);
	jpeg_destroy((j_common_ptr)&jpeg_compress);


	for(i = 0; i < 3; i++)
	{
		free(mcu_rows[i]);
	}
	
//	printf("compress_output %d\n", __LINE__);

	return out_size;
}



FILE *tar_fd = 0;
int frames_written = 0;
int append_file(char *path, unsigned char *data, int size)
{
	if(!tar_fd)
	{
		tar_fd = fopen(path, "w");
		printf("writing to %s\n", path);

	}
	
	if(!tar_fd)
	{
		printf("Couldn't open %s\n", path);
		exit(1);
	}

	fwrite(data, size, 1, tar_fd);
	fflush(tar_fd);
	frames_written++;


printf("append_file %d: wrote %d\r", __LINE__, frames_written);
fflush(stdout);
	
	return frames_written;
}





