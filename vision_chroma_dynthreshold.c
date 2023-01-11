#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vision.h"


// 640x480
// how many pixels above the key pixel to search for the next key
#define SEARCH_H 2
//#define SEARCH_H 4
//#define SEARCH_H 6
// maximum shift in the key pixel X
#define MAX_DX 0.25
//#define MAX_DX 1
// Edge detection kernel size
#define EDGE_SIZE 4
//#define EDGE_SIZE 16
//#define EDGE_SIZE 32

// 160x120
//#define THRESHOLD 1
//#define SEARCH_H 2
//#define MAX_DX 1
//#define EDGE_SIZE 5

#undef CLAMP
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))

int threshold;

int vtor_tab[0x100], vtog_tab[0x100];
int utog_tab[0x100], utob_tab[0x100];
int rtoy_tab[0x100], gtoy_tab[0x100], btoy_tab[0x100];
int rtou_tab[0x100], gtou_tab[0x100], btou_tab[0x100];
int rtov_tab[0x100], gtov_tab[0x100], btov_tab[0x100];

#define YUV_TO_RGB(y, u, v, r, g, b) \
{ \
	(r) = ((y + vtor_tab[v]) >> 16); \
	(g) = ((y + utog_tab[u] + vtog_tab[v]) >> 16); \
	(b) = ((y + utob_tab[u]) >> 16); \
	CLAMP(r, 0, 0xff); \
	CLAMP(g, 0, 0xff); \
	CLAMP(b, 0, 0xff); \
}

#define RGB_TO_YUV(y, u, v, r, g, b) \
{ \
	y = ((rtoy_tab[r] + gtoy_tab[g] + btoy_tab[b]) >> 16); \
	u = ((rtou_tab[r] + gtou_tab[g] + btou_tab[b]) >> 16); \
	v = ((rtov_tab[r] + gtov_tab[g] + btov_tab[b]) >> 16); \
	CLAMP(y, 0, 0xff); \
	CLAMP(u, 0, 0xff); \
	CLAMP(v, 0, 0xff); \
}

// Compression coefficients straight out of jpeglib
#define R_TO_Y    0.29900
#define G_TO_Y    0.58700
#define B_TO_Y    0.11400

#define R_TO_U    -0.16874
#define G_TO_U    -0.33126
#define B_TO_U    0.50000

#define R_TO_V    0.50000
#define G_TO_V    -0.41869
#define B_TO_V    -0.08131

// Decompression coefficients straight out of jpeglib
#define V_TO_R    1.40200
#define V_TO_G    -0.71414

#define U_TO_G    -0.34414
#define U_TO_B    1.77200

void init_yuv()
{
	int i;
	for(i = 0; i < 0x100; i++)
	{
		rtoy_tab[i] = (int)(R_TO_Y * 0x10000 * i);
		rtou_tab[i] = (int)(R_TO_U * 0x10000 * i);
		rtov_tab[i] = (int)(R_TO_V * 0x10000 * i);

		gtoy_tab[i] = (int)(G_TO_Y * 0x10000 * i);
		gtou_tab[i] = (int)(G_TO_U * 0x10000 * i);
		gtov_tab[i] = (int)(G_TO_V * 0x10000 * i);

		btoy_tab[i] = (int)(B_TO_Y * 0x10000 * i);
		btou_tab[i] = (int)(B_TO_U * 0x10000 * i) + 0x800000;
		btov_tab[i] = (int)(B_TO_V * 0x10000 * i) + 0x800000;
	}

	for(i = -0x80; i < 0x80; i++)
	{
		vtor_tab[i + 0x80] = (int)(V_TO_R * 0x10000 * i);
		vtog_tab[i + 0x80] = (int)(V_TO_G * 0x10000 * i);

		utog_tab[i + 0x80] = (int)(U_TO_G * 0x10000 * i);
		utob_tab[i + 0x80] = (int)(U_TO_B * 0x10000 * i);
	}
}


int detect_blobs(int fill_it)
{
// scan the entire mask
	int i, j, k;
	int *fill_x = (int*)malloc(sizeof(int) * vision.working_w * vision.working_h);
	int *fill_y = (int*)malloc(sizeof(int) * vision.working_w * vision.working_h);
	int *best_fill_x = (int*)malloc(sizeof(int) * vision.working_w * vision.working_h);
	int *best_fill_y = (int*)malloc(sizeof(int) * vision.working_w * vision.working_h);
	int best_total_filled = 0;	
	
	for(i = 0; i < vision.working_h; i++)
	{
		for(j = 0; j < vision.working_w; j++)
		{
// got a new blob
			if(vision.mask[i * vision.working_w + j] == 1)
			{
				int total_filled = 0;
				int prev_filled = 0;
				
				vision.mask[i * vision.working_w + j] = 2;
				fill_x[total_filled] = j;
				fill_y[total_filled] = i;
				total_filled++;
				
				while(total_filled > prev_filled)
				{
					int start = prev_filled;
					prev_filled = total_filled;
					
					for(k = start; k < prev_filled; k++)
					{
						int x = fill_x[k];
						int y = fill_y[k];
						
						if(y > 0)
						{
							if(vision.mask[(y - 1) * vision.working_w + x] == 1)
							{
								vision.mask[(y - 1) * vision.working_w + x] = 2;
								fill_x[total_filled] = x;
								fill_y[total_filled] = y - 1;
								total_filled++;
							}
						}
						
						if(y < vision.working_h - 1)
						{
							if(vision.mask[(y + 1) * vision.working_w + x] == 1)
							{
								vision.mask[(y + 1) * vision.working_w + x] = 2;
								fill_x[total_filled] = x;
								fill_y[total_filled] = y + 1;
								total_filled++;
							}
						}
						
						if(x > 0)
						{
							if(vision.mask[y * vision.working_w + x - 1] == 1)
							{
								vision.mask[y * vision.working_w + x - 1] = 2;
								fill_x[total_filled] = x - 1;
								fill_y[total_filled] = y;
								total_filled++;
							}
						}
						
						if(x < vision.working_w - 1)
						{
							if(vision.mask[y * vision.working_w + x + 1] == 1)
							{
								vision.mask[y * vision.working_w + x + 1] = 2;
								fill_x[total_filled] = x + 1;
								fill_y[total_filled] = y;
								total_filled++;
							}
						}
					}
				}

// store if biggest blob
				if(total_filled > best_total_filled)
				{
					best_total_filled = total_filled;
					memcpy(best_fill_x, fill_x, total_filled * sizeof(int));
					memcpy(best_fill_y, fill_y, total_filled * sizeof(int));
				}
				
			}
		}
	}

	int64_t distance_accum = 0;
//	if(!fill_it)
	{
		int accum_x = 0, accum_y = 0;
		for(i = 0; i < best_total_filled; i++)
		{
			accum_x += best_fill_x[i];
			accum_y += best_fill_y[i];
		}
		accum_x /= best_total_filled;
		accum_y /= best_total_filled;

		for(i = 0; i < best_total_filled; i++)
		{
			distance_accum += sqrt(SQR(best_fill_x[i] - accum_x) + 
				SQR(best_fill_y[i] - accum_y));
		}
		distance_accum /= best_total_filled;
		printf("detect_blobs %d %d %d\n", __LINE__, threshold, distance_accum);
	}

// color only best blob
// TODO: store separate best_fill tables with each threshold
//	if(fill_it)
	{
		bzero(vision.mask, vision.working_w * vision.working_h);
		for(i = 0; i < best_total_filled; i++)
		{
			vision.mask[best_fill_y[i] * vision.working_w + best_fill_x[i]] = 1;
		}
	}

	free(fill_x);
	free(fill_y);
	free(best_fill_x);
	free(best_fill_y);
	
	return distance_accum;
}

int test_pixel(int x, int y)
{
	if(x < 0 || y < 0 || x >= vision.working_w || y >= vision.working_h)
	{
		return 0;
	}


//printf("test_pixel x1=%d x2=%d y=%d %d\n", x1, x2, y, right_accum - left_accum);
	return *(vision.accum + x + y * vision.working_w);
}

int test_line(int x1, int y1, int x2, int y2)
{
	int w = labs(x2 - x1);
	int h = labs(y2 - y1);
	int result = 0;


	if(!w && !h)
	{
		result = test_pixel(x1, y1);
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
			result += test_pixel(i, y);
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
			result += test_pixel(x, i);
		}
	}

	return result;
}


// edge detection
void edge_detection()
{
	int i, j;
	bzero(vision.accum, vision.working_h * vision.working_w * sizeof(int));
// horizontal edges
	for(i = 0; i < vision.working_h; i++)
	{
		int right_accum = 0;
		int left_accum = 0;
		unsigned char *row = vision.mask + i * vision.working_w;
		int *dst = vision.accum + i * vision.working_w + EDGE_SIZE;

// calculate 1st pixel 
		for(j = 0; j < EDGE_SIZE; j++)
		{
			left_accum += row[j];
		}
		
		for(j = EDGE_SIZE; j < EDGE_SIZE * 2; j++)
		{
			right_accum += row[j];
		}
		
		for(j = EDGE_SIZE; j < vision.working_w - EDGE_SIZE; j++)
		{
			left_accum -= row[j - EDGE_SIZE];
			left_accum += row[j];
			right_accum -= row[j];
			right_accum += row[j + EDGE_SIZE];
			*(dst++) = right_accum - left_accum;
		}
	}
}


void normalize()
{
	int i, j;

	int max_y = 0, min_y = 0xff;
	int max_u = 0, min_u = 0xff;
	int max_v = 0, min_v = 0xff;
	for(i = 0; i < vision.working_h; i++)
	{
		unsigned char *y = &vision.y_buffer[i * vision.working_w];
		unsigned char *u = &vision.u_buffer[i * vision.working_w];
		unsigned char *v = &vision.v_buffer[i * vision.working_w];
		for(j = 0; j < vision.working_w; j++)
		{
			if(*y > max_y)
			{
				max_y = *y;
			}
			if(*y < min_y)
			{
				min_y = *y;
			}
			y++;

			if(*u > max_u)
			{
				max_u = *u;
			}
			if(*u < min_u)
			{
				min_u = *u;
			}
			u++;

			if(*v > max_v)
			{
				max_v = *v;
			}
			if(*v < min_v)
			{
				min_v = *v;
			}
			v++;
		}
	}

// printf("normalize %d %d %d %d\n", 
// __LINE__,
// (max_y - min_y) * 100 / 255,
// (max_u - min_u) * 100 / 255,
// (max_v - min_v) * 100 / 255);

	if(max_y > min_y && max_u > min_u && max_v > min_v)
	{
		for(i = 0; i < vision.working_h; i++)
		{
			unsigned char *y = &vision.y_buffer[i * vision.working_w];
			unsigned char *u = &vision.u_buffer[i * vision.working_w];
			unsigned char *v = &vision.v_buffer[i * vision.working_w];
			for(j = 0; j < vision.working_w; j++)
			{
				*y = (*y - min_y) * 0xff / (max_y - min_y);
				y++;
/*
 *  				*u = (*u - min_u) * 0xff / (max_u - min_u);
 *  				u++;
 *  				*v = (*v - min_v) * 0xff / (max_v - min_v);
 *  				v++;
 */
			}
		}
	}


// 	int max_r = 0;
// 	int min_r = 0xff;
// 	int max_g = 0;
// 	int min_g = 0xff;
// 	int max_b = 0;
// 	int min_b = 0xff;
// YUV to RGB conversion
// 	for(i = 0; i < vision.working_h; i++)
// 	{
// 		unsigned char *y = &vision.y_buffer[i * vision.working_w];
// 		unsigned char *u = &vision.u_buffer[i * vision.working_w];
// 		unsigned char *v = &vision.v_buffer[i * vision.working_w];
// 		for(j = 0; j < vision.working_w; j++)
// 		{
// 			int r, g, b, y_, u_, v_;
// 			y_ = ((int)*y) << 16;
// 			u_ = *u;
// 			v_ = *v;
// 			YUV_TO_RGB(y_, u_, v_, r, g, b)
// 			*y++ = r;
// 			*u++ = g;
// 			*v++ = b;
// 
// 			if(r > max_r) max_r = r;
// 			if(r < min_r) min_r = r;
// 			if(g > max_g) max_g = g;
// 			if(g < min_g) min_g = g;
// 			if(b > max_b) max_b = b;
// 			if(b < min_b) min_b = b;
// 		}
// 	}
// 
// 	if(max_r > min_r &&
// 		max_g > min_g &&
// 		max_b > min_b)
// 	{
// 		for(i = 0; i < vision.working_h; i++)
// 		{
// 			unsigned char *r = &vision.y_buffer[i * vision.working_w];
// 			unsigned char *g = &vision.u_buffer[i * vision.working_w];
// 			unsigned char *b = &vision.v_buffer[i * vision.working_w];
// 			for(j = 0; j < vision.working_w; j++)
// 			{
// 				*r = (*r - min_r) * 0xff / (max_r - min_r);
// 				*g = (*g - min_g) * 0xff / (max_g - min_g);
// 				*b = (*b - min_b) * 0xff / (max_b - min_b);
// 				r++;
// 				g++;
// 				b++;
// 			}
// 		}
// 	}
}


void chroma_key()
{
// Use center X pixels of current line as next keys
// Advance 1 line up to get next key
	int i, j;
	int key_y, key_r, key_g, key_b;
	float prev_x = vision.working_w / 2;
	bzero(vision.mask, vision.working_w * vision.working_h);

	for(key_y = vision.working_h - 1; key_y > vision.working_h / 2; key_y--)
	{
		float key_x1 = vision.working_w / 2;
		float key_x2 = key_x1 + 1;

//printf("detect_path %d frame=%d key_y=%d\n", __LINE__, vision.frames_written, key_y);
		if(key_y < vision.working_h - 1)
		{
// get average X of all masked pixels
			int accum = 0;
			int total = 0;
			for(i = 0; i < vision.working_w; i++)
			{
				if(vision.mask[key_y * vision.working_w + i])
				{
					accum += i;
					total++;
				}
			}

// no masked pixels.  Give up.
			if(total == 0)
			{
				break;
			}
			
			
//			key_x1 = accum / total - vision.working_w / 6;
//			key_x2 = key_x1 + vision.working_w / 3;

			key_x1 = accum / total;
			if(key_x1 - prev_x > MAX_DX)
			{
				key_x1 = prev_x + MAX_DX;
			}
			
			if(key_x1 - prev_x < -MAX_DX)
			{
				key_x1 = prev_x - MAX_DX;
			}
			
			key_x2 = key_x1 + 1;

			if(key_x1 < 0) key_x1 = 0;
			if(key_x2 > vision.working_w) key_x2 = vision.working_w;
		}
		
		prev_x = key_x1;
//printf("detect_path %d key_x1=%d key_x2=%d\n", __LINE__, key_x1, key_x2);


// use each pixel as a key
		int key_x;
		int key_y1 = key_y - SEARCH_H;
		int key_y2 = key_y;
		if(key_y1 < 0) key_y1 = 0;
		if(key_y2 < 1) key_y2 = 1;
		for(key_y = key_y1; key_y < key_y2; key_y++)
		{
			for(key_x = key_x1; key_x < key_x2; key_x++)
			{
				key_r = vision.y_buffer[key_y * vision.working_w + key_x];
				key_g = vision.u_buffer[key_y * vision.working_w + key_x];
				key_b = vision.v_buffer[key_y * vision.working_w + key_x];

	//printf("detect_path %d key_x=%d\n", __LINE__, key_x);
// mask based on the current key
				for(i = 0; i < vision.working_h; i++)
				{
					for(j = 0; j < vision.working_w; j++)
					{
						int offset = i * vision.working_w + j;
						
						if(!vision.mask[offset])
						{
							int r = vision.y_buffer[offset];
							int g = vision.u_buffer[offset];
							int b = vision.v_buffer[offset];

//							int distance = SQR(r - key_r);
 							int distance = sqrt(SQR(r - key_r) +
 								SQR(g - key_g) +
 								SQR(b - key_b));
							if(distance < threshold && 
								!vision.mask[i * vision.working_w + j])
							{
								vision.mask[i * vision.working_w + j] = 1;
							}
						}
					}
				}

// color searched pixels differently
//				vision.mask[key_y * vision.working_w + key_x] = 2;
//printf("detect_path %d\n", __LINE__);
			}
		}
	}
}


void to_output()
{
	int i, j;
#if 1
// copy working image to output image
	memcpy(vision.out_y, vision.y_buffer, vision.working_w * vision.working_h);
	memcpy(vision.out_u, vision.u_buffer, vision.working_w * vision.working_h);
	memcpy(vision.out_v, vision.v_buffer, vision.working_w * vision.working_h);

// 	for(i = 0; i < vision.working_h; i++)
// 	{
// 		unsigned char *r = &vision.y_buffer[i * vision.working_w];
// 		unsigned char *g = &vision.u_buffer[i * vision.working_w];
// 		unsigned char *b = &vision.v_buffer[i * vision.working_w];
// 		unsigned char *dst_y = &vision.out_y[i * vision.working_w];
// 		unsigned char *dst_u = &vision.out_u[i * vision.working_w];
// 		unsigned char *dst_v = &vision.out_v[i * vision.working_w];
// 
// 		for(j = 0; j < vision.working_w; j++)
// 		{
// 			int y, u, v;
// 			RGB_TO_YUV(y, u, v, *r, *g, *b)
// 			*dst_y++ = y;
// 			*dst_u++ = u;
// 			*dst_v++ = v;
// 			r++;
// 			g++;
// 			b++;
// 		}
// 	}
#endif // 0

#if 1
// overlay mask on output image
	for(i = 0; i < vision.working_h; i++)
	{
		for(j = 0; j < vision.working_w; j++)
		{
			int offset = i * vision.working_w + j;
			
			if(vision.mask[offset])
			{
// draw key pixel
				if(vision.mask[offset] == 2)
				{
					vision.out_y[offset] = 0xff;
					vision.out_u[offset] = 0x0;
					vision.out_v[offset] = 0xff;
				}
				else
// draw masked pixel
				if(vision.mask[offset] == 1)
				{
					vision.out_y[offset] = 0;
					vision.out_u[offset] = 0x80;
					vision.out_v[offset] = 0x80;
				}
				else
				{
					vision.out_y[offset] = 0xff;
					vision.out_u[offset] = 0x80;
					vision.out_v[offset] = 0x80;

//  				vision.out_y[offset] = vision.y_buffer[offset];
//  				vision.out_u[offset] = vision.u_buffer[offset];
//  				vision.out_v[offset] = vision.v_buffer[offset];
				}
			}
		}
	}
#endif // 0
}


void detect_path()
{
	int i, j, k, l;
	int best_distance = 0x7fffffff;
	int best_threshold = -1;
	for(threshold = vision.threshold1; threshold < vision.threshold2; threshold++)
	{
		vision.mask = vision.masks[threshold - vision.threshold1];
		chroma_key();
		int current_distance = detect_blobs(0);
		
		if(abs(current_distance - 40) < best_distance)
		{
			best_distance = abs(current_distance - 160);
			best_threshold = threshold;
		}
		
		to_output();
		compress_jpeg();
	}

	printf("detect_path %d best_threshold=%d\n", __LINE__, best_threshold);
// redo blob detection with best threshold
	threshold = best_threshold;
	vision.mask = vision.masks[threshold - vision.threshold1];
// reset mask values
	for(i = 0; i < vision.working_w * vision.working_h; i++)
	{
		if(vision.mask[i])
		{
			vision.mask[i] = 1;
		}
	}
	detect_blobs(1);



// Color average of all masked pixels
// 	for(i = 0; i < vision.working_h; i++)
// 	{
// 		int accum = 0;
// 		int total = 0;
// 		for(j = 0; j < vision.working_w; j++)
// 		{
// 			if(vision.mask[i * vision.working_w + j])
// 			{
// 				accum += j;
// 				total++;
// 			}
// 		}
// 		if(total > 0)
// 		{
// 			accum /= total;
// 			vision.mask[i * vision.working_w + accum] = 2;
// 		}
// 	}

	edge_detection();

// vertical edges
/*
 * 	for(i = 0; i < vision.working_w; i++)
 * 	{
 * 		int top_accum = 0;
 * 		int bottom_accum = 0;
 * 		unsigned char *column = vision.mask + i;
 * 		int *dst = vision.accum + i + EDGE_SIZE * vision.working_w;
 * 
 * // calculate 1st pixel 
 * 		for(j = 0; j < EDGE_SIZE; j++)
 * 		{
 * 			top_accum += column[j * vision.working_w];
 * 		}
 * 		
 * 		for(j = EDGE_SIZE; j < EDGE_SIZE * 2; j++)
 * 		{
 * 			bottom_accum += column[j * vision.working_w];
 * 		}
 * 		
 * 		for(j = EDGE_SIZE; j < vision.working_h - EDGE_SIZE; j++)
 * 		{
 * 			top_accum -= column[(j - EDGE_SIZE) * vision.working_w];
 * 			top_accum += column[j * vision.working_w];
 * 			bottom_accum -= column[j * vision.working_w];
 * 			bottom_accum += column[(j + EDGE_SIZE) * vision.working_w];
 * 			*dst += (bottom_accum - top_accum);
 * 			dst += vision.working_w;
 * 		}
 * 	}
 */


#if 0
// create output image from edge
	int min = 0x7fffffff;
	int max = -0x7fffffff;
	for(i = 0; i < vision.working_h; i++)
	{
		int *row = vision.accum + i * vision.working_w;
		for(j = 0; j < vision.working_w; j++)
		{
			if(*row > max)
			{
				max = *row;
			}
			
			if(*row < min)
			{
				min = *row;
			}

			row++;
		}
	}

	for(i = 0; i < vision.working_h; i++)
	{
		int *in_row = vision.accum + i * vision.working_w;
		unsigned char *out_y = vision.out_y + i * vision.working_w;
		unsigned char *out_u = vision.out_u + i * vision.working_w;
		unsigned char *out_v = vision.out_v + i * vision.working_w;
		for(j = 0; j < vision.working_w; j++)
		{
			*out_y = (*in_row - min) * 0xff / (max - min);
			*out_u = 0x80;
			*out_v = 0x80;
			out_y++;
			out_u++;
			out_v++;
			in_row++;
		}
	}
#endif // 0

	to_output();


// find edge of left side
// constrain search range to reasonable values
	int x1, y1, x2 = vision.working_w / 2, y2 = vision.working_h / 2;
	int left_x1, left_x2, left_y1, left_y2, left_diff = 0;
	for(y2 = vision.working_h / 4; y2 < vision.working_h * 3 / 4; y2++)
	{
		x1 = 0;
		for(y1 = y2 /* + vision.working_h / 4 */; y1 < vision.working_h; y1++)
		{
			int diff = test_line(x1, y1, x2, y2);

			if(diff > left_diff)
			{
				left_x1 = x1;
				left_x2 = x2;
				left_y1 = y1;
				left_y2 = y2;
				left_diff = diff;
			}
		}
		y1 = vision.working_h - 1;
		for(x1 = 0; x1 < vision.working_w / 2; x1++)
		{
			int diff = test_line(x1, y1, x2, y2);

			if(diff > left_diff)
			{
				left_x1 = x1;
				left_x2 = x2;
				left_y1 = y1;
				left_y2 = y2;
				left_diff = diff;
			}
		}
	}

// edge of right side
	x1 = vision.working_w / 2;
	int right_x1, right_x2, right_y1, right_y2, right_diff = 0;
	for(y1 = vision.working_h / 4; y1 < vision.working_h * 3 / 4; y1++)
	{
		x2 = vision.working_w;
		for(y2 = y1 + vision.working_h / 4; y2 < vision.working_h; y2++)
		{
			int diff = -test_line(x1, y1, x2, y2);
			
			if(diff > right_diff)
			{
				right_x1 = x1;
				right_x2 = x2;
				right_y1 = y1;
				right_y2 = y2;
				right_diff = diff;
			}
		}

		y2 = vision.working_h - 1;
		for(x2 = vision.working_w / 2; x2 < vision.working_w; x2++)
		{
			int diff = -test_line(x1, y1, x2, y2);
			
			if(diff > right_diff)
			{
				right_x1 = x1;
				right_x2 = x2;
				right_y1 = y1;
				right_y2 = y2;
				right_diff = diff;
			}
		}
	}

#if 1
// draw center of path
	if(left_x2 != left_x1 && 
		right_x2 != right_x1)
	{
		float left_slope = (float)(left_y2 - left_y1) / (left_x2 - left_x1);
		float right_slope = (float)(right_y2 - right_y1) / (right_x2 - right_x1);
		
		if(fabs(left_slope) > 0.001 && fabs(right_slope) > 0.001)
		{
			float left_intercept = -left_slope * left_x1 + left_y1;
			float right_intercept = -right_slope * right_x1 + right_y1;
			float vanish_x = (right_intercept - left_intercept) / (left_slope - right_slope);
			float vanish_y = left_slope * vanish_x + left_intercept;
			float bottom_left_x = ((vision.working_h - 1) - left_intercept) / left_slope;
			float bottom_right_x = ((vision.working_h - 1) - right_intercept) / right_slope;

		//	draw_line(vanish_x - 5, vanish_y, vanish_x + 5, vanish_y);
		//	draw_line(vanish_x, vanish_y - 5, vanish_x, vanish_y + 5);
			draw_line(vanish_x, vanish_y, (bottom_left_x + bottom_right_x) / 2, vision.working_h - 1);
		}
	}

	draw_line(left_x1, left_y1, left_x2, left_y2);
	draw_line(right_x1, right_y1, right_x2, right_y2);
#endif // 0
}





