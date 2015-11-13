#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vision.h"


// 640x480
// how many pixels above the key pixel to search for the next key
//#define SEARCH_H 4
// maximum shift in the key pixel X
//#define MAX_DX 1
// Edge detection kernel size
//#define EDGE_SIZE 16

// 160x120
//#define MAX_DX 1
//#define EDGE_SIZE 8

#undef CLAMP
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define TO_RAD(x) (((float)(x)) * 2 * M_PI / 360)
#define TO_DEG(x) ((x) * 360 /

int threshold;
int search_h;

void to_output(vision_engine_t *engine);
void geometry_triangles(vision_engine_t *engine);

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


void draw_key(vision_engine_t *engine)
{
	int i;
	// draw color key pixels
	for(i = 0; i < vision.working_w * vision.working_h; i++)
	{
		int key_x = engine->key_x[i];
		int key_y = engine->key_y[i];
		if(key_x < 0)
		{
			break;
		}

		if(key_x >= 0 && 
			key_x < vision.working_w && 
			key_y >= 0 && 
			key_y < vision.working_h)
		{
			engine->mask[key_y * vision.working_w + key_x] = 2;
		}
	}
}

int detect_blobs(vision_engine_t *engine, int fill_it)
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
			if(engine->mask[i * vision.working_w + j] == 1)
			{
				int total_filled = 0;
				int prev_filled = 0;
				
				engine->mask[i * vision.working_w + j] = 2;
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
							if(engine->mask[(y - 1) * vision.working_w + x] == 1)
							{
								engine->mask[(y - 1) * vision.working_w + x] = 2;
								fill_x[total_filled] = x;
								fill_y[total_filled] = y - 1;
								total_filled++;
							}
						}
						
						if(y < vision.working_h - 1)
						{
							if(engine->mask[(y + 1) * vision.working_w + x] == 1)
							{
								engine->mask[(y + 1) * vision.working_w + x] = 2;
								fill_x[total_filled] = x;
								fill_y[total_filled] = y + 1;
								total_filled++;
							}
						}
						
						if(x > 0)
						{
							if(engine->mask[y * vision.working_w + x - 1] == 1)
							{
								engine->mask[y * vision.working_w + x - 1] = 2;
								fill_x[total_filled] = x - 1;
								fill_y[total_filled] = y;
								total_filled++;
							}
						}
						
						if(x < vision.working_w - 1)
						{
							if(engine->mask[y * vision.working_w + x + 1] == 1)
							{
								engine->mask[y * vision.working_w + x + 1] = 2;
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
/*
 * 	{
 * 		int accum_x = 0, accum_y = 0;
 * 		for(i = 0; i < best_total_filled; i++)
 * 		{
 * 			accum_x += best_fill_x[i];
 * 			accum_y += best_fill_y[i];
 * 		}
 * 		accum_x /= best_total_filled;
 * 		accum_y /= best_total_filled;
 * 
 * 		for(i = 0; i < best_total_filled; i++)
 * 		{
 * 			distance_accum += sqrt(SQR(best_fill_x[i] - accum_x) + 
 * 				SQR(best_fill_y[i] - accum_y));
 * 		}
 * 		distance_accum /= best_total_filled;
 * 		printf("detect_blobs %d %d %d\n", __LINE__, threshold, distance_accum);
 * 	}
 */

// color only best blob
// TODO: store separate best_fill tables with each threshold
//	if(fill_it)
	{
		bzero(engine->mask, vision.working_w * vision.working_h);
		for(i = 0; i < best_total_filled; i++)
		{
			engine->mask[best_fill_y[i] * vision.working_w + best_fill_x[i]] = 1;
		}
		draw_key(engine);
	}

	free(fill_x);
	free(fill_y);
	free(best_fill_x);
	free(best_fill_y);
	
	return distance_accum;
}

int test_pixel(vision_engine_t *engine, int x, int y)
{
	if(x < 0 || y < 0 || x >= vision.working_w || y >= vision.working_h)
	{
		return 0;
	}


//printf("test_pixel x1=%d x2=%d y=%d %d\n", x1, x2, y, right_accum - left_accum);
	return *(engine->accum + x + y * vision.working_w);
}

int test_line(vision_engine_t *engine, int x1, int y1, int x2, int y2)
{
	int w = labs(x2 - x1);
	int h = labs(y2 - y1);
	int result = 0;


	if(!w && !h)
	{
		result = test_pixel(engine, x1, y1);
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
			int y = y1 + (i - x1) * numerator / denominator;
			result += test_pixel(engine, i, y);
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
			int x = x1 + (i - y1) * numerator / denominator;
			result += test_pixel(engine, x, i);
		}
	}

	return result;
}


// edge detection
void edge_detection(vision_engine_t *engine)
{
	int i, j;
	bzero(engine->accum, vision.working_h * vision.working_w * sizeof(int));
// horizontal edges
	for(i = 0; i < vision.working_h; i++)
	{
		int right_accum = 0;
		int left_accum = 0;
		unsigned char *row = engine->mask + i * vision.working_w;
		int *dst = engine->accum + i * vision.working_w + vision.edge_size;

// calculate 1st pixel 
		for(j = 0; j < vision.edge_size; j++)
		{
			left_accum += (row[j] ? 1 : 0);
		}
		
		for(j = vision.edge_size; j < vision.edge_size * 2; j++)
		{
			right_accum += (row[j] ? 1 : 0);
		}
		
		for(j = vision.edge_size; j < vision.working_w - vision.edge_size; j++)
		{
			left_accum -= (row[j - vision.edge_size] ? 1 : 0);
			left_accum += (row[j] ? 1 : 0);
			right_accum -= (row[j] ? 1 : 0);
			right_accum += (row[j + vision.edge_size] ? 1 : 0);
			*(dst++) = right_accum - left_accum;
		}
	}


// vertical edges
/*
 * 	for(i = 0; i < vision.working_w; i++)
 * 	{
 * 		int top_accum = 0;
 * 		int bottom_accum = 0;
 * 		unsigned char *column = vision.mask + i;
 * 		int *dst = vision.accum + i + vision.edge_size * vision.working_w;
 * 
 * // calculate 1st pixel 
 * 		for(j = 0; j < vision.edge_size; j++)
 * 		{
 * 			top_accum += column[j * vision.working_w];
 * 		}
 * 		
 * 		for(j = vision.edge_size; j < vision.edge_size * 2; j++)
 * 		{
 * 			bottom_accum += column[j * vision.working_w];
 * 		}
 * 		
 * 		for(j = vision.edge_size; j < vision.working_h - vision.edge_size; j++)
 * 		{
 * 			top_accum -= column[(j - vision.edge_size) * vision.working_w];
 * 			top_accum += column[j * vision.working_w];
 * 			bottom_accum -= column[j * vision.working_w];
 * 			bottom_accum += column[(j + vision.edge_size) * vision.working_w];
 * 			*dst += (bottom_accum - top_accum);
 * 			dst += vision.working_w;
 * 		}
 * 	}
 */

}


#if 0
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
#endif // 0



// chroma key based on a single color
void chroma_key_single(vision_engine_t *engine, int key_r, int key_g, int key_b)
{
// mask based on the current color
	int i, j;
	for(i = 0; i < vision.working_h; i++)
	{
		int offset = i * vision.working_w;
		unsigned char *mask_row = engine->mask + offset;
		for(j = 0; j < vision.working_w; j++)
		{
			if(!(*mask_row))
			{
				int r = engine->in_y[offset];
				int g = engine->in_u[offset];
				int b = engine->in_v[offset];

//				int distance = SQR(r - key_r);
 				int distance = sqrt(SQR(r - key_r) +
 					SQR(g - key_g) +
 					SQR(b - key_b));
				if(distance < threshold)
// sqrt is too slow & unnecessary for small thresholds
//				if(abs(r - key_r) < threshold && 
//					abs(g - key_g) < threshold &&
//					abs(b - key_b) < threshold)
				{
					*mask_row = 1;
				}
			}

			mask_row++;
			offset++;
		}
	}
}

void chroma_key_history(vision_engine_t *engine)
{
#if (HISTORY_SIZE > 0)
	int i;
	for(i = 0; i < HISTORY_SIZE; i++)
	{
		if(engine->r_keys[i] != 0)
		{
			chroma_key_single(engine, 
				engine->r_keys[i], 
				engine->g_keys[i], 
				engine->b_keys[i]);
		}
	}
#endif
}

// chroma key based on a range of colors
void chroma_key_rect(vision_engine_t *engine)
{
	int i, j;
	int key_y, key_r, key_g, key_b;
	float prev_x = vision.working_w / 2;
	int key_index = 0;

	int x1 = vision.working_w / 2 - vision.working_w / 8;
	int x2 = vision.working_w / 2 + vision.working_w / 8;
	int y1 = vision.working_h * 2 / 3;
	int y2 = vision.working_h;
	
	for(i = y1; i < y2; i += 1)
	{
		for(j = x1; j < x2; j += 1)
		{
			key_r = engine->in_y[i * vision.working_w + j];
			key_g = engine->in_u[i * vision.working_w + j];
			key_b = engine->in_v[i * vision.working_w + j];


// mask based on the current key
			chroma_key_single(engine, key_r, key_g, key_b);

// store the current key in history
#if (HISTORY_SIZE > 0)
			if(!(engine->total_frames % 3))
			{
				engine->r_keys[engine->key_ptr] = key_r;
				engine->g_keys[engine->key_ptr] = key_g;
				engine->b_keys[engine->key_ptr] = key_b;
				engine->key_ptr++;
				if(engine->key_ptr >= HISTORY_SIZE)
				{
					engine->key_ptr = 0;
				}
			}
#endif

		}
	}
	
// color searched pixels for debugging
	for(i = x1; i < x2; i++)
	{
		engine->key_x[key_index] = i;
		engine->key_y[key_index] = y1;
		key_index++;
		engine->key_x[key_index] = i;
		engine->key_y[key_index] = y2;
		key_index++;
	}
	
	for(i = y1; i < y2; i++)
	{
		engine->key_x[key_index] = x1;
		engine->key_y[key_index] = i;
		key_index++;
		engine->key_x[key_index] = x2;
		engine->key_y[key_index] = i;
		key_index++;
	}
}

// chroma key based on a range of colors
void chroma_key_triangle(vision_engine_t *engine)
{
	int i, j;
	int key_y, key_r, key_g, key_b;
	float prev_x = vision.working_w / 2;
	int key_index = 0;

	int top_x = vision.working_w / 2;
	int top_y = vision.working_h * 2 / 3;
	int left_x = vision.working_w / 6;
	int right_x = vision.working_w * 5 / 6;

	float left_slope = (float)(top_y - vision.working_h) / (top_x - left_x);
	float right_slope = (float)(vision.working_h - top_y) / (right_x - top_x);

	for(i = top_y; i < vision.working_h; i += 2)
	{
// (y - b) / m = x
		int x1 = (int)((i - top_y) / left_slope + top_x);
		int x2 = (int)((i - top_y) / right_slope + top_x);
		for(j = x1; j < x2; j += 2)
		{
			key_r = engine->in_y[i * vision.working_w + j];
			key_g = engine->in_u[i * vision.working_w + j];
			key_b = engine->in_v[i * vision.working_w + j];


// mask based on the current key
			chroma_key_single(engine, key_r, key_g, key_b);

// color searched pixels for debugging
			engine->key_x[key_index] = j;
			engine->key_y[key_index] = i;
			key_index++;
		}
	}
}



void chroma_key_mean(vision_engine_t *engine)
{
// Use center X pixels of current line as next keys
// Advance 1 line up to get next key
	int i, j;
	int key_y, key_r, key_g, key_b;
	float prev_x = vision.working_w / 2;
	int key_index = 0;


	for(key_y = vision.working_h - 1; key_y > vision.horizon_y; key_y--)
	{
		float key_x1 = vision.working_w / 2;
		float key_x2 = key_x1 + 1;

//printf("detect_path %d frame=%d key_y=%d\n", __LINE__, vision.frames_written, key_y);
		if(key_y < vision.working_h - 1)
		{

// get average X of all masked pixels
			int accum = 0;
			int total = 0;
			unsigned char *row = engine->mask + key_y * vision.working_w;
			for(i = 0; i < vision.working_w; i++)
			{
				if(row[i])
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
			

			key_x1 = accum / total;

 			if(key_x1 - prev_x > vision.max_dx)
  			{
  				key_x1 = prev_x + vision.max_dx;
  			}
  			
  			if(key_x1 - prev_x < -vision.max_dx)
  			{
  				key_x1 = prev_x - vision.max_dx;
  			}
  		
 

			key_x2 = key_x1 + 1;

			if(key_x1 < 0) key_x1 = 0;
			if(key_x2 > vision.working_w) key_x2 = vision.working_w;
		}
		
		prev_x = key_x1;
//printf("detect_path %d key_x1=%d key_x2=%d\n", __LINE__, key_x1, key_x2);


// use each pixel as a key
		int key_x;
		int key_y1 = key_y - search_h;
		int key_y2 = key_y;
// search every y
		int search_step = 1;
// Only do 2 searches
//		int search_step = search_h - 1;
		if(search_step <= 0) search_step = 1;
		
		if(key_y1 < 0) key_y1 = 0;
		if(key_y2 < 1) key_y2 = 1;
// overwrites key_y here
		for(key_y = key_y1; key_y < key_y2; key_y += search_step)
		{
			for(key_x = key_x1; key_x < key_x2; key_x++)
			{
				key_r = engine->in_y[key_y * vision.working_w + key_x];
				key_g = engine->in_u[key_y * vision.working_w + key_x];
				key_b = engine->in_v[key_y * vision.working_w + key_x];


// mask based on the current key
				chroma_key_single(engine, key_r, key_g, key_b);
			}
		}

// color searched pixels for debugging
		engine->key_x[key_index] = key_x1;
		engine->key_y[key_index] = key_y1;
		key_index++;

// reset key_y
		key_y = key_y2;
	}



}

// Use single line from center to assumed vanishing point for keys
void chroma_key_ray(vision_engine_t *engine, int top_x)
{
	int key_y, key_r, key_g, key_b;
	int key_index = 0;

	bzero(engine->mask, vision.working_w * vision.working_h);
	memset(engine->key_x, 0xff, vision.working_w * vision.working_h * sizeof(int));
	memset(engine->key_y, 0xff, vision.working_w * vision.working_h * sizeof(int));

	int key_y1 = vision.working_h / 2;
	for(key_y = vision.working_h - 1; key_y >= key_y1; key_y--)
	{
		float fraction = (float)(vision.working_h - key_y) / 
			(vision.working_h - key_y1);
		float key_x1 = top_x * fraction + 
			vision.working_w / 2 * (1.0f - fraction);
		float key_x2 = key_x1 + 1;

		
//printf("detect_path %d key_x1=%d key_y=%d\n", __LINE__, (int)key_x1, key_y);


// use each pixel as a key
		int key_x;
// search every y
		int search_step = 1;
// Only do 2 searches
//		int search_step = search_h - 1;
		if(search_step <= 0) search_step = 1;
		
// overwrites key_y here
		for(key_x = key_x1; key_x < key_x2; key_x++)
		{
			key_r = engine->in_y[key_y * vision.working_w + key_x];
			key_g = engine->in_u[key_y * vision.working_w + key_x];
			key_b = engine->in_v[key_y * vision.working_w + key_x];


// mask based on the current key
			chroma_key_single(engine, key_r, key_g, key_b);

// color searched pixels for debugging
			engine->key_x[key_index] = key_x;
			engine->key_y[key_index] = key_y;
			key_index++;
//printf("detect_path %d\n", __LINE__);
		}

	}



}

void chroma_key_rays(vision_engine_t *engine)
{
	int i;
	for(i = 0; i < vision.working_w; i += 4)
	{
		chroma_key_ray(engine, i);
		detect_blobs(engine, 1);
//		edge_detection(engine);
		
		draw_key(engine);
		to_output(engine);
		
//		geometry_triangles(engine);

		
		compress_output(engine->out_y, 
 			engine->out_u,
			engine->out_v,
 			vision.output_w,
 			vision.output_h);
		append_file(vision.picture_out, vision.out_size);
	}

}



void to_output(vision_engine_t *engine)
{
	int i, j;



#if 0
// create output image from edge
	int min = 0x7fffffff;
	int max = -0x7fffffff;
	for(i = 0; i < vision.working_h; i++)
	{
		int *row = engine->accum + i * vision.working_w;
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
		int *in_row = engine->accum + i * vision.working_w;
		unsigned char *out_y = engine->out_y + i * vision.working_w;
		unsigned char *out_u = engine->out_u + i * vision.working_w;
		unsigned char *out_v = engine->out_v + i * vision.working_w;
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

#if 1
// copy working image to output image
	memcpy(engine->out_y, engine->in_y, vision.working_w * vision.working_h);
	memcpy(engine->out_u, engine->in_u, vision.working_w * vision.working_h);
	memcpy(engine->out_v, engine->in_v, vision.working_w * vision.working_h);

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


// overlay mask on output image
	for(i = 0; i < vision.working_h; i++)
	{
		unsigned char *mask_row = engine->mask + i * vision.working_w;
		unsigned char *y = engine->out_y + i * vision.working_w;
		unsigned char *u = engine->out_u + i * vision.working_w;
		unsigned char *v = engine->out_v + i * vision.working_w;

		for(j = 0; j < vision.working_w; j++)
		{
			if(*mask_row)
			{
// draw key pixel
				if(*mask_row == 2)
				{
					*y = 0x80 + *y / 2;
					*u = 0x0 + *u / 2;
					*v = 0x80 + *v / 2;
				}
				else
// draw masked pixel
				if(*mask_row == 1)
				{
					*y = *y / 2;
					*u = 0x80 + *u / 2;
					*v = 0x0 + *v / 2;
				}
			}
			
			mask_row++;
			y++;
			u++;
			v++;
		}
	}
#endif // 0

}

// convert angle, y into xy coords
void param_to_xy(int *x1_out, 
	int *y1_out, 
	int *x2_out, 
	int *y2_out, 
	int y, 
	float angle)
{
	float slope = tan(angle);
	float y1 = y - slope * vision.working_w / 2;
	float y2 = y + slope * vision.working_w / 2;
	float x1 = 0;
	float x2 = vision.working_w;

	if(y1 < 0)
	{
		y1 = 0;
// (y - b) / m = x
		x1 = -y / slope + vision.working_w / 2;
	}
	
	if(y1 > vision.working_h)
	{
		y1 = vision.working_h;
		x1 = (vision.working_h - y) / slope + vision.working_w / 2;
	}

	if(y2 < 0)
	{
		y2 = 0;
		x2 = -y / slope + vision.working_w / 2;
	}

	if(y2 > vision.working_h)
	{
		y2 = vision.working_h;
		x2 = (vision.working_h - y) / slope + vision.working_w / 2;
	}

	CLAMP(x1, 0, vision.working_w);
	CLAMP(x2, 0, vision.working_w);
	CLAMP(y1, 0, vision.working_h);
	CLAMP(y2, 0, vision.working_h);

	*x1_out = x1;
	*y1_out = y1;
	*x2_out = x2;
	*y2_out = y2;
}

void geometry_range_polar(vision_engine_t *engine,
	int y_step, 
	float a_step, 
	int y_center, 
	int y_range, 
	float a_center, 
	float a_range,
	int *best_y,
	float *best_angle,
	int *best_diff,
	int want_greatest)
{
	int y;
	for(y = y_center - y_range; y < y_center + y_range; y += y_step)
	{
		float angle;
		for(angle = a_center - a_range; angle < a_center + a_range; angle += a_step)
		{
			int x1, y1, x2, y2;
			param_to_xy(&x1, 
				&y1, 
				&x2, 
				&y2, 
				y, 
				angle);
			
			int diff = test_line(engine, x1, y1, x2, y2);
			if((want_greatest && diff > *best_diff) ||
				(!want_greatest && diff < *best_diff))
			{
				*best_y = y;
				*best_angle = angle;
				*best_diff = diff;
			}
			
// printf("geometry_range %d y=%d angle=%d %d %d %d %d\n", 
// __LINE__, 
// y, 
// (int)TO_DEG(angle), 
// (int)x1, 
// (int)y1, 
// (int)x2, 
// (int)y2);
//draw_line(engine, x1, y1, x2, y2);
		}
//return;
	}
}

void geometry_range_xy(vision_engine_t *engine,
	int y_step, 
	int x_step, 
	int y_center, 
	int y_range, 
	int x_center, 
	int x_range,
	int *best_y,
	int *best_x,
	int *best_diff,
	int want_greatest)
{
	int y;
	for(y = y_center - y_range; y < y_center + y_range; y += y_step)
	{
		int x;
		for(x = x_center - x_range; x < x_center + x_range; x += x_step)
		{
			int x1, y1, x2, y2;
			if(want_greatest)
			{
// left edge
				x1 = 0;
				y1 = y;
				x2 = x;
				y2 = vision.working_h / 2;
			}
			else
			{
// right edge
				x1 = x;
				y1 = vision.working_h / 2;
				x2 = vision.working_w;
				y2 = y;
			}
			
			int diff = test_line(engine, x1, y1, x2, y2);
			if((want_greatest && diff > *best_diff) ||
				(!want_greatest && diff < *best_diff))
			{
				*best_y = y;
				*best_x = x;
				*best_diff = diff;
			}
			
// printf("geometry_range %d y=%d angle=%d %d %d %d %d\n", 
// __LINE__, 
// y, 
// (int)TO_DEG(angle), 
// (int)x1, 
// (int)y1, 
// (int)x2, 
// (int)y2);
//draw_line(engine, x1, y1, x2, y2);
		}
//return;
	}
}



void geometry_xy(vision_engine_t *engine)
{
	int y_step;
	int y_center = vision.working_h * 3 / 4;
	int y_range = vision.working_h / 4;
	int x_step;
	int x_center = vision.working_w / 2;
	int x_range = vision.working_w / 4;
	int best_y = 0;
	int best_x = 0;
	int best_left_diff = -1;

	int left_x1, left_x2, left_y1, left_y2;
	int right_x1, right_x2, right_y1, right_y2;

// right edge
	for(y_step = 4, x_step = 4; y_step > 0; y_step >>= 1, x_step >>= 1)
	{
		geometry_range_xy(engine, 
			y_step, 
			x_step, 
			y_center,
			y_range, 
			x_center, 
			x_range,
			&best_y,
			&best_x,
			&best_left_diff,
			0);

		y_center = best_y;
		x_center = best_x;
		y_range = y_step;
		x_range = x_step;
		y_step /= 2;
		x_step /= 2;
	}

	right_x1 = x_center;
	right_y1 = vision.working_h / 2;
	right_x2 = vision.working_w;
	right_y2 = y_center;

// left edge
	y_center = vision.working_h * 3 / 4;
	y_range = vision.working_h / 4;
	x_step;
	x_center = vision.working_w / 2;
	x_range = vision.working_w / 4;
	best_y = 0;
	best_x = 0;
	int best_right_diff = 1;
	for(y_step = 4, x_step = 4; y_step > 0; y_step >>= 1, x_step >>= 1)
	{
		geometry_range_xy(engine, 
			y_step, 
			x_step, 
			y_center,
			y_range, 
			x_center, 
			x_range,
			&best_y,
			&best_x,
			&best_right_diff,
			1);

		y_center = best_y;
		x_center = best_x;
		y_range = y_step;
		x_range = x_step;
		y_step /= 2;
		x_step /= 2;
	}

	left_x1 = 0;
	left_y1 = y_center;
	left_x2 = x_center;
	left_y2 = vision.working_h / 2;




// draw path
	if(left_x2 != left_x1 && 
		right_x2 != right_x1)
	{
		float left_slope = (float)(left_y2 - left_y1) / (left_x2 - left_x1);
		float right_slope = (float)(right_y2 - right_y1) / (right_x2 - right_x1);
		
		if(fabs(left_slope) > 0.001 && 
			fabs(right_slope) > 0.001 && 
			fabs(left_slope - right_slope) > 0.001)
		{
			float vanish_x = 0;
			float vanish_y = 0;
			float left_intercept = -left_slope * left_x1 + left_y1;
			float right_intercept = -right_slope * right_x1 + right_y1;
			vanish_x = (right_intercept - left_intercept) / (left_slope - right_slope);
			vanish_y = left_slope * vanish_x + left_intercept;
			float bottom_left_x = ((vision.working_h - 1) - left_intercept) / left_slope;
			float bottom_right_x = ((vision.working_h - 1) - right_intercept) / right_slope;
			float bottom_x = (bottom_left_x + bottom_right_x) / 2;


	 		engine->vanish_x = (int)vanish_x;
			engine->vanish_y = (int)vanish_y;
			engine->bottom_x = (int)bottom_x;


//			if(vanish_y < vision.working_h / 2 + vision.working_h / 8 && 
//				vanish_y > vision.working_h / 2 - vision.working_h / 8 &&
//				vanish_x > vision.working_w * 1 / 3 &&
//				vanish_x < vision.working_w * 2 / 3)
// filtering is done in the frame writer
			if(1)
			{
				draw_line(engine, vanish_x, vanish_y, bottom_x, vision.working_h - 1);
				draw_line(engine, left_x1, left_y1, vanish_x, vanish_y);
				draw_line(engine, vanish_x, vanish_y, right_x2, right_y2);
 
				engine->failed = 0;
			}
			else
			{
				engine->failed = 1;
			}
		}
	}

}





void geometry_triangles(vision_engine_t *engine)
{
	int top_x;
	int bottom_x;
	double best_diff = 0;
	int best_top_x = 0;
	int best_bottom_x = 0;
	int top_y = vision.working_h / 2;
	int bottom_w = vision.working_w * 2;
	int bottom_y = vision.working_h;

	for(top_x = vision.working_w * 1 / 4; 
		top_x < vision.working_w * 3 / 4; 
		top_x++)
	{
		for(bottom_x = 0; bottom_x < vision.working_w; bottom_x++)
		{
			int bottom_x1 = bottom_x - bottom_w / 2;
			int bottom_x2 = bottom_x + bottom_w / 2;
			int diff = test_line(engine, 
				bottom_x1, 
				bottom_y, 
				top_x, 
				top_y);
			diff -= test_line(engine, 
				top_x, 
				top_y, 
				bottom_x2, 
				bottom_y);


			if(best_diff < diff)
			{
				best_diff = diff;
				best_top_x = top_x;
				best_bottom_x = bottom_x;
			}
		}
	}
	
	draw_line(engine, best_top_x, top_y, best_bottom_x - bottom_w / 2, bottom_y);
	draw_line(engine, best_top_x, top_y, best_bottom_x + bottom_w / 2, bottom_y);
	
	
}


void geometry_polar(vision_engine_t *engine)
{
	int y_step;
	int y_center = vision.working_h / 2;
	int y_range = vision.working_h / 4;
	float a_step = TO_RAD(4);
	float a_center = TO_RAD(45);
	float a_range = TO_RAD(35);
	int best_y = 0;
	float best_angle = 0;
	int best_diff = -1;

	int left_x1, left_x2, left_y1, left_y2;
	int right_x1, right_x2, right_y1, right_y2;

// right edge
	for(y_step = 4; y_step > 0; y_step >>= 1)
	{
		geometry_range_polar(engine, 
			y_step, 
			a_step, 
			y_center,
			y_range, 
			a_center, 
			a_range,
			&best_y,
			&best_angle,
			&best_diff,
			0);

		y_center = best_y;
		a_center = best_angle;
		y_range = y_step;
		a_range = a_step;
		y_step /= 2;
		a_step /= 2;
	}

	param_to_xy(&right_x1, 
		&right_y1, 
		&right_x2, 
		&right_y2, 
		best_y, 
		best_angle);

// left edge
	y_center = vision.working_h / 2;
	y_range = vision.working_h / 4;
	a_step = TO_RAD(4);
	a_center = TO_RAD(-45);
	a_range = TO_RAD(35);
	best_y = 0;
	best_angle = 0;
	best_diff = 1;
	for(y_step = 4; y_step > 0; y_step >>= 1)
	{
		geometry_range_polar(engine, 
			y_step, 
			a_step, 
			y_center,
			y_range, 
			a_center, 
			a_range,
			&best_y,
			&best_angle,
			&best_diff,
			1);

		y_center = best_y;
		a_center = best_angle;
		y_range = y_step;
		a_range = a_step;
		y_step /= 2;
		a_step /= 2;
	}

	param_to_xy(&left_x1, 
		&left_y1, 
		&left_x2, 
		&left_y2, 
		best_y, 
		best_angle);

// draw path
	if(left_x2 != left_x1 && 
		right_x2 != right_x1)
	{
		float left_slope = (float)(left_y2 - left_y1) / (left_x2 - left_x1);
		float right_slope = (float)(right_y2 - right_y1) / (right_x2 - right_x1);
		
		if(fabs(left_slope) > 0.001 && 
			fabs(right_slope) > 0.001 && 
			fabs(left_slope - right_slope) > 0.001)
		{
			float vanish_x = 0;
			float vanish_y = 0;
			float left_intercept = -left_slope * left_x1 + left_y1;
			float right_intercept = -right_slope * right_x1 + right_y1;
			vanish_x = (right_intercept - left_intercept) / (left_slope - right_slope);
			vanish_y = left_slope * vanish_x + left_intercept;
			float bottom_left_x = ((vision.working_h - 1) - left_intercept) / left_slope;
			float bottom_right_x = ((vision.working_h - 1) - right_intercept) / right_slope;
			float bottom_x = (bottom_left_x + bottom_right_x) / 2;
			
	 		engine->vanish_x = (int)vanish_x;
			engine->vanish_y = (int)vanish_y;
			engine->bottom_x = (int)bottom_x;

/*
 * 			if(vanish_y < vision.working_h / 2 + vision.working_h / 8 && 
 * 				vanish_y > vision.working_h / 2 - vision.working_h / 8 &&
 * 				vanish_x > vision.working_w * 1 / 3 &&
 * 				vanish_x < vision.working_w * 2 / 3)
 */
// filtering is done in the frame writer
			if(1)
			{
				draw_line(engine, vanish_x, vanish_y, bottom_x, vision.working_h - 1);
				draw_line(engine, left_x1, left_y1, vanish_x, vanish_y);
				draw_line(engine, vanish_x, vanish_y, right_x2, right_y2);
				engine->failed = 0;
			}
			else
			{
				engine->failed = 1;
			}

		}
	}

}








void detect_path(vision_engine_t *engine)
{
	int i, j, k, l;
	cartimer_t profiler;
	reset_timer(&profiler);
	
	


	threshold = vision.threshold;
	search_h = vision.search_h;


	bzero(engine->mask, vision.working_w * vision.working_h);
	memset(engine->key_x, 0xff, vision.working_w * vision.working_h * sizeof(int));
	memset(engine->key_y, 0xff, vision.working_w * vision.working_h * sizeof(int));

 	chroma_key_rect(engine);
	chroma_key_mean(engine);
//	chroma_key_rays(engine);
//	chroma_key_history(engine);

//	printf("detect_path %d %d\n", __LINE__, get_timer_difference(&profiler));
//	reset_timer(&profiler);

	detect_blobs(engine, 1);

//	printf("detect_path %d %d\n", __LINE__, get_timer_difference(&profiler));
//	reset_timer(&profiler);


	edge_detection(engine);

//	printf("detect_path %d %d\n", __LINE__, get_timer_difference(&profiler));
//	reset_timer(&profiler);


	to_output(engine);

//	printf("detect_path %d %d\n", __LINE__, get_timer_difference(&profiler));
//	reset_timer(&profiler);

//	geometry_xy(engine);
	geometry_polar(engine);
//	geometry_triangles(engine);

//	printf("detect_path %d %d\n", __LINE__, get_timer_difference(&profiler));
//	reset_timer(&profiler);
	engine->total_frames++;
}



