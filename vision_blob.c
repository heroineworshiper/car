#include "vision.h"

// detect path through chroma keying & blob fill


void detect_path()
{
	int i, j, k;


// blob fill the path to detect the horizon
	int chroma_x1 = 25 * vision.image_w / 100;
	int chroma_x2 = 60 * vision.image_w / 100;
	int chroma_y1 = 55 * vision.image_h / 100;
	int chroma_y2 = 100 * vision.image_h / 100;
	int accum_y = 0, accum_u = 0, accum_v = 0;
	int total_accum = (chroma_y2 - chroma_y1) * (chroma_x2 - chroma_x1);

	int fill_x[vision.image_w * vision.image_h];
	int fill_y[vision.image_w * vision.image_h];
	int total_filled = 0;
	int prev_filled = 0;
	int threshold = 8;


	for(i = chroma_y1; i < chroma_y2; i++)
	{
		unsigned char *y_row = vision.y_buffer + i * vision.image_w + chroma_x1;
		unsigned char *u_row = vision.u_buffer + i * vision.image_w + chroma_x1;
		unsigned char *v_row = vision.v_buffer + i * vision.image_w + chroma_x1;
		for(j = chroma_x1; j < chroma_x2; j++)
		{
			accum_y += *y_row++;
			accum_u += *u_row++;
			accum_v += *v_row++;
		}
	}
	
	accum_y /= total_accum;
	accum_u /= total_accum;
	accum_v /= total_accum;

// get threshold
	threshold = 0;
	for(i = chroma_y1; i < chroma_y2; i++)
	{
		unsigned char *y_row = vision.y_buffer + i * vision.image_w + chroma_x1;
		unsigned char *u_row = vision.u_buffer + i * vision.image_w + chroma_x1;
		unsigned char *v_row = vision.v_buffer + i * vision.image_w + chroma_x1;
		for(j = chroma_x1; j < chroma_x2; j++)
		{
			int diff = sqrt(SQR(*y_row - accum_y) +
				SQR(*u_row - accum_u) +
				SQR(*v_row - accum_v));
			if(diff > threshold) threshold = diff;
		}
	}

//printf("detect_path %d %d %d %d\n", __LINE__, accum_y, accum_u, accum_v);
	bzero(vision.mask, vision.image_w * vision.image_h);
	for(i = chroma_y1; i < chroma_y2; i++)
	{
		unsigned char *y_row = vision.y_buffer + i * vision.image_w + chroma_x1;
		unsigned char *u_row = vision.u_buffer + i * vision.image_w + chroma_x1;
		unsigned char *v_row = vision.v_buffer + i * vision.image_w + chroma_x1;


		for(j = chroma_x1; j < chroma_x2; j++)
		{
			int diff = sqrt(SQR(*y_row - accum_y) +
				SQR(*u_row - accum_u) +
				SQR(*v_row - accum_v));
//printf("detect_path %d %d\n", __LINE__, diff);
			if(diff < threshold)
			{
				vision.mask[i * vision.image_w + j] = 1;
				fill_x[total_filled] = j;
				fill_y[total_filled] = i;
				total_filled++;
			}
			y_row++;
			u_row++;
			v_row++;
		}
	}

//printf("detect_path %d %d\n", __LINE__, total_filled);
	while(total_filled > prev_filled)
	{
		int start = prev_filled;
		prev_filled = total_filled;
		for(i = start; i < prev_filled; i++)
		{
			int x = fill_x[i];
			int y = fill_y[i];
			unsigned char *y_row = vision.y_buffer + y * vision.image_w + x;
			unsigned char *u_row = vision.u_buffer + y * vision.image_w + x;
			unsigned char *v_row = vision.v_buffer + y * vision.image_w + x;


			if(y > 0 && vision.mask[(y - 1) * vision.image_w + x] == 0)
			{
				int diff = sqrt(SQR(*(y_row - vision.image_w) - accum_y) +
					SQR(*(u_row - vision.image_w) - accum_u) +
					SQR(*(v_row - vision.image_w) - accum_v));
				if(diff < threshold)
				{
					vision.mask[(y - 1) * vision.image_w + x] = 1;
					fill_x[total_filled] = x;
					fill_y[total_filled] = y - 1;
					total_filled++;
				}
			}

			if(y < vision.image_h - 1 && vision.mask[(y + 1) * vision.image_w + x] == 0)
			{
				int diff = sqrt(SQR(*(y_row + vision.image_w) - accum_y) +
					SQR(*(u_row + vision.image_w) - accum_u) +
					SQR(*(v_row + vision.image_w) - accum_v));
				if(diff < threshold)
				{
					vision.mask[(y + 1) * vision.image_w + x] = 1;
					fill_x[total_filled] = x;
					fill_y[total_filled] = y + 1;
					total_filled++;
				}
			}

			if(x > 0 && vision.mask[y * vision.image_w + x - 1] == 0)
			{
				int diff = sqrt(SQR(*(y_row - 1) - accum_y) +
					SQR(*(u_row - 1) - accum_u) +
					SQR(*(v_row - 1) - accum_v));
				if(diff < threshold)
				{
					vision.mask[y * vision.image_w + x - 1] = 1;
					fill_x[total_filled] = x - 1;
					fill_y[total_filled] = y;
					total_filled++;
				}
			}


			if(x < vision.image_w - 1 && vision.mask[y * vision.image_w + x + 1] == 0)
			{
				int diff = sqrt(SQR(*(y_row + 1) - accum_y) +
					SQR(*(u_row + 1) - accum_u) +
					SQR(*(v_row + 1) - accum_v));
				if(diff < threshold)
				{
					vision.mask[y * vision.image_w + x + 1] = 1;
					fill_x[total_filled] = x + 1;
					fill_y[total_filled] = y;
					total_filled++;
				}
			}
		}
	}

// get horizon
//	int x1 = vision.top_x1 * vision.image_w / 100;
//	int x2 = vision.top_x2 * vision.image_w / 100;
	int x1 = 0;
	int x2 = vision.image_w;
	int y1 = 0;
	int y2 = 0;

	int got_it = 0;
	for(y1 = 0; y1 < vision.image_h && !got_it; y1++)
	{
		int total = 0;
		for(i = x1; i < x2 && !got_it; i++)
		{
			if(vision.mask[y1 * vision.image_w + i] == 1)
			{
				total++;
				if(total > 8)
				{
					y2 = y1 + 1;
					got_it = 1;
				}
			}
		}
		if(got_it) break;
	}



#if 0
// detect vanishing point
	int vanish_x, vanish_y;

	int best_vanish_x = vision.image_w / 2;
	int best_vanish_y = vision.image_h / 2;
	int best_accum = 0;

// detect horizon
	bzero(vision.accum, vision.image_w * vision.image_h * sizeof(int));
	for(vanish_y = y1; vanish_y < y2; vanish_y += 1)
	{

// line ends at vanishing point
		for(vanish_x = x1; vanish_x < x2; vanish_x += 1)
		{
// calculate average difference between all lines going to vanishing point
			int64_t accum = 0;
			int table_entry = 0;
			int prev_value = 0;
			int first_value = 1;

// lines must be adjacent
// left edge	
// 	 		for(i = 0; i <= vision.image_h; i += 8)
// 	 		{
// 	 			int new_value = line_color(0, i, vanish_x, vanish_y);
// 	 			if(!first_value)
// 	 			{
// 	 				accum += SQR(new_value - prev_value);
// 	 			}
// 	 			prev_value = new_value;
// 	 			first_value = 0;
// 	 		}


// bottom edge
			for(i = 0; i <= vision.image_w; i++)
			{
				int new_value = line_color(i, vision.image_h, vanish_x, vanish_y);
				if(!first_value)
				{
					accum += SQR(new_value - prev_value);
				}
				prev_value = new_value;
				first_value = 0;
			}

// right edge
//  			for(i = vision.image_h; i >= 0; i -= 8)
//  			{
//  				int new_value = line_color(vision.image_w, i, vanish_x, vanish_y);
//  				if(!first_value)
//  				{
//  					accum += SQR(new_value - prev_value);
//  				}
//  				prev_value = new_value;
//  				first_value = 0;
//  			}

// top edge
// 			for(i = vision.image_w; i >= 0; i--)
// 			{
// 				int new_value = line_color(i, 0, vanish_x, vanish_y);
// 				if(!first_value)
// 				{
// 					accum += SQR(new_value - prev_value);
// 				}
// 				prev_value = new_value;
// 				first_value = 0;
// 			}

// point with the most differences between adjacent lines
			if(accum > best_accum)
			{
				best_accum = accum;
				best_vanish_x = vanish_x;
				best_vanish_y = vanish_y;
			}

// draw total
//			vision.accum[vanish_y * vision.image_w + vanish_x] = accum;

/*
 * 			printf("detect_path %d x=%d y=%d accum=%ld\n", 
 * 				__LINE__, 
 * 				vanish_x, 
 * 				vanish_y, 
 * 				accum);
 */
		}
	}


	vision.path_x = (int)best_vanish_x;
#endif // 0


#ifndef X86
// send result to ARM
/*
 * 	unsigned char packet[16];
 * 	packet[0] = 0xff;
 * 	packet[1] = 0x2d;
 * 	packet[2] = 0xd4;
 * 	packet[3] = 0xe5;
 * 	packet[4] = ((int)best_vanish_x) & 0xff;
 * 	packet[5] = (((int)best_vanish_x) >> 8) & 0xff;
 * 	write_spi(packet, 6);
 */
#endif

#if 0
// draw accumulator on output
	int min_accum = 0x7fffffff;
	int max_accum = 0;
	for(i = y1; i < y2; i++)
	{
		for(j = x1; j < x2; j++)
		{
			int offset = i * vision.image_w + j;
			if(vision.accum[offset] > max_accum) max_accum = vision.accum[offset];
			if(vision.accum[offset] != 0 && vision.accum[offset] < min_accum) min_accum = vision.accum[offset];
		}
	}

	for(i = y1; i < y2; i++)
	{
		for(j = x1; j < x2; j++)
		{
			int offset = i * vision.image_w + j;
			vision.y_buffer[offset] = (vision.accum[offset] - min_accum) * 0xff  /
				(max_accum - min_accum);
			vision.u_buffer[offset] = 0x80;
			vision.v_buffer[offset] = 0x80;
		}
	}

#endif // 0



// draw mask on output
	for(i = 0; i < vision.image_h; i++)
	{
		unsigned char *y_row = vision.y_buffer + i * vision.image_w;
		unsigned char *u_row = vision.u_buffer + i * vision.image_w;
		unsigned char *v_row = vision.v_buffer + i * vision.image_w;
		unsigned char *mask_row = vision.mask + i * vision.image_w;
		for(j = 0; j < vision.image_w; j++)
		{
			if(*mask_row)
			{
				*y_row = 0xff;
				*u_row = 0;
				*v_row = 0;
			}


			mask_row++;
			y_row++;
			u_row++;
			v_row++;
		}
	}

	draw_line(chroma_x1, chroma_y1, chroma_x2, chroma_y1);
	draw_line(chroma_x2, chroma_y1, chroma_x2, chroma_y2);
	draw_line(chroma_x2, chroma_y2, chroma_x1, chroma_y2);
	draw_line(chroma_x1, chroma_y2, chroma_x1, chroma_y1);


// 	draw_line(0, y1, vision.image_w, y1);
//	draw_line(best_vanish_x, best_vanish_y - 5, best_vanish_x, best_vanish_y + 5);
// 	draw_line(best_vanish_x - 5, best_vanish_y, best_vanish_x + 5, best_vanish_y);

 	draw_line(x1, y1, x2, y1);
// 	draw_line(x2, y1, x2, y2);
// 	draw_line(x2, y2, x1, y2);
// 	draw_line(x1, y2, x1, y1);

}

