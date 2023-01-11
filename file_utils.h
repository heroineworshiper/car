#ifndef FILE_UTILS_H
#define FILE_UTILS_H




int compress_output(unsigned char *buffer,
	int allocated,
	unsigned char *out_y, 
	unsigned char *out_u,
	unsigned char *out_v,
	int w,
	int h);
int append_file(char *path, unsigned char *data, int size);



#endif


