#ifndef ARM_NAV_H
#define ARM_NAV_H



#define NAV_BUFSIZE 256
typedef struct
{
	void (*current_function)();
	
	unsigned char receive_buf[NAV_BUFSIZE];
	unsigned char receive_buf2[NAV_BUFSIZE];
	unsigned char send_buf[NAV_BUFSIZE];
	int receive_offset;
	int receive_size;
	int send_offset;
	int send_size;
	int counter;
	unsigned char data;
	int got_data;
} nav_t;




#endif

