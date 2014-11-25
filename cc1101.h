#ifndef CC1101_H
#define CC1101_H

#define PACKET_SIZE 8

typedef struct 
{
	void (*current_function)();
	unsigned char data;
	unsigned char packet[PACKET_SIZE];
	unsigned char next_packet[PACKET_SIZE];
	int got_packet;
	int counter;
} radio_t;

extern radio_t radio;



#endif

