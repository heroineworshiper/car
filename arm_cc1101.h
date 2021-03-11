#ifndef ARM_CC1101_H
#define ARM_CC1101_H


#include "arm_truck.h"


#ifdef USE_CC1101

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


#endif // USE_CC1101

#endif

