#ifndef ARM_XBEE_H
#define ARM_XBEE_H


#include "arm_truck.h"


#ifdef USE_XBEE




typedef struct 
{
	void (*current_function)();
	unsigned char data;
	unsigned char packet[PACKET_SIZE];
	unsigned char next_packet[PACKET_SIZE];
	int got_packet;
	int counter;

#ifdef USE_HOPPING
// time for configuration & detecting loss of signal
    int start_time;
// the current frequency index
    int current_freq;
// the xbee is initialized 
    int initialized;
// synchronized with the hopping pattern
    int locked_in;
// data to send to the XBee
#define OUT_PACKET_SIZE 32
    unsigned char out_packet[OUT_PACKET_SIZE];
    unsigned char *buffer;
    int buffer_ptr;
    int buffer_size;
// length received from the RX packet
    int length;
    int packet_offset;
#endif


} radio_t;

extern radio_t radio;



#endif // USE_XBEE




#endif





