#ifndef ARM_DEBUG_H
#define ARM_DEBUG_H

#include <stdint.h>


void print_text(const char *string);
void print_bin(uint8_t number);
void print_hex(int number);
void print_number(int number);
void print_number_unsigned(uint16_t number);
void flush_serial();
void handle_serial();
void init_serial();
void send_byte(unsigned char x);




#endif


