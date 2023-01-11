// Convert hex files to either binary flash image or bootloader update image
// Created by Adam Williams

// Inserts the 0 value padding bytes of the dspic.

// dspicFirmwareUpdate1.bin format
// -----------------------------------------
// all in little endian
// 2 bytes: number of sections
// 
// 
// 4 bytes: starting address of a section
// 4 bytes: bytes of data 
// n bytes: data
// 4 bytes: checksum 
// 
// 4 bytes: starting address of a section
// 4 bytes: bytes of data 
// n bytes: data
// 4 bytes: checksum 
//
// the starting address of each section is /2 from its address in memory
// the data is stripped of its stuffing bytes
// The 1st section is always the program.
// The 2nd section is always the interrupt vector table.
// The IVT is padded to 1024 bytes.
// Chksum is the sum of the unsigned bytes in the section data.




#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFSIZE 0x100000
#define FILL_BYTE 0xff
#define STRSIZE 1024
#define MAX_SECTIONS 16

typedef struct
{
	int start;
	int size;
} section_t;

section_t sections[MAX_SECTIONS];
unsigned char *data;
FILE *out;

void write_section(section)
{
	int j;
// strip stuffing bytes
	int packed_size = 0;
	int address = sections[section].start;
	unsigned char *packed_data = 
		(unsigned char*)calloc(1, sections[section].size + 1024);
	for(j = 0; j < sections[section].size; j += 4)
	{
		packed_data[packed_size++] = data[address++];
		packed_data[packed_size++] = data[address++];
		packed_data[packed_size++] = data[address++];
		address++;
	}


// for the IVT, pad it to 1024 bytes
	while(packed_size < 1024)
		packed_data[packed_size++] = 0x0;

// starting address / 2
	address = sections[section].start / 2;
	fputc(address & 0xff, out);
	fputc((address >> 8) & 0xff, out);
	fputc((address >> 16) & 0xff, out);
	fputc((address >> 24) & 0xff, out);
// packed size
	fputc(packed_size & 0xff, out);
	fputc((packed_size >> 8) & 0xff, out);
	fputc((packed_size >> 16) & 0xff, out);
	fputc((packed_size >> 24) & 0xff, out);
// data
	fwrite(packed_data, packed_size, 1, out);
// chksum
	uint32_t accum = 0;
	for(j = 0; j < packed_size; j++)
	{
		accum += packed_data[j];
	}
	fputc(accum & 0xff, out);
	fputc((accum >> 8) & 0xff, out);
	fputc((accum >> 16) & 0xff, out);
	fputc((accum >> 24) & 0xff, out);

	free(packed_data);
}

int main(int argc, char *argv[])
{
	int i, j, k;
	int max_byte = 0;
	int min_byte = 0x7fffffff;
	int start_address = 0;
	unsigned char *used = (unsigned char*)calloc(1, BUFSIZE);
	char string[STRSIZE] = { 0 };
	char input_path[STRSIZE] = { 0 };
	char output_path[STRSIZE] = { 0 };
	uint32_t address_lo = 0;
	uint32_t address_hi = 0;
	uint32_t address = 0;
// create a firmware update image
	int do_update = 0;
	int total_sections = 0;
	data = (unsigned char*)calloc(1, BUFSIZE);

// set padding bytes to 0
	for(i = 0; i < BUFSIZE; i += 4)
	{
		data[i + 0] = 0xff;
		data[i + 1] = 0xff;
		data[i + 2] = 0xff;
		data[i + 3] = 0x0;
	}

	if(argc < 3)
	{
		printf("Usage: hextobin <input file> <output file>\n");
		printf("To generate an update image: hextobin -s dist/DSPIC33E_slot1/production/skulpt-aim-dspic-firmware.production.hex dspicFirmwareUpdate.bin\n");
		exit(1);
	}

	for(i = 1; i < argc; i++)
	{
		if(!strcmp(argv[i], "-s"))
		{
			do_update = 1;
		}
		else
		if(!input_path[0])
		{
			strcpy(input_path, argv[i]);
		}
		else
		if(!output_path[0])
		{
			strcpy(output_path, argv[i]);
		}
		else
		{
			printf("main %d: don't know what to do with %s\n", __LINE__, argv[i]);
			exit(1);
		}
	}
	

	FILE *in = fopen(input_path, "r");
	if(!in)
	{
		perror("fopen");
		exit(1);
	}


	while(fgets(string, 1024, in))
	{
// Convert string to binary
		unsigned char string2[1024];
		unsigned char *in_ptr = (unsigned char*)string + 1;
		unsigned char *out_ptr = string2;
		for(j = 0; j < strlen(string); j += 2)
		{
			int character = toupper(*in_ptr++);
			if(character >= '0' && character <= '9')
				*out_ptr = (character - '0') << 4;
			else
				*out_ptr = (10 + character - 'A') << 4;

			character = toupper(*in_ptr++);
			if(character >= '0' && character <= '9')
				*out_ptr |= character - '0';
			else
				*out_ptr |= 10 + character - 'A';
			out_ptr++;
		}


		unsigned char *ptr = string2;

// Number of bytes of data in the line
		int data_bytes = *ptr++;
		if(!data_bytes) break;

// Starting address of data
		address_lo = (*ptr++) << 8;
		address_lo |= *ptr++;

		int type = *ptr++;

// Data is the number of a page
		if(type == 4)
		{
			address_hi = (*ptr++) << 8;
			address_hi |= *ptr++;
//printf("main %d: address_hi=0x%04x\n", __LINE__, address_hi);
		}
		else
		if(type == 0)
// Data is program data
		{
			address = (address_hi << 16) | address_lo;
		
			if(address > BUFSIZE)
			{
				printf("main %d: address too high address=0x%x\n", __LINE__, address);
			}
			else
			{
			
				if(min_byte > address)
					min_byte = address;

				for(j = 0; j < data_bytes; j++)
				{
					data[address] = *ptr++;
					used[address] = 1;
					address++;
				}

				if(max_byte < address)
					max_byte = address;
			}
		}
		else
		{
			printf("Unknown record type %d\n", type);
		}

// Checksum
		ptr++;
	}


printf("main %d: min_byte=0x%x max_byte=0x%x\n", __LINE__, min_byte, max_byte);
// Detect sections
	int current_used = 0;
	int area_start = 0;
	int area_end = 0;
	for(i = min_byte; i < max_byte; i++)
	{
		if(used[i] && !current_used)
		{
			current_used = 1;
			area_start = i;
		}
		else
		if(!used[i] && current_used)
		{
			current_used = 0;
			area_end = i;
			printf("Area used: 0x%x - 0x%x\n", area_start, area_end);
			sections[total_sections].start = area_start;
			sections[total_sections].size = area_end - area_start;
			total_sections++;
		}
	}

	if(current_used)
	{
		printf("Area used: 0x%x-0x%x\n", area_start, max_byte);
		sections[total_sections].start = area_start;
		sections[total_sections].size = max_byte - area_start;
		total_sections++;
	}



	out = fopen(output_path, "w");
	if(!out)
	{
		printf("Couldn't open %s for writing.\n", argv[2]);
		exit(1);
	}


// write program data & IVT but not config data
	int ivt_section = -1;
	int program_section = -1;

	if(do_update)
	{
		for(i = 0; i < total_sections; i++)
		{
			if(sections[i].start == 0x0) ivt_section = i;
			else
			if(sections[i].start == 0x10000) program_section = i;
		}


		if(ivt_section != 0 ||
			program_section != 1)
		{
			printf("main %d: invalid section offsets.  Expected 0x0 & 0x10000.  Writing anyway.\n", __LINE__);
			
//			do_update = 0;
		}
	}

	
	if(do_update)
	{

// number of sections
		fputc(0x02, out);
		fputc(0x00, out);

// ignore the config section at 0x557F0 * 2
		write_section(program_section);
		write_section(ivt_section);
		
	}
	else
	{
		fwrite(data, max_byte, 1, out);
	}
	
	fclose(out);
	
	
// Dump data
// 			for(j = 0x1900; j < 0x1a00; j += 16)
// 			{
// 				printf("%04x %04x %04x %04x %04x %04x %04x %04x\n", 
// 					data[j / 2 + 0],
// 					data[j / 2 + 1],
// 					data[j / 2 + 2],
// 					data[j / 2 + 3],
// 					data[j / 2 + 4],
// 					data[j / 2 + 5],
// 					data[j / 2 + 6],
// 					data[j / 2 + 7]);
// 			}

	




}
