/*
 * STM32 Controller for direct drive truck
 * Copyright (C) 2012-2021 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// simple flash filesystem

#include "arm_fs.h"
#include "linux.h"
#include "uart.h"
#include "stm32f4xx_flash.h"


// start of FS
#define FS_START 0x0800c000
// end of FS
#define FS_END 0x08010000

#define MAX_FILE_SIZE 65535

// find the last occurance of a file
uint32_t next_address(uint32_t code)
{
	uint32_t address = FS_START;
    uint32_t result = 0xffffffff;
	while(address < FS_END)
	{
		unsigned char *ptr = (unsigned char *)address;
        uint32_t got_code = *(uint32_t*)ptr;
		if(got_code == 0xffffffff)
		{
// end of files.  Return last hit.
			break;
		}

// got a desired file
        if(got_code == code)
        {
            result = address;
        }

// length of current file
        uint32_t length = *(uint32_t*)(ptr + 4);
        if(length > MAX_FILE_SIZE)
        {
            TRACE2
// glitch
            print_text("invalid size ");
            print_number(length);
            break;
        }
        
        uint32_t new_address = address + 8 + length;
        if(new_address > address)
        {
            address = new_address;
        }
        else
        {
// glitch
            TRACE2
            print_text("glitch\n");
            break;
        }
	}

    TRACE2
    print_text("filename=");
    print_hex(code);
    print_text("address=");
    print_hex(result);
	return result;
}

// get end of filesystem
uint32_t last_address()
{
    uint32_t address = FS_START;
	while(address < FS_END)
	{
        unsigned char *ptr = (unsigned char *)address;
        uint32_t got_code = *(uint32_t*)ptr;
        if(got_code == 0xffffffff)
        {
// end of storage
            return address;
        }
        else
        {
// length of current file
            int length = *(uint32_t*)(ptr + 4);
            if(length > MAX_FILE_SIZE)
            {
                TRACE2
// glitch
                print_text("invalid size ");
                print_number(length);
                break;
            }


            uint32_t new_address = address + 8 + length;
            if(new_address > address)
            {
                address = new_address;
            }
            else
            {
// glitch
                TRACE2
                print_text("glitch\n");
                break;
            }
        }
    }
// didn't find an end
    return 0xffffffff;
}

uint8_t* make_backup(uint32_t except_file, int *size_return)
{
// compute the size of everything except the given file
    int pass = 0;
    uint8_t *result = 0;
    int offset = 0;
    *size_return = 0;

    for(pass = 0; pass < 2; pass++)
    {
        if(pass == 1)
        {
            if(*size_return == 0)
            {
                TRACE2
                print_text("Nothing to back up\n");
                break;
            }

            result = kmalloc(*size_return, 0);
            memset(result, 0xff, *size_return);
        }

        uint32_t address = FS_START;
        while(address < FS_END)
        {
            unsigned char *ptr = (unsigned char *)address;
            uint32_t got_code = *(uint32_t*)ptr;
            if(got_code == 0xffffffff)
            {
// end of storage
                break;
            }

            uint32_t length = *(uint32_t*)(ptr + 4);

            if(length == 0 || length > MAX_FILE_SIZE)
            {
// glitch
                TRACE2
                print_text("invalid size ");
                print_number(length);
                
                if(length > MAX_FILE_SIZE)
                {
                    break;
                }
            }

// got a unique file in its most recent location
            if(length > 0 && 
                got_code != except_file &&
                next_address(got_code) == address)
            {
                TRACE2
                print_text("found filename=");
                print_hex(got_code);
                print_text("length=");
                print_number(length);
                flush_uart();

// add it to the size
                if(pass == 0)
                {
                    *size_return += length + 8;
                }
                else
// store it in the backup
                {
                    memcpy(result + offset,
                        ptr,
                        length + 8);
                    offset += length + 8;
                }
            }

// next source file
            uint32_t new_address = address + 8 + length;
            if(new_address > address)
            {
                address = new_address;
            }
            else
            {
// glitch
                TRACE2
                print_text("glitch\n");
                break;
            }
        }
    }
    
    TRACE2
    print_text("backup size=");
    print_number(*size_return);
    flush_uart();
    
    return result;
}

void enable_flash_write()
{
   	USART_Cmd(UART5, DISABLE);
   	USART_Cmd(USART1, DISABLE);
   	USART_Cmd(USART6, DISABLE);

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | 
		FLASH_FLAG_OPERR | 
		FLASH_FLAG_WRPERR | 
    	FLASH_FLAG_PGAERR | 
		FLASH_FLAG_PGPERR |
		FLASH_FLAG_PGSERR); 
}

void disable_flash_write()
{
	FLASH_Lock(); 


   	USART_Cmd(USART6, ENABLE);
   	USART_Cmd(USART1, ENABLE);
   	USART_Cmd(UART5, ENABLE);
}

void save_file(uint32_t code, unsigned char *buffer, int bytes)
{
    int attempts = 0;
    int max_attempts = 3;
    int i;

    TRACE2
    if(bytes % 4)
    {
        print_text("Size must be a multiple of 4 bytes.  Got ");
        print_number(bytes);
        return;
    }
    
    for(attempts = 0; attempts < max_attempts; attempts++)
    {
// try writing it

// get next address to write file
	    uint32_t address = last_address();
	    if(address == 0xffffffff ||
            address + bytes + 8 > FS_END)
	    {
// erase it all
            int backup_size;
            uint8_t *backup = make_backup(code, &backup_size);

		    address = FS_START;
            enable_flash_write();
    	    if (FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3) != FLASH_COMPLETE)
		    {
		    }
            
// rewrite the backup data
            if(backup)
            {
                for(i = 0; i < backup_size; i += 4)
                {
                    FLASH_ProgramWord(address, *(int*)(backup + i));
                    address += 4;
                }
                kfree(backup);
            }
    	    disable_flash_write();

	    }

	    TRACE2
        print_text("saving to address=");
        print_hex(address);

        uint32_t start_address = address;
        enable_flash_write();
	    FLASH_ProgramWord(address, code);
        address += 4;
	    FLASH_ProgramWord(address, bytes);
	    address += 4;

	    for(i = 0; i < bytes; i += 4)
	    {
		    FLASH_ProgramWord(address, *(int*)(buffer + i));
            address += 4;
	    }


	    disable_flash_write();


        TRACE2
        print_text("bytes free=");
        print_number(FS_END - address);

// verify it
        int pass = 1;
        const unsigned char *ptr = (unsigned char*)start_address;
        if(code != *(uint32_t*)ptr)
        {
            TRACE2
            print_text("stored code doesn't match\n");
            pass = 0;
        }
        
        if(bytes != *(int*)(ptr + 4))
        {
            TRACE2
            print_text("stored length doesn't match\n");
            pass = 0;
        }

        for(i = 0; i < bytes && pass; i++)
        {
            if(ptr[8 + i] != buffer[i])
            {
                TRACE2
                print_text("stored data doesn't match\n");
                pass = 0;
                break;
            }
        }

        if(pass)
        {
	        TRACE2
	        print_text("Saved file\n");
            break;
        }
        else
        if(attempts < max_attempts - 1)
        {
	        TRACE2
	        print_text("Flash verify failed.\n");
        }
        else
        {
	        TRACE2
	        print_text("All flash verifies failed.  Giving up & going to a movie.\n");
        }
    }
}

void list_flash()
{
    uint32_t address = FS_START;
    TRACE2

	while(address < FS_END)
	{
        unsigned char *ptr = (unsigned char *)address;
        uint32_t got_code = *(uint32_t*)ptr;
        if(got_code == 0xffffffff)
        {
// end of storage
            TRACE2
            print_text("bytes free=");
            print_number(FS_END - address);
            break;
        }
        else
        {
            uint32_t length = *(uint32_t*)(ptr + 4);

            TRACE2
            print_text("address=");
            print_hex(address);
            print_text("filename=");
            print_hex(got_code);
            print_text("size=");
            print_number(length);
            flush_uart();

            if(length > MAX_FILE_SIZE)
            {
// glitch
                TRACE2
                print_text("invalid size ");
                print_number(length);
                break;
            }

            uint32_t new_address = address + 8 + length;
            if(new_address > address)
            {
                address = new_address;
            }
            else
            {
// glitch
                TRACE2
                print_text("glitch\n");
                break;
            }
        }
    }
}







