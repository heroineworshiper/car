/*
 * UWB leash for truck.  Single sided ranging
 *
 * Copyright (C) 2025 Adam Williams <broadcast at earthling dot net>
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

// waste anything but space.  Space is our shortest material.
// TODO: tabulate configuration writes

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#include <pic16f1508.h>
#include <stdint.h>
#include <string.h>

// ticks per second
#define HZ 977

// number of this board
#define ROAMER 0
#define BASE 1
#define LEFT 2
#define RIGHT 3
#define BOARD BASE

//#define DEBUG

#define LED_LAT LATA2
#define LED_TRIS TRISA2
#define SYNC_LAT LATA5
#define SYNC_PORT PORTA5
#define SYNC_TRIS TRISA5
#define CS_LAT LATA4
#define CS_TRIS TRISA4
#define MISO_PORT RC5
#define MISO_TRIS TRISC5
#define MOSI_LAT LATC4
#define MOSI_TRIS TRISC4
#define CLK_LAT LATC3
#define CLK_TRIS TRISC3
#define RESET_LAT LATC6
#define RESET_TRIS TRISC6
#define WAKEUP_LAT LATC7
#define WAKEUP_TRIS TRISC7




void send_uart(uint8_t c)
{
    while(!TRMT) ;
    TXREG = c;
}

#ifdef DEBUG
static uint16_t number;
static uint8_t force;
void print_digit(uint16_t place)
{
	if(number >= place || force)
	{ 
		force = 1; 
		send_uart('0' + number / place); 
		number %= place; 
	}
}

void print_number(uint16_t number_arg)
{
	number = number_arg;
	force = 0;
	print_digit(10000);
	print_digit(1000);
	print_digit(100);
	print_digit(10);
	send_uart('0' + (number % 10)); 
	send_uart(' ');
}

const uint8_t hex_table[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'a', 'b', 'c', 'd', 'e', 'f'  
};

void print_hex2(uint8_t v)
{
    send_uart(hex_table[v >> 4]);
    send_uart(hex_table[v & 0xf]);
    send_uart(' ');
}

void print_hex8(uint32_t v)
{
    uint8_t i;
    for(i = 7; i != 0xff; i--)
        send_uart(hex_table[(v >> (i * 4)) & 0xf]);
    send_uart(' ');
}

void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		send_uart(*s);
		s++;
	}
}

void print_lf()
{
    send_uart('\n');
}
#endif // DEBUG

void print_bin32(uint32_t v)
{
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
        send_uart(v & 0xff);
        v >>= 8;
    }
}

void delay(uint16_t ticks)
{
    while(ticks > 0)
    {
        while(!TMR0IF) ;
        TMR0IF = 0;
        ticks--;
    }
}

uint8_t spi_transfer(uint8_t data)
{
    uint8_t i;
    uint8_t result = 0;
    for(i = 0; i < 8; i++)
    {
        MOSI_LAT = ((data & 0x80) ? 1 : 0);
        data <<= 1;
        CLK_LAT = 1;
        result <<= 1;
        result |= MISO_PORT ? 1 : 0;
        CLK_LAT = 0;
    }
    return result;
}

// from https://github.com/Fhilb/DW3000_Arduino/blob/main/DW3000/src/
// Adapted for pic16
// No useful effect from ANTENNA_DELAY
//#define ANTENNA_DELAY 0x3FCA
//#define ANTENNA_DELAY 0x1000

// units of 16ps
//#define TRANSMIT_DELAY 0x3B9ACA00 // 16ms for printing debug
#define TRANSMIT_DELAY 0x1dcd6500 // 8ms for maximum speed

// low bits in the 40 bit timestamp used by DX_TIME register
#define DX_TIME_MASK 0xfffffe00

// Default Values for Destination and Sender IDs
#define DESTINATION 0x0  
#define SENDER 0x0



#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3

#define PMSC_STATE_IDLE 0x3

#define FCS_LEN 2

#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F

#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000

#define SYS_STATUS_FRAME_TX_SUCC 0x80

#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6

#define CHANNEL_5 0x0
#define CHANNEL_9 0x1

#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02

#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0

#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1

#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0

//Masks
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F

//Registers
#define GEN_CFG_AES_LOW 0x00
    #define SYS_TIME 0x1c
    #define TX_FCTRL 0x24
    #define DX_TIME 0x2c // page 87
    #define SYS_ENABLE 0x3c
    #define SYS_STATUS 0x44 // page 93
    #define TX_TIME 0x74
#define GEN_CFG_AES_HIGH 0x01
    #define TX_ANTD 0x04
    #define TX_POWER 0x0c
    #define CHAN_CTRL 0x14
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC 0x04
    #define EC_CTRL 0x00 // page 128
    #define RX_CAL 0x0C
    #define RX_CAL_RESI 0x14
    #define RX_CAL_RESQ 0x1C
    #define RX_CAL_STS 0x20
#define GPIO_CTRL_REG 0x5
    #define GPIO_MODE 0x00
#define DRX_REG 0x06
    #define DRX_CAR_INT 0x29
#define RF_CONF 0x07  // page 149
    #define RF_ENABLE 0x00  // page 149
    #define RF_TX_CTRL_1 0x1a
    #define RF_TX_CTRL_2 0x1c
    #define SAR_TEST 0x34 // page 153
    #define LDO_TUNE 0x40 // page 154
    #define LDO_CTRL 0x48 // page 155
    #define LDO_RLOAD 0x51 // page 155
#define RF_CAL_REG 0x8
    #define SAR_CTRL 0x0
    #define SAR_STATUS 0x4
    #define SAR_READING 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0x0C
    #define IP_TS 0x00  // page 179
#define CIA_REG2 0xD
#define CIA_REG3 0xE
    #define FP_CONF 0x04
#define DIG_DIAG 0x0F // page 205
    #define SYS_STATE 0x30 // page 216
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F

// Initial Radio Configuration
const uint16_t config[] = {
    CHANNEL_5,           // Channel
    PREAMBLE_128,        // Preamble Length
    9,                   // Preamble Code user manual page 28 (Same for RX and TX!)
    PAC8,                // PAC
    DATARATE_6_8MB,      // Datarate
    PHR_MODE_STANDARD,   // PHR Mode
    PHR_RATE_850KB       // PHR Rate
};

/*
 Internal helper function to send and receive Bytes through the SPI Interface
 @param b The bytes to be sent as an array
 @param len The length of the array b
 @param rec_len The length of bytes that should be received back after sending the data b
 @return Returns 0 or the received bytes from the chip
 */
uint32_t send_bytes(uint8_t *data, uint8_t len, uint8_t rec_len)
{
    CS_LAT = 0;
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        spi_transfer(data[i]);
    }
    
    uint32_t val = 0;
    if(rec_len > 0)
    {
        for(i = 0; i < rec_len; i++)
        {
            uint32_t tmp = spi_transfer(0x00);
            val |= (tmp << (8 * i));
        }
    }
    
    
    CS_LAT = 1;
    return val;
}

/*
 Helper function to read or write to a specific chip register address
 @param base The base register address
 @param sub  The sub register address
 @param data The data that should be written (if anything should be written, else left at 0)
 @param dataLen The length of the data that should be sent in bits. Can be left at zero to automatically determine the data length,
        but can be very useful if e.g. 32 bits of 0x00 should be written
 @param readWriteBit Defines if a read or write operation will be performed. 0 if data should be read back, 1 if only a write should be performed.
 @return Returns 0 or the result of the read operation
*/
uint32_t readOrWriteFullAddress(uint8_t base, 
    uint8_t sub, 
    uint32_t data, 
    uint8_t dataLen, 
    uint8_t readWriteBit) 
{
// base address
    uint16_t header = ((base & 0x1F) << 1);

// write
    if (readWriteBit) header |= 0x80;
// offset
    if (sub > 0) 
    {
        header |= 0x40;
        header <<= 8;
        header |= (((uint16_t)sub & 0x7F) << 2);
    }

    uint8_t header_size = ((header > 0xFF) ? 2 : 1);
    uint32_t res = 0;
    uint8_t payload[2 + 4];

    if (header_size == 1) {
        payload[0] = header;
    }
    else {
        payload[0] = header >> 8;
        payload[1] = header & 0xFF;
// datasheet page 27
// print_text("HEADER: ");
// print_hex2(headerArr[0]);
// print_hex2(headerArr[1]);
// print_text("\n");
    }

    if (!readWriteBit) {
        res = send_bytes(payload, header_size, (dataLen > 0) ? dataLen : 4);
// print_text("ADDR: ");
// print_hex2(base);
// print_hex2(sub);
// print_text("VALUE: ");
// print_hex8(res);
// print_text("\n");
        return res;
    }
    else 
    {
        if (dataLen == 0) {
            if((data & 0xff000000)) dataLen = 4;
            else
            if((data & 0xff0000)) dataLen = 3;
            else
            if((data & 0xff00)) dataLen = 2;
            else
                dataLen = 1;
        }

        for (uint8_t i = 0; i < dataLen; i++) {
            payload[header_size + i] = (data >> (i * 8)) & 0xFF;
        }

        res = send_bytes(payload, 2 + dataLen, 0);
        return res;
    }
}

/*
 Writes to a specific chip register address
 @param base The chips base register address
 @param sub The chips sub register address
 @param data The data that should be written
 @param dataLen The length of the data that should be written
 @return The result of the write operation (typically 0)
*/
uint32_t write_len(uint8_t base, uint8_t sub, uint32_t data, uint8_t dataLen) 
{
    return readOrWriteFullAddress(base, sub, data, dataLen, 1);
}

/*
 Writes to a specific chip register address and automatically determines the dataLen parameter
 @param base The chips base register address
 @param sub The chips sub register address
 @param data The data that should be written
 @return The result of the write operation (typically 0)
*/
uint32_t write(uint8_t base, uint8_t sub, uint32_t data) 
{
    return readOrWriteFullAddress(base, sub, data, 0, 1);
}

/*
 Reads from a specific chip register address
 @param base The chips base register address
 @param sub The chips sub register address
 @return The result of the read operation
*/
uint32_t read(uint8_t base, uint8_t sub) {
    return readOrWriteFullAddress(base, sub, 0, 0, 0);
}

uint32_t read_len(uint8_t base, uint8_t sub, uint8_t len) {
    return readOrWriteFullAddress(base, sub, 0, len, 0);
}

/*
 Reads just 1 Byte from the chip
 @param base The chips base register address
 @param sub The chips sub register address
 @return The result of the read operation
*/
uint8_t read8bit(uint8_t base, uint8_t sub) {
    return (uint8_t)(read(base, sub) >> 24);
}


/*
 Reads a specific OTP (One Time Programmable) Memory address inside the chips register
 @param addr The OTP Memory address
 @return The result of the read operation
 */
uint32_t readOTP(uint8_t addr) {
    write(OTP_IF_REG, 0x04, addr);
    write(OTP_IF_REG, 0x08, 0x02);

    return read(OTP_IF_REG, 0x10);
}


/*
 Writes a Fast Command to the chip (See DW3000 User Manual chapter 9 for more)
 @param cmd The command that should be sent
*/
void writeFastCommand(uint8_t cmd) 
{
    uint8_t header = 0x81 | ((cmd & 0x1F) << 1);

    send_bytes(&header, 1, 0);
}


#define CONFIGURE_AS_TX \
    write(RF_CONF, RF_TX_CTRL_2, 0x34); /* OG_DELAY page 152 */ \
    write(GEN_CFG_AES_HIGH, TX_POWER, 0xfcfcfcfc); /* TX POWER page 107 */

void clear_system_status()
{
    write(GEN_CFG_AES_LOW, SYS_STATUS, 0x3F7FFFFF);
}


#define STANDARD_TX \
    writeFastCommand(0x01);

#define STANDARD_RX \
    writeFastCommand(0x02);

#define TX_INSTANT_RX \
    writeFastCommand(0x0C);

#define DELAYED_TX \
    writeFastCommand(0x3);

#define DELAYED_TX_THEN_RX \
    writeFastCommand(0x0F);

// required to transmit again after a receive timeout
#define CANCEL_RX writeFastCommand(0);

// must write all 32 bits or it'll lock up after a wraparound
#define WRITE_TX_DELAY(value) \
    write_len(GEN_CFG_AES_LOW, DX_TIME, value, 4);

void start_sar()
{
    write(RF_CAL_REG, SAR_CTRL, 0);
    write(RF_CAL_REG, SAR_CTRL, 1);
}

/*
 Checks if a frame got received successfully
 @return 1 if successfully received; 2 if RX Status Error occured; 0 if no frame got received
*/
uint8_t receivedFrameSucc() {
    uint32_t sys_stat = read(GEN_CFG_AES_LOW, SYS_STATUS);

//     print_text("sys_stat=");
//     print_hex8(sys_stat);
//     print_text("\n");

    if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) {
        return 1;
    }
    else if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
// packet received but contained errors
// print_text("RX ERROR=");
// print_hex8(sys_stat);
// print_lf();
        return 2;
    }
    return 0;
}

/*
 Checks if a frame got sent successfully
 @return 1 if successfully sent; 2 if TX Status Error occured; 0 if no frame got sent
*/
uint8_t sentFrameSucc() 
{ 
    uint32_t sys_stat = read(GEN_CFG_AES_LOW, SYS_STATUS);
// print_text("sys_stat=");
// print_hex8(sys_stat);
// print_text("\n");

    if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) {
        return 1;
    }
    return 0;
}


// 40 bit routines.  no int64 on pic16
uint8_t result_hi;
uint32_t result_lo;
void add40(uint8_t hi1, uint32_t lo1, uint8_t hi2, uint32_t lo2)
{
    result_lo = lo1 + lo2;
    result_hi = hi1 + hi2;
    if(result_lo < lo1) result_hi++;
}

void sub40(uint8_t hi1, uint32_t lo1, uint8_t hi2, uint32_t lo2)
{
    result_lo = lo1 - lo2;
    result_hi = hi1 - hi2;
    if(result_lo > lo1) result_hi--;
}

void readRXTimestamp() 
{
    result_lo = read(CIA_REG1, IP_TS);
    result_hi = read(CIA_REG1, IP_TS + 4) & 0xFF;
}

/*
 Reads the internal TX Timestamp. The timestamp is a relative timestamp to the chips internal clock. Units of ~15.65ps. (See DW3000 User Manual 3.2 for more)
 @return The TX Timestamp in units of ~15.65ps
*/
void readTXTimestamp() 
{
    result_lo = read(GEN_CFG_AES_LOW, TX_TIME);
    result_hi = read(GEN_CFG_AES_LOW, TX_TIME + 4) & 0xFF;
}

uint32_t getClockOffset()
{
    uint32_t raw_offset = read(DRX_REG, DRX_CAR_INT) & ((uint32_t)0x1FFFFF);

// extend sign bit
    if (raw_offset & ((uint32_t)0x100000)) {
        raw_offset |= (uint32_t)0xffe00000;
    }
    return raw_offset;
}

/*
 Writes the given data to the chips TX Frame buffer
 @param frame_data The data that should be written onto the chip
*/
#define SET_TX_FRAME(frame_data) \
    write(TX_BUFFER, 0x00, frame_data);

/*
 Sets the frames data length in bytes
 @param frameLen The length of the data in bytes
*/
void setFrameLength(uint16_t frameLen) 
{ // set Frame length in Bytes
    frameLen = frameLen + FCS_LEN;
    uint32_t curr_cfg = read(GEN_CFG_AES_LOW, TX_FCTRL);
//     if (frameLen > 1023) 
//     {
//         print_text("Frame length + FCS_LEN (2) is longer than 1023. Aborting!\n");
//         return;
//     }
    uint32_t tmp_cfg = (curr_cfg & 0xFFFFFC00) | frameLen;

    write(GEN_CFG_AES_LOW, TX_FCTRL, tmp_cfg);
}



/*
 A delayed TX is typically performed to have a known delay between sender and receiver.
 As the chips delayedTX function has a lower timestamp resolution, the frame gets sent
 always a little bit earlier than its supposed to be. For example:
 Delay: 10ms. Receive Time of Frame: 2010us (2.01ms). It should send at 12.01ms, but will be
 sent at 12ms as the last digits get cut off. These last digits can be precalculated and
 sent inside the frame to be added to the RX Timestamp of the receiver. In the above case, 0.01ms
 The DX_Time resolution is ~4ns, the chips internal clock resolution is ~15.65ps.

 This function calculates the missing delay time, adds it to the frames payload and sets the fixed delay (TRANSMIT_DELAY).
*/
void delayedTX()
{
// no int64 on pic16
// 40 bit rx_ts = readRXTimestamp()
    readRXTimestamp();
    uint32_t rx_lo = result_lo;
    uint8_t rx_hi = result_hi;

// 40 bit next_tx = rx_ts + TRANSMIT_DELAY
    add40(rx_hi, rx_lo, 0, TRANSMIT_DELAY);
    uint32_t next_tx_lo = result_lo;
    uint8_t next_tx_hi = result_hi;

// 32 bit time written to the transmitter register.  Only 31 bits are used.
// exact_tx_timestamp = (rx_ts + TRANSMIT_DELAY) >> 8;
    uint32_t exact_tx_timestamp = next_tx_lo >> 8;
    exact_tx_timestamp |= ((uint32_t)next_tx_hi) << 24;

// quantized 40 bit time the transmitter sends at
// calc_tx_timestamp = ((rx_ts + TRANSMIT_DELAY) & DX_TIME_MASK) + ANTENNA_DELAY;
    uint32_t calc_tx_lo = next_tx_lo & DX_TIME_MASK;
    uint8_t calc_tx_hi = next_tx_hi;
// neglect antenna delay
//     add40(calc_tx_hi, calc_tx_lo, 0, ANTENNA_DELAY);
//     calc_tx_lo = result_lo;
//     calc_tx_hi = result_hi;

// quantized 40 bit time difference between receive & transmit
// the 9 bit modulus of the 40 bit rx_ts makes this unique for every packet
// reply_delay = calc_tx_timestamp - rx_ts;
    sub40(calc_tx_hi, calc_tx_lo, rx_hi, rx_lo);
    uint32_t reply_delay = result_lo;


// get temperature
        uint8_t temp_value = 0x5;
        if((read(RF_CAL_REG, SAR_STATUS) & 0x1))
        {
            uint32_t result = read(RF_CAL_REG, SAR_READING);
            write(RF_CAL_REG, SAR_STATUS, 0);
// print_hex8(result);
// print_lf();
            temp_value = (result >> 8) & 0xff;
            start_sar();
        }

// set payload
    write(TX_BUFFER, 0x00, reply_delay);
    write(TX_BUFFER, 0x04, temp_value);  
    setFrameLength(5);

// Write 32 bit transmit time to register 
    WRITE_TX_DELAY(exact_tx_timestamp);
    DELAYED_TX
// DEBUG time the gap between the command & transmission
LED_LAT = !LED_LAT;

// print_text("rx_ts=");
// print_hex2(rx_hi);
// print_hex8(rx_lo);
// print_text("next_tx=");
// print_hex2(next_tx_hi);
// print_hex8(next_tx_lo);
// print_lf();
// print_text("exact_tx=");
// print_hex8(exact_tx_timestamp);
// print_lf();
// print_text("calc_tx=");
// print_hex2(calc_tx_hi);
// print_hex8(calc_tx_lo);
// print_lf();
// print_text("reply_delay=");
// print_hex8(reply_delay);
// print_lf();
}


// SOFT RESET
// void clearAONConfig() 
// {
//     write_len(AON_REG, 0x0, 0x00, 2);
//     write_len(AON_REG, 0x14, 0x00, 1);
// 
//     write_len(AON_REG, 0x04, 0x00, 1); //clear control of aon reg
// 
//     write(AON_REG, 0x04, 0x02);
// 
//     delay(1);
// }
// 
// void softReset() 
// {
//     clearAONConfig();
// 
//     write(PMSC_REG, 0x04, 0x1); //force clock to FAST_RC/4 clock
// 
//     write_len(PMSC_REG, 0x00, 0x00, 2); //init reset
// 
//     delay(100);
// 
//     write(PMSC_REG, 0x00, 0xFFFF); //return back
// 
//     write_len(PMSC_REG, 0x04, 0x00, 1); //set clock back to Auto mode
// }
// 
// /*
//  Set bit in a defined register address
//  @param reg_addr The registers base address
//  @param sub_addr The registers sub address
//  @param shift The bit that should be modified, relative to the base and sub address (0 for bit 0, 1 for bit 1, etc.)
//  @param b The state that the bit should be set to. True if should be set to 1, False if 0
// */
// void setBit(uint8_t reg_addr, uint8_t sub_addr, uint8_t shift, uint8_t b) 
// {
//     uint8_t tmpByte = read8bit(reg_addr, sub_addr);
//     if (b) {
//         tmpByte |= 1 << shift;
//     }
//     else {
//         tmpByte &= 0xff ^ (1 << shift);
//     }
//     write(reg_addr, sub_addr, tmpByte);
// }
// 
// /*
//  Sets bit to High (1) in a defined register
//  @param reg_addr The registers base address
//  @param sub_addr The registers sub address
//  @param shift The bit that should be modified, relative to the base and sub address (0 for bit 0, 1 for bit 1, etc.)
// */
// void setBitHigh(uint8_t reg_addr, uint8_t sub_addr, uint8_t shift) {
//     setBit(reg_addr, sub_addr, shift, 1);
// }
// 
// uint8_t checkForIDLE() 
// {
//     return ((read(DIG_DIAG, SYS_STATE) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE) || 
//         ((read(GEN_CFG_AES_LOW, SYS_STATUS) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK));
// }

void init_radio()
{
//#define DEBUG_INIT
    RESET_LAT = 0;
    RESET_TRIS = 0;
    delay(1);
    RESET_TRIS = 1;
    delay(2);

    SYNC_LAT = 0;
    SYNC_TRIS = 0;
    CS_LAT = 1;
    CS_TRIS = 0;
    MOSI_LAT = 0;
    MOSI_TRIS = 0;
    CLK_LAT = 0;
    CLK_TRIS = 0;
// datasheet page 9
    WAKEUP_LAT = 0;
    WAKEUP_TRIS = 0;

    uint32_t res = 0;

// SOFT RESET
//     setBitHigh(GEN_CFG_AES_LOW, 0x10, 4);
//     while (!checkForIDLE());
//     softReset();
//     delay(200);
//     while (!checkForIDLE()) ;

    uint32_t ldo_low = readOTP(0x04);
    uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = readOTP(0xA);
// print_text("ldo: ");
// print_hex8(ldo_low);
// print_hex8(ldo_high);
// print_hex8(bias_tune);
// print_text("\n");
    bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

//    if(ldo_low != 0) write(RF_CONF, LDO_TUNE, ldo_low);
//    if(ldo_high != 0) write(RF_CONF, LDO_TUNE + 4, ldo_high);

    if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
        write(0x11, 0x1F, bias_tune);

        write(0x0B, 0x08, 0x0100);
    }

    uint32_t xtrim_value = readOTP(0x1E);

//if xtrim_value from OTP memory is 0, choose 0x2E as default value
    if(xtrim_value == 0)
        xtrim_value = 0x2e;

    write(FS_CTRL_REG, 0x14, xtrim_value);

// writeSysConfig begin
    uint32_t usr_cfg = ((uint32_t)STDRD_SYS_CONFIG & 0xFFF) | 
        (config[5] << 3) | 
        (config[6] << 4);

    write(GEN_CFG_AES_LOW, 0x10, usr_cfg);

    uint32_t otp_write = 0x1400;

    if (config[1] >= 256) {
        otp_write |= 0x04;
    }


    write(OTP_IF_REG, 0x08, otp_write); //set OTP config
    write_len(DRX_REG, 0x00, 0x00, 1); //reset DTUNE0_CONFIG
    write(DRX_REG, 0x0, config[3]);

    //64 = STS length
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);

    write_len(GEN_CFG_AES_LOW, TX_FCTRL + 5, 0x00, 1);

    write(DRX_REG, 0x0C, 0xAF5F584C);

    uint32_t chan_ctrl_val = read(GEN_CFG_AES_HIGH, CHAN_CTRL);  //Fetch and adjust CHAN_CTRL data
    chan_ctrl_val &= (~0x1FFF);

    chan_ctrl_val |= config[0]; //Write RF_CHAN

    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);

    write(GEN_CFG_AES_HIGH, CHAN_CTRL, chan_ctrl_val);  //Write new CHAN_CTRL data with updated values

    uint32_t tx_fctrl_val = read(GEN_CFG_AES_LOW, TX_FCTRL);

    tx_fctrl_val |= (config[1] << 12); //Add preamble length
    tx_fctrl_val |= (config[4] << 10); //Add data rate

    write(GEN_CFG_AES_LOW, TX_FCTRL, tx_fctrl_val);
    write(DRX_REG, 0x02, 0x81);

    uint32_t rf_tx_ctrl_2 = 0x1C071134;
    uint32_t pll_conf = 0x0F3C;

    if (config[0]) {
        rf_tx_ctrl_2 &= ~0x00FFFF;
        rf_tx_ctrl_2 |= 0x000001;
        pll_conf &= 0x00FF;
        pll_conf |= 0x001F;
    }

    write(RF_CONF, RF_TX_CTRL_2, rf_tx_ctrl_2);
    write(FS_CTRL_REG, 0x00, pll_conf);
    write(RF_CONF, LDO_RLOAD, 0x14);
// RF_TX_CTRL_1 page 151
    write(RF_CONF, RF_TX_CTRL_1, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81);
    write(GEN_CFG_AES_LOW, SYS_STATUS, 0x02);
    write(PMSC_REG, 0x04, 0x300200); //Set clock to auto mode
    write(PMSC_REG, 0x08, 0x0138);

    uint8_t success = 0;
    for (uint8_t i = 0; i < 100; i++) {
        if (read(GEN_CFG_AES_LOW, 0x0) & 0x2) {
            success = 1;
            break;
        }
    }

#ifdef DEBUG_INIT
    if (!success) {
        print_text("Couldn't lock PLL Clock!\n");
    }
    else {
        print_text("PLL is now locked.\n");
    }
#endif

    uint32_t otp_val = read(OTP_IF_REG, 0x08);
    otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;

    write(OTP_IF_REG, 0x08, otp_val);
    write(RX_TUNE_REG, 0x19, 0xF0);

    uint32_t ldo_ctrl_val = read(RF_CONF, LDO_CTRL); //Save original LDO_CTRL data
    uint32_t tmp_ldo = (0x105 |
        0x100 |
        0x4 |
        0x1);

    write(RF_CONF, LDO_CTRL, tmp_ldo);
    write(EXT_SYNC, RX_CAL_RESQ, 0x020000); //Calibrate RX

    read(EXT_SYNC, RX_CAL);

    delay(20);

    write(EXT_SYNC, RX_CAL, 0x11); //Enable calibration

    success = 0;
    for (uint8_t i = 0; i < 100; i++) {
        if (read(EXT_SYNC, RX_CAL_STS)) {
            success = 1;
            break;
        }
        delay(10);
    }

#ifdef DEBUG_INIT
    if (success) {
        print_text("PGF calibration complete.\n");
    }
    else {
        print_text("PGF calibration failed!\n");
    }
#endif

    write(EXT_SYNC, RX_CAL, 0x00);
    write(EXT_SYNC, RX_CAL_STS, 0x01);

    uint32_t result = read(EXT_SYNC, RX_CAL_RESI);
#ifdef DEBUG_INIT
    if (result == 0x1fffffff) {
        print_text("PGF_CAL failed in stage I!\n");
    }
#endif
    result = read(EXT_SYNC, RX_CAL_RESQ);
#ifdef DEBUG_INIT
    if (result == 0x1fffffff) {
        print_text("PGF_CAL failed in stage Q!\n");
    }
#endif

    write(RF_CONF, LDO_CTRL, ldo_ctrl_val); //Restore original LDO_CTRL data
    write(0x0E, 0x02, 0x01); //Enable full CIA diagnostics to get signal strength information
//set default antenna delay
//    write(GEN_CFG_AES_HIGH, TX_ANTD, ANTENNA_DELAY); 
    write(GEN_CFG_AES_LOW, SYS_ENABLE, 0xFFFFFFFF); //Set Status Enable
    write(GEN_CFG_AES_LOW, SYS_ENABLE + 4, 0xFFFF);
    write_len(0x0A, 0x00, 0x000900, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE  //0xA

    /*
     * Set RX and TX config
     */
    write(RX_TUNE_REG, 0x1C, 0x10000240); //DGC_CFG0
    write(RX_TUNE_REG, 0x20, 0x1B6DA489); //DGC_CFG1
    write(RX_TUNE_REG, 0x38, 0x0001C0FD); //DGC_LUT_0
    write(RX_TUNE_REG, 0x3C, 0x0001C43E); //DGC_LUT_1
    write(RX_TUNE_REG, 0x40, 0x0001C6BE); //DGC_LUT_2
    write(RX_TUNE_REG, 0x44, 0x0001C77E); //DGC_LUT_3
    write(RX_TUNE_REG, 0x48, 0x0001CF36); //DGC_LUT_4
    write(RX_TUNE_REG, 0x4C, 0x0001CFB5); //DGC_LUT_5
    write(RX_TUNE_REG, 0x50, 0x0001CFF5); //DGC_LUT_6
    write(RX_TUNE_REG, 0x18, 0xE5E5); //THR_64 value set to 0x32
    read(EXT_SYNC, 0x20);

    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    write(DRX_REG, 0x0, 0x81101C);
    write(RF_CONF, SAR_TEST, 0x4); // Enable temp sensor readings


    /*
     * Things to do as documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */
    write(RF_CONF, LDO_CTRL, 0x14); //LDO_RLOAD to 0x14 //0x7
    write(RF_CONF, RF_TX_CTRL_1, 0x0E); //RF_TX_CTRL_1 to 0x0E
    write(RF_CONF, RF_TX_CTRL_2, 0x1C071134); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)
    write(FS_CTRL_REG, 0x00, 0x1F3C); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)  //0x9
    write(FS_CTRL_REG, 0x80, 0x81); //PLL_CAL config to 0x81
    write(PMSC_REG, 0x04, 0xB40200);
    write(PMSC_REG, 0x08, 0x80030738);

// temperature compensated antenna delay.  Doesn't do anything
//    result = read(CIA_REG3, FP_CONF);
// calibration temperature at 22C page 68
//    uint32_t cal_temp = readOTP(0x9);
//     print_text("cal_temp=");
//     print_hex8(cal_temp);
//     print_lf();
//     uint32_t antenna_delay = readOTP(0xb);
//     print_text("antenna_delay=");
//     print_hex8(antenna_delay);
//     print_lf();

//    write(CIA_REG3, FP_CONF, result | (((uint32_t)1) << 20) | (cal_temp << 11));

// SYNC pin resets the clock.  page 67, 128
// SYNC pin is dead.  Might require conditions like idle mode.
//    write(EXT_SYNC, EC_CTRL, (0x1 << 11) | (33 << 3));
// res = read(GPIO_CTRL_REG, GPIO_MODE);
// print_text("GPIO_MODE=");
// print_hex8(res);
// print_text("\n");

#ifdef DEBUG_INIT
    print_text("Radio Initialization finished.\n");
#endif


}

void main()
{
// 16 Mhz internal clock
    OSCCON = 0b01111000;

    LED_TRIS = 0;
    LED_LAT = 1;
    
 
// serial port
    TXSTA = 0b00100100;
// disable receive
    RCSTA = 0b10000000;
    BAUDCON = 0b00001000;
// 9600 baud at 16Mhz
//#define BAUD_CODE 416
// 19200 baud
//#define BAUD_CODE 208
// 115200 baud
#define BAUD_CODE 35
    SPBRGH = BAUD_CODE >> 8;
    SPBRGL = BAUD_CODE & 0xff;

// tick timer
    OPTION_REG = 0b10000011;

// digital mode
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;

#ifdef DEBUG
//    print_text("\n\n\n\nWelcome to UWB\n");
#if (BOARD == ROAMER)
    print_text("\n\n\n\nROAMER\n");
#endif
#if (BOARD == BASE)
    print_text("\n\n\n\nBASE\n");
#endif
#endif

    init_radio();

    CONFIGURE_AS_TX
    start_sar();
    
    clear_system_status();

    uint8_t rx_status = 0;
    uint8_t tx_status = 0;
    uint16_t timeout = 0;
    uint32_t clock_offset_accum = 0;
    uint32_t prop_delay_accum = 0;
    uint32_t temp_accum = 0;
    uint32_t roamer_temp_accum = 0;
    uint8_t roamer_temp_count = 0;
    uint8_t counter = 0;
    uint8_t temp_counter = 0;
    while(1)
    {
#if (BOARD == ROAMER)
//        print_text("Waiting\n");

        STANDARD_RX


        while(!(rx_status = receivedFrameSucc())) ;
        if(rx_status == 1)
        {
            clear_system_status();
            delayedTX();


//             SET_TX_FRAME(0)
//             setFrameLength(1);
//             STANDARD_TX


//            print_text("Got it\n");
//            LED_LAT = !LED_LAT;

            timeout = 0;
            while(!(tx_status = sentFrameSucc()))
            {
                if(TMR0IF)
                {
                    TMR0IF = 0;
                    timeout++;
                    if(timeout >= HZ / 4)
                    {
//                        print_text("TX ERROR\n");
                        break;
                    }
                }
            }
// uint32_t sys_stat = read(GEN_CFG_AES_LOW, SYS_STATUS);
// uint32_t sys_stat2 = read_len(GEN_CFG_AES_LOW, SYS_STATUS + 4, 2);
// print_hex8(sys_stat2);
// print_hex8(sys_stat);
// print_lf();
// uint32_t test = read(GEN_CFG_AES_LOW, SYS_TIME);
// print_hex8(test);
// print_lf();
        }
        else
        {
//            print_text("RX Error\n");
        }
        clear_system_status();

// reset the clock
//         SYNC_LAT = 1;
//         delay(1);
//         SYNC_LAT = 0;
#endif // ROAMER


#if (BOARD == BASE)
//        print_text("Sending\n");
        SET_TX_FRAME(0)
        setFrameLength(1);
        TX_INSTANT_RX

        timeout = 0;
        while(!(tx_status = sentFrameSucc())) 
        {
            if(TMR0IF)
            {
                TMR0IF = 0;
                timeout++;
                if(timeout >= HZ / 4) break;
            }
        }


        if(tx_status == 0)
        {
//            print_text("TX Timeout\n");
        }
        else
        {
            timeout = 0;
            while(!(rx_status = receivedFrameSucc())) 
            {
                if(TMR0IF)
                {
                    TMR0IF = 0;
                    timeout++;
                    if(timeout >= 0x12) break;
                }
            }

            if(rx_status == 1)
            {
//                print_text("Got ACK ");
// payload
                uint32_t reply_delay = read(RX_BUFFER_0_REG, 0x00);
                uint8_t roamer_temp = read(RX_BUFFER_0_REG, 0x04);
                if(roamer_temp != 0)
                {
                    roamer_temp_accum += roamer_temp;
                    roamer_temp_count++;
                }


                if((read(RF_CAL_REG, SAR_STATUS) & 0x1))
                {
                    uint32_t result = read(RF_CAL_REG, SAR_READING);
                    write(RF_CAL_REG, SAR_STATUS, 0);
// print_hex8(result);
// print_lf();
                    temp_accum += (result >> 8) & 0xff;
                    temp_counter++;
                    start_sar();
                }

// carrier frequency deviation
                uint32_t clock_offset = getClockOffset();

                readTXTimestamp();
                uint32_t tx_lo = result_lo;
                uint8_t tx_hi = result_hi;

                readRXTimestamp();
                uint32_t rx_lo = result_lo;
                uint8_t rx_hi  = result_hi;

                sub40(rx_hi, rx_lo, tx_hi, tx_lo);
                uint32_t round_lo = result_lo;
                uint8_t round_hi = result_hi;

// TODO: reply_delay has to be scaled based on the carrier frequency deviation
// (clock_offset).  Needs to be done on the ARM but it's <5 parts per million.
// For chan 5: 1.0 + clock_offset * -0.0005731 / 1000000
                sub40(round_hi, round_lo, 0, reply_delay);
// total propagation delay
                uint32_t prop_delay = result_lo;
//                 print_hex8(round_lo);
//                 print_hex8(round_hi);

                clock_offset_accum += clock_offset;
                prop_delay_accum += prop_delay;
                counter++;
// getting 64Hz with all prints disabled
                if(counter >= 50)
                {
                    if(clock_offset_accum & ((uint32_t)0x80000000))
                    {
                        clock_offset_accum = -clock_offset_accum;
                        clock_offset_accum /= counter;
                        clock_offset_accum = -clock_offset_accum;
                    }
                    else
                        clock_offset_accum /= counter;
                    prop_delay_accum /= counter;
                    temp_accum = temp_accum * 10 / temp_counter;
                    roamer_temp_accum = roamer_temp_accum * 10 / temp_counter;
                    
                    send_uart(0xff);
                    send_uart(0xd2);
                    print_bin32(clock_offset_accum);
                    print_bin32(temp_accum);
                    print_bin32(roamer_temp_accum);
                    print_bin32(prop_delay_accum);

//                    print_hex8(clock_offset_accum);
//                    print_number(temp_accum);
//                    print_number(roamer_temp_accum);
//                    print_number(prop_delay_accum);
//                    print_lf();
                    clock_offset_accum = 0;
                    prop_delay_accum = 0;
                    temp_accum = 0;
                    temp_counter = 0;
                    roamer_temp_accum = 0;
                    temp_counter = 0;
                    counter = 0;
                }
                

// print_hex2(timeout);
// print_lf();
            }
            else
            if(rx_status == 0)
            {
//                print_text("RX Timeout\n");
                CANCEL_RX
            }
            else
            {
//                print_text("RX Error ");
// uint32_t sys_stat = read(GEN_CFG_AES_LOW, SYS_STATUS);
// uint32_t sys_stat2 = read_len(GEN_CFG_AES_LOW, SYS_STATUS + 4, 2);
// print_hex8(sys_stat2);
// print_hex8(sys_stat);
// print_lf();
            }
        }
        clear_system_status();
        LED_LAT = !LED_LAT;
#endif // BASE
    }
}






