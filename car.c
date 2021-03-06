// mane board for passively stable 2 wheeler


// 18f1320
// usb_programmer -p 18f1320 -e
// usb_programmer -p 18f1320 -c 0x300000 1100100000000000
// Internal oscillator
// usb_programmer -p 18f1320 -c 0x300002 0001111100001010
// brownout protection
// usb_programmer -p 18f1320 -c 0x300006 0000000010000001
// low voltage programming disabled
// make car
// usb_programmer -p 18f1320 -z 0 car.hex
// usb_programmer -p 18f1320 -z 0  -r car.hex

// if it randomly crashes, reflash it








#include <pic18fregs.h>
#include <stdint.h>
#include "util.h"

#define CLOCKSPEED 8000000
#define BUFFER_SIZE 8
#define BAUD 9600
// clock jitter is keeping it as low as possible
//#define BAUD_RATE_CODE (CLOCKSPEED / (BAUD * 4) - 1)
#define BAUD_RATE_CODE 208
#define SYNC_CODE 0xe5
#define THROTTLE_PERIOD 32767
// multiply gyro output by this to get steering feedback
//#define AUTO_STEERING_GAIN 4
#define AUTO_STEERING_GAIN 1
// limit for auto steering magnitude
//#define AUTO_STEERING_LIMIT STEERING_PERIOD / 4
#define AUTO_STEERING_LIMIT STEERING_PERIOD / 2
// integral step
#define STEERING_I_STEP 32
#define STEERING_I_LIMIT STEERING_PERIOD / 4
//#define STEERING_I_LIMIT STEERING_PERIOD / 2
// total readings to use for the center calculation
#define GYRO_TOTAL 1024
// maximum range for center calculation
#define GYRO_LIMIT 128
// timer for LED flashing
#define LED_DELAY 64
// timer for packet flashing
#define LED_DELAY2 2
// total readings for each GYRO_RESULT
#define GYRO_OVERSAMPLE_TOTAL 32
// amount heading has to be off to command auto steering voltage
//#define HEADING_DEADBAND 0x0100
#define HEADING_DEADBAND 0x0000
// amount user changes heading in each packet
#define HEADING_CHANGE 0x0200
// enable UART output
//#define DEBUG

// analog range
#define STEERING_MID 0x8000
// deadband
#define STEERING_MIN 0x800
#define STEERING_MAX 0x7f00

#define THROTTLE_MIN 0x100
#define THROTTLE_MAX 0xff00

#define SDO_LAT LATAbits.LATA2
#define SDO_TRIS TRISAbits.TRISA2

#define SCK_LAT LATAbits.LATA3
#define SCK_TRIS TRISAbits.TRISA3

#define CS0_LAT LATBbits.LATB5
#define CS0_TRIS TRISBbits.TRISB5

#define LEFT_FWD_LAT LATBbits.LATB3
#define LEFT_FWD_TRIS TRISBbits.TRISB3
#define LEFT_BWD_LAT LATAbits.LATA1
#define LEFT_BWD_TRIS TRISAbits.TRISA1

#define RIGHT_FWD_LAT LATAbits.LATA7
#define RIGHT_FWD_TRIS TRISAbits.TRISA7
#define RIGHT_BWD_LAT LATAbits.LATA6
#define RIGHT_BWD_TRIS TRISAbits.TRISA6

#define LED_LAT LATAbits.LATA0
#define LED_TRIS TRISAbits.TRISA0


// transfer types
#define WRITE_BURST_FLAG              0x40
#define READ_SINGLE_FLAG              0x80
#define READ_BURST_FLAG               0xC0

// number of frequency channels
#define NUMBER_OF_FCHANNELS      10

// arrays
#define CC1101_PATABLE           0x3E        // PATABLE address
#define CC1101_TXFIFO            0x3F        // TX FIFO address
#define CC1101_RXFIFO            0x3F        // RX FIFO address

// commands
#define CC1101_SRES              0x30        // Reset CC1101 chip
#define CC1101_SFSTXON           0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             // Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        // Turn off crystal oscillator
#define CC1101_SCAL              0x33        // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             // setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             // If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             // WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        // Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        // No operation. May be used to get access to the chip status byte


// registers

#define CC1101_IOCFG2            0x00        // GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        // GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        // GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        // RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        // Sync Word, High Byte
#define CC1101_SYNC0             0x05        // Sync Word, Low Byte
#define CC1101_PKTLEN            0x06        // Packet Length
#define CC1101_PKTCTRL1          0x07        // Packet Automation Control
#define CC1101_PKTCTRL0          0x08        // Packet Automation Control
#define CC1101_ADDR              0x09        // Device Address
#define CC1101_CHANNR            0x0A        // Channel Number
#define CC1101_FSCTRL1           0x0B        // Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        // Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        // Frequency Control Word, High Byte
#define CC1101_FREQ1             0x0E        // Frequency Control Word, Middle Byte
#define CC1101_FREQ0             0x0F        // Frequency Control Word, Low Byte

// page 76
#define CC1101_MDMCFG4           0x10        // Modem Configuration
#define CC1101_MDMCFG3           0x11        // Modem Configuration
#define CC1101_MDMCFG2           0x12        // Modem Configuration
#define CC1101_MDMCFG1           0x13        // Modem Configuration
#define CC1101_MDMCFG0           0x14        // Modem Configuration
#define CC1101_DEVIATN           0x15        // Modem Deviation Setting

// page 80
#define CC1101_MCSM2             0x16        // Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        // Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        // Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        // Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        // Bit Synchronization Configuration
// page 85
#define CC1101_AGCCTRL2          0x1B        // AGC Control
#define CC1101_AGCCTRL1          0x1C        // AGC Control
#define CC1101_AGCCTRL0          0x1D        // AGC Control
#define CC1101_WOREVT1           0x1E        // High Byte Event0 Timeout
#define CC1101_WOREVT0           0x1F        // Low Byte Event0 Timeout
#define CC1101_WORCTRL           0x20        // Wake On Radio Control
// page 89
#define CC1101_FREND1            0x21        // Front End RX Configuration
#define CC1101_FREND0            0x22        // Front End TX Configuration
#define CC1101_FSCAL3            0x23        // Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        // Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        // Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        // Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        // RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        // RC Oscillator Configuration
#define CC1101_FSTEST            0x29        // Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        // Production Test
#define CC1101_AGCTEST           0x2B        // AGC Test
#define CC1101_TEST2             0x2C        // Various Test Settings
#define CC1101_TEST1             0x2D        // Various Test Settings
#define CC1101_TEST0             0x2E        // Various Test Settings

#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High Byte of WOR Time
#define CC1101_WORTIME0          0x37        // Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        // Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result 

// values

#define CC1101_DEFVAL_IOCFG2     0x29        // GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG1     0x2E        // GDO1 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG0     0x06        // GDO0 Output Pin Configuration
#define CC1101_DEFVAL_FIFOTHR    0x07        // RX FIFO and TX FIFO Thresholds
#define CC1101_DEFVAL_SYNC1      0xB5        // Synchronization word, high byte
#define CC1101_DEFVAL_SYNC0      0x47        // Synchronization word, low byte
#define CC1101_DEFVAL_PKTLEN     0xFF        // Packet Length
#define CC1101_DEFVAL_PKTCTRL1   0x06        // Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x05        // Packet Automation Control
#define CC1101_DEFVAL_ADDR       0xFF        // Device Address
#define CC1101_DEFVAL_CHANNR     0x00        // Channel Number


// page 75
// Carrier frequency = 868 MHz
#define CC1101_DEFVAL_FREQ2_868  0x21        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_868  0x62        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_868  0x76        // Frequency Control Word, Low Byte
// Carrier frequency = 902 MHz
#define CC1101_DEFVAL_FREQ2_915  0x22        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_915  0xB1        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_915  0x3B        // Frequency Control Word, Low Byte
// carrier frequency 433Mhz
#define CC1101_DEFVAL_FREQ2_433  0x10
#define CC1101_DEFVAL_FREQ1_433  0xa7
#define CC1101_DEFVAL_FREQ0_433  0x62


#define CC1101_DEFVAL_MCSM2      0x07        // Main Radio Control State Machine Configuration
//#define CC1101_DEFVAL_MCSM1      0x30        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM1      0x20        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM0      0x18        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x16        // Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0x6C        // Bit Synchronization Configuration
#define CC1101_DEFVAL_AGCCTRL2   0x43        // AGC Control
#define CC1101_DEFVAL_AGCCTRL1   0x40        // AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0x91        // AGC Control
#define CC1101_DEFVAL_WOREVT1    0x87        // High Byte Event0 Timeout
#define CC1101_DEFVAL_WOREVT0    0x6B        // Low Byte Event0 Timeout
#define CC1101_DEFVAL_WORCTRL    0xFB        // Wake On Radio Control

#define CC1101_DEFVAL_FSCAL3     0xE9        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_RCCTRL1    0x41        // RC Oscillator Configuration
#define CC1101_DEFVAL_RCCTRL0    0x00        // RC Oscillator Configuration
#define CC1101_DEFVAL_FSTEST     0x59        // Frequency Synthesizer Calibration Control
#define CC1101_DEFVAL_PTEST      0x7F        // Production Test
#define CC1101_DEFVAL_AGCTEST    0x3F        // AGC Test
#define CC1101_DEFVAL_TEST2      0x81        // Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        // Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        // Various Test Settings


//#include "pwmtables.h"

typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
		unsigned got_packet : 1;
// active part of PWM
		unsigned left_active : 1;
		unsigned right_active : 1;
// direction of throttle from user
		unsigned throttle_reverse : 1;
// set when the gyro center is to be calculated
		unsigned need_center : 1;
// set when a new heading is to be set on the next throttle start
		unsigned need_heading : 1;
	};
	
	unsigned char value;
} flags_t;

typedef union
{
	struct
	{
		int32_t heading;
		int32_t future_heading;
	};
	
	struct
	{
		int32_t gyro_accum;
		uint16_t gyro_min;
		uint16_t gyro_max;
	};
} gyro_t;

volatile flags_t flags;
void (*uart_state)();

uint8_t buffer[BUFFER_SIZE];
uint8_t next_buffer[BUFFER_SIZE];
uint8_t buffer_ptr;
// throttle from user
uint16_t throttle;
// steering from user
uint16_t steering;
gyro_t gyro;
uint16_t gyro_counter;
uint16_t gyro_center;
uint16_t led_counter;
void (*gyro_state)();
uint8_t gyro_oversample;
uint16_t gyro_result;
int16_t steering_i;
// used by interrupt handler only
uint8_t uart_in_byte;
// temporaries because stack is too small
int16_t steering_magnitude;
int32_t steering_magnitude2;
int16_t abs_magnitude;
uint16_t value;


// debug output
#ifdef DEBUG
uint8_t uart_out_buffer[8];
uint8_t uart_out_size;
uint8_t uart_out_offset;
#endif

#define SELECT_CHIP CS0_LAT = 0;

#define DESELECT_CHIP CS0_LAT = 1;

void write_spi(uint8_t value)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		SDO_LAT = value & 0x80;
		SCK_LAT = 1;
		value <<= 1;
		SCK_LAT = 0;
	}
}

void write_strobe(uint8_t cmd)
{
	SELECT_CHIP
	write_spi(cmd);
	DESELECT_CHIP
}

// set register on device
void write_reg(uint8_t reg, uint8_t value)
{
	SELECT_CHIP
	write_spi(reg);
	write_spi(value);
	DESELECT_CHIP
}

void write_burst_flash(uint8_t reg, const uint8_t *data, uint8_t size)
{
	uint8_t i;
	SELECT_CHIP
	write_spi(reg | WRITE_BURST_FLAG);
	for(i = 0; i < size; i++)
	{
		write_spi(data[i]);
	}
	DESELECT_CHIP
}


void reset_cc1101()
{
	const uint8_t patable[] =
	{
		0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0
	};
	
	DESELECT_CHIP
	SELECT_CHIP
	DESELECT_CHIP
	SELECT_CHIP
	write_spi(CC1101_SRES);
	DESELECT_CHIP
	

// page 76
	write_reg(CC1101_MDMCFG4,  0x8C); 	   // filter bandwidth
	write_reg(CC1101_MDMCFG3,  0x22); 	   // data rate
	write_reg(CC1101_MDMCFG2,  0x93); 	   // Modem Configuration
	write_reg(CC1101_MDMCFG1,  0x22); 	   // channel spacing
	write_reg(CC1101_MDMCFG0,  0xF8); 	   // channel spacing

// page 79
	write_reg(CC1101_DEVIATN,  0x47); 	   // Modem Deviation Setting
// page 80
	write_reg(CC1101_MCSM0,    0x18);		   // Main Radio Control State Machine Configuration



// page 89
	write_reg(CC1101_FREND1,  0x56);  	   // Front End RX Configuration
	write_reg(CC1101_FREND0,  0x10);  	   // transmit power
	
	write_burst_flash(CC1101_PATABLE, patable, 0x8);

}

void cc1101_receiver()
{
// initialize receiver mode
// set address
	write_reg(CC1101_ADDR, 0x0);
// set channel
	write_reg(CC1101_CHANNR, 0x0);
// set carrier frequency
    write_reg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433);
    write_reg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433);
    write_reg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433);

// disable address check
	write_reg(CC1101_PKTCTRL1, 0x04);
// asynchronous serial mode
	write_reg(CC1101_PKTCTRL0, 0x30);

// enable receive mode
	write_strobe(CC1101_SRX);
// put serial out on GDO0
	write_reg(CC1101_IOCFG0, 0x0d);
}





uint16_t get_chksum(uint8_t *buffer, uint8_t size)
{
	uint8_t i;
	uint16_t result = 0;
	uint16_t result2;

	size /= 2;
	for(i = 0; i < size; i++)
	{
		uint16_t prev_result = result;
// Not sure if word aligned
		uint16_t value = (buffer[0]) | (buffer[1] << 8);
		result += value;
// Carry bit
		if(result < prev_result) result++;
		buffer += 2;
	}

	result2 = (result & 0xff) << 8;
	result2 |= (result & 0xff00) >> 8;
	return result2;
}

void reset_heading()
{
	flags.need_heading = 0;
	gyro.heading = 0;
	steering_i = 0;
}

void get_heading()
{
// update heading
	int16_t step = gyro_result - gyro_center;
	gyro.heading += step;
	gyro.future_heading = gyro.heading + step * 4;	
}




void get_center()
{
	led_counter++;
	if(led_counter >= LED_DELAY)
	{
		LED_LAT ^= 1;
		led_counter = 0;
	}


	if(gyro_result > gyro.gyro_max)
		gyro.gyro_max = gyro_result;
	if(gyro_result < gyro.gyro_min)
		gyro.gyro_min = gyro_result;
// gyro out of range
	if(gyro.gyro_max - gyro.gyro_min > GYRO_LIMIT)
	{
		gyro_counter = 0;
		gyro.gyro_accum = 0;
		gyro.gyro_min = 0xffff;
		gyro.gyro_max = 0x0;
	}
	else
	{

		gyro.gyro_accum += gyro_result;
		gyro_counter++;
		if(gyro_counter >= GYRO_TOTAL)
		{

// got center
			gyro_counter = 0;

#ifdef DEBUG
			if(uart_out_offset >= uart_out_size)
			{
				uart_out_buffer[0] = 0x00;
				uart_out_buffer[1] = 0xff;
				uart_out_buffer[2] = 0x2d;
				uart_out_buffer[3] = 0xd4;
				uart_out_buffer[4] = (gyro_max >> 8);
				uart_out_buffer[5] = gyro_max & 0xff;
				uart_out_offset = 0;
				uart_out_size = 6;
			}
#endif

			LED_LAT = 0;
			gyro.gyro_accum /= GYRO_TOTAL;
			gyro_center = gyro.gyro_accum;
			flags.need_center = 0;
			gyro_counter = 0;
			gyro.gyro_accum = 0;
			gyro.heading = 0;
			gyro.future_heading = 0;
			gyro_state = get_heading;
		}
	}
}



void start_center()
{
	gyro_state = get_center;
}

void handle_packet()
{
	uint16_t chksum = get_chksum(buffer, BUFFER_SIZE - 2);

	if((chksum & 0xff) == buffer[BUFFER_SIZE - 2] &&
		((chksum >> 8) & 0xff) == buffer[BUFFER_SIZE - 1])
	{
// packet good
		if(gyro_state != get_center)
		{
			led_counter++;
			if(led_counter >= LED_DELAY2)
			{
				LED_LAT ^= 1;
				led_counter = 0;
			}
		}

		INTCONbits.GIE = 0;
		flags.throttle_reverse = buffer[1] & 0x1;

		throttle = buffer[2] | (buffer[3] << 8);

		if(throttle >= THROTTLE_MAX)
		{
		
			if(flags.need_center) start_center();
			else
			if(flags.need_heading) reset_heading();
			
			throttle = THROTTLE_PERIOD;
		}
		else
		if(throttle < THROTTLE_MIN)
		{
// get heading when throttle restarts
			flags.need_heading = 1;
			throttle = 0;
		}
		else
		{

// scale THROTTLE_MAX - THROTTLE_MIN to 0 - THROTTLE_PERIOD
			if(flags.need_center) start_center();
			else
			if(flags.need_heading) reset_heading();

// control has no variable throttle
			throttle = 0;
//			throttle -= THROTTLE_MIN;
//			throttle = throttle_table[throttle >> 8] << 8;
// divide by 2 for better control
//			throttle /= 2;
		}



// handle steering
		if(!flags.need_center)
		{
			uint16_t steering = buffer[4] | (buffer[5] << 8);

			if(steering < STEERING_MID + STEERING_MIN &&
				steering > STEERING_MID - STEERING_MIN)
			{
	// no steering command
				steering_magnitude = 0;
			}
			else
			if(steering >= STEERING_MID + STEERING_MAX)
			{

	// full deflection
	// take current heading
				gyro.heading = 0;
				steering_magnitude = STEERING_PERIOD;
			}
			else
			if(steering <= STEERING_MID - STEERING_MAX)
			{
				gyro.heading = 0;
				steering_magnitude = -STEERING_PERIOD;
			}
			else
			{
	// if STEERING_MAX - STEERING_MIN, increment heading
				if(steering < STEERING_MID)
				{
					gyro.heading -= HEADING_CHANGE;
				}
				else
				{

					gyro.heading += HEADING_CHANGE;
				}
				steering_magnitude = 0;
			}
		}
		else
		{
			steering_magnitude = 0;
		}

		INTCONbits.GIE = 1;


/*
 * if(uart_out_offset >= uart_out_size)
 * {
 * uart_out_buffer[0] = 0x00;
 * uart_out_buffer[1] = 0xff;
 * uart_out_buffer[2] = 0x2d;
 * uart_out_buffer[3] = 0xd4;
 * uart_out_buffer[4] = throttle >> 8;
 * uart_out_buffer[5] = throttle & 0xff;
 * uart_out_buffer[6] = steering_magnitude >> 8;
 * uart_out_buffer[7] = steering_magnitude & 0xff;
 * uart_out_offset = 0;
 * uart_out_size = 8;
 * }
 */

	}

	flags.got_packet = 0;

}



// gyro voltage
void handle_analog()
{
	uint16_t result;
	PIR1bits.ADIF = 0;
	COPY_REGISTER16(result, ADRESL); 
	gyro_result += result;
	ADCON0bits.GO_DONE = 1;	
	
	gyro_oversample++;
	if(gyro_oversample >= GYRO_OVERSAMPLE_TOTAL)
	{
		gyro_state();
		gyro_oversample = 0;
		gyro_result = 0;
	}
}


void gyro_idle()
{
}



#ifdef DEBUG
void handle_uart_out()
{
	if(uart_out_offset < uart_out_size)
	{		
		TXREG = uart_out_buffer[uart_out_offset++];
	}
}
#endif


void sync_code1();
void get_data()
{
	next_buffer[buffer_ptr++] = uart_in_byte;
	if(buffer_ptr >= BUFFER_SIZE)
	{
		uart_state = sync_code1;

// only store if previous packet hasn't been processed
		if(!flags.got_packet)
		{
			uint8_t i;
			flags.got_packet = 1;
			for(i = 0; i < BUFFER_SIZE; i++)
				buffer[i] = next_buffer[i];
		}
	}
}


void sync_code3()
{
	if(uart_in_byte == 0xd4)
	{
		buffer_ptr = 0;
		uart_state = get_data;
	}
	else
	{
		uart_state = sync_code1;
	}
}

void sync_code2()
{
	if(uart_in_byte == 0x2d)
	{
		uart_state = sync_code3;
	}
	else
	{
		uart_state = sync_code1;
	}
}

void sync_code1()
{
	if(uart_in_byte == 0xff)
	{
		uart_state = sync_code2;
	}
}


void handle_uart()
{
	flags.interrupt_complete = 0;
	uart_in_byte = RCREG;

	uart_state();
}


// left wheel
void handle_timer0()
{
	INTCONbits.TMR0IF = 0;
	flags.interrupt_complete = 0;

// disable if waiting for gyro center
	if(flags.need_center) return;

// on side of PWM
	if(flags.left_active)
	{
		flags.left_active = 0;
		
// turn off direction which is inactive
		if(flags.throttle_reverse)
		{
			LEFT_FWD_LAT = 0;
			RIGHT_FWD_LAT = 0;
		}
		else
		{
			LEFT_BWD_LAT = 0;
			RIGHT_BWD_LAT = 0;
		}

// turn off direction which is active
		if(throttle < THROTTLE_PERIOD)
		{
			LEFT_FWD_LAT = 0;
			LEFT_BWD_LAT = 0;
			RIGHT_FWD_LAT = 0;
			RIGHT_BWD_LAT = 0;
		}

		value = 0xffff - THROTTLE_PERIOD + throttle;
		
		SET_TIMER_REGISTER(TMR0L, value);
	}
	else
	{
// pin off
		if(throttle > 0)
		{
			if(flags.throttle_reverse)
			{
				LEFT_BWD_LAT = 1;
				RIGHT_BWD_LAT = 1;
			}
			else
			{
				LEFT_FWD_LAT = 1;
				RIGHT_FWD_LAT = 1;
			}
		}
		
		flags.throttle_active = 1;
		value = 0xffff - throttle;
		SET_TIMER_REGISTER(TMR0L, value);
	}
}






// right wheel
void handle_timer1()
{
	PIR1bits.TMR1IF = 0;
	flags.interrupt_complete = 0;


// disable if waiting for gyro center
	if(flags.need_center) return;


// on side of PWM
	if(flags.right_active)
	{
		flags.right_active = 0;
		
// turn off direction which is inactive
		if(flags.throttle_reverse)
		{
			LEFT_FWD_LAT = 0;
			RIGHT_FWD_LAT = 0;
		}
		else
		{
			LEFT_BWD_LAT = 0;
			RIGHT_BWD_LAT = 0;
		}

// turn off direction which is active
		if(throttle < THROTTLE_PERIOD)
		{
			LEFT_FWD_LAT = 0;
			LEFT_BWD_LAT = 0;
			RIGHT_FWD_LAT = 0;
			RIGHT_BWD_LAT = 0;
		}
	}
	
	

	steering_magnitude2 = 0;
	abs_magnitude = ABS(steering_magnitude);



/*
 * 	if(uart_out_offset >= uart_out_size)
 * 	{
 * 		uart_out_buffer[0] = 0x00;
 * 		uart_out_buffer[1] = 0xff;
 * 		uart_out_buffer[2] = 0x2d;
 * 		uart_out_buffer[3] = 0xd4;
 * 		uart_out_buffer[4] = (future_heading >> 24) & 0xff;
 * 		uart_out_buffer[5] = (future_heading >> 16) & 0xff;
 * 		uart_out_buffer[6] = (future_heading >> 8) & 0xff;
 * 		uart_out_buffer[7] = future_heading & 0xff;
 * 		uart_out_offset = 0;
 * 		uart_out_size = 8;
 * 	}
 */

// use manual steering magnitude if user is steering
// use feedback steering magnitude if software is steering
	if(abs_magnitude > 0)
	{
		steering_magnitude2 = steering_magnitude;
	}
	else
	{
// automatic steering if user isn't steering

#ifdef USE_AUTO_STEERING
		if(throttle > 0)
		{
// update integral
			uint8_t update_i = 0;
			if(gyro.future_heading >= HEADING_DEADBAND)
			{
// turn left
				steering_i += STEERING_I_STEP;
				if(steering_i > STEERING_I_LIMIT)
					steering_i = STEERING_I_LIMIT;
			}
			else
			if(gyro.future_heading < -HEADING_DEADBAND)
			{
// turn right
				steering_i -= STEERING_I_STEP;
				if(steering_i < -STEERING_I_LIMIT)
					steering_i = -STEERING_I_LIMIT;
			}

			steering_magnitude2 = steering_i;

// turn left
			if(gyro.future_heading >= HEADING_DEADBAND)
			{
				steering_magnitude2 += (gyro.future_heading - HEADING_DEADBAND) *
					AUTO_STEERING_GAIN;
				if(steering_magnitude2 > AUTO_STEERING_LIMIT)
				{
					steering_magnitude2 = AUTO_STEERING_LIMIT;
				}
			}
			else
			if(gyro.future_heading < -HEADING_DEADBAND)
			{
// turn right
				steering_magnitude2 -= (-HEADING_DEADBAND - gyro.future_heading) *
					AUTO_STEERING_GAIN;
				if(steering_magnitude2 < -AUTO_STEERING_LIMIT)
				{
					steering_magnitude2 = -AUTO_STEERING_LIMIT;
				}
			}

// reverse direction
			if(flags.throttle_reverse)
			{
				steering_magnitude2 = -steering_magnitude2;
			}

#endif // USE_AUTO_STEERING


#ifdef DEBUG
if(uart_out_offset >= uart_out_size)
{
	uart_out_buffer[0] = 0x00;
	uart_out_buffer[1] = 0xff;
	uart_out_buffer[2] = 0x2d;
	uart_out_buffer[3] = 0xd4;
	uart_out_buffer[4] = (steering_magnitude2 >> 8) & 0xff;
	uart_out_buffer[5] = steering_magnitude2 & 0xff;
	uart_out_offset = 0;
	uart_out_size = 6;
}
#endif

		}
	}





	abs_magnitude = ABS(steering_magnitude2);


// pin on
	if(flags.steering_active)
	{
		flags.steering_active = 0;

// turn off direction which is inactive
		if(steering_magnitude2 < 0)
		{
			RIGHT_LAT = 0;
		}
		else
		{
			LEFT_LAT = 0;
		}

// turn off direction which is active if not maximum speed
		if(abs_magnitude < STEERING_PERIOD)
		{
			LEFT_LAT = 0;
			RIGHT_LAT = 0;
		}

		value = 0xffff - STEERING_PERIOD + abs_magnitude;
		SET_TIMER_REGISTER(TMR1L, value);
	}
	else
	{

// pin off
		if(abs_magnitude > 0)
		{
			if(steering_magnitude2 < 0)
			{
				LEFT_LAT = 1;
			}
			else
			{
				RIGHT_LAT = 1;
			}
		}

		flags.steering_active = 1;
		value = 0xffff - abs_magnitude;
		SET_TIMER_REGISTER(TMR1L, value);
	}
}





void main() __interrupt 0
{
	OSCCON = 0b00000000;
	FSR1L = 0x80;
	
	flags.value = 0;
	
// pin modes
// pin mode: 1 = digital  0 = analog
	ADCON1 = 0b11101111;
	ADCON0 = 0b00010001;
	ADCON2 = 0b10111110;
	ADCON0bits.GO_DONE = 1;




	SDO_LAT = 0;
	SDO_TRIS = 0;

	SCK_LAT = 0;
	SCK_TRIS = 0;
	
	CS0_LAT = 1;
	CS0_TRIS = 0;


	reset_cc1101();
	cc1101_receiver();


	OSCCON = 0b01110000;
// need tuning to get the UART to match the transitter, which has been skewed
// by RF emissions
	OSCTUNE = 12;

// don't set heading until center is calculated, to avoid stepping on variables
	gyro.gyro_accum = 0;
	gyro_center = 0;
	gyro_counter = 0;
	flags.need_center = 1;
	flags.need_heading = 1;
	gyro_state = gyro_idle;
	gyro.gyro_min = 0xffff;
	gyro.gyro_max = 0x0;
	gyro_oversample = 0;
	gyro_result = 0;
	led_counter = 0;
	
	throttle = 0;
	steering_magnitude = 0;
	steering_i = 0;
	buffer_ptr = 0;

	LEFT_FWD_LAT = 0;
	LEFT_BWD_LAT = 0;
	RIGHT_FWD_LAT = 0;
	RIGHT_BWD_LAT = 0;
	LEFT_FWD_TRIS = 0;
	LEFT_BWD_TRIS = 0;
	RIGHT_FWD_TRIS = 0;
	RIGHT_BWD_TRIS = 0;

	LED_LAT = 0;
	LED_TRIS = 0;

	T0CON = 0b10001000;
	INTCONbits.TMR0IE = 1;
	T1CON = 0b10000001;
	PIE1bits.TMR1IE = 1;

	uart_state = sync_code1;
	TXSTA = 0b00100100;
	RCSTA = 0b10010000;
	BAUDCTL = 0b00001000;
	SPBRG = (uint8_t)(BAUD_RATE_CODE & 0xff);
	SPBRGH = (uint8_t)((BAUD_RATE_CODE >> 8) & 0xff);
	PIR1bits.RCIF = 0;
	PIE1bits.RCIE = 1;
#ifdef DEBUG
	uart_out_size = 0;
	uart_out_offset = 0;
#endif

/*
 * 	uart_out_buffer[0] = 0x00;
 * 	uart_out_buffer[1] = 0xff;
 * 	uart_out_buffer[2] = 0x2d;
 * 	uart_out_buffer[3] = 0xd4;
 * 	uart_out_buffer[4] = 0xde;
 * 	uart_out_buffer[5] = 0xad;
 * 	uart_out_buffer[6] = 0xbe;
 * 	uart_out_buffer[7] = 0xef;
 * 	uart_out_offset = 0;
 * 	uart_out_size = 8;
 */

// enable interrupts
	INTCONbits.GIE = 1;
	INTCONbits.PEIE = 1;

	while(1)
	{
		ClrWdt();
		if(flags.got_packet) handle_packet();
		if(PIR1bits.ADIF) handle_analog();
#ifdef DEBUG
		if(PIR1bits.TXIF) handle_uart_out();
#endif
	}
}





void interrupt_handler() __interrupt 1
{
	while(1)
	{
		ClrWdt();
		flags.interrupt_complete = 1;
		
		if(PIR1bits.RCIF) handle_uart();
		if(INTCONbits.TMR0IF) handle_timer0();
		if(PIR1bits.TMR1IF) handle_timer1();
		
		if(flags.interrupt_complete) break;
	}
}




