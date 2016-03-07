; remote control for RC car
; using buttons


; 18f1220
; usb_programmer -p 18f1320 -c 0x300000 1100100000000000
; Internal oscillator
; usb_programmer -p 18f1320 -c 0x300002 0001111100001010
; brownout protection
; usb_programmer -p 18f1320 -c 0x300006 0000000010000001
; low voltage programming disabled
; make car
; usb_programmer -p 18f1320 -z 256 car_remote.hex
; usb_programmer -p 18f1320 -z 256 -r car_remote.hex



; XBee radio configuration:
; enter command mode
; +++
; set the frequency
; atch17 or atch0c
; set destination address to broadcast mode
; atdlffff
; set receive address to broadcast mode
; atmyffff
; packet timeout
; atro03
; baud rate  7=115200 6=57600 3=9600
; atbd 3
; change terminal baud rate now
; flash configuration
; atwr
; exit command mode
; atcn





PROCESSOR 18f1220
#include "p18f1220.inc"
#include "util.inc"
#include "chksum.inc"
#include "cc1101.inc"

; dead pins on through hole chip:
; B4
; A4


;#define USE_XBEE


#define DIRECTION_PIN 3
#define DIRECTION_PORT PORTB

#define THROTTLE_PIN 0
#define THROTTLE_PORT PORTB

#define STEERING_PIN0 5
#define STEERING_PORT0 PORTB

#define STEERING_PIN1 6
#define STEERING_PORT1 PORTB

#define STEERING_PIN2 7
#define STEERING_PORT2 PORTB

#define STEERING_PIN3 2
#define STEERING_PORT3 PORTB

#define SDO_PIN 2
#define SDO_LAT LATA
#define SDO_TRIS TRISA

#define SCK_PIN 3
#define SCK_LAT LATA
#define SCK_TRIS TRISA

#define CS0_PIN 1
#define CS0_LAT LATA
#define CS0_TRIS TRISA

#define DEBUG_PIN 7
#define DEBUG_LAT LATA
#define DEBUG_TRIS TRISA





#define CLOCKSPEED 8000000
#define BUFFER_SIZE 12


#define BAUD 9600
#define BAUD_RATE_CODE (CLOCKSPEED / (BAUD * 4) - 1)

#define TIMER_RELOAD (-40960)


#define SYNC_CODE 0xe5


	VARSTART H'00', H'100'
	VARADD FLAGS, 1
cblock 0x0
	NEED_PACKET : 1
endc


	VARADD BUFFER, BUFFER_SIZE
	VARADD BUFFER_PTR, 2
	VARADD TEMP, 1
	VARADD TEMP1, 1
	VARADD TEMP2, 1
	CHKSUM_VARS




	ORG RESETVECTOR
	goto start

	ORG INTVECTORHI
	goto start

	ORG INTVECTORLO
	goto start



#include "cc1101.s"
#include "chksum.s"


start:
	BANKSEL H'00'
	SET_REGISTER OSCCON, B'00000000'


; pin modes
; pin mode: 1 = digital  0 = analog
	SET_REGISTER ADCON1, B'11111111'
	clrf FLAGS

	bcf DEBUG_TRIS, DEBUG_PIN
	bcf DEBUG_LAT, DEBUG_PIN


	SET_REGISTER TXSTA, B'00100100'
	SET_REGISTER RCSTA, B'10000000'
	SET_REGISTER BAUDCTL, B'00001000'
	SET_REGISTER16 SPBRG, BAUD_RATE_CODE

; must delay between each packet
#ifdef USE_XBEE
	SET_REGISTER T0CON, B'10001000'
#endif


	SET_REGISTER16 BUFFER_PTR, BUFFER + BUFFER_SIZE
	bcf INTCON2, RBPU


	bcf SDO_LAT, SDO_PIN
	bcf SDO_TRIS, SDO_PIN

	bcf SCK_LAT, SCK_PIN
	bcf SCK_TRIS, SCK_PIN
	
	bsf CS0_LAT, CS0_PIN
	bcf CS0_TRIS, CS0_PIN







	call reset_chip




	CC1101_TRANSMITTER

	SET_REGISTER OSCCON, B'01110000'


loop:
	clrwdt


#ifdef USE_XBEE
	btfsc INTCON, TMR0IF
	call handle_timer0
#endif

	btfsc PIR1, TXIF
	call handle_uart
	
	bra loop

#ifdef USE_XBEE
handle_timer0:
	SET_TIMER_LITERAL16 TMR0L, TIMER_RELOAD
	bcf INTCON, TMR0IF
	bsf FLAGS, NEED_PACKET
	return
#endif


handle_uart:
	SKIP_LESS_LITERAL BUFFER_PTR, BUFFER + BUFFER_SIZE
	bra handle_uart2
		SET_POINTER0_REG BUFFER_PTR
		COPY_REGISTER TXREG, POSTINC0
		COPY_REGISTER16 BUFFER_PTR, POINTER0
		return


; start new packet
handle_uart2:
; wait to allow the xbee to transmit
#ifdef USE_XBEE
	btfss FLAGS, NEED_PACKET
	return
		bcf FLAGS, NEED_PACKET
#endif

	btg DEBUG_LAT, DEBUG_PIN

		SET_REGISTER BUFFER + 0, 0x00
		SET_REGISTER BUFFER + 1, 0xff
		SET_REGISTER BUFFER + 2, 0x2d
		SET_REGISTER BUFFER + 3, 0xd4
		SET_REGISTER BUFFER + 4, SYNC_CODE
		clrf BUFFER + 5
		clrf BUFFER + 6
		clrf BUFFER + 7
		clrf BUFFER + 8
		clrf BUFFER + 9
		clrf BUFFER + 10

; buttons
		COPY_BIT BUFFER + 6, 0, DIRECTION_PORT, DIRECTION_PIN
		COPY_BIT BUFFER + 6, 1, THROTTLE_PORT, THROTTLE_PIN
		COPY_BIT BUFFER + 6, 2, STEERING_PORT0, STEERING_PIN0
		COPY_BIT BUFFER + 6, 3, STEERING_PORT1, STEERING_PIN1
		COPY_BIT BUFFER + 6, 4, STEERING_PORT2, STEERING_PIN2
		COPY_BIT BUFFER + 6, 5, STEERING_PORT3, STEERING_PIN3


; send the raw value to the receiver
;COPY_REGISTER16 BUFFER + 8, STEERING

		GET_CHKSUM BUFFER + BUFFER_SIZE - 2, BUFFER + 4, BUFFER_SIZE - 6
		
		SET_REGISTER BUFFER_PTR, BUFFER
		return


	
	







END











