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


PROCESSOR 18f1220
#include "p18f1220.inc"
#include "util.inc"
#include "chksum.inc"
#include "cc1101.inc"



#define SDO_PIN 2
#define SDO_LAT LATA
#define SDO_TRIS TRISA

#define SCK_PIN 3
#define SCK_LAT LATA
#define SCK_TRIS TRISA

#define CS0_PIN 5
#define CS0_LAT LATB
#define CS0_TRIS TRISB

#define DEBUG_PIN 0
#define DEBUG_LAT LATA
#define DEBUG_TRIS TRISA




; bits in FLAGS
cblock 0x0
	HAVE_ANALOG : 1
	GET_THROTTLE : 1
endc

#define CLOCKSPEED 8000000
#define BUFFER_SIZE 12
#define BAUD 9600
#define BAUD_RATE_CODE (CLOCKSPEED / (BAUD * 4) - 1)
#define SYNC_CODE 0xe5


#define THROTTLE0 0x4000
#define THROTTLE1 0xc000
#define SLOW_THROTTLE 0xc000
#define FAST_THROTTLE 0xffff

; voltages are
; 65408 - left
; 48512 - slow left
; 31936 - slow right
; 16000 - right
#define STEERING0 16000 / 2
#define STEERING1 (16000 + 31936) / 2
#define STEERING2 (31936 + 48512) / 2
#define STEERING3 (65408 + 48512) / 2
#define LEFT 0x0000
#define SLOW_LEFT 0x4000
#define SLOW_RIGHT 0xc000
#define RIGHT 0xffff

	VARSTART H'00', H'100'
	VARADD FLAGS, 1
	VARADD THROTTLE, 2
	VARADD STEERING, 2
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
	SET_REGISTER ADCON1, B'10101111'
	SET_REGISTER ADCON0, B'00010001'
;	SET_REGISTER ADCON0, B'00011001'
	SET_REGISTER ADCON2, B'00111110'

	bcf DEBUG_TRIS, DEBUG_PIN
	bcf DEBUG_LAT, DEBUG_PIN


	SET_REGISTER TXSTA, B'00100100'
	SET_REGISTER RCSTA, B'10000000'
	SET_REGISTER BAUDCTL, B'00001000'
	SET_REGISTER16 SPBRG, BAUD_RATE_CODE



	bsf ADCON0, 1
	clrf FLAGS
	CLEAR_REGISTER16 THROTTLE
	SET_REGISTER16 STEERING, 0x8000
	SET_REGISTER16 BUFFER_PTR, BUFFER + BUFFER_SIZE
	bsf FLAGS, GET_THROTTLE



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

	btg DEBUG_LAT, DEBUG_PIN

	btfsc PIR1, ADIF
	call handle_analog
	
	btfsc PIR1, TXIF
	call handle_uart
	
	bra loop



handle_uart:
	SKIP_LESS_LITERAL BUFFER_PTR, BUFFER + BUFFER_SIZE
	bra handle_uart2
		SET_POINTER0_REG BUFFER_PTR
		COPY_REGISTER TXREG, POSTINC0
		COPY_REGISTER16 BUFFER_PTR, POINTER0
		return

handle_uart2:
	btfss FLAGS, HAVE_ANALOG
	return

		SET_REGISTER BUFFER + 0, 0x00
		SET_REGISTER BUFFER + 1, 0xff
		SET_REGISTER BUFFER + 2, 0x2d
		SET_REGISTER BUFFER + 3, 0xd4
		SET_REGISTER BUFFER + 4, SYNC_CODE
		SET_REGISTER BUFFER + 5, 0
; reverse
		COPY_BIT BUFFER + 5, 0, PORTA, 1


; convert buttons to analog
;		SKIP_LESS_LITERAL16 THROTTLE, THROTTLE0
;		bra convert_throttle1
;			SET_REGISTER16 BUFFER + 6, 0x0000
;			bra convert_throttle3
;convert_throttle1:
;		SKIP_LESS_LITERAL16 THROTTLE, THROTTLE1
;		bra convert_throttle2
;			SET_REGISTER16 BUFFER + 6, FAST_THROTTLE
;			bra convert_throttle3
;convert_throttle2:
;		SET_REGISTER16 BUFFER + 6, SLOW_THROTTLE
;convert_throttle3:


		CLEAR_REGISTER16 BUFFER + 6
		SKIP_LESS_LITERAL16 THROTTLE, 0x8000
		bra convert_throttle2
		SET_REGISTER16 BUFFER + 6, FAST_THROTTLE
convert_throttle2:


		SKIP_LESS_LITERAL16 STEERING, STEERING0
		bra convert_steering1
			SET_REGISTER16 BUFFER + 8, 0x8000
			bra convert_steering_done
convert_steering1:
		SKIP_LESS_LITERAL16 STEERING, STEERING1
		bra convert_steering2
			SET_REGISTER16 BUFFER + 8, RIGHT
			bra convert_steering_done
convert_steering2:
		SKIP_LESS_LITERAL16 STEERING, STEERING2
		bra convert_steering3
			SET_REGISTER16 BUFFER + 8, SLOW_RIGHT
			bra convert_steering_done
convert_steering3:
		SKIP_LESS_LITERAL16 STEERING, STEERING3
		bra convert_steering4
			SET_REGISTER16 BUFFER + 8, SLOW_LEFT
			bra convert_steering_done
convert_steering4:
		SET_REGISTER16 BUFFER + 8, LEFT
convert_steering_done:

; send the raw value to the receiver
;COPY_REGISTER16 BUFFER + 8, STEERING

		GET_CHKSUM BUFFER + BUFFER_SIZE - 2, BUFFER + 4, BUFFER_SIZE - 6
		
		SET_REGISTER BUFFER_PTR, BUFFER
		return


handle_analog:
	bcf PIR1, ADIF


	btfss FLAGS, GET_THROTTLE
	bra handle_analog2
;		SET_REGISTER ADCON0, B'00010001'
		SET_REGISTER ADCON0, B'00011001'
		bcf FLAGS, GET_THROTTLE
		COPY_REGISTER16 THROTTLE, ADRESL
		bsf ADCON0, 1
		return

handle_analog2:
;	SET_REGISTER ADCON0, B'00011001'
	SET_REGISTER ADCON0, B'00010001'
	bsf FLAGS, GET_THROTTLE
	bsf FLAGS, HAVE_ANALOG
	COPY_REGISTER16 STEERING, ADRESL
	bsf ADCON0, 1
	return
	
	
	







END











