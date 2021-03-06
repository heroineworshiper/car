; mane board for RC car using gyro for steering


; 18f1320
; usb_programmer -p 18f1320 -c 0x300000 1100100000000000
; Internal oscillator
; usb_programmer -p 18f1320 -c 0x300002 0001111100001010
; brownout protection
; usb_programmer -p 18f1320 -c 0x300006 0000000010000001
; low voltage programming disabled
; make car
; usb_programmer -p 18f1320 -z 256 car.hex
; usb_programmer -p 18f1320 -z 256  -r car.hex


PROCESSOR 18f1320
#include "p18f1320.inc"
#include "util.inc"
#include "chksum.inc"
#include "cc1101.inc"


; enable steering feedback
#define USE_AUTO_STEERING


#define SDO_PIN 2
#define SDO_LAT LATA
#define SDO_TRIS TRISA

#define SCK_PIN 3
#define SCK_LAT LATA
#define SCK_TRIS TRISA

#define CS0_PIN 5
#define CS0_LAT LATB
#define CS0_TRIS TRISB






; bits in FLAGS
cblock 0x0
	INTERRUPT_COMPLETE : 1
	GOT_PACKET : 1
	THROTTLE_ACTIVE : 1
; steering pin on
	STEERING_ACTIVE : 1
; direction of steering
	STEERING_LEFT : 1
; direction of throttle
	THROTTLE_REVERSE : 1
; set when the gyro center is to be calculated
	NEED_CENTER : 1
; set when a new heading is to be set on the next throttle start
	NEED_HEADING : 1
endc


#define CLOCKSPEED 8000000
#define BUFFER_SIZE 8
#define BAUD 9600
; clock jitter is keeping it as low as possible
#define BAUD_RATE_CODE (CLOCKSPEED / (BAUD * 4) - 1)
#define SYNC_CODE 0xe5
#define THROTTLE_PERIOD 65535
#define STEERING_PERIOD 32767
; limit for auto steering magnitude
;#define AUTO_STEERING_LIMIT STEERING_PERIOD / 2
#define AUTO_STEERING_LIMIT STEERING_PERIOD / 4
; total readings for each integral update
#define STEERING_I_DOWNSAMPLE 1
; integral step
#define STEERING_I_STEP 1
; total readings to use for the center calculation
#define GYRO_TOTAL 256
; total readings for each GYRO_RESULT
#define GYRO_OVERSAMPLE_TOTAL 32
; amount heading has to be off to command auto steering voltage
#define HEADING_DEADBAND 0x0100
; amount user changes heading in each packet
#define HEADING_CHANGE 0x0200

; analog range
#define STEERING_MID 0x8000
; deadband
#define STEERING_MIN 0x800
#define STEERING_MAX 0x7f00
;#define STEERING_NUDGE STEERING_PERIOD / 4

#define THROTTLE_MIN 0x100
#define THROTTLE_MAX 0xff00


	VARSTART H'00', H'100'
	VARADD FLAGS, 1
	VARADD UART_STATE, 2
	
	
	VARADD BUFFER, BUFFER_SIZE
	VARADD NEXT_BUFFER, BUFFER_SIZE
	VARADD BUFFER_PTR, 2
	VARADD THROTTLE, 2
	VARADD HEADING, 4
	VARADD FUTURE_HEADING, 4
	VARADD GYRO_COUNTER, 2
	VARADD GYRO_CENTER, 2
	VARADD GYRO_ACCUM, 4
	VARADD GYRO_MIN, 2
	VARADD GYRO_MAX, 2
	VARADD GYRO_STATE, 2
	VARADD GYRO_OVERSAMPLE, 1
	VARADD GYRO_RESULT, 2
	VARADD STEERING, 2
	VARADD STEERING_MAGNITUDE, 2
	VARADD STEERING_MAGNITUDE2, 2
	VARADD STEERING_DIRECTION, 1
	VARADD STEERING_I_COUNTER, 2
	VARADD STEERING_I, 2
	VARADD STEERING_I_DIRECTION, 1
; debug output
	VARADD UART_OUT_BUFFER, 8
	VARADD UART_OUT_SIZE, 1
	VARADD UART_OUT_OFFSET, 1
	VARADD TEMP, 2
	VARADD TEMP1, 2
	VARADD TEMP2, 2
	VARADD INT_TEMP, 2
	VARADD PC_TEMP, 1
	CHKSUM_VARS



	ORG RESETVECTOR
	goto start

	ORG INTVECTORHI
	goto interrupt_handler

	ORG INTVECTORLO
	goto interrupt_handler




#include "cc1101.s"
#include "chksum.s"
#include "pwmtables.s"


start:
	BANKSEL H'00'
	SET_REGISTER OSCCON, B'00000000'
	clrf FLAGS

; pin modes
; pin mode: 1 = digital  0 = analog
	SET_REGISTER ADCON1, B'11101111'
	SET_REGISTER ADCON0, B'00010001'
	SET_REGISTER ADCON2, B'10111110'
	bsf ADCON0, 1




	bcf SDO_LAT, SDO_PIN
	bcf SDO_TRIS, SDO_PIN

	bcf SCK_LAT, SCK_PIN
	bcf SCK_TRIS, SCK_PIN
	
	bsf CS0_LAT, CS0_PIN
	bcf CS0_TRIS, CS0_PIN







	call reset_chip

	CC1101_RECEIVER

	SET_REGISTER OSCCON, B'01110000'
; need tuning to get the UART to match the transitter, which has been skewed
; by RF emissions
	SET_REGISTER OSCTUNE, 12



	CLEAR_REGISTER32 GYRO_ACCUM
	CLEAR_REGISTER32 HEADING
	CLEAR_REGISTER32 GYRO_CENTER
	CLEAR_REGISTER32 GYRO_COUNTER
	bsf FLAGS, NEED_CENTER
	bsf FLAGS, NEED_HEADING
	SET_REGISTER16 GYRO_STATE, gyro_idle
	SET_REGISTER16 GYRO_MIN, 0xffff
	SET_REGISTER16 GYRO_MAX, 0x0
	clrf GYRO_OVERSAMPLE
	CLEAR_REGISTER16 GYRO_RESULT
	
	CLEAR_REGISTER16 THROTTLE
	CLEAR_REGISTER16 STEERING_MAGNITUDE
	CLEAR_REGISTER16 STEERING_I
	clrf STEERING_I_DIRECTION
	CLEAR_REGISTER16 STEERING_I_COUNTER
	SET_REGISTER16 STEERING, 0x8000
	SET_REGISTER16 BUFFER_PTR, NEXT_BUFFER

	bcf TRISB, 3
	bcf TRISA, 1
	bcf TRISA, 7
	bcf TRISA, 6
	bcf LATB, 3
	bcf LATA, 1
	bcf LATA, 7
	bcf LATA, 6

	bcf TRISA, 0
	bsf LATA, 0

	SET_REGISTER T0CON, B'10001000'
	bsf INTCON, TMR0IE
	SET_REGISTER T1CON, B'10000001'
	bsf PIE1, TMR1IE

	SET_REGISTER16 UART_STATE, sync_code1
	SET_REGISTER TXSTA, B'00100100'
	SET_REGISTER RCSTA, B'10010000'
	SET_REGISTER BAUDCTL, B'00001000'
	SET_REGISTER16 SPBRG, BAUD_RATE_CODE
	bcf PIR1, RCIF
	bsf PIE1, RCIE
	clrf UART_OUT_SIZE
	clrf UART_OUT_OFFSET

; enable interrupts
	bsf INTCON, PEIE
	bsf INTCON, GIE




loop:
	clrwdt
	
	btfsc FLAGS, GOT_PACKET
	call handle_packet

	btfsc PIR1, ADIF
	call handle_analog

	btfsc PIR1, TXIF
	call handle_uart_out

	bra loop






interrupt_handler:
	COPY_REGISTER PC_TEMP, PCLATH
interrupt_handler2:
	clrwdt
	bsf FLAGS, INTERRUPT_COMPLETE

	btfsc PIR1, RCIF
	call handle_uart

	btfsc INTCON, TMR0IF
	call handle_timer0

	btfsc PIR1, TMR1IF
	call handle_timer1


; loop again if interrupt was handled
	btfss FLAGS, INTERRUPT_COMPLETE
	bra interrupt_handler2
		COPY_REGISTER PCLATH, PC_TEMP
; return after popping context from fast register stack
		retfie FAST






handle_uart:
	bcf FLAGS, INTERRUPT_COMPLETE
	COPY_REGISTER INT_TEMP, RCREG

	SET_PC_REG UART_STATE


sync_code1:
	SKIP_EQUAL_LITERAL INT_TEMP, 0xff
	return
		SET_REGISTER16 UART_STATE, sync_code2
		return

sync_code2:
	SET_REGISTER16 UART_STATE, sync_code1
	SKIP_EQUAL_LITERAL INT_TEMP, 0x2d
	return
		SET_REGISTER16 UART_STATE, sync_code3
		return

sync_code3:
	SET_REGISTER16 UART_STATE, sync_code1
	SKIP_EQUAL_LITERAL INT_TEMP, 0xd4
	return
		SET_REGISTER16 BUFFER_PTR, NEXT_BUFFER
		SET_REGISTER16 UART_STATE, get_data
		return

get_data:
	SET_POINTER2_REG BUFFER_PTR
	COPY_REGISTER POSTINC2, INT_TEMP
	COPY_REGISTER16 BUFFER_PTR, POINTER2
	SKIP_GREATEREQUAL_LITERAL BUFFER_PTR, NEXT_BUFFER + BUFFER_SIZE
	return

		SET_REGISTER16 UART_STATE, sync_code1

; only store if previous packet hasn't been processed
		btfsc FLAGS, GOT_PACKET
		return
		
			bsf FLAGS, GOT_PACKET
			COPY_REGISTER BUFFER + 0, NEXT_BUFFER + 0
			COPY_REGISTER BUFFER + 1, NEXT_BUFFER + 1
			COPY_REGISTER BUFFER + 2, NEXT_BUFFER + 2
			COPY_REGISTER BUFFER + 3, NEXT_BUFFER + 3
			COPY_REGISTER BUFFER + 4, NEXT_BUFFER + 4
			COPY_REGISTER BUFFER + 5, NEXT_BUFFER + 5
			COPY_REGISTER BUFFER + 6, NEXT_BUFFER + 6
			COPY_REGISTER BUFFER + 7, NEXT_BUFFER + 7
			return




handle_uart_out:
	SKIP_LESS UART_OUT_OFFSET, UART_OUT_SIZE
	return
		
		SET_POINTER0_LITERAL UART_OUT_BUFFER
		ADD POINTER0, UART_OUT_OFFSET
		COPY_REGISTER TXREG, POSTINC0
		incf UART_OUT_OFFSET, F
		return







handle_packet:
	GET_CHKSUM TEMP, BUFFER, BUFFER_SIZE - 2
	
	SKIP_EQUAL TEMP + 0, BUFFER + BUFFER_SIZE - 2
	bra packet_fail
	SKIP_EQUAL TEMP + 1, BUFFER + BUFFER_SIZE - 1
	bra packet_fail

; packet good
		bcf TRISA, 0
		btg LATA, 0

		bcf INTCON, GIE
		COPY_BIT FLAGS, THROTTLE_REVERSE, BUFFER + 1, 0

		COPY_REGISTER16 THROTTLE, BUFFER + 2

		SKIP_GREATEREQUAL_LITERAL16 THROTTLE, THROTTLE_MAX
		bra throttle_max_done
		
			btfsc FLAGS, NEED_CENTER
			call start_center
		
			btfsc FLAGS, NEED_HEADING
			call reset_heading
			
			SET_REGISTER16 THROTTLE, THROTTLE_PERIOD
			bra throttle_done

throttle_max_done:
		SKIP_LESS_LITERAL16 THROTTLE, THROTTLE_MIN
		bra throttle_min_done

; get heading when throttle restarts
			bsf FLAGS, NEED_HEADING
			SET_REGISTER16 THROTTLE, 0
			bra throttle_done

; scale THROTTLE_MAX - THROTTLE_MIN to 0 - THROTTLE_PERIOD
throttle_min_done:
	btfsc FLAGS, NEED_CENTER
	call start_center

	btfsc FLAGS, NEED_HEADING
	call reset_heading

	SUBTRACT_LITERAL16 THROTTLE, THROTTLE_MIN
	SET_FLASH_LITERAL throttle_table
	ADD_REG16_UNSIGNEDREG8 TBLPTRL, THROTTLE + 1
	TBLRD*
	COPY_REGISTER THROTTLE + 1, TABLAT
	clrf THROTTLE + 0
; divide by 2 for better control
	bcf STATUS, C
	rrcf THROTTLE + 1, F
	rrcf THROTTLE + 0, F

throttle_done:



; handle steering
		COPY_REGISTER16 STEERING, BUFFER + 4
		SKIP_GREATEREQUAL_LITERAL16 STEERING, STEERING_MID
		bra steering_left1

; turn right
			bcf FLAGS, STEERING_LEFT
			COPY_REGISTER16 STEERING_MAGNITUDE, STEERING
			SUBTRACT_LITERAL16 STEERING_MAGNITUDE, STEERING_MID
			bra clamp_steering

steering_left1:
; turn left
			bsf FLAGS, STEERING_LEFT
			SET_REGISTER16 STEERING_MAGNITUDE, STEERING_MID - 1
			SUBTRACT16 STEERING_MAGNITUDE, STEERING

clamp_steering:
		SKIP_GREATEREQUAL_LITERAL16 STEERING_MAGNITUDE, STEERING_MAX
		bra steering_max_done

; full deflection
; take current heading
			SET_REGISTER32 HEADING, 0x8000, 0x0000
			SET_REGISTER16 STEERING_MAGNITUDE, STEERING_PERIOD
			bra steering_done


steering_max_done:
	SKIP_LESS_LITERAL16 STEERING_MAGNITUDE, STEERING_MIN
	bra steering_min_done
; no steering command
		CLEAR_REGISTER16 STEERING_MAGNITUDE
		bra steering_done

steering_min_done:
; if STEERING_MAX - STEERING_MIN, increment heading
	CLEAR_REGISTER16 STEERING_MAGNITUDE
	btfss FLAGS, STEERING_LEFT
	bra step_steering_right

		ADD_LITERAL32 HEADING, 0x0000, HEADING_CHANGE
		bra steering_done

step_steering_right:
		SUBTRACT_LITERAL32 HEADING, 0x0000, HEADING_CHANGE
	

; if STEERING_MAX - STEERING_MIN, use fixed value
;	SET_REGISTER16 STEERING_MAGNITUDE, STEERING_NUDGE
; take current heading
;	SET_REGISTER32 HEADING, 0x8000, 0x0000
		
steering_done:
		bsf INTCON, GIE

packet_fail:
	bcf FLAGS, GOT_PACKET
	return



reset_heading:
	bcf FLAGS, NEED_HEADING
	SET_REGISTER32 HEADING, 0x8000, 0x0000
	return




start_center:
	SET_REGISTER16 GYRO_STATE, get_center
	return





; throttle PWM
handle_timer0:
	bcf INTCON, TMR0IF
	bcf FLAGS, INTERRUPT_COMPLETE

; disable if waiting for gyro center
	btfsc FLAGS, NEED_CENTER
	return

; pin on
	btfss FLAGS, THROTTLE_ACTIVE
	bra handle_throttle1
		bcf FLAGS, THROTTLE_ACTIVE
		
; turn off direction which is inactive
		btfsc FLAGS, THROTTLE_REVERSE
; forward
		bcf LATB, 3
		btfss FLAGS, THROTTLE_REVERSE
; reverse
		bcf LATA, 1

; turn off direction which is active
		SKIP_LESS_LITERAL16 THROTTLE, THROTTLE_PERIOD
		bra handle_throttle2
; forward
		bcf LATB, 3
; reverse
		bcf LATA, 1

handle_throttle2:
		SET_REGISTER16 INT_TEMP, 0xffff
		SUBTRACT_LITERAL16 INT_TEMP, THROTTLE_PERIOD
		ADD16 INT_TEMP, THROTTLE
		
		COPY_TO_TIMER TMR0L, INT_TEMP
		return


; pin off
handle_throttle1:
	SKIP_NONZERO16 THROTTLE
	bra handle_throttle3
	
	btfss FLAGS, THROTTLE_REVERSE
	bsf LATB, 3
	btfsc FLAGS, THROTTLE_REVERSE
	bsf LATA, 1

handle_throttle3:
	bsf FLAGS, THROTTLE_ACTIVE
	SET_REGISTER16 INT_TEMP, 0xffff
	SUBTRACT16 INT_TEMP, THROTTLE
	COPY_TO_TIMER TMR0L, INT_TEMP
	return







; steering PWM
handle_timer1:
	bcf PIR1, TMR1IF
	bcf FLAGS, INTERRUPT_COMPLETE

; disable if waiting for gyro center
	btfsc FLAGS, NEED_CENTER
	return






; use manual steering magnitude if user is steering
; use feedback steering magnitude if software is steering
	SKIP_NONZERO16 STEERING_MAGNITUDE
	bra auto_steering
		
		COPY_REGISTER16 STEERING_MAGNITUDE2, STEERING_MAGNITUDE
		COPY_REGISTER STEERING_DIRECTION, FLAGS
		bra auto_steering_done









; automatic steering if user isn't steering
auto_steering:

	clrf STEERING_DIRECTION
	CLEAR_REGISTER16 STEERING_MAGNITUDE2

#ifdef USE_AUTO_STEERING
	SKIP_NONZERO16 THROTTLE
	bra auto_steering_done

; turn left
	SKIP_GREATEREQUAL_LITERAL32 FUTURE_HEADING, 0x8000, HEADING_DEADBAND
	bra auto_steering2
		bsf STEERING_DIRECTION, STEERING_LEFT
		COPY_REGISTER32 TEMP1, FUTURE_HEADING
		SUBTRACT_LITERAL32 TEMP1, 0x8000, HEADING_DEADBAND
		bra auto_steering_magnitude

auto_steering2:
; turn right
	SKIP_LESS_LITERAL32 FUTURE_HEADING, 0x7fff, -HEADING_DEADBAND
	bra auto_steering_magnitude
		bcf STEERING_DIRECTION, STEERING_LEFT
		SET_REGISTER32 TEMP1, 0x7fff, -HEADING_DEADBAND
		SUBTRACT32 TEMP1, FUTURE_HEADING

auto_steering_magnitude:
; reverse direction
	btfsc FLAGS, THROTTLE_REVERSE
	btg STEERING_DIRECTION, STEERING_LEFT

; scale magnitude
;	DIVIDE_REG32 TEMP1
	
	SKIP_GREATEREQUAL_LITERAL32 TEMP1, 0x0000, AUTO_STEERING_LIMIT
	bra auto_steering_magnitude2
		SET_REGISTER16 STEERING_MAGNITUDE2, AUTO_STEERING_LIMIT
		bra auto_steering_done

auto_steering_magnitude2:
	COPY_REGISTER16 STEERING_MAGNITUDE2, TEMP1
#endif ; USE_AUTO_STEERING


auto_steering_done:








; pin on
	btfss FLAGS, STEERING_ACTIVE
	bra handle_steering1
		bcf FLAGS, STEERING_ACTIVE

; turn off direction which is inactive
		btfss STEERING_DIRECTION, STEERING_LEFT
		bcf LATA, 7
		btfsc STEERING_DIRECTION, STEERING_LEFT
		bcf LATA, 6

; turn off direction which is active if not maximum speed
		SKIP_LESS_LITERAL16 STEERING_MAGNITUDE2, STEERING_PERIOD
		bra handle_steering3
			bcf LATA, 7
			bcf LATA, 6

handle_steering3:
		SET_REGISTER16 INT_TEMP, 0xffff
		SUBTRACT_LITERAL16 INT_TEMP, STEERING_PERIOD
		ADD16 INT_TEMP, STEERING_MAGNITUDE2
		COPY_TO_TIMER TMR1L, INT_TEMP
		return


; pin off
handle_steering1:
	SKIP_NONZERO16 STEERING_MAGNITUDE2
	bra handle_steering2
	
	btfsc STEERING_DIRECTION, STEERING_LEFT
	bsf LATA, 7
	btfss STEERING_DIRECTION, STEERING_LEFT
	bsf LATA, 6

handle_steering2:
	bsf FLAGS, STEERING_ACTIVE
	SET_REGISTER16 INT_TEMP, 0xffff
	SUBTRACT16 INT_TEMP, STEERING_MAGNITUDE2
	COPY_TO_TIMER TMR1L, INT_TEMP
	return



; gyro voltage
handle_analog:
	bcf PIR1, ADIF

	ADD16 GYRO_RESULT, ADRESL
	bsf ADCON0, 1
	
	
	incf GYRO_OVERSAMPLE, F
	SKIP_GREATEREQUAL_LITERAL GYRO_OVERSAMPLE, GYRO_OVERSAMPLE_TOTAL
	return
	
	
; debug
bcf TRISB, 2
btg LATB, 2


; debug
;SKIP_GREATEREQUAL UART_OUT_OFFSET, UART_OUT_SIZE
;bra debug_done
;SET_REGISTER UART_OUT_BUFFER + 0, 0x00
;SET_REGISTER UART_OUT_BUFFER + 1, 0xff
;SET_REGISTER UART_OUT_BUFFER + 2, 0x2d
;SET_REGISTER UART_OUT_BUFFER + 3, 0xd4
;COPY_REGISTER UART_OUT_BUFFER + 4, GYRO_RESULT + 1
;COPY_REGISTER UART_OUT_BUFFER + 5, GYRO_RESULT + 0
;SET_REGISTER UART_OUT_SIZE, 6
;SET_REGISTER UART_OUT_OFFSET, 0
;debug_done:


	SET_PC_REG GYRO_STATE


gyro_idle:
	bra gyro_done
	return



get_center:
	SKIP_GREATER16 GYRO_RESULT, GYRO_MAX
	bra get_center2
		COPY_REGISTER16 GYRO_MAX, GYRO_RESULT

get_center2:
	SKIP_LESS16 GYRO_RESULT, GYRO_MIN
	bra get_center3
		COPY_REGISTER16 GYRO_MIN, GYRO_RESULT


get_center3:
	ADD_REG32_UNSIGNEDREG16 GYRO_ACCUM, GYRO_RESULT
	INC16 GYRO_COUNTER
	SKIP_GREATEREQUAL_LITERAL16 GYRO_COUNTER, GYRO_TOTAL
		bra gyro_done

; got center
	CLEAR_REGISTER16 GYRO_COUNTER
	SUBTRACT16 GYRO_MAX, GYRO_MIN

; gyro out of range
	SKIP_GREATER_LITERAL16 GYRO_MAX, 32
	bra get_center4

		CLEAR_REGISTER32 GYRO_ACCUM
		SET_REGISTER16 GYRO_MIN, 0xffff
		SET_REGISTER16 GYRO_MAX, 0x0
		bra gyro_done


get_center4:
	bcf LATA, 0
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	DIVIDE_REG32 GYRO_ACCUM
	COPY_REGISTER16 GYRO_CENTER, GYRO_ACCUM
	bcf FLAGS, NEED_CENTER
	CLEAR_REGISTER16 GYRO_COUNTER
	CLEAR_REGISTER32 GYRO_ACCUM
; debug
SKIP_GREATEREQUAL UART_OUT_OFFSET, UART_OUT_SIZE
bra debug_done2
SET_REGISTER UART_OUT_BUFFER + 0, 0x00
SET_REGISTER UART_OUT_BUFFER + 1, 0xff
SET_REGISTER UART_OUT_BUFFER + 2, 0x2d
SET_REGISTER UART_OUT_BUFFER + 3, 0xd4
COPY_REGISTER UART_OUT_BUFFER + 4, GYRO_CENTER + 1
COPY_REGISTER UART_OUT_BUFFER + 5, GYRO_CENTER + 0
SET_REGISTER UART_OUT_SIZE, 6
SET_REGISTER UART_OUT_OFFSET, 0
debug_done2:


	SET_REGISTER16 GYRO_STATE, get_heading
	bra gyro_done




get_heading:
; update heading
	COPY_REGISTER16 TEMP, GYRO_CENTER
	SUBTRACT16 TEMP, GYRO_RESULT
	ADD_REG32_REG16 HEADING, TEMP
	COPY_REGISTER32 FUTURE_HEADING, HEADING
	ADD_REG32_REG16 FUTURE_HEADING, TEMP
	ADD_REG32_REG16 FUTURE_HEADING, TEMP
	ADD_REG32_REG16 FUTURE_HEADING, TEMP
	ADD_REG32_REG16 FUTURE_HEADING, TEMP
	
; debug
SKIP_GREATEREQUAL UART_OUT_OFFSET, UART_OUT_SIZE
bra debug_done3
SET_REGISTER UART_OUT_BUFFER + 0, 0x00
SET_REGISTER UART_OUT_BUFFER + 1, 0xff
SET_REGISTER UART_OUT_BUFFER + 2, 0x2d
SET_REGISTER UART_OUT_BUFFER + 3, 0xd4
COPY_REGISTER UART_OUT_BUFFER + 4, HEADING + 3
COPY_REGISTER UART_OUT_BUFFER + 5, HEADING + 2
COPY_REGISTER UART_OUT_BUFFER + 6, HEADING + 1
COPY_REGISTER UART_OUT_BUFFER + 7, HEADING + 0
SET_REGISTER UART_OUT_SIZE, 8
SET_REGISTER UART_OUT_OFFSET, 0
debug_done3:


; update gyro bias
if 0
	ADD_REG32_UNSIGNEDREG16 GYRO_ACCUM, GYRO_RESULT
	INC16 GYRO_COUNTER
	SKIP_GREATEREQUAL_LITERAL16 GYRO_COUNTER, GYRO_TOTAL
	bra gyro_done
	
		CLEAR_REGISTER16 GYRO_COUNTER
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM
		DIVIDE_REG32 GYRO_ACCUM


		SKIP_GREATEREQUAL16 GYRO_ACCUM, GYRO_CENTER
		bra get_heading2
			INC16 GYRO_CENTER
			bra get_heading3
		
get_heading2:
		SKIP_LESS16 GYRO_ACCUM, GYRO_CENTER
		bra get_heading3
			DEC16 GYRO_CENTER

get_heading3:
; debug
;SKIP_GREATEREQUAL UART_OUT_OFFSET, UART_OUT_SIZE
;bra debug_done
;SET_REGISTER UART_OUT_BUFFER + 0, 0x00
;SET_REGISTER UART_OUT_BUFFER + 1, 0xff
;SET_REGISTER UART_OUT_BUFFER + 2, 0x2d
;SET_REGISTER UART_OUT_BUFFER + 3, 0xd4
;COPY_REGISTER UART_OUT_BUFFER + 4, GYRO_CENTER + 1
;COPY_REGISTER UART_OUT_BUFFER + 5, GYRO_CENTER + 0
;SET_REGISTER UART_OUT_SIZE, 6
;SET_REGISTER UART_OUT_OFFSET, 0
;debug_done:


		CLEAR_REGISTER32 GYRO_ACCUM
endif ; 0


gyro_done:
	CLEAR_REGISTER16 GYRO_RESULT
	clrf GYRO_OVERSAMPLE
	return









END











