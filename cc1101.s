

write_burst_flash:
	SELECT_CHIP
	WRITE_SPI_REG TEMP1
write_burst_loop:
	TBLRD*+
	WRITE_SPI_REG TABLAT
	decfsz TEMP2, f
	bra write_burst_loop
		DESELECT_CHIP
		return



set_defaults:
;	WRITE_REG CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2
;	WRITE_REG CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1
;	WRITE_REG CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0
;	WRITE_REG CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR
;	WRITE_REG CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN
;	WRITE_REG CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1
;	WRITE_REG CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0

; page 75
;	WRITE_REG CC1101_FSCTRL1,  0x08        ; Frequency Synthesizer Control
;	WRITE_REG CC1101_FSCTRL0,  0x00        ; Frequency Synthesizer Control

; page 76
	WRITE_REG CC1101_MDMCFG4,  0x8C        ; filter bandwidth
	WRITE_REG CC1101_MDMCFG3,  0x22        ; data rate
	WRITE_REG CC1101_MDMCFG2,  0x93        ; Modem Configuration
	WRITE_REG CC1101_MDMCFG1,  0x22        ; channel spacing
	WRITE_REG CC1101_MDMCFG0,  0xF8        ; channel spacing

; page 79
	WRITE_REG CC1101_DEVIATN,  0x47        ; Modem Deviation Setting
; page 80
;	WRITE_REG CC1101_MCSM2,  0x16        ; Main Radio Control State Machine Configuration
;	WRITE_REG CC1101_MCSM1,  0x17        ; Main Radio Control State Machine Configuration
	WRITE_REG CC1101_MCSM0,  0x18        ; Main Radio Control State Machine Configuration

;	WRITE_REG CC1101_FOCCFG,  0x19        ; Frequency Offset Compensation Configuration
;	WRITE_REG CC1101_BSCFG,  0x1A        ; Bit Synchronization Configuration

; page 85
;	WRITE_REG CC1101_AGCCTRL2,  0x1B        ; AGC Control
;	WRITE_REG CC1101_AGCCTRL1,  0x1C        ; AGC Control
;	WRITE_REG CC1101_AGCCTRL0,  0x1D        ; AGC Control
;	WRITE_REG CC1101_WOREVT1,  0x1E        ; High Byte Event0 Timeout
;	WRITE_REG CC1101_WOREVT0,  0x1F        ; Low Byte Event0 Timeout
;	WRITE_REG CC1101_WORCTRL,  0x20        ; Wake On Radio Control

; page 89
	WRITE_REG CC1101_FREND1,  0x56        ; Front End RX Configuration
	WRITE_REG CC1101_FREND0,  0x10        ; transmit power

;	WRITE_REG CC1101_FSCAL3,  0x23        ; Frequency Synthesizer Calibration
;	WRITE_REG CC1101_FSCAL2,  0x24        ; Frequency Synthesizer Calibration
;	WRITE_REG CC1101_FSCAL1,  0x25        ; Frequency Synthesizer Calibration
;	WRITE_REG CC1101_FSCAL0,  0x26        ; Frequency Synthesizer Calibration

;	WRITE_REG CC1101_RCCTRL1,  0x27        ; RC Oscillator Configuration
;	WRITE_REG CC1101_RCCTRL0,  0x28        ; RC Oscillator Configuration
;	WRITE_REG CC1101_FSTEST,  CC1101_DEFVAL_FSTEST
;	WRITE_REG CC1101_PTEST,  CC1101_DEFVAL_PTEST
;	WRITE_REG CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST
;	WRITE_REG CC1101_TEST2,  CC1101_DEFVAL_TEST2
;	WRITE_REG CC1101_TEST1,  CC1101_DEFVAL_TEST1
;	WRITE_REG CC1101_TEST0,  CC1101_DEFVAL_TEST0

	return

; transmit power page 60
patable:
	DB H'c0', H'c0', H'c0', H'c0', H'c0', H'c0', H'c0', H'c0'


reset_chip:
	DESELECT_CHIP
	SELECT_CHIP
	DESELECT_CHIP
	SELECT_CHIP
	WRITE_SPI CC1101_SRES
	DESELECT_CHIP
	
	call set_defaults
	
	WRITE_BURST_FLASH CC1101_PATABLE, patable, 0x8
	return


write_spi:
WRITE_SPI_BIT macro
	COPY_BIT SDO_LAT, SDO_PIN, TEMP, 7
	bsf SCK_LAT, SCK_PIN
	rlncf TEMP, f
	bcf SCK_LAT, SCK_PIN
	ENDM

	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT
	WRITE_SPI_BIT

	return


