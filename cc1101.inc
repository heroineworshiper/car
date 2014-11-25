
; transfer types
#define WRITE_BURST_FLAG              0x40
#define READ_SINGLE_FLAG              0x80
#define READ_BURST_FLAG               0xC0

; number of frequency channels
#define NUMBER_OF_FCHANNELS      10

; arrays
#define CC1101_PATABLE           0x3E        ; PATABLE address
#define CC1101_TXFIFO            0x3F        ; TX FIFO address
#define CC1101_RXFIFO            0x3F        ; RX FIFO address

; commands
#define CC1101_SRES              0x30        ; Reset CC1101 chip
#define CC1101_SFSTXON           0x31        ; Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             ; Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        ; Turn off crystal oscillator
#define CC1101_SCAL              0x33        ; Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             ; setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        ; Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        ; In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             ; If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        ; Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        ; Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             ; WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        ; Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        ; Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        ; Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        ; Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        ; No operation. May be used to get access to the chip status byte


; registers

#define CC1101_IOCFG2            0x00        ; GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        ; GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        ; GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        ; RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        ; Sync Word, High Byte
#define CC1101_SYNC0             0x05        ; Sync Word, Low Byte
#define CC1101_PKTLEN            0x06        ; Packet Length
#define CC1101_PKTCTRL1          0x07        ; Packet Automation Control
#define CC1101_PKTCTRL0          0x08        ; Packet Automation Control
#define CC1101_ADDR              0x09        ; Device Address
#define CC1101_CHANNR            0x0A        ; Channel Number
#define CC1101_FSCTRL1           0x0B        ; Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        ; Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        ; Frequency Control Word, High Byte
#define CC1101_FREQ1             0x0E        ; Frequency Control Word, Middle Byte
#define CC1101_FREQ0             0x0F        ; Frequency Control Word, Low Byte

; page 76
#define CC1101_MDMCFG4           0x10        ; Modem Configuration
#define CC1101_MDMCFG3           0x11        ; Modem Configuration
#define CC1101_MDMCFG2           0x12        ; Modem Configuration
#define CC1101_MDMCFG1           0x13        ; Modem Configuration
#define CC1101_MDMCFG0           0x14        ; Modem Configuration
#define CC1101_DEVIATN           0x15        ; Modem Deviation Setting

; page 80
#define CC1101_MCSM2             0x16        ; Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        ; Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        ; Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        ; Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        ; Bit Synchronization Configuration
; page 85
#define CC1101_AGCCTRL2          0x1B        ; AGC Control
#define CC1101_AGCCTRL1          0x1C        ; AGC Control
#define CC1101_AGCCTRL0          0x1D        ; AGC Control
#define CC1101_WOREVT1           0x1E        ; High Byte Event0 Timeout
#define CC1101_WOREVT0           0x1F        ; Low Byte Event0 Timeout
#define CC1101_WORCTRL           0x20        ; Wake On Radio Control
; page 89
#define CC1101_FREND1            0x21        ; Front End RX Configuration
#define CC1101_FREND0            0x22        ; Front End TX Configuration
#define CC1101_FSCAL3            0x23        ; Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        ; Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        ; Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        ; Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        ; RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        ; RC Oscillator Configuration
#define CC1101_FSTEST            0x29        ; Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        ; Production Test
#define CC1101_AGCTEST           0x2B        ; AGC Test
#define CC1101_TEST2             0x2C        ; Various Test Settings
#define CC1101_TEST1             0x2D        ; Various Test Settings
#define CC1101_TEST0             0x2E        ; Various Test Settings

#define CC1101_PARTNUM           0x30        ; Chip ID
#define CC1101_VERSION           0x31        ; Chip ID
#define CC1101_FREQEST           0x32        ; Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        ; Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        ; Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        ; Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        ; High Byte of WOR Time
#define CC1101_WORTIME0          0x37        ; Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        ; Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        ; Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        ; Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        ; Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        ; Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        ; Last RC Oscillator Calibration Result 

; values

#define CC1101_DEFVAL_IOCFG2     0x29        ; GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG1     0x2E        ; GDO1 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG0     0x06        ; GDO0 Output Pin Configuration
#define CC1101_DEFVAL_FIFOTHR    0x07        ; RX FIFO and TX FIFO Thresholds
#define CC1101_DEFVAL_SYNC1      0xB5        ; Synchronization word, high byte
#define CC1101_DEFVAL_SYNC0      0x47        ; Synchronization word, low byte
#define CC1101_DEFVAL_PKTLEN     0xFF        ; Packet Length
#define CC1101_DEFVAL_PKTCTRL1   0x06        ; Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x05        ; Packet Automation Control
#define CC1101_DEFVAL_ADDR       0xFF        ; Device Address
#define CC1101_DEFVAL_CHANNR     0x00        ; Channel Number


; page 75
; Carrier frequency = 868 MHz
#define CC1101_DEFVAL_FREQ2_868  0x21        ; Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_868  0x62        ; Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_868  0x76        ; Frequency Control Word, Low Byte
; Carrier frequency = 902 MHz
#define CC1101_DEFVAL_FREQ2_915  0x22        ; Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_915  0xB1        ; Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_915  0x3B        ; Frequency Control Word, Low Byte
; carrier frequency 433Mhz
#define CC1101_DEFVAL_FREQ2_433  0x10
#define CC1101_DEFVAL_FREQ1_433  0xa7
#define CC1101_DEFVAL_FREQ0_433  0x62


#define CC1101_DEFVAL_MCSM2      0x07        ; Main Radio Control State Machine Configuration
;#define CC1101_DEFVAL_MCSM1      0x30        ; Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM1      0x20        ; Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM0      0x18        ; Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x16        ; Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0x6C        ; Bit Synchronization Configuration
#define CC1101_DEFVAL_AGCCTRL2   0x43        ; AGC Control
#define CC1101_DEFVAL_AGCCTRL1   0x40        ; AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0x91        ; AGC Control
#define CC1101_DEFVAL_WOREVT1    0x87        ; High Byte Event0 Timeout
#define CC1101_DEFVAL_WOREVT0    0x6B        ; Low Byte Event0 Timeout
#define CC1101_DEFVAL_WORCTRL    0xFB        ; Wake On Radio Control

#define CC1101_DEFVAL_FSCAL3     0xE9        ; Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        ; Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        ; Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        ; Frequency Synthesizer Calibration
#define CC1101_DEFVAL_RCCTRL1    0x41        ; RC Oscillator Configuration
#define CC1101_DEFVAL_RCCTRL0    0x00        ; RC Oscillator Configuration
#define CC1101_DEFVAL_FSTEST     0x59        ; Frequency Synthesizer Calibration Control
#define CC1101_DEFVAL_PTEST      0x7F        ; Production Test
#define CC1101_DEFVAL_AGCTEST    0x3F        ; AGC Test
#define CC1101_DEFVAL_TEST2      0x81        ; Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        ; Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        ; Various Test Settings




SELECT_CHIP macro
	bcf CS0_LAT, CS0_PIN
	ENDM

DESELECT_CHIP macro
	bsf CS0_LAT, CS0_PIN
	ENDM

; write literal to SPI
WRITE_SPI macro value
	SET_REGISTER TEMP, value
	call write_spi
	ENDM

; write contents of memory to SPI
WRITE_SPI_REG macro reg
	COPY_REGISTER TEMP, reg
	call write_spi
	ENDM

WRITE_STROBE macro cmd
	SELECT_CHIP
	WRITE_SPI cmd
	DESELECT_CHIP
	ENDM
	
; set register on device
WRITE_REG macro reg, value
	SELECT_CHIP
	WRITE_SPI reg
	WRITE_SPI value
	DESELECT_CHIP
	ENDM

; write flash memory to SPI
WRITE_BURST_FLASH macro reg, data, size
	SET_REGISTER TEMP1, reg | WRITE_BURST_FLAG
	SET_FLASH_LITERAL data
	SET_REGISTER TEMP2, size
	call write_burst_flash
	ENDM





CC1101_TRANSMITTER macro
; initialize transmitter mode
; set address
	WRITE_REG CC1101_ADDR, 0
; set channel
	WRITE_REG CC1101_CHANNR, 0
; set carrier frequency
    WRITE_REG CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433
    WRITE_REG CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433
    WRITE_REG CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433



; disable address check
	WRITE_REG CC1101_PKTCTRL1, 0x04
; asynchronous serial mode
	WRITE_REG CC1101_PKTCTRL0, 0x30

; enable transmit mode
	WRITE_STROBE CC1101_STX
; put serial in on GDO0
	WRITE_REG CC1101_IOCFG0, 0x0d
	ENDM






CC1101_RECEIVER macro

; initialize receiver mode
; set address
	WRITE_REG CC1101_ADDR, 0
; set channel
	WRITE_REG CC1101_CHANNR, 0x0
; set carrier frequency
    WRITE_REG CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433
    WRITE_REG CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433
    WRITE_REG CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433

; disable address check
	WRITE_REG CC1101_PKTCTRL1, 0x04
; asynchronous serial mode
	WRITE_REG CC1101_PKTCTRL0, 0x30

; enable receive mode
	WRITE_STROBE CC1101_SRX
; put serial out on GDO0
	WRITE_REG CC1101_IOCFG0, 0x0d
	ENDM





