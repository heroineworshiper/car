# the leg tester
AVR_DIR := /root/arduino-1.8.5/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_CFLAGS := -O2 -mmcu=atmega8
AVR_LFLAGS := -O2 -mmcu=atmega8 -Wl,--section-start=.text=0x0000 -nostdlib
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
AVR_DUDE := avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b19200



# other vehicles
PICMODEL := 18f26k20
SDCC_PATH := /root/sdcc
SDCC := $(SDCC_PATH)/bin/sdcc
#SDCC_CFLAGS := --optimize-df --optimize-cmp --obanksel=2 -mpic16 -p$(PICMODEL) -Dpic$(PICMODEL) -I/usr/local/share/sdcc/non-free/include/pic16/
#SDCC_LFLAGS := $(SDCC_CFLAGS) --no-crt -Wl -s,linker.lkr -L/usr/local/share/sdcc/non-free/lib/pic16/
GCC := gcc
GXX := g++
USB_DIR := ../copter/libusb-1.0.0/libusb
USB_LIB := $(USB_DIR)/.libs/libusb-1.0.a
OBJDIR := $(shell uname --machine)
SDCC_PICMODEL := 18f1320
SDCC_PATH := /usr/local/
SDCC := sdcc
# --obanksel=2 isn't working
SDCC_CFLAGS := --optimize-df --optimize-cmp --obanksel=1 -mpic16 -p$(SDCC_PICMODEL) -Dpic$(SDCC_PICMODEL) -I. -I$(SDCC_PATH)/share/sdcc/include/pic16/ -I$(SDCC_PATH)/share/sdcc/non-free/include/pic16 -I../copter/pic 
SDCC_LFLAGS := $(SDCC_CFLAGS) --no-crt -Wl -s,18f1320.lkr  -L$(SDCC_PATH)/share/sdcc/non-free/lib/pic16
# float, no USB
GCC_ARM := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-gcc
OBJCOPY := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-objcopy
ARM_CFLAGS := \
	-O2 \
	-c \
	-mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-I. \
	-I../copter/arm \
	-I../copter/stm32f4
ARM_LIBM := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-file-name=libm.a)
ARM_LIBC := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-libgcc-file-name)
ARM_LFLAGS := -mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-nostdlib \
	-nostdinc \
	$(ARM_LIBM) $(ARM_LIBC)
GCC_PI := /opt/pi/bin/bcm2708hardfp-g++

#OPENCV_DIR := ../opencv-2.4.11/bin/
#OPENCV_CFLAGS := $(shell PKG_CONFIG_PATH=$(OPENCV_DIR)/lib/pkgconfig pkg-config --cflags opencv )
#OPENCV_LFLAGS := $(shell PKG_CONFIG_PATH=$(OPENCV_DIR)/lib/pkgconfig pkg-config --libs opencv )

OPENCV_PI_DIR := ../opencv-2.4.11.arm/
OPENCV_PI_CFLAGS := -I$(OPENCV_PI_DIR)/build/ \
	-I$(OPENCV_PI_DIR)/modules/core/include/ \
	-I$(OPENCV_PI_DIR)/modules/features2d/include/ \
	-I$(OPENCV_PI_DIR)/modules/legacy/include/ \
	-I$(OPENCV_PI_DIR)/modules/calib3d/include/ \
	-I$(OPENCV_PI_DIR)/modules/flann/include/ \
	-I$(OPENCV_PI_DIR)/modules/imgproc/include/ \
	-I$(OPENCV_PI_DIR)/modules/highgui/include/ \
	-I$(OPENCV_PI_DIR)/modules/video/include/ \
	-I$(OPENCV_PI_DIR)/modules/ml/include/ \
	-I$(OPENCV_PI_DIR)/modules/nonfree/include/ \
	-I$(OPENCV_PI_DIR)/modules/ocl/include/ \
	-I$(OPENCV_PI_DIR)/modules/objdetect/include/
OPENCV_PI_LFLAGS := $(OPENCV_PI_DIR)/libopencv.a

PI_CFLAGS := -O2 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -Ijpeg


GCC_ODROID := /opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-g++
#ODROID_CFLAGS := -O3 -pipe -march=armv7-a -mcpu=cortex-a9 -mfloat-abi=hard -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -Ijpeg
ODROID_CFLAGS := -O3 -pipe -mcpu=cortex-a15 -mfloat-abi=hard -mfpu=neon-vfpv4 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -Ijpeg
ODROID_LFLAGS := 

$(shell echo $(SDCC_CFLAGS) > sdcc_cflags)
$(shell echo $(SDCC_LFLAGS) > sdcc_lflags)
$(shell echo $(GCC_ARM) $(ARM_CFLAGS) > arm_gcc )



SRCS := \
	buttons.s \
	buttons.inc \
	display.s \
	display.inc \
	distance.s \
	distance.inc \
	dock.inc \
	dock.s \
	gps.s \
	gps.inc \
	lcd.inc \
	lcd.s \
	mmc.inc \
	mmc.s \
	pic_util.inc \
	sound.inc \
	sound.s \
	sounds.hex \
	speedo.s \
	state.inc \
	state.s

INTERVAL_SRCS := \
	pic_mrf49xa.inc \
	pic_mrf49xa.s \
	pic_util.inc \
	interval.inc \
	interval.s \
	interval_buttons.inc \
	interval_buttons.s \
	interval_rec.s \
	interval_disp.inc \
	interval_disp.s \
	interval_state.inc \
	interval_state.s \
	lcd.inc \
	lcd.s \
	touch_buttons.inc \
	touch_buttons.s

ARM_OBJS := \
	../copter/stm32f4/startup_main.o \
	../copter/arm/arm_math.o \
	../copter/arm/uart.o \
	../copter/arm/linux.o \
	../copter/stm32f4/misc.o \
	../copter/stm32f4/stm32f4xx_rcc.o \
	../copter/stm32f4/stm32f4xx_usart.o \
	../copter/stm32f4/stm32f4xx_gpio.o \
	../copter/stm32f4/stm32f4xx_dcmi.o \
	../copter/stm32f4/stm32f4xx_dma.o \
	../copter/stm32f4/stm32f4xx_i2c.o \
	stm32f4xx_it.o \
	../copter/stm32f4/stm32f4xx_iwdg.o \
	../copter/stm32f4/stm32f4xx_tim.o \
	../copter/stm32f4/stm32f4xx_adc.o \
	../copter/stm32f4/stm32f4xx_flash.o \
	stm32f4xx_exti.o \
	../copter/stm32f4/stm32f4xx_syscfg.o \
        ../copter/stm32f4/system_stm32f4xx.o


VISION_OBJS := \
	vision.o \
	vision_engine.o \
	vision_server.o \
	jpeg/cdjpeg.o \
	jpeg/jcapimin.o \
	jpeg/jcapistd.o \
	jpeg/jccoefct.o \
	jpeg/jccolor.o \
	jpeg/jcdctmgr.o \
	jpeg/jchuff.o \
	jpeg/jcinit.o \
	jpeg/jcmainct.o \
	jpeg/jcmarker.o \
	jpeg/jcmaster.o \
	jpeg/jcomapi.o \
	jpeg/jcparam.o \
	jpeg/jcphuff.o \
	jpeg/jcprepct.o \
	jpeg/jcsample.o \
	jpeg/jctrans.o \
	jpeg/jdapimin.o \
	jpeg/jdapistd.o \
	jpeg/jdatadst.o \
	jpeg/jdatasrc.o \
	jpeg/jdcoefct.o \
	jpeg/jdcolor.o \
	jpeg/jddctmgr.o \
	jpeg/jdhuff.o \
	jpeg/jdinput.o \
	jpeg/jdmainct.o \
	jpeg/jdmarker.o \
	jpeg/jdmaster.o \
	jpeg/jdmerge.o \
	jpeg/jdphuff.o \
	jpeg/jdpostct.o \
	jpeg/jdsample.o \
	jpeg/jdtrans.o \
	jpeg/jerror.o \
	jpeg/jfdctflt.o \
	jpeg/jfdctfst.o \
	jpeg/jfdctint.o \
	jpeg/jidctflt.o \
	jpeg/jidctfst.o \
	jpeg/jidctint.o \
	jpeg/jidctred.o \
	jpeg/jmemmgr.o \
	jpeg/jmemnobs.o \
	jpeg/jquant1.o \
	jpeg/jquant2.o \
	jpeg/jutils.o \
	jpeg/rdbmp.o \
	jpeg/rdcolmap.o \
	jpeg/rdgif.o \
	jpeg/rdppm.o \
	jpeg/rdrle.o \
	jpeg/rdswitch.o \
	jpeg/rdtarga.o \
	jpeg/transupp.o \
	jpeg/wrbmp.o \
	jpeg/wrgif.o \
	jpeg/wrppm.o \
	jpeg/wrrle.o \
	jpeg/wrtarga.o





ARM_CAR_OBJS := \
	arm_car.o \

ARM_TRUCK_OBJS := \
	arm_cc1101.o \
	arm_nav.o \
	arm_truck.o \
	arm_imu.o \
	arm_hardi2c.o \
        arm_xbee.o

ARM_TRUCK2_OBJS := \
        arm_motors.o \
	arm_truck2.o \
	arm_hardi2c.o

ARM_CAM_OBJS := \
	arm_cam.o

PI_OBJS := \
	vision.o






all: truck.bin car_remote.hex


motor: motor.c avr_debug.c motor.h
	$(AVR_GCC) $(AVR_CFLAGS) -o motor.o motor.c
	$(AVR_GCC) $(AVR_LFLAGS) -o motor.elf motor.o
	$(AVR_OBJCOPY) motor.elf motor.hex


# program motor.hex
motor_isp: motor
	$(AVR_DUDE) -Uflash:w:motor.hex:i -Ulock:w:0x0F:m


leg: arm_leg.c arm_debug.c leg.c avr_debug.c pwm_routines.c leg.h
	$(GCC) -O3 -o leg arm_leg.c arm_debug.c
	$(AVR_GCC) $(AVR_CFLAGS) -o leg.o leg.c avr_debug.c
	$(AVR_GCC) $(AVR_LFLAGS) -o leg.elf leg.o
	$(AVR_OBJCOPY) leg.elf leg.hex

# program leg.hex
leg_isp: leg
	$(AVR_DUDE) -Uflash:w:leg.hex:i -Ulock:w:0x0F:m


# reset it
reset:
	avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b19200




# ODROID versions
#vision: $(VISION_OBJS)
#	$(GCC_ODROID) -g -o vision $(VISION_OBJS) -lpthread -lm

#$(VISION_OBJS): 
#	$(GCC_ODROID) -g  $(ODROID_CFLAGS) -c $*.c -o $*.o


# PI version
#vision: $(VISION_OBJS)
#	$(GCC_PI) $(PI_CFLAGS) -o vision $(VISION_OBJS) $(OPENCV_PI_LFLAGS) -lpthread -lm
#	$(GCC_PI) $(PI_CFLAGS) -o vision $(VISION_OBJS) -lpthread -lm

#$(VISION_OBJS):
#	$(GCC_PI) $(PI_CFLAGS) $(OPENCV_PI_CFLAGS) -c $*.c -o $*.o
#	$(GCC_PI) $(PI_CFLAGS) -c $*.c -o $*.o

# X86 version
vision: $(VISION_OBJS)
#	$(GXX) -DX86 -O2 -g -o vision $(VISION_OBJS) $(OPENCV_LFLAGS) -lpthread -lm
	$(GXX) -DX86 -O2 -g -o vision $(VISION_OBJS) -lpthread -lm


$(VISION_OBJS):
#	$(GXX) -O2 $(OPENCV_CFLAGS) -g -c $*.c -o $*.o -DX86 -Ijpeg 
	$(GXX) -O2 -g -c $*.c -o $*.o -DX86 -Ijpeg 



car: car.bin

car_remote.hex: car_remote.s
	gpasm -I../copter/pic -o car_remote.hex car_remote.s

car.bin: $(ARM_CAR_OBJS) $(ARM_OBJS)
	$(GCC_ARM) -o car.elf \
		$(ARM_CAR_OBJS) \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-T../copter/stm32f4/main.ld
	$(OBJCOPY) -O binary car.elf car.bin

truck.bin: $(ARM_TRUCK_OBJS) $(ARM_OBJS)
	$(GCC_ARM) -o truck.elf \
		$(ARM_TRUCK_OBJS) \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-T../copter/stm32f4/main.ld
	$(OBJCOPY) -O binary truck.elf truck.bin

truck2.bin: $(ARM_TRUCK2_OBJS) $(ARM_OBJS) motor_table
	$(GCC_ARM) -o truck2.elf \
		$(ARM_TRUCK2_OBJS) \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-T../copter/stm32f4/main.ld
	$(OBJCOPY) -O binary truck2.elf truck2.bin

cam.bin: $(ARM_CAM_OBJS) $(ARM_OBJS)
	$(GCC_ARM) -o cam.elf \
		$(ARM_CAM_OBJS) \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-T../copter/stm32f4/main.ld
	$(OBJCOPY) -O binary cam.elf cam.bin

$(ARM_OBJS) $(ARM_TRUCK_OBJS) arm_motors.o arm_truck2.o $(ARM_CAR_OBJS) $(ARM_CAM_OBJS):
	`cat arm_gcc` -c $< -o $*.o

car: car.hex car_remote.s
	gpasm -I../copter/pic -o car_remote.hex car_remote.s

car.hex: car.o
	$(SDCC) $(SDCC_LFLAGS) -o $@ car.o

car.o: car.c
	$(SDCC) $(SDCC_CFLAGS) -c $< -o $@

sensor: sensor.s
	gpasm -I../copter/pic -o sensor.hex sensor.s

remote: remote_tx.s remote_rx.s
	gpasm -I../copter/pic -o remote_tx.hex remote_tx.s
	gpasm -I../copter/pic -o remote_rx.hex remote_rx.s

motor_table: motor_table.c
	gcc -o motor_table motor_table.c -lm


cfl2.hex: cfl2.s
	gpasm -o cfl2.hex cfl2.s

speedo.hex: $(SRCS)
	gpasm -o speedo.hex speedo.s

blink.hex: blink.s
	gpasm -o blink.hex blink.s -I../copter/pic 

interval.hex: $(INTERVAL_SRCS)
	gpasm -o interval.hex interval.s

boost.hex: boost.s
	gpasm -o boost.hex boost.s

flashlight.hex: flashlight.s
	gpasm -o flashlight.hex flashlight.s

ping.hex: ping.s
	gpasm -o ping.hex ping.s

ping_rx.hex: ping_rx.s pic_mrf49xa.s pic_mrf49xa.inc
	gpasm -o ping_rx.hex ping_rx.s

ping_tx.hex: ping_tx.s pic_mrf49xa.s pic_mrf49xa.inc
	gpasm -o ping_tx.hex ping_tx.s

gain.hex: gain.s
	gpasm -o gain.hex gain.s

preamp.hex: preamp.s
	gpasm -o preamp.hex preamp.s

projector.hex: projector.s
	gpasm -o projector.hex projector.s

turret.hex: turret.s
	gpasm -o turret.hex turret.s

pump.hex: pump.s
	gpasm -o pump.hex pump.s

interval_rec.hex: $(INTERVAL_SRCS)
	gpasm -o interval_rec.hex interval_rec.s

sonar.hex: sonar.s
	gpasm -o sonar.hex sonar.s

usb_programmer: usb_programmer.c parapin.c
	$(GCC) -O3 -o usb_programmer usb_programmer.c parapin.c -I$(USB_DIR) $(USB_LIB) -lpthread -lrt

usb_download: usb_download.c
	$(GCC) -O3 -o usb_download usb_download.c -I$(USB_DIR) $(USB_LIB) -lpthread -lrt

sonar: sonar.c
	$(GCC) -O3 -o sonar sonar.c

parse: parse.c
	$(GCC) -O3 -o parse parse.c  -lm

laser_painting: laser_painting.c laser_birthday.h
	$(GCC) -O3 -o laser_painting laser_painting.c -lpthread -lm

edit_painting: edit_painting.C
	$(GXX) -O3 -o edit_painting edit_painting.C \
		-I../hvirtual/guicast \
		../hvirtual/guicast/$(OBJDIR)/libguicast.a \
		-lX11 \
		-lXext \
		-lXv \
		-lpthread \
		-lm \
		-lpng

compressaudio: compressaudio.c
	$(GCC) -O3 -o compressaudio compressaudio.c

decompressaudio: decompressaudio.c
	$(GCC) -O3 -o decompressaudio decompressaudio.c

pcmtoasm: pcmtoasm.c
	$(GCC) -O3 -o pcmtoasm pcmtoasm.c

mmc.hex: mmc.s mmc.inc pic_util.inc
	gpasm -o $@ mmc.s

sounds.hex: pcmtoasm
	pcmtoasm sounds.inc sounds.s *.pcm
#	pcmtoasm sounds.inc sounds.s 0.pcm
	gpasm -o $@ sounds.s

distance.hex: distance.s distance.inc
	gpasm -o $@ distance.s

lidar_test: lidar_test.c
	gcc -g lidar_test.c -o lidar_test -lm -lX11 -lXext -ljpeg


clean:
	rm -f $(ARM_OBJS) \
		$(VISION_OBJS) \
		vision \
		*.o \
		*.hex \
		*.lst \
		*.asm \
		*.cod \
		*.ps \
		*.z \
		motor_table


test.hex: test.s pic_util.inc
	gpasm -o test.hex test.s

test_button.hex: test_button.s pic_util.inc
	gpasm -o test_button.hex test_button.s

test2.hex: test2.s pic_util.inc
	gpasm -o test2.hex test2.s

test4.hex: test4.s pic_util.inc
	gpasm -o test4.hex test4.s


speedo.o:
	$(SDCC) `cat sdcc_cflags` -c $< -o $*.o


speedo.o: speedo.c

tables: tables.c
	gcc -o tables tables.c -lm

teleprompt.hex: teleprompt.s pic_util.inc
	gpasm -o teleprompt.hex teleprompt.s

teleprompt: teleprompt.C
	$(GXX) -O3 -g -o teleprompt teleprompt.C \
		-I../hvirtual/guicast \
		-I../hvirtual/thirdparty/freetype-2.1.4/include \
		-I$(USB_DIR) \
		$(USB_DIR)/.libs/libusb-1.0.a \
		../hvirtual/guicast/$(OBJDIR)/libguicast.a \
		../hvirtual/thirdparty/freetype-2.1.4/$(OBJDIR)/libfreetype.a \
		-lX11 \
		-lXext \
		-lXv \
		-lpthread \
		-lm \
		-lpng \
		-lrt



oscilloscope: oscilloscope.c
	$(GXX) -O3 -g -o oscilloscope oscilloscope.c \
		-I$(USB_DIR) \
		$(USB_DIR)/.libs/libusb-1.0.a \
		-lpthread \
		-lrt



../copter/arm/arm_math.o: 	     ../copter/arm/arm_math.c
../copter/arm/uart.o: 		     ../copter/arm/uart.c
../copter/arm/linux.o:	       ../copter/arm/linux.c
../copter/stm32f4/startup_main.o:    ../copter/stm32f4/startup_main.s
../copter/stm32f4/misc.o: 	     ../copter/stm32f4/misc.c
../copter/stm32f4/stm32f4xx_rcc.o:   ../copter/stm32f4/stm32f4xx_rcc.c
../copter/stm32f4/stm32f4xx_usart.o: ../copter/stm32f4/stm32f4xx_usart.c
../copter/stm32f4/stm32f4xx_gpio.o:  ../copter/stm32f4/stm32f4xx_gpio.c
../copter/stm32f4/stm32f4xx_dcmi.o:  ../copter/stm32f4/stm32f4xx_dcmi.c
../copter/stm32f4/stm32f4xx_dma.o:   ../copter/stm32f4/stm32f4xx_dma.c
../copter/stm32f4/stm32f4xx_i2c.o:   ../copter/stm32f4/stm32f4xx_i2c.c
stm32f4xx_it.o:    stm32f4xx_it.c
../copter/stm32f4/stm32f4xx_iwdg.o:  ../copter/stm32f4/stm32f4xx_iwdg.c
../copter/stm32f4/stm32f4xx_tim.o:   ../copter/stm32f4/stm32f4xx_tim.c
../copter/stm32f4/stm32f4xx_adc.o:   ../copter/stm32f4/stm32f4xx_adc.c
../copter/stm32f4/stm32f4xx_flash.o: ../copter/stm32f4/stm32f4xx_flash.c
stm32f4xx_exti.o: stm32f4xx_exti.c
../copter/stm32f4/stm32f4xx_syscfg.o: ../copter/stm32f4/stm32f4xx_syscfg.c
../copter/stm32f4/system_stm32f4xx.o: ../copter/stm32f4/system_stm32f4xx.c
arm_cam.o: 			     arm_cam.c
arm_car.o: 			     arm_car.c
arm_motors.o:                        arm_motors.c
arm_nav.o:                           arm_nav.c
arm_truck.o: 			     arm_truck.c
arm_truck2.o: 			     arm_truck2.c
arm_cc1101.o: 			     arm_cc1101.c
arm_xbee.o: 			     arm_xbee.c
arm_hardi2c.o:                       arm_hardi2c.c
arm_imu.o: 			     arm_imu.c
vision.o: vision.c
vision_surface.o: vision_surface.c
vision_engine.o: vision_engine.c
vision_server.o: vision_server.c


