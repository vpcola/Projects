# put your *.o targets here, make should handle the rest!

SRCS =  newlibstubs.c bmp.c dcmi_ov9655.c stm32f4xx_it.c system_stm32f4xx.c i2c.c main.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJSIZE=arm-none-eabi-size

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
 
ODFLAGS	  = -x

###################################################

vpath %.c src
vpath %.a lib
vpath %.a disext
vpath %.a FatFs
vpath %.a FreeRTOS

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc -Idisext/inc
CFLAGS += -IFatFs
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 
CFLAGS += -IFreeRTOS/CMSIS_RTOS -IFreeRTOS/include -IFreeRTOS/portable/GCC/ARM_CM4F

LDFLAGS += -Wl,-Map,$(PROJ_NAME).map

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib disext FatFs FreeRTOS proj

all: lib disext FatFs FreeRTOS proj

lib:
	$(MAKE) -C lib

disext:
	$(MAKE) -C disext
	
FatFs:
	$(MAKE) -C FatFs	

FreeRTOS:
	$(MAKE) -C FreeRTOS

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Ldisext -ldisext -LFatFs -lFatFs -Llib -lstm32f4 -LFreeRTOS -lFreeRTOS $(LDFLAGS)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -x $(PROJ_NAME).elf > $(PROJ_NAME).dmp
	@$(OBJSIZE) -d $(PROJ_NAME).elf 

clean:
	$(MAKE) -C lib clean
	$(MAKE) -C disext clean
	$(MAKE) -C FatFs clean
	$(MAKE) -C FreeRTOS clean
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).dmp
	rm -f $(PROJ_NAME).map
