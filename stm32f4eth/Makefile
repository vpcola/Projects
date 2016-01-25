# put your *.o targets here, make should handle the rest!

SRCS =  newlibstubs.c \
	bmp.c \
	dcmi_ov9655.c \
	stm32f4xx_it.c \
	system_stm32f4xx.c \
	netconf.c \
	httpserver-socket.c \
	stm32f4x7_eth_bsp.c \
	fs.c \
	i2c.c \
	main.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
 
ODFLAGS	  = -x

###################################################

vpath %.c src
vpath %.a lib
vpath %.a board
vpath %.a FatFs
vpath %.a lwip
vpath %.a FreeRTOS

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc -Iboard/inc
CFLAGS += -IFatFs
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 
CFLAGS += -IFreeRTOS/include -IFreeRTOS/portable/GCC/ARM_CM3
CFLAGS += -Iconf -Iinc -I./lib/inc -I./lib/inc/core -I./lib/inc/peripherals
CFLAGS += -I./lwip/src/include -I./lwip/src/include/ipv4 -I./lwip/src/include/lwip -I./lwip/src/include/netif
CFLAGS += -I./lwip/port/STM32F4x7 -I./lwip/port/STM32F4x7/arch -I./lwip/port/STM32F4x7/FreeRTOS
CFLAGS += -I./FreeRTOS/include -I./FreeRTOS/portable/GCC/ARM_CM3
CFLAGS += -I./board/inc

SRCS += lib/startup_stm32f4xx.s # add startup file to build
	

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib board FatFs FreeRTOS lwip proj

all: lib board FatFs FreeRTOS lwip proj

lib:
	$(MAKE) -C lib

board:
	$(MAKE) -C board
	
FatFs:
	$(MAKE) -C FatFs	

FreeRTOS:
	$(MAKE) -C FreeRTOS

lwip:
	$(MAKE) -C lwip

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -LFatFs -lFatFs -Lboard -lboard -Llwip -llwip -Llib -lstm32f4  -LFreeRTOS -lFreeRTOS
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -x $(PROJ_NAME).elf > $(PROJ_NAME).dmp
	$(OBJDUMP) -S $(PROJ_NAME).elf > $(PROJ_NAME).lst

clean:
	$(MAKE) -C lib clean
	$(MAKE) -C board clean
	$(MAKE) -C FatFs clean
	$(MAKE) -C lwip clean
	$(MAKE) -C FreeRTOS clean
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).dmp
	rm -f $(PROJ_NAME).lst
