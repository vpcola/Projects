CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding #-nostdlib 
CFLAGS += -I../inc -Iinc -I../lib/inc -I../lib/inc/core -I../lib/inc/peripherals

#SRCS  = stm32f4_discovery.c
SRCS = fonts.c \
       stm32f4_discovery.c \
       stm32f4_discovery_lcd.c \
       stm32f4_discovery_debug.c \
       stm32f4_discovery_sdio_sd.c \
       stm32f4x7_eth.c
       
#       stm32f4_discovery_lis302dl.c

OBJS = $(SRCS:.c=.o)

.PHONY: libboard.a

all: libboard.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libboard.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libboard.a
