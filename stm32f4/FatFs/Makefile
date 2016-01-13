CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding #-nostdlib 
CFLAGS += -I../inc -Iinc -I../lib/inc -I../lib/inc/core -I../lib/inc/peripherals
CFLAGS += -I../disext/inc

SRCS = ff.c \
	diskio.c


OBJS = $(SRCS:.c=.o)

.PHONY: libFatFs.a

all: libFatFs.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libFatFs.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libFatFs.a
