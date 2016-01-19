CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src ./portable/MemMang ./portable/GCC/ARM_CM4F ./CMSIS_RTOS

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding #-nostdlib 
CFLAGS += -I../inc -Iinc -I../lib/inc -I../lib/inc/core -I../lib/inc/peripherals
CFLAGS += -I./include -I./portable/GCC/ARM_CM4F -I./CMSIS_RTOS

#SRCS  = stm32f4_discovery.c
SRCS = port.c \
       list.c \
       queue.c \
       tasks.c \
       event_groups.c \
       timers.c \
       heap_4.c \
       cmsis_os.c 

       
#       stm32f4_discovery_lis302dl.c

OBJS = $(SRCS:.c=.o)

.PHONY: libFreeRTOS.a

all: libFreeRTOS.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libFreeRTOS.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libFreeRTOS.a
