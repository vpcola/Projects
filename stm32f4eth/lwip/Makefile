CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src/core src/core/ipv4 src/api src/netif port/STM32F4x7/FreeRTOS

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding #-nostdlib 
CFLAGS += -I../conf -I../inc -I../lib/inc -I../lib/inc/core -I../lib/inc/peripherals
CFLAGS += -I./src/include -I./src/include/ipv4 -I./src/include/lwip -I./src/include/netif
CFLAGS += -I./port/STM32F4x7 -I./port/STM32F4x7/arch -I./port/STM32F4x7/FreeRTOS
CFLAGS += -I../FreeRTOS/include -I../FreeRTOS/portable/GCC/ARM_CM3
CFLAGS += -I../board/inc

SRCS = api_lib.c api_msg.c err.c \
       netbuf.c netdb.c netifapi.c \
       sockets.c tcpip.c dhcp.c \
       dns.c init.c mem.c memp.c netif.c \
       pbuf.c raw.c stats.c sys.c tcp.c \
       tcp_in.c tcp_out.c udp.c autoip.c \
       icmp.c igmp.c inet.c inet_chksum.c \
       ip.c ip_addr.c ip_frag.c etharp.c \
       loopif.c slipif.c ethernetif.c \
       sys_arch.c

OBJS = $(SRCS:.c=.o)

.PHONY: liblwip.a

all: liblwip.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

liblwip.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) liblwip.a
