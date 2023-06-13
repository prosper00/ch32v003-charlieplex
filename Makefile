
# main-sw for software PWM version
# main-pwn for timer hardware PWM version
TARGET:=main-sw

all : $(TARGET).bin

# CFLAGS+=-DSTDOUT_UART
CFLAGS+=-DTINYVECTOR
ADDITIONAL_C_FILES+=wiring.c

include ch32v003fun.mk

flash : cv_flash
clean : cv_clean

