default: all

Makefile_path := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

Toolchain = arm-none-eabi-

CC = $(Toolchain)gcc
CXX= $(Toolchain)g++
ifeq (, $(shell which ccache))
else
	CC := ccache $(CC)
	CXX:= ccache $(CXX)
endif

CFlags =
LFlags =

# Generic flags
CFlags += \
	-fdiagnostics-color=always \
	-Wall \
	-Wextra \
	-g \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-fno-common \
	-fno-exceptions \
	--static

LFlags += \
	-Wl,--gc-sections \
	-nostartfiles \
	-lm \
	-lstdc++_nano \
	-lc \
	-lg \
	-lrdimon


# MCU specific flags (stm32f401re)
LINKER_SCRIPTS_DIR = $(Makefile_path)/hal_common/linker_scripts

CFlags += \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16

LFlags += \
	-T $(LINKER_SCRIPTS_DIR)/stm32f303.ld


# Use LibOpenCm3
LIBOPENCM3_DIR = $(Makefile_path)/hal_common/libopencm3
CFlags += -I $(LIBOPENCM3_DIR)/include -DSTM32F3
LFlags += -L $(LIBOPENCM3_DIR)/lib -lopencm3_stm32f3

libopencm3:
	$(MAKE) -C $(LIBOPENCM3_DIR) -j

# Openocd configuration
OPENOCD_CFG = /usr/share/openocd/scripts/board/st_nucleo_f3.cfg




all: tsmr.hex

%.cpp.o: %.cpp Makefile
	@$(CXX) $(CFlags) $< -o $@ -c
	@echo CXX $<

%.c.o: %.c Makefile
	@$(CC) $(CFlags) $< -o $@ -c
	@echo CC $<


tsmr.elf: main.c.o \
		lowlevel/can.c.o \
		lowlevel/clock.c.o \
		lowlevel/encoders.c.o \
		lowlevel/gpio.c.o \
		lowlevel/motors.c.o
	@$(CC) $^ $(LFlags) -o $@
	@echo LINK $@


%.hex: %.elf
	@arm-none-eabi-objcopy -Oihex $^ $@
	@echo OBJCOPY $@



%.flash: %.hex
	sudo openocd -f $(OPENOCD_CFG) \
		-c "init" \
		-c "reset init" \
		-c "flash write_image erase $^" \
		-c "reset" \
		-c "shutdown"

clean:
	rm -f *.o *.a *.hex *.elf
	rm -f lowlevel/*.o
