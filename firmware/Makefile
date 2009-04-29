#########################################
#
#       CHIP DETAILS + AVRDUDE
#
AVRDUDE              := avrdude
MCU                  := attiny24
PROGRAMMER_MCU       := t24
AVRDUDE_PROGRAMMERID := usbasp
AVRDUDE_PORT         := /dev/parport0



#########################################
#
#	COMPILING C
#
CC       := avr-gcc
OPTLEVEL := s
CFLAGS   :=	-g						\
		   -mmcu=$(MCU)			\
		   -O$(OPTLEVEL)			\
		   -fpack-struct			\
		   -fshort-enums			\
		   -funsigned-bitfields	\
		   -funsigned-char		\
		   -W							\
		   -Wall						\
         -Wextra              \
		   -Wstrict-prototypes	\
		   -I.                  \
         -I..



#########################################
#
#       MISC VARIABLES
#
OBJCOPY  := avr-objcopy
OBJDUMP  := avr-objdump
HEXFORMAT := ihex
COPYOPTS :=		-j .text			\
		         -j .data			\
		         -O $(HEXFORMAT)
SIZE     := avr-size
AVRSIM   := simulavr
GDB      := avr-gdb
PORT     := 1212


#########################################
#
#	PROJECT DETAILS
#
PROJECT  := moski

PRG      := $(PROJECT)

SRC      := moski.c i2cs.c temp.c motors.c encoders.c
ASRC     := 
GCCLIB   :=

OBJS     := $(patsubst %.c,%.o,$(SRC))

GDBINITFILE := gdbinit-$(PRG)

#########################################
#
#       MAKE FUNCTIONS
#
.PHONY: all hex elf writeflash fuses verify install sim clean


all: $(PRG).hex

hex: $(PRG).hex
elf: $(PRG).elf

%.o:		%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(PRG).elf:	$(OBJS)
	$(CC) $(CFLAGS) -o $(PRG).elf $(OBJS) $(GCCLIB)

$(PRG).hex:	$(PRG).elf
	$(OBJCOPY) $(COPYOPTS) $< $@

writeflash:	hex
	$(AVRDUDE) -p $(PROGRAMMER_MCU)	\
			-e			\
			-c $(AVRDUDE_PROGRAMMERID) \
			-P $(AVRDUDE_PORT)	\
			-v			\
			-U flash:w:$(PROJECT).hex:i

verify:		hex
	$(AVRDUDE) -p $(PROGRAMMER_MCU)         \
			-c $(AVRDUDE_PROGRAMMERID) \
			-P $(AVRDUDE_PORT)      \
			-n			\
			-v                      \
			-D			\
			-U flash:v:$(PROJECT).hex:i

#
# FUSES
#
#       7 6 5 4 3 2 1 0
# lfuse 1 1 0 1 1 1 1 1 = 0xDF
#       | | | | | | | |
#       | | | | | | | +-> CKSEL0 \
#       | | | | | | |            |
#       | | | | | | +---> CKSEL1 |
#       | | | | | |              |--> External Crystal Oscillator at 8MHz<
#       | | | | | +-----> CKSEL2 |
#       | | | | |                |
#       | | | | +-------> CKSEL3 /
#       | | | |
#       | | | +---------> SUT0  \
#       | | |                   |---> Crystal Oscillator, BOD enabled
#       | | +-----------> SUT1  /
#       | |
#       | +-------------> CKOUT ----> No clock output
#       |
#       +---------------> CKDIV8 ---> Don't divide by 8
#
#
# hfuse 1 1 0 1 1 1 0 1 = 0xDD
#       | | | | | | | |
#       | | | | | | | +-> BODLEVEL0 \
#       | | | | | | |               |
#       | | | | | | +---> BODLEVEL1 |-> BOD Enabled
#       | | | | | |                 |   Min 2.5V  Typical 2.7V  Max 2.9V
#       | | | | | +-----> BODLEVEL2 /
#       | | | | |
#       | | | | +-------> EESAVE -----> Do not save EEPROM during Chip Erase.
#       | | | |
#       | | | +---------> WDTON ------> Watchdog not enabled by hardware.
#       | | |
#       | | +-----------> SPIEN ------> SPI programming enabled.
#       | |
#       | +-------------> DWEN -------> DebugWIRE disabled.
#       |
#       +---------------> RSTDISBL ---> External reset enabled.
fuses:
	$(AVRDUDE) -p $(PROGRAMMER_MCU)         \
			-c $(AVRDUDE_PROGRAMMERID) \
			-P $(AVRDUDE_PORT)      \
			-n			\
			-v                      \
         -U lfuse:w:0xDF:m \
         -U hfuse:w:0xDD:m


install:	writeflash

sim:		hex
	$(AVRSIM) --gdbserver --device $(MCU) --port $(PORT) &
	
	@echo "file $(PRG).elf" > $(GDBINITFILE)
	@echo "target remote localhost:$(PORT)" \
		>> $(GDBINITFILE)
	@echo "load"        >> $(GDBINITFILE) 
	@echo "break main"  >> $(GDBINITFILE)
	@echo "continue"    >> $(GDBINITFILE)

	$(GDB) -x $(GDBINITFILE)

clean:
	$(RM) $(OBJS) $(PRG).elf $(PRG).hex
