#########################################
#
#       CHIP DETAILS + AVRDUDE
#
MCU            := atmega164p
PROGRAMMER_MCU := m164p
AVRDUDE_PROGRAMMERID := usbasp

#########################################
#
#	PROJECT DETAILS
#
PROJECT        := moski

#########################################
#
#       MAKE FUNCTIONS
#
.PHONY: all clean

all: dox

clean:
	$(REMOVE) $(OBJS) $(PRG).elf $(PRG).hex

dox:
	@doxygen
	@make -C latex refman.pdf
	@mv latex/refman.pdf doc/moski.pdf
	@rm -r latex/

