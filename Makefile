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
.PHONY: all dox hex clean

all: hex

hex:
	@(cd firmware; $(MAKE))

clean:
	$(REMOVE) $(OBJS) $(PRG).elf $(PRG).hex

dox: doc/moski.pdf

doc/moski.pdf:
	@doxygen
	@make -C latex refman.pdf
	@mv latex/refman.pdf doc/moski.pdf
	@rm -r latex/

