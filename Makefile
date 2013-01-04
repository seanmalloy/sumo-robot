# WinAVR Sample makefile written by Eric B. Weddington, Jörg Wunsch, et al.
# Released to the Public Domain
# Please read the make user manual!
#
# Additional material for this makefile was submitted by:
#  Tim Henigan
#  Peter Fleury
#  Reiner Patommel
#  Sander Pool
#  Frederik Rouleau
#  Markus Pfaff
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF (for use with AVR Studio 3.x or VMLAB).
#
# make extcoff = Convert ELF to AVR Extended COFF (for use with AVR Studio
#                4.07 or greater).
#
# make program = Download the hex file to the device, using avrdude.  Please
#                customize the avrdude settings below first!
#
# make filename.s = Just compile filename.c into the assembler code only
#
# To rebuild project do "make clean" then "make all".
#


# Processor type
MCU = atmega8

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# Target file name (without extension).
TARGET = sumo

# Optimization level, can be [0, 1, 2, 3, s]. 0 turns off optimization.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s


# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).cpp

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
# ASRC =


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = .,/usr/local/avr/include


# Optional compiler flags.
#  -g:                   generate debugging information (for GDB, or for COFF conversion)
#  -O*:                  optimization level
#  -f...:                tuning, see gcc manual and avr-libc documentation
#  -Wall...:             warning level
#  -Wa,...:              tell GCC to pass this to the assembler.
#  -ahlms:               create assembler listing
#  -Wstrict-prototypes   used for C only
CFLAGS = -g -O$(OPT) \
-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
-Wall -I .,/usr/local/avr/include \
-mmcu=$(MCU) -o $(TARGET)


# Define directories, if needed.
# DIRAVR = c:/winavr
# DIRAVRBIN = $(DIRAVR)/bin
# DIRAVRUTILS = $(DIRAVR)/utils/bin
# DIRINC = .
# DIRLIB = $(DIRAVR)/avr/lib

DIRAVR = /usr/local
DIRAVRBIN = $(DIRAVR)/bin
DIRAVRUTILS = $(DIRAVR)/utils/bin
DIRINC = .
DIRLIB = $(DIRAVR)/avr/lib

# avrdude vars
AVRDUDE = /home/sean/avrdude/bin/avrdude

# Define programs and commands.
SHELL = ksh
CC = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size


# HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
# ELFSIZE = $(SIZE) -A $(TARGET).elf



# Define all object files.
# OBJ = $(SRC:.c=.o) $(ASRC:.S=.o)

# Define all listing files.
# LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
# ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
# ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)



# Make binary.
prog:
	$(CC) $(CFLAGS) $(SRC)

hex:
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET) $(TARGET).hex

upload:
	$(AVRDUDE) -p m8 -c stk500v2 -P /dev/ttyU0 -U flash:w:$(TARGET).hex

# Target: clean project.
clean:
	rm -f *.o *.hex *~ a.out $(TARGET)

