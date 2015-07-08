##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16 -nostdlib -fstack-protector
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu99
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x500
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x200
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = orchard

# Imported source files and paths
CHIBIOS = ..
# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_kl1x.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/KINETIS/KL1x/platform.mk
include $(CHIBIOS)/os/hal/boards/KOSAGI_KW01_ORCHARD/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
# Other files (optional).
include $(CHIBIOS)/test/rt/test.mk

# uGFX includes
GFXLIB =$(CHIBIOS)/ext/ugfx
include $(GFXLIB)/gfx.mk
include $(GFXLIB)/boards/base/Kosagi-KinetisW-OrchardBM15/board.mk
include $(GFXLIB)/drivers/gdisp/SSD1306/driver.mk

# libfixmath
LIBFIXMATH = $(CHIBIOS)/ext/libfixmath
include $(LIBFIXMATH)/build.mk

#C90TFS Freescale Flash drivers
LIBC90TFS = $(CHIBIOS)/ext/C90TFS
include $(LIBC90TFS)/build.mk

# libfixmatrix
LIBFIXMATRIX = $(CHIBIOS)/ext/libfixmatrix
include $(LIBFIXMATRIX)/build.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/KL16Z128.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = main.c \
       orchard-shell.c \
       orchard-app.c \
       orchard-ui.c \
       orchard-vectors.c \
       orchard-test.c \
       $(wildcard cmd-*.c) \
       $(wildcard app-*.c) \
       $(wildcard ui-*.c) \
       $(wildcard test-*.c) \
       accel.c \
       charger.c \
       gasgauge.c \
       captouch.c \
       gpiox.c \
       oled.c \
       analog.c \
       orchard-events.c \
       orchard-math.c \
       radio.c \
       led.c \
       hex.c \
       hsvrgb.c \
       flash.c \
       storage.c \
       genes.c \
       paging.c \
       userconfig.c \
       gitversion.c \
       $(STARTUPSRC) \
       $(PORTSRC) \
       $(KERNSRC) \
       $(HALSRC) \
       $(OSALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS)/os/various/shell.c \
       $(GFXSRC) \
       $(LIBFIXMATHSRC) \
       $(LIBC90TFSSRC) \
       $(LIBFIXMATRIXSRC) \

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM) ws2812b_ll.s

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) $(LIBFIXMATHINC) \
         $(LIBC90TFSINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) $(GFXINC) \
         $(CHIBIOS)/os/hal/lib/streams $(CHIBIOS)/os/various


#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m0

#TRGT = arm-elf-
#TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
SREC = $(CP) -O srec

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB -msoft-float

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# Keypad layout selection:
#     BM      burning man key layout
#     BC1     bootcamp key layout v1
KEY_LAYOUT ?= BC1

# Board revision values:
#     EVT1    first rev of Burning Man board
#     EVT1B   first rev of the Bootcamp 2015 board
BOARD_REV ?= EVT1B

# ORCHARD_REV_EVT1   is the burning man 1st rev board
# ORCHARD_REV_EVT1B  is the bootcamp 1st rev board
# List all user C define here, like -D_DEBUG=1
UDEFS = -DKEY_LAYOUT=LAYOUT_$(KEY_LAYOUT) -DORCHARD_BOARD_REV=ORCHARD_REV_$(BOARD_REV)

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = lib/armv6-m-libgcc.a lib/libc-nano-armv6-m.a

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

gitversion.c: $(CHIBIOS)/.git/HEAD $(CHIBIOS)/.git/index
	echo "const char *gitversion = \"$(shell git rev-parse HEAD)\";" > $@

POST_MAKE_ALL_RULE_HOOK: build/factory-test

build/factory-test: factory-test.c
	$(CC) factory-test.c -o build/factory-test -Wall -g

factory-test.c:
