#
# Interface driver for the VRBRAIN board
#

MODULE_COMMAND	 = vrinput

SRCS		= controls/controls.c \
              controls/registers.c \
              controls/sbus.c \
              vrinput.cpp

EXTRACFLAGS	= -Wno-error
MODULE_STACKSIZE = 1200

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os
