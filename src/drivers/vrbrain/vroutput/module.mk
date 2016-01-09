#
# Interface driver for the VRBRAIN outputs board
#

MODULE_COMMAND	 = vroutput
SRCS		 = vroutput.cpp

MODULE_STACKSIZE = 1200

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os
