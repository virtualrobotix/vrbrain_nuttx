#
# Board-specific definitions for the VRBRAIN 4.5
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRBRAIN_V45
CONFIG_SUBTYPE_BOARD = $(CONFIG_BOARD)P

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
