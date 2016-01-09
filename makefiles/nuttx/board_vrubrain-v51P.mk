#
# Board-specific definitions for the MICRO VRBRAIN 5.1
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRUBRAIN_V51
CONFIG_SUBTYPE_BOARD = $(CONFIG_BOARD)P

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
