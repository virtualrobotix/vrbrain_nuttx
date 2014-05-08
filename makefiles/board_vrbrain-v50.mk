#
# Board-specific definitions for the VRBRAIN 5.0
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRBRAIN_V50

include $(VRBRAIN_MK_DIR)/toolchain_gnu-arm-eabi.mk
