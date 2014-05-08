#
# Board-specific definitions for the VRBRAIN 4.0
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRBRAIN_V40

include $(VRBRAIN_MK_DIR)/toolchain_gnu-arm-eabi.mk
