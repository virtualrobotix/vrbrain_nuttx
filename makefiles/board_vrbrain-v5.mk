#
# Board-specific definitions for the VRBRAIN
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRBRAIN_V5

include $(VRBRAIN_MK_DIR)/toolchain_gnu-arm-eabi.mk
