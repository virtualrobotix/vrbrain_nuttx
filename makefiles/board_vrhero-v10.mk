#
# Board-specific definitions for the VRHERO 1.0
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRHERO_V10

include $(VRBRAIN_MK_DIR)/toolchain_gnu-arm-eabi.mk
