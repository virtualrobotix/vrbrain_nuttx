#
# Board-specific definitions for the VRBRAIN 5.1
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRBRAIN_V51

include $(VRX_MK_DIR)/toolchain_gnu-arm-eabi.mk
