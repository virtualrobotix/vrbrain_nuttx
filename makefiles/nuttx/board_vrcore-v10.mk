#
# Board-specific definitions for the VRcore 1.0
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = VRCORE_V10
CONFIG_SUBTYPE_BOARD = $(CONFIG_BOARD)

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
