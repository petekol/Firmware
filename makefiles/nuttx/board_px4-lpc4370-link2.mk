#
# Board-specific definitions for the PX4_LPC4370LINK2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = PX4_LPC4370-LINK2

include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk
