#
# Board-specific definitions for the PX4_LPC43370WS
#

#
# Configure the toolchain, CONFIG_BOARD value is a c define name - do not use special chars
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = PX4LPC4337_WS

include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk
