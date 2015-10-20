#
# Board-specific definitions for the PX4_LPC4370LINK2
#

#
# Configure the toolchain, CONFIG_BOARD value is a c define name - do not use special chars
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = PX4LPC4370_LINK2

include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk
