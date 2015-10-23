#
# Board-specific startup code for the PX4-STM32F4Discovery
#

SRCS		 = px4link2_init.c \
		   px4link2_led.c \
		   px4link2_spi.c

MAXOPTIMIZATION	 = -Os