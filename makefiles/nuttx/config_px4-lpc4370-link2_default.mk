#
# Makefile for the px4-lpc4370-link2_default configuration
#

#
# Use the configuration's ROMFS, copy the px4-lpc4370-link2 firmware into
# the ROMFS if it's available
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_test
ROMFS_OPTIONAL_FILES = 

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/mpu9250
MODULES		+= drivers/hmc5883
MODULES		+= modules/sensors
MODULES		+= drivers/mkblctrl

#
# System commands
#
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/param
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/esc_calib
MODULES		+= systemcmds/hardfault_log
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/mtd
MODULES		+= systemcmds/dumpfile
MODULES		+= systemcmds/ver
#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB


#
# Libraries
#
LIBRARIES	+= lib/mathlib/CMSIS
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/conversion
MODULES		+= platforms/nuttx

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                ) \
	$(call _B, cu,                     ,                          2048,  cu_main                    )
