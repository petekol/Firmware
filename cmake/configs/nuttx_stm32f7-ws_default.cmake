include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_uavcan_num_ifaces 1)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32f7
	drivers/stm32f7/adc
	drivers/stm32f7/tone_alarm
	drivers/led
	drivers/boards/stm32f7-ws
	drivers/mpu9250
	drivers/hmc5883
	drivers/gps
	drivers/pwm_out_sim
	drivers/hott
	drivers/hott/hott_telemetry
	drivers/hott/hott_sensors
	drivers/blinkm
	modules/sensors
	drivers/mkblctrl
	#drivers/pwm_input
	drivers/bmp280

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	# systemcmds/hardfault_log
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/sd_bench
	systemcmds/tests

	#
	# General system control
	#
	modules/commander
	# modules/load_mon
	modules/navigator
	modules/mavlink
	# modules/gpio_led
	# modules/uavcan
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/ekf2
	modules/local_position_estimator

	#
	# Vehicle Control
	#
	# modules/segway # XXX Needs GCC 4.7 fix
	modules/fw_pos_control_l1
	modules/fw_att_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	#modules/sdlog2
	#modules/logger

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	#lib/rc
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/DriverFramework/framework
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common
	platforms/nuttx/px4_layer

	#
	# OBC challenge
	#
	modules/bottle_drop
)

set(config_extra_builtin_cmds
	##serdis
	##sercon
	)

set(config_extra_libs
	)

set(config_io_extra_libs
	)

##add_custom_target(sercon)
##set_target_properties(sercon PROPERTIES
##	PRIORITY "SCHED_PRIORITY_DEFAULT"
##	MAIN "sercon"
##	STACK_MAIN "2048")

##add_custom_target(serdis)
##set_target_properties(serdis PROPERTIES
##	PRIORITY "SCHED_PRIORITY_DEFAULT"
##	MAIN "serdis"
##	STACK_MAIN "2048")
