/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4_WS internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <px4_defines.h>
#include <arch/board/board.h>

__BEGIN_DECLS

/* these headers are not C++ safe */


/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define BOARD_LED_GPIO		PINCONF_GPIO0p8
#define BOARD_LED_OUT		(GPIO_MODE_OUTPUT | GPIO_PORT0 | GPIO_PIN8)

/* MPU select is on SSP1 SS*/
#define BOARD_MPU_CS_GPIO	PINCONF_GPIO0p15
#define BOARD_MPU_CS_OUT	(GPIO_MODE_OUTPUT | GPIO_PORT0 | GPIO_PIN15)

#define BOARD_MAG_CS_GPIO	PINCONF_GPIO1p12
#define BOARD_MAG_CS_OUT	(GPIO_MODE_OUTPUT | GPIO_PORT1 | GPIO_PIN12) // 13 for baro

/* bus 2 is ssp1 */
#define PX4_SPI_BUS_SENSORS	2

/* High-resolution timer */
#define HRT_TIMER		0	/* use timer0 for the HRT */
#define HRT_TIMER_CHANNEL	0	/* use capture/compare channel */


/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_GYRO		1
#define PX4_SPIDEV_ACCEL_MAG 2
#define PX4_SPIDEV_BARO		3
#define PX4_SPIDEV_MPU		4
#define PX4_SPIDEV_HMC		5

/* I2C busses */
#define PX4_I2C_BUS_ONBOARD	2

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15


/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void lpc43_spiinitialize(void);


#endif /* __ASSEMBLY__ */

__END_DECLS
