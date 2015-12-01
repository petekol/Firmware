/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4link2_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <chip.h>
#include "lpc43_gpio.h"
#include "lpc43_pinconfig.h"
#include "lpc43_ssp.h"

#include <px4_defines.h>

#include "board_config.h"



/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc43_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

__EXPORT void lpc43_spiinitialize(void) {

	lpc43_pin_config(BOARD_MPU_CS_GPIO);
	lpc43_gpio_config(BOARD_MPU_CS_OUT);

	lpc43_pin_config(BOARD_MAG_CS_GPIO);
	lpc43_gpio_config(BOARD_MAG_CS_OUT);

	lpc43_pin_config(BOARD_BARO_CS_GPIO);
	lpc43_gpio_config(BOARD_BARO_CS_OUT);

	/* deselect all */
	lpc43_gpio_write(BOARD_MPU_CS_OUT, true);
	lpc43_gpio_write(BOARD_MAG_CS_OUT, true);
	lpc43_gpio_write(BOARD_BARO_CS_OUT, true);
}

__EXPORT void lpc43_ssp1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
		bool selected) {
	/* SPI select is active low, so write !selected to select the device */

	lpc43_gpio_write(BOARD_MPU_CS_OUT, (devid == PX4_SPIDEV_MPU)?!selected:true  );
	lpc43_gpio_write(BOARD_MAG_CS_OUT, (devid == PX4_SPIDEV_HMC)?!selected:true  );
	lpc43_gpio_write(BOARD_BARO_CS_OUT, (devid == PX4_SPIDEV_BARO)?!selected:true  );

}

__EXPORT uint8_t lpc43_ssp1status(FAR struct spi_dev_s *dev,
		enum spi_dev_e devid) {
	return ((devid == PX4_SPIDEV_MPU ||
			 devid == PX4_SPIDEV_HMC ||
			 devid == PX4_SPIDEV_BARO
			)?SPI_STATUS_PRESENT:0);
}
