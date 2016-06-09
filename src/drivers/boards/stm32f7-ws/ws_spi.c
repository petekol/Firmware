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
 * @file px4fmu_spi.c
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
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32_spi.h>
#include "board_config.h"
#include <systemlib/err.h>

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
	stm32_configgpio(GPIO_SPI_CS_MPU9250);
	stm32_configgpio(GPIO_SPI_CS_HMC5983);
	stm32_configgpio(GPIO_SPI_CS_BMP280);

	stm32_configgpio(GPIO_DRDY_MPU9250);
	stm32_configgpio(GPIO_DRDY_HMC5983);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
	stm32_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
	stm32_gpiowrite(GPIO_SPI_CS_BMP280, 1);

}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
		bool selected) {
	/* SPI select is active low, so write !selected to select the device */

	stm32_gpiowrite(GPIO_SPI_CS_MPU9250, (devid == PX4_SPIDEV_MPU)?!selected:true  );
	stm32_gpiowrite(GPIO_SPI_CS_HMC5983, (devid == PX4_SPIDEV_HMC)?!selected:true  );
	stm32_gpiowrite(GPIO_SPI_CS_BMP280, (devid == PX4_SPIDEV_BARO)?!selected:true  );

}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev,
		enum spi_dev_e devid) {
	return ((devid == PX4_SPIDEV_MPU ||
			 devid == PX4_SPIDEV_HMC ||
			 devid == PX4_SPIDEV_BARO
			)?SPI_STATUS_PRESENT:0);
}

__EXPORT void board_spi_reset(int ms)
{
	/* off */
	stm32_configgpio(GPIO_SPI_CS_OFF_MPU9250);
	stm32_configgpio(GPIO_SPI_CS_OFF_HMC5983);
	stm32_configgpio(GPIO_SPI_CS_OFF_BMP280);
	stm32_configgpio(GPIO_DRDY_OFF_MPU9250);
	stm32_configgpio(GPIO_DRDY_OFF_HMC5983);


	stm32_gpiowrite(GPIO_SPI_CS_OFF_MPU9250, 0);
	stm32_gpiowrite(GPIO_SPI_CS_OFF_HMC5983, 0);
	stm32_gpiowrite(GPIO_SPI_CS_OFF_BMP280, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_MPU9250, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_HMC5983, 0);

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);



	/* on */
	stm32_spiinitialize();

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);
}
