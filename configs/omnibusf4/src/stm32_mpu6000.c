/****************************************************************************
 * configs/omnibusf4/src/stm32_mpu6000.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <nuttx/sensors/mpu60x0.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "omnibusf4.h"


#define SPIPORT_MPU6000   1
#define SPIMINOR_MPU6000  0
#define GPIO_CS_MPU6000   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|	\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_EXTI_MPU6000 (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_50MHz| \
			   GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN4)
#define DEVNODE_MPU6000   "/dev/imu0"

struct mpu60x0_config_s;
extern int mpu60x0_register(FAR const char* path,
			    FAR struct spi_dev_s* spi,
			    FAR struct mpu60x0_config_s* config);

int stm32_mpu6000_initialize(void)
{
	static const char* path = DEVNODE_MPU6000;
	int port = SPIPORT_MPU6000;
	int minor = SPIMINOR_MPU6000;
	int exti = GPIO_EXTI_MPU6000;
	int cs = GPIO_CS_MPU6000;

	stm32_configgpio(GPIO_CS_MPU6000);
	stm32_configgpio(GPIO_EXTI_MPU6000);
		
	UNUSED(path);
	UNUSED(minor);
	/* get the spi bus instance */
	struct spi_dev_s* spi = stm32_spibus_initialize(port);
	if (spi == NULL)
		return -ENODEV;
	/* configure EXTI */
	/* ... */

	/* TODO: callbacks for interrupt, slave-select */
	/* TODO: can we make the driver agnostic to port type? */
	/* TODO: pass the info on to mpu6000_register(...) */
	/* ... */
	int ret = mpu60x0_register(DEVNODE_MPU6000, spi, NULL);

	return 0;
}

