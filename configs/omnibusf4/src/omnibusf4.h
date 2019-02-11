/****************************************************************************
 * configs/omnibusf4/src/omnibusf4.h
 *
 *   Copyright (C) 2019, Bill Gatliff. All rights reserved.
 *   Copyright (C) 2011-2012, 2015-2016, 2018 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __CONFIGS_OMNIBUSF4_SRC_OMNIBUSF4_H
#define __CONFIGS_OMNIBUSF4_SRC_OMNIBUSF4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_RTC_DRIVER 1
#define HAVE_HCIUART    1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#undef  SDIO_MINOR     /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO

#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif

  /* SD card bringup does not work if performed on the IDLE thread because it
   * will cause waiting.  Use either:
   *
   *  CONFIG_LIB_BOARDCTL=y, OR
   *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
   */

#  if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARD_INITTHREAD)
#    warning "SDIO initialization cannot be perfomed on the IDLE thread"
#    undef HAVE_SDIO
#  endif
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we have the prequisites for an HCI UART */

#if !defined(CONFIG_STM32_HCIUART) || !defined(CONFIG_BLUETOOTH_UART)
#  undef HAVE_HCIUART
#elif defined(CONFIG_STM32_USART1_HCIUART)
#  define HCIUART_SERDEV HCIUART1
#elif defined(CONFIG_STM32_USART2_HCIUART)
#  define HCIUART_SERDEV HCIUART2
#elif defined(CONFIG_STM32_USART3_HCIUART)
#  define HCIUART_SERDEV HCIUART3
#elif defined(CONFIG_STM32_USART6_HCIUART)
#  define HCIUART_SERDEV HCIUART6
#elif defined(CONFIG_STM32_UART7_HCIUART)
#  define HCIUART_SERDEV HCIUART7
#elif defined(CONFIG_STM32_UART8_HCIUART)
#  define HCIUART_SERDEV HCIUART8
#else
#  error No HCI UART specifified
#endif

/* TODO: OMNIBUSF4 GPIOs **************************************************/

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)


/* PWM outputs */

#define GPIO_TIM3_CH3OUT  (GPIO_ALT | GPIO_AF2 | GPIO_PORTB | GPIO_PIN0) /* S1_OUT */
#define GPIO_TIM3_CH4OUT  (GPIO_ALT | GPIO_AF2 | GPIO_PORTB | GPIO_PIN1) /* S2_OUT */
#define GPIO_TIM2_CH4OUT  (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN3) /* S3_OUT */
#define GPIO_TIM2_CH3OUT  (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN2) /* S4_OUT */
	
/* TODO: SPI chip selects */

#define GPIO_CS_MEMS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define GPIO_MAX31855_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_MAX6675_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_MAX7219_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)


/* TODO: USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTD|GPIO_PIN5)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif


/* TODO: MicroSD (not available on all OMNIBUSF4's)
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB15       NCD           Pulled up externally
 * PC9        DAT1          Configured by driver
 * PC8        DAT0          "        " "" "    "
 * PC12       CLK           "        " "" "    "
 * PD2        CMD           "        " "" "    "
 * PC11       CD/DAT3       "        " "" "    "
 * PC10       DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_SDIO)
#  define GPIO_SDIO_NCD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                           GPIO_PORTB|GPIO_PIN15)
#endif


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the omnibusf4
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

 /****************************************************************************
  * Name: stm32_i2sdev_initialize
  *
  * Description:
  *   Called to configure I2S chip select GPIO pins for the omnibusf4
  *   board.
  *
  ****************************************************************************/

FAR struct i2s_dev_s *stm32_i2sdev_initialize(int port);

/****************************************************************************
 * Name: stm32_bh1750initialize
 *
 * Description:
 *   Called to configure an I2C and to register BH1750FVI for the
 *   omnibusf4 board.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BH1750FVI
int stm32_bh1750initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 *   Called to configure an I2C and to register BMP180 for the
 *   omnibusf4 board.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int stm32_bmp180initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemgpios(const uint32_t *gpios, int ngpios);
#endif

/****************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize address line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemaddr(int naddrs);
#endif

/****************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemdata(int ndata);
#endif

/****************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_enablefsmc(void);
#endif

/****************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_disablefsmc(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_HCIUART
int hciuart_dev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_OMNIBUSF4_SRC_OMNIBUSF4_H */
