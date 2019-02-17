/****************************************************************************
 * drivers/sensors/mpu60x0.c
 * Support for the Invensense MPU6000 and MPU6050 MotionTracking(tm)
 * 6-axis accelerometer and gyroscope.
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright+
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
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPU60X0) \
	&& defined(CONFIG_SPI_EXCHANGE)

/* TODO: I2C support and autodetection */
/* TODO: SPI 1MHz for config registers, 20MHz for data registers */

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu60x0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/

struct sensor_data_s
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
};

/* TODO: config structure */
struct mpu60x0_config_s
{
};

struct mpu60x0_dev_s
{
  FAR struct spi_dev_s* spi;           /* SPI instance */
  sem_t chiplock;                      /* mutex for the physical chip */
  sem_t lock;                          /* mutex for this structure */
  struct mpu60x0_config_s* config;
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/


#if 0

/****************************************************************************
 * Private data storage
 ****************************************************************************/

/* Default accelerometer initialization sequence */

/* Configure ADXL372 to read live data (not using FIFO).
 * 1. Set to standby mode. The below can't be set while running.
 * 2. Configure the FIFO to be bypassed.
 * 3. Configure interrupts as disabled, because ADXL372 irpts are used.
 * 4. Configure the Output Data Rate (ODR) as 1600 Hz.
 * 5. Configure normal mode (vs low noise) and 800Hz bandwidth.
 * 6. Set to operational mode; 370ms filter settle; LPF=enb; HPF=dis;
 */

static struct adxl372_reg_pair_s g_initial_adxl372_cr_values[] =
{
  /* Set to standby mode */

  {
    .addr  = ADXL372_POWER_CTL,
    .value = 0
  },
  {
    .addr  = ADXL372_FIFO_CTL,
    .value = ADXL372_FIFO_BYPASSED
  },

  /* Interrupts disabled. */

  {
    .addr  = ADXL372_INT1_MAP,
    .value = 0
  },
  {
    .addr  = ADXL372_TIMING,
    .value = ADXL372_TIMING_ODR1600
  },
  {
    .addr  = ADXL372_MEASURE,
    .value = ADXL372_MEAS_BW800
  },
  {
    .addr  = ADXL372_POWER_CTL,
    .value = ADXL372_POWER_HPF_DISABLE | ADXL372_POWER_MODE_MEASURE
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl372_read_register
 ****************************************************************************/

static uint8_t adxl372_read_register(FAR struct adxl372_dev_s *dev,
                                     uint8_t reg_addr)
{
  uint8_t reg_data;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read. */

  SPI_SEND(dev->spi, reg_addr | ADXL372_READ);

  /* Write an idle byte while receiving the requested data */

  reg_data = (uint8_t) (SPI_SEND(dev->spi, 0xff));

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}

/******************************************************************************
 * Name: adxl372_read_registerblk
 ******************************************************************************/

 static void adxl372_read_registerblk(FAR struct adxl372_dev_s *dev,
                                      uint8_t reg_addr,
                                      FAR uint8_t *reg_data,
                                      uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading */

  SPI_SEND(dev->spi, reg_addr | ADXL372_READ);

  /* Write idle bytes while receiving the requested data */

  while ( 0 != xfercnt-- )
    {
      *reg_data++ = (uint8_t)SPI_SEND(dev->spi, 0xff);
    }

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_write_register
 ****************************************************************************/

static void adxl372_write_register(FAR struct adxl372_dev_s *dev,
                                   uint8_t reg_addr, uint8_t reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to write */

  SPI_SEND(dev->spi, reg_addr | ADXL372_WRITE);

  /* Transmit the content which should be written into the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_write_registerblk
 ****************************************************************************/

 static void adxl372_write_registerblk(FAR struct adxl372_dev_s *dev,
                                       uint8_t reg_addr,
                                       FAR uint8_t *reg_data,
                                       uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low which selects the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to start writing */

  SPI_SEND(dev->spi, reg_addr | ADXL372_WRITE);

  /* Transmit the content which should be written in the register block */

  while ( 0 != xfercnt-- )
    {
      SPI_SEND(dev->spi, *reg_data++);
    }

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_reset
 *
 * Description:
 *   ADXL Accelerometer Reset
 *   1. Make sure that a reset is not in progress.
 *   2. Write ADXL372_RESET_VALUE (0x52) to ADXL372_RESET register.
 *   3. Wait for the reset to finish.
 *
 ****************************************************************************/

static void adxl372_reset(FAR struct adxl372_dev_s *dev)
{
  uint wdcnt = 10;

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  while (wdcnt > 0 && (0 != adxl372_read_register(dev, ADXL372_RESET)))
    {
      up_mdelay(1);
    }

  /* Reset ADXL372 Accelerometer. Write only. Begin a boot. */

  adxl372_write_register(dev, ADXL372_RESET, ADXL372_RESET_VALUE);

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  wdcnt = 10;
  while (wdcnt>0 && (0 != adxl372_read_register(dev, ADXL372_RESET)))
    {
      up_mdelay(1);
    }
}

/****************************************************************************
 * Name: adxl372_read_id
 *
 * Description:
 *
 *   Read the ADXL372 Accelerometer's ID Registers.
 *   There are 4 ID Registers...
 *
 *     Manufacturer should be ADXL372_DEVID_AD_VALUE (0xAD).
 *     Family should be ADXL372_DEVID_MST_VALUE (0x1D).
 *     Part ID should be ADXL372_PARTID_VALUE (0xFA, Octal 372)
 *     Revision is returned, but not expected to be checked.
 *     All of the above are returned as an uint32_t. Should be 0xAD1DFAxx.
 *
 ****************************************************************************/

static uint32_t adxl372_read_id(FAR struct adxl372_dev_s *dev)
{
  union
  {
    uint32_t adxl_devid32;
    uint8_t  adxl_devid[4];
  } un;

  un.adxl_devid[3] = adxl372_read_register(dev, ADXL372_DEVID_AD);
  un.adxl_devid[2] = adxl372_read_register(dev, ADXL372_DEVID_MST);
  un.adxl_devid[1] = adxl372_read_register(dev, ADXL372_PARTID);
  un.adxl_devid[0] = adxl372_read_register(dev, ADXL372_REVID);
  return un.adxl_devid32;
}

/****************************************************************************
 * Name: adxl372_dvr_open
 ****************************************************************************/

static int adxl372_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;
  FAR struct adxl372_reg_pair_s *initp;
  uint32_t pnpid;
  int sz;
  int ret;
  int i;

#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
#endif

  sninfo("adxl372_open: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  ret = nxsem_trywait(&priv->devicesem);
  if (ret < 0)
    {
      sninfo("INFO: ADXL372 Accelerometer is already opened.\n");
      return -EBUSY;
    }

  /* Read the ID registers */

  pnpid = adxl372_read_id(priv);
  priv->readonly = false;

  sninfo("ADXL372_ID = 0x%08x\n", pnpid);

  if ((pnpid & 0xffffff00) != (ADXL372_DEVID_AD_VALUE << 24 |
                               ADXL372_DEVID_MST_VALUE << 16 |
                               ADXL372_PARTID_VALUE << 8))
    {
      snwarn("ERROR: Invalid ADXL372_ID = 0x%08x\n", pnpid);

      priv->readonly = true;
      set_errno(ENODEV);
    }
  else /* ID matches */
    {
      adxl372_reset(priv);   /* Perform a sensor reset */

      /* Choose the initialization sequence */

      if (priv->config->initial_cr_values_size == 0 ||
          priv->config->initial_cr_values == NULL)
        {
          initp = g_initial_adxl372_cr_values;      /* Default values */
          sz = ADXL372_INITIAL_CR_SIZE;
        }
      else
        {
          initp = priv->config->initial_cr_values;  /* User supplied values */
          sz = priv->config->initial_cr_values_size;
        }

      /* Apply the initialization sequence */

      for (i = 0; i < sz; i++)
        {
          adxl372_write_register(priv, initp[i].addr, initp[i].value);
        }

#ifdef CONFIG_DEBUG_SENSORS_INFO
      /* Read back the content of all control registers for debug purposes */

      reg_content = adxl372_read_register(priv, ADXL372_FIFO_CTL);
      sninfo("ADXL372_FIFO_CTL = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_INT1_MAP);
      sninfo("ADXL372_INT1_MAP = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_TIMING);
      sninfo("ADXL372_TIMING = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_MEASURE);
      sninfo("ADXL372_MEASURE = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_POWER_CTL);
      sninfo("ADXL372_POWER_CTL = 0x%02x\n", reg_content);
#endif
    }

  priv->seek_address = (uint8_t) ADXL372_XDATA_H;
  return OK;
}

/****************************************************************************
 * Name: adxl372_dvr_close
 ****************************************************************************/

static int adxl372_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Perform a reset to place the sensor in standby mode.*/

  adxl372_reset(priv);

  /* Release the sensor */

  nxsem_post(&priv->devicesem);
  return OK;
}

/****************************************************************************
 * Name: adxl372_dvr_read
 ****************************************************************************/

static ssize_t adxl372_dvr_read(FAR void *instance_handle, FAR char *buffer,
                                size_t buflen)
{
  FAR struct adxl372_dev_s *priv = ((FAR struct adxl372_dev_s *)instance_handle);
  union
  {
    int16_t d16;
    char    d8[2];
  } un;
  FAR char *p1;
  FAR char *p2;
  int i;

  DEBUGASSERT(priv != NULL);

  adxl372_read_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                          buflen);

  /* Permute accelerometer data out fields */

  if (priv->seek_address == ADXL372_XDATA_H && buflen >= 6)
    {
      p1 = p2 = buffer;
      for (i=0; i<3; i++)
        {
          un.d8[1] = *p1++;
          un.d8[0] = *p1++;
          un.d16   = un.d16 >> 4;
          *p2++    = un.d8[0];
          *p2++    = un.d8[1];
        }
    }

  return buflen;
}

/****************************************************************************
 * Name: adxl372_dvr_write
 ****************************************************************************/

static ssize_t adxl372_dvr_write(FAR void *instance_handle,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  if (priv->readonly)
    {
      set_errno(EROFS);
      return -1;
    }

  adxl372_write_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                            buflen);

  return buflen;
}

/****************************************************************************
 * Name: adxl372_dvr_seek
 ****************************************************************************/

static off_t adxl372_dvr_seek(FAR void *instance_handle, off_t offset,
                              int whence)
{
  off_t reg;
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  switch (whence)
    {
      case SEEK_CUR:  /* Incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > ADXL372_LAST)
          {
            set_errno(-EINVAL);
            return -1;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* Seek to the 1st X-data register */
        priv->seek_address = ADXL372_XDATA_H;
        break;

      case SEEK_SET:  /* Seek to designated address */
        if (0 > offset || offset > ADXL372_LAST)
          {
            set_errno(-EINVAL);
            return -1;
          }

        priv->seek_address = offset;
        break;

      default:        /* invalid whence */
        set_errno(-EINVAL);
        return -1;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: adxl372_dvr_ioctl
 ****************************************************************************/

static int adxl372_dvr_ioctl(FAR void *instance_handle, int cmd,
                             unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

    default:
      snerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: adxl372_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct adxl372_dev_s.
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adxl372_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;
  FAR struct spi_dev_s *spi = priv->spi;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(spi, true);

  SPI_SETFREQUENCY(spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(spi, ADXL372_SPI_MODE);

  /* Set CS to low which selects the ADXL372 */

  SPI_SELECT(spi, priv->config->spi_devid, true);

  /* Perform an SPI exchange block operation. */

  SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(spi, priv->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);
 }

#endif

/* TODO: file operations */


/****************************************************************************
 * Name: mpu60x0_open
 ****************************************************************************/

 static int mpu60x0_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;
  int ret = 0;

  UNUSED(inode);
  UNUSED(priv);

  return ret;
}

/****************************************************************************
 * Name: mpu60x0_close
 ****************************************************************************/

static int mpu60x0_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;
  int ret = 0;

  UNUSED(inode);
  UNUSED(priv);
snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return ret;
}

/****************************************************************************
 * Name: mpu60x0_read
 ****************************************************************************/

static ssize_t mpu60x0_read(FAR struct file *filep, FAR char* buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return 0;
}

/****************************************************************************
 * Name: mpu60x0_write
 ****************************************************************************/

static ssize_t mpu60x0_write(FAR struct file *filep, FAR const char* buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p %d\n", __FUNCTION__, inode, priv, len);

  return len;
}

/****************************************************************************
 * Name: mpu60x0_seek
 ****************************************************************************/

static off_t mpu60x0_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return 0;
}

/****************************************************************************
 * Name: mpu60x0_ioctl
 ****************************************************************************/

static int mpu60x0_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu60x0_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return -EINVAL;
}

static const struct file_operations g_mpu60x0_fops =
{
  mpu60x0_open,
  mpu60x0_close,
  mpu60x0_read,
  mpu60x0_write,
  mpu60x0_seek,
  mpu60x0_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};


/****************************************************************************
 * Name: mpu60x0_register
 *
 * Description:
 *   Register the mpu60x0 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/acl0"
 *   spi      - SPI interface for chip communications
 *   config   - Configuration information 
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu60x0_register(FAR const char* path,
                     FAR struct spi_dev_s* spi,
		     FAR struct mpu60x0_config_s* config)
{
  FAR struct mpu60x0_dev_s* priv;
  int ret;

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the device structure. */

  priv = (FAR struct mpu60x0_dev_s*)kmm_malloc(sizeof(struct mpu60x0_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate accelerometer instance\n");
      return -ENOMEM;
    }
  nxsem_init(&priv->chiplock, 0, 1);
  nxsem_init(&priv->lock, 0, 1);

  priv->spi = spi;
  priv->config = config;

  ret = register_driver(path, &g_mpu60x0_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register mpu60x0 interface: %d\n", ret);

      nxsem_destroy(&priv->chiplock);
      nxsem_destroy(&priv->lock);
      
      kmm_free(priv);
      return ret;
    }

  return 0;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPU60X0 && CONFIG_SPI_EXCHANGE */
