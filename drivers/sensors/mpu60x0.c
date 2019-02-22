/****************************************************************************
 * drivers/sensors/mpu60x0.c
 *
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
 * TODO: Theory of Operation
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPU60X0) \
	&& defined(CONFIG_SPI_EXCHANGE)

/* TODO: I2C support and autodetection */
/* TODO: SPI 1MHz for config registers, 20MHz for data registers */
/* TODO: for slave device support, register a new i2c_master device */

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu60x0.h>

/* creates a mask of @m bits, i.e. MASK(2) -> 00000011 */
#define MASK(m) ((1 << ((m) + 1)) - 1)

typedef enum {
	SELF_TEST_X = 0x0d,
	SELF_TEST_Y = 0x0e,
	SELF_TEST_Z = 0x0f,
	SELF_TEST_A = 0x10,
	SMPLRT_DIV = 0x19,

	/* TODO: these help us create fields, but not read them back... */
	/*
	 * __shift : number of empty bits to the right of the field
	 * __width : width of the field, in bits
	 *
	 * single-bit fields don't have __shift or __mask
	 */
	CONFIG = 0x1a,
	CONFIG__EXT_SYNC_SET__shift = 3,
	CONFIG__EXT_SYNC_SET__width = 2,
	CONFIG__DLPF_CFG__shift = 0,
	CONFIG__DLPF_CFG__width = 2,
	
	GYRO_CONFIG = 0x1b,
	GYRO_CONFIG__XG_ST = (1 << 7),
	GYRO_CONFIG__YG_ST = (1 << 6),
	GYRO_CONFIG__ZG_ST = (1 << 5),
	GYRO_CONFIG__FS_SEL__shift = 3,
	GYRO_CONFIG__FS_SEL__width = 2,
	
	ACCEL_CONFIG = 0x1c,
	ACCEL_CONFIG__XA_ST = (1 << 7),
	ACCEL_CONFIG__YA_ST = (1 << 6),
	ACCEL_CONFIG__ZA_ST = (1 << 5),
	ACCEL_CONFIG__AFS_SEL__shift = 3,
	ACCEL_CONFIG__AFS_SEL__width = 2,

	MOT_THR = 0x1f,
	FIFO_EN = 0x23,
	I2C_MST_CTRL = 0x24,
	I2C_SLV0_ADDR = 0x25,
	I2C_SLV0_REG = 0x26,
	I2C_SLV0_CTRL = 0x27,
	I2C_SLV1_ADDR = 0x28,
	I2C_SLV1_REG = 0x29,
	I2C_SLV1_CTRL = 0x2a,
	I2C_SLV2_ADDR = 0x2b,
	I2C_SLV2_REG = 0x2c,
	I2C_SLV2_CTRL = 0x2d,
	I2C_SLV3_ADDR = 0x2e,
	I2C_SLV3_REG = 0x2f,
	I2C_SLV3_CTRL = 0x30,
	I2C_SLV4_ADDR = 0x31,
	I2C_SLV4_REG = 0x32,
	I2C_SLV4_DO = 0x33,
	I2C_SLV4_CTRL = 0x34,
	I2C_SLV4_DI = 0x35, /* RO */
	I2C_MST_STATUS = 0x36, /* RO */

	INT_PIN_CFG = 0x37,
	INT_PIN_CFG__INT_LEVEL = (1 << 7),
	INT_PIN_CFG__INT_OPEN = (1 << 6),
	INT_PIN_CFG__LATCH_INT_EN = (1 << 5),
	INT_PIN_CFG__INT_RD_CLEAR = (1 << 4),
	INT_PIN_CFG__FSYNC_INT_LEVEL = (1 << 3),
	INT_PIN_CFG__FSYNC_INT_EN = (1 << 2),
	INT_PIN_CFG__I2C_BYPASS_EN = (1 << 1),

	INT_ENABLE = 0x38,
	INT_STATUS = 0x3a, /* RO */

	ACCEL_XOUT_H = 0x3b, /* RO */
	ACCEL_XOUT_L = 0x3c, /* RO */
	ACCEL_YOUT_H = 0x3d, /* RO */
	ACCEL_YOUT_L = 0x3e, /* RO */
	ACCEL_ZOUT_H = 0x3f, /* RO */
	ACCEL_ZOUT_L = 0x40, /* RO */
	TEMP_OUT_H = 0x41, /* RO */
	TEMP_OUT_L = 0x42, /* RO */
	GYRO_XOUT_H = 0x43, /* RO */
	GYRO_XOUT_L = 0x44, /* RO */
	GYRO_YOUT_H = 0x45, /* RO */
	GYRO_YOUT_L = 0x46, /* RO */
	GYRO_ZOUT_H = 0x47, /* RO */
	GYRO_ZOUT_L = 0x48, /* RO */

	EXT_SENS_DATA_00 = 0x49, /* RO */
	EXT_SENS_DATA_01 = 0x4a, /* RO */
	EXT_SENS_DATA_02 = 0x4b, /* RO */
	EXT_SENS_DATA_03 = 0x4c, /* RO */
	EXT_SENS_DATA_04 = 0x4d, /* RO */
	EXT_SENS_DATA_05 = 0x4e, /* RO */
	EXT_SENS_DATA_06 = 0x4f, /* RO */
	EXT_SENS_DATA_07 = 0x50, /* RO */
	EXT_SENS_DATA_08 = 0x51, /* RO */
	EXT_SENS_DATA_09 = 0x52, /* RO */
	EXT_SENS_DATA_10 = 0x53, /* RO */
	EXT_SENS_DATA_11 = 0x54, /* RO */
	EXT_SENS_DATA_12 = 0x55, /* RO */
	EXT_SENS_DATA_13 = 0x56, /* RO */
	EXT_SENS_DATA_14 = 0x57, /* RO */
	EXT_SENS_DATA_15 = 0x58, /* RO */
	EXT_SENS_DATA_16 = 0x59, /* RO */
	EXT_SENS_DATA_17 = 0x5a, /* RO */
	EXT_SENS_DATA_18 = 0x5b, /* RO */
	EXT_SENS_DATA_19 = 0x5c, /* RO */
	EXT_SENS_DATA_20 = 0x5d, /* RO */
	EXT_SENS_DATA_21 = 0x5e, /* RO */
	EXT_SENS_DATA_22 = 0x5f, /* RO */
	EXT_SENS_DATA_23 = 0x60, /* RO */
	I2C_SLV0_DO = 0x63,
	I2C_SLV1_DO = 0x64,
	I2C_SLV2_DO = 0x65,
	I2C_SLV3_DO = 0x66,
	I2C_MST_DELAY_CTRL = 0x67,

	SIGNAL_PATH_RESET = 0x68,
	SIGNAL_PATH_RESET__GYRO_RESET = (1 << 2),
	SIGNAL_PATH_RESET__ACCEL_RESET = (1 << 1),
	SIGNAL_PATH_RESET__TEMP_RESET = (1 << 0),
	SIGNAL_PATH_RESET__ALL_RESET = (1 << 3) - 1,

	MOT_DETECT_CTRL = 0x69,

	USER_CTRL = 0x6a,
	USER_CTRL__FIFO_EN = (1 << 6),
	USER_CTRL__I2C_MST_EN = (1 << 5),
	USER_CTRL__I2C_IF_DIS = (1 << 4),
	USER_CTRL__FIFO_RESET = (1 << 2),
	USER_CTRL__I2C_MST_RESET = (1 << 1),
	USER_CTRL__SIG_COND_RESET = (1 << 0),

	PWR_MGMT_1 = 0x6b, /* reset: 0x40 */
	PWR_MGMT_1__DEVICE_RESET = (1 << 7),
	PWR_MGMT_1__SLEEP = (1 << 6),
	PWR_MGMT_1__CYCLE = (1 << 5),
	PWR_MGMT_1__TEMP_DIS = (1 << 3),
	PWR_MGMT_1__CLK_SEL__shift = 0,
	PWR_MGMT_1__CLK_SEL__width = 3,

	PWR_MGMT_2 = 0x6c,
	FIFO_COUNTH = 0x72,
	FIFO_COUNTL = 0x73,
	FIFO_R_W = 0x74,
	WHO_AM_I = 0x75, /* RO reset: 0x68 */
} mpu_regaddr_t;

/* SPI read/write codes */
#define MPU_REG_READ 0x80
#define MPU_REG_WRITE 0

/* set_sample_rate():
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) where Gyroscope
 * Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
 * when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate fixed at is 1kHz. This means that for a
 * Sample Rate greater than 1kHz, the same accelerometer sample may be output
 * to the FIFO, DMP, and sensor registers more than once.
 *
 */


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
struct mpu_config_s
{
	int spi_devid;
};

struct mpu_dev_s
{
  FAR struct spi_dev_s* spi;           /* SPI instance */
  sem_t lock;                          /* mutex for this structure */
  struct mpu_config_s* config;
};


/* TODO: there are ways of combining this with __mpu_write_reg()... */
static int __mpu_read_reg(FAR struct mpu_dev_s* dev,
			  mpu_regaddr_t reg_addr,
			  uint8_t *buf, uint8_t len)
{
	if (dev->spi == NULL)
		return -ENODEV;

	int ret = len;

	SPI_LOCK(dev->spi, true);

	SPI_SETFREQUENCY(dev->spi, 1000000);
	SPI_SETMODE(dev->spi, SPIDEV_MODE0);

	SPI_SELECT(dev->spi, dev->config->spi_devid, true);

	SPI_SEND(dev->spi, reg_addr | MPU_REG_READ);

	while (0 != len--)
		*buf++ = (uint8_t)(SPI_SEND(dev->spi, 0xff));

	SPI_SELECT(dev->spi, dev->config->spi_devid, false);
	SPI_LOCK(dev->spi, false);

	return ret;
}

static inline uint8_t __mpu_read_WHO_AM_I(FAR struct mpu_dev_s* dev)
{
	uint8_t val = 0xff;
	__mpu_read_reg(dev, WHO_AM_I, &val, sizeof(val));
	return val;
}

static int __mpu_write_reg(FAR struct mpu_dev_s* dev,
			   mpu_regaddr_t reg_addr,
			   const uint8_t *buf, uint8_t len)
{
	if (dev->spi == NULL)
		return -ENODEV;

	int ret = len;

	SPI_LOCK(dev->spi, true);

	SPI_SETFREQUENCY(dev->spi, 1000000);
	SPI_SETMODE(dev->spi, SPIDEV_MODE0);

	SPI_SELECT(dev->spi, dev->config->spi_devid, true);

	SPI_SEND(dev->spi, reg_addr | MPU_REG_WRITE);

	while (0 != len--)
		SPI_SEND(dev->spi, *buf++);

	SPI_SELECT(dev->spi, dev->config->spi_devid, false);
	SPI_LOCK(dev->spi, false);

	return ret;
}

/* reads the whole IMU data file in one pass */
static inline int __mpu_read_imu(FAR struct mpu_dev_s* dev, uint8_t* buf, uint8_t len)
{
	uint8_t nreg = (GYRO_ZOUT_L - ACCEL_XOUT_H + 1);
	__mpu_read_reg(dev, ACCEL_XOUT_H, buf, nreg < len ? nreg : len);
	return nreg;
}

static inline uint8_t __mpu_read_PWR_MGMT_1(FAR struct mpu_dev_s* dev)
{
	uint8_t buf = 0xff;
	__mpu_read_reg(dev, PWR_MGMT_1, &buf, sizeof(buf));
	return buf;
}

static inline int __mpu_write_SIGNAL_PATH_RESET(FAR struct mpu_dev_s* dev, uint8_t val)
{
	return __mpu_write_reg(dev, SIGNAL_PATH_RESET, &val, sizeof(val));
}

static inline int __mpu_write_INT_PIN_CFG(FAR struct mpu_dev_s* dev, uint8_t val)
{
	return __mpu_write_reg(dev, INT_PIN_CFG, &val, sizeof(val));
}

static inline int __mpu_write_PWR_MGMT_1(FAR struct mpu_dev_s* dev, uint8_t val)
{
	return __mpu_write_reg(dev, PWR_MGMT_1, &val, sizeof(val));
}

static inline int __mpu_write_PWR_MGMT_2(FAR struct mpu_dev_s* dev, uint8_t val)
{
	return __mpu_write_reg(dev, PWR_MGMT_2, &val, sizeof(val));
}

static inline int __mpu_write_USER_CTRL(FAR struct mpu_dev_s* dev, uint8_t val)
{
	return __mpu_write_reg(dev, USER_CTRL, &val, sizeof(val));
}

static inline int __mpu_write_GYRO_CONFIG(FAR struct mpu_dev_s* dev, uint8_t fs_sel)
{
	uint8_t val = (fs_sel & MASK(GYRO_CONFIG__FS_SEL__width))
		<< GYRO_CONFIG__FS_SEL__shift;

	return __mpu_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
}

static inline int __mpu_write_ACCEL_CONFIG(FAR struct mpu_dev_s* dev, uint8_t afs_sel)
{
	uint8_t val = (afs_sel & MASK(ACCEL_CONFIG__AFS_SEL__width))
		<< ACCEL_CONFIG__AFS_SEL__shift;

	return __mpu_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
}

static inline int __mpu_write_CONFIG(FAR struct mpu_dev_s* dev,
				     uint8_t ext_sync_set, uint8_t dlpf_cfg)
{
	/* TODO: template macro, i.e. REG_FIELD(CONFIG, EXT_SYNC_SET, <value>) */
	uint8_t val = (ext_sync_set & MASK(CONFIG__EXT_SYNC_SET__width))
		<< CONFIG__EXT_SYNC_SET__shift;
	val |= ((dlpf_cfg & MASK(CONFIG__DLPF_CFG__width))
		<< CONFIG__DLPF_CFG__shift);

	return __mpu_write_reg(dev, CONFIG, &val, sizeof(val));
}

static int mpu_reset(FAR struct mpu_dev_s* dev)
{
	/* TODO: lock the dev structure */

	/* awaken chip, issue hardware reset */
	__mpu_write_PWR_MGMT_1(dev, PWR_MGMT_1__DEVICE_RESET);
	/* wait for reset cycle to finish */
	do {
		up_mdelay(50); /* msecs (arbitrary) */
	} while (__mpu_read_PWR_MGMT_1(dev) & PWR_MGMT_1__DEVICE_RESET);

	/* reset signal paths */
	__mpu_write_SIGNAL_PATH_RESET(dev, SIGNAL_PATH_RESET__ALL_RESET);
	up_mdelay(2);
	
	/* disable SLEEP, use PLL with z-axis clock source */
	__mpu_write_PWR_MGMT_1(dev, 3);
	up_mdelay(2);
	
	/* disable i2c if we're on spi */
	if (dev->spi)
		__mpu_write_USER_CTRL(dev, USER_CTRL__I2C_IF_DIS);

	/* disable low-power mode, enable all gyros and accelerometers */
	__mpu_write_PWR_MGMT_2(dev, 0);

	/* no FSYNC, accel LPF 184 Hz, gyro LPF 188 Hz */
	__mpu_write_CONFIG(dev, 0, 1);

	/* +/- 500 deg/sec */
	__mpu_write_GYRO_CONFIG(dev, 1);

	/* +/- 4g */
	__mpu_write_ACCEL_CONFIG(dev, 1);

	/* clear INT on any read */
	__mpu_write_INT_PIN_CFG(dev, INT_PIN_CFG__INT_RD_CLEAR);

	/* TODO: unlock the dev structure */
	return 0;
}
      
#if 0

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

 static int mpu_open(FAR struct file *filep)
{
	return 0;
}

/****************************************************************************
 * Name: mpu60x0_close
 ****************************************************************************/

static int mpu60x0_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *priv = inode->i_private;
  int ret = 0;

  UNUSED(inode);
  UNUSED(priv);

  return ret;
}

/****************************************************************************
 * Name: mpu60x0_read
 ****************************************************************************/

static ssize_t mpu60x0_read(FAR struct file* filep, FAR char* buf, size_t len)
{
  FAR struct inode* inode = filep->f_inode;
  FAR struct mpu_dev_s* dev = inode->i_private;

  if (len > 14) len = 14; /* TODO: find a better way to deal with this */
  ssize_t ret = __mpu_read_imu(dev, (uint8_t*)buf, len);
  return ret;
}

/****************************************************************************
 * Name: mpu60x0_write
 ****************************************************************************/

static ssize_t mpu60x0_write(FAR struct file *filep, FAR const char* buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *priv = inode->i_private;

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
  FAR struct mpu_dev_s *priv = inode->i_private;

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
  FAR struct mpu_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return -EINVAL;
}

static const struct file_operations g_mpu60x0_fops =
{
  mpu_open,
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
		     FAR struct mpu_config_s* config)
{
  FAR struct mpu_dev_s* priv;
  int ret;

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the device structure. */

  priv = (FAR struct mpu_dev_s*)kmm_malloc(sizeof(struct mpu_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate accelerometer instance\n");
      return -ENOMEM;
    }
  nxsem_init(&priv->lock, 0, 1);

  priv->spi = spi;
  priv->config = config;

  ret = register_driver(path, &g_mpu60x0_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register mpu60x0 interface: %d\n", ret);

      nxsem_destroy(&priv->lock);
      
      kmm_free(priv);
      return ret;
    }

  _info("whoami = %x\n", __mpu_read_WHO_AM_I(priv));
  _info("whoami = %x\n", __mpu_read_WHO_AM_I(priv));

  mpu_reset(priv);

  _info("whoami = %x\n", __mpu_read_WHO_AM_I(priv));

  return 0;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPU60X0 && CONFIG_SPI_EXCHANGE */
