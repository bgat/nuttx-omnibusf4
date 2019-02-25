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
#include <limits.h>
#include <nuttx/mutex.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu60x0.h>

/* sets bit @n */
#define BIT(n) (1 << (n))

/* creates a mask of @m bits, i.e. MASK(2) -> 00000011 */
#define MASK(m) (BIT((m) + 1) - 1)

/* masks and shifts @v into bit field @m */
#define TO_BITFIELD(m,v) ((v) & MASK(m ##__width) << (m ##__shift))

/* un-masks and un-shifts bit field @m from @v */
#define FROM_BITFIELD(m,v) (((v) >> (m ##__shift)) & MASK(m ## __width))



typedef enum {
	SELF_TEST_X = 0x0d,
	SELF_TEST_Y = 0x0e,
	SELF_TEST_Z = 0x0f,
	SELF_TEST_A = 0x10,
	SMPLRT_DIV = 0x19,

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
	GYRO_CONFIG__XG_ST = BIT(7),
	GYRO_CONFIG__YG_ST = BIT(6),
	GYRO_CONFIG__ZG_ST = BIT(5),
	GYRO_CONFIG__FS_SEL__shift = 3,
	GYRO_CONFIG__FS_SEL__width = 2,
	
	ACCEL_CONFIG = 0x1c,
	ACCEL_CONFIG__XA_ST = BIT(7),
	ACCEL_CONFIG__YA_ST = BIT(6),
	ACCEL_CONFIG__ZA_ST = BIT(5),
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
	INT_PIN_CFG__INT_LEVEL = BIT(7),
	INT_PIN_CFG__INT_OPEN = BIT(6),
	INT_PIN_CFG__LATCH_INT_EN = BIT(5),
	INT_PIN_CFG__INT_RD_CLEAR = BIT(4),
	INT_PIN_CFG__FSYNC_INT_LEVEL = BIT(3),
	INT_PIN_CFG__FSYNC_INT_EN = BIT(2),
	INT_PIN_CFG__I2C_BYPASS_EN = BIT(1),

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
	SIGNAL_PATH_RESET__GYRO_RESET = BIT(2),
	SIGNAL_PATH_RESET__ACCEL_RESET = BIT(1),
	SIGNAL_PATH_RESET__TEMP_RESET = BIT(0),
	SIGNAL_PATH_RESET__ALL_RESET = BIT(3) - 1,

	MOT_DETECT_CTRL = 0x69,

	USER_CTRL = 0x6a,
	USER_CTRL__FIFO_EN = BIT(6),
	USER_CTRL__I2C_MST_EN = BIT(5),
	USER_CTRL__I2C_IF_DIS = BIT(4),
	USER_CTRL__FIFO_RESET = BIT(2),
	USER_CTRL__I2C_MST_RESET = BIT(1),
	USER_CTRL__SIG_COND_RESET = BIT(0),

	PWR_MGMT_1 = 0x6b, /* reset: 0x40 */
	PWR_MGMT_1__DEVICE_RESET = BIT(7),
	PWR_MGMT_1__SLEEP = BIT(6),
	PWR_MGMT_1__CYCLE = BIT(5),
	PWR_MGMT_1__TEMP_DIS = BIT(3),
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

/*
 * Describes the mpu60x0 sensor register file. This structure reflects the
 * underlying hardware, so don't change it!
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

/* used by the driver to manage the device */
struct mpu_dev_s
{
	FAR struct spi_dev_s* spi;       /* SPI instance */
	mutex_t lock;                    /* mutex for this structure */
	struct mpu_config_s* config;

	struct sensor_data_s buf;        /* temporary buffer (for read(), etc.) */
	size_t bufpos;                   /* cursor into @buf, in bytes (!) */
};

/* NOTE: in the following, functions named with a double leading-underscore
 * must be invoked ONLY if the mpu_dev_s lock is held!
 */

/*
 * TODO: this should really be __mpu_spi_transfer(), since read() and write()
 * are basically the same thing...
 */
static int __mpu_read_reg(FAR struct mpu_dev_s* dev,
			  mpu_regaddr_t reg_addr,
			  uint8_t *buf, uint8_t len)
{
	if (dev->spi == NULL)
		return -ENODEV;

	int ret = len;

	SPI_LOCK(dev->spi, true);
	SPI_SETMODE(dev->spi, SPIDEV_MODE0);

	/* read at 20MHz if it's a data register, 1MHz otherwise (per
	 * datasheet) */
	if ((reg_addr >= ACCEL_XOUT_H)
	    && ((reg_addr + len) <= I2C_SLV0_DO))
		SPI_SETFREQUENCY(dev->spi, 20000000);
	else
		SPI_SETFREQUENCY(dev->spi, 1000000);

	SPI_SELECT(dev->spi, dev->config->spi_devid, true);

	/* send the read request */
	SPI_SEND(dev->spi, reg_addr | MPU_REG_READ);

	/* clock in the data */
	while (0 != len--)
		*buf++ = (uint8_t)(SPI_SEND(dev->spi, 0xff));

	SPI_SELECT(dev->spi, dev->config->spi_devid, false);
	SPI_LOCK(dev->spi, false);

	return ret;
}

static int __mpu_write_reg(FAR struct mpu_dev_s* dev,
			   mpu_regaddr_t reg_addr,
			   const uint8_t *buf, uint8_t len)
{
	if (dev->spi == NULL)
		return -ENODEV;

	int ret = len;

	SPI_LOCK(dev->spi, true);
	SPI_SETMODE(dev->spi, SPIDEV_MODE0);

	/* all writeable registers use 1MHz */
	SPI_SETFREQUENCY(dev->spi, 1000000);

	SPI_SELECT(dev->spi, dev->config->spi_devid, true);

	/* send the write request */
	SPI_SEND(dev->spi, reg_addr | MPU_REG_WRITE);

	/* send the data */
	while (0 != len--)
		SPI_SEND(dev->spi, *buf++);

	SPI_SELECT(dev->spi, dev->config->spi_devid, false);
	SPI_LOCK(dev->spi, false);

	return ret;
}

/* reads the whole IMU data file in one pass, which is necessary to assure that
 * the values are all sampled as close to the same time as the hardware
 * permits */
static inline int __mpu_read_imu(FAR struct mpu_dev_s* dev, struct sensor_data_s* buf)
{
	return __mpu_read_reg(dev, ACCEL_XOUT_H, (uint8_t*)buf, sizeof(*buf));
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

/*
 * GYRO_CONFIG(0x1b) :   XG_ST YG_ST ZG_ST FS_SEL1 FS_SEL0 x  x  x
 *
 *    XG_ST, YG_ST, ZG_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z gyros
 *
 *    FS_SEL[10] : full-scale range select
 *         0 -> +/-  250 deg/sec
 *         1 -> +/-  500 deg/sec
 *         2 -> +/- 1000 deg/sec
 *         3 -> +/- 2000 deg/sec
 */
static inline int __mpu_write_GYRO_CONFIG(FAR struct mpu_dev_s* dev, uint8_t fs_sel)
{
	uint8_t val = TO_BITFIELD(GYRO_CONFIG__FS_SEL, fs_sel);
	return __mpu_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
}

/*
 * ACCEL_CONFIG(0x1c) :   XA_ST YA_ST ZA_ST AFS_SEL1 AFS_SEL0 x  x  x
 *
 *    XA_ST, YA_ST, ZA_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z accelerometers
 *
 *    AFS_SEL[10] : full-scale range select
 *         0 -> +/-  2 g
 *         1 -> +/-  4 g
 *         2 -> +/-  8 g
 *         3 -> +/- 16 g
 */
static inline int __mpu_write_ACCEL_CONFIG(FAR struct mpu_dev_s* dev, uint8_t afs_sel)
{
	uint8_t val = TO_BITFIELD(ACCEL_CONFIG__AFS_SEL, afs_sel);
	return __mpu_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
}

/*
 * CONFIG (0x1a) :   x   x   EXT_SYNC_SET[2..0] DLPF_CFG[2..0]
 *
 *    EXT_SYNC_SET  : frame sync bit position
 *    DLPF_CFG      : digital low-pass filter bandwidth
 * (see datasheet)
 */
static inline int __mpu_write_CONFIG(FAR struct mpu_dev_s* dev,
				     uint8_t ext_sync_set, uint8_t dlpf_cfg)
{
	uint8_t val = TO_BITFIELD(CONFIG__EXT_SYNC_SET, ext_sync_set)
		| TO_BITFIELD(CONFIG__DLPF_CFG, dlpf_cfg);
	return __mpu_write_reg(dev, CONFIG, &val, sizeof(val));
}

/*
 * WHO_AM_I (0x75) : read-only, always returns 0x68 for mpu60x0
 */
static inline uint8_t __mpu_read_WHO_AM_I(FAR struct mpu_dev_s* dev)
{
	uint8_t val = 0xff;
	__mpu_read_reg(dev, WHO_AM_I, &val, sizeof(val));
	return val;
}

/*
 * Locks and unlocks the @dev data structure (mutex)
 */
static void inline mpu_lock(FAR struct mpu_dev_s* dev)
{
	nxmutex_lock(&dev->lock);
}
static void inline mpu_unlock(FAR struct mpu_dev_s* dev)
{
	nxmutex_unlock(&dev->lock);
}

/*
 * Resets the mpu60x0, sets it to a default configuration
 */
static int mpu_reset(FAR struct mpu_dev_s* dev)
{
	mpu_lock(dev);
	
	/* awaken chip, issue hardware reset */
	__mpu_write_PWR_MGMT_1(dev, PWR_MGMT_1__DEVICE_RESET);

	/* wait for reset cycle to finish (note: per the datasheet, we don't
	 * need to hold NSS for this) */
	do {
		up_mdelay(50); /* msecs (arbitrary) */
	} while (__mpu_read_PWR_MGMT_1(dev) & PWR_MGMT_1__DEVICE_RESET);

	/* reset signal paths */
	__mpu_write_SIGNAL_PATH_RESET(dev, SIGNAL_PATH_RESET__ALL_RESET);
	up_mdelay(2);
	
	/* disable SLEEP, use PLL with z-axis clock source */
	__mpu_write_PWR_MGMT_1(dev, 3);
	up_mdelay(2);
	
	/* disable i2c if we're on spi (right now SPI is all we support, but
	 * that won't be true forever) */
	if (dev->spi)
		__mpu_write_USER_CTRL(dev, USER_CTRL__I2C_IF_DIS);

	/* disable low-power mode, enable all gyros and accelerometers */
	__mpu_write_PWR_MGMT_2(dev, 0);

	/* no FSYNC, set accel LPF at 184 Hz, gyro LPF at 188 Hz */
	__mpu_write_CONFIG(dev, 0, 1);

	/* +/- 1000 deg/sec */
	__mpu_write_GYRO_CONFIG(dev, 2);

	/* +/- 8g */
	__mpu_write_ACCEL_CONFIG(dev, 2);

	/* clear INT on any read (we aren't using that pin right now) */
	__mpu_write_INT_PIN_CFG(dev, INT_PIN_CFG__INT_RD_CLEAR);

	mpu_unlock(dev);
	return 0;
}


/****************************************************************************
 * Name: mpu_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Don't do that.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* reset the register cache */
  mpu_lock(dev);
  dev->bufpos = 0;
  mpu_unlock(dev);

  return 0;
}

/****************************************************************************
 * Name: mpu_close
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* reset the register cache */
  mpu_lock(dev);
  dev->bufpos = 0;
  mpu_unlock(dev);

  return 0;
}

/****************************************************************************
 * Name: mpu_read
 *
 * Returns a snapshot of the accelerometer, temperature, and gyro registers.
 *
 * Note: the chip uses traditional, twos-complement notation, i.e. "0" is
 * encoded as 0, and full-scale-negative is 0x8000, and full-scale-positive is
 * 0x7fff. If we read the registers sequentially and directly into memory (we
 * do), the measurements from each sensor are stored as big endian words.
 *
 * In contrast, ASN.1 maps "0" to 0x8000, full-scale-negative to 0, and
 * full-scale-positive to 0xffff. So if we want to send in a format that an
 * ASN.1 PER-decoder would recognize, treat the chip values as unsigned, add
 * 0x8000 to each measurement, and send each word in big-endian order. The
 * result will be something you would describe like this (confirmed with
 * asn1scc):
 *
 *    Sint16  ::= INTEGER(-32768..32767)
 *
 *    Mpu60x0Sample ::= SEQUENCE {
 *        accel-X  Sint16,
 *        accel-Y  Sint16,
 *        accel-Z  Sint16,
 *        temp     Sint16,
 *        gyro-X   Sint16,
 *        gyro-Y   Sint16,
 *        gyro-Z   Sint16
 * }
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file* filep, FAR char* buf, size_t len)
{
	FAR struct inode* inode = filep->f_inode;
	FAR struct mpu_dev_s* dev = inode->i_private;

	mpu_lock(dev);

	/* populate the register cache if it seems empty */
	if (!dev->bufpos)
		__mpu_read_imu(dev, &dev->buf);
  
	/* how many bytes are available to send? */
	size_t send_len = sizeof(dev->buf) - dev->bufpos;

	/* send as many as will fit */
	if (send_len > len)
		send_len = len;
	if (send_len)
		memcpy(buf, ((uint8_t*)&dev->buf) + dev->bufpos, send_len);

	/* move the cursor, to mark them as sent */
	dev->bufpos += send_len;

	/* if we've sent the last byte, reset the buffer */
	if (dev->bufpos >= sizeof(dev->buf))
		dev->bufpos = 0;

	mpu_unlock(dev);
  
	return send_len;
}

/****************************************************************************
 * Name: mpu_write
 ****************************************************************************/

static ssize_t mpu_write(FAR struct file *filep, FAR const char* buf, size_t len)
{
  FAR struct inode* inode = filep->f_inode;
  FAR struct mpu_dev_s* dev = inode->i_private;

  snerr("%s: %p %p %d\n", __FUNCTION__, inode, dev, len);

  return len;
}

/****************************************************************************
 * Name: mpu60x0_seek
 ****************************************************************************/

static off_t mpu_seek(FAR struct file *filep, off_t offset, int whence)
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

static int mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *priv = inode->i_private;

  UNUSED(inode);
  UNUSED(priv);

  snerr("%s: %p %p\n", __FUNCTION__, inode, priv);

  return -EINVAL;
}

static const struct file_operations g_mpu_fops =
{
  mpu_open,
  mpu_close,
  mpu_read,
  mpu_write,
  mpu_seek,
  mpu_ioctl
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
 *   Registers the mpu60x0 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/imu0"
 *   spi      - SPI interface for chip communications
 *   config   - Configuration information 
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Note:
 *   When we someday support i2c, we'll add a parameter for it here; then
 *   users will set either @i2c or @spi to NULL depending on which one
 *   they're using.
 ****************************************************************************/

int mpu60x0_register(FAR const char* path,
                     FAR struct spi_dev_s* spi,
		     /* FAR struct i2c_master_s* i2c, */
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
  memset(priv, 0, sizeof(*priv));
  nxmutex_init(&priv->lock);

  priv->spi = spi;
  priv->config = config;

  ret = register_driver(path, &g_mpu_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register mpu60x0 interface: %d\n", ret);

      nxmutex_destroy(&priv->lock);
      
      kmm_free(priv);
      return ret;
    }

  _info("whoami = %x\n", __mpu_read_WHO_AM_I(priv));

  mpu_reset(priv);

  _info("whoami = %x\n", __mpu_read_WHO_AM_I(priv));

  return 0;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPU60X0 && CONFIG_SPI_EXCHANGE */
