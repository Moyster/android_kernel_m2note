/*
 * STMicroelectronics lsm6ds3 core driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <asm/unaligned.h>

#include <linux/iio/common/st_sensors.h>
#include "st_lsm6ds3.h"

#include <linux/meizu-sys.h>
#include <linux/meizu-sensors.h>

#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <cust_eint.h>

#define MS_TO_NS(msec)				((msec) * 1000 * 1000)

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#define MIN_BNZ(a, b)				(((a) < (b)) ? ((a == 0) ? \
						(b) : (a)) : ((b == 0) ? \
						(a) : (b)))

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define ST_LSM6DS3_WAI_ADDRESS			0x0f
#define ST_LSM6DS3_WAI_EXP			0x69
#define ST_LSM6DS3_AXIS_EN_MASK			0x38
#define ST_LSM6DS3_INT1_ADDR			0x0d
#define ST_LSM6DS3_INT2_ADDR			0x0e
#define ST_LSM6DS3_MD1_ADDR			0x5e
#define ST_LSM6DS3_ODR_LIST_NUM			5
#define ST_LSM6DS3_ODR_POWER_OFF_VAL		0x00
#define ST_LSM6DS3_ODR_26HZ_VAL			0x02
#define ST_LSM6DS3_ODR_52HZ_VAL			0x03
#define ST_LSM6DS3_ODR_104HZ_VAL		0x04
#define ST_LSM6DS3_ODR_208HZ_VAL		0x05
#define ST_LSM6DS3_ODR_416HZ_VAL		0x06
#define ST_LSM6DS3_FS_LIST_NUM			4
#define ST_LSM6DS3_BDU_ADDR			0x12
#define ST_LSM6DS3_BDU_MASK			0x40
#define ST_LSM6DS3_EN_BIT			0x01
#define ST_LSM6DS3_DIS_BIT			0x00
#define ST_LSM6DS3_FUNC_EN_ADDR			0x19
#define ST_LSM6DS3_FUNC_EN_MASK			0x04
#define ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define ST_LSM6DS3_FUNC_CFG_ACCESS_MASK2	0x04
#define ST_LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define ST_LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define ST_LSM6DS3_FUNC_CFG_START2_ADDR		0x63
#define ST_LSM6DS3_PASS_THROUGH_MODE_ADDR	0x1a
#define ST_LSM6DS3_PASS_THROUGH_MODE_MASK	0x04
#define ST_LSM6DS3_INTER_PULLUP_ADDR		0x1a
#define ST_LSM6DS3_INTER_PULLUP_MASK		0x08
#define ST_LSM6DS3_SENSORHUB_ADDR		0x1a
#define ST_LSM6DS3_SENSORHUB_MASK		0x01
#define ST_LSM6DS3_STARTCONFIG_ADDR		0x1a
#define ST_LSM6DS3_STARTCONFIG_MASK		0x10
#define ST_LSM6DS3_SELFTEST_ADDR		0x14
#define ST_LSM6DS3_SELFTEST_ACCEL_MASK		0x03
#define ST_LSM6DS3_SELFTEST_GYRO_MASK		0x0c
#define ST_LSM6DS3_SELF_TEST_DISABLED_VAL	0x00
#define ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL	0x01
#define ST_LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define ST_LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define ST_LSM6DS3_LIR_ADDR			0x58
#define ST_LSM6DS3_LIR_MASK			0x01
#define ST_LSM6DS3_TIMER_EN_ADDR		0x58
#define ST_LSM6DS3_TIMER_EN_MASK		0x80
#define ST_LSM6DS3_PEDOMETER_EN_ADDR		0x58
#define ST_LSM6DS3_PEDOMETER_EN_MASK		0x40
#define ST_LSM6DS3_INT2_ON_INT1_ADDR		0x13
#define ST_LSM6DS3_INT2_ON_INT1_MASK		0x20
#define ST_LSM6DS3_MIN_DURATION_MS		1638
#define ST_LSM6DS3_ROUNDING_ADDR		0x16
#define ST_LSM6DS3_ROUNDING_MASK		0x04
#define ST_LSM6DS3_FIFO_MODE_ADDR		0x0a
#define ST_LSM6DS3_FIFO_MODE_MASK		0x07
#define ST_LSM6DS3_FIFO_MODE_BYPASS		0x00
#define ST_LSM6DS3_FIFO_MODE_CONTINUOS		0x06
#define ST_LSM6DS3_FIFO_THRESHOLD_IRQ_MASK	0x08
#define ST_LSM6DS3_FIFO_ODR_ADDR		0x0a
#define ST_LSM6DS3_FIFO_ODR_MASK		0x78
#define ST_LSM6DS3_FIFO_ODR_MAX			0x08
#define ST_LSM6DS3_FIFO_ODR_OFF			0x00
#define ST_LSM6DS3_FIFO_DECIMATOR_ADDR		0x08
#define ST_LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define ST_LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define ST_LSM6DS3_FIFO_DECIMATOR2_ADDR		0x09
#define ST_LSM6DS3_FIFO_DS3_DECIMATOR_MASK	0x07
#define ST_LSM6DS3_FIFO_DS4_DECIMATOR_MASK	0x38
#define ST_LSM6DS3_FIFO_THR_L_ADDR		0x06
#define ST_LSM6DS3_FIFO_THR_H_ADDR		0x07
#define ST_LSM6DS3_FIFO_THR_H_MASK		0x0f
#define ST_LSM6DS3_FIFO_THR_IRQ_MASK		0x08
#define ST_LSM6DS3_RESET_ADDR			0x12
#define ST_LSM6DS3_RESET_MASK			0x01
#define ST_LSM6DS3_MAX_FIFO_SIZE		8192
#define ST_LSM6DS3_FIFO_LENGHT_4096		128
#define ST_LSM6DS3_MAX_FIFO_LENGHT		(ST_LSM6DS3_MAX_FIFO_SIZE / \
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE)

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define ST_LSM6DS3_ACCEL_ODR_ADDR		0x10
#define ST_LSM6DS3_ACCEL_ODR_MASK		0xf0
#define ST_LSM6DS3_ACCEL_FS_ADDR		0x10
#define ST_LSM6DS3_ACCEL_FS_MASK		0x0c
#define ST_LSM6DS3_ACCEL_FS_2G_VAL		0x00
#define ST_LSM6DS3_ACCEL_FS_4G_VAL		0x02
#define ST_LSM6DS3_ACCEL_FS_8G_VAL		0x03
#define ST_LSM6DS3_ACCEL_FS_16G_VAL		0x01
#define ST_LSM6DS3_ACCEL_FS_2G_GAIN		IIO_G_TO_M_S_2(61)  /* scale: 0.000598 */
#define ST_LSM6DS3_ACCEL_FS_4G_GAIN		IIO_G_TO_M_S_2(122)
#define ST_LSM6DS3_ACCEL_FS_8G_GAIN		IIO_G_TO_M_S_2(244)
#define ST_LSM6DS3_ACCEL_FS_16G_GAIN		IIO_G_TO_M_S_2(488)
#define ST_LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR		0x2a
#define ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR		0x2c
#define ST_LSM6DS3_ACCEL_AXIS_EN_ADDR		0x18
#define ST_LSM6DS3_ACCEL_STD			3

/* CUSTOM VALUES FOR GYRO SENSOR */
#define ST_LSM6DS3_GYRO_ODR_ADDR		0x11
#define ST_LSM6DS3_GYRO_ODR_MASK		0xf0
#define ST_LSM6DS3_GYRO_FS_ADDR			0x11
#define ST_LSM6DS3_GYRO_FS_MASK			0x0c
#define ST_LSM6DS3_GYRO_FS_245_VAL		0x00
#define ST_LSM6DS3_GYRO_FS_500_VAL		0x01
#define ST_LSM6DS3_GYRO_FS_1000_VAL		0x02
#define ST_LSM6DS3_GYRO_FS_2000_VAL		0x03
#define ST_LSM6DS3_GYRO_FS_245_GAIN		IIO_DEGREE_TO_RAD(4375)
#define ST_LSM6DS3_GYRO_FS_500_GAIN		IIO_DEGREE_TO_RAD(8750)
#define ST_LSM6DS3_GYRO_FS_1000_GAIN		IIO_DEGREE_TO_RAD(17500)
#define ST_LSM6DS3_GYRO_FS_2000_GAIN		IIO_DEGREE_TO_RAD(70000)
#define ST_LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
#define ST_LSM6DS3_GYRO_OUT_Y_L_ADDR		0x24
#define ST_LSM6DS3_GYRO_OUT_Z_L_ADDR		0x26
#define ST_LSM6DS3_GYRO_AXIS_EN_ADDR		0x19
#define ST_LSM6DS3_GYRO_STD			6

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define ST_LSM6DS3_SIGN_MOTION_EN_ADDR		0x19
#define ST_LSM6DS3_SIGN_MOTION_EN_MASK		0x01

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define ST_LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define ST_LSM6DS3_STEP_COUNTER_OUT_L_ADDR	0x4b
#define ST_LSM6DS3_STEP_COUNTER_RES_ADDR	0x19
#define ST_LSM6DS3_STEP_COUNTER_RES_MASK	0x06
#define ST_LSM6DS3_STEP_COUNTER_RES_ALL_EN	0x03
#define ST_LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define ST_LSM6DS3_TILT_EN_ADDR			0x58
#define ST_LSM6DS3_TILT_EN_MASK			0x20
#define ST_LSM6DS3_TILT_DRDY_IRQ_MASK		0x02

#define ST_LSM6DS3_ACCEL_SUFFIX_NAME		"accel"
#define ST_LSM6DS3_GYRO_SUFFIX_NAME		"gyro"
#define ST_LSM6DS3_STEP_COUNTER_SUFFIX_NAME	"step_c"
#define ST_LSM6DS3_STEP_DETECTOR_SUFFIX_NAME	"step_d"
#define ST_LSM6DS3_SIGN_MOTION_SUFFIX_NAME	"sign_motion"
#define ST_LSM6DS3_TILT_SUFFIX_NAME		"tilt"

#define ST_LSM6DS3_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			st_lsm6ds3_sysfs_get_sampling_frequency, \
			st_lsm6ds3_sysfs_set_sampling_frequency)

#define ST_LSM6DS3_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			st_lsm6ds3_sysfs_sampling_frequency_avail)

#define ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			st_lsm6ds3_sysfs_scale_avail, NULL , 0);

static struct st_lsm6ds3_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 accel_mask;
	u8 gyro_mask;
} st_lsm6ds3_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_LSM6DS3_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

struct st_lsm6ds3_odr_reg {
	unsigned int hz;
	u8 value;
};

static struct st_lsm6ds3_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct st_lsm6ds3_odr_reg odr_avl[ST_LSM6DS3_ODR_LIST_NUM];
} st_lsm6ds3_odr_table = {
	.addr[ST_INDIO_DEV_ACCEL] = ST_LSM6DS3_ACCEL_ODR_ADDR,
	.mask[ST_INDIO_DEV_ACCEL] = ST_LSM6DS3_ACCEL_ODR_MASK,
	.addr[ST_INDIO_DEV_GYRO] = ST_LSM6DS3_GYRO_ODR_ADDR,
	.mask[ST_INDIO_DEV_GYRO] = ST_LSM6DS3_GYRO_ODR_MASK,
	.odr_avl[0] = { .hz = 26, .value = ST_LSM6DS3_ODR_26HZ_VAL },
	.odr_avl[1] = { .hz = 52, .value = ST_LSM6DS3_ODR_52HZ_VAL },
	.odr_avl[2] = { .hz = 104, .value = ST_LSM6DS3_ODR_104HZ_VAL },
	.odr_avl[3] = { .hz = 208, .value = ST_LSM6DS3_ODR_208HZ_VAL },
	.odr_avl[4] = { .hz = 416, .value = ST_LSM6DS3_ODR_416HZ_VAL },
};

struct st_lsm6ds3_fs_reg {
	unsigned int gain;
	u8 value;
};

static struct st_lsm6ds3_fs_table {
	u8 addr;
	u8 mask;
	struct st_lsm6ds3_fs_reg fs_avl[ST_LSM6DS3_FS_LIST_NUM];
} st_lsm6ds3_fs_table[ST_INDIO_DEV_NUM] = {
	[ST_INDIO_DEV_ACCEL] = {
		.addr = ST_LSM6DS3_ACCEL_FS_ADDR,
		.mask = ST_LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_16G_VAL },
	},
	[ST_INDIO_DEV_GYRO] = {
		.addr = ST_LSM6DS3_GYRO_FS_ADDR,
		.mask = ST_LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_GYRO_FS_245_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_GYRO_FS_500_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_2000_VAL },
	}
};

// by zhangjiajing, add IIO_CHAN_INFO_CALIBBIAS
static const struct iio_chan_spec st_lsm6ds3_accel_ch[] = {
	ST_LSM6DS3_ACC_CHANNELS(IIO_ACCEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3_ACC_CHANNELS(IIO_ACCEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3_ACC_CHANNELS(IIO_ACCEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3_gyro_ch[] = {
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_Z_L_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3_sign_motion_ch[] = {
	{
		.type = IIO_SIGN_MOTION,
		.channel = 0,
		.modified = 0,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_step_c_ch[] = {
	{
		.type = IIO_STEP_COUNTER,
		.modified = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.address = ST_LSM6DS3_STEP_COUNTER_OUT_L_ADDR,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_step_d_ch[] = {
	{
		.type = IIO_STEP_DETECTOR,
		.channel = 0,
		.modified = 0,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_tilt_ch[] = {
	{
		.type = IIO_TILT,
		.channel = 0,
		.modified = 0,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

int st_lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}
EXPORT_SYMBOL(st_lsm6ds3_write_data_with_mask);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
static int st_lsm6ds3_enable_sensor_hub(struct lsm6ds3_data *cdata, bool enable)
{
	int err;

	if (enable) {
		if (cdata->sensors_enabled & ST_LSM6DS3_EXT_SENSORS) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_STARTCONFIG_ADDR,
						ST_LSM6DS3_STARTCONFIG_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DS3_EXTRA_DEPENDENCY) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DS3_EXT_SENSORS) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_SENSORHUB_ADDR,
						ST_LSM6DS3_SENSORHUB_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		}
	} else {
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_STARTCONFIG_ADDR,
						ST_LSM6DS3_STARTCONFIG_MASK,
						ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		usleep_range(1500, 4000);

		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_SENSORHUB_ADDR,
						ST_LSM6DS3_SENSORHUB_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;
	}

	return 0;
}

int st_lsm6ds3_enable_passthrough(struct lsm6ds3_data *cdata, bool enable)
{
	int err;
	u8 reg_value;

	if (enable)
		reg_value = ST_LSM6DS3_EN_BIT;
	else
		reg_value = ST_LSM6DS3_DIS_BIT;

	if (enable) {
		err = st_lsm6ds3_enable_sensor_hub(cdata, false);
		if (err < 0)
			return err;

#ifdef CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_INTER_PULLUP_ADDR,
						ST_LSM6DS3_INTER_PULLUP_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP */
	}

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_PASS_THROUGH_MODE_ADDR,
					ST_LSM6DS3_PASS_THROUGH_MODE_MASK,
					reg_value, enable);
	if (err < 0)
		return err;

	if (!enable) {
#ifdef CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_INTER_PULLUP_ADDR,
						ST_LSM6DS3_INTER_PULLUP_MASK,
						ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP */

		err = st_lsm6ds3_enable_sensor_hub(cdata, true);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_enable_passthrough);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

static int st_lsm6ds3_set_fifo_enable(struct lsm6ds3_data *cdata, bool status)
{
	int err;
	u8 reg_value;
	struct timespec ts;

	if (status)
		reg_value = ST_LSM6DS3_FIFO_ODR_MAX;
	else
		reg_value = ST_LSM6DS3_FIFO_ODR_OFF;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_ODR_ADDR,
					ST_LSM6DS3_FIFO_ODR_MASK,
					reg_value, true);
	if (err < 0)
		return err;

	get_monotonic_boottime(&ts);
	cdata->gyro_timestamp = timespec_to_ns(&ts);
	cdata->accel_timestamp = cdata->gyro_timestamp;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	cdata->ext0_timestamp = cdata->gyro_timestamp;
	cdata->ext1_timestamp = cdata->gyro_timestamp;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	return 0;
}

int st_lsm6ds3_set_fifo_mode(struct lsm6ds3_data *cdata, enum fifo_mode fm)
{
	int err;
	u8 reg_value;
	bool enable_fifo;

	switch (fm) {
	case BYPASS:
		reg_value = ST_LSM6DS3_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = ST_LSM6DS3_FIFO_MODE_CONTINUOS;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_fifo_enable(cdata, enable_fifo);
	if (err < 0)
		return err;

	return st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_FIFO_MODE_ADDR,
				ST_LSM6DS3_FIFO_MODE_MASK, reg_value, true);
}
EXPORT_SYMBOL(st_lsm6ds3_set_fifo_mode);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
static int st_lsm6ds3_force_accel_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr)
{
	int i;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == ST_LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
			st_lsm6ds3_odr_table.addr[sdata->sindex],
			st_lsm6ds3_odr_table.mask[sdata->sindex],
			st_lsm6ds3_odr_table.odr_avl[i].value, true);
}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

int st_lsm6ds3_set_fifo_decimators_and_threshold(struct lsm6ds3_data *cdata)
{
	int err;
	struct iio_dev *indio_dev;
	u16 min_num_pattern, max_num_pattern;
	unsigned int min_odr = 416, max_odr = 0;
	u8 accel_decimator = 0, gyro_decimator = 0;
	u16 num_pattern_accel = 0, num_pattern_gyro = 0;
	struct lsm6ds3_sensor_data *sdata_accel, *sdata_gyro;
	u16 fifo_len, fifo_threshold, fifo_len_accel = 0, fifo_len_gyro = 0;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	int i;
	bool force_accel_odr = false;
	u8 ext0_decimator = 0, ext1_decimator = 0;
	u16 num_pattern_ext1 = 0, fifo_len_ext1 = 0;
	u16 num_pattern_ext0 = 0, fifo_len_ext0 = 0;
	struct lsm6ds3_sensor_data *sdata_ext0 = NULL, *sdata_ext1 = NULL;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	indio_dev = cdata->indio_dev[ST_INDIO_DEV_ACCEL];
	sdata_accel = iio_priv(indio_dev);
	if ((1 << sdata_accel->sindex) & cdata->sensors_enabled) {
		if (min_odr > sdata_accel->c_odr)
			min_odr = sdata_accel->c_odr;

		if (max_odr < sdata_accel->c_odr)
			max_odr = sdata_accel->c_odr;

		fifo_len_accel = (indio_dev->buffer->length / 2);
	}

	indio_dev = cdata->indio_dev[ST_INDIO_DEV_GYRO];
	sdata_gyro = iio_priv(indio_dev);
	if ((1 << sdata_gyro->sindex) & cdata->sensors_enabled) {
		if (min_odr > sdata_gyro->c_odr)
			min_odr = sdata_gyro->c_odr;

		if (max_odr < sdata_gyro->c_odr)
			max_odr = sdata_gyro->c_odr;

		fifo_len_gyro = (indio_dev->buffer->length / 2);
	}

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if (cdata->ext0_available) {
		indio_dev = cdata->indio_dev[ST_INDIO_DEV_EXT0];
		sdata_ext0 = iio_priv(indio_dev);
		if ((1 << sdata_ext0->sindex) & cdata->sensors_enabled) {
			if (min_odr > sdata_ext0->c_odr)
				min_odr = sdata_ext0->c_odr;

			if (max_odr < sdata_ext0->c_odr) {
				force_accel_odr = true;
				max_odr = sdata_ext0->c_odr;
			}

			fifo_len_ext0 = (indio_dev->buffer->length / 2);
		}
	}

	if (cdata->ext1_available) {
		indio_dev = cdata->indio_dev[ST_INDIO_DEV_EXT1];
		sdata_ext1 = iio_priv(indio_dev);
		if ((1 << sdata_ext1->sindex) & cdata->sensors_enabled) {
			if (min_odr > sdata_ext1->c_odr)
				min_odr = sdata_ext1->c_odr;

			if (max_odr < sdata_ext1->c_odr) {
				force_accel_odr = true;
				max_odr = sdata_ext1->c_odr;
			}

			fifo_len_ext1 = (indio_dev->buffer->length / 2);
		}
	}

	if (force_accel_odr) {
		err = st_lsm6ds3_force_accel_odr(sdata_accel, max_odr);
		if (err < 0)
			return err;
	} else {
		for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3_odr_table.odr_avl[i].hz ==
							sdata_accel->c_odr)
				break;
		}
		if (i == ST_LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if (cdata->sensors_enabled & (1 << sdata_accel->sindex)) {
			cdata->accel_samples_to_discard = ST_LSM6DS3_ACCEL_STD;

			err = st_lsm6ds3_write_data_with_mask(cdata,
				st_lsm6ds3_odr_table.addr[sdata_accel->sindex],
				st_lsm6ds3_odr_table.mask[sdata_accel->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
			if (err < 0)
				return err;
		}
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	if ((1 << sdata_accel->sindex) & cdata->sensors_enabled) {
		cdata->accel_samples_in_pattern =
						(sdata_accel->c_odr / min_odr);
		num_pattern_accel = MAX(fifo_len_accel /
					cdata->accel_samples_in_pattern, 1);
		cdata->accel_deltatime = (1000000000ULL / sdata_accel->c_odr);
		accel_decimator = max_odr / sdata_accel->c_odr;
	} else
		cdata->accel_samples_in_pattern = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR_ADDR,
					ST_LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK,
					accel_decimator, true);
	if (err < 0)
		return err;

	if ((1 << sdata_gyro->sindex) & cdata->sensors_enabled) {
		cdata->gyro_samples_in_pattern = (sdata_gyro->c_odr / min_odr);
		num_pattern_gyro = MAX(fifo_len_gyro /
					cdata->gyro_samples_in_pattern, 1);
		cdata->gyro_deltatime = (1000000000ULL / sdata_gyro->c_odr);
		gyro_decimator = max_odr / sdata_gyro->c_odr;
	} else
		cdata->gyro_samples_in_pattern = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR_ADDR,
					ST_LSM6DS3_FIFO_GYRO_DECIMATOR_MASK,
					gyro_decimator, true);
	if (err < 0)
		return err;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if (cdata->ext0_available) {
		if ((1 << sdata_ext0->sindex) & cdata->sensors_enabled) {
			cdata->ext0_samples_in_pattern =
						(sdata_ext0->c_odr / min_odr);
			num_pattern_ext0 = MAX(fifo_len_ext0 /
					cdata->ext0_samples_in_pattern, 1);
			cdata->ext0_deltatime =
					(1000000000ULL / sdata_ext0->c_odr);
			ext0_decimator = max_odr / sdata_ext0->c_odr;
		} else
			cdata->ext0_samples_in_pattern = 0;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR2_ADDR,
					ST_LSM6DS3_FIFO_DS3_DECIMATOR_MASK,
					ext0_decimator, true);
		if (err < 0)
			return err;
	}

	if (cdata->ext1_available) {
		if ((1 << sdata_ext1->sindex) & cdata->sensors_enabled) {
			cdata->ext1_samples_in_pattern =
						(sdata_ext1->c_odr / min_odr);
			num_pattern_ext1 = MAX(fifo_len_ext1 /
					cdata->ext1_samples_in_pattern, 1);
			cdata->ext1_deltatime =
					(1000000000ULL / sdata_ext1->c_odr);
			ext1_decimator = max_odr / sdata_ext1->c_odr;
		} else
			cdata->ext1_samples_in_pattern = 0;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR2_ADDR,
					ST_LSM6DS3_FIFO_DS4_DECIMATOR_MASK,
					ext1_decimator, true);
		if (err < 0)
			return err;
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	min_num_pattern = MIN_BNZ(MIN_BNZ(MIN_BNZ(num_pattern_gyro,
		num_pattern_accel), num_pattern_ext0), num_pattern_ext1);
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	min_num_pattern = MIN_BNZ(num_pattern_gyro, num_pattern_accel);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if ((cdata->accel_samples_in_pattern +
				cdata->gyro_samples_in_pattern +
					cdata->ext0_samples_in_pattern +
					cdata->ext1_samples_in_pattern) > 0) {
		max_num_pattern = ST_LSM6DS3_MAX_FIFO_SIZE /
					((cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern +
					cdata->ext0_samples_in_pattern +
					cdata->ext1_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE);

		if (min_num_pattern > max_num_pattern)
			min_num_pattern = max_num_pattern;
	}

	fifo_len = (cdata->accel_samples_in_pattern +
			cdata->gyro_samples_in_pattern +
			cdata->ext0_samples_in_pattern +
			cdata->ext1_samples_in_pattern) *
			min_num_pattern * ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	if ((cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) > 0) {
		max_num_pattern = ST_LSM6DS3_MAX_FIFO_SIZE /
					((cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE);

		if (min_num_pattern > max_num_pattern)
			min_num_pattern = max_num_pattern;
	}

	fifo_len = (cdata->accel_samples_in_pattern +
			cdata->gyro_samples_in_pattern) *
			min_num_pattern * ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	if (fifo_len > 0) {
		fifo_threshold = fifo_len / 2;

		err = cdata->tf->write(cdata, ST_LSM6DS3_FIFO_THR_L_ADDR,
					1, (u8 *)&fifo_threshold, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_THR_H_ADDR,
					ST_LSM6DS3_FIFO_THR_H_MASK,
					*(((u8 *)&fifo_threshold) + 1), true);
		if (err < 0)
			return err;

		cdata->fifo_threshold = fifo_len;
	}
#if 0 /* alloc the fifo_data when probe to avoid kfree all the time */
	kfree(cdata->fifo_data);
	cdata->fifo_data = 0;

	if (fifo_len > 0) {
		cdata->fifo_data = kmalloc(cdata->fifo_threshold, GFP_KERNEL);
		if (!cdata->fifo_data)
			return -ENOMEM;
	}
#endif
	return fifo_len;
}
EXPORT_SYMBOL(st_lsm6ds3_set_fifo_decimators_and_threshold);

int st_lsm6ds3_reconfigure_fifo(struct lsm6ds3_data *cdata,
						bool disable_irq_and_flush)
{
	int err, fifo_len;

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		st_lsm6ds3_flush_works();
	}

	mutex_lock(&cdata->fifo_lock);

	st_lsm6ds3_cleanup_fifo(cdata);

	err = st_lsm6ds3_set_fifo_mode(cdata, BYPASS);
	if (err < 0)
		goto reconfigure_fifo_irq_restore;

	fifo_len = st_lsm6ds3_set_fifo_decimators_and_threshold(cdata);
	if (fifo_len < 0) {
		err = fifo_len;
		goto reconfigure_fifo_irq_restore;
	}

	if (fifo_len > 0) {
		err = st_lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
		if (err < 0)
			goto reconfigure_fifo_irq_restore;
	}

reconfigure_fifo_irq_restore:
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3_reconfigure_fifo);

int st_lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = ST_LSM6DS3_EN_BIT;
	else
		value = ST_LSM6DS3_DIS_BIT;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				(1 << ST_INDIO_DEV_GYRO)) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_GYRO:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				(1 << ST_INDIO_DEV_ACCEL)) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_SIGN_MOTION:
		if (sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_STEP_DETECTOR))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		reg_addr = ST_LSM6DS3_INT2_ADDR;
		mask = ST_LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_STEP_DETECTOR:
		if (sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_SIGN_MOTION))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_TILT:
		reg_addr = ST_LSM6DS3_MD1_ADDR;
		mask = ST_LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
		if ((sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_ACCEL)) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_GYRO)) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_EXT1)))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_EXT1:
		if ((sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_ACCEL)) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_GYRO)) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_EXT0)))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	default:
		return -EINVAL;
	}

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
						reg_addr, mask, value, true);
}
EXPORT_SYMBOL(st_lsm6ds3_set_drdy_irq);

int st_lsm6ds3_set_axis_enable(struct lsm6ds3_sensor_data *sdata, u8 value)
{
	u8 reg_addr;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		reg_addr = ST_LSM6DS3_ACCEL_AXIS_EN_ADDR;
		break;
	case ST_INDIO_DEV_GYRO:
		reg_addr = ST_LSM6DS3_GYRO_AXIS_EN_ADDR;
		break;
	default:
		return 0;
	}

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
				reg_addr, ST_LSM6DS3_AXIS_EN_MASK, value, true);
}
EXPORT_SYMBOL(st_lsm6ds3_set_axis_enable);

int st_lsm6ds3_enable_accel_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err, i=0;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_odr_table.odr_avl[i].hz == sdata->c_odr)
			break;
	}
	/* if the c_odr is not set, use the second odr item(104Hz) */
	if (i == ST_LSM6DS3_ODR_LIST_NUM)
		i=2;

	if (!((sdata->cdata->sensors_enabled &
			ST_LSM6DS3_ACCEL_DEPENDENCY) & ~(1 << sdata->sindex))) {
		if (enable) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.mask[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.mask[ST_INDIO_DEV_ACCEL],
				ST_LSM6DS3_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_enable_accel_dependency);

static int st_lsm6ds3_set_extra_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!((sdata->cdata->sensors_enabled &
			ST_LSM6DS3_EXTRA_DEPENDENCY) & ~(1 << sdata->sindex))) {
		if (enable) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	return st_lsm6ds3_enable_accel_dependency(sdata, enable);
}

static int st_lsm6ds3_enable_pedometer(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	u8 value = ST_LSM6DS3_DIS_BIT;

	if ((sdata->cdata->sensors_enabled & ~(1 << sdata->sindex)) &
						ST_LSM6DS3_PEDOMETER_DEPENDENCY)
		return 0;

	if (enable)
		value = ST_LSM6DS3_EN_BIT;

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						value, true);

}

static int st_lsm6ds3_enable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
	case ST_INDIO_DEV_GYRO:
		for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == ST_LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == ST_INDIO_DEV_ACCEL) {
			sdata->cdata->accel_samples_to_discard =
							ST_LSM6DS3_ACCEL_STD;
		}

		sdata->cdata->gyro_samples_to_discard = ST_LSM6DS3_GYRO_STD;

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		sdata->c_odr = st_lsm6ds3_odr_table.odr_avl[i].hz;

		break;
	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_SIGN_MOTION_EN_ADDR,
					ST_LSM6DS3_SIGN_MOTION_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		if ((sdata->cdata->sensors_enabled & ~(1 << sdata->sindex)) &
					ST_LSM6DS3_PEDOMETER_DEPENDENCY) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;

			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_enable_pedometer(sdata, true);
			if (err < 0)
				return err;
		}

		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TIMER_EN_ADDR,
					ST_LSM6DS3_TIMER_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6ds3_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_TILT:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TILT_EN_ADDR,
					ST_LSM6DS3_TILT_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;

	sdata->cdata->sensors_enabled |= (1 << sdata->sindex);

	return 0;
}

static int st_lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXTRA_DEPENDENCY) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[0].value, true);
		} else {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				ST_LSM6DS3_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_GYRO:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				ST_LSM6DS3_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_SIGN_MOTION_EN_ADDR,
					ST_LSM6DS3_SIGN_MOTION_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TIMER_EN_ADDR,
					ST_LSM6DS3_TIMER_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_TILT:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TILT_EN_ADDR,
					ST_LSM6DS3_TILT_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	sdata->cdata->sensors_enabled &= ~(1 << sdata->sindex);

	return 0;
}

int st_lsm6ds3_set_enable(struct lsm6ds3_sensor_data *sdata, bool enable)
{
	if (enable)
		return st_lsm6ds3_enable_sensors(sdata);
	else
		return st_lsm6ds3_disable_sensors(sdata);
}
EXPORT_SYMBOL(st_lsm6ds3_set_enable);

static int st_lsm6ds3_set_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr)
{
	int err, i;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == ST_LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->cdata->sensors_enabled & (1 << sdata->sindex)) {
		disable_irq(sdata->cdata->irq);
		st_lsm6ds3_flush_works();

		if (sdata->sindex == ST_INDIO_DEV_ACCEL)
			sdata->cdata->accel_samples_to_discard =
							ST_LSM6DS3_ACCEL_STD;

		sdata->cdata->gyro_samples_to_discard = ST_LSM6DS3_GYRO_STD;

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);
			return err;
		}

		sdata->c_odr = st_lsm6ds3_odr_table.odr_avl[i].hz;

		st_lsm6ds3_reconfigure_fifo(sdata->cdata, false);
		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = st_lsm6ds3_odr_table.odr_avl[i].hz;

	return 0;
}

static int st_lsm6ds3_set_fs(struct lsm6ds3_sensor_data *sdata,
							unsigned int gain)
{
	int err, i;

	for (i = 0; i < ST_LSM6DS3_FS_LIST_NUM; i++) {
		if (st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}
	if (i == ST_LSM6DS3_FS_LIST_NUM)
		return -EINVAL;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
		st_lsm6ds3_fs_table[sdata->sindex].addr,
		st_lsm6ds3_fs_table[sdata->sindex].mask,
		st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].value, true);
	if (err < 0)
		return err;

	sdata->c_gain[0] = gain;

	return 0;
}

/*lsm6ds3 calibration*/
#define CALIB_DATA_AMOUNT 100
#define CALIB_DATA_DISCARD 300
static int st_lsm6ds3_acc_get_bias(struct lsm6ds3_sensor_data *sdata)
{
	int err = 0;
	int i;

	int16_t raw[3] = {0,0,0};
	int32_t sum[3] = {0,0,0};

	st_lsm6ds3_set_enable(sdata, false);
	st_lsm6ds3_set_odr(sdata, ST_LSM6DS3_ODR_26HZ_VAL);
	st_lsm6ds3_set_fs(sdata, ST_LSM6DS3_ACCEL_FS_2G_GAIN);

	err = st_lsm6ds3_set_enable(sdata, true);
	if (err < 0) {
		return -EBUSY;
	}

	msleep(50);

	/* accel_samples_to_discard */
	for (i =0; i < CALIB_DATA_DISCARD; i++) {
		err = sdata->cdata->tf->read(sdata->cdata, ST_LSM6DS3_ACCEL_OUT_X_L_ADDR,
				ST_LSM6DS3_BYTE_FOR_CHANNEL * 3, (uint8_t *)&raw, true);
		if (err < 0) {
			dev_err(sdata->cdata->dev, "failed to read channel raw data.\n");
			return err;
		}
		msleep(10);
	}

	for (i =0; i < CALIB_DATA_AMOUNT; i++) {
		err = sdata->cdata->tf->read(sdata->cdata, ST_LSM6DS3_ACCEL_OUT_X_L_ADDR,
				ST_LSM6DS3_BYTE_FOR_CHANNEL * 3, (uint8_t *)&raw, true);
		if (err < 0) {
			dev_err(sdata->cdata->dev, "failed to read channel raw data.\n");
			return err;
		}
		msleep(50);

		sum[0] += raw[0];
		sum[1] += raw[1];
		sum[2] += raw[2];
	}

	sdata->c_bias[0]  = -(sum[0] / CALIB_DATA_AMOUNT);
	sdata->c_bias[1]  = -(sum[1] / CALIB_DATA_AMOUNT);

	// when caculating the z bias:
	// (|raw + bias|) * scale = 9.8 m/s^2
	// and scale = 9.8 m/s^2 * FS_2G_GAIN / 1000000 = 0.000598
	// so |raw + bias| = 9.8 / 0.000598 = 16388)
	raw[2]  = sum[2] / CALIB_DATA_AMOUNT;
	/*
	if (raw[2] >= 16388) {
		sdata->c_bias[2] = -(raw[2] - 16388);
	} else if (raw[2] < 16388 && raw[2] >= 0) {
		sdata->c_bias[2] = -(raw[2] - 16388);
	} else if (raw[2] < 0 && raw[2] >= -16388) {
		sdata->c_bias[2] = -(raw[2] + 16388);
	} else { // raw[2] < -16388
		sdata->c_bias[2] = -(raw[2] + 16388);
	}*/

	sdata->c_bias[2]  = -((raw[2] > 0) ? (raw[2] - 16388) : (raw[2] + 16388));

	return err;
}

static int st_lsm6ds3_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	int offset;
	int bias;
	u8 outdata[ST_LSM6DS3_BYTE_FOR_CHANNEL];
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6ds3_set_enable(sdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		if (sdata->sindex == ST_INDIO_DEV_ACCEL)
			msleep(40);

		if (sdata->sindex == ST_INDIO_DEV_GYRO)
			msleep(120);

		err = sdata->cdata->tf->read(sdata->cdata, ch->address,
				ST_LSM6DS3_BYTE_FOR_CHANNEL, outdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		err = st_lsm6ds3_set_enable(sdata, false);

		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->c_gain[0];
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_CALIBBIAS:
		dev_dbg(sdata->cdata->dev, "get channel[%d] calibbiaa.\n", ch->channel);

		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
			mutex_lock(&indio_dev->mlock);

			if (!sdata->c_bias[0] && !sdata->c_bias[1] && !sdata->c_bias[2]) {
				err = st_lsm6ds3_acc_get_bias(sdata);
				if (err < 0) {
					mutex_unlock(&indio_dev->mlock);
					return err;
				}
			}

			mutex_unlock(&indio_dev->mlock);

			switch (ch->address) {
			case ST_LSM6DS3_ACCEL_OUT_X_L_ADDR:
				bias = sdata->c_bias[0];
				break;
			case ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR:
				bias = sdata->c_bias[1];
				break;
			case ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR:
				bias = sdata->c_bias[2];
				break;
			default:
				return -EINVAL;
			}

			*val = bias;
			break;
		default:
			return -EINVAL;
		}

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OFFSET:

		switch (ch->address) {
		case ST_LSM6DS3_ACCEL_OUT_X_L_ADDR:
			offset = sdata->c_bias[0];
			break;
		case ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR:
			offset = sdata->c_bias[1];
			break;
		case ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR:
			offset = sdata->c_bias[2];
			break;
		default:
			return -EINVAL;
		}

		// we dont use the offset to cali data
		//*val = offset;
		*val = 0;
		dev_dbg(sdata->cdata->dev, "get channel[%x] offset: %d.\n",
			(unsigned int)ch->address, *val);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return 0;
}

static int st_lsm6ds3_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6ds3_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);
		break;

	case IIO_CHAN_INFO_CALIBBIAS:
		err = 0;
		switch (chan->address) {
		case ST_LSM6DS3_ACCEL_OUT_X_L_ADDR:
			sdata->c_bias[0] = val;
			break;
		case ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR:
			sdata->c_bias[1] = val;
			break;
		case ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR:
			sdata->c_bias[2] = val;
			break;
		default:
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}

	return err;
}

static int st_lsm6ds3_reset_steps(struct lsm6ds3_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			ST_LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & ST_LSM6DS3_FUNC_EN_MASK)
		reg_value = ST_LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = ST_LSM6DS3_DIS_BIT;

	err = st_lsm6ds3_write_data_with_mask(cdata,
				ST_LSM6DS3_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3_STEP_COUNTER_RES_MASK,
				ST_LSM6DS3_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
				ST_LSM6DS3_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int st_lsm6ds3_init_sensor(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	cdata->sensors_enabled = 0;
	cdata->reset_steps = false;
	cdata->sign_motion_event_ready = false;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_RESET_ADDR,
				ST_LSM6DS3_RESET_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6ds3_set_enable(sdata, false);
		if (err < 0)
			return err;

		err = st_lsm6ds3_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;

		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
		case ST_INDIO_DEV_GYRO:
			sdata->num_data_channels =
					ARRAY_SIZE(st_lsm6ds3_accel_ch) - 1;

			err = st_lsm6ds3_set_fs(sdata, sdata->c_gain[0]);
			if (err < 0)
				return err;

			break;
		case ST_INDIO_DEV_STEP_COUNTER:
			sdata->num_data_channels =
					ARRAY_SIZE(st_lsm6ds3_step_c_ch) - 1;
			break;
		default:
			break;
		}
	}

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_LIR_ADDR,
				ST_LSM6DS3_LIR_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_BDU_ADDR,
				ST_LSM6DS3_BDU_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_set_fifo_enable(sdata->cdata, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_ROUNDING_ADDR,
					ST_LSM6DS3_ROUNDING_MASK,
					ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_INT2_ON_INT1_ADDR,
					ST_LSM6DS3_INT2_ON_INT1_MASK,
					ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1, &default_reg_value, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);

	sdata->c_odr = 0;

	return 0;

st_lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int st_lsm6ds3_set_selftest(struct lsm6ds3_sensor_data *sdata, int index)
{
	int err;
	u8 mode, mask;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		mask = ST_LSM6DS3_SELFTEST_ACCEL_MASK;
		mode = st_lsm6ds3_selftest_table[index].accel_value;
		break;
	case ST_INDIO_DEV_GYRO:
		mask = ST_LSM6DS3_SELFTEST_GYRO_MASK;
		mode = st_lsm6ds3_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				ST_LSM6DS3_SELFTEST_ADDR, mask, mode, true);
	if (err < 0)
		return err;

	return 0;
}

static ssize_t st_lsm6ds3_sysfs_set_max_delivery_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 duration;
	int err, err2;
	unsigned int max_delivery_rate;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->c_odr)
		return size;

	duration = max_delivery_rate / ST_LSM6DS3_MIN_DURATION_MS;

	mutex_lock(&sdata->cdata->bank_registers_lock);

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1, &duration, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank;

	mutex_unlock(&sdata->cdata->bank_registers_lock);

	sdata->c_odr = max_delivery_rate;

	return size;

st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank:
	do {
		err2 = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);

		msleep(500);
	} while (err2 < 0);

st_lsm6ds3_sysfs_set_max_delivery_rate_mutex_unlock:
	mutex_unlock(&sdata->cdata->bank_registers_lock);
	return err;
}

static ssize_t st_lsm6ds3_sysfs_get_max_delivery_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6ds3_sysfs_reset_counter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	err = st_lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6ds3_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6ds3_sysfs_set_sampling_frequency(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&indio_dev->mlock);

	if (sdata->c_odr != odr)
		err = st_lsm6ds3_set_odr(sdata, odr);

	mutex_unlock(&indio_dev->mlock);

	return err < 0 ? err : size;
}

static ssize_t st_lsm6ds3_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
					st_lsm6ds3_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < ST_LSM6DS3_FS_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
			st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3_sysfs_get_selftest_available(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s, %s, %s\n",
				st_lsm6ds3_selftest_table[0].string_mode,
				st_lsm6ds3_selftest_table[1].string_mode,
				st_lsm6ds3_selftest_table[2].string_mode);
}

static ssize_t st_lsm6ds3_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 index;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		index = sdata->cdata->accel_selftest_status;
		break;
	case ST_INDIO_DEV_GYRO:
		index = sdata->cdata->gyro_selftest_status;
		break;
	default:
		return -EINVAL;
	}
#if 0
	return sprintf(buf, "%s\n",
				st_lsm6ds3_selftest_table[index].string_mode);
#else
	return sprintf(buf, "%d\n", 1);
#endif
}

static ssize_t st_lsm6ds3_sysfs_set_selftest_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < ARRAY_SIZE(st_lsm6ds3_selftest_table); i++) {
		if (strncmp(buf, st_lsm6ds3_selftest_table[i].string_mode,
								size - 2) == 0)
			break;
	}
	if (i == ARRAY_SIZE(st_lsm6ds3_selftest_table))
		return -EINVAL;

	err = st_lsm6ds3_set_selftest(sdata, i);
	if (err < 0)
		return err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		sdata->cdata->accel_selftest_status = i;
		break;
	case ST_INDIO_DEV_GYRO:
		sdata->cdata->gyro_selftest_status = i;
		break;
	default:
		return -EINVAL;
	}

	return size;
}

ssize_t st_lsm6ds3_sysfs_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ST_LSM6DS3_FIFO_LENGHT_4096);
}

ssize_t st_lsm6ds3_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	disable_irq(sdata->cdata->irq);
	st_lsm6ds3_flush_works();

	mutex_lock(&sdata->cdata->fifo_lock);

	st_lsm6ds3_read_fifo(sdata->cdata, true);

	mutex_unlock(&sdata->cdata->fifo_lock);

	enable_irq(sdata->cdata->irq);

	return size;
}


static ssize_t st_lsm6ds3_sysfs_get_sensor_phone_calling(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 index;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->sensor_phone_calling);
}

static ssize_t st_lsm6ds3_sysfs_set_sensor_phone_calling(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int flag;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &flag);
	if (err < 0)
		return err;

	mutex_lock(&indio_dev->mlock);

	sdata->sensor_phone_calling = !!flag;

	mutex_unlock(&indio_dev->mlock);

	return err < 0 ? err : size;
}
static ST_LSM6DS3_DEV_ATTR_SAMP_FREQ();
static ST_LSM6DS3_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);
static ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(in_anglvel_scale_available);
static ST_LSM6DS3_FIFO_LENGHT();
static ST_LSM6DS3_FIFO_FLUSH();

static IIO_DEVICE_ATTR(reset_counter, S_IWUSR,
				NULL, st_lsm6ds3_sysfs_reset_counter, 0);

static IIO_DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO,
				st_lsm6ds3_sysfs_get_max_delivery_rate,
				st_lsm6ds3_sysfs_set_max_delivery_rate, 0);

static IIO_DEVICE_ATTR(self_test_available, S_IRUGO,
				st_lsm6ds3_sysfs_get_selftest_available,
				NULL, 0);

static IIO_DEVICE_ATTR(self_test, S_IWUSR | S_IRUGO,
				st_lsm6ds3_sysfs_get_selftest_status,
				st_lsm6ds3_sysfs_set_selftest_status, 0);

static IIO_DEVICE_ATTR(sensor_phone_calling, S_IWUSR | S_IRUGO,
				st_lsm6ds3_sysfs_get_sensor_phone_calling,
				st_lsm6ds3_sysfs_set_sensor_phone_calling, 0);

static struct attribute *st_lsm6ds3_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	&iio_dev_attr_sensor_phone_calling.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_accel_attribute_group = {
	.attrs = st_lsm6ds3_accel_attributes,
};

static const struct iio_info st_lsm6ds3_accel_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_accel_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static struct attribute *st_lsm6ds3_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_gyro_attribute_group = {
	.attrs = st_lsm6ds3_gyro_attributes,
};

static const struct iio_info st_lsm6ds3_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_gyro_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static struct attribute *st_lsm6ds3_sign_motion_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_sign_motion_attribute_group = {
	.attrs = st_lsm6ds3_sign_motion_attributes,
};

static const struct iio_info st_lsm6ds3_sign_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_sign_motion_attribute_group,
};

static struct attribute *st_lsm6ds3_step_c_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_max_delivery_rate.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_step_c_attribute_group = {
	.attrs = st_lsm6ds3_step_c_attributes,
};

static const struct iio_info st_lsm6ds3_step_c_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_step_c_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
};

static struct attribute *st_lsm6ds3_step_d_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_step_d_attribute_group = {
	.attrs = st_lsm6ds3_step_d_attributes,
};

static const struct iio_info st_lsm6ds3_step_d_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_step_d_attribute_group,
};

static struct attribute *st_lsm6ds3_tilt_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_tilt_attribute_group = {
	.attrs = st_lsm6ds3_tilt_attributes,
};

static const struct iio_info st_lsm6ds3_tilt_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_tilt_attribute_group,
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_lsm6ds3_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_LSM6DS3_TRIGGER_SET_STATE,
};
#define ST_LSM6DS3_TRIGGER_OPS (&st_lsm6ds3_trigger_ops)
#else
#define ST_LSM6DS3_TRIGGER_OPS NULL
#endif

static bool iio_buffer_is_active(struct iio_dev *indio_dev,
				 struct iio_buffer *buf)
{
	struct list_head *p;

	list_for_each(p, &indio_dev->buffer_list)
		if (p == &buf->buffer_list)
			return true;

	return false;
}

static int st_lsm6ds3_iio_disable(struct iio_dev *indio_dev)
{
	struct iio_buffer *pbuf;
	bool inlist;

	pbuf = indio_dev->buffer;
	mutex_lock(&indio_dev->mlock);
	inlist = iio_buffer_is_active(indio_dev, pbuf);
	if (inlist)
		iio_update_buffers(indio_dev, NULL, indio_dev->buffer);
	mutex_unlock(&indio_dev->mlock);
	return 0;
}

static int st_lsm6ds3_iio_enable(struct iio_dev *indio_dev)
{
	struct iio_buffer *pbuf;
	bool inlist;

	pbuf = indio_dev->buffer;
	mutex_lock(&indio_dev->mlock);
	inlist = iio_buffer_is_active(indio_dev, pbuf);
	if (!inlist)
		iio_update_buffers(indio_dev, indio_dev->buffer, NULL);
	mutex_unlock(&indio_dev->mlock);
	return 0;
}

static int mz_sensors_lsm6ds3_acc_self_test(struct device *dev)
{
	int ret;
	int self_test_result;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *acc = iio_priv(indio_dev);
	struct lsm6ds3_sensor_data *gyr =
		iio_priv(acc->cdata->indio_dev[ST_INDIO_DEV_GYRO]);

	struct lsm6ds3_data *cdata = acc->cdata;
	int acc_enable = (1 << acc->sindex) & acc->cdata->sensors_enabled;
	int gyr_enable = (1 << gyr->sindex) & gyr->cdata->sensors_enabled;

	st_lsm6ds3_iio_disable(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	st_lsm6ds3_iio_disable(cdata->indio_dev[ST_INDIO_DEV_GYRO]);

	ret = lsm6ds3_acc_selftest(acc, &self_test_result);
	if (ret < 0) {
		dev_err(dev, "lsm6ds3_acc_selftest failed\n");
		return ret;
	}

	if (acc_enable) {
		st_lsm6ds3_iio_enable(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	}

	if (gyr_enable) {
		st_lsm6ds3_iio_enable(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
	}

	dev_info(dev, "lsm6ds3_acc_selftest completed\n");
	return self_test_result;
}

static int mz_sensors_lsm6ds3_acc_calibrate(struct device *dev)
{
	int err;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	err = st_lsm6ds3_acc_get_bias(sdata);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		dev_err(dev, "lsm6ds3_acc_calibrate failed\n");
		return err;
	}

	mutex_unlock(&indio_dev->mlock);
	dev_info(dev, "lsm6ds3_acc_calibrate succeeded\n");
	return 0;
}

static int mz_sensors_lsm6ds3_get_calibbias(struct device *dev, int32_t calibbias[3])
{
	int err;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	calibbias[0] = sdata->c_bias[0];
	calibbias[1] = sdata->c_bias[1];
	calibbias[2] = sdata->c_bias[2];

	dev_info(dev, "lsm6ds3_acc_get_calibbias succeeded\n");
	return 0;
}

static int mz_sensors_lsm6ds3_get_offset(struct device *dev, int32_t offset[3])
{
	int err;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	offset[0] = sdata->c_offset[0];
	offset[1] = sdata->c_offset[1];
	offset[2] = sdata->c_offset[2];

	dev_info(dev, "lsm6ds3_acc_get_offset succeeded\n");
	return 0;
}

static int mz_sensors_lsm6ds3_set_offset(struct device *dev, int32_t offset, int axis)
{
	int err;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	/* bugfix: fac calibrate error [http://redmine.meizu.com/issues/173055] */
	if ( (axis == 2) && (offset < -16000) )
		offset = 0;

	sdata->c_offset[axis] = offset;

	dev_info(dev, "lsm6ds3_acc_set_offset(%d,%d) succeeded\n", axis, offset);
	return 0;
}

static int mz_sensors_lsm6ds3_acc_get_name(struct device *dev, char **name)
{
	*name = "lsm6ds3-acc";

	dev_info(dev, "lsm6ds3_acc_get_name succeeded\n");
	return 0;
}

static int mz_sensors_lsm6ds3_gyr_get_name(struct device *dev, char **name)
{
	*name = "lsm6ds3-gyr";

	dev_info(dev, "lsm6ds3_gyr_get_name succeeded\n");
	return 0;
}


static int mz_sensors_lsm6ds3_gyr_self_test(struct device *dev)
{
	int ret;
	int self_test_result;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *gyr = iio_priv(indio_dev);
	struct lsm6ds3_sensor_data *acc =
		iio_priv(gyr->cdata->indio_dev[ST_INDIO_DEV_ACCEL]);

	struct lsm6ds3_data *cdata = acc->cdata;
	int acc_enable = (1 << acc->sindex) & acc->cdata->sensors_enabled;
	int gyr_enable = (1 << gyr->sindex) & gyr->cdata->sensors_enabled;

	st_lsm6ds3_iio_disable(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	st_lsm6ds3_iio_disable(cdata->indio_dev[ST_INDIO_DEV_GYRO]);

	ret = lsm6ds3_gyr_selftest(gyr, &self_test_result);
	if (ret < 0) {
		dev_err(dev, "lsm6ds3_gyr_selftest failed\n");
		return ret;
	}

	if (acc_enable) {
		st_lsm6ds3_iio_enable(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	}

	if (gyr_enable) {
		st_lsm6ds3_iio_enable(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
	}

	dev_info(dev, "lsm6ds3_gyr_selftest completed\n");
	return self_test_result;
}

struct meizu_sensors_ops meizu_lsm6ds3_acc_ops = {
	.self_test = &mz_sensors_lsm6ds3_acc_self_test,
	.calibrate = &mz_sensors_lsm6ds3_acc_calibrate,
	.get_calibbias = &mz_sensors_lsm6ds3_get_calibbias,
	.get_offset = &mz_sensors_lsm6ds3_get_offset,
	.set_offset = &mz_sensors_lsm6ds3_set_offset,
	.get_name = &mz_sensors_lsm6ds3_acc_get_name,
};

struct meizu_sensors_ops meizu_lsm6ds3_gyr_ops = {
	.get_name = &mz_sensors_lsm6ds3_gyr_get_name,
	.self_test = &mz_sensors_lsm6ds3_gyr_self_test,
};

static void lsm6ds3_early_suspend(struct early_suspend *h)
{
	struct lsm6ds3_data *cdata = container_of(h, struct lsm6ds3_data,
			early_suspend);

	st_lsm6ds3_common_suspend(cdata);

}

static void lsm6ds3_late_resume(struct early_suspend *h)
{
	struct lsm6ds3_data *cdata = container_of(h, struct lsm6ds3_data,
			early_suspend);

	st_lsm6ds3_common_resume(cdata);
}

int st_lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq)
{
	u8 wai = 0x00;
	int i, n, err;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	mutex_init(&cdata->passthrough_lock);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	cdata->fifo_data = kmalloc(ST_LSM6DS3_MAX_FIFO_SIZE, GFP_KERNEL);
	if (!cdata->fifo_data)
		return -ENOMEM;

	err = cdata->tf->read(cdata, ST_LSM6DS3_WAI_ADDRESS, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != ST_LSM6DS3_WAI_EXP) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->indio_dev[i] = iio_device_alloc(sizeof(*sdata));
		if (cdata->indio_dev[i] == NULL) {
			err = -ENOMEM;
			goto iio_device_free;
		}
		sdata = iio_priv(cdata->indio_dev[i]);
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->buffer_data = kmalloc(32, GFP_KERNEL);
		if (sdata->buffer_data == NULL)
			return -ENOMEM;

		if ((i == ST_INDIO_DEV_ACCEL) || (i == ST_INDIO_DEV_GYRO)) {
			sdata->c_odr = st_lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain[0] =
					st_lsm6ds3_fs_table[i].fs_avl[0].gain;
		}
		cdata->indio_dev[i]->modes = INDIO_DIRECT_MODE;
	}

	if (irq > 0) {
		cdata->irq = irq;
		dev_info(cdata->dev, "driver use DRDY int pin 1\n");
	}

	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_ACCEL_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->info = &st_lsm6ds3_accel_info;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->channels = st_lsm6ds3_accel_ch;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_accel_ch);

	cdata->indio_dev[ST_INDIO_DEV_GYRO]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_GYRO_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->info = &st_lsm6ds3_gyro_info;
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->channels = st_lsm6ds3_gyro_ch;
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_gyro_ch);

	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_SIGN_MOTION_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->info =
						&st_lsm6ds3_sign_motion_info;
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->channels =
						st_lsm6ds3_sign_motion_ch;
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_sign_motion_ch);

	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_STEP_COUNTER_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->info =
						&st_lsm6ds3_step_c_info;
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->channels =
						st_lsm6ds3_step_c_ch;
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_step_c_ch);

	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_STEP_DETECTOR_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->info =
						&st_lsm6ds3_step_d_info;
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->channels =
						st_lsm6ds3_step_d_ch;
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_step_d_ch);

	cdata->indio_dev[ST_INDIO_DEV_TILT]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_TILT_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_TILT]->info = &st_lsm6ds3_tilt_info;
	cdata->indio_dev[ST_INDIO_DEV_TILT]->channels = st_lsm6ds3_tilt_ch;
	cdata->indio_dev[ST_INDIO_DEV_TILT]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_tilt_ch);

	err = st_lsm6ds3_init_sensor(cdata);
	if (err < 0)
		goto iio_device_free;

	err = st_lsm6ds3_allocate_rings(cdata);
	if (err < 0)
		goto iio_device_free;

	if (irq > 0) {
		err = st_lsm6ds3_allocate_triggers(cdata,
							ST_LSM6DS3_TRIGGER_OPS);
		if (err < 0)
			goto deallocate_ring;
	}

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_device_register(cdata->indio_dev[n]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}

	err = st_lsm6ds3_i2c_master_probe(cdata);
	if (err < 0)
		goto iio_device_unregister_and_trigger_deallocate;

	device_init_wakeup(cdata->dev, true);
#if 0
	err = meizu_sysfslink_register_name(&cdata->indio_dev[ST_INDIO_DEV_ACCEL]->dev, "accelerometer");
	if (err < 0) {
		dev_err(cdata->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_device_unregister_and_trigger_deallocate;
	}

	err = meizu_sysfslink_register_name(&cdata->indio_dev[ST_INDIO_DEV_GYRO]->dev, "gyroscope");
	if (err < 0) {
		dev_err(cdata->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_device_unregister_and_trigger_deallocate;
	}
#endif
#if 1
	err = meizu_sensor_register(MEIZU_SENSOR_ID_ACC,
		&cdata->indio_dev[ST_INDIO_DEV_ACCEL]->dev, &meizu_lsm6ds3_acc_ops);
	if (err < 0) {
		dev_err(cdata->dev, "failed to meizu_sensor_register.\n");
		goto iio_device_unregister_and_trigger_deallocate;
	}

	err = meizu_sensor_register(MEIZU_SENSOR_ID_GYR,
		&cdata->indio_dev[ST_INDIO_DEV_GYRO]->dev, &meizu_lsm6ds3_gyr_ops);
	if (err < 0) {
		dev_err(cdata->dev, "failed to meizu_sensor_register.\n");
		goto iio_device_unregister_and_trigger_deallocate;
	}
#endif

	cdata->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	cdata->early_suspend.suspend = lsm6ds3_early_suspend;
	cdata->early_suspend.resume = lsm6ds3_late_resume;
	register_early_suspend(&cdata->early_suspend);

	return 0;

iio_device_unregister_and_trigger_deallocate:
	for (n--; n >= 0; n--)
		iio_device_unregister(cdata->indio_dev[n]);

	if (irq > 0)
		st_lsm6ds3_deallocate_triggers(cdata);
deallocate_ring:
	st_lsm6ds3_deallocate_rings(cdata);
iio_device_free:
	for (i--; i >= 0; i--)
		iio_device_free(cdata->indio_dev[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3_common_probe);

void st_lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq)
{
	int i;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_unregister(cdata->indio_dev[i]);

	if (irq > 0)
		st_lsm6ds3_deallocate_triggers(cdata);

	st_lsm6ds3_deallocate_rings(cdata);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_free(cdata->indio_dev[i]);

	st_lsm6ds3_i2c_master_exit(cdata);
}
EXPORT_SYMBOL(st_lsm6ds3_common_remove);

#ifdef CONFIG_PM
int st_lsm6ds3_common_suspend(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 tmp_sensors_enabled;
	struct lsm6ds3_sensor_data *sdata;

	tmp_sensors_enabled = cdata->sensors_enabled;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		if ((1 << i) & ST_LSM6DS3_WAKE_UP_SENSORS)
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6ds3_set_enable(sdata, false);
		if (err < 0)
			return err;
	}
	cdata->sensors_enabled = tmp_sensors_enabled;

	sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	if (sdata->sensor_phone_calling && (cdata->sensors_enabled & (1 << sdata->sindex))) {
		/* enable acc when sensor phone calling is set by flyme */
		err = st_lsm6ds3_set_enable(sdata, true);
		if (err < 0)
			return err;
	}

	if ((cdata->sensors_enabled & ST_LSM6DS3_WAKE_UP_SENSORS) || sdata->sensor_phone_calling) {
		mt_eint_unmask(CUST_EINT_GYRO_NUM);
		if (device_may_wakeup(cdata->dev))
			enable_irq_wake(cdata->irq);
	} else {
		mt_eint_mask(CUST_EINT_GYRO_NUM);
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_common_suspend);

int st_lsm6ds3_common_resume(struct lsm6ds3_data *cdata)
{
	int err, i;
	struct lsm6ds3_sensor_data *sdata;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		if ((1 << i) & ST_LSM6DS3_WAKE_UP_SENSORS)
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		if ((1 << sdata->sindex) & cdata->sensors_enabled) {
			err = st_lsm6ds3_set_enable(sdata, true);
			if (err < 0)
				return err;
		}
	}

	if (cdata->sensors_enabled & ST_LSM6DS3_WAKE_UP_SENSORS) {
		if (device_may_wakeup(cdata->dev))
			disable_irq_wake(cdata->irq);
	}
	mt_eint_unmask(CUST_EINT_GYRO_NUM);
	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_common_resume);
#endif /* CONFIG_PM */

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 core driver");
MODULE_LICENSE("GPL v2");
