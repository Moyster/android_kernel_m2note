/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 * v. 2.1.2
 * Licensed under the GPL-2.
 */

#ifndef ST_LSM6DS3_H
#define ST_LSM6DS3_H

#include <linux/types.h>
#include <linux/iio/trigger.h>
#include <linux/earlysuspend.h>
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
#include <linux/i2c.h>
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#define LSM6DS3_DEV_NAME			"lsm6ds3"

//#define LSM6DS3_DEBUG 1
#ifdef LSM6DS3_DEBUG
#define LOG_TAG_LSM6DS3 "[lsm6ds3]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG LOG_TAG_LSM6DS3 format , ## arg)
#endif

#define ST_INDIO_DEV_ACCEL			0
#define ST_INDIO_DEV_GYRO			1
#define ST_INDIO_DEV_SIGN_MOTION		2
#define ST_INDIO_DEV_STEP_COUNTER		3
#define ST_INDIO_DEV_STEP_DETECTOR		4
#define ST_INDIO_DEV_TILT			5
#define ST_INDIO_DEV_NUM			6

#define ST_INDIO_DEV_EXT0			ST_INDIO_DEV_NUM
#define ST_INDIO_DEV_EXT1			(ST_INDIO_DEV_NUM + 1)

#define ST_LSM6DS3_ACCEL_DEPENDENCY	((1 << ST_INDIO_DEV_ACCEL) | \
					(1 << ST_INDIO_DEV_STEP_COUNTER) | \
					(1 << ST_INDIO_DEV_TILT) | \
					(1 << ST_INDIO_DEV_SIGN_MOTION) | \
					(1 << ST_INDIO_DEV_STEP_DETECTOR) | \
					(1 << ST_INDIO_DEV_EXT0) | \
					(1 << ST_INDIO_DEV_EXT1))

#define ST_LSM6DS3_PEDOMETER_DEPENDENCY ((1 << ST_INDIO_DEV_STEP_COUNTER) | \
					(1 << ST_INDIO_DEV_STEP_DETECTOR) | \
					(1 << ST_INDIO_DEV_SIGN_MOTION))

#define ST_LSM6DS3_EXTRA_DEPENDENCY	((1 << ST_INDIO_DEV_STEP_COUNTER) | \
					(1 << ST_INDIO_DEV_TILT) | \
					(1 << ST_INDIO_DEV_SIGN_MOTION) | \
					(1 << ST_INDIO_DEV_STEP_DETECTOR) | \
					(1 << ST_INDIO_DEV_EXT0) | \
					(1 << ST_INDIO_DEV_EXT1))

#define ST_LSM6DS3_USE_BUFFER		((1 << ST_INDIO_DEV_ACCEL) | \
					(1 << ST_INDIO_DEV_GYRO) | \
					(1 << ST_INDIO_DEV_STEP_COUNTER))

#define ST_LSM6DS3_EXT_SENSORS		((1 << ST_INDIO_DEV_EXT0) | \
					(1 << ST_INDIO_DEV_EXT1))

#ifndef CONFIG_ST_LSM6DS3_IIO_SENSORS_WAKEUP
#define ST_LSM6DS3_WAKE_UP_SENSORS	((1 << ST_INDIO_DEV_SIGN_MOTION) | \
					(1 << ST_INDIO_DEV_TILT))
#else /* CONFIG_ST_LSM6DS3_IIO_SENSORS_WAKEUP */
#define ST_LSM6DS3_WAKE_UP_SENSORS	((1 << ST_INDIO_DEV_SIGN_MOTION) | \
					(1 << ST_INDIO_DEV_TILT) | \
					(1 << ST_INDIO_DEV_ACCEL) | \
					(1 << ST_INDIO_DEV_GYRO) | \
					(1 << ST_INDIO_DEV_STEP_COUNTER) | \
					(1 << ST_INDIO_DEV_STEP_DETECTOR) | \
					(1 << ST_INDIO_DEV_EXT0) | \
					(1 << ST_INDIO_DEV_EXT1))
#endif /* CONFIG_ST_LSM6DS3_IIO_SENSORS_WAKEUP */

#define ST_LSM6DS3_TX_MAX_LENGTH		12
#define ST_LSM6DS3_RX_MAX_LENGTH		8193

#define ST_LSM6DS3_BYTE_FOR_CHANNEL		2
#define ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE	6

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
#define ST_LSM6DS3_NUM_CLIENTS			2
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
#define ST_LSM6DS3_NUM_CLIENTS			0
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#define ST_LSM6DS3_LSM_CHANNELS(device_type, modif, index, mod, \
						endian, sbits, rbits, addr, s) \
{ \
	.type = device_type, \
	.modified = modif, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = s, \
		.realbits = rbits, \
		.shift = sbits - rbits, \
		.storagebits = sbits, \
		.endianness = endian, \
	}, \
}

// by zhangjiajing, add IIO_CHAN_INFO_CALIBBIAS
#define ST_LSM6DS3_ACC_CHANNELS(device_type, modif, index, mod, \
						endian, sbits, rbits, addr, s) \
{ \
	.type = device_type, \
	.modified = modif, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE) |\
			BIT(IIO_CHAN_INFO_OFFSET) |\
			BIT(IIO_CHAN_INFO_CALIBBIAS), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = s, \
		.realbits = rbits, \
		.shift = sbits - rbits, \
		.storagebits = sbits, \
		.endianness = endian, \
	}, \
}

#define ST_LSM6DS3_FIFO_LENGHT() \
	IIO_DEVICE_ATTR(hw_fifo_lenght, S_IRUGO, \
				st_lsm6ds3_sysfs_get_hw_fifo_lenght, NULL, 0);

#define ST_LSM6DS3_FIFO_FLUSH() \
	IIO_DEVICE_ATTR(flush, S_IWUSR, NULL, st_lsm6ds3_sysfs_flush_fifo, 0);

enum fifo_mode {
	BYPASS = 0,
	CONTINUOS,
};

struct st_lsm6ds3_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ST_LSM6DS3_RX_MAX_LENGTH];
	u8 tx_buf[ST_LSM6DS3_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lsm6ds3_data {
	const char *name;

	bool reset_steps;
	bool sign_motion_event_ready;

	u8 *fifo_data;
	u8 sensors_enabled;
	u8 gyro_selftest_status;
	u8 accel_selftest_status;
	u8 accel_samples_in_pattern;
	u8 gyro_samples_in_pattern;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	u8 ext0_samples_in_pattern;
	u8 ext1_samples_in_pattern;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	u8 accel_samples_to_discard;
	u8 gyro_samples_to_discard;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	u8 ext_samples_to_discard[ST_LSM6DS3_NUM_CLIENTS];
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	u16 fifo_threshold;

	int irq;

	s64 timestamp;
	int64_t accel_deltatime;
	int64_t accel_timestamp;
	int64_t gyro_deltatime;
	int64_t gyro_timestamp;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	int64_t ext0_deltatime;
	int64_t ext0_timestamp;
	int64_t ext1_deltatime;
	int64_t ext1_timestamp;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	struct work_struct data_work;

	struct device *dev;
	struct iio_dev *indio_dev[ST_INDIO_DEV_NUM + ST_LSM6DS3_NUM_CLIENTS];
	struct iio_trigger *trig[ST_INDIO_DEV_NUM + ST_LSM6DS3_NUM_CLIENTS];
	struct mutex bank_registers_lock;
	struct mutex fifo_lock;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	struct i2c_client *master_client[ST_LSM6DS3_NUM_CLIENTS];
	struct mutex passthrough_lock;
	bool ext0_available;
	bool ext1_available;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	const struct st_lsm6ds3_transfer_function *tf;
	struct st_lsm6ds3_transfer_buffer tb;
	struct early_suspend early_suspend;
};

struct st_lsm6ds3_transfer_function {
	int (*write) (struct lsm6ds3_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock);
	int (*read) (struct lsm6ds3_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock);
};

struct lsm6ds3_sensor_data {
	struct lsm6ds3_data *cdata;

	unsigned int c_odr;
	unsigned int c_gain[3];
	// by zhangjiajing, add acc bias support
	int c_bias[3];
	int c_offset[3];
	int sensor_phone_calling;
	u8 num_data_channels;
	u8 sindex;
	u8 *buffer_data;
};

int st_lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock);

int st_lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq);
void st_lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq);

int st_lsm6ds3_set_enable(struct lsm6ds3_sensor_data *sdata, bool enable);
int st_lsm6ds3_set_axis_enable(struct lsm6ds3_sensor_data *sdata, u8 value);
int st_lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state);
int st_lsm6ds3_set_fifo_mode(struct lsm6ds3_data *cdata, enum fifo_mode fm);
int st_lsm6ds3_reconfigure_fifo(struct lsm6ds3_data *cdata,
						bool disable_irq_and_flush);

ssize_t st_lsm6ds3_sysfs_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf);
ssize_t st_lsm6ds3_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
int st_lsm6ds3_i2c_read_fifo(struct lsm6ds3_data *cdata, int len, u8 *data);
int lsm6ds3_acc_selftest(struct lsm6ds3_sensor_data *acc, int *test_result);
int lsm6ds3_gyr_selftest(struct lsm6ds3_sensor_data *gyr, int *test_result);

#ifdef CONFIG_IIO_BUFFER
int st_lsm6ds3_allocate_rings(struct lsm6ds3_data *cdata);
void st_lsm6ds3_deallocate_rings(struct lsm6ds3_data *cdata);
int st_lsm6ds3_trig_set_state(struct iio_trigger *trig, bool state);
void st_lsm6ds3_read_fifo(struct lsm6ds3_data *cdata, bool check_fifo_len);
void st_lsm6ds3_cleanup_fifo(struct lsm6ds3_data *cdata);
int st_lsm6ds3_set_fifo_decimators_and_threshold(struct lsm6ds3_data *cdata);
#define ST_LSM6DS3_TRIGGER_SET_STATE (&st_lsm6ds3_trig_set_state)
#else /* CONFIG_IIO_BUFFER */
static inline int st_lsm6ds3_allocate_rings(struct lsm6ds3_data *cdata)
{
	return 0;
}
static inline void st_lsm6ds3_deallocate_rings(struct lsm6ds3_data *cdata)
{
}
#define ST_LSM6DS3_TRIGGER_SET_STATE NULL
#endif /* CONFIG_IIO_BUFFER */

#ifdef CONFIG_IIO_TRIGGER
void st_lsm6ds3_flush_works(void);
int st_lsm6ds3_allocate_triggers(struct lsm6ds3_data *cdata,
				const struct iio_trigger_ops *trigger_ops);

void st_lsm6ds3_deallocate_triggers(struct lsm6ds3_data *cdata);

#else /* CONFIG_IIO_TRIGGER */
static inline int st_lsm6ds3_allocate_triggers(struct lsm6ds3_data *cdata,
			const struct iio_trigger_ops *trigger_ops, int irq)
{
	return 0;
}
static inline void st_lsm6ds3_deallocate_triggers(struct lsm6ds3_data *cdata,
								int irq)
{
	return;
}
static inline void st_lsm6ds3_flush_works()
{
	return;
}
#endif /* CONFIG_IIO_TRIGGER */

#ifdef CONFIG_PM
int st_lsm6ds3_common_suspend(struct lsm6ds3_data *cdata);
int st_lsm6ds3_common_resume(struct lsm6ds3_data *cdata);
#endif /* CONFIG_PM */

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
int st_lsm6ds3_i2c_master_probe(struct lsm6ds3_data *cdata);
int st_lsm6ds3_i2c_master_exit(struct lsm6ds3_data *cdata);
int st_lsm6ds3_enable_passthrough(struct lsm6ds3_data *cdata, bool enable);
int st_lsm6ds3_enable_accel_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable);
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
static inline int st_lsm6ds3_i2c_master_probe(struct lsm6ds3_data *cdata)
{
	return 0;
}
static inline int st_lsm6ds3_i2c_master_exit(struct lsm6ds3_data *cdata)
{
	return 0;
}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#endif /* ST_LSM6DS3_H */
