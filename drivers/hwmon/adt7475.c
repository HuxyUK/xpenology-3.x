/*
 * adt7475 - Thermal sensor driver for the ADT7475 chip and derivatives
 * Copyright (C) 2007-2008, Advanced Micro Devices, Inc.
 * Copyright (C) 2008 Jordan Crouse <jordan@cosmicpenguin.net>
 * Copyright (C) 2008 Hans de Goede <hdegoede@redhat.com>
 * Copyright (C) 2009 Jean Delvare <khali@linux-fr.org>
 *
 * Derived from the lm83 driver by Jean Delvare
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/err.h>

/* Indexes for the sysfs hooks */

#define INPUT		0
#define MIN		1
#define MAX		2
#define CONTROL		3
#define OFFSET		3
#define AUTOMIN		4
#define THERM		5
#define HYSTERSIS	6

/* These are unique identifiers for the sysfs functions - unlike the
   numbers above, these are not also indexes into an array
*/

#define ALARM		9
#define FAULT		10

/* 7475 Common Registers */
#define REG_DEVREV2		0x12    /* ADT7490 only */

#define REG_VTT			0x1E    /* ADT7490 only */
#define REG_EXTEND3		0x1F    /* ADT7490 only */

#define REG_VOLTAGE_BASE	0x20
#define REG_TEMP_BASE		0x25
#define REG_TACH_BASE		0x28
#define REG_PWM_BASE		0x30
#define REG_PWM_MAX_BASE	0x38

#define REG_DEVID		0x3D
#define REG_VENDID		0x3E
#define REG_DEVID2		0x3F

#define REG_STATUS1		0x41
#define REG_STATUS2		0x42

#define REG_VID			0x43	/* ADT7476 only */

#define REG_VOLTAGE_MIN_BASE	0x44
#define REG_VOLTAGE_MAX_BASE	0x45

#define REG_TEMP_MIN_BASE	0x4E
#define REG_TEMP_MAX_BASE	0x4F

#define REG_TACH_MIN_BASE	0x54

#define REG_PWM_CONFIG_BASE	0x5C

#define REG_TEMP_TRANGE_BASE	0x5F

#define REG_PWM_MIN_BASE	0x64

#define REG_TEMP_TMIN_BASE	0x67
#define REG_TEMP_THERM_BASE	0x6A

#define REG_REMOTE1_HYSTERSIS	0x6D
#define REG_REMOTE2_HYSTERSIS	0x6E

#define REG_TEMP_OFFSET_BASE	0x70

#ifdef CONFIG_SYNO_ADT7490_FEATURES
#define REG_CONFIG1				0x40	/* ADT7490 only */
#define REG_PECI0				0x33	/* ADT7490 only */
#define REG_PECI1_BASE			0x1A	/* ADT7490 only */
#define REG_PECI_CONFIG			0x88	/* ADT7490 only */
#define REG_PECI_MIN			0x3B	/* ADT7490 only */
#define REG_PECI_RANGE			0x3C	/* ADT7490 only */
#define REG_PECI_OFFSET_BASE	0x94	/* ADT7490 only */
#define REG_PECI_LOW_LIMIT		0x34	/* ADT7490 only */
#define REG_PECI_HIGH_LIMIT		0x35	/* ADT7490 only */
#define REG_ACOUSTIC			0x62	/* ADT7490 only */
#endif

#define REG_CONFIG2		0x73

#define REG_EXTEND1		0x76
#define REG_EXTEND2		0x77

#define REG_CONFIG3		0x78
#define REG_CONFIG5		0x7C
#define REG_CONFIG4		0x7D

#define REG_STATUS4		0x81	/* ADT7490 only */

#define REG_VTT_MIN		0x84	/* ADT7490 only */
#define REG_VTT_MAX		0x86	/* ADT7490 only */

#define VID_VIDSEL		0x80	/* ADT7476 only */

#define CONFIG2_ATTN		0x20

#define CONFIG3_SMBALERT	0x01
#define CONFIG3_THERM		0x02

#define CONFIG4_PINFUNC		0x03
#define CONFIG4_MAXDUTY		0x08
#define CONFIG4_ATTN_IN10	0x30
#define CONFIG4_ATTN_IN43	0xC0

#define CONFIG5_TWOSCOMP	0x01
#define CONFIG5_TEMPOFFSET	0x02
#define CONFIG5_VIDGPIO		0x10	/* ADT7476 only */

/* ADT7475 Settings */

#define ADT7475_VOLTAGE_COUNT	5	/* Not counting Vtt */
#define ADT7475_TEMP_COUNT	3
#define ADT7475_TACH_COUNT	4
#define ADT7475_PWM_COUNT	3
#ifdef CONFIG_SYNO_ADT7490_FEATURES
#define ADT7490_PECI_COUNT	4	/*ADT7490 only*/
#define SYNO_IS_ADT7490(client) !strcmp(client->name, "adt7490")
#endif

/* Macro to read the registers */

#define adt7475_read(reg) i2c_smbus_read_byte_data(client, (reg))

/* Macros to easily index the registers */

#define TACH_REG(idx) (REG_TACH_BASE + ((idx) * 2))
#define TACH_MIN_REG(idx) (REG_TACH_MIN_BASE + ((idx) * 2))

#define PWM_REG(idx) (REG_PWM_BASE + (idx))
#define PWM_MAX_REG(idx) (REG_PWM_MAX_BASE + (idx))
#define PWM_MIN_REG(idx) (REG_PWM_MIN_BASE + (idx))
#define PWM_CONFIG_REG(idx) (REG_PWM_CONFIG_BASE + (idx))

#define VOLTAGE_REG(idx) (REG_VOLTAGE_BASE + (idx))
#define VOLTAGE_MIN_REG(idx) (REG_VOLTAGE_MIN_BASE + ((idx) * 2))
#define VOLTAGE_MAX_REG(idx) (REG_VOLTAGE_MAX_BASE + ((idx) * 2))

#ifdef CONFIG_SYNO_ADT7490_FEATURES
#define PECI_REG(idx) (idx == 0 ? REG_PECI0:REG_PECI1_BASE + (idx-1))	/* ADT7490 only */
#define PECI_OFFSET_REG(idx) (REG_PECI_OFFSET_BASE + (idx))
#endif
#define TEMP_REG(idx) (REG_TEMP_BASE + (idx))
#define TEMP_MIN_REG(idx) (REG_TEMP_MIN_BASE + ((idx) * 2))
#define TEMP_MAX_REG(idx) (REG_TEMP_MAX_BASE + ((idx) * 2))
#define TEMP_TMIN_REG(idx) (REG_TEMP_TMIN_BASE + (idx))
#define TEMP_THERM_REG(idx) (REG_TEMP_THERM_BASE + (idx))
#define TEMP_OFFSET_REG(idx) (REG_TEMP_OFFSET_BASE + (idx))
#define TEMP_TRANGE_REG(idx) (REG_TEMP_TRANGE_BASE + (idx))

static const unsigned short normal_i2c[] = { 0x2c, 0x2d, 0x2e, I2C_CLIENT_END };

enum chips { adt7473, adt7475, adt7476, adt7490 };

static const struct i2c_device_id adt7475_id[] = {
	{ "adt7473", adt7473 },
	{ "adt7475", adt7475 },
	{ "adt7476", adt7476 },
	{ "adt7490", adt7490 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adt7475_id);

struct adt7475_data {
	struct device *hwmon_dev;
	struct mutex lock;

	unsigned long measure_updated;
	unsigned long limits_updated;
	char valid;

	u8 config4;
	u8 config5;
	u8 has_voltage;
	u8 bypass_attn;		/* Bypass voltage attenuator */
	u8 has_pwm2:1;
	u8 has_fan4:1;
	u8 has_vid:1;
	u32 alarms;
	u16 voltage[3][6];
	u16 temp[7][3];
	u16 tach[2][4];
	u8 pwm[4][3];
	u8 range[3];
	u8 pwmctl[3];
	u8 pwmchan[3];

	u8 vid;
	u8 vrm;
#ifdef CONFIG_SYNO_ADT7490_FEATURES
	u8 pwmsynoctl[4];
	u16 peci[7][4];	/* ADT7490 only */
	u8 peci_range[4];
#endif
};

static struct i2c_driver adt7475_driver;
static struct adt7475_data *adt7475_update_device(struct device *dev);
static void adt7475_read_hystersis(struct i2c_client *client);
static void adt7475_read_pwm(struct i2c_client *client, int index);

/* Given a temp value, convert it to register value */

static inline u16 temp2reg(struct adt7475_data *data, long val)
{
	u16 ret;

	if (!(data->config5 & CONFIG5_TWOSCOMP)) {
		val = SENSORS_LIMIT(val, -64000, 191000);
		ret = (val + 64500) / 1000;
	} else {
		val = SENSORS_LIMIT(val, -128000, 127000);
		if (val < -500)
			ret = (256500 + val) / 1000;
		else
			ret = (val + 500) / 1000;
	}

	return ret << 2;
}

/* Given a register value, convert it to a real temp value */

static inline int reg2temp(struct adt7475_data *data, u16 reg)
{
	if (data->config5 & CONFIG5_TWOSCOMP) {
		if (reg >= 512)
			return (reg - 1024) * 250;
		else
			return reg * 250;
	} else
		return (reg - 256) * 250;
}

static inline int tach2rpm(u16 tach)
{
	if (tach == 0 || tach == 0xFFFF)
		return 0;

	return (90000 * 60) / tach;
}

static inline u16 rpm2tach(unsigned long rpm)
{
	if (rpm == 0)
		return 0;

	return SENSORS_LIMIT((90000 * 60) / rpm, 1, 0xFFFF);
}

/* Scaling factors for voltage inputs, taken from the ADT7490 datasheet */
static const int adt7473_in_scaling[ADT7475_VOLTAGE_COUNT + 1][2] = {
	{ 45, 94 },	/* +2.5V */
	{ 175, 525 },	/* Vccp */
	{ 68, 71 },	/* Vcc */
	{ 93, 47 },	/* +5V */
	{ 120, 20 },	/* +12V */
	{ 45, 45 },	/* Vtt */
};

static inline int reg2volt(int channel, u16 reg, u8 bypass_attn)
{
	const int *r = adt7473_in_scaling[channel];

	if (bypass_attn & (1 << channel))
		return DIV_ROUND_CLOSEST(reg * 2250, 1024);
	return DIV_ROUND_CLOSEST(reg * (r[0] + r[1]) * 2250, r[1] * 1024);
}

static inline u16 volt2reg(int channel, long volt, u8 bypass_attn)
{
	const int *r = adt7473_in_scaling[channel];
	long reg;

	if (bypass_attn & (1 << channel))
		reg = (volt * 1024) / 2250;
	else
		reg = (volt * r[1] * 1024) / ((r[0] + r[1]) * 2250);
	return SENSORS_LIMIT(reg, 0, 1023) & (0xff << 2);
}

static u16 adt7475_read_word(struct i2c_client *client, int reg)
{
	u16 val;

	val = i2c_smbus_read_byte_data(client, reg);
	val |= (i2c_smbus_read_byte_data(client, reg + 1) << 8);

	return val;
}

static void adt7475_write_word(struct i2c_client *client, int reg, u16 val)
{
	i2c_smbus_write_byte_data(client, reg + 1, val >> 8);
	i2c_smbus_write_byte_data(client, reg, val & 0xFF);
}

/* Find the nearest value in a table - used for pwm frequency and
   auto temp range */
static int find_nearest(long val, const int *array, int size)
{
	int i;

	if (val < array[0])
		return 0;

	if (val > array[size - 1])
		return size - 1;

	for (i = 0; i < size - 1; i++) {
		int a, b;

		if (val > array[i + 1])
			continue;

		a = val - array[i];
		b = array[i + 1] - val;

		return (a <= b) ? i : i + 1;
	}

	return 0;
}

static ssize_t show_voltage(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	unsigned short val;

	switch (sattr->nr) {
	case ALARM:
		return sprintf(buf, "%d\n",
			       (data->alarms >> sattr->index) & 1);
	default:
		val = data->voltage[sattr->nr][sattr->index];
		return sprintf(buf, "%d\n",
			       reg2volt(sattr->index, val, data->bypass_attn));
	}
}

static ssize_t set_voltage(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned char reg;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	data->voltage[sattr->nr][sattr->index] =
				volt2reg(sattr->index, val, data->bypass_attn);

	if (sattr->index < ADT7475_VOLTAGE_COUNT) {
		if (sattr->nr == MIN)
			reg = VOLTAGE_MIN_REG(sattr->index);
		else
			reg = VOLTAGE_MAX_REG(sattr->index);
	} else {
		if (sattr->nr == MIN)
			reg = REG_VTT_MIN;
		else
			reg = REG_VTT_MAX;
	}

	i2c_smbus_write_byte_data(client, reg,
				  data->voltage[sattr->nr][sattr->index] >> 2);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_temp(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out;

	switch (sattr->nr) {
	case HYSTERSIS:
		mutex_lock(&data->lock);
		out = data->temp[sattr->nr][sattr->index];
		if (sattr->index != 1)
			out = (out >> 4) & 0xF;
		else
			out = (out & 0xF);
		/* Show the value as an absolute number tied to
		 * THERM */
		out = reg2temp(data, data->temp[THERM][sattr->index]) -
			out * 1000;
		mutex_unlock(&data->lock);
		break;

	case OFFSET:
		/* Offset is always 2's complement, regardless of the
		 * setting in CONFIG5 */
		mutex_lock(&data->lock);
		out = (s8)data->temp[sattr->nr][sattr->index];
		if (data->config5 & CONFIG5_TEMPOFFSET)
			out *= 1000;
		else
			out *= 500;
		mutex_unlock(&data->lock);
		break;

	case ALARM:
		out = (data->alarms >> (sattr->index + 4)) & 1;
		break;

	case FAULT:
		/* Note - only for remote1 and remote2 */
		out = !!(data->alarms & (sattr->index ? 0x8000 : 0x4000));
		break;

	default:
		/* All other temp values are in the configured format */
		out = reg2temp(data, data->temp[sattr->nr][sattr->index]);
	}

	return sprintf(buf, "%d\n", out);
}

static ssize_t set_temp(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned char reg = 0;
	u8 out;
	int temp;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	/* We need the config register in all cases for temp <-> reg conv. */
	data->config5 = adt7475_read(REG_CONFIG5);

	switch (sattr->nr) {
	case OFFSET:
		if (data->config5 & CONFIG5_TEMPOFFSET) {
			val = SENSORS_LIMIT(val, -63000, 127000);
			out = data->temp[OFFSET][sattr->index] = val / 1000;
		} else {
			val = SENSORS_LIMIT(val, -63000, 64000);
			out = data->temp[OFFSET][sattr->index] = val / 500;
		}
		break;

	case HYSTERSIS:
		/* The value will be given as an absolute value, turn it
		   into an offset based on THERM */

		/* Read fresh THERM and HYSTERSIS values from the chip */
		data->temp[THERM][sattr->index] =
			adt7475_read(TEMP_THERM_REG(sattr->index)) << 2;
		adt7475_read_hystersis(client);

		temp = reg2temp(data, data->temp[THERM][sattr->index]);
		val = SENSORS_LIMIT(val, temp - 15000, temp);
		val = (temp - val) / 1000;

#ifdef CONFIG_SYNO_ADT7490_FEATURES
		if (sattr->index != 1) {
			data->temp[HYSTERSIS][sattr->index] &= 0x0F;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF) << 4;
		} else {
			data->temp[HYSTERSIS][sattr->index] &= 0xF0;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF);
		}
#else
		if (sattr->index != 1) {
			data->temp[HYSTERSIS][sattr->index] &= 0xF0;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF) << 4;
		} else {
			data->temp[HYSTERSIS][sattr->index] &= 0x0F;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF);
		}
#endif

		out = data->temp[HYSTERSIS][sattr->index];
		break;

	default:
		data->temp[sattr->nr][sattr->index] = temp2reg(data, val);

		/* We maintain an extra 2 digits of precision for simplicity
		 * - shift those back off before writing the value */
		out = (u8) (data->temp[sattr->nr][sattr->index] >> 2);
	}

	switch (sattr->nr) {
	case MIN:
		reg = TEMP_MIN_REG(sattr->index);
		break;
	case MAX:
		reg = TEMP_MAX_REG(sattr->index);
		break;
	case OFFSET:
		reg = TEMP_OFFSET_REG(sattr->index);
		break;
	case AUTOMIN:
		reg = TEMP_TMIN_REG(sattr->index);
		break;
	case THERM:
		reg = TEMP_THERM_REG(sattr->index);
		break;
	case HYSTERSIS:
		if (sattr->index != 2)
			reg = REG_REMOTE1_HYSTERSIS;
		else
			reg = REG_REMOTE2_HYSTERSIS;

		break;
	}

	i2c_smbus_write_byte_data(client, reg, out);

	mutex_unlock(&data->lock);
	return count;
}

/* Table of autorange values - the user will write the value in millidegrees,
   and we'll convert it */
static const int autorange_table[] = {
	2000, 2500, 3330, 4000, 5000, 6670, 8000,
	10000, 13330, 16000, 20000, 26670, 32000, 40000,
	53330, 80000
};

static ssize_t show_point2(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out, val;

	mutex_lock(&data->lock);
	out = (data->range[sattr->index] >> 4) & 0x0F;
	val = reg2temp(data, data->temp[AUTOMIN][sattr->index]);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", val + autorange_table[out]);
}

static ssize_t set_point2(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int temp;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	/* Get a fresh copy of the needed registers */
	data->config5 = adt7475_read(REG_CONFIG5);
	data->temp[AUTOMIN][sattr->index] =
		adt7475_read(TEMP_TMIN_REG(sattr->index)) << 2;
	data->range[sattr->index] =
		adt7475_read(TEMP_TRANGE_REG(sattr->index));

	/* The user will write an absolute value, so subtract the start point
	   to figure the range */
	temp = reg2temp(data, data->temp[AUTOMIN][sattr->index]);
	val = SENSORS_LIMIT(val, temp + autorange_table[0],
		temp + autorange_table[ARRAY_SIZE(autorange_table) - 1]);
	val -= temp;

	/* Find the nearest table entry to what the user wrote */
	val = find_nearest(val, autorange_table, ARRAY_SIZE(autorange_table));

	data->range[sattr->index] &= ~0xF0;
	data->range[sattr->index] |= val << 4;

	i2c_smbus_write_byte_data(client, TEMP_TRANGE_REG(sattr->index),
				  data->range[sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_tach(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out;

	if (sattr->nr == ALARM)
		out = (data->alarms >> (sattr->index + 10)) & 1;
	else
		out = tach2rpm(data->tach[sattr->nr][sattr->index]);

	return sprintf(buf, "%d\n", out);
}

static ssize_t set_tach(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	data->tach[MIN][sattr->index] = rpm2tach(val);

	adt7475_write_word(client, TACH_MIN_REG(sattr->index),
			   data->tach[MIN][sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n", data->pwm[sattr->nr][sattr->index]);
}

static ssize_t show_pwmchan(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n", data->pwmchan[sattr->index]);
}

static ssize_t show_pwmctrl(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n", data->pwmctl[sattr->index]);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned char reg = 0;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	switch (sattr->nr) {
	case INPUT:
		/* Get a fresh value for CONTROL */
		data->pwm[CONTROL][sattr->index] =
			adt7475_read(PWM_CONFIG_REG(sattr->index));

		/* If we are not in manual mode, then we shouldn't allow
		 * the user to set the pwm speed */
		if (((data->pwm[CONTROL][sattr->index] >> 5) & 7) != 7) {
			mutex_unlock(&data->lock);
			return count;
		}

		reg = PWM_REG(sattr->index);
		break;

	case MIN:
		reg = PWM_MIN_REG(sattr->index);
		break;

	case MAX:
		reg = PWM_MAX_REG(sattr->index);
		break;
	}

	data->pwm[sattr->nr][sattr->index] = SENSORS_LIMIT(val, 0, 0xFF);
	i2c_smbus_write_byte_data(client, reg,
				  data->pwm[sattr->nr][sattr->index]);

	mutex_unlock(&data->lock);

	return count;
}

/* Called by set_pwmctrl and set_pwmchan */

static int hw_set_pwm(struct i2c_client *client, int index,
		      unsigned int pwmctl, unsigned int pwmchan)
{
	struct adt7475_data *data = i2c_get_clientdata(client);
	long val = 0;

	switch (pwmctl) {
	case 0:
		val = 0x03;	/* Run at full speed */
		break;
	case 1:
		val = 0x07;	/* Manual mode */
		break;
	case 2:
		switch (pwmchan) {
		case 1:
			/* Remote1 controls PWM */
			val = 0x00;
			break;
		case 2:
			/* local controls PWM */
			val = 0x01;
			break;
		case 4:
			/* remote2 controls PWM */
			val = 0x02;
			break;
		case 6:
			/* local/remote2 control PWM */
			val = 0x05;
			break;
		case 7:
			/* All three control PWM */
			val = 0x06;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	data->pwmctl[index] = pwmctl;
	data->pwmchan[index] = pwmchan;

	data->pwm[CONTROL][index] &= ~0xE0;
	data->pwm[CONTROL][index] |= (val & 7) << 5;

	i2c_smbus_write_byte_data(client, PWM_CONFIG_REG(index),
				  data->pwm[CONTROL][index]);

	return 0;
}

static ssize_t set_pwmchan(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	int r;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	adt7475_read_pwm(client, sattr->index);
	r = hw_set_pwm(client, sattr->index, data->pwmctl[sattr->index], val);
	if (r)
		count = r;
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t set_pwmctrl(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	int r;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	adt7475_read_pwm(client, sattr->index);
	r = hw_set_pwm(client, sattr->index, val, data->pwmchan[sattr->index]);
	if (r)
		count = r;
	mutex_unlock(&data->lock);

	return count;
}

/* List of frequencies for the PWM */
static const int pwmfreq_table[] = {
	11, 14, 22, 29, 35, 44, 58, 88
};

static ssize_t show_pwmfreq(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n",
		       pwmfreq_table[data->range[sattr->index] & 7]);
}

static ssize_t set_pwmfreq(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	int out;
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	out = find_nearest(val, pwmfreq_table, ARRAY_SIZE(pwmfreq_table));

	mutex_lock(&data->lock);

	data->range[sattr->index] =
		adt7475_read(TEMP_TRANGE_REG(sattr->index));
	data->range[sattr->index] &= ~7;
	data->range[sattr->index] |= out;

	i2c_smbus_write_byte_data(client, TEMP_TRANGE_REG(sattr->index),
				  data->range[sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_pwm_at_crit(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	return sprintf(buf, "%d\n", !!(data->config4 & CONFIG4_MAXDUTY));
}

static ssize_t set_pwm_at_crit(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	mutex_lock(&data->lock);
	data->config4 = i2c_smbus_read_byte_data(client, REG_CONFIG4);
	if (val)
		data->config4 |= CONFIG4_MAXDUTY;
	else
		data->config4 &= ~CONFIG4_MAXDUTY;
	i2c_smbus_write_byte_data(client, REG_CONFIG4, data->config4);
	mutex_unlock(&data->lock);

	return count;
}

#ifdef CONFIG_SYNO_ADT7490_FEATURES

/* set peci */
static ssize_t set_peci(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned char reg = 0;
	u8 out = 0;
	long val = 0;
	long temp = 0;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* We need the config register in all cases for temp <-> reg conv. */
	data->config5 = adt7475_read(REG_CONFIG5);

	switch (sattr->nr) {
		case OFFSET:
			val = SENSORS_LIMIT(val, -127000, 127000);
			out = data->peci[OFFSET][sattr->index] = val / 1000;
			break;

		case HYSTERSIS:
			/* The value will be given as an absolute value, turn it
			   into an offset based on THERM */
			/* Read fresh THERM and HYSTERSIS values from the chip */
			/* in ADT7490 register 0x3D is for PECI Tcontrol*/
			data->peci[THERM][sattr->index] = adt7475_read(REG_DEVID) << 2;
			adt7475_read_hystersis(client);
			temp = reg2temp(data, data->peci[THERM][sattr->index]);
			val = SENSORS_LIMIT(val, temp - 15000, temp);
			val = (temp - val) / 1000;

			data->peci[HYSTERSIS][sattr->index] &= 0xF0;
			data->peci[HYSTERSIS][sattr->index] |= (val & 0xF);

			out = data->peci[HYSTERSIS][sattr->index];
			break;

		default:
			data->peci[sattr->nr][sattr->index] = temp2reg(data, val);
			/* We maintain an extra 2 digits of precision for simplicity
			 * - shift those back off before writing the value */
			out = (u8) (data->peci[sattr->nr][sattr->index] >> 2);
	}

	switch (sattr->nr) {
	case MIN:
		reg = REG_PECI_LOW_LIMIT;
		break;
	case MAX:
		reg = REG_PECI_HIGH_LIMIT;
		break;
	case OFFSET:
		reg = PECI_OFFSET_REG(sattr->index);
		break;
	case AUTOMIN:
		reg = REG_PECI_MIN;
		break;
	case THERM:
		reg = REG_DEVID;
		break;
	case HYSTERSIS:
		reg = REG_REMOTE2_HYSTERSIS;
		break;
	}

	i2c_smbus_write_byte_data(client, reg, out);

	mutex_unlock(&data->lock);
	return count;
}
/*
 * Show temperature from PECI interface
 */
static ssize_t show_peci(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	// this function is adt7490 only
	struct adt7475_data *data = adt7475_update_device(dev);
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out = 0;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	switch (sattr->nr) {
		case OFFSET:
			mutex_lock(&data->lock);
			out = (s8)data->peci[sattr->nr][sattr->index] * 1000;
			mutex_unlock(&data->lock);
			break;

		case HYSTERSIS:
			mutex_lock(&data->lock);
			out = data->peci[sattr->nr][sattr->index];
			out &= 0xF;
			out = reg2temp(data, data->peci[THERM][sattr->index]) - out * 1000;
			mutex_unlock(&data->lock);
			break;

		default:
			/* show peci temperature */
			mutex_lock(&data->lock);
			out = reg2temp(data, data->peci[sattr->nr][sattr->index]);
			mutex_unlock(&data->lock);
			break;
	}

	return sprintf(buf, "%d\n", out);
}

static ssize_t show_adt_full_duty_cycle(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config1;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}
	config1 = adt7475_read(REG_CONFIG1);

	return sprintf(buf, "%d\n", (config1 & 0x8) > 0 ? 1 : 0);
}

static ssize_t set_adt_full_duty_cycle(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config1;
	long val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	config1 = adt7475_read(REG_CONFIG1);
	if (val) {
		config1 |= 0x8;
	} else {
		config1 &= ~0x8;
	}

	i2c_smbus_write_byte_data(client, REG_CONFIG1, config1);

	return count;
}

static ssize_t show_peci_error(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}
	config = adt7475_read(REG_VID);
	return sprintf(buf, "%d\n", config);
}

static ssize_t show_enh_acou_reg(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}
	config = adt7475_read(REG_ACOUSTIC);
	return sprintf(buf, "%d\n", config >> 5);
}

static ssize_t set_enh_acou_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config;
	long val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	config = adt7475_read(REG_ACOUSTIC);
	config &= ~0xE0;
	config |= (val & 0x7) << 5;

	i2c_smbus_write_byte_data(client, REG_ACOUSTIC, config);
	return count;
}

static ssize_t show_adtenable(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config1;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}
	config1 = adt7475_read(REG_CONFIG1);
	return sprintf(buf, "%d\n", config1 & 0x1);
}

static ssize_t set_adtenable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	// this function is adt7490 only
	struct i2c_client *client = to_i2c_client(dev);
	u8 config1;
	long val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	config1 = adt7475_read(REG_CONFIG1);
	if (val) {
		config1 |= 0x1;
	} else {
		config1 &= ~0x1;
	}
	i2c_smbus_write_byte_data(client, REG_CONFIG1, config1);
	return count;
}

/*
 * Show high frequency configure of pwm
 */
static ssize_t show_pwmHighFreq(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct adt7475_data *data = i2c_get_clientdata(client);
	int out;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	mutex_lock(&data->lock);
	out = (data->range[sattr->index] & 0x8) >> 3;
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", out);
}

/*
 * Set pwm output as high frequency
 */
static ssize_t set_pwmHighFreq(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct adt7475_data *data = i2c_get_clientdata(client);
	long val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	data->range[sattr->index] =
		adt7475_read(TEMP_TRANGE_REG(sattr->index));
	if (val) {
		data->range[sattr->index] |= 0x8;
	} else {
		data->range[sattr->index] &= ~0x8;
	}

	i2c_smbus_write_byte_data(client, TEMP_TRANGE_REG(sattr->index),
				  data->range[sattr->index]);
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_peci_point2(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = adt7475_update_device(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out, val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	mutex_lock(&data->lock);
	out = (data->peci_range[sattr->index] >> 4) & 0x0F;
	val = reg2temp(data, data->peci[AUTOMIN][sattr->index]);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", val + autorange_table[out]);
}

static ssize_t set_peci_point2(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int temp;
	long val;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	/* Get a fresh copy of the needed registers */
	data->config5 = adt7475_read(REG_CONFIG5);
	data->peci[AUTOMIN][sattr->index] = adt7475_read(REG_PECI_MIN) << 2;
	data->peci_range[sattr->index] = adt7475_read(REG_PECI_RANGE);

	/* The user will write an absolute value, so subtract the start point
	   to figure the range */
	temp = reg2temp(data, data->peci[AUTOMIN][sattr->index]);
	val = SENSORS_LIMIT(val, temp + autorange_table[0],
		temp + autorange_table[ARRAY_SIZE(autorange_table) - 1]);
	val -= temp;

	/* Find the nearest table entry to what the user wrote */
	val = find_nearest(val, autorange_table, ARRAY_SIZE(autorange_table));

	data->peci_range[sattr->index] &= ~0xF0;
	data->peci_range[sattr->index] |= val << 4;

	i2c_smbus_write_byte_data(client, REG_PECI_RANGE,
				  data->peci_range[sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_pwm_syno_control(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}
	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	adt7475_read_pwm(client, sattr->index);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", data->pwmsynoctl[sattr->index]);
}

/*
 * This function is set the pwm control sensor,
 * In ADT7490 spec, it could choose many way to control the pwm
 * we implement all the valid control mapping in this fucntion
 *
 * We don't use pwm_enable and pwm_auto_channel to set pwm control source
 */
static ssize_t set_pwm_syno_control(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	long val = 0;
	long inputVal = 0;
	long valt = 0;
	int index = sattr->index;

	if (!SYNO_IS_ADT7490(client)) {
		return -EINVAL;
	}

	if (strict_strtol(buf, 10, &inputVal))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	data->pwm[CONTROL][index] = adt7475_read(PWM_CONFIG_REG(index));

	switch (inputVal) {
		case 1:
			valt = 0;
			val = 0x03;	/* Run at maximun duty cycle (set by pwmMax) */
			break;
		case 2:
			valt = 0;
			val = 0x07;	/* Manual mode*/
			break;
		case 3:
			valt = 1;
			val = 0x07;	/* Source from all peci and all sensor */
			break;
		case 4:
			valt = 1;
			val = 0x05;	/* Source from all peci */
			break;
		case 5:
			valt = 1;
			val = 0x00;	/* Source from peci0 */
			break;
		case 6:
			valt = 1;
			val = 0x01;	/* Source from peci1 */
			break;
		case 7:
			valt = 1;
			val = 0x02;	/* Source from peci2 */
			break;
		case 8:
			valt = 1;
			val = 0x03;	/* Source from peci3 */
			break;
		case 9:
			valt = 0;
			val = 0x06;	/* Source from all sensors */
			break;
		case 10:
			valt = 0;
			val = 0x05;	/* Source from local and remote2 */
			break;
		case 11:
			valt = 0;
			val = 0x00;	/* Source from remote1 */
			break;
		case 12:
			valt = 0;
			val = 0x01;	/* Source from local */
			break;
		case 13:
			valt = 0;
			val = 0x02;	/* Source from remote2 */
			break;
		default:
			valt = 0;
			val = 0x03;	/* Run at maximun duty cycle (set by pwmMax) */
			break;
	}

	data->pwm[CONTROL][index] &= ~0xE8;
	data->pwm[CONTROL][index] |= (valt & 1) << 3;
	data->pwm[CONTROL][index] |= (val & 7) << 5;

	i2c_smbus_write_byte_data(client, PWM_CONFIG_REG(index),
				  data->pwm[CONTROL][index]);
	mutex_unlock(&data->lock);

	return count;
}
#endif

static ssize_t show_vrm(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct adt7475_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (int)data->vrm);
}

static ssize_t set_vrm(struct device *dev, struct device_attribute *devattr,
		       const char *buf, size_t count)
{
	struct adt7475_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;
	if (val < 0 || val > 255)
		return -EINVAL;
	data->vrm = val;

	return count;
}

static ssize_t show_vid(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct adt7475_data *data = adt7475_update_device(dev);
	return sprintf(buf, "%d\n", vid_from_reg(data->vid, data->vrm));
}

static SENSOR_DEVICE_ATTR_2(in0_input, S_IRUGO, show_voltage, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(in0_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 0);
static SENSOR_DEVICE_ATTR_2(in0_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 0);
static SENSOR_DEVICE_ATTR_2(in0_alarm, S_IRUGO, show_voltage, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(in1_input, S_IRUGO, show_voltage, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(in1_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 1);
static SENSOR_DEVICE_ATTR_2(in1_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 1);
static SENSOR_DEVICE_ATTR_2(in1_alarm, S_IRUGO, show_voltage, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(in2_input, S_IRUGO, show_voltage, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(in2_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 2);
static SENSOR_DEVICE_ATTR_2(in2_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 2);
static SENSOR_DEVICE_ATTR_2(in2_alarm, S_IRUGO, show_voltage, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(in3_input, S_IRUGO, show_voltage, NULL, INPUT, 3);
static SENSOR_DEVICE_ATTR_2(in3_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 3);
static SENSOR_DEVICE_ATTR_2(in3_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 3);
static SENSOR_DEVICE_ATTR_2(in3_alarm, S_IRUGO, show_voltage, NULL, ALARM, 3);
static SENSOR_DEVICE_ATTR_2(in4_input, S_IRUGO, show_voltage, NULL, INPUT, 4);
static SENSOR_DEVICE_ATTR_2(in4_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 4);
static SENSOR_DEVICE_ATTR_2(in4_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 4);
static SENSOR_DEVICE_ATTR_2(in4_alarm, S_IRUGO, show_voltage, NULL, ALARM, 8);
static SENSOR_DEVICE_ATTR_2(in5_input, S_IRUGO, show_voltage, NULL, INPUT, 5);
static SENSOR_DEVICE_ATTR_2(in5_max, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MAX, 5);
static SENSOR_DEVICE_ATTR_2(in5_min, S_IRUGO | S_IWUSR, show_voltage,
			    set_voltage, MIN, 5);
static SENSOR_DEVICE_ATTR_2(in5_alarm, S_IRUGO, show_voltage, NULL, ALARM, 31);
static SENSOR_DEVICE_ATTR_2(temp1_input, S_IRUGO, show_temp, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(temp1_alarm, S_IRUGO, show_temp, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(temp1_fault, S_IRUGO, show_temp, NULL, FAULT, 0);
static SENSOR_DEVICE_ATTR_2(temp1_max, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MAX, 0);
static SENSOR_DEVICE_ATTR_2(temp1_min, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MIN, 0);
static SENSOR_DEVICE_ATTR_2(temp1_offset, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, OFFSET, 0);
static SENSOR_DEVICE_ATTR_2(temp1_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_temp, set_temp, AUTOMIN, 0);
static SENSOR_DEVICE_ATTR_2(temp1_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_point2, set_point2, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp1_crit, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    THERM, 0);
static SENSOR_DEVICE_ATTR_2(temp1_crit_hyst, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, HYSTERSIS, 0);
static SENSOR_DEVICE_ATTR_2(temp2_input, S_IRUGO, show_temp, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(temp2_alarm, S_IRUGO, show_temp, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(temp2_max, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MAX, 1);
static SENSOR_DEVICE_ATTR_2(temp2_min, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MIN, 1);
static SENSOR_DEVICE_ATTR_2(temp2_offset, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, OFFSET, 1);
static SENSOR_DEVICE_ATTR_2(temp2_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_temp, set_temp, AUTOMIN, 1);
static SENSOR_DEVICE_ATTR_2(temp2_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_point2, set_point2, 0, 1);
static SENSOR_DEVICE_ATTR_2(temp2_crit, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    THERM, 1);
static SENSOR_DEVICE_ATTR_2(temp2_crit_hyst, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, HYSTERSIS, 1);
static SENSOR_DEVICE_ATTR_2(temp3_input, S_IRUGO, show_temp, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(temp3_alarm, S_IRUGO, show_temp, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(temp3_fault, S_IRUGO, show_temp, NULL, FAULT, 2);
static SENSOR_DEVICE_ATTR_2(temp3_max, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MAX, 2);
static SENSOR_DEVICE_ATTR_2(temp3_min, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    MIN, 2);
static SENSOR_DEVICE_ATTR_2(temp3_offset, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, OFFSET, 2);
static SENSOR_DEVICE_ATTR_2(temp3_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_temp, set_temp, AUTOMIN, 2);
static SENSOR_DEVICE_ATTR_2(temp3_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_point2, set_point2, 0, 2);
static SENSOR_DEVICE_ATTR_2(temp3_crit, S_IRUGO | S_IWUSR, show_temp, set_temp,
			    THERM, 2);
static SENSOR_DEVICE_ATTR_2(temp3_crit_hyst, S_IRUGO | S_IWUSR, show_temp,
			    set_temp, HYSTERSIS, 2);
static SENSOR_DEVICE_ATTR_2(fan1_input, S_IRUGO, show_tach, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(fan1_min, S_IRUGO | S_IWUSR, show_tach, set_tach,
			    MIN, 0);
static SENSOR_DEVICE_ATTR_2(fan1_alarm, S_IRUGO, show_tach, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(fan2_input, S_IRUGO, show_tach, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(fan2_min, S_IRUGO | S_IWUSR, show_tach, set_tach,
			    MIN, 1);
static SENSOR_DEVICE_ATTR_2(fan2_alarm, S_IRUGO, show_tach, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(fan3_input, S_IRUGO, show_tach, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(fan3_min, S_IRUGO | S_IWUSR, show_tach, set_tach,
			    MIN, 2);
static SENSOR_DEVICE_ATTR_2(fan3_alarm, S_IRUGO, show_tach, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(fan4_input, S_IRUGO, show_tach, NULL, INPUT, 3);
static SENSOR_DEVICE_ATTR_2(fan4_min, S_IRUGO | S_IWUSR, show_tach, set_tach,
			    MIN, 3);
static SENSOR_DEVICE_ATTR_2(fan4_alarm, S_IRUGO, show_tach, NULL, ALARM, 3);
static SENSOR_DEVICE_ATTR_2(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm, INPUT,
			    0);
static SENSOR_DEVICE_ATTR_2(pwm1_freq, S_IRUGO | S_IWUSR, show_pwmfreq,
			    set_pwmfreq, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_enable, S_IRUGO | S_IWUSR, show_pwmctrl,
			    set_pwmctrl, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_channels_temp, S_IRUGO | S_IWUSR,
			    show_pwmchan, set_pwmchan, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_point1_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MIN, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_point2_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MAX, 0);
static SENSOR_DEVICE_ATTR_2(pwm2, S_IRUGO | S_IWUSR, show_pwm, set_pwm, INPUT,
			    1);
static SENSOR_DEVICE_ATTR_2(pwm2_freq, S_IRUGO | S_IWUSR, show_pwmfreq,
			    set_pwmfreq, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_enable, S_IRUGO | S_IWUSR, show_pwmctrl,
			    set_pwmctrl, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_channels_temp, S_IRUGO | S_IWUSR,
			    show_pwmchan, set_pwmchan, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_point1_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MIN, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_point2_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MAX, 1);
static SENSOR_DEVICE_ATTR_2(pwm3, S_IRUGO | S_IWUSR, show_pwm, set_pwm, INPUT,
			    2);
static SENSOR_DEVICE_ATTR_2(pwm3_freq, S_IRUGO | S_IWUSR, show_pwmfreq,
			    set_pwmfreq, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_enable, S_IRUGO | S_IWUSR, show_pwmctrl,
			    set_pwmctrl, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_channels_temp, S_IRUGO | S_IWUSR,
			    show_pwmchan, set_pwmchan, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_point1_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MIN, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_point2_pwm, S_IRUGO | S_IWUSR, show_pwm,
			    set_pwm, MAX, 2);
#ifdef CONFIG_SYNO_ADT7490_FEATURES
static SENSOR_DEVICE_ATTR_2(peci0_input, S_IRUGO, show_peci, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(peci0_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_peci, set_peci, AUTOMIN, 0);
static SENSOR_DEVICE_ATTR_2(peci0_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_peci_point2, set_peci_point2, 0, 0);
static SENSOR_DEVICE_ATTR_2(peci0_crit, S_IRUGO | S_IWUSR, show_peci, set_peci,
			    THERM, 0);
static SENSOR_DEVICE_ATTR_2(peci0_crit_hyst, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, HYSTERSIS, 0);
static SENSOR_DEVICE_ATTR_2(peci0_offset, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, OFFSET, 0);
static SENSOR_DEVICE_ATTR_2(peci0_min, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MIN, 0);
static SENSOR_DEVICE_ATTR_2(peci0_max, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MAX, 0);
static SENSOR_DEVICE_ATTR_2(peci1_input, S_IRUGO, show_peci, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(peci1_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_peci, set_peci, AUTOMIN, 1);
static SENSOR_DEVICE_ATTR_2(peci1_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_peci_point2, set_peci_point2, 0, 1);
static SENSOR_DEVICE_ATTR_2(peci1_crit, S_IRUGO | S_IWUSR, show_peci, set_peci,
			    THERM, 1);
static SENSOR_DEVICE_ATTR_2(peci1_crit_hyst, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, HYSTERSIS, 1);
static SENSOR_DEVICE_ATTR_2(peci1_offset, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, OFFSET, 1);
static SENSOR_DEVICE_ATTR_2(peci1_min, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MIN, 1);
static SENSOR_DEVICE_ATTR_2(peci1_max, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MAX, 1);
static SENSOR_DEVICE_ATTR_2(peci2_input, S_IRUGO, show_peci, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(peci2_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_peci, set_peci, AUTOMIN, 2);
static SENSOR_DEVICE_ATTR_2(peci2_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_peci_point2, set_peci_point2, 0, 2);
static SENSOR_DEVICE_ATTR_2(peci2_crit, S_IRUGO | S_IWUSR, show_peci, set_peci,
			    THERM, 2);
static SENSOR_DEVICE_ATTR_2(peci2_crit_hyst, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, HYSTERSIS, 2);
static SENSOR_DEVICE_ATTR_2(peci2_offset, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, OFFSET, 2);
static SENSOR_DEVICE_ATTR_2(peci2_min, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MIN, 2);
static SENSOR_DEVICE_ATTR_2(peci2_max, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MAX, 2);
static SENSOR_DEVICE_ATTR_2(peci3_input, S_IRUGO, show_peci, NULL, INPUT, 3);
static SENSOR_DEVICE_ATTR_2(peci3_auto_point1_temp, S_IRUGO | S_IWUSR,
			    show_peci, set_peci, AUTOMIN, 3);
static SENSOR_DEVICE_ATTR_2(peci3_auto_point2_temp, S_IRUGO | S_IWUSR,
			    show_peci_point2, set_peci_point2, 0, 3);
static SENSOR_DEVICE_ATTR_2(peci3_crit, S_IRUGO | S_IWUSR, show_peci, set_peci,
			    THERM, 3);
static SENSOR_DEVICE_ATTR_2(peci3_crit_hyst, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, HYSTERSIS, 3);
static SENSOR_DEVICE_ATTR_2(peci3_offset, S_IRUGO | S_IWUSR, show_peci,
			    set_peci, OFFSET, 3);
static SENSOR_DEVICE_ATTR_2(peci3_min, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MIN, 3);
static SENSOR_DEVICE_ATTR_2(peci3_max, S_IRUGO | S_IWUSR, show_peci, set_peci,
				MAX, 3);
static SENSOR_DEVICE_ATTR_2(enable, S_IRUGO | S_IWUSR, show_adtenable,
			    set_adtenable, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_high_freq, S_IWUSR | S_IRUGO, show_pwmHighFreq, 
				set_pwmHighFreq, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm2_high_freq, S_IWUSR | S_IRUGO, show_pwmHighFreq, 
				set_pwmHighFreq, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm3_high_freq, S_IWUSR | S_IRUGO, show_pwmHighFreq, 
				set_pwmHighFreq, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm1_syno_control, S_IRUGO | S_IWUSR, show_pwm_syno_control,
				set_pwm_syno_control, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm2_syno_control, S_IRUGO | S_IWUSR, show_pwm_syno_control,
				set_pwm_syno_control, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm3_syno_control, S_IRUGO | S_IWUSR, show_pwm_syno_control,
				set_pwm_syno_control, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(full_duty_cycle, S_IRUGO | S_IWUSR, show_adt_full_duty_cycle,
			    set_adt_full_duty_cycle, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(peci_error, S_IRUGO, show_peci_error, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(enhanced_acoustic_register, S_IWUSR | S_IRUGO, show_enh_acou_reg,
				set_enh_acou_reg, INPUT, 0);
#endif

/* Non-standard name, might need revisiting */
static DEVICE_ATTR(pwm_use_point2_pwm_at_crit, S_IWUSR | S_IRUGO,
		   show_pwm_at_crit, set_pwm_at_crit);

static DEVICE_ATTR(vrm, S_IWUSR | S_IRUGO, show_vrm, set_vrm);
static DEVICE_ATTR(cpu0_vid, S_IRUGO, show_vid, NULL);

static struct attribute *adt7475_attrs[] = {
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in1_alarm.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_fault.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_offset.dev_attr.attr,
	&sensor_dev_attr_temp1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp2_offset.dev_attr.attr,
	&sensor_dev_attr_temp2_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp2_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_fault.dev_attr.attr,
	&sensor_dev_attr_temp3_alarm.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_min.dev_attr.attr,
	&sensor_dev_attr_temp3_offset.dev_attr.attr,
	&sensor_dev_attr_temp3_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp3_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp3_crit.dev_attr.attr,
	&sensor_dev_attr_temp3_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_min.dev_attr.attr,
	&sensor_dev_attr_fan1_alarm.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_min.dev_attr.attr,
	&sensor_dev_attr_fan2_alarm.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan3_min.dev_attr.attr,
	&sensor_dev_attr_fan3_alarm.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_freq.dev_attr.attr,
	&sensor_dev_attr_pwm1_enable.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm3_freq.dev_attr.attr,
	&sensor_dev_attr_pwm3_enable.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_point2_pwm.dev_attr.attr,
#ifdef CONFIG_SYNO_ADT7490_FEATURES
	&sensor_dev_attr_peci0_input.dev_attr.attr,
	&sensor_dev_attr_peci0_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_peci0_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_peci0_crit.dev_attr.attr,
	&sensor_dev_attr_peci0_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_peci0_offset.dev_attr.attr,
	&sensor_dev_attr_peci0_min.dev_attr.attr,
	&sensor_dev_attr_peci0_max.dev_attr.attr,

	&sensor_dev_attr_peci1_input.dev_attr.attr,
	&sensor_dev_attr_peci1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_peci1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_peci1_crit.dev_attr.attr,
	&sensor_dev_attr_peci1_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_peci1_offset.dev_attr.attr,
	&sensor_dev_attr_peci1_min.dev_attr.attr,
	&sensor_dev_attr_peci1_max.dev_attr.attr,

	&sensor_dev_attr_peci2_input.dev_attr.attr,
	&sensor_dev_attr_peci2_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_peci2_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_peci2_crit.dev_attr.attr,
	&sensor_dev_attr_peci2_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_peci2_offset.dev_attr.attr,
	&sensor_dev_attr_peci2_min.dev_attr.attr,
	&sensor_dev_attr_peci2_max.dev_attr.attr,

	&sensor_dev_attr_peci3_input.dev_attr.attr,
	&sensor_dev_attr_peci3_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_peci3_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_peci3_crit.dev_attr.attr,
	&sensor_dev_attr_peci3_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_peci3_offset.dev_attr.attr,
	&sensor_dev_attr_peci3_min.dev_attr.attr,
	&sensor_dev_attr_peci3_max.dev_attr.attr,

	&sensor_dev_attr_enable.dev_attr.attr,
	&sensor_dev_attr_pwm1_high_freq.dev_attr.attr,
	&sensor_dev_attr_pwm2_high_freq.dev_attr.attr,
	&sensor_dev_attr_pwm3_high_freq.dev_attr.attr,
	&sensor_dev_attr_pwm1_syno_control.dev_attr.attr,
	&sensor_dev_attr_pwm2_syno_control.dev_attr.attr,
	&sensor_dev_attr_pwm3_syno_control.dev_attr.attr,
	&sensor_dev_attr_full_duty_cycle.dev_attr.attr,
	&sensor_dev_attr_peci_error.dev_attr.attr,
	&sensor_dev_attr_enhanced_acoustic_register.dev_attr.attr,
#endif
	&dev_attr_pwm_use_point2_pwm_at_crit.attr,
	NULL,
};

static struct attribute *fan4_attrs[] = {
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan4_min.dev_attr.attr,
	&sensor_dev_attr_fan4_alarm.dev_attr.attr,
	NULL
};

static struct attribute *pwm2_attrs[] = {
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm2_freq.dev_attr.attr,
	&sensor_dev_attr_pwm2_enable.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point2_pwm.dev_attr.attr,
	NULL
};

static struct attribute *in0_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in0_alarm.dev_attr.attr,
	NULL
};

static struct attribute *in3_attrs[] = {
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in3_alarm.dev_attr.attr,
	NULL
};

static struct attribute *in4_attrs[] = {
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	&sensor_dev_attr_in4_alarm.dev_attr.attr,
	NULL
};

static struct attribute *in5_attrs[] = {
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in5_max.dev_attr.attr,
	&sensor_dev_attr_in5_min.dev_attr.attr,
	&sensor_dev_attr_in5_alarm.dev_attr.attr,
	NULL
};

static struct attribute *vid_attrs[] = {
	&dev_attr_cpu0_vid.attr,
	&dev_attr_vrm.attr,
	NULL
};

static struct attribute_group adt7475_attr_group = { .attrs = adt7475_attrs };
static struct attribute_group fan4_attr_group = { .attrs = fan4_attrs };
static struct attribute_group pwm2_attr_group = { .attrs = pwm2_attrs };
static struct attribute_group in0_attr_group = { .attrs = in0_attrs };
static struct attribute_group in3_attr_group = { .attrs = in3_attrs };
static struct attribute_group in4_attr_group = { .attrs = in4_attrs };
static struct attribute_group in5_attr_group = { .attrs = in5_attrs };
static struct attribute_group vid_attr_group = { .attrs = vid_attrs };

static int adt7475_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int vendid, devid, devid2;
	const char *name;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	vendid = adt7475_read(REG_VENDID);
	devid2 = adt7475_read(REG_DEVID2);
	if (vendid != 0x41 ||		/* Analog Devices */
	    (devid2 & 0xf8) != 0x68)
		return -ENODEV;

	devid = adt7475_read(REG_DEVID);
	if (devid == 0x73)
		name = "adt7473";
	else if (devid == 0x75 && client->addr == 0x2e)
		name = "adt7475";
	else if (devid == 0x76)
		name = "adt7476";
	else if ((devid2 & 0xfc) == 0x6c)
		name = "adt7490";
	else {
		dev_dbg(&adapter->dev,
			"Couldn't detect an ADT7473/75/76/90 part at "
			"0x%02x\n", (unsigned int)client->addr);
		return -ENODEV;
	}

	strlcpy(info->type, name, I2C_NAME_SIZE);

	return 0;
}

static void adt7475_remove_files(struct i2c_client *client,
				 struct adt7475_data *data)
{
	sysfs_remove_group(&client->dev.kobj, &adt7475_attr_group);
	if (data->has_fan4)
		sysfs_remove_group(&client->dev.kobj, &fan4_attr_group);
	if (data->has_pwm2)
		sysfs_remove_group(&client->dev.kobj, &pwm2_attr_group);
	if (data->has_voltage & (1 << 0))
		sysfs_remove_group(&client->dev.kobj, &in0_attr_group);
	if (data->has_voltage & (1 << 3))
		sysfs_remove_group(&client->dev.kobj, &in3_attr_group);
	if (data->has_voltage & (1 << 4))
		sysfs_remove_group(&client->dev.kobj, &in4_attr_group);
	if (data->has_voltage & (1 << 5))
		sysfs_remove_group(&client->dev.kobj, &in5_attr_group);
	if (data->has_vid)
		sysfs_remove_group(&client->dev.kobj, &vid_attr_group);
}

static int adt7475_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	static const char *names[] = {
		[adt7473] = "ADT7473",
		[adt7475] = "ADT7475",
		[adt7476] = "ADT7476",
		[adt7490] = "ADT7490",
	};

	struct adt7475_data *data;
	int i, ret = 0, revision;
	u8 config2, config3;
#ifdef CONFIG_SYNO_ADT7490_FEATURES
	u8 config1, configPECI;
#endif

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	/* Initialize device-specific values */
	switch (id->driver_data) {
	case adt7476:
		data->has_voltage = 0x0e;	/* in1 to in3 */
		revision = adt7475_read(REG_DEVID2) & 0x07;
		break;
	case adt7490:
		data->has_voltage = 0x3e;	/* in1 to in5 */
		revision = adt7475_read(REG_DEVID2) & 0x03;
		if (revision == 0x03)
			revision += adt7475_read(REG_DEVREV2);
		break;
	default:
		data->has_voltage = 0x06;	/* in1, in2 */
		revision = adt7475_read(REG_DEVID2) & 0x07;
	}

#ifdef CONFIG_SYNO_ADT7490_FEATURES
	if (SYNO_IS_ADT7490(client)) {
		config1 = adt7475_read(REG_CONFIG1);
		// which means adt7490 is not been told to start
		if (!(config1 & 0x1)) {
			// which means adt7490 is ready to go...
			if (config1 & 0x4) {
				config1 |= 0x11;
				i2c_smbus_write_byte_data(client, REG_CONFIG1, config1);
			}
		}

		configPECI = adt7475_read(REG_PECI_CONFIG);
		configPECI = 0x00;
		i2c_smbus_write_byte_data(client, REG_PECI_CONFIG, configPECI);
	}
#endif

	config3 = adt7475_read(REG_CONFIG3);
	/* Pin PWM2 may alternatively be used for ALERT output */
	if (!(config3 & CONFIG3_SMBALERT))
		data->has_pwm2 = 1;
	/* Meaning of this bit is inverted for the ADT7473-1 */
	if (id->driver_data == adt7473 && revision >= 1)
		data->has_pwm2 = !data->has_pwm2;

	data->config4 = adt7475_read(REG_CONFIG4);
	/* Pin TACH4 may alternatively be used for THERM */
	if ((data->config4 & CONFIG4_PINFUNC) == 0x0)
		data->has_fan4 = 1;

	/* THERM configuration is more complex on the ADT7476 and ADT7490,
	   because 2 different pins (TACH4 and +2.5 Vin) can be used for
	   this function */
	if (id->driver_data == adt7490) {
		if ((data->config4 & CONFIG4_PINFUNC) == 0x1 &&
		    !(config3 & CONFIG3_THERM))
			data->has_fan4 = 1;
	}
	if (id->driver_data == adt7476 || id->driver_data == adt7490) {
		if (!(config3 & CONFIG3_THERM) ||
		    (data->config4 & CONFIG4_PINFUNC) == 0x1)
			data->has_voltage |= (1 << 0);		/* in0 */
	}

	/* On the ADT7476, the +12V input pin may instead be used as VID5,
	   and VID pins may alternatively be used as GPIO */
	if (id->driver_data == adt7476) {
		u8 vid = adt7475_read(REG_VID);
		if (!(vid & VID_VIDSEL))
			data->has_voltage |= (1 << 4);		/* in4 */

		data->has_vid = !(adt7475_read(REG_CONFIG5) & CONFIG5_VIDGPIO);
	}

	/* Voltage attenuators can be bypassed, globally or individually */
	config2 = adt7475_read(REG_CONFIG2);
	if (config2 & CONFIG2_ATTN) {
		data->bypass_attn = (0x3 << 3) | 0x3;
	} else {
		data->bypass_attn = ((data->config4 & CONFIG4_ATTN_IN10) >> 4) |
				    ((data->config4 & CONFIG4_ATTN_IN43) >> 3);
	}
	data->bypass_attn &= data->has_voltage;

	/* Call adt7475_read_pwm for all pwm's as this will reprogram any
	   pwm's which are disabled to manual mode with 0% duty cycle */
	for (i = 0; i < ADT7475_PWM_COUNT; i++)
		adt7475_read_pwm(client, i);

	ret = sysfs_create_group(&client->dev.kobj, &adt7475_attr_group);
	if (ret)
		goto efree;

	/* Features that can be disabled individually */
	if (data->has_fan4) {
		ret = sysfs_create_group(&client->dev.kobj, &fan4_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_pwm2) {
		ret = sysfs_create_group(&client->dev.kobj, &pwm2_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_voltage & (1 << 0)) {
		ret = sysfs_create_group(&client->dev.kobj, &in0_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_voltage & (1 << 3)) {
		ret = sysfs_create_group(&client->dev.kobj, &in3_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_voltage & (1 << 4)) {
		ret = sysfs_create_group(&client->dev.kobj, &in4_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_voltage & (1 << 5)) {
		ret = sysfs_create_group(&client->dev.kobj, &in5_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_vid) {
		data->vrm = vid_which_vrm();
		ret = sysfs_create_group(&client->dev.kobj, &vid_attr_group);
		if (ret)
			goto eremove;
	}

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto eremove;
	}

	dev_info(&client->dev, "%s device, revision %d\n",
		 names[id->driver_data], revision);
	if ((data->has_voltage & 0x11) || data->has_fan4 || data->has_pwm2)
		dev_info(&client->dev, "Optional features:%s%s%s%s%s\n",
			 (data->has_voltage & (1 << 0)) ? " in0" : "",
			 (data->has_voltage & (1 << 4)) ? " in4" : "",
			 data->has_fan4 ? " fan4" : "",
			 data->has_pwm2 ? " pwm2" : "",
			 data->has_vid ? " vid" : "");
	if (data->bypass_attn)
		dev_info(&client->dev, "Bypassing attenuators on:%s%s%s%s\n",
			 (data->bypass_attn & (1 << 0)) ? " in0" : "",
			 (data->bypass_attn & (1 << 1)) ? " in1" : "",
			 (data->bypass_attn & (1 << 3)) ? " in3" : "",
			 (data->bypass_attn & (1 << 4)) ? " in4" : "");

	return 0;

eremove:
	adt7475_remove_files(client, data);
efree:
	kfree(data);
	return ret;
}

static int adt7475_remove(struct i2c_client *client)
{
	struct adt7475_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	adt7475_remove_files(client, data);
	kfree(data);

	return 0;
}

static struct i2c_driver adt7475_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "adt7475",
	},
	.probe		= adt7475_probe,
	.remove		= adt7475_remove,
	.id_table	= adt7475_id,
	.detect		= adt7475_detect,
	.address_list	= normal_i2c,
};

static void adt7475_read_hystersis(struct i2c_client *client)
{
	struct adt7475_data *data = i2c_get_clientdata(client);

	data->temp[HYSTERSIS][0] = (u16) adt7475_read(REG_REMOTE1_HYSTERSIS);
	data->temp[HYSTERSIS][1] = data->temp[HYSTERSIS][0];
	data->temp[HYSTERSIS][2] = (u16) adt7475_read(REG_REMOTE2_HYSTERSIS);
#ifdef CONFIG_SYNO_ADT7490_FEATURES
	data->peci[HYSTERSIS][0] = (u16) adt7475_read(REG_REMOTE2_HYSTERSIS);
	data->peci[HYSTERSIS][1] = (u16) adt7475_read(REG_REMOTE2_HYSTERSIS);
	data->peci[HYSTERSIS][2] = (u16) adt7475_read(REG_REMOTE2_HYSTERSIS);
	data->peci[HYSTERSIS][3] = (u16) adt7475_read(REG_REMOTE2_HYSTERSIS);
#endif
}

#ifdef CONFIG_SYNO_ADT7490_FEATURES
static unsigned int adt7490_pwmctl_read(const unsigned int pwmReg)
{
	unsigned int valt = pwmReg & 0x8;
	unsigned int pwmSource = (pwmReg >> 5) & 7;
	unsigned int ret = 1;

	if (valt) {
		switch (pwmSource) {
			case 0x0:
				ret = 5;
				break;
			case 0x1:
				ret = 6;
				break;
			case 0x2:
				ret = 7;
				break;
			case 0x3:
				ret = 8;
				break;
			case 0x5:
				ret = 4;
				break;
			case 0x7:
				ret = 3;
				break;
			default:
				ret = 0;
				break;
		}
	} else {
		switch (pwmSource) {
			case 0x0:
				ret = 11;
				break;
			case 0x1:
				ret = 12;
				break;
			case 0x2:
				ret = 13;
				break;
			case 0x3:
				ret = 1;
				break;
			case 0x5:
				ret = 10;
				break;
			case 0x6:
				ret = 9;
				break;
			case 0x7:
				ret = 2;
				break;
			default:
				ret = 0;
				break;
		}
	}
	return ret;
}
#endif

static void adt7475_read_pwm(struct i2c_client *client, int index)
{
	struct adt7475_data *data = i2c_get_clientdata(client);
	unsigned int v;

	data->pwm[CONTROL][index] = adt7475_read(PWM_CONFIG_REG(index));


	/* Figure out the internal value for pwmctrl and pwmchan
	   based on the current settings */
	v = (data->pwm[CONTROL][index] >> 5) & 7;
#ifdef CONFIG_SYNO_ADT7490_FEATURES
	data->pwmsynoctl[index] = adt7490_pwmctl_read(data->pwm[CONTROL][index]);
#endif
	if (v == 3)
		data->pwmctl[index] = 0;
	else if (v == 7)
		data->pwmctl[index] = 1;
	else if (v == 4) {
		/* The fan is disabled - we don't want to
		   support that, so change to manual mode and
		   set the duty cycle to 0 instead
		*/
		data->pwm[INPUT][index] = 0;
		data->pwm[CONTROL][index] &= ~0xE0;
		data->pwm[CONTROL][index] |= (7 << 5);

		i2c_smbus_write_byte_data(client, PWM_CONFIG_REG(index),
					  data->pwm[INPUT][index]);

		i2c_smbus_write_byte_data(client, PWM_CONFIG_REG(index),
					  data->pwm[CONTROL][index]);

		data->pwmctl[index] = 1;
	} else {
		data->pwmctl[index] = 2;

		switch (v) {
		case 0:
			data->pwmchan[index] = 1;
			break;
		case 1:
			data->pwmchan[index] = 2;
			break;
		case 2:
			data->pwmchan[index] = 4;
			break;
		case 5:
			data->pwmchan[index] = 6;
			break;
		case 6:
			data->pwmchan[index] = 7;
			break;
		}
	}
}

static struct adt7475_data *adt7475_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adt7475_data *data = i2c_get_clientdata(client);
	u16 ext;
	int i;

	mutex_lock(&data->lock);

	/* Measurement values update every 2 seconds */
	if (time_after(jiffies, data->measure_updated + HZ * 2) ||
	    !data->valid) {
		data->alarms = adt7475_read(REG_STATUS2) << 8;
		data->alarms |= adt7475_read(REG_STATUS1);

		ext = (adt7475_read(REG_EXTEND2) << 8) |
			adt7475_read(REG_EXTEND1);
		for (i = 0; i < ADT7475_VOLTAGE_COUNT; i++) {
			if (!(data->has_voltage & (1 << i)))
				continue;
			data->voltage[INPUT][i] =
				(adt7475_read(VOLTAGE_REG(i)) << 2) |
				((ext >> (i * 2)) & 3);
		}

		for (i = 0; i < ADT7475_TEMP_COUNT; i++)
			data->temp[INPUT][i] =
				(adt7475_read(TEMP_REG(i)) << 2) |
				((ext >> ((i + 5) * 2)) & 3);
		if (data->has_voltage & (1 << 5)) {
			data->alarms |= adt7475_read(REG_STATUS4) << 24;
			ext = adt7475_read(REG_EXTEND3);
			data->voltage[INPUT][5] = adt7475_read(REG_VTT) << 2 |
				((ext >> 4) & 3);
		}

#ifdef CONFIG_SYNO_ADT7490_FEATURES
		if (SYNO_IS_ADT7490(client)) {
			for (i = 0; i < ADT7490_PECI_COUNT; i++) {
				/* Adjust values so they match the input precision */
				data->peci[MIN][i] = adt7475_read(REG_PECI_LOW_LIMIT) << 2;
				data->peci[MAX][i] = adt7475_read(REG_PECI_HIGH_LIMIT) << 2;
				data->peci[AUTOMIN][i] = adt7475_read(REG_PECI_MIN) << 2;
				data->peci[THERM][i] = adt7475_read(REG_DEVID) << 2;
				data->peci[INPUT][i] = adt7475_read(PECI_REG(i)) << 2;
				data->peci[OFFSET][i] = adt7475_read(PECI_OFFSET_REG(i));
				data->peci_range[i] = adt7475_read(REG_PECI_RANGE);
			}
		}
#endif

		for (i = 0; i < ADT7475_TACH_COUNT; i++) {
			if (i == 3 && !data->has_fan4)
				continue;
			data->tach[INPUT][i] =
				adt7475_read_word(client, TACH_REG(i));
		}

		/* Updated by hw when in auto mode */
		for (i = 0; i < ADT7475_PWM_COUNT; i++) {
			if (i == 1 && !data->has_pwm2)
				continue;
			data->pwm[INPUT][i] = adt7475_read(PWM_REG(i));
		}

		if (data->has_vid)
			data->vid = adt7475_read(REG_VID) & 0x3f;

		data->measure_updated = jiffies;
	}

	/* Limits and settings, should never change update every 60 seconds */
	if (time_after(jiffies, data->limits_updated + HZ * 60) ||
	    !data->valid) {
		data->config4 = adt7475_read(REG_CONFIG4);
		data->config5 = adt7475_read(REG_CONFIG5);

		for (i = 0; i < ADT7475_VOLTAGE_COUNT; i++) {
			if (!(data->has_voltage & (1 << i)))
				continue;
			/* Adjust values so they match the input precision */
			data->voltage[MIN][i] =
				adt7475_read(VOLTAGE_MIN_REG(i)) << 2;
			data->voltage[MAX][i] =
				adt7475_read(VOLTAGE_MAX_REG(i)) << 2;
		}

		if (data->has_voltage & (1 << 5)) {
			data->voltage[MIN][5] = adt7475_read(REG_VTT_MIN) << 2;
			data->voltage[MAX][5] = adt7475_read(REG_VTT_MAX) << 2;
		}

		for (i = 0; i < ADT7475_TEMP_COUNT; i++) {
			/* Adjust values so they match the input precision */
			data->temp[MIN][i] =
				adt7475_read(TEMP_MIN_REG(i)) << 2;
			data->temp[MAX][i] =
				adt7475_read(TEMP_MAX_REG(i)) << 2;
			data->temp[AUTOMIN][i] =
				adt7475_read(TEMP_TMIN_REG(i)) << 2;
			data->temp[THERM][i] =
				adt7475_read(TEMP_THERM_REG(i)) << 2;
			data->temp[OFFSET][i] =
				adt7475_read(TEMP_OFFSET_REG(i));
		}
		adt7475_read_hystersis(client);

		for (i = 0; i < ADT7475_TACH_COUNT; i++) {
			if (i == 3 && !data->has_fan4)
				continue;
			data->tach[MIN][i] =
				adt7475_read_word(client, TACH_MIN_REG(i));
		}

		for (i = 0; i < ADT7475_PWM_COUNT; i++) {
			if (i == 1 && !data->has_pwm2)
				continue;
			data->pwm[MAX][i] = adt7475_read(PWM_MAX_REG(i));
			data->pwm[MIN][i] = adt7475_read(PWM_MIN_REG(i));
			/* Set the channel and control information */
			adt7475_read_pwm(client, i);
		}

		data->range[0] = adt7475_read(TEMP_TRANGE_REG(0));
		data->range[1] = adt7475_read(TEMP_TRANGE_REG(1));
		data->range[2] = adt7475_read(TEMP_TRANGE_REG(2));

		data->limits_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->lock);

	return data;
}

static int __init sensors_adt7475_init(void)
{
	return i2c_add_driver(&adt7475_driver);
}

static void __exit sensors_adt7475_exit(void)
{
	i2c_del_driver(&adt7475_driver);
}

MODULE_AUTHOR("Advanced Micro Devices, Inc");
MODULE_DESCRIPTION("adt7475 driver");
MODULE_LICENSE("GPL");

module_init(sensors_adt7475_init);
module_exit(sensors_adt7475_exit);
