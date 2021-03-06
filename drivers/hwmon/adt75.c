/*
 * An hwmon driver for the Analog Devices ADT75
 *
 * Based on ad7418.c
 * Copyright (C) 2006-07 Tower Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License,
 * as published by the Free Software Foundation - version 2.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "lm75.h"

#define DRV_VERSION "0.1"

/* ADT75 registers */
#define ADT75_REG_TEMP_IN	0x00
#define ADT75_REG_CONF		0x01
#define ADT75_REG_TEMP_HYST	0x02
#define ADT75_REG_TEMP_OS	0x03

static const u8 ADT75_REG_TEMP[] = { ADT75_REG_TEMP_IN,
					ADT75_REG_TEMP_HYST,
					ADT75_REG_TEMP_OS };

struct adt75_data {
	struct device		*hwmon_dev;
	struct attribute_group	attrs;
	struct mutex		lock;
	char			valid;
	unsigned long		last_updated;	/* In jiffies */
	s16			temp[3];	/* Register values */
};

static int adt75_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int adt75_remove(struct i2c_client *client);

static const struct i2c_device_id adt75_id[] = {
	{ "adt75", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adt75_id);

static struct i2c_driver adt75_driver = {
	.driver = {
		.name	= "adt75",
	},
	.probe		= adt75_probe,
	.remove		= adt75_remove,
	.id_table	= adt75_id,
};

static inline int adt75_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_word_data(client, reg);
}

static inline int adt75_write(struct i2c_client *client, u8 reg, u16 value)
{
	return i2c_smbus_write_word_data(client, reg, value);
}

static void adt75_init_client(struct i2c_client *client)
{
	int ret = adt75_write(client, ADT75_REG_CONF, 0x0000);
	if (ret < 0)
		dev_err(&client->dev, "cannot write configuration register\n");
}

static struct adt75_data *adt75_update_device(struct device *dev)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	struct adt75_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {

		for (i = 0; i < 3; i++)
			data->temp[i] = adt75_read(client, ADT75_REG_TEMP[i]);

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->lock);

	return data;
}

static ssize_t show_temp(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct adt75_data *data = adt75_update_device(dev);
	return sprintf(buf, "%d\n",
		LM75_TEMP_FROM_REG(data->temp[attr->index]));
}

static ssize_t set_temp(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct i2c_client *client = to_i2c_client(dev);
	struct adt75_data *data = i2c_get_clientdata(client);
	long temp = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->lock);
	data->temp[attr->index] = LM75_TEMP_TO_REG(temp);
	adt75_write(client,
		    ADT75_REG_TEMP[attr->index],
		    data->temp[attr->index]);
	mutex_unlock(&data->lock);
	return count;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IWUSR | S_IRUGO,
				show_temp, set_temp, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO,
				show_temp, set_temp, 2);

static struct attribute *adt75_attributes[] = {
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};

static int adt75_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adt75_data *data;
	int err;

	data = kzalloc(sizeof(struct adt75_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);
	data->attrs.attrs = adt75_attributes;

	dev_info(&client->dev, "%s chip found\n", client->name);

	/* Initialize the ADT75 chip */
	adt75_init_client(client);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
exit_free:
	kfree(data);
exit:
	return err;
}

static int adt75_remove(struct i2c_client *client)
{
	struct adt75_data *data = i2c_get_clientdata(client);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	kfree(data);
	return 0;
}

static int __init adt75_init(void)
{
	return i2c_add_driver(&adt75_driver);
}

static void __exit adt75_exit(void)
{
	i2c_del_driver(&adt75_driver);
}

MODULE_AUTHOR("Alessandro Zummo <a.zummo@towertech.it>");
MODULE_DESCRIPTION("ADT75 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(adt75_init);
module_exit(adt75_exit);
