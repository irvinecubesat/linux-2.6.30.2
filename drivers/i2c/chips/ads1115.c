/*  Copyright (c) 2009  Christoph Mair <christoph.mair@gmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>


/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4A, 0x4B, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(ads1115);

#define ADS1115_CONFIG_REGISTER			0x01
#define ADS1115_CONVERSION_REGISTER		0x02
#define ADS1115_LO_THRESH_REGISTER		0x03
#define ADS1115_HI_THRESH_REGISTER		0x04

#define ADS1115_START_SINGLE_CONVERSION		(0x01 << 15)
#define ADS1115_CONVERSION_STATUS_MASK		(0x01 << 15)

#define ADS1115_MUX_MASK			(0x07 << 12)
#define ADS1115_MUX_AIN0_AIN1			(0x00 << 12)
#define ADS1115_MUX_AIN0_AIN3			(0x01 << 12)
#define ADS1115_MUX_AIN1_AIN3			(0x02 << 12)
#define ADS1115_MUX_AIN2_AIN3			(0x03 << 12)
#define ADS1115_MUX_AIN0			(0x04 << 12)
#define ADS1115_MUX_AIN1			(0x05 << 12)
#define ADS1115_MUX_AIN2			(0x06 << 12)
#define ADS1115_MUX_AIN3			(0x07 << 12)

#define ADS1115_PGA_MASK			(0x07 << 9)
#define ADS1115_PGA_6144			(0x00 << 9)
#define ADS1115_PGA_4096			(0x01 << 9)
#define ADS1115_PGA_2048			(0x02 << 9)
#define ADS1115_PGA_1024			(0x03 << 9)
#define ADS1115_PGA_0512			(0x04 << 9)
#define ADS1115_PGA_0256			(0x05 << 9)

#define ADS1115_CONVERSION_MODE_MASK		(0x01 << 8)
#define ADS1115_CONVERSION_SINGLESHOT		(0x01 << 8)
#define ADS1115_CONVERSION_CONTINUOUS		(0x00 << 8)

#define ADS1115_RATE_MASK			(0x07 << 5)
#define ADS1115_RATE_8				(0x00 << 5)
#define ADS1115_RATE_16				(0x01 << 5)
#define ADS1115_RATE_32				(0x02 << 5)
#define ADS1115_RATE_64				(0x03 << 5)
#define ADS1115_RATE_128			(0x04 << 5)
#define ADS1115_RATE_250			(0x05 << 5)
#define ADS1115_RATE_475			(0x06 << 5)
#define ADS1115_RATE_860			(0x07 << 5)

#define ADS1115_COMPARATOR_WINDOW_MASK		(0x01 << 4)
#define ADS1115_COMPARATOR_WINDOW		(0x01 << 4)

#define ADS1115_COMPARATOR_ACTIVE_MASK		(0x01 << 3)
#define ADS1115_COMPARATOR_ACTIVE_HIGH		(0x01 << 3)

#define ADS1115_COMPARATOR_LATCHING_MASK	(0x01 << 2)
#define ADS1115_COMPARATOR_LATCHING		(0x01 << 2)

#define ADS1115_COMPARATOR_QUEUE_MASK		(0x03 << 0)
#define ADS1115_COMPARATOR_QUEUE1		(0x00 << 0)
#define ADS1115_COMPARATOR_QUEUE2		(0x01 << 0)
#define ADS1115_COMPARATOR_QUEUE4		(0x02 << 0)
#define ADS1115_COMPARATOR_OFF			(0x03 << 0)

/* Each client has this additional data */
struct ads1115_data {
	/* device configuration word:
		Bit [15]	Operational status/single-shot conversion start
					write '1': start single conversion
					write '0': ignored
					read  '1': device idle
					read '0': conversion in progress
		Bits [14:12]	Input Mux (ADS1115 only)
					000 : AINP = AIN0 and AINN = AIN1 (default)
					001 : AINP = AIN0 and AINN = AIN3
					010 : AINP = AIN1 and AINN = AIN3
					011 : AINP = AIN2 and AINN = AIN3
					100 : AINP = AIN0 and AINN = GND
					101 : AINP = AIN1 and AINN = GND
					110 : AINP = AIN2 and AINN = GND
					111 : AINP = AIN3 and AINN = GND
		Bits [11:9]	PGA (ADS1114/5 only)
					000 : FS = ±6.144V
					001 : FS = ±4.096V
					010 : FS = ±2.048V (default)
					011 : FS = ±1.024V
					100 : FS = ±0.512V
					101 : FS = ±0.256V
					110 : FS = ±0.256V
					111 : FS = ±0.256V
		Bits 8		Device operation mode
					0 : Continuous conversion mode
					1 : Power-down single-shot mode (default)
		Bit [7:5]	Data rate
					000 : 8SPS
					001 : 16SPS
					010 : 32SPS
					011 : 64SPS
					100 : 128SPS (default)
					101 : 250SPS
					110 : 475SPS
					111 : 860SPS
		Bit [4]		Comperator mode (ADS1114/5 only)
					0 : Traditional comparator with hysteresis (default)
					1 : Window comparator
		Bit [3]		Comparator polarity
					0 : Active low (default)
					1 : Active high
		Bit [2]		Latching comparator
					0 : Non-latching comparator (default)
					1 : Latching comparator
		Bit [1:0]	Comparator queue and disable (ADS1114/5 only)
					00 : Assert after one conversion
					01 : Assert after two conversions
					10 : Assert after four conversions
					11 : Disable comparator (default)
	*/
	u16 config;
};

static void ads1115_init_client(struct i2c_client *client);

static s32 ads1115_read_conversion_result(struct i2c_client *client)
{
	s32 result = i2c_smbus_read_word_data(client, ADS1115_CONVERSION_REGISTER);
	printk(KERN_INFO "ads1115 result: %d\n", result);
	return result;
}


static s32 ads1115_read_config(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	s32 result = i2c_smbus_read_word_data(client, ADS1115_CONFIG_REGISTER);
	if (result >= 0) {
		data->config = result;
	}
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static s32 ads1115_write_config(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	s32 result = i2c_smbus_write_word_data(client, ADS1115_CONFIG_REGISTER, data->config);
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static s32 ads1115_read_lo_tresh(struct i2c_client *client)
{
	s32 result = i2c_smbus_read_word_data(client, ADS1115_LO_THRESH_REGISTER);
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static s32 ads1115_write_lo_thresh(struct i2c_client *client, s16 lo_thresh)
{
	s32 result = i2c_smbus_write_word_data(client, ADS1115_LO_THRESH_REGISTER, lo_thresh);
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static s32 ads1115_read_hi_tresh(struct i2c_client *client)
{
	s32 result = i2c_smbus_read_word_data(client, ADS1115_HI_THRESH_REGISTER);
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static s32 ads1115_write_hi_thresh(struct i2c_client *client, s16 hi_thresh)
{
	s32 result = i2c_smbus_write_word_data(client, ADS1115_HI_THRESH_REGISTER, hi_thresh);
	printk(KERN_INFO "ads1115 config: %d\n", result);
	return result;
}


static void ads1115_set_start_single_conversion(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	data->config &= ADS1115_START_SINGLE_CONVERSION;
}


static u8 ads1115_is_busy(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	return (data->config & ADS1115_CONVERSION_STATUS_MASK);
}


static void ads1115_set_input(struct i2c_client *client, u8 mux)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	data->config = (data->config & ~ADS1115_MUX_MASK) | mux;
}

static u8 ads1115_get_input(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	return data->config & ADS1115_MUX_MASK;
}

static void ads1115_set_pga(struct i2c_client *client, u8 pga)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	data->config = (data->config & ~ADS1115_PGA_MASK) | pga;
}

static u8 ads1115_get_pga(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	return data->config & ADS1115_PGA_MASK;
}


static void ads1115_set_conversion_mode(struct i2c_client *client, u8 singleshot)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	data->config = (data->config & ~ADS1115_CONVERSION_SINGLESHOT) & singleshot;
}


static u8 ads1115_get_conversion_mode(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	return data->config & ADS1115_CONVERSION_MODE_MASK;
}


static void ads1115_set_data_rate(struct i2c_client *client, u8 rate)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	data->config = (data->config & ~ADS1115_RATE_MASK) | rate;
}


static u8 ads1115_get_data_rate(struct i2c_client *client)
{
	struct ads1115_data *data = i2c_get_clientdata(client);
	return data->config & ADS1115_RATE_MASK;
}


/*
	Todo: write functions for the threshold
*/



/* sysfs callbacks */
static ssize_t start_conversion(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	ads1115_set_start_single_conversion(client);
	ads1115_write_config(client);
	return count;
}

static ssize_t show_conversion_in_progress(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	ads1115_read_config(client);
	return sprintf(buf, "%d\n", ads1115_is_busy(client));
}
static DEVICE_ATTR(start_conversion, S_IWUSR | S_IRUGO, show_conversion_in_progress, start_conversion);


static ssize_t set_input(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ads1115_data *data = i2c_get_clientdata(client);
	unsigned long input = simple_strtoul(buf, NULL, 10);
	ads1115_set_input(client, input * ADS1115_MUX_AIN0_AIN1);
	return count;
}

static ssize_t show_input(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	ads1115_read_config(client);
	return sprintf(buf, "%d\n", ads1115_get_input(client));
}
static DEVICE_ATTR(input, S_IWUSR | S_IRUGO, show_input, set_input);

/*
pga
conversion mode
data rate
*/

static struct attribute *ads1115_attributes[] = {
	&dev_attr_start_conversion.attr,
	&dev_attr_input.attr,
	NULL
};

static const struct attribute_group ads1115_attr_group = {
	.attrs = ads1115_attributes,
};


/* Return 0 if detection is successful, -ENODEV otherwise */
static int ads1115_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	const char *client_name;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	if (client->addr >= 0x48 && client->addr <= 0x4B) {
		if (!i2c_smbus_read_word_data(client, ADS1115_CONFIG_REGISTER)) {
			goto exit;
		}

		/* Config register can be read. Assume chip as identified */
		client_name = "ads1115";
		strlcpy(info->type, client_name, I2C_NAME_SIZE);
		printk(KERN_INFO "ADS1115 found!\n");
		return 0;
	}

	exit:
	printk(KERN_INFO "ADS1115 not found at address %x!\n", client->addr);
	return -ENODEV;
}

static int ads1115_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ads1115_data *data;
	int err;

	printk(KERN_INFO "ads1115 probe!\n");

	data = kzalloc(sizeof(struct ads1115_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);

	/* Initialize the BMP085 chip */
	ads1115_init_client(client);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ads1115_attr_group);
	if (err)
		goto exit_free;

	printk(KERN_INFO "ads1115 probe succeeded!\n");
	return 0;

	exit_free:
	kfree(data);
	exit:
	return err;
}

static int ads1115_remove(struct i2c_client *client)
{
	printk(KERN_INFO "ads1115 remove!\n");
	sysfs_remove_group(&client->dev.kobj, &ads1115_attr_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/* Called when we have found a new HMC5843. */
static void ads1115_init_client(struct i2c_client *client)
{
	ads1115_read_config(client);
}

static const struct i2c_device_id ads1115_id[] = {
	{ "ads1115", 0 },
	{ }
};

static struct i2c_driver ads1115_driver = {
	.driver = {
		.name	= "ads1115",
	},
	.probe		= ads1115_probe,
	.remove		= ads1115_remove,
	.id_table	= ads1115_id,

	.detect		= ads1115_detect,
	.address_data	= &addr_data,
};

static int __init ads1115_init(void)
{
	printk(KERN_INFO "init!\n");
	return i2c_add_driver(&ads1115_driver);
}

static void __exit ads1115_exit(void)
{
	printk(KERN_INFO "exit!\n");
	i2c_del_driver(&ads1115_driver);
}


MODULE_AUTHOR("Christoph Mair <christoph.mair@gmail.com");
MODULE_DESCRIPTION("ADS1115 ADC driver");
MODULE_LICENSE("GPL");

module_init(ads1115_init);
module_exit(ads1115_exit);
