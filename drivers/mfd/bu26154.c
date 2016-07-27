/*
 * bu26154.c  --  ROHM BU26154 chip multi-function driver
 *  Copyright (C) 2016 SATO Corporation
 *
 * For licencing details see kernel-base/COPYING
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bu26154.h>

#define RESET_DELAY		30

static const struct mfd_cell bu26154s[] = {
	{
		.name = "bu26154-ts",
		.of_compatible = "rohm,bu26154-ts",
	},
	{
		.name = "bu26154-audio",
		.of_compatible = "rohm,bu26154-audio",
	},
};

static int bu26154_init_chip(struct bu26154 *bu26154)
{
	int ret;

	/* Ensure MAPCON register points to MAP0 */
	ret = bu26154_set_map(bu26154, BU26154_MAPCON_MAP0);
	if (ret < 0) {
		dev_err(bu26154->dev, "BU26154_MAPCON reg write failed(MAP0)\n");
		return ret;
	}

	/* Software reset */
	ret = bu26154_reg_write(bu26154, BU26154_RESET_REG, 0x0);
	if (ret < 0) {
		dev_err(bu26154->dev, "BU26154_RESET reg write failed\n");
		return ret;
	}
	msleep(RESET_DELAY);

	return 0;
}

static const struct regmap_config bu26154_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = BU26154_MAX_REGISTER - 1,
};

static int bu26154_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct bu26154 *bu26154;
	int gpio, error;

	bu26154 = devm_kzalloc(&i2c->dev, sizeof(struct bu26154),
			       GFP_KERNEL);
	if (bu26154 == NULL)
		return -ENOMEM;

	gpio = of_get_named_gpio(i2c->dev.of_node, "enable-gpio", 0);
	if (gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(gpio)) {
		error = devm_gpio_request_one(&i2c->dev, gpio,
					      GPIOF_OUT_INIT_HIGH, "codec_pwr_en");
		if (error) {
			dev_err(&i2c->dev, "faild to request gpio %d, ret = %d\n",
				gpio, error);
			return error;
		}
	}

	i2c_set_clientdata(i2c, bu26154);
	bu26154->dev = &i2c->dev;
	bu26154->i2c_client = i2c;
	rt_mutex_init(&bu26154->map_lock);

	bu26154->regmap = devm_regmap_init_i2c(i2c, &bu26154_regmap_config);
	if (IS_ERR(bu26154->regmap)) {
		error = PTR_ERR(bu26154->regmap);
		dev_err(bu26154->dev, "regmap initialization failed(%d)\n", error);
		return error;
	}

	/* configure the BU26154 chip */
	error = bu26154_init_chip(bu26154);
	if (error) {
		dev_err(bu26154->dev, "error in bu26154 config(%d)\n", error);
		return error;
	}

	return mfd_add_devices(bu26154->dev, -1, bu26154s,
			       ARRAY_SIZE(bu26154s), NULL, 0, NULL);
}

static int bu26154_i2c_remove(struct i2c_client *i2c)
{
	struct bu26154 *bu26154 = i2c_get_clientdata(i2c);

	mfd_remove_devices(bu26154->dev);
	return 0;
}

static const struct i2c_device_id bu26154_i2c_id[] = {
       { "bu26154", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, bu26154_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id bu26154_of_match[] = {
	{.compatible = "rohm,bu26154", },
	{},
};
MODULE_DEVICE_TABLE(of, bu26154_of_match);
#endif

static struct i2c_driver bu26154_i2c_driver = {
	.driver = {
		   .name = "bu26154",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(bu26154_of_match),
	},
	.probe = bu26154_i2c_probe,
	.remove = bu26154_i2c_remove,
	.id_table = bu26154_i2c_id,
};

static int __init bu26154_i2c_init(void)
{
	return i2c_add_driver(&bu26154_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(bu26154_i2c_init);

static void __exit bu26154_i2c_exit(void)
{
	i2c_del_driver(&bu26154_i2c_driver);
}
module_exit(bu26154_i2c_exit);

MODULE_DESCRIPTION("ROHM BU26154 multi-function driver");
MODULE_LICENSE("GPL");
