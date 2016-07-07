/*
 * Copyright (C) 2016 SATO Corporation
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/bu26154.h>
#include <linux/input/bu26154-ts.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define PEN_DOWN_INTR		0
#define PEN_UP_DEFAULT		msecs_to_jiffies(30)
#define PERIOD_TIME_DEFAULT	2
#define MAX_12BIT		((1 << 12) - 1)

#define TADC_CR_INIT		(BU26154_TADC_CR_TCHA1 | \
				 BU26154_TADC_CR_TCHA0 | \
				 BIT(3)) /* bit-3 must be set */

#define TADC_CONV_X		0x0
#define TADC_CONV_Y		BU26154_TADC_CR_TCHA0
#define TADC_CONV_Z1		BU26154_TADC_CR_TCHA1
#define TADC_CONV_Z2		(BU26154_TADC_CR_TCHA1 | BU26154_TADC_CR_TCHA0)

#define BU26154_NAME		"rohm_bu26154"

struct bu26154_ts_data {
	struct bu26154 *mfd;
	wait_queue_head_t wait;
	const struct bu26154_platform_device *chip;
	struct input_dev *in_dev;
	struct regulator *regulator;
	unsigned int irq;
	unsigned int intr_pin;
	struct timer_list timer;
	bool touch_stopped;
	bool low_power_mode;
};

static inline int bu26154_ts_chip_init(struct bu26154_ts_data *data)
{
	struct bu26154 *bu26154 = data->mfd;
	struct device *dev = &data->in_dev->dev;
	int ret;

	/* Ensure MAPCON register points to MAP0 */
	ret = bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	if (ret < 0) {
		dev_err(dev, "BU26154_MAPCON reg write failed(MAP0)\n");
		return ret;
	}

	/* Enable Clock */
	ret = bu26154_reg_set_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_TCLKEN);
	if (ret < 0) {
		dev_err(dev, "BU26154_CLKEN reg write failed\n");
		goto out;
	}

	/* Change MAPCON register to MAP1 */
	ret = bu26154_set_map(bu26154, BU26154_MAPCON_MAP1);
	if (ret < 0) {
		dev_err(dev, "BU26154_MAPCON reg write failed(MAP1)\n");
		goto out;
	}

	/* Initialize Touch ADC Control Register */
	ret = bu26154_reg_write(bu26154, BU26154_TS_ADCCR_REG, TADC_CR_INIT);
	if (ret < 0) {
		dev_err(dev, "BU26154_TS_ADCCR reg write failed\n");
		goto out;
	}

out:
	/* Return back to MAP0 */
	bu26154_set_map_unlock(bu26154, BU26154_MAPCON_MAP0);
	return ret;
}

static inline int bu26154_ts_update_power_state(struct bu26154_ts_data *data)
{
	struct bu26154 *bu26154 = data->mfd;
	struct device *dev = &data->in_dev->dev;
	int ret;

	/* Ensure MAPCON register points to MAP0 */
	ret = bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	if (ret < 0) {
		dev_err(dev, "BU26154_MAPCON reg write failed(MAP0)\n");
		return ret;
	}

	if (!data->low_power_mode) {
		bu26154_reg_set_bits(bu26154, BU26154_CLKEN_REG,
				     BU26154_CLKEN_TCLKEN);
	} else {
		bu26154_reg_clear_bits(bu26154, BU26154_CLKEN_REG,
				       BU26154_CLKEN_TCLKEN);
	}
	bu26154_map_unlock(bu26154);

	return 0;
}

static int bu26154_ts_enter_low_power(struct bu26154_ts_data *data)
{
	if (data->low_power_mode)
		return 0;

	data->low_power_mode = true;
	return bu26154_ts_update_power_state(data);
}

static int bu26154_ts_leave_low_power(struct bu26154_ts_data *data)
{
	if (!data->low_power_mode)
		return 0;

	data->low_power_mode = false;
	return bu26154_ts_update_power_state(data);
}

static int bu26154_read_coordinates(struct bu26154_ts_data *data, unsigned int *buf)
{
	struct bu26154 *bu26154 = data->mfd;
	struct device *dev = &data->in_dev->dev;
	int i, error;

	/* Change MAPCON to MAP1 */
	error = bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP1);
	if (error < 0) {
		dev_err(dev, "%s: BU26154_MAPCON reg write failed(MAP1)\n",
			     __func__);
		return error;
	}

	/* Read XYZ coordinates */
	for (i = 0; i < 4; i++) {
		u8 ad[2];
		u8 val = BU26154_TADC_CR_TCHEN;

		if (i == 0)
			val |= TADC_CONV_X;
		else if (i == 1)
			val |= TADC_CONV_Y;
		else if (i == 2)
			val |= TADC_CONV_Z1;
		else
			val |= TADC_CONV_Z2;

		bu26154_reg_update_bits(bu26154, BU26154_TS_ADCCR_REG,
					(BU26154_TADC_CR_TCHEN |
					 BU26154_TADC_CR_TCHA1 |
					 BU26154_TADC_CR_TCHA0), val);

		udelay(43); /* wait for ADC conversion */
		bu26154_block_read(bu26154, BU26154_TS_TOUTCHAD1, 2, ad);
		buf[i] = (ad[0] << 4) | (ad[1] >> 4);
	}

	/* Restore TADC Control Register to the initial value */
	bu26154_reg_write(bu26154, BU26154_TS_ADCCR_REG, TADC_CR_INIT);
	/* Return back to MAP0 */
	bu26154_set_map_unlock(bu26154, BU26154_MAPCON_MAP0);
	return 0;
}

static inline void bu26154_ts_event_release(struct bu26154_ts_data *data)
{
	input_report_abs(data->in_dev, ABS_PRESSURE, 0);
	input_report_key(data->in_dev, BTN_TOUCH, 0);
	input_sync(data->in_dev);
}

static void bu26154_release_timer(unsigned long handle)
{
	struct bu26154_ts_data *data = (void *)handle;
	bu26154_ts_event_release(data);
}

static int bu26154_do_touch_report(struct bu26154_ts_data *data)
{
	unsigned int x, y, z1, z2, z, buf[4];

	if (bu26154_read_coordinates(data, buf))
		return -EINVAL;

	x = buf[0] & MAX_12BIT;
	y = buf[1] & MAX_12BIT;
	z1 = buf[2] & MAX_12BIT;
	z2 = buf[3] & MAX_12BIT;

	if (likely(x && z1)) {
		/*
		 * Calculate pressure:
		 *   Resistance(touch) = x plate resistance *
		 *   x postion/4096 * ((z2 / z1) - 1)
		 */
		z = (z2 - z1) * x * data->chip->x_plate_ohms;
		z /= z1;
		z = (z + 2047) >> 12;

		if (z < data->chip->pressure_min ||
				z > data->chip->pressure_max) {
			mod_timer(&data->timer, jiffies + data->chip->penup_timeout);
			return 0;
		}
		del_timer(&data->timer);

		if (data->chip->x_flip)
			x = data->chip->x_max - (x - data->chip->x_min);
		if (data->chip->y_flip)
			y = data->chip->y_max - (y - data->chip->y_min);

		if (x < data->chip->x_min || x > data->chip->x_max ||
				y < data->chip->y_min || y > data->chip->y_max) {
			return 0;
		}

		input_report_abs(data->in_dev, ABS_X, x);
		input_report_abs(data->in_dev, ABS_Y, y);
		input_report_abs(data->in_dev, ABS_PRESSURE, z);
		input_report_key(data->in_dev, BTN_TOUCH, 1);
		input_sync(data->in_dev);
	}

	return 0;
}

static irqreturn_t bu26154_gpio_irq(int irq, void *device_data)
{
	struct bu26154_ts_data *data = device_data;
	struct device *dev = &data->in_dev->dev;
	int retval;

	do {
		retval = bu26154_do_touch_report(data);
		if (retval < 0) {
			dev_err(dev, "bu26154_do_touch_report failed\n");
			return IRQ_NONE;
		}

		data->intr_pin = gpio_get_value(data->chip->touch_pin);
		if (data->intr_pin == PEN_DOWN_INTR) {
			wait_event_timeout(data->wait, data->touch_stopped,
					   msecs_to_jiffies(data->chip->period_time));
		} else {
			mod_timer(&data->timer, jiffies);
		}
	} while (!data->intr_pin && !data->touch_stopped);

	return IRQ_HANDLED;
}

static void bu26154_free_irq(struct bu26154_ts_data *bu26154_data)
{
	bu26154_data->touch_stopped = true;
	wake_up(&bu26154_data->wait);
	free_irq(bu26154_data->irq, bu26154_data);
}

#ifdef CONFIG_OF
static const struct bu26154_platform_device *
bu26154_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct bu26154_platform_device *pdata;

	if (!np) {
		dev_err(dev, "no device tree or platform data\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->y_flip = pdata->x_flip = false;

	pdata->x_flip = of_property_read_bool(np, "rohm,flip-x");
	pdata->y_flip = of_property_read_bool(np, "rohm,flip-y");

	of_property_read_u32(np, "rohm,x-min", &pdata->x_min);
	of_property_read_u32(np, "rohm,y-min", &pdata->y_min);
	of_property_read_u32(np, "rohm,pressure-min", &pdata->pressure_min);
	of_property_read_u32(np, "rohm,absfuzz", &pdata->absfuzz);
	of_property_read_u32(np, "rohm,absflat", &pdata->absflat);

	if (of_property_read_u32(np, "rohm,x-max", &pdata->x_max))
		pdata->x_max = MAX_12BIT;

	if (of_property_read_u32(np, "rohm,y-max", &pdata->y_max))
		pdata->y_max = MAX_12BIT;

	if (of_property_read_u32(np, "rohm,pressure-max", &pdata->pressure_max))
		pdata->pressure_max = MAX_12BIT;

	if (of_property_read_u32(np, "rohm,event-period-time", &pdata->period_time))
		pdata->period_time = PERIOD_TIME_DEFAULT;

	if (of_property_read_u32(np, "rohm,penup-timeout", &pdata->penup_timeout))
		pdata->penup_timeout = PEN_UP_DEFAULT;

	if (of_property_read_u32(np, "rohm,x-plate-ohms", &pdata->x_plate_ohms)) {
		dev_warn(dev, "%s has not 'x-plate-ohms' property\n", np->full_name);
		pdata->x_plate_ohms = 450;
	}

	pdata->touch_pin = of_get_named_gpio(np, "touch-gpio", 0);
	pdata->wakeup = of_property_read_bool(np, "wakeup-source");
	pdata->ext_clk = false;

	return pdata;
}
#else
static const struct bu26154_platform_device *
bu26154_parse_dt(struct device *dev)
{
	dev_err(dev, "no platform data available\n");
	return ERR_PTR(-EINVAL);
}
#endif

static int bu26154_ts_probe(struct platform_device *pdev)
{
	struct bu26154 *bu26154 = dev_get_drvdata(pdev->dev.parent);
	const struct bu26154_platform_device *pdata;
	struct bu26154_ts_data *bu26154_data;
	struct input_dev *in_dev;
	int error;

	pdata = dev_get_platdata(bu26154->dev);
	if (!pdata) {
		pdata = bu26154_parse_dt(&pdev->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!gpio_is_valid(pdata->touch_pin)) {
		dev_err(&pdev->dev, "invalid touch_pin supplied\n");
		return -EINVAL;
	}

	bu26154_data = devm_kzalloc(&pdev->dev, sizeof(struct bu26154_ts_data),
				    GFP_KERNEL);

	in_dev = input_allocate_device();
	if (!bu26154_data || !in_dev) {
		dev_err(&pdev->dev, "device memory alloc failed\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	bu26154_data->in_dev = in_dev;
	bu26154_data->chip = pdata;
	bu26154_data->mfd = bu26154;
	bu26154_data->irq = gpio_to_irq(pdata->touch_pin);

	bu26154_data->regulator = regulator_get(&pdev->dev, "tvdd");
	if (IS_ERR(bu26154_data->regulator)) {
		dev_err(&pdev->dev, "regulator_get failed\n");
		error = PTR_ERR(bu26154_data->regulator);
		goto err_free_mem;
	}

	error = regulator_enable(bu26154_data->regulator);
	if (error < 0) {
		dev_err(&pdev->dev, "regulator enable failed\n");
		goto err_put_regulator;
	}

	bu26154_data->touch_stopped = false;
	init_waitqueue_head(&bu26154_data->wait);
	setup_timer(&bu26154_data->timer, bu26154_release_timer,
		    (unsigned long)bu26154_data);

	error = bu26154_ts_chip_init(bu26154_data);
	if (error < 0) {
		dev_err(&pdev->dev, "ts chip init failed\n");
		goto err_disable_regulator;
	}

	/* register the device to input subsystem */
	in_dev->name = BU26154_NAME;
	in_dev->id.bustype = BUS_I2C;
	in_dev->dev.parent = &pdev->dev;

	__set_bit(EV_SYN, in_dev->evbit);
	__set_bit(EV_KEY, in_dev->evbit);
	__set_bit(EV_ABS, in_dev->evbit);
	__set_bit(ABS_X, in_dev->absbit);
	__set_bit(ABS_Y, in_dev->absbit);
	__set_bit(ABS_PRESSURE, in_dev->absbit);
	__set_bit(BTN_TOUCH, in_dev->keybit);

	input_set_capability(in_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(in_dev, ABS_X, pdata->x_min, pdata->x_max,
			     pdata->absfuzz, pdata->absflat);
	input_set_abs_params(in_dev, ABS_Y, pdata->y_min, pdata->y_max,
			     pdata->absfuzz, pdata->absflat);
	input_set_abs_params(in_dev, ABS_PRESSURE, pdata->pressure_min,
			     pdata->pressure_max, 0, 0);

	input_set_drvdata(in_dev, bu26154_data);

	error = request_threaded_irq(bu26154_data->irq, NULL, bu26154_gpio_irq,
				     IRQF_TRIGGER_FALLING | IRQF_SHARED | IRQF_ONESHOT,
				     BU26154_NAME, bu26154_data);
	if (error) {
		dev_err(&pdev->dev, "request irq %d failed\n",
			bu26154_data->irq);
		goto err_disable_regulator;
	}

	error = input_register_device(in_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_free_irq;
	}

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	platform_set_drvdata(pdev, bu26154_data);
	return 0;

err_free_irq:
	bu26154_free_irq(bu26154_data);
err_disable_regulator:
	regulator_disable(bu26154_data->regulator);
err_put_regulator:
	regulator_put(bu26154_data->regulator);
err_free_mem:
	input_free_device(in_dev);

	return error;
}

static int bu26154_ts_remove(struct platform_device *pdev)
{
	struct bu26154_ts_data *bu26154_data = platform_get_drvdata(pdev);

	bu26154_free_irq(bu26154_data);

	input_unregister_device(bu26154_data->in_dev);

	regulator_disable(bu26154_data->regulator);
	regulator_put(bu26154_data->regulator);

	device_init_wakeup(&pdev->dev, false);

	return 0;
}

#ifdef CONFIG_PM
static int bu26154_ts_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bu26154_ts_data *bu26154_data = platform_get_drvdata(pdev);

	bu26154_data->touch_stopped = true;
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(bu26154_data->irq);

	bu26154_ts_enter_low_power(bu26154_data);
	return 0;
}

static int bu26154_ts_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bu26154_ts_data *bu26154_data = platform_get_drvdata(pdev);

	bu26154_ts_leave_low_power(bu26154_data);

	bu26154_data->touch_stopped = false;
	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(bu26154_data->irq);
	return 0;
}
#endif

static const struct dev_pm_ops bu26154_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bu26154_ts_suspend, bu26154_ts_resume)
};

#ifdef CONFIG_OF
static const struct of_device_id bu26154_ts_ids[] = {
	{ .compatible = "rohm,bu26154-ts", },
	{ },
};
#endif

static struct platform_driver bu26154_ts_driver = {
	.driver	= {
		.name	=	BU26154_NAME,
		.owner	=	THIS_MODULE,
		.of_match_table = of_match_ptr(bu26154_ts_ids),
		.pm	=	&bu26154_dev_pm_ops,
	},
	.probe		=	bu26154_ts_probe,
	.remove		=	bu26154_ts_remove,
};
module_platform_driver(bu26154_ts_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("bu26154 touch screen controller driver");
