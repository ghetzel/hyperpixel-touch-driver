/*
 * Pimoroni Hyperpixel touchscreen (I2C bus, screen AUO A035VL01-V0)
 *
 * Copyright (C) 2017 Gary Hetzel
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define HYPERPIXEL_DRIVER_NAME       "hyperpixel"
#define HYPERPIXEL_MAX_TOUCHES       2
#define HYPERPIXEL_I2C_BUS           3
#define HYPERPIXEL_I2C_ADDR_A035VL01 0x5c
#define HYPERPIXEL_GPIO_BCM          27

struct hyperpixel_dev {
	struct i2c_client *client;
	struct input_dev *input;
};


// -----------------------------------------------------------------------------
static irqreturn_t hyperpixel_touch_irq(int irq, void *irq_data)
{
	struct hyperpixel_dev *hpx = irq_data;
	struct i2c_client *client = hpx->client;
	// struct input_dev *input = hpx->input;
	int error;
	int x1, y1, x2, y2;
	u8 data[8];

	// read touch details from I2C
	error = i2c_smbus_read_i2c_block_data(client, 0x40, 8, data);

	if (error < 0) {
		dev_err(&client->dev, "Data read error %d\n", error);
		return IRQ_HANDLED;
	}

	x1 = data[0] | (data[4] << 8);
	y1 = data[1] | (data[5] << 8);
	x2 = data[2] | (data[6] << 8);
	y2 = data[3] | (data[7] << 8);

	printk(KERN_INFO "HPX: 1=(%d,%d) 2=(%d,%d)", x1, y1, x2, y2);

	// TODO: pickup from https://github.com/pimoroni/hyperpixel/blob/master/requirements/usr/bin/hyperpixel-touch#L234
	//
	// if (x1 && y1) {
	//     input_event(input, EV_ABS, ABS_MT_SLOT, 0);
	//     input_report_abs(input, ABS_X, x1);
	//     input_report_abs(input, ABS_Y, y1);
	//     input_report_abs(ABS_MT_TRACKING_ID, 0);
	//     input_report_abs(ABS_MT_POSITION_X, x1);
	//     input_report_abs(ABS_MT_POSITION_Y, y1);
	//     input_report_(BTN_TOUCH, 1);
	//     input_report_abs(ABS_X, x1);
	//     input_report_abs(ABS_Y, y1);
	// }

	// input_sync(input);

	// if (x2 && y2) {
	//     input_report_abs(input, ABS_MT_POSITION_X, x2);
	//     input_report_abs(input, ABS_MT_POSITION_Y, y2);
	// }

	return IRQ_HANDLED;
}

// -----------------------------------------------------------------------------
static int hyperpixel_touch_open(struct input_dev *dev)
{
	struct hyperpixel_dev *hpx = input_get_drvdata(dev);
	struct i2c_client *client = hpx->client;

	enable_irq(client->irq);

	return 0;
}

static void hyperpixel_touch_close(struct input_dev *dev)
{
	struct hyperpixel_dev *hpx = input_get_drvdata(dev);
	struct i2c_client *client = hpx->client;

	disable_irq(client->irq);
}


// -----------------------------------------------------------------------------
static int hyperpixel_touch_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct hyperpixel_dev *hpx;
	struct device *dev = &client->dev;
	struct input_dev *input;
	int error;

	dev_dbg(dev, "Probing for Pimoroni Hyperpixel Touschreen driver");

	if (client->irq <= 0) {
		// dev_err(dev, "No IRQ!\n");
		// return -EINVAL;
		client->irq = 84;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	// allocate and configure the device as it will appear in /dev/input
	//
	hpx = kzalloc(sizeof(*hpx), GFP_KERNEL);
	input = input_allocate_device();

	if (!hpx || !input) {
		error = -ENOMEM;
		printk(KERN_CRIT "Failed to allocate devices: %d\n", error);
		goto err_free_irq;
	}

	hpx->client = client;
	hpx->input = input;

	input->name = "Pimoroni Hyperpixel Touchscreen Driver";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;
	input->open = hyperpixel_touch_open;
	input->close = hyperpixel_touch_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 480, 0, 0);
	input_mt_init_slots(input, HYPERPIXEL_MAX_TOUCHES, 0);
	input_set_abs_params(input, ABS_MT_SLOT, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 480, 0, 0);

	// associate our struct with the input device
	input_set_drvdata(input, hpx);
	i2c_set_clientdata(client, hpx);

	// setup interrupt handler
	error = request_threaded_irq(client->irq, NULL, hyperpixel_touch_irq,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		client->name, hpx);

	if (error) {
		dev_err(dev,
			"Failed to enable IRQ %d, error: %d\n", client->irq, error);
		goto err_free_mem;
	}

	// disabled IRQ, hyperpixel_touch_open will enable it
	disable_irq(client->irq);

	error = input_register_device(input);

	if (error) {
		dev_err(dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	// GPIO initialization
	// -------------------
	error = gpio_request(HYPERPIXEL_GPIO_BCM, HYPERPIXEL_DRIVER_NAME);

	if (error) {
		dev_err(dev,
			"Failed to request GPIO, error: %d\n", error);
		goto err_free_irq;
	}

	error = gpio_direction_input(HYPERPIXEL_GPIO_BCM);

	if (error) {
		dev_err(dev,
			"Failed to initialize GPIO, error: %d\n", error);
		goto err_free_irq;
	}

	gpio_export(client->addr, false);

	// configure controller interrupts
	//   EN_INT      = 1    enable interrupts
	//   INT_POL     = 1    interrupt is high-active
	//   INT_MODE[1] = 1 \  assert interrupt
	//   INT_MODE[0] = 0 /  on touch
	//
	error = i2c_smbus_write_byte_data(client, 0x6e, 0xe);

	if (error) {
		dev_err(dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	printk(KERN_INFO "Pimoroni Hyperpixel successfully initialized.");
	return 0;

err_free_irq:
	free_irq(client->irq, hpx);

err_free_mem:
	input_free_device(input);
	kfree(hpx);

	return error;
}


// -----------------------------------------------------------------------------
static int hyperpixel_touch_remove(struct i2c_client *client)
{
	struct hyperpixel_dev *hpx = i2c_get_clientdata(client);

	free_irq(client->irq, hpx);
	input_unregister_device(hpx->input);
	kfree(hpx);
	printk(KERN_INFO "Pimoroni Hyperpixel disabled.");

	return 0;
}

static const struct i2c_device_id hyperpixel_touch_ids[] = {
	{HYPERPIXEL_DRIVER_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, hyperpixel_touch_ids);


static struct i2c_driver hyperpixel_touch_driver = {
	.driver = {
		.name   = HYPERPIXEL_DRIVER_NAME,
		// .pm = &hyperpixel_touch_power_management,
	},
	.probe      = hyperpixel_touch_probe,
	.remove     = hyperpixel_touch_remove,
	.id_table   = hyperpixel_touch_ids,
};
module_i2c_driver(hyperpixel_touch_driver);

// static SIMPLE_DEV_PM_OPS(hyperpixel_touch_power_management,
//     hyperpixel_touch_suspend, hyperpixel_touch_resume);

MODULE_AUTHOR("Gary Hetzel <garyhetzel@gmail.com>");
MODULE_DESCRIPTION("Pimoroni Hyperpixel Touchscreen driver");
MODULE_LICENSE("GPL");
