/*
 * Pimoroni Hyperpixel touchscreen (I2C bus, screen AUO A035VL01-V4?)
 *
 * Copyright (C) 2017 Gary Hetzel
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/types.h>

#define POLL_INTERVAL                3
#define HYPERPIXEL_DRIVER_NAME       "hyperpixel"
#define HYPERPIXEL_MAX_TOUCHES       2
#define HYPERPIXEL_I2C_BUS           3
#define HYPERPIXEL_I2C_ADDR_A035VL01 0x5c
#define HYPERPIXEL_GPIO_BCM          27
#define HYPERPIXEL_DEFAULT_WIDTH     800
#define HYPERPIXEL_COLS              13
#define HYPERPIXEL_PIXELS_PER_COL    61
#define HYPERPIXEL_DEFAULT_HEIGHT    480
#define HYPERPIXEL_ROWS              7
#define HYPERPIXEL_PIXELS_PER_ROW    68

struct coords {
	int x1;
	int y1;
	int x2;
	int y2;
};

struct hyperpixel_dev {
	struct i2c_client *client;
	struct input_dev *input;
	struct input_polled_dev *poll;
	struct coords *last;
	int width;
	int height;
};

static void hyperpixel_touch_sync(struct hyperpixel_dev *hpx)
{
	struct i2c_client *client = hpx->client;
	struct input_dev *input = hpx->input;
	int error = 0;
	int col = 0;
	int row = 0;
	int sx1 = 0;
	int sy1 = 0;
	int sx2 = 0;
	int sy2 = 0;
	int x1_line = 0;
	int y1_line = 0;
	int x2_line = 0;
	int y2_line = 0;
	int x1 = 0;
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int c1_down = 0;
	int c2_down = 0;
	int reported = 0;

	u8 data[8];
	s32 sx1_adc, sy1_adc, sx2_adc, sy2_adc;

	// read touch details from I2C
	error = i2c_smbus_read_i2c_block_data(client, 0x40, 8, data);

	if (error < 0) {
		dev_err(&client->dev, "Data read error %d\n", error);
		return;
	}

	// receive raw coordinates from the touchscreen
	sx1 = data[0] | (data[4] << 8);
	sy1 = data[1] | (data[5] << 8);
	sx2 = data[2] | (data[6] << 8);
	sy2 = data[3] | (data[7] << 8);

	// translate coordinates (which come in with (0,0) bottom-right) such
	// that (0,0) is top-left
	//
	// NB: I think this is because the screen is mounted upside down
	//
	x1 = hpx->width - sx1;
	y1 = hpx->height - sy1;
	x2 = hpx->width - sx2;
	y2 = hpx->height - sy2;

	// figure out which ADC channel lines to read from
	for (col = 1; col <= HYPERPIXEL_COLS; col++) {
		int done = 0;
		int threshold = (HYPERPIXEL_PIXELS_PER_COL * col);

		if ( x1 < threshold ) {
			done++;
		} else {
			x1_line++;
		}

		if ( x2 < threshold ) {
			done++;
		} else {
			x2_line++;
		}

		if (done == 2) {
			break;
		}
	}

	for (row = 1; row <= HYPERPIXEL_ROWS; row++) {
		int done = 0;
		int threshold = (HYPERPIXEL_PIXELS_PER_ROW * row);

		if ( y1 < threshold ) {
			done++;
		} else {
			y1_line++;
		}

		if ( y2 < threshold ) {
			done++;
		} else {
			y2_line++;
		}

		if (done == 2) {
			break;
		}
	}

	// read states of the ADC for the coordinates. this tells us if there is
	// a touch registered at that location
	sx1_adc = i2c_smbus_read_word_data(client, x1_line);
	sy1_adc = i2c_smbus_read_word_data(client, y1_line);
	sx2_adc = i2c_smbus_read_word_data(client, x2_line);
	sy2_adc = i2c_smbus_read_word_data(client, y2_line);

	c1_down = (sx1_adc > 100 || sy1_adc > 100);
	c2_down = (sx2_adc > 100 || sy2_adc > 100);

	// First Contact
	// -------------
	if (c1_down) {
		if (sx1 && sy1) {
			// send traditional single-touch (ST) events
			input_report_abs(input, ABS_X, x1);
			input_report_abs(input, ABS_Y, y1);
			reported = 1;

			// send multitouch events
			input_event(input, EV_ABS, ABS_MT_SLOT, 0);
			input_report_abs(input, ABS_MT_TRACKING_ID, 0);
			input_report_abs(input, ABS_MT_POSITION_X, x1);
			input_report_abs(input, ABS_MT_POSITION_Y, y1);
			input_report_key(input, BTN_TOUCH, 1);

		} else if (!hpx->last || x1 != hpx->last->x1 || y1 != hpx->last->y1) {
			if(hpx->last) {
				if(x1 != hpx->last->x1) {
					input_report_abs(input, ABS_X, x1);
					reported = 1;
				}

				if(y1 != hpx->last->y1) {
					input_report_abs(input, ABS_Y, y1);
					reported = 1;
				}

			}

			input_report_abs(input, ABS_MT_POSITION_X, x1);
			input_report_abs(input, ABS_MT_POSITION_Y, y1);
		}

		input_event(input, EV_SYN, SYN_MT_REPORT, 0);
	} else {
		input_event(input, EV_ABS, ABS_MT_SLOT, 0);
		input_report_abs(input, ABS_MT_TRACKING_ID, -1);
		input_report_key(input, BTN_TOUCH, 0);
	}

	// Second Contact
	// --------------
	if (c2_down) {
		if (sx2 && sy2) {
			// send traditional single-touch (ST) events, but only if there
			// wasn't any info for contact one
			if (!sx1 && !sy1) {
				input_report_abs(input, ABS_X, x2);
				input_report_abs(input, ABS_Y, y2);
				reported = 1;
			}

			// send multitouch events
			input_event(input, EV_ABS, ABS_MT_SLOT, 1);
			input_report_abs(input, ABS_MT_TRACKING_ID, 1);
			input_report_abs(input, ABS_MT_POSITION_X, x2);
			input_report_abs(input, ABS_MT_POSITION_Y, y2);
			input_report_key(input, BTN_TOUCH, 1);

		} else if (!hpx->last || x2 != hpx->last->x2 || y2 != hpx->last->y2) {
			if(hpx->last) {
				if(x2 != hpx->last->x2) {
					input_report_abs(input, ABS_X, x2);
					reported = 1;
				}

				if(y2 != hpx->last->y2) {
					input_report_abs(input, ABS_Y, y2);
					reported = 1;
				}

			}

			input_report_abs(input, ABS_MT_POSITION_X, x2);
			input_report_abs(input, ABS_MT_POSITION_Y, y2);
		}

		input_event(input, EV_SYN, SYN_MT_REPORT, 0);
	} else {
		input_event(input, EV_ABS, ABS_MT_SLOT, 1);
		input_report_abs(input, ABS_MT_TRACKING_ID, -1);
		input_report_key(input, BTN_TOUCH, 0);
	}

	input_event(input, EV_SYN, SYN_REPORT, 0);
	input_sync(input);

	if (reported) {
		// printk(KERN_INFO "HPX: [1]=(%d,%d) [2]=(%d,%d)", x1, y1, x2, y2);
		hpx->last = &(struct coords){ x1, y1, x2, y2 };
	}

	return;
}

// -----------------------------------------------------------------------------
// static irqreturn_t hyperpixel_touch_hardirq(int irq, void *irq_data)
// {
// 	struct hyperpixel_dev *hpx = irq_data;

// 	if(hpx) {
// 		printk(KERN_INFO "HPX: i understood that reference\n");
// 		return IRQ_HANDLED;
// 	} else {
// 		printk(KERN_INFO "HPX: NOPE\n");
// 		return IRQ_NONE;
// 	}
// }

// -----------------------------------------------------------------------------
static irqreturn_t hyperpixel_touch_irq(int irq, void *irq_data)
{
	struct hyperpixel_dev *hpx = irq_data;

	if(hpx) {
		hyperpixel_touch_sync(hpx);
	}

	return IRQ_HANDLED;
}

static void hyperpixel_touch_poll(struct input_polled_dev *dev)
{
	struct hyperpixel_dev *hpx = dev->private;

	if(hpx) {
		hyperpixel_touch_sync(hpx);
	}
}

// -----------------------------------------------------------------------------
// static int hyperpixel_touch_open(struct input_dev *dev)
// {
// 	struct hyperpixel_dev *hpx = input_get_drvdata(dev);
// 	struct i2c_client *client = hpx->client;

// 	enable_irq(client->irq);

// 	return 0;
// }

// static void hyperpixel_touch_close(struct input_dev *dev)
// {
// 	struct hyperpixel_dev *hpx = input_get_drvdata(dev);
// 	struct i2c_client *client = hpx->client;

// 	disable_irq(client->irq);
// }


// -----------------------------------------------------------------------------
static int hyperpixel_touch_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct hyperpixel_dev *hpx;
	struct input_polled_dev *poll;
	struct input_dev *input;
	struct device *dev = &client->dev;
	int error;

	dev_dbg(dev, "Probing for Pimoroni Hyperpixel Touschreen driver");

	if (client->irq <= 0) {
		// dev_err(dev, "No IRQ!\n");
		// return -EINVAL;
		client->irq = 79;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	// allocate and configure the device as it will appear in /dev/input
	//
	hpx = kzalloc(sizeof(*hpx), GFP_KERNEL);

	hpx->width = HYPERPIXEL_DEFAULT_WIDTH;
	hpx->height = HYPERPIXEL_DEFAULT_HEIGHT;

	if (!hpx) {
		error = -ENOMEM;
		printk(KERN_CRIT "Failed to allocate devices: %d\n", error);
		goto err_free_irq;
	}

	// setup polled or IRQ-driven input
	if(POLL_INTERVAL) {
		poll = input_allocate_polled_device();

		if (!poll) {
			error = -ENOMEM;
			printk(KERN_CRIT "Failed to allocate polled input: %d\n", error);
			goto err_free_irq;
		}

		hpx->poll = poll;
		input = poll->input;

		hpx->poll->private = hpx;
		hpx->poll->poll_interval = POLL_INTERVAL;
		hpx->poll->poll = hyperpixel_touch_poll;

		printk(KERN_INFO "Touchscreen polling on %dms interval\n", hpx->poll->poll_interval);
	} else {
		input = input_allocate_device();

		if (!input) {
			error = -ENOMEM;
			printk(KERN_CRIT "Failed to allocate input: %d\n", error);
			goto err_free_irq;
		}

		printk(KERN_INFO "Touchscreen configured for interrupt-driven input\n");
	}

	hpx->client = client;
	hpx->input = input;

	input->name = "Pimoroni Hyperpixel Touchscreen Driver";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;
	// input->open = hyperpixel_touch_open;
	// input->close = hyperpixel_touch_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, hpx->width, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, hpx->height, 0, 0);
	input_mt_init_slots(input, HYPERPIXEL_MAX_TOUCHES, 0);
	input_set_abs_params(input, ABS_MT_SLOT, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, hpx->width, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, hpx->height, 0, 0);

	// associate our struct with the input device
	input_set_drvdata(input, hpx);
	i2c_set_clientdata(client, hpx);

	// setup interrupt handler
	// error = request_threaded_irq(client->irq, hyperpixel_touch_hardirq,
	// 	hyperpixel_touch_irq,
	// 	IRQF_SHARED,
	// 	client->name, hpx);

	error = request_irq(client->irq, hyperpixel_touch_irq,
		IRQF_SHARED,
		client->name, hpx);

	if (error) {
		dev_err(dev,
			"Failed to enable IRQ %d, error: %d\n", client->irq, error);
		goto err_free_mem;
	}

	// register input devices
	if(hpx->poll) {
		error = input_register_polled_device(hpx->poll);
	} else {
		// disabled IRQ, hyperpixel_touch_open will enable it
		// disable_irq(client->irq);

		error = input_register_device(hpx->input);
	}

	if (error) {
		dev_err(dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_input;
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

err_free_input:
	if (hpx->poll) {
		input_free_polled_device(hpx->poll);
	} else {
		input_free_device(hpx->input);
	}

err_free_irq:
	free_irq(client->irq, hpx);
	gpio_free(HYPERPIXEL_GPIO_BCM);

err_free_mem:
	kfree(hpx);

	return error;
}


// -----------------------------------------------------------------------------
static int hyperpixel_touch_remove(struct i2c_client *client)
{
	struct hyperpixel_dev *hpx = i2c_get_clientdata(client);

	if(hpx) {
		free_irq(client->irq, hpx);

		if (hpx->poll) {
			input_unregister_polled_device(hpx->poll);
		} else {
			input_unregister_device(hpx->input);
		}

		kfree(hpx);
		gpio_free(HYPERPIXEL_GPIO_BCM);
		printk(KERN_INFO "Pimoroni Hyperpixel disabled.");
	}

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
