/*
 * PiMoroni Hyperpixel touchscreen (I2C bus, screen AUO A035VL01-V0)
 *
 * Copyright (C) 2017 Gary Hetzel
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/input.h>    /* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/pm.h>

#define HYPERPIXEL_BUS         0x03
#define HYPERPIXEL_ADDR        0x5c

// the example uses this global.  i don't like it, but don't yet know
// enough to not use it
struct hyperpixel_dev {
    struct i2c_client *client;
    struct input_dev *input;
};

static irqreturn_t hyperpixel_touch_irq(int irq, void *dev_id)
{
    int error;
    int x1, y1, x2, y2;
    struct hyperpixel_dev *hpx = dev_id;
    struct input_dev *input = hpx->input;
    u8 *data[8];

    // read touch details from I2C
    error = i2c_smbus_read_i2c_block_data(hpx->client, 0x40, 8, data);

    if (error < 0) {
        dev_err(input, "Data read error %d\n", error);
        return;
    }

    x1 = data[0] | (data[4] << 8);
    y1 = data[1] | (data[5] << 8);
    x2 = data[2] | (data[6] << 8);
    y2 = data[3] | (data[7] << 8);

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

    // if (x2 && y2) {
    //     input_report_abs(input, ABS_MT_POSITION_X, x2);
    //     input_report_abs(input, ABS_MT_POSITION_Y, y2);
    // }

    return IRQ_HANDLED;
}

// Think these are for request_threaded_irq
// -----------------------------------------------------------------------------
// static int hyperpixel_touch_open(struct input_dev *dev)
// {
//     struct hyperpixel_touch *hpx = input_get_drvdata(dev);
//     struct i2c_client *client = hpx->client;

//     enable_irq(client->irq);

//     return 0;
// }

// static void hyperpixel_touch_close(struct input_dev *dev)
// {
//     struct hyperpixel_touch *hpx = input_get_drvdata(dev);
//     struct i2c_client *client = hpx->client;

//     disable_irq(client->irq);
// }
// -----------------------------------------------------------------------------

static int hyperpixel_touch_probe(struct i2c_client *client,
                     const struct i2c_device_id *id)
{
    struct hyperpixel_dev *hpx;
    struct input_dev *input;
    int error;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "i2c_check_functionality error\n");
        return -EIO;
    }

    // allocate and configure the device as it will appear in /dev/input
    //
    hpx = kzalloc(sizeof(*hpx), GFP_KERNEL);
    input = input_allocate_device();

    if (!hpx || !input) {
        error = -ENOMEM;
        goto err_free_irq;
    }

    hpx->client = client;
    hpx->input = input;

    input->name = "PiMoroni Hyperpixel Touchscreen Driver";
    input->id.bustype = BUS_I2C;
    input->id.vendor = 0xFF; // TODO: find out what this ought to be
    input->id.version = 0x01;
    input->dev.parent = &client->dev;

    // this has something to do with using request_threaded_irq
    //
    // input->open = hyperpixel_touch_open;
    // input->close = hyperpixel_touch_close;

    input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

    __set_bit(BTN_TOUCH, input->keybit);

    input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
    input_set_abs_params(input, ABS_Y, 0, 480, 0, 0);
    input_set_abs_params(input, ABS_MT_SLOT, 0, 1, 0, 0);
    input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 65535, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, 800, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 480, 0, 0);

    // associate our struct with the input device
    input_set_drvdata(input, hpx);

    // setup interrupt handler
    error = request_irq(client->irq, hyperpixel_touch_irq, 0, "hyperpixel_touch", hpx);

    if (error) {
        dev_err(&client->dev,
            "Failed to enable IRQ, error: %d\n", error);
        goto err_free_mem;
    }

    // for threaded IRQ
    // disable_irq(client->irq);

    error = input_register_device(input);

    if (error) {
        dev_err(&client->dev,
            "Failed to register input device, error: %d\n", error);
        goto err_free_irq;
    }

    i2c_set_clientdata(client, hpx);
    return 0;

    // from https://github.com/pimoroni/hyperpixel/blob/master/requirements/usr/bin/hyperpixel-touch#L99-L101
    //
    // gpio.setmode(gpio.BCM)
    // gpio.setwarnings(False)
    // gpio.setup(INT, gpio.IN)  // INT = 27

    // configure controller interrupts
    //   EN_INT      = 1    enable interrupts
    //   INT_POL     = 1    interrupt is high-active
    //   INT_MODE[1] = 1 \  assert interrupt
    //   INT_MODE[0] = 0 /  on touch
    //
    i2c_smbus_write_byte_data(client, 0x6e, 0xe);

err_free_irq:
    free_irq(client->irq, hpx);

err_free_mem:
    input_free_device(input);
    kfree(hpx);

    return error;
}

static int hyperpixel_touch_remove(struct i2c_client *client)
{
    struct hyperpixel_dev *hpx = i2c_get_clientdata(client);

    free_irq(client->irq, hpx);
    input_unregister_device(hpx->input);
    kfree(hpx);

    return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id hyperpixel_touch_id[] = {
    { "HPX_TOUCH_A035VL01", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, hyperpixel_touch_id);

static struct i2c_driver hyperpixel_touch_driver = {
    .driver = {
        .name   = "hyperpixel_touch",
        // .pm = &hyperpixel_touch_power_management,
    },

    .probe      = hyperpixel_touch_probe,
    .remove     = hyperpixel_touch_remove,
    .id_table   = hyperpixel_touch_id,
};
module_i2c_driver(hyperpixel_touch_driver);

MODULE_AUTHOR("Gary Hetzel <garyhetzel@gmail.com>");
MODULE_DESCRIPTION("PiMoroni Hyperpixel Touchscreen driver");
MODULE_LICENSE("GPL");
