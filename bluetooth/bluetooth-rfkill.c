/* Copyright (C) 2020 Synaptics.                                       */
/* This software is licensed under the terms of the GNU General Public */
/* License version 2, as published by the Free Software Foundation, and*/
/* may be copied, distributed, and modified under those terms.         */
/*                                                                     */
/* This program is distributed in the hope that it will be useful,     */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of      */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the        */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/slab.h>


#define BT_PWR_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n", __func__, ## arg)
#define BT_PWR_INFO(fmt, arg...) pr_info("%s: " fmt "\n", __func__, ## arg)
#define BT_PWR_ERR(fmt, arg...)  pr_err("%s: " fmt "\n", __func__, ## arg)

struct bluetooth_plat_data {
	struct gpio_desc *power_gpio;
	struct rfkill *rfkill;
};

static int bluetooth_set_power(void *data, bool blocked)
{
	struct bluetooth_plat_data *pdata = data;

	if (!blocked) {
		gpiod_set_value(pdata->power_gpio, 0);
		mdelay(10);
		BT_PWR_INFO("%s: power up = %d\n", __func__, blocked);
		gpiod_set_value(pdata->power_gpio, 1);
		mdelay(150);
	} else {
		gpiod_set_value(pdata->power_gpio, 0);
		BT_PWR_INFO("%s: power down = %d\n", __func__, blocked);
		mdelay(10);
	}

	BT_PWR_INFO("%s: onoff = %d\n", __func__, blocked);

	return 0;
}

static struct rfkill_ops rfkill_bluetooth_ops = {
	.set_block = bluetooth_set_power,
};

static int rfkill_bluetooth_probe(struct platform_device *pdev)
{
	int ret;
	bool default_state = true;
	struct bluetooth_plat_data *pdata;

	BT_PWR_INFO("%s\n", __func__);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->power_gpio = devm_gpiod_get(&pdev->dev, "bt-power", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->power_gpio)) {
		dev_err(&pdev->dev, "cannot get bt-power gpio\n");
		return PTR_ERR(pdata->power_gpio);
	}

	BT_PWR_INFO("[BT] bt power gpio acquired");

	pdata->rfkill = rfkill_alloc("bt_power", &pdev->dev,
				     RFKILL_TYPE_BLUETOOTH,
				     &rfkill_bluetooth_ops, pdata);

	rfkill_init_sw_state(pdata->rfkill, 0);

	ret = rfkill_register(pdata->rfkill);
	if (ret) {
		rfkill_destroy(pdata->rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, pdata);
	bluetooth_set_power(pdata, default_state);

	return 0;
}

static void rfkill_bluetooth_remove(struct platform_device *pdev)
{
	struct bluetooth_plat_data *pdata = platform_get_drvdata(pdev);

	BT_PWR_INFO("%s\n", __func__);

	rfkill_unregister(pdata->rfkill);
	rfkill_destroy(pdata->rfkill);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id rfkill_of_match[] = {
	{.compatible = "syna,rfkill"},
	{ }
};
MODULE_DEVICE_TABLE(of, rfkill_of_match);

static struct platform_driver rfkill_bluetooth_driver = {
	.probe  = rfkill_bluetooth_probe,
	.remove = rfkill_bluetooth_remove,
	.driver = {
		.name = "bluetooth-rfkill",
		.of_match_table = rfkill_of_match,
	},
};
module_platform_driver(rfkill_bluetooth_driver);
MODULE_DESCRIPTION("synaptics bluetooth rfkill driver");
MODULE_LICENSE("GPL v2");
