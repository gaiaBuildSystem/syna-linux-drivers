/*
 * This file is part of DSPG Technologies' Guardian driver.
 *
 * The Guardian driver is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 *
 * The Guardian driver is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Guardian driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>

struct piog_data {
	struct device		*dev;
	struct gpio_desc	*protectee;
	bool			high;
	bool			low;
	bool			dir;
};

static int
piog_parse_dt(struct piog_data *data)
{
	struct device_node *pnode = data->dev->of_node;

	data->high = of_property_read_bool(pnode, "high");
	data->low  = of_property_read_bool(pnode, "low");
	data->dir  = of_property_read_bool(pnode, "out");

	data->protectee = devm_gpiod_get(data->dev, "protectee", GPIOD_ASIS);
	if (IS_ERR(data->protectee)) {
		dev_err(data->dev, "could not parse 'protectee' property\n");
		return PTR_ERR(data->protectee);
	}

	return 0;
}

static int
piog_probe(struct platform_device *pdev)
{
	struct piog_data *data = pdev->dev.platform_data;
	int ret;

	dev_info(&pdev->dev, "probing DSPG PIO Guardian");

	if (!data) {
		data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
		if (!data) {
			dev_err(&pdev->dev, "no memory for device data\n");
			return -ENOMEM;
		}
	}

	data->dev = &pdev->dev;
	ret = piog_parse_dt(data);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not parse device tree\n");
		return ret;
	}

	if (data->dir)
		ret = gpiod_direction_output(data->protectee, data->high ? 1 : 0);
	else
		ret = gpiod_direction_input(data->protectee);

	if (ret) {
		dev_err(data->dev, "failed to configure protected gpio\n");
		return ret;
	}

	return 0;
}

static int
piog_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id piog_of_match[] = {
	{ .compatible = "dspg,pio-guardian", },
	{ /* guardian */ },
};
MODULE_DEVICE_TABLE(of, piog_of_match);

static struct platform_driver piog_driver = {
	.driver = {
		.name		= "dspg-pio-guardian",
		.owner		= THIS_MODULE,
		.pm		= NULL,
		.of_match_table	= piog_of_match,
	},
	.probe	= piog_probe,
	.remove	= piog_remove,
};

static int __init piog_init(void)
{
	return platform_driver_register(&piog_driver);
}

static void __exit piog_exit(void)
{
	platform_driver_unregister(&piog_driver);
}

module_init(piog_init);
module_exit(piog_exit);

MODULE_DESCRIPTION("DSPG PIO Guardian");
MODULE_LICENSE("GPL v2");
