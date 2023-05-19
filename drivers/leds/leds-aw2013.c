// SPDX-License-Identifier: GPL-2.0+
// Driver for Awinic AW2013 3-channel LED driver

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define AW2013_MAX_LEDS 3

/* Reset and ID register */
#define AW2013_RSTR 0x00
#define AW2013_RSTR_RESET 0x55
#define AW2013_RSTR_CHIP_ID 0x33

/* Global control register */
#define AW2013_GCR 0x01
#define AW2013_GCR_ENABLE BIT(0)

/* LED channel enable register */
#define AW2013_LCTR 0x30
#define AW2013_LCTR_LE(x) BIT((x))

/* LED channel control registers */
#define AW2013_LCFG(x) (0x31 + (x))
#define AW2013_LCFG_IMAX_MASK (BIT(0) | BIT(1)) // Should be 0-3
#define AW2013_LCFG_MD BIT(4)
#define AW2013_LCFG_FI BIT(5)
#define AW2013_LCFG_FO BIT(6)

/* LED channel PWM registers */
#define AW2013_REG_PWM(x) (0x34 + (x))

/* LED channel timing registers */
#define AW2013_LEDT0(x) (0x37 + (x) * 3)
#define AW2013_LEDT0_T1(x) ((x) << 4) // Should be 0-7
#define AW2013_LEDT0_T2(x) (x) // Should be 0-5

#define AW2013_LEDT1(x) (0x38 + (x) * 3)
#define AW2013_LEDT1_T3(x) ((x) << 4) // Should be 0-7
#define AW2013_LEDT1_T4(x) (x) // Should be 0-7

#define AW2013_LEDT2(x) (0x39 + (x) * 3)
#define AW2013_LEDT2_T0(x) ((x) << 4) // Should be 0-8
#define AW2013_LEDT2_REPEAT(x) (x) // Should be 0-15

#define AW2013_REG_MAX 0x77

#define AW2013_TIME_STEP 130 /* ms */

struct aw2013;

struct aw2013_led {
	struct aw2013 *chip;
	struct led_classdev cdev;
	u32 num;
	u64 on_ms;
	u64 off_ms;
	unsigned int imax;
};

struct aw2013 {
	struct mutex mutex; /* held when writing to registers */
	struct regulator *vcc_regulator;
	struct i2c_client *client;
	struct aw2013_led leds[AW2013_MAX_LEDS];
	struct regmap *regmap;
	struct work_struct blink_work;
	int num_leds;
	bool enabled;
	bool blinking;
};

static int aw2013_chip_init(struct aw2013 *chip)
{
	int i, ret;

	ret = regmap_write(chip->regmap, AW2013_GCR, AW2013_GCR_ENABLE);
	if (ret) {
		dev_err(&chip->client->dev, "Failed to enable the chip: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < chip->num_leds; i++) {
		ret = regmap_update_bits(chip->regmap,
					 AW2013_LCFG(chip->leds[i].num),
					 AW2013_LCFG_IMAX_MASK,
					 chip->leds[i].imax);
		if (ret) {
			dev_err(&chip->client->dev,
				"Failed to set maximum current for led %d: %d\n",
				chip->leds[i].num, ret);
			return ret;
		}
	}

	return ret;
}

static void aw2013_chip_disable(struct aw2013 *chip)
{
	int ret;

	if (!chip->enabled)
		return;

	regmap_write(chip->regmap, AW2013_GCR, 0);

	ret = regulator_disable(chip->vcc_regulator);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to disable regulator: %d\n", ret);
		return;
	}

	chip->enabled = false;
}

static int aw2013_chip_enable(struct aw2013 *chip)
{
	int ret;

	if (chip->enabled)
		return 0;

	ret = regulator_enable(chip->vcc_regulator);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to enable regulator: %d\n", ret);
		return ret;
	}
	chip->enabled = true;

	ret = aw2013_chip_init(chip);
	if (ret)
		aw2013_chip_disable(chip);

	return ret;
}

static bool aw2013_chip_in_use(struct aw2013 *chip)
{
	int i;

	for (i = 0; i < chip->num_leds; i++)
		if (chip->leds[i].cdev.brightness)
			return true;

	return false;
}

static int aw2013_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
	int ret, num;

	mutex_lock(&led->chip->mutex);

	if (aw2013_chip_in_use(led->chip)) {
		ret = aw2013_chip_enable(led->chip);
		if (ret)
			goto error;
	}

	num = led->num;

	ret = regmap_write(led->chip->regmap, AW2013_REG_PWM(num), brightness);
	if (ret)
		goto error;

	if (brightness) {
		ret = regmap_update_bits(led->chip->regmap, AW2013_LCTR,
					 AW2013_LCTR_LE(num), 0xFF);
	} else {
		ret = regmap_update_bits(led->chip->regmap, AW2013_LCTR,
					 AW2013_LCTR_LE(num), 0);
		if (ret)
			goto error;
		ret = regmap_update_bits(led->chip->regmap, AW2013_LCFG(num),
					 AW2013_LCFG_MD, 0);
	}
	if (ret)
		goto error;

	if (!aw2013_chip_in_use(led->chip))
		aw2013_chip_disable(led->chip);

error:
	mutex_unlock(&led->chip->mutex);

	return ret;
}

static void aw2013_blink_work(struct work_struct *work)
{
	struct aw2013 *chip =
		container_of(work, struct aw2013, blink_work);
	struct i2c_client *client = chip->client;
	struct device *dev = &client->dev;
	struct aw2013_led *led = container_of(
		dev_get_drvdata(dev), struct aw2013_led, cdev);
	int ret, num = led->num;
	unsigned long off = 0, on = 0;

	if (!led->off_ms && !led->on_ms) {
		led->off_ms = led->cdev.blink_delay_off;
		led->on_ms = led->cdev.blink_delay_on;
	}

	mutex_lock(&chip->mutex);

	/* Convert into values the HW will understand. */
	off = min(5, ilog2((led->off_ms - 1) / AW2013_TIME_STEP) + 1);
	on = min(7, ilog2((led->on_ms - 1) / AW2013_TIME_STEP) + 1);

	led->cdev.blink_delay_off = BIT(off) * AW2013_TIME_STEP;
	led->cdev.blink_delay_on = BIT(on) * AW2013_TIME_STEP;

	/* Set timings */
	ret = regmap_write(chip->regmap,
			   AW2013_LEDT0(num), AW2013_LEDT0_T2(on));
	if (ret) {
		mutex_unlock(&chip->mutex);
		return;
	}
	ret = regmap_write(chip->regmap,
			   AW2013_LEDT1(num), AW2013_LEDT1_T4(off));
	if (ret) {
		mutex_unlock(&chip->mutex);
		return;
	}

	/* Finally, enable the LED */
	ret = regmap_update_bits(chip->regmap, AW2013_LCFG(num),
				 AW2013_LCFG_MD, 0xFF);
	if (ret) {
		mutex_unlock(&chip->mutex);
		return;
	}

	ret = regmap_update_bits(chip->regmap, AW2013_LCTR,
				 AW2013_LCTR_LE(num), 0xFF);

	mutex_unlock(&chip->mutex);
}

static int aw2013_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on, unsigned long *delay_off)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
	int ret;

	if (!led->cdev.brightness) {
		led->cdev.brightness = LED_FULL;
		ret = aw2013_brightness_set(&led->cdev, led->cdev.brightness);
		if (ret)
			goto out;
	}

	/* Never on - just set to off */
	if (!*delay_on) {
		led->chip->blinking = false;
		led->cdev.brightness = LED_OFF;
		return aw2013_brightness_set(&led->cdev, LED_OFF);
	}

	/* Never off - brightness is already set, disable blinking */
	if (!*delay_off) {
		mutex_lock(&led->chip->mutex);
		ret = regmap_update_bits(led->chip->regmap, 
					 AW2013_LCFG(led->num), AW2013_LCFG_MD, 0);
		mutex_unlock(&led->chip->mutex);
		led->chip->blinking = false;
		goto out;
	}

	led->off_ms = *delay_off;
	led->on_ms = *delay_on;
	led->chip->blinking = true;

	schedule_work(&led->chip->blink_work);

out:
	return ret;
}

static ssize_t aw2013_breath_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led,
								cdev);

	return led->chip->blinking;
}

static ssize_t aw2013_breath_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
		container_of(led_cdev, struct aw2013_led, cdev);
	bool blinking;
	ssize_t ret = -EINVAL;

	ret = kstrtobool(buf, &blinking);
	if (ret < 0)
		return ret;

	led->chip->blinking = blinking;
	led->cdev.brightness = led->chip->blinking ? LED_FULL : LED_OFF;
	ret = aw2013_brightness_set(&led->cdev, led->cdev.brightness);
	if (ret)
		return ret;

	schedule_work(&led->chip->blink_work);

	return len;
}

static DEVICE_ATTR(breath, 0644, aw2013_breath_show, aw2013_breath_store);

static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_breath.attr,
	NULL,
};

static const struct attribute_group aw2013_led_attr_group = {
	.attrs = aw2013_led_attributes
};

static int aw2013_probe_dt(struct aw2013 *chip)
{
	struct device_node *np = dev_of_node(&chip->client->dev), *child;
	int count, ret = 0, i = 0, j = 0;
	struct aw2013_led *led;

	count = of_get_available_child_count(np);
	if (!count || count > AW2013_MAX_LEDS)
		return -EINVAL;

	regmap_write(chip->regmap, AW2013_RSTR, AW2013_RSTR_RESET);

	for_each_available_child_of_node(np, child) {
		u32 source;
		u32 imax;

		ret = of_property_read_u32(child, "reg", &source);
		if (ret != 0 || source >= AW2013_MAX_LEDS) {
			dev_err(&chip->client->dev,
				"Couldn't read LED address: %d\n", ret);
			count--;
			continue;
		}

		led = &chip->leds[i];
		led->num = source;
		led->chip = chip;

		if (!of_property_read_u32(child, "led-max-microamp", &imax)) {
			led->imax = min_t(u32, imax / 5000, 3);
		} else {
			led->imax = 1; // 5mA
			dev_info(&chip->client->dev,
				 "DT property led-max-microamp is missing\n");
		}

		INIT_WORK(&chip->blink_work, aw2013_blink_work);

		led->cdev.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led->cdev.brightness_set_blocking = aw2013_brightness_set;
		led->cdev.blink_set = aw2013_blink_set;
		led->cdev.blink_delay_off = 500;
		led->cdev.blink_delay_on = 500;

		ret = devm_led_classdev_register(&chip->client->dev,
						     &led->cdev);
		if (ret < 0) {
			of_node_put(child);
			goto err_out;
		}

		ret = sysfs_create_group(&led->cdev.dev->kobj,
					&aw2013_led_attr_group);
		if (ret) {
			dev_err(&chip->client->dev, "led sysfs ret: %d\n", ret);
			goto err_out;
		}

		i++;
	}

	if (!count)
		return -EINVAL;

	chip->num_leds = i;

	return 0;

err_out:
	for (j = 0; j < chip->num_leds; j++)
		sysfs_remove_group(&chip->leds[j].cdev.dev->kobj,
				   &aw2013_led_attr_group);
	cancel_work_sync(&chip->blink_work);
	return ret;
}

static const struct regmap_config aw2013_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW2013_REG_MAX,
};

static int aw2013_probe(struct i2c_client *client)
{
	struct aw2013 *chip;
	int ret;
	unsigned int chipid;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->mutex);
	mutex_lock(&chip->mutex);

	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &aw2013_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto error;
	}

	chip->vcc_regulator = devm_regulator_get(&client->dev, "vcc");
	ret = PTR_ERR_OR_ZERO(chip->vcc_regulator);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Failed to request regulator: %d\n", ret);
		goto error;
	}

	ret = regulator_enable(chip->vcc_regulator);
	if (ret) {
		dev_err(&client->dev,
			"Failed to enable regulator: %d\n", ret);
		goto error;
	}

	ret = regmap_read(chip->regmap, AW2013_RSTR, &chipid);
	if (ret) {
		dev_err(&client->dev, "Failed to read chip ID: %d\n",
			ret);
		goto error_reg;
	}

	if (chipid != AW2013_RSTR_CHIP_ID) {
		dev_err(&client->dev, "Chip reported wrong ID: %x\n",
			chipid);
		ret = -ENODEV;
		goto error_reg;
	}

	ret = aw2013_probe_dt(chip);
	if (ret < 0)
		goto error_reg;

	ret = regulator_disable(chip->vcc_regulator);
	if (ret) {
		dev_err(&client->dev,
			"Failed to disable regulator: %d\n", ret);
		goto error;
	}

	mutex_unlock(&chip->mutex);

	return 0;

error_reg:
	regulator_disable(chip->vcc_regulator);

error:
	mutex_destroy(&chip->mutex);
	return ret;
}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013 *chip = i2c_get_clientdata(client);
	int i, parsed_leds = chip->num_leds;

	for (i = 0; i < parsed_leds; i++)
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
				   &aw2013_led_attr_group);
	cancel_work_sync(&chip->blink_work);

	aw2013_chip_disable(chip);

	mutex_destroy(&chip->mutex);

	return 0;
}

static const struct of_device_id aw2013_match_table[] = {
	{ .compatible = "awinic,aw2013", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, aw2013_match_table);

static struct i2c_driver aw2013_driver = {
	.driver = {
		.name = "leds-aw2013",
		.of_match_table = of_match_ptr(aw2013_match_table),
	},
	.probe_new = aw2013_probe,
	.remove = aw2013_remove,
};

module_i2c_driver(aw2013_driver);

MODULE_AUTHOR("Nikita Travkin <nikitos.tr@gmail.com>");
MODULE_DESCRIPTION("AW2013 LED driver");
MODULE_LICENSE("GPL v2");
