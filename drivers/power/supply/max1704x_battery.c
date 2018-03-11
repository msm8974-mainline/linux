/*
* max17048 driver based off max17040 driver
 incomplete
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#define MAX17048

#define MAX1704x_VCELL	0x02
#define MAX1704x_SOC	0x04
#define MAX1704x_MODE	0x06
#define MAX1704x_VER	0x08

#define MAX17048_HIBRT	0x0A
#define MAX17048_CONFIG	0x0C
#define MAX17048_VALRT	0x14
#define MAX17048_CRATE	0x16
#define MAX17048_VRESET	0x18
#define MAX17048_STATUS	0x1A

#define MAX1704x_DELAY		1000

struct max1704x_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		*battery;

	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
};

static int max1704x_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max1704x_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max1704x_write_reg(struct i2c_client *client, int reg, u16 value)
{
	int ret;

	ret = i2c_smbus_write_word_swapped(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max1704x_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_word_swapped(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void max1704x_reset(struct i2c_client *client)
{
#ifdef MAX17048
	max1704x_write_reg(client, MAX1704x_MODE, 0x4000); // quickstart
#else
	max1704x_write_reg(client, MAX17040_CMD, 0x0054);
#endif
}

static void max1704x_get_vcell(struct i2c_client *client)
{
	struct max1704x_chip *chip = i2c_get_clientdata(client);
	u16 vcell;

	vcell = max1704x_read_reg(client, MAX1704x_VCELL);

	chip->vcell = vcell;
}

static void max1704x_get_soc(struct i2c_client *client)
{
	struct max1704x_chip *chip = i2c_get_clientdata(client);
	u16 soc;

	soc = max1704x_read_reg(client, MAX1704x_SOC);

	chip->soc = soc;
}

static void max1704x_get_version(struct i2c_client *client)
{
	u16 version;

	version = max1704x_read_reg(client, MAX1704x_VER);

	dev_warn(&client->dev, "MAX1704x Fuel-Gauge Ver 0x%x\n", version);
}

static void max1704x_work(struct work_struct *work)
{
	struct max1704x_chip *chip;

	chip = container_of(work, struct max1704x_chip, work.work);

	max1704x_get_vcell(chip->client);
	max1704x_get_soc(chip->client);

	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX1704x_DELAY);
}

static enum power_supply_property max1704x_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc max1704x_battery_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max1704x_get_property,
	.properties	= max1704x_battery_props,
	.num_properties	= ARRAY_SIZE(max1704x_battery_props),
};

static int max1704x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct max1704x_chip *chip;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	i2c_set_clientdata(client, chip);
	psy_cfg.drv_data = chip;

	chip->battery = power_supply_register(&client->dev,
				&max1704x_battery_desc, &psy_cfg);
	if (IS_ERR(chip->battery)) {
		dev_err(&client->dev, "failed: power supply register\n");
		return PTR_ERR(chip->battery);
	}

	max1704x_reset(client);
	max1704x_get_version(client);

	// TODO alert irq

	INIT_DEFERRABLE_WORK(&chip->work, max1704x_work);
	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX1704x_DELAY);

	return 0;
}

static int max1704x_remove(struct i2c_client *client)
{
	struct max1704x_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(chip->battery);
	cancel_delayed_work(&chip->work);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int max1704x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1704x_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max1704x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1704x_chip *chip = i2c_get_clientdata(client);

	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX1704x_DELAY);
	return 0;
}

static SIMPLE_DEV_PM_OPS(max1704x_pm_ops, max1704x_suspend, max1704x_resume);
#define MAX1704x_PM_OPS (&max1704x_pm_ops)

#else

#define MAX1704x_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id max1704x_of_match[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max1704x_of_match);

static struct i2c_driver max1704x_i2c_driver = {
	.driver	= {
		.name	= "max1704x",
		.of_match_table = max1704x_of_match,
		.pm	= MAX1704x_PM_OPS,
	},
	.probe		= max1704x_probe,
	.remove		= max1704x_remove,
};
module_i2c_driver(max1704x_i2c_driver);

MODULE_DESCRIPTION("MAX1704x Fuel Gauge");
MODULE_LICENSE("GPL");
