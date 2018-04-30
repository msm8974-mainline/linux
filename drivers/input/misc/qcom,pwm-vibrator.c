/*
simple dumb gpio vibrator, based off pwm-vibra
 */

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/clk.h>

struct pwm_vibrator {
	struct input_dev *input;
	struct regulator *vcc;
	struct gpio_desc *gpio;
	struct clk *clk;

	struct work_struct play_work;
	u16 level, set_level;
};

static int pwm_vibrator_set_level(struct pwm_vibrator *vibrator, u16 level)
{
	struct device *pdev = vibrator->input->dev.parent;
	int err;

	if (!vibrator->level && level) {
		// set clock params
		err = clk_set_rate(vibrator->clk, 24000);
		if (err) {
			dev_err(pdev, "failed to set clock rate: %d", err);
			return err;
		}

		// clk interface doesnt let us control reg_d paramater...
#if 1
	{
		int d_value = (0xffff - level) / 512;
		void __iomem *mm = ioremap(0xFD8C3000, 0x1000) + 0x450;
#define REG_CMD_RCGR	0x00
#define REG_CFG_RCGR	0x04
#define REG_M		0x08
#define REG_N		0x0C
#define REG_D		0x10
#define REG_CBCR	0x24

		writel((2 << 12) | /* dual edge mode */
		       (0 << 8) |  /* cxo */
		       (7 << 0),
		       mm + REG_CFG_RCGR);
		writel(1, mm + REG_M);
		writel(128, mm + REG_N);
		writel(d_value, mm + REG_D);
		writel(1, mm + REG_CMD_RCGR);
	writel(1, mm + REG_CBCR);
	}
#endif

		err = clk_prepare_enable(vibrator->clk);
		if (err) {
			dev_err(pdev, "failed to enable clock: %d", err);
			return err;
		}

		err = regulator_enable(vibrator->vcc);
		if (err) {
			clk_disable_unprepare(vibrator->clk);
			dev_err(pdev, "failed to enable regulator: %d", err);
			return err;
		}

		gpiod_set_value_cansleep(vibrator->gpio, 1);
	} else if (vibrator->level && !level) {
		gpiod_set_value_cansleep(vibrator->gpio, 0);
		regulator_disable(vibrator->vcc);
		clk_disable_unprepare(vibrator->clk);
	} else if (level) {
		// change clock params
	}

	vibrator->level = level;
	return 0;
}

static void pwm_vibrator_play_work(struct work_struct *work)
{
	struct pwm_vibrator *vibrator = container_of(work,
					struct pwm_vibrator, play_work);

	pwm_vibrator_set_level(vibrator, vibrator->set_level);
}

static int pwm_vibrator_play_effect(struct input_dev *dev, void *data,
				    struct ff_effect *effect)
{
	struct pwm_vibrator *vibrator = input_get_drvdata(dev);

	printk("pwm_vibrator_play_effect %u %u\n", effect->u.rumble.strong_magnitude, effect->u.rumble.weak_magnitude);

	vibrator->set_level = effect->u.rumble.strong_magnitude ?: effect->u.rumble.weak_magnitude;

	schedule_work(&vibrator->play_work);

	return 0;
}

static void pwm_vibrator_close(struct input_dev *input)
{
	struct pwm_vibrator *vibrator = input_get_drvdata(input);

	pwm_vibrator_set_level(vibrator, 0);
}

static int pwm_vibrator_probe(struct platform_device *pdev)
{
	struct pwm_vibrator *vibrator;
	int err;

	vibrator = devm_kzalloc(&pdev->dev, sizeof(*vibrator), GFP_KERNEL);
	if (!vibrator)
		return -ENOMEM;

	vibrator->input = devm_input_allocate_device(&pdev->dev);
	if (!vibrator->input)
		return -ENOMEM;

	vibrator->vcc = devm_regulator_get(&pdev->dev, "vcc");
	err = PTR_ERR_OR_ZERO(vibrator->vcc);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request regulator: %d", err);
		return err;
	}

	vibrator->clk = devm_clk_get(&pdev->dev, "pwm");
	err = PTR_ERR_OR_ZERO(vibrator->clk);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request src clock: %d", err);
		return err;
	}

	vibrator->gpio = devm_gpiod_get_optional(&pdev->dev, "enable", GPIOD_OUT_LOW);
	err = PTR_ERR_OR_ZERO(vibrator->gpio);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request gpio: %d", err);
		return err;
	}

	INIT_WORK(&vibrator->play_work, pwm_vibrator_play_work);

	vibrator->input->name = "pwm-vibrator";
	vibrator->input->id.bustype = BUS_HOST;
	vibrator->input->dev.parent = &pdev->dev;
	vibrator->input->close = pwm_vibrator_close;

	input_set_drvdata(vibrator->input, vibrator);
	input_set_capability(vibrator->input, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(vibrator->input, NULL, pwm_vibrator_play_effect);
	if (err) {
		dev_err(&pdev->dev, "Couldn't create FF dev: %d", err);
		return err;
	}

	err = input_register_device(vibrator->input);
	if (err) {
		dev_err(&pdev->dev, "Couldn't register input dev: %d", err);
		return err;
	}

	platform_set_drvdata(pdev, vibrator);

	return 0;
}

static int __maybe_unused pwm_vibrator_suspend(struct device *dev)
{
	struct pwm_vibrator *vibrator = dev_get_drvdata(dev);

	cancel_work_sync(&vibrator->play_work);
	pwm_vibrator_set_level(vibrator, 0);
	return 0;
}

static int __maybe_unused pwm_vibrator_resume(struct device *dev)
{
	struct pwm_vibrator *vibrator = dev_get_drvdata(dev);

	pwm_vibrator_set_level(vibrator, vibrator->set_level);
	return 0;
}

static SIMPLE_DEV_PM_OPS(pwm_vibrator_pm_ops,
			 pwm_vibrator_suspend, pwm_vibrator_resume);

#ifdef CONFIG_OF
static const struct of_device_id pwm_vibra_dt_match_table[] = {
	{ .compatible = "qcom,pwm-vibrator" },
	{},
};
MODULE_DEVICE_TABLE(of, pwm_vibra_dt_match_table);
#endif

static struct platform_driver pwm_vibrator_driver = {
	.probe	= pwm_vibrator_probe,
	.driver	= {
		.name	= "qcom,pwm-vibrator",
		.pm	= &pwm_vibrator_pm_ops,
		.of_match_table = of_match_ptr(pwm_vibra_dt_match_table),
	},
};
module_platform_driver(pwm_vibrator_driver);
