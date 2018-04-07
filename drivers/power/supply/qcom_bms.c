// SPDX-License-Identifier: GPL

/*
 * Qualcomm Battery Monitoring System driver
 *
 * Copyright (C) 2018 Craig Tatlor <ctatlor97@gmail.com>
 */

#include <linux/module.h>
#include <linux/fixp-arith.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/iio/consumer.h>

#define REG_BMS_OCV_FOR_SOC_DATA0	0x90
#define REG_BMS_SHDW_CC_DATA0		0xA8
#define REG_BMS_CC_DATA_CTL		0x42
#define REG_BMS_CC_CLEAR_CTL		0x4

#define BMS_HOLD_OREG_DATA		BIT(0)
#define BMS_CLEAR_SHDW_CC		BIT(6)

#define BMS_CC_READING_RESOLUTION_N	542535
#define BMS_CC_READING_RESOLUTION_D	10000
#define BMS_CC_READING_TICKS		56
#define BMS_SLEEP_CLK_HZ		32764

#define SECONDS_PER_HOUR		3600
#define TEMPERATURE_COLS		5
#define MAX_CAPACITY_ROWS		50

/* lookup table for ocv -> capacity conversion */
struct bms_ocv_lut {
	int rows;
	s8 temp_legend[TEMPERATURE_COLS];
	u8 capacity_legend[MAX_CAPACITY_ROWS];
	u32 lut[MAX_CAPACITY_ROWS][TEMPERATURE_COLS];
};

/* lookup table for battery temperature -> fcc conversion */
struct bms_fcc_lut {
	s8 temp_legend[TEMPERATURE_COLS];
	u32 lut[TEMPERATURE_COLS];
};

struct bms_device_info {
	struct device *dev;
	struct regmap *regmap;
	struct bms_ocv_lut ocv_lut;
	struct power_supply_desc bat_desc;
	struct bms_fcc_lut fcc_lut;
	struct iio_channel *adc;
	struct mutex bms_output_lock;
	u32 base_addr;

	int ocv_thr_irq;
	u32 ocv;
};

static bool between(int left, int right, int val)
{
	if (left <= val && val <= right)
		return true;

	if (left >= val && val >= right)
		return true;

	return false;
}

static int interpolate_capacity(int temp, u32 ocv,
				struct bms_ocv_lut *ocv_lut)
{
	int pcj_minus_one = 0, pcj = 0, i2 = 0, i3 = 0, i, j;

	for (j = 0; j < TEMPERATURE_COLS; j++)
		if (temp <= ocv_lut->temp_legend[j])
			break;

	if (ocv >= ocv_lut->lut[0][j])
		return ocv_lut->capacity_legend[0];

	if (ocv <= ocv_lut->lut[ocv_lut->rows-1][j-1])
		return ocv_lut->capacity_legend[ocv_lut->rows-1];

	for (i = 0; i < ocv_lut->rows-1; i++) {
		if (between(ocv_lut->lut[i][j],
			    ocv_lut->lut[i+1][j], ocv))
			i2 = i;

		if (between(ocv_lut->lut[i][j-1],
			    ocv_lut->lut[i+1][j-1], ocv))
			i3 = i;
	}

	/* interpolate two capacities */
	pcj = fixp_linear_interpolate(ocv_lut->lut[i2][j],
				      ocv_lut->capacity_legend[i2],
				      ocv_lut->lut[i2+1][j],
				      ocv_lut->capacity_legend[i2+1],
				      ocv);

	pcj_minus_one = fixp_linear_interpolate(ocv_lut->lut[i3][j-1],
						ocv_lut->capacity_legend[i3],
						ocv_lut->lut[i3+1][j-1],
						ocv_lut->capacity_legend[i3+1],
						ocv);

	/* interpolate them with the battery temperature */
	return fixp_linear_interpolate(ocv_lut->temp_legend[j-1],
				       pcj_minus_one,
				       ocv_lut->temp_legend[j],
				       pcj,
				       temp);
}

static int interpolate_fcc(int temp, struct bms_fcc_lut *fcc_lut)
{
	int i;

	for (i = 0; i < TEMPERATURE_COLS; i++)
		if (temp <= fcc_lut->temp_legend[i])
			break;

	return fixp_linear_interpolate(fcc_lut->temp_legend[i-1],
			     fcc_lut->lut[i-1],
			     fcc_lut->temp_legend[i],
			     fcc_lut->lut[i],
			     temp);
}

static int bms_lock_output_data(struct bms_device_info *di)
{
	int ret;

	ret = regmap_update_bits(di->regmap, di->base_addr +
				 REG_BMS_CC_DATA_CTL,
				 BMS_HOLD_OREG_DATA, BMS_HOLD_OREG_DATA);
	if (ret) {
		dev_err(di->dev, "failed to lock bms output: %d", ret);
		return ret;
	}

	/*
	 * Sleep for at least 100 microseconds here to make sure
	 * there has been at least three cycles of the sleep clock
	 * so that the registers are correctly locked.
	 */
	usleep_range(100, 1000);

	return 0;
}

static int bms_unlock_output_data(struct bms_device_info *di)
{
	int ret;

	ret = regmap_update_bits(di->regmap, di->base_addr +
				 REG_BMS_CC_DATA_CTL,
				 BMS_HOLD_OREG_DATA, 0);
	if (ret) {
		dev_err(di->dev, "failed to unlock bms output: %d", ret);
		return ret;
	}

	return 0;
}

static int bms_read_ocv(struct bms_device_info *di, u32 *ocv)
{
	int ret;
	u16 read_ocv;

	mutex_lock(&di->bms_output_lock);

	ret = bms_lock_output_data(di);
	if (ret)
		goto err_lock;

	ret = regmap_bulk_read(di->regmap, di->base_addr+
			       REG_BMS_OCV_FOR_SOC_DATA0, &read_ocv, 2);
	if (ret) {
		dev_err(di->dev, "open circuit voltage read failed: %d", ret);
		goto err_read;
	}

	dev_dbg(di->dev, "read open circuit voltage of: %d mv", read_ocv);


	*ocv = read_ocv * 1000;

err_read:
	bms_unlock_output_data(di);

err_lock:
	mutex_unlock(&di->bms_output_lock);

	return ret;
}

static int bms_read_cc(struct bms_device_info *di, s64 *cc_uah)
{
	int ret;
	s64 cc_raw_s36, cc_raw, cc_uv, cc_pvh;

	mutex_lock(&di->bms_output_lock);

	ret = bms_lock_output_data(di);
	if (ret)
		goto err_lock;

	ret = regmap_bulk_read(di->regmap, di->base_addr +
			       REG_BMS_SHDW_CC_DATA0,
			       &cc_raw_s36, 5);
	if (ret) {
		dev_err(di->dev, "coulomb counter read failed: %d", ret);
		goto err_read;
	}

	ret = bms_unlock_output_data(di);
	if (ret)
		goto err_lock;

	mutex_unlock(&di->bms_output_lock);

	cc_raw = sign_extend32(cc_raw_s36, 28);

	/* convert raw to uv */
	cc_uv = div_s64(cc_raw * BMS_CC_READING_RESOLUTION_N,
			BMS_CC_READING_RESOLUTION_D);

	/* convert uv to pvh */
	cc_pvh = div_s64(cc_uv * BMS_CC_READING_TICKS * 100000,
			 BMS_SLEEP_CLK_HZ * SECONDS_PER_HOUR);

	/* divide by impedance */
	*cc_uah = div_s64(cc_pvh, 10000);

	dev_dbg(di->dev, "read coulomb counter value of: %lld uah", *cc_uah);

	return 0;

err_read:
	bms_unlock_output_data(di);

err_lock:
	mutex_unlock(&di->bms_output_lock);

	return ret;
}

static void bms_reset_cc(struct bms_device_info *di)
{
	int ret;

	mutex_lock(&di->bms_output_lock);

	ret = regmap_update_bits(di->regmap, di->base_addr +
				 REG_BMS_CC_CLEAR_CTL,
				 BMS_CLEAR_SHDW_CC,
				 BMS_CLEAR_SHDW_CC);
	if (ret) {
		dev_err(di->dev, "coulomb counter reset failed: %d", ret);
		goto err_lock;
	}

	/* wait at least three sleep cycles for cc to reset */
	usleep_range(100, 1000);

	ret = regmap_update_bits(di->regmap, di->base_addr +
				 REG_BMS_CC_CLEAR_CTL,
				 BMS_CLEAR_SHDW_CC, 0);
	if (ret)
		dev_err(di->dev, "coulomb counter re-enable failed: %d", ret);

err_lock:
	mutex_unlock(&di->bms_output_lock);
}

static int bms_calculate_capacity(struct bms_device_info *di, int *capacity)
{
	unsigned long fcc;
	int ret, temp, ocv_capacity, temp_degc;
	s64 cc = 0;

	ret = iio_read_channel_raw(di->adc, &temp);
	if (ret < 0) {
		dev_err(di->dev, "failed to read temperature: %d", ret);
		return ret;
	}

	temp_degc = DIV_ROUND_CLOSEST(temp, 1000);

	ret = bms_read_cc(di, &cc);
	if (ret < 0) {
		dev_err(di->dev, "failed to read coulomb counter: %d", ret);
		return ret;
	}

	/* interpolate capacity from open circuit voltage */
	ocv_capacity = interpolate_capacity(temp_degc, di->ocv,
					    &di->ocv_lut);

	/* interpolate the full charge capacity from temperature */
	fcc = interpolate_fcc(temp_degc, &di->fcc_lut);

	/* append coloumb counter to capacity */
	*capacity = DIV_ROUND_CLOSEST(fcc * ocv_capacity, 100);
	*capacity = div_s64((*capacity - cc) * 100, fcc);

	return 0;
}



/*
 * Return power_supply property
 */
static int bms_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct bms_device_info *di = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bms_calculate_capacity(di, &val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (val->intval == INT_MAX || val->intval == INT_MIN)
		ret = -EINVAL;

	return ret;
}

static enum power_supply_property bms_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
};

static irqreturn_t bms_ocv_thr_irq_handler(int irq, void *dev_id)
{
	struct bms_device_info *di = dev_id;

	if (bms_read_ocv(di, &di->ocv) < 0)
		return IRQ_HANDLED;

	bms_reset_cc(di);
	return IRQ_HANDLED;
}

static int bms_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct bms_device_info *di;
	struct power_supply *bat;
	int ret;

	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &pdev->dev;

	di->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!di->regmap) {
		dev_err(di->dev, "Unable to get regmap");
		return -EINVAL;
	}

	di->adc = devm_iio_channel_get(&pdev->dev, "temp");
	if (IS_ERR(di->adc))
		return PTR_ERR(di->adc);

	ret = of_property_read_u32(di->dev->of_node, "reg", &di->base_addr);
	if (ret < 0)
		return ret;

	ret = of_property_read_u8_array(di->dev->of_node,
						 "qcom,ocv-temp-legend-celsius",
						 (u8 *)di->ocv_lut.temp_legend,
						 TEMPERATURE_COLS);
	if (ret < 0) {
		dev_err(di->dev, "no open circuit voltage temperature legend found");
		return ret;
	}

	di->ocv_lut.rows = of_property_read_variable_u8_array(di->dev->of_node,
						 "qcom,ocv-capacity-legend",
						 di->ocv_lut.capacity_legend, 0,
						 MAX_CAPACITY_ROWS);
	if (di->ocv_lut.rows < 0) {
		dev_err(di->dev, "no open circuit voltage capacity legend found");
		return ret;
	}

	ret = of_property_read_variable_u32_array(di->dev->of_node,
						  "qcom,ocv-lut-microvolt",
						  (u32 *)di->ocv_lut.lut,
						  TEMPERATURE_COLS,
						  TEMPERATURE_COLS *
						  MAX_CAPACITY_ROWS);
	if (ret < 0) {
		dev_err(di->dev, "no open circuit voltage lut array found");
		return ret;
	}

	ret = of_property_read_u8_array(di->dev->of_node,
						 "qcom,fcc-temp-legend-celsius",
						 (u8 *)di->fcc_lut.temp_legend,
						 TEMPERATURE_COLS);
	if (ret < 0) {
		dev_err(di->dev, "no full charge capacity temperature legend found");
		return ret;
	}

	ret = of_property_read_u32_array(di->dev->of_node,
						  "qcom,fcc-lut-microamp-hours",
						  di->fcc_lut.lut,
						  TEMPERATURE_COLS);
	if (ret < 0) {
		dev_err(di->dev, "no full charge capacity lut array found");
		return ret;
	}

	ret = bms_read_ocv(di, &di->ocv);
	if (ret < 0) {
		dev_err(di->dev, "failed to read initial open circuit voltage: %d",
			ret);
		return ret;
	}

	mutex_init(&di->bms_output_lock);

	di->ocv_thr_irq = platform_get_irq_byname(pdev, "ocv_thr");

	ret = devm_request_threaded_irq(di->dev, di->ocv_thr_irq, NULL,
					bms_ocv_thr_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					pdev->name, di);
	if (ret < 0) {
		dev_err(di->dev, "failed to request handler for open circuit voltage threshold IRQ");
		return ret;
	}


	di->bat_desc.name = "bms";
	di->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat_desc.properties = bms_props;
	di->bat_desc.num_properties = ARRAY_SIZE(bms_props);
	di->bat_desc.get_property = bms_get_property;

	psy_cfg.drv_data = di;
	bat = devm_power_supply_register(di->dev, &di->bat_desc, &psy_cfg);

	return PTR_ERR_OR_ZERO(bat);
}

static const struct of_device_id bms_of_match[] = {
	{.compatible = "qcom,pm8941-bms", },
	{ },
};
MODULE_DEVICE_TABLE(of, bms_of_match);

static struct platform_driver bms_driver = {
	.probe = bms_probe,
	.driver = {
		.name = "qcom-bms",
		.of_match_table = of_match_ptr(bms_of_match),
	},
};
module_platform_driver(bms_driver);

MODULE_AUTHOR("Craig Tatlor <ctatlor97@gmail.com>");
MODULE_DESCRIPTION("Qualcomm BMS driver");
MODULE_LICENSE("GPL");
