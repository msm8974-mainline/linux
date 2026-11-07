// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 *
 * SAW power controller driver
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/smp.h>

#include <linux/regulator/driver.h>

#include <soc/qcom/spm.h>

#define FIELD_SET(current, mask, val)	\
	(((current) & ~(mask)) | FIELD_PREP((mask), (val)))

#define SPM_CTL_INDEX		0x7f
#define SPM_CTL_INDEX_SHIFT	4
#define SPM_CTL_EN		BIT(0)

/* These registers might be specific to SPM 1.1 */
#define SPM_VCTL_VLVL			GENMASK(7, 0)
#define SPM_PMIC_DATA_0_VLVL		GENMASK(7, 0)
#define SPM_PMIC_DATA_1_MIN_VSEL	GENMASK(5, 0)
#define SPM_PMIC_DATA_1_MAX_VSEL	GENMASK(21, 16)

#define SPM_1_1_AVS_CTL_AVS_ENABLED	BIT(27)
#define SPM_AVS_CTL_MAX_VLVL		GENMASK(22, 17)
#define SPM_AVS_CTL_MIN_VLVL		GENMASK(15, 10)

/* SAW2 v2.1 specific definitions */
#define SPM_2_1_VCTL_VLVL		GENMASK(7, 0)
#define SPM_2_1_VCTL_PORT		GENMASK(18, 16)
#define SPM_2_1_AVS_CTL_AVS_ENABLED	BIT(0)
#define SPM_2_1_PMIC_STS_CURR_VLVL	GENMASK(7, 0)
#define SPM_2_1_PMIC_STS_STATE		GENMASK(17, 16)

/*
 * The gang-rail SAW2 also carries the SMPS phase count (VCTL port 1) and
 * FTS mode (VCTL port 2); the data byte for port 2 selects PFM (0x00) or
 * PWM (0x80).
 */
#define SPM_SAW_PHASE_PORT		1
#define SPM_SAW_FTS_PORT		2
#define SPM_FTS_PWM			0x80

enum spm_reg {
	SPM_REG_CFG,
	SPM_REG_SPM_CTL,
	SPM_REG_DLY,
	SPM_REG_PMIC_DLY,
	SPM_REG_PMIC_DATA_0,
	SPM_REG_PMIC_DATA_1,
	SPM_REG_VCTL,
	SPM_REG_SEQ_ENTRY,
	SPM_REG_STS0,
	SPM_REG_STS1,
	SPM_REG_PMIC_STS,
	SPM_REG_AVS_CTL,
	SPM_REG_AVS_LIMIT,
	SPM_REG_RST,
	SPM_REG_NR,
};

#define MAX_PMIC_DATA		2
#define MAX_SEQ_DATA		64

struct spm_reg_data {
	const u16 *reg_offset;
	u32 spm_cfg;
	u32 spm_dly;
	u32 pmic_dly;
	u32 pmic_data[MAX_PMIC_DATA];
	u32 avs_ctl;
	u32 avs_limit;
	u8 seq[MAX_SEQ_DATA];
	u8 start_index[PM_SLEEP_MODE_NR];

	smp_call_func_t set_vdd;
	/* for now we support only a single range */
	struct linear_range *range;
	unsigned int ramp_delay;
	unsigned int init_uV;
};

struct spm_driver_data {
	void __iomem *reg_base;
	const struct spm_reg_data *reg_data;
	struct device *dev;
	/*
	 * volt_sel_req is the selector a set_vdd() call must program;
	 * volt_sel is the last selector CONFIRMED by the PMIC handshake and
	 * is what get_voltage_sel() reports.  They must stay separate: if a
	 * failed write updated volt_sel, the regulator core's retry would
	 * compare the request against the poisoned cache, skip the hardware
	 * write AND the ramp delay, and report success - after which the OPP
	 * core raises the CPU clock against a rail that never moved.
	 */
	unsigned int volt_sel;
	unsigned int volt_sel_req;
	/*
	 * Counters for the rail handshake, exposed through debugfs.  A failed
	 * handshake is the one event on this platform that is known to end in
	 * a silent reset when software then clocks the CPU against a rail that
	 * did not move, so it must be countable rather than only appearing as
	 * a rate-limited log line.
	 */
	unsigned int vdd_writes;
	unsigned int vdd_timeouts;
	struct dentry *debugfs;
	int reg_cpu;
	/*
	 * set_vdd() cannot return a value (it is an smp_call_func_t), so it
	 * records the outcome here for spm_set_voltage_sel() to propagate.
	 */
	int set_vdd_ret;
};

static const u16 spm_reg_offset_v4_1[SPM_REG_NR] = {
	[SPM_REG_AVS_CTL]	= 0x904,
	[SPM_REG_AVS_LIMIT]	= 0x908,
};

static const struct spm_reg_data spm_reg_660_gold_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4580458,
};

static const struct spm_reg_data spm_reg_660_silver_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x101c031,
	.avs_limit = 0x4580458,
};

static const struct spm_reg_data spm_reg_8998_gold_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4700470,
};

static const struct spm_reg_data spm_reg_8998_silver_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4200420,
};

static const u16 spm_reg_offset_v3_0[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_SEQ_ENTRY]	= 0x400,
};

/* SPM register data for 8909 */
static const struct spm_reg_data spm_reg_8909_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x76, 0x76, 0x0B, 0x94, 0x5B, 0x80,
		0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

/* SPM register data for 8916 */
static const struct spm_reg_data spm_reg_8916_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x3B, 0x76, 0x76, 0x0B, 0x94, 0x5B,
		0x80, 0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const struct spm_reg_data spm_reg_8939_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x50, 0x1B, 0x10, 0x80,
		0x30, 0x90, 0x5B, 0x60, 0x50, 0x03, 0x60, 0x76, 0x76, 0x0B,
		0x50, 0x1B, 0x94, 0x5B, 0x80, 0x10, 0x26, 0x30, 0x50, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const u16 spm_reg_offset_v2_3[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_PMIC_DATA_0]	= 0x40,
	[SPM_REG_PMIC_DATA_1]	= 0x44,
};

/* SPM register data for 8976 */
static const struct spm_reg_data spm_reg_8976_gold_l2 = {
	.reg_offset = spm_reg_offset_v2_3,
	.spm_cfg = 0x14,
	.spm_dly = 0x3c11840a,
	.pmic_data[0] = 0x03030080,
	.pmic_data[1] = 0x00030000,
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 3,
};

static const struct spm_reg_data spm_reg_8976_silver_l2 = {
	.reg_offset = spm_reg_offset_v2_3,
	.spm_cfg = 0x14,
	.spm_dly = 0x3c102800,
	.pmic_data[0] = 0x03030080,
	.pmic_data[1] = 0x00030000,
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 2,
};

static const u16 spm_reg_offset_v2_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

/* Extended v2.1 offset table with voltage control registers for 8974/8084 */
static const u16 spm_reg_offset_v2_1_cpu[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_STS0]		= 0x0C,
	[SPM_REG_PMIC_STS]	= 0x14,
	[SPM_REG_RST]		= 0x18,
	[SPM_REG_VCTL]		= 0x1C,
	[SPM_REG_AVS_CTL]	= 0x20,
	[SPM_REG_AVS_LIMIT]	= 0x24,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_PMIC_DATA_0]	= 0x40,
	[SPM_REG_PMIC_DATA_1]	= 0x44,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

static void smp_set_vdd_v1_1(void *data);

static struct linear_range spm_v1_1_regulator_range =
	REGULATOR_LINEAR_RANGE(700000, 0, 56, 12500);

/*
 * The selector written to VCTL[7:0] is the raw FTS2 setpoint, zero-based
 * from 0 V in 5 mV steps; the usable range is 350 mV (70) to 1.275 V (255).
 */
static struct linear_range spm_v2_1_regulator_range =
	REGULATOR_LINEAR_RANGE(350000, 70, 255, 5000);

/*
 * SPM register data for 8974, 8084 per-CPU.
 *
 * These SAWs run idle/sleep sequences only: on MSM8974 the L2/APCS SAW is the
 * one wired to the PMIC over SPMI, and the vendor accordingly gives its per-CPU
 * nodes no pmic-data at all.  Use the plain v2.1 offset table so PMIC_DATA_0/1
 * are not even mapped here - the extended table plus a PMIC payload would let a
 * per-CPU sequencer drive the shared Krait rail behind the regulator's back.
 */
static const struct spm_reg_data spm_reg_8974_8084_cpu  = {
	.reg_offset = spm_reg_offset_v2_1,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x03, 0x0B, 0x0F, 0x00, 0x20, 0x80, 0x10, 0xE8, 0x5B, 0x03,
		0x3B, 0xE8, 0x5B, 0x82, 0x10, 0x0B, 0x30, 0x06, 0x26, 0x30,
		0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 3,
};

static void smp_set_vdd_v2_1_l2(void *data);

/*
 * SPM register data for 8974, 8084 L2/APCS master SAW2.
 * On MSM8974, only the L2 SAW2 is connected to the PMIC via SPMI for
 * voltage control.  Per-CPU SAW2 units handle only idle/sleep sequences.
 * The L2 SAW2 controls the shared Krait voltage rail (gang supply).
 */
static const struct spm_reg_data spm_reg_8974_8084_l2  = {
	.reg_offset = spm_reg_offset_v2_1_cpu,
	.spm_cfg = 0x14,
	.spm_dly = 0x3C102800,
	.pmic_data[0] = 0x02030080,
	.pmic_data[1] = 0x00030000,
	.seq = { 0x1F, 0x00, 0x03, 0x00, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 0,
	.set_vdd = smp_set_vdd_v2_1_l2,
	.range = &spm_v2_1_regulator_range,
	.init_uV = 1100000,
	.ramp_delay = 1250,
};

/* SPM register data for 8226 */
static const struct spm_reg_data spm_reg_8226_cpu  = {
	.reg_offset = spm_reg_offset_v2_1,
	.spm_cfg = 0x0,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x3B, 0x76, 0x76, 0x0B, 0x94, 0x5B,
		0x80, 0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const u16 spm_reg_offset_v1_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_STS0]		= 0x0c,
	[SPM_REG_STS1]		= 0x10,
	[SPM_REG_VCTL]		= 0x14,
	[SPM_REG_AVS_CTL]	= 0x18,
	[SPM_REG_SPM_CTL]	= 0x20,
	[SPM_REG_PMIC_DLY]	= 0x24,
	[SPM_REG_PMIC_DATA_0]	= 0x28,
	[SPM_REG_PMIC_DATA_1]	= 0x2C,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

static const struct spm_reg_data spm_reg_8064_cpu = {
	.reg_offset = spm_reg_offset_v1_1,
	.spm_cfg = 0x1F,
	.pmic_dly = 0x02020004,
	.pmic_data[0] = 0x0084009C,
	.pmic_data[1] = 0x00A4001C,
	.seq = { 0x03, 0x0F, 0x00, 0x24, 0x54, 0x10, 0x09, 0x03, 0x01,
		0x10, 0x54, 0x30, 0x0C, 0x24, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 2,
	.set_vdd = smp_set_vdd_v1_1,
	.range = &spm_v1_1_regulator_range,
	.init_uV = 1300000,
	.ramp_delay = 1250,
};

static inline void spm_register_write(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	if (drv->reg_data->reg_offset[reg])
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
}

/* Ensure a guaranteed write, before return */
static inline void spm_register_write_sync(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	u32 ret;

	if (!drv->reg_data->reg_offset[reg])
		return;

	do {
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		ret = readl_relaxed(drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		if (ret == val)
			break;
		cpu_relax();
	} while (1);
}

static inline u32 spm_register_read(struct spm_driver_data *drv,
				    enum spm_reg reg)
{
	return readl_relaxed(drv->reg_base + drv->reg_data->reg_offset[reg]);
}

static DEFINE_RAW_SPINLOCK(spm_lock);

void spm_set_low_power_mode(struct spm_driver_data *drv,
			    enum pm_sleep_mode mode)
{
	unsigned long flags;
	u32 start_index;
	u32 ctl_val;

	start_index = drv->reg_data->start_index[mode];

	raw_spin_lock_irqsave(&spm_lock, flags);
	ctl_val = spm_register_read(drv, SPM_REG_SPM_CTL);
	ctl_val &= ~(SPM_CTL_INDEX << SPM_CTL_INDEX_SHIFT);
	ctl_val |= start_index << SPM_CTL_INDEX_SHIFT;
	ctl_val |= SPM_CTL_EN;
	spm_register_write_sync(drv, SPM_REG_SPM_CTL, ctl_val);
	raw_spin_unlock_irqrestore(&spm_lock, flags);
}

static int spm_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);
	unsigned long flags;

	drv->volt_sel_req = selector;
	drv->set_vdd_ret = 0;
	drv->vdd_writes++;

	if (drv->reg_cpu >= 0) {
		int ret = smp_call_function_single(drv->reg_cpu,
						  drv->reg_data->set_vdd,
						  drv, true);
		return ret ? : drv->set_vdd_ret;
	}

	/*
	 * Run the L2/APCS gang-rail write (RST kick -> VCTL -> PMIC_STS poll)
	 * under spm_lock with interrupts disabled.  This rail has no
	 * associated CPU (reg_cpu < 0) so set_vdd() executes in the caller's
	 * context on an arbitrary CPU.  Taking spm_lock serializes this write
	 * against any sibling core entering or exiting cpuidle power-collapse
	 * (spm_set_low_power_mode), preventing concurrent SPM state-machine
	 * corruption on the shared APCS/PMIC bus.
	 */
	raw_spin_lock_irqsave(&spm_lock, flags);
	drv->reg_data->set_vdd(drv);
	raw_spin_unlock_irqrestore(&spm_lock, flags);

	/*
	 * Propagate a failed rail write.  Without this the OPP core sees
	 * success and goes on to raise the clock, so a rail that never reached
	 * the requested level is followed by a frequency the silicon has not
	 * been given the voltage for.  The vendor driver returns -EIO here and
	 * aborts the transition for the same reason.
	 */
	return drv->set_vdd_ret;
}

static int spm_stats_show(struct seq_file *s, void *data)
{
	struct spm_driver_data *drv = s->private;

	seq_printf(s, "vdd_writes   %u\n", drv->vdd_writes);
	seq_printf(s, "vdd_timeouts %u\n", drv->vdd_timeouts);
	seq_printf(s, "volt_sel     %u (confirmed)\n", drv->volt_sel);
	seq_printf(s, "volt_sel_req %u (last requested)\n", drv->volt_sel_req);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(spm_stats);

static int spm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);

	return drv->volt_sel;
}

static const struct regulator_ops spm_reg_ops = {
	.set_voltage_sel	= spm_set_voltage_sel,
	.get_voltage_sel	= spm_get_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static void smp_set_vdd_v1_1(void *data)
{
	struct spm_driver_data *drv = data;
	unsigned int vctl, data0, data1, avs_ctl, sts;
	unsigned int vlevel, volt_sel;
	bool avs_enabled;

	volt_sel = drv->volt_sel_req;
	vlevel = volt_sel | 0x80; /* band */

	avs_ctl = spm_register_read(drv, SPM_REG_AVS_CTL);
	vctl = spm_register_read(drv, SPM_REG_VCTL);
	data0 = spm_register_read(drv, SPM_REG_PMIC_DATA_0);
	data1 = spm_register_read(drv, SPM_REG_PMIC_DATA_1);

	avs_enabled = avs_ctl & SPM_1_1_AVS_CTL_AVS_ENABLED;

	/* If AVS is enabled, switch it off during the voltage change */
	if (avs_enabled) {
		avs_ctl &= ~SPM_1_1_AVS_CTL_AVS_ENABLED;
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}

	/* Kick the state machine back to idle */
	spm_register_write(drv, SPM_REG_RST, 1);

	vctl = FIELD_SET(vctl, SPM_VCTL_VLVL, vlevel);
	data0 = FIELD_SET(data0, SPM_PMIC_DATA_0_VLVL, vlevel);
	data1 = FIELD_SET(data1, SPM_PMIC_DATA_1_MIN_VSEL, volt_sel);
	data1 = FIELD_SET(data1, SPM_PMIC_DATA_1_MAX_VSEL, volt_sel);

	spm_register_write(drv, SPM_REG_VCTL, vctl);
	spm_register_write(drv, SPM_REG_PMIC_DATA_0, data0);
	spm_register_write(drv, SPM_REG_PMIC_DATA_1, data1);

	if (read_poll_timeout_atomic(spm_register_read,
				     sts, sts == vlevel,
				     1, 200, false,
				     drv, SPM_REG_STS1)) {
		dev_err_ratelimited(drv->dev, "timeout setting the voltage (%x %x)!\n", sts, vlevel);
		goto enable_avs;
	}
	drv->volt_sel = volt_sel;

	if (avs_enabled) {
		unsigned int max_avs = volt_sel;
		unsigned int min_avs = max(max_avs, 4U) - 4;

		avs_ctl = FIELD_SET(avs_ctl, SPM_AVS_CTL_MIN_VLVL, min_avs);
		avs_ctl = FIELD_SET(avs_ctl, SPM_AVS_CTL_MAX_VLVL, max_avs);
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}

enable_avs:
	if (avs_enabled) {
		avs_ctl |= SPM_1_1_AVS_CTL_AVS_ENABLED;
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}
}


/*
 * Write one of the L2 SAW2's non-voltage PMIC ports: the SMPS phase count on
 * port 1, the FTS mode on port 2.  Unlike the voltage port (0) these do not
 * update PMIC_STS[7:0], so completion is confirmed by waiting for the PMIC FSM
 * to return to idle (PMIC_STS[17:16] == 0), as the vendor does.  The next
 * voltage write clears the port field back to 0.
 */
static int spm_saw_write_pmic_port(struct spm_driver_data *drv, u32 port,
				   u32 data)
{
	unsigned int vctl, pmic_sts;

	vctl = spm_register_read(drv, SPM_REG_VCTL);
	vctl &= ~(SPM_2_1_VCTL_VLVL | SPM_2_1_VCTL_PORT);
	vctl |= FIELD_PREP(SPM_2_1_VCTL_PORT, port) | (data & 0xff);
	spm_register_write(drv, SPM_REG_VCTL, vctl);

	return read_poll_timeout_atomic(spm_register_read, pmic_sts,
					!(pmic_sts & SPM_2_1_PMIC_STS_STATE),
					1, 200, false, drv, SPM_REG_PMIC_STS);
}

/*
 * smp_set_vdd_v2_1_l2 - Set voltage via L2/APCS SAW2 (MSM8974 gang supply).
 *
 * On MSM8974 only the L2 SAW2 at 0xf9012000 is wired to the PMIC via SPMI.
 * This function is called directly (not via smp_call_function_single) because
 * L2 SAW2 MMIO is bus-accessible from any CPU.
 */
static void smp_set_vdd_v2_1_l2(void *data)
{
	struct spm_driver_data *drv = data;
	unsigned int vctl, avs_ctl, pmic_sts;
	unsigned int vlevel, volt_sel;
	bool avs_enabled;

	volt_sel = drv->volt_sel_req;
	vlevel = volt_sel;

	avs_ctl = spm_register_read(drv, SPM_REG_AVS_CTL);
	avs_enabled = avs_ctl & SPM_2_1_AVS_CTL_AVS_ENABLED;

	if (avs_enabled) {
		avs_ctl &= ~SPM_2_1_AVS_CTL_AVS_ENABLED;
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}

	spm_register_write(drv, SPM_REG_RST, 1);

	vctl = spm_register_read(drv, SPM_REG_VCTL);

	/*
	 * PORT must be 0 (the vctl port): nonzero ports route the data byte
	 * to the PMIC phase/PFM controls instead of the voltage setpoint,
	 * and PMIC_STS only reflects writes made through port 0.
	 */
	vctl &= ~(SPM_2_1_VCTL_VLVL | SPM_2_1_VCTL_PORT);
	vctl |= vlevel;

	spm_register_write(drv, SPM_REG_VCTL, vctl);

	/*
	 * Wait for the PMIC arbiter to latch the new setpoint.  The analog
	 * ramp is waited out by the regulator core via ramp_delay.
	 */
	if (read_poll_timeout_atomic(spm_register_read,
				     pmic_sts,
				     (pmic_sts & SPM_2_1_PMIC_STS_CURR_VLVL) == vlevel,
				     1, 200, false,
				     drv, SPM_REG_PMIC_STS)) {
		dev_err_ratelimited(drv->dev, "timeout setting the voltage (%x %x)!\n",
				    pmic_sts & 0xff, vlevel);
		drv->set_vdd_ret = -ETIMEDOUT;
		drv->vdd_timeouts++;
		return;
	}

	/*
	 * AVS is left disabled deliberately.  It is off by design on this SoC:
	 * the vendor programs AVS_CTL/AVS_LIMIT to zero on every SAW at init
	 * and never arms it.  The window this code used to rebuild cannot even
	 * be expressed in the registers it writes - MIN_VLVL and MAX_VLVL are
	 * 6 bits wide, inherited from v1.1 whose selectors fit, while a v2.1
	 * selector reaches 255, so a window built from one is silently
	 * truncated: selector 180 (900 mV) becomes 48 (240 mV), authorising the
	 * hardware to track the shared Krait rail down to an arbitrary level
	 * with no software involvement.
	 */
	drv->volt_sel = volt_sel;
	drv->set_vdd_ret = 0;
}

static int spm_get_cpu(struct device *dev)
{
	int cpu;
	bool found;

	for_each_possible_cpu(cpu) {
		struct device_node *cpu_node, *saw_node;

		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			continue;

		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		found = (saw_node == dev->of_node);
		of_node_put(saw_node);
		of_node_put(cpu_node);

		if (found)
			return cpu;
	}

	/* L2 SPM is not bound to any CPU, voltage setting is not supported */

	return -EOPNOTSUPP;
}

static int spm_register_regulator(struct device *dev, struct spm_driver_data *drv)
{
	struct regulator_config config = {
		.dev = dev,
		.driver_data = drv,
	};
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	int ret;
	bool found;

	if (!drv->reg_data->set_vdd)
		return 0;

	rdesc = devm_kzalloc(dev, sizeof(*rdesc), GFP_KERNEL);
	if (!rdesc)
		return -ENOMEM;

	rdesc->name = "spm";
	rdesc->of_match = of_match_ptr("regulator");
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->owner = THIS_MODULE;
	rdesc->ops = &spm_reg_ops;

	rdesc->linear_ranges = drv->reg_data->range;
	rdesc->n_linear_ranges = 1;
	rdesc->n_voltages = rdesc->linear_ranges[rdesc->n_linear_ranges - 1].max_sel + 1;
	rdesc->ramp_delay = drv->reg_data->ramp_delay;

	ret = spm_get_cpu(dev);
	if (ret >= 0) {
		drv->reg_cpu = ret;
		dev_dbg(dev, "SAW2 regulator bound to CPU %d\n", drv->reg_cpu);
	} else {
		drv->reg_cpu = -1;
		dev_dbg(dev, "SAW2 L2 regulator (shared rail)\n");
	}

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		drv->debugfs = debugfs_create_dir(dev_name(dev), NULL);
		debugfs_create_file("rail", 0444, drv->debugfs, drv,
				    &spm_stats_fops);
	}

	ret = linear_range_get_selector_high(drv->reg_data->range,
					     drv->reg_data->init_uV,
					     &drv->volt_sel,
					     &found);
	if (ret) {
		dev_err(dev, "Initial uV value out of bounds\n");
		return ret;
	}
	drv->volt_sel_req = drv->volt_sel;

	/*
	 * Program initial voltage to hardware so the SAW2 VCTL register
	 * matches the value we report via get_voltage_sel.
	 */
	if (drv->reg_data->set_vdd) {
		if (drv->reg_cpu >= 0)
			smp_call_function_single(drv->reg_cpu,
						 drv->reg_data->set_vdd,
						 drv, true);
		else
			drv->reg_data->set_vdd(drv);
	}

	rdev = devm_regulator_register(dev, rdesc, &config);
	if (IS_ERR(rdev)) {
		dev_err(dev, "failed to register regulator\n");
		return PTR_ERR(rdev);
	}

	return 0;
}

static const struct of_device_id spm_match_table[] = {
	{ .compatible = "qcom,sdm660-gold-saw2-v4.1-l2",
	  .data = &spm_reg_660_gold_l2 },
	{ .compatible = "qcom,sdm660-silver-saw2-v4.1-l2",
	  .data = &spm_reg_660_silver_l2 },
	{ .compatible = "qcom,msm8226-saw2-v2.1-cpu",
	  .data = &spm_reg_8226_cpu },
	{ .compatible = "qcom,msm8909-saw2-v3.0-cpu",
	  .data = &spm_reg_8909_cpu },
	{ .compatible = "qcom,msm8916-saw2-v3.0-cpu",
	  .data = &spm_reg_8916_cpu },
	{ .compatible = "qcom,msm8939-saw2-v3.0-cpu",
	  .data = &spm_reg_8939_cpu },
	{ .compatible = "qcom,msm8974-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,msm8974-saw2-v2.1-l2",
	  .data = &spm_reg_8974_8084_l2 },
	{ .compatible = "qcom,msm8976-gold-saw2-v2.3-l2",
	  .data = &spm_reg_8976_gold_l2 },
	{ .compatible = "qcom,msm8976-silver-saw2-v2.3-l2",
	  .data = &spm_reg_8976_silver_l2 },
	{ .compatible = "qcom,msm8998-gold-saw2-v4.1-l2",
	  .data = &spm_reg_8998_gold_l2 },
	{ .compatible = "qcom,msm8998-silver-saw2-v4.1-l2",
	  .data = &spm_reg_8998_silver_l2 },
	{ .compatible = "qcom,apq8084-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,apq8064-saw2-v1.1-cpu",
	  .data = &spm_reg_8064_cpu },
	{ },
};
MODULE_DEVICE_TABLE(of, spm_match_table);

static int spm_dev_probe(struct platform_device *pdev)
{
	const struct of_device_id *match_id;
	struct spm_driver_data *drv;
	void __iomem *addr;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(drv->reg_base))
		return PTR_ERR(drv->reg_base);

	match_id = of_match_node(spm_match_table, pdev->dev.of_node);
	if (!match_id)
		return -ENODEV;

	drv->reg_data = match_id->data;
	drv->dev = &pdev->dev;
	platform_set_drvdata(pdev, drv);

	/* Write the SPM sequences first.. */
	addr = drv->reg_base + drv->reg_data->reg_offset[SPM_REG_SEQ_ENTRY];
	__iowrite32_copy(addr, drv->reg_data->seq,
			ARRAY_SIZE(drv->reg_data->seq) / 4);

	/*
	 * ..and then the control registers.
	 * On some SoC if the control registers are written first and if the
	 * CPU was held in reset, the reset signal could trigger the SPM state
	 * machine, before the sequences are completely written.
	 */
	/*
	 * Write these whenever the SoC maps them, rather than only when the
	 * value is nonzero: zero is a meaningful value here - it means "AVS
	 * off" - and skipping it leaves whatever the bootloader armed in place.
	 */
	if (drv->reg_data->reg_offset[SPM_REG_AVS_CTL])
		spm_register_write(drv, SPM_REG_AVS_CTL, drv->reg_data->avs_ctl);
	if (drv->reg_data->reg_offset[SPM_REG_AVS_LIMIT])
		spm_register_write(drv, SPM_REG_AVS_LIMIT, drv->reg_data->avs_limit);
	spm_register_write(drv, SPM_REG_CFG, drv->reg_data->spm_cfg);
	spm_register_write(drv, SPM_REG_DLY, drv->reg_data->spm_dly);
	spm_register_write(drv, SPM_REG_PMIC_DLY, drv->reg_data->pmic_dly);
	spm_register_write(drv, SPM_REG_PMIC_DATA_0,
				drv->reg_data->pmic_data[0]);
	spm_register_write(drv, SPM_REG_PMIC_DATA_1,
				drv->reg_data->pmic_data[1]);

	/* Set up Standby as the default low power mode */
	if (drv->reg_data->reg_offset[SPM_REG_SPM_CTL])
		spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);

	/*
	 * MSM8974 gang rail: put the SMPS into its heavy-load configuration
	 * (PWM mode, all four phases) once at boot.  The bootloader leaves the
	 * buck in a light-load configuration; it holds up under idle and
	 * frequency transitions, but sustained multi-core load draws more
	 * current than that configuration can supply and the SoC-side rail
	 * droops into a silent PS_HOLD reset while VBAT stays healthy
	 * (measured: identical 4-core load dies in seconds to minutes in the
	 * boot configuration and runs indefinitely in PWM/4-phase).  The vendor
	 * reaches the same state through its per-transition phase management;
	 * setting it statically avoids that machinery (a per-transition port
	 * write A/B-tested worse here) at the cost of PFM idle efficiency.
	 */
	if (drv->reg_data == &spm_reg_8974_8084_l2) {
		if (spm_saw_write_pmic_port(drv, SPM_SAW_FTS_PORT, SPM_FTS_PWM))
			dev_warn(&pdev->dev, "failed to set the SMPS to PWM\n");
		udelay(50);	/* PFM -> PWM exit settle */
		if (spm_saw_write_pmic_port(drv, SPM_SAW_PHASE_PORT, 3))
			dev_warn(&pdev->dev, "failed to set 4-phase mode\n");
		udelay(50);	/* phase-count raise settle */
	}

	if (IS_ENABLED(CONFIG_REGULATOR))
		return spm_register_regulator(&pdev->dev, drv);

	return 0;
}

static struct platform_driver spm_driver = {
	.probe = spm_dev_probe,
	.driver = {
		.name = "qcom_spm",
		.of_match_table = spm_match_table,
	},
};

static int __init qcom_spm_init(void)
{
	return platform_driver_register(&spm_driver);
}
arch_initcall(qcom_spm_init);

MODULE_DESCRIPTION("Qualcomm Subsystem Power Manager (SPM)");
MODULE_LICENSE("GPL v2");
