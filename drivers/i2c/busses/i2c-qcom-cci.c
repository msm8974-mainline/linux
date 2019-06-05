/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define CCI_HW_VERSION			0x0
#define CCI_RESET_CMD			0x004
#define CCI_RESET_CMD_MASK		0x0f73f3f7
#define CCI_RESET_CMD_M0_MASK		0x000003f1
#define CCI_RESET_CMD_M1_MASK		0x0003f001
#define CCI_QUEUE_START			0x008
#define CCI_HALT_REQ			0x034
#define CCI_HALT_REQ_I2C_M0_Q0Q1	(1 << 0)
#define CCI_HALT_REQ_I2C_M1_Q0Q1	(1 << 1)

#define CCI_I2C_Mm_SCL_CTL(m)		(0x100 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_0(m)		(0x104 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_1(m)		(0x108 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_2(m)		(0x10c + 0x100 * (m))
#define CCI_I2C_Mm_MISC_CTL(m)		(0x110 + 0x100 * (m))

#define CCI_I2C_Mm_READ_DATA(m)			(0x118 + 0x100 * (m))
#define CCI_I2C_Mm_READ_BUF_LEVEL(m)		(0x11c + 0x100 * (m))
#define CCI_I2C_Mm_Qn_EXEC_WORD_CNT(m, n)	(0x300 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_CUR_WORD_CNT(m, n)	(0x304 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_CUR_CMD(m, n)		(0x308 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_REPORT_STATUS(m, n)	(0x30c + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_LOAD_DATA(m, n)		(0x310 + 0x200 * (m) + 0x100 * (n))

#define CCI_IRQ_GLOBAL_CLEAR_CMD	0xc00
#define CCI_IRQ_MASK_0			0xc04
#define CCI_IRQ_MASK_0_I2C_M0_RD_DONE		(1 << 0)
#define CCI_IRQ_MASK_0_I2C_M0_Q0_REPORT		(1 << 4)
#define CCI_IRQ_MASK_0_I2C_M0_Q1_REPORT		(1 << 8)
#define CCI_IRQ_MASK_0_I2C_M1_RD_DONE		(1 << 12)
#define CCI_IRQ_MASK_0_I2C_M1_Q0_REPORT		(1 << 16)
#define CCI_IRQ_MASK_0_I2C_M1_Q1_REPORT		(1 << 20)
#define CCI_IRQ_MASK_0_RST_DONE_ACK		(1 << 24)
#define CCI_IRQ_MASK_0_I2C_M0_Q0Q1_HALT_ACK	(1 << 25)
#define CCI_IRQ_MASK_0_I2C_M1_Q0Q1_HALT_ACK	(1 << 26)
#define CCI_IRQ_MASK_0_I2C_M0_ERROR		0x18000ee6
#define CCI_IRQ_MASK_0_I2C_M1_ERROR		0x60ee6000
#define CCI_IRQ_CLEAR_0			0xc08
#define CCI_IRQ_STATUS_0		0xc0c
#define CCI_IRQ_STATUS_0_I2C_M0_RD_DONE		(1 << 0)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT	(1 << 4)
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT	(1 << 8)
#define CCI_IRQ_STATUS_0_I2C_M1_RD_DONE		(1 << 12)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT	(1 << 16)
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT	(1 << 20)
#define CCI_IRQ_STATUS_0_RST_DONE_ACK		(1 << 24)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK	(1 << 25)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK	(1 << 26)
#define CCI_IRQ_STATUS_0_I2C_M0_ERROR		0x18000ee6
#define CCI_IRQ_STATUS_0_I2C_M1_ERROR		0x60ee6000

#define CCI_TIMEOUT_MS 100
#define NUM_MASTERS 2
#define NUM_QUEUES 2

#define CCI_RES_MAX 6

enum cci_i2c_cmd {
	CCI_I2C_SET_PARAM = 1,
	CCI_I2C_WAIT,
	CCI_I2C_WAIT_SYNC,
	CCI_I2C_WAIT_GPIO_EVENT,
	CCI_I2C_TRIG_I2C_EVENT,
	CCI_I2C_LOCK,
	CCI_I2C_UNLOCK,
	CCI_I2C_REPORT,
	CCI_I2C_WRITE,
	CCI_I2C_READ,
	CCI_I2C_WRITE_DISABLE_P,
	CCI_I2C_READ_DISABLE_P,
};

enum {
	I2C_MODE_STANDARD,
	I2C_MODE_FAST,
	I2C_MODE_FAST_PLUS,
};

enum cci_i2c_queue_t {
	QUEUE_0,
	QUEUE_1
};

enum cci_i2c_master_t {
	MASTER_0,
	MASTER_1
};

struct resources {
	char *clock[CCI_RES_MAX];
	u32 clock_rate[CCI_RES_MAX];
	char *reg[CCI_RES_MAX];
	char *interrupt[CCI_RES_MAX];
};

struct hw_params {
	u16 thigh;
	u16 tlow;
	u16 tsu_sto;
	u16 tsu_sta;
	u16 thd_dat;
	u16 thd_sta;
	u16 tbuf;
	u8 scl_stretch_en;
	u16 trdhld;
	u16 tsp;
};

struct cci_clock {
	struct clk *clk;
	const char *name;
	u32 freq;
};

struct cci_master {
	u32 status;
	u8 complete_pending;
	struct completion irq_complete;
};

struct cci {
	struct device *dev;
	struct i2c_adapter adap;
	void __iomem *base;
	u32 irq;
	char irq_name[30];
	struct cci_clock *clock;
	int nclocks;
	u8 mode;
	u16 queue_size[NUM_QUEUES];
	struct cci_master master[NUM_MASTERS];
};

static const struct resources res_v1_0_8 = {
	.clock = { "camss_top_ahb",
		   "cci_ahb",
		   // "camss_ahb",
		   "cci" },
	.clock_rate = { 0,
			80000000,
			// 0,
			19200000 },
	.reg = { "cci" },
	.interrupt = { "cci" }
};

static const struct resources res_v1_4_0 = {
	.clock = { "camss_top_ahb",
		   "cci_ahb",
		   "camss_ahb",
		   "cci" },
	.clock_rate = { 0,
			0,
			0,
			37500000 },
	.reg = { "cci" },
	.interrupt = { "cci" }
};

static const struct hw_params hw_params_v1_0_8[3] = {
	{	/* I2C_MODE_STANDARD */
		.thigh = 78,
		.tlow = 114,
		.tsu_sto = 28,
		.tsu_sta = 28,
		.thd_dat = 10,
		.thd_sta = 77,
		.tbuf = 118,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 1
	},
	{	/* I2C_MODE_FAST */
		.thigh = 20,
		.tlow = 28,
		.tsu_sto = 21,
		.tsu_sta = 21,
		.thd_dat = 13,
		.thd_sta = 18,
		.tbuf = 32, // 25 ?
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	}
};

static const struct hw_params hw_params_v1_4_0[3] = {
	{	/* I2C_MODE_STANDARD */
		.thigh = 201,
		.tlow = 174,
		.tsu_sto = 204,
		.tsu_sta = 231,
		.thd_dat = 22,
		.thd_sta = 162,
		.tbuf = 227,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	},
	{	/* I2C_MODE_FAST */
		.thigh = 38,
		.tlow = 56,
		.tsu_sto = 40,
		.tsu_sta = 40,
		.thd_dat = 22,
		.thd_sta = 35,
		.tbuf = 62,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	},
	{	/* I2C_MODE_FAST_PLUS */
		.thigh = 16,
		.tlow = 22,
		.tsu_sto = 17,
		.tsu_sta = 18,
		.thd_dat = 16,
		.thd_sta = 15,
		.tbuf = 24,
		.scl_stretch_en = 0,
		.trdhld = 3,
		.tsp = 3
	}
};

static const u16 queue_0_size_v1_0_8 = 64;
static const u16 queue_1_size_v1_0_8 = 16;

static const u16 queue_0_size_v1_4_0 = 64;
static const u16 queue_1_size_v1_4_0 = 16;

/*
 * cci_enable_clocks - Enable multiple clocks
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 * @dev: Device
 *
 * Return 0 on success or a negative error code otherwise
 */
int cci_enable_clocks(int nclocks, struct cci_clock *clock, struct device *dev)
{
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
		if (clock[i].freq) {
			long rate;

			rate = clk_round_rate(clock[i].clk, clock[i].freq);
			if (rate < 0) {
				dev_err(dev, "clk round rate failed: %ld\n",
					rate);
				goto error;
			}

			ret = clk_set_rate(clock[i].clk, clock[i].freq);
			if (ret < 0) {
				dev_err(dev, "clk set rate failed: %d\n", ret);
				goto error;
			}
		}

		ret = clk_prepare_enable(clock[i].clk);
		if (ret) {
			dev_err(dev, "clock enable failed, ret: %d\n", ret);
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		clk_disable_unprepare(clock[i].clk);

	return ret;
}

/*
 * cci_disable_clocks - Disable multiple clocks
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 */
void cci_disable_clocks(int nclocks, struct cci_clock *clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i].clk);
}

static irqreturn_t cci_isr(int irq, void *dev)
{
	struct cci *cci = dev;
	u32 val;

	val = readl(cci->base + CCI_IRQ_STATUS_0);
	writel(val, cci->base + CCI_IRQ_CLEAR_0);
	writel(0x1, cci->base + CCI_IRQ_GLOBAL_CLEAR_CMD);

	if (val & CCI_IRQ_STATUS_0_RST_DONE_ACK) {
		if (cci->master[MASTER_0].complete_pending) {
			cci->master[MASTER_0].complete_pending = 0;
			complete(&cci->master[MASTER_0].irq_complete);
		}

		if (cci->master[MASTER_1].complete_pending) {
			cci->master[MASTER_1].complete_pending = 0;
			complete(&cci->master[MASTER_1].irq_complete);
		}
	}

	if (val & CCI_IRQ_STATUS_0_I2C_M0_RD_DONE ||
			val & CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT ||
			val & CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT) {
		cci->master[MASTER_0].status = 0;
		complete(&cci->master[MASTER_0].irq_complete);
	}

	if (val & CCI_IRQ_STATUS_0_I2C_M1_RD_DONE ||
			val & CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT ||
			val & CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT) {
		cci->master[MASTER_1].status = 0;
		complete(&cci->master[MASTER_1].irq_complete);
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK)) {
		cci->master[MASTER_0].complete_pending = 1;
		writel(CCI_RESET_CMD_M0_MASK, cci->base + CCI_RESET_CMD);
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK)) {
		cci->master[MASTER_1].complete_pending = 1;
		writel(CCI_RESET_CMD_M1_MASK, cci->base + CCI_RESET_CMD);
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M0_ERROR)) {
		dev_err_ratelimited(cci->dev, "MASTER_0 error 0x%08x\n", val);
		cci->master[MASTER_0].status = -EIO;
		writel(CCI_HALT_REQ_I2C_M0_Q0Q1, cci->base + CCI_HALT_REQ);
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M1_ERROR)) {
		dev_err_ratelimited(cci->dev, "MASTER_1 error 0x%08x\n", val);
		cci->master[MASTER_1].status = -EIO;
		writel(CCI_HALT_REQ_I2C_M1_Q0Q1, cci->base + CCI_HALT_REQ);
	}

	return IRQ_HANDLED;
}

static int cci_reset(struct cci *cci)
{
	unsigned long time;

	cci->master[MASTER_0].complete_pending = 1;
	writel(CCI_RESET_CMD_MASK, cci->base + CCI_RESET_CMD);
	time = wait_for_completion_timeout(
				&cci->master[MASTER_0].irq_complete,
				msecs_to_jiffies(CCI_TIMEOUT_MS));
	if (!time) {
		dev_err(cci->dev, "CCI reset timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int cci_init(struct cci *cci, const struct hw_params *hw)
{
	u32 val = CCI_IRQ_MASK_0_I2C_M0_RD_DONE |
			CCI_IRQ_MASK_0_I2C_M0_Q0_REPORT |
			CCI_IRQ_MASK_0_I2C_M0_Q1_REPORT |
			CCI_IRQ_MASK_0_I2C_M1_RD_DONE |
			CCI_IRQ_MASK_0_I2C_M1_Q0_REPORT |
			CCI_IRQ_MASK_0_I2C_M1_Q1_REPORT |
			CCI_IRQ_MASK_0_RST_DONE_ACK |
			CCI_IRQ_MASK_0_I2C_M0_Q0Q1_HALT_ACK |
			CCI_IRQ_MASK_0_I2C_M1_Q0Q1_HALT_ACK |
			CCI_IRQ_MASK_0_I2C_M0_ERROR |
			CCI_IRQ_MASK_0_I2C_M1_ERROR;
	int i;

	writel(val, cci->base + CCI_IRQ_MASK_0);

	for (i = 0; i < NUM_MASTERS; i++) {
		val = hw->thigh << 16 | hw->tlow;
		writel(val, cci->base + CCI_I2C_Mm_SCL_CTL(i));

		val = hw->tsu_sto << 16 | hw->tsu_sta;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_0(i));

		val = hw->thd_dat << 16 | hw->thd_sta;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_1(i));

		val = hw->tbuf;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_2(i));

		val = hw->scl_stretch_en << 8 | hw->trdhld << 4 | hw->tsp;
		writel(val, cci->base + CCI_I2C_Mm_MISC_CTL(i));

		cci->master[i].status = 0;
	}

	return 0;
}

static int cci_run_queue(struct cci *cci, u8 master, u8 queue)
{
	unsigned long time;
	u32 val;
	int ret;

	val = readl(cci->base + CCI_I2C_Mm_Qn_CUR_WORD_CNT(master, queue));
	writel(val, cci->base + CCI_I2C_Mm_Qn_EXEC_WORD_CNT(master, queue));

	val = 1 << ((master * 2) + queue);
	writel(val, cci->base + CCI_QUEUE_START);

	time = wait_for_completion_timeout(&cci->master[master].irq_complete,
					   msecs_to_jiffies(CCI_TIMEOUT_MS));
	if (!time) {
		dev_err(cci->dev, "master %d queue %d timeout\n",
			master, queue);
		return -ETIMEDOUT;
	}

	ret = cci->master[master].status;
	if (ret < 0)
		dev_err(cci->dev, "master %d queue %d error %d\n",
			master, queue, ret);

	return ret;
}

static int cci_validate_queue(struct cci *cci, u32 len, u8 master, u8 queue)
{
	int ret = 0;
	u32 val;

	val = readl(cci->base + CCI_I2C_Mm_Qn_CUR_WORD_CNT(master, queue));

	if (val + len + 1 > cci->queue_size[queue]) {
		val = CCI_I2C_REPORT | (1 << 8);
		writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

		ret = cci_run_queue(cci, master, queue);
	}

	return ret;
}

static int cci_i2c_read(struct cci *cci, u16 addr, u8 *buf, u16 len)
{
	u8 master = MASTER_0;
	u8 queue = QUEUE_1;
	u32 val;
	u32 words_read, words_exp;
	int i, index, first;
	int ret;

	if (len > cci->adap.quirks->max_read_len)
		return -EOPNOTSUPP;

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * This is to avoid overflow / underflow of queue.
	 */
	ret = cci_validate_queue(cci, cci->queue_size[queue], master, queue);
	if (ret < 0)
		return ret;

	val = CCI_I2C_SET_PARAM | (addr & 0x7f) << 4;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	val = CCI_I2C_READ | len << 4;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	ret = cci_run_queue(cci, master, queue);
	if (ret < 0)
		return ret;

	words_read = readl(cci->base + CCI_I2C_Mm_READ_BUF_LEVEL(master));
	words_exp = len / 4 + 1;
	if (words_read != words_exp) {
		dev_err(cci->dev, "words read = %d, words expected = %d\n",
			words_read, words_exp);
		return -EIO;
	}

	index = 0;
	first = 1;
	do {
		val = readl(cci->base + CCI_I2C_Mm_READ_DATA(master));

		for (i = 0; i < 4 && index < len; i++) {
			if (first) {
				first = 0;
				continue;
			}
			buf[index++] = (val >> (i * 8)) & 0xff;
		}
	} while (--words_read);

	return 0;
}

static int cci_i2c_write(struct cci *cci, u16 addr, u8 *buf, u16 len)
{
	u8 master = MASTER_0;
	u8 queue = QUEUE_0;
	u8 load[12] = { 0 };
	u8 i, j;
	u32 val;
	int ret;

	if (len > cci->adap.quirks->max_write_len)
		return -EOPNOTSUPP;

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * This is to avoid overflow / underflow of queue.
	 */
	ret = cci_validate_queue(cci, cci->queue_size[queue], master, queue);
	if (ret < 0)
		return ret;

	val = CCI_I2C_SET_PARAM | (addr & 0x7f) << 4;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	i = 0;
	load[i++] = CCI_I2C_WRITE | len << 4;

	for (j = 0; j < len; j++)
		load[i++] = buf[j];

	for (j = 0; j < i; j += 4) {
		val = load[j];
		val |= load[j + 1] << 8;
		val |= load[j + 2] << 16;
		val |= load[j + 3] << 24;
		writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));
	}

	val = CCI_I2C_REPORT | 1 << 8;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	return cci_run_queue(cci, master, queue);
}

static int cci_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct cci *cci = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	if (!num)
		return -EOPNOTSUPP;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = cci_i2c_read(cci, msgs[i].addr, msgs[i].buf,
					   msgs[i].len);
		else
			ret = cci_i2c_write(cci, msgs[i].addr, msgs[i].buf,
					    msgs[i].len);

		if (ret < 0) {
			dev_err(cci->dev, "cci i2c xfer error %d", ret);
			break;
		}
	}

	if (i == num && !ret) /* all operations are successful */
		ret = num;

	return ret;
}

static u32 cci_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm cci_algo = {
	.master_xfer	= cci_xfer,
	.functionality	= cci_func,
};

static const struct i2c_adapter_quirks cci_quirks_v1_0_8 = {
	.max_write_len = 10,
	.max_read_len = 12,
};

static const struct i2c_adapter_quirks cci_quirks_v1_4_0 = {
	.max_write_len = 11,
	.max_read_len = 12,
};

/*
 * cci_probe - Probe CCI platform device
 * @pdev: Pointer to CCI platform device
 *
 * Return 0 on success or a negative error code on failure
 */
static int cci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct resources *res;
	const struct hw_params *hw;
	struct cci *cci;
	struct resource *r;
	int ret = 0;
	u32 val;
	int i;

	cci = devm_kzalloc(&pdev->dev, sizeof(*cci), GFP_KERNEL);
	if (!cci)
		return -ENOMEM;

	cci->dev = dev;
	platform_set_drvdata(pdev, cci);

	if (of_device_is_compatible(dev->of_node, "qcom,cci-v1.0.8")) {
		res = &res_v1_0_8;
		hw = hw_params_v1_0_8;
		cci->queue_size[0] = queue_0_size_v1_0_8;
		cci->queue_size[1] = queue_1_size_v1_0_8;
		cci->adap.quirks = &cci_quirks_v1_0_8;
	} else if (of_device_is_compatible(dev->of_node, "qcom,cci-v1.4.0")) {
		res = &res_v1_4_0;
		hw = hw_params_v1_4_0;
		cci->queue_size[0] = queue_0_size_v1_4_0;
		cci->queue_size[1] = queue_1_size_v1_4_0;
		cci->adap.quirks = &cci_quirks_v1_4_0;
	} else {
		return -EINVAL;
	}

	cci->adap.algo = &cci_algo;
	cci->adap.dev.parent = cci->dev;
	cci->adap.dev.of_node = dev->of_node;
	i2c_set_adapdata(&cci->adap, cci);

	strlcpy(cci->adap.name, "Qualcomm Camera Control Interface",
		sizeof(cci->adap.name));

	cci->mode = I2C_MODE_STANDARD;
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &val);
	if (!ret) {
		if (val == 400000)
			cci->mode = I2C_MODE_FAST;
		else if (val == 1000000)
			cci->mode = I2C_MODE_FAST_PLUS;
	}

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	cci->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(cci->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(cci->base);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					 res->interrupt[0]);
	if (!r) {
		dev_err(dev, "missing IRQ\n");
		return -EINVAL;
	}

	cci->irq = r->start;
	snprintf(cci->irq_name, sizeof(cci->irq_name), "%s", dev_name(dev));
	ret = devm_request_irq(dev, cci->irq, cci_isr,
			       IRQF_TRIGGER_RISING, cci->irq_name, cci);
	if (ret < 0) {
		dev_err(dev, "request_irq failed, ret: %d\n", ret);
		return ret;
	}

	disable_irq(cci->irq);

	/* Clocks */

	cci->nclocks = 0;
	while (res->clock[cci->nclocks])
		cci->nclocks++;

	cci->clock = devm_kzalloc(dev, cci->nclocks *
				  sizeof(*cci->clock), GFP_KERNEL);
	if (!cci->clock)
		return -ENOMEM;

	for (i = 0; i < cci->nclocks; i++) {
		struct cci_clock *clock = &cci->clock[i];

		clock->clk = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(clock->clk))
			return PTR_ERR(clock->clk);

		clock->name = res->clock[i];
		clock->freq = res->clock_rate[i];
	}

	ret = cci_enable_clocks(cci->nclocks, cci->clock, dev);
	if (ret < 0)
		return ret;

	val = readl_relaxed(cci->base + CCI_HW_VERSION);
	dev_info(dev, "%s: CCI HW version = 0x%08x", __func__, val);

	init_completion(&cci->master[0].irq_complete);
	init_completion(&cci->master[1].irq_complete);

	enable_irq(cci->irq);

	ret = cci_reset(cci);
	if (ret < 0)
		return ret;

	ret = cci_init(cci, &hw[cci->mode]);
	if (ret < 0)
		return ret;

	ret = i2c_add_adapter(&cci->adap);

	return ret;
}

/*
 * cci_remove - Remove CCI platform device
 * @pdev: Pointer to CCI platform device
 *
 * Always returns 0.
 */
static int cci_remove(struct platform_device *pdev)
{
	struct cci *cci = platform_get_drvdata(pdev);

	disable_irq(cci->irq);
	cci_disable_clocks(cci->nclocks, cci->clock);

	i2c_del_adapter(&cci->adap);

	return 0;
}

static const struct of_device_id cci_dt_match[] = {
	{ .compatible = "qcom,cci-v1.0.8" },
	{ .compatible = "qcom,cci-v1.4.0" },
	{}
};
MODULE_DEVICE_TABLE(of, cci_dt_match);

static struct platform_driver qcom_cci_driver = {
	.probe  = cci_probe,
	.remove = cci_remove,
	.driver = {
		.name = "i2c-qcom-cci",
		.of_match_table = cci_dt_match,
	},
};

module_platform_driver(qcom_cci_driver);

MODULE_ALIAS("platform:i2c-qcom-cci");
MODULE_DESCRIPTION("Qualcomm Camera Control Interface driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");
