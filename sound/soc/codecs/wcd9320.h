#ifndef __WCD9320_H__
#define __WCD9320_H__

#include <linux/slimbus.h>
#include <linux/regulator/consumer.h>
#include "wcd-slim.h"
#include "wcd-clsh.h"

#define WCD9320_NUM_REGISTERS 0x400
#define WCD9320_MAX_REGISTER (WCD9320_NUM_REGISTERS-1)
#define WCD9320_CACHE_SIZE WCD9320_NUM_REGISTERS

#define WCD_INTERFACE_TYPE_SLIMBUS	1

#define WCD9XXX_MCLK_CLK_9P6HZ 9600000
#define SLIMBUS_PRESENT_TIMEOUT 100
#define WCD9320_MAX_SUPPLY	8

enum {
	/* INTR_REG 0 */
	WCD9320_IRQ_SLIMBUS = 0,
	WCD9320_IRQ_FLL_LOCK_LOSS = 1,
	WCD9320_IRQ_HPH_PA_OCPL_FAULT,
	WCD9320_IRQ_HPH_PA_OCPR_FAULT,
	WCD9320_IRQ_EAR_PA_OCP_FAULT,
	WCD9320_IRQ_HPH_PA_CNPL_COMPLETE,
	WCD9320_IRQ_HPH_PA_CNPR_COMPLETE,
	WCD9320_IRQ_EAR_PA_CNP_COMPLETE,
	/* INTR_REG 1 */
	WCD9320_IRQ_MBHC_SW_DET,
	WCD9320_IRQ_MBHC_ELECT_INS_REM_DET,
	WCD9320_IRQ_MBHC_BUTTON_PRESS_DET,
	WCD9320_IRQ_MBHC_BUTTON_RELEASE_DET,
	WCD9320_IRQ_MBHC_ELECT_INS_REM_LEG_DET,
	WCD9320_IRQ_RESERVED_0,
	WCD9320_IRQ_RESERVED_1,
	WCD9320_IRQ_RESERVED_2,
	/* INTR_REG 2 */
	WCD9320_IRQ_LINE_PA1_CNP_COMPLETE,
	WCD9320_IRQ_LINE_PA2_CNP_COMPLETE,
	WCD9320_IRQ_LINE_PA3_CNP_COMPLETE,
	WCD9320_IRQ_LINE_PA4_CNP_COMPLETE,
	WCD9320_IRQ_SOUNDWIRE,
	WCD9320_IRQ_VDD_DIG_RAMP_COMPLETE,
	WCD9320_IRQ_RCO_ERROR,
	WCD9320_IRQ_SVA_ERROR,
	/* INTR_REG 3 */
	WCD9320_IRQ_MAD_AUDIO,
	WCD9320_IRQ_MAD_BEACON,
	WCD9320_IRQ_MAD_ULTRASOUND,
	WCD9320_IRQ_VBAT_ATTACK,
	WCD9320_IRQ_VBAT_RESTORE,
	WCD9320_IRQ_SVA_OUTBOX1,
	WCD9320_IRQ_SVA_OUTBOX2,
	WCD9320_NUM_IRQS,
};

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	AIF2_PB,
	AIF2_CAP,
	AIF3_PB,
	AIF3_CAP,
	AIF4_PB,
	AIF_MIX1_PB,
	AIF4_MAD_TX,
	AIF4_VIFEED,
	NUM_CODEC_DAIS,
};

#define WCD9320_NUM_INTERPOLATORS 9

enum {
	SPLINE_SRC0,
	SPLINE_SRC1,
	SPLINE_SRC2,
	SPLINE_SRC3,
	SPLINE_SRC_MAX,
};

enum wcd_clock_type {
	WCD_CLK_OFF,
	WCD_CLK_RCO,
	WCD_CLK_MCLK,
};

struct wcd9335_slim_ch {
	int id;
};

struct wcd_slim_codec_dai_data {
	u32 rate;				/* sample rate          */
	u32 bit_width;				/* sit width 16,24,32   */
	struct list_head wcd_slim_ch_list;	/* channel list         */
	u16 grph;				/* slimbus group handle */
	unsigned long ch_mask;
	unsigned long port_mask;
	wait_queue_head_t dai_wait; //FIXME
	struct slim_stream_config sconfig;
	struct slim_stream_runtime *sruntime;
};

struct wcd9320 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;

	/* data */
	int irq;
	struct clk *codec_clk;
	int reset_gpio;
	int irq_gpio;
	int clk_gpio;
	int codec_clk_rate;
	int num_of_supplies;
	struct regulator_bulk_data supplies[WCD9320_MAX_SUPPLY];

	u8 version;
	u8 intf_type;
	/* slimbus specific*/

	struct wcd_slim_data slim_data;
	struct slim_device *slim;
	struct slim_device *slim_ifd;
	struct regmap *ifd_regmap;
	struct slim_device *slim_slave;
};

#define WCD9XXX_RCO_CALIBRATION_DELAY_INC_US 5000

struct wcd9320_priv {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;

	/* data */
	struct clk *codec_clk;
	u32 codec_clk_rate;
	u8 version;


	/* slimbus specific */

	struct slim_device *slim;
	struct slim_device *slim_slave;
	struct regmap *if_regmap;
	struct wcd_slim_data *slim_data;

	u32 num_rx_port;
	u32 num_tx_port;
	struct wcd_slim_codec_dai_data  dai[NUM_CODEC_DAIS];

	/* Codec specifics */
	const struct wcd_slim_codec_type *codec_type;
	struct snd_soc_component *component;
	u32 rx_bias_count;
	/*track tasha interface type*/
	u8 intf_type;
	/*compander*/
	int comp_enabled;
	int comp_fs;
	/* class h specific data */
	struct wcd_clsh_cdc_data clsh_d;
	/* to track the status */
	unsigned long status_mask;
	/* Port values for Rx and Tx codec_dai */
	unsigned int rx_port_value;
	unsigned int tx_port_value;
	/* Tasha Interpolator Mode Select for EAR, HPH_L and HPH_R */
	u32 hph_mode;
	u16 prim_int_users[WCD9320_NUM_INTERPOLATORS];
	int spl_src_users[SPLINE_SRC_MAX];
//	struct wcd_slim_resmgr_v2 *resmgr;
///
	int master_bias_users;
	int clk_mclk_users;
	int clk_rco_users;

	struct mutex codec_bg_clk_lock;
	struct mutex master_bias_lock;
	enum wcd_clock_type clk_type;
///
	struct mutex sido_lock;
	struct mutex micb_lock;
	int native_clk_users;
	int power_active_ref;
	int hph_l_gain;
	int hph_r_gain;
};

int wcd9320_regmap_register_patch(struct regmap *regmap, int version);

#endif /* __WCD9320_H__ */
