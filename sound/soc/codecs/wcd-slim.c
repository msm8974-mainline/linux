#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include "wcd-slim.h"

static int wcd_slim_alloc_slim_sh_ch(struct slim_device *slim,
				    u8 wcd_slim_pgd_la, u32 cnt,
				    struct wcd_slim_ch *channels, u32 path)
{
	int ret = 0;
	u32 ch_idx ;

	/* The slimbus channel allocation seem take longer time
	 * so do the allocation up front to avoid delay in start of
	 * playback
	 */
	for (ch_idx = 0; ch_idx < cnt; ch_idx++) {
		ret = slim_get_slaveport(wcd_slim_pgd_la,
					channels[ch_idx].port,
					&channels[ch_idx].sph, path);
		if (ret < 0) {
			pr_err("%s: slave port failure id[%d] ret[%d]\n",
				__func__, channels[ch_idx].ch_num, ret);
			goto err;
		}

		ret = slim_query_ch(slim,
				    channels[ch_idx].ch_num,
				    &channels[ch_idx].ch_h);
		if (ret < 0) {
			pr_err("%s: slim_query_ch failed ch-num[%d] ret[%d]\n",
				__func__, channels[ch_idx].ch_num, ret);
			goto err;
		}
	}
err:
	return ret;
}

static int wcd_slim_dealloc_slim_sh_ch(struct slim_device *slim,
			u32 cnt, struct wcd_slim_ch *channels)
{
	int idx = 0;
	int ret = 0;
	/* slim_dealloc_ch */
	for (idx = 0; idx < cnt; idx++) {
		ret = slim_dealloc_ch(slim, channels[idx].ch_h);
		if (ret < 0) {
			pr_err("%s: slim_dealloc_ch fail ret[%d] ch_h[%d]\n",
				__func__, ret, channels[idx].ch_h);
		}
	}
	return ret;
}

int wcd_slim_init_slimslave(struct wcd_slim_data *wcd, u8 wcd_slim_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot)
{
	int ret = 0;
	int i;

	if (wcd->rx_chs) {
		wcd->num_rx_port = rx_num;
		for (i = 0; i < rx_num; i++) {
			wcd->rx_chs[i].ch_num = rx_slot[i];
			INIT_LIST_HEAD(&wcd->rx_chs[i].list);
		}

		ret = wcd_slim_alloc_slim_sh_ch(wcd->slim, wcd_slim_pgd_la,
						wcd->num_rx_port,
						wcd->rx_chs,
						SLIM_SINK);
		if (ret) {
			pr_err("%s: Failed to alloc %d rx slimbus channels\n",
				__func__, wcd->num_rx_port);
			kfree(wcd->rx_chs);
			wcd->rx_chs = NULL;
			wcd->num_rx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus rx ports\n",
			wcd->num_rx_port);
	}

	if (wcd->tx_chs) {
		wcd->num_tx_port = tx_num;
		for (i = 0; i < tx_num; i++) {
			wcd->tx_chs[i].ch_num = tx_slot[i];
			INIT_LIST_HEAD(&wcd->tx_chs[i].list);
		}
		ret = wcd_slim_alloc_slim_sh_ch(wcd->slim, wcd_slim_pgd_la,
						wcd->num_tx_port,
						wcd->tx_chs,
						SLIM_SRC);
		if (ret) {
			pr_err("%s: Failed to alloc %d tx slimbus channels\n",
				__func__, wcd->num_tx_port);
			kfree(wcd->tx_chs);
			wcd->tx_chs = NULL;
			wcd->num_tx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus tx ports\n",
			wcd->num_tx_port);
	}
	return 0;
}

int wcd_slim_deinit_slimslave(struct wcd_slim_data *wcd)
{
	if (wcd->num_rx_port) {
		wcd_slim_dealloc_slim_sh_ch(wcd->slim,
					wcd->num_rx_port,
					wcd->rx_chs);
		wcd->num_rx_port = 0;
	}
	if (wcd->num_tx_port) {
		wcd_slim_dealloc_slim_sh_ch(wcd->slim,
					wcd->num_tx_port,
					wcd->tx_chs);
		wcd->num_tx_port = 0;
	}
	return 0;
}

/* Enable slimbus slave device for RX path */
int wcd_slim_cfg_slim_sch_rx(struct wcd_slim_data *wcd,
			    struct list_head *wcd_slim_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph)
{
	u8 ch_cnt = 0;
	u16 ch_h[SLIM_MAX_RX_PORTS] = {0};
	u8  payload = 0;
	u16 codec_port = 0;
	int ret;
	struct slim_ch prop;
	struct wcd_slim_ch *rx;
	int size = ARRAY_SIZE(ch_h);

	/* Configure slave interface device */
	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		payload |= 1 << rx->shift;
		if (ch_cnt < size) {
			ch_h[ch_cnt] = rx->ch_h;
			ch_cnt++;
			pr_debug("list ch->ch_h %d ch->sph %d\n",
				 rx->ch_h, rx->sph);
		} else {
			pr_err("%s: allocated channel number %u is out of max rangae %d\n",
			       __func__, ch_cnt,
			       size);
			ret = EINVAL;
			goto err;
		}
	}

	pr_debug("%s: ch_cnt[%d] rate=%d WATER_MARK_VAL %d\n",
		 __func__, ch_cnt, rate, WATER_MARK_VAL);
	/* slim_define_ch api */
	prop.prot = SLIM_AUTO_ISO;
	if (rate == 44100) {
		prop.baser = SLIM_RATE_11025HZ;
		prop.ratem = (rate/11025);
	} else {
		prop.baser = SLIM_RATE_4000HZ;
		prop.ratem = (rate/4000);
	}
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.sampleszbits = bit_width;

	pr_debug("Before slim_define_ch:\n"
		 "ch_cnt %d,ch_h[0] %d ch_h[1] %d, grph %d\n",
		 ch_cnt, ch_h[0], ch_h[1], *grph);

	ret = slim_define_ch(wcd->slim, &prop, ch_h, ch_cnt,
			     true, grph);
	if (ret < 0) {
		pr_err("%s: slim_define_ch failed ret[%d]\n",
		       __func__, ret);
		goto err;
	}

	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		codec_port = rx->port;
		pr_debug("%s: codec_port %d rx 0x%p, payload %d\n"
			 "wcd->rx_port_ch_reg_base0 0x%x\n"
			 "wcd->port_rx_cfg_reg_base 0x%x\n",
			 __func__, codec_port, rx, payload,
			 wcd->rx_port_ch_reg_base,
			wcd->port_rx_cfg_reg_base);

		/* look for the valid port range and chose the
		 * payload accordingly
		 */
		/* write to interface device */
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd->rx_port_ch_reg_base, codec_port),
				payload);

		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd->rx_port_ch_reg_base, codec_port),
				payload, ret);
			goto err;
		}
		/* configure the slave port for water mark and enable*/
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				wcd->port_rx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}

		ret = slim_connect_sink(wcd->slim, &rx->sph, 1, rx->ch_h);
		if (ret < 0) {
			pr_err("%s: slim_connect_sink failed ret[%d]\n",
				__func__, ret);
			goto err_close_slim_sch;
		}
	}
	/* slim_control_ch */
	ret = slim_control_ch(wcd->slim, *grph, SLIM_CH_ACTIVATE,
			      true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err_close_slim_sch;
	}
	return 0;

err_close_slim_sch:
	/*  release all acquired handles */
	wcd_slim_close_slim_sch_rx(wcd, wcd_slim_ch_list, *grph);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_cfg_slim_sch_rx);

/* Enable slimbus slave device for RX path */
int wcd_slim_cfg_slim_sch_tx(struct wcd_slim_data *wcd,
			    struct list_head *wcd_slim_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph)
{
	u16 ch_cnt = 0;
	u16 payload = 0;
	u16 ch_h[SLIM_MAX_TX_PORTS] = {0};
	u16 codec_port;
	int ret = 0;
	struct wcd_slim_ch *tx;
	int size = ARRAY_SIZE(ch_h);

	struct slim_ch prop;

	list_for_each_entry(tx, wcd_slim_ch_list, list) {
		payload |= 1 << tx->shift;
		if (ch_cnt < size) {
			ch_h[ch_cnt] = tx->ch_h;
			ch_cnt++;
		} else {
			pr_err("%s: allocated channel number %u is out of max rangae %d\n",
			       __func__, ch_cnt,
			       size);
			ret = EINVAL;
			goto err;
		}
	}

	/* slim_define_ch api */
	prop.prot = SLIM_AUTO_ISO;
	prop.baser = SLIM_RATE_4000HZ;
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.ratem = (rate/4000);
	prop.sampleszbits = bit_width;
	ret = slim_define_ch(wcd->slim, &prop, ch_h, ch_cnt,
			     true, grph);
	if (ret < 0) {
		pr_err("%s: slim_define_ch failed ret[%d]\n",
		       __func__, ret);
		goto err;
	}

	pr_debug("%s: ch_cnt[%d] rate[%d] bitwidth[%u]\n", __func__, ch_cnt,
		 rate, bit_width);
	list_for_each_entry(tx, wcd_slim_ch_list, list) {
		codec_port = tx->port;
		pr_debug("%s: codec_port %d tx 0x%p, payload 0x%x\n",
			 __func__, codec_port, tx, payload);
		/* write to interface device */
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload & 0x00FF);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload, ret);
			goto err;
		}
		/* ports 8,9 */
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				(payload & 0xFF00)>>8);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				payload, ret);
			goto err;
		}
		/* configure the slave port for water mark and enable*/
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				wcd->port_tx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}

		ret = slim_connect_src(wcd->slim, tx->sph, tx->ch_h);

		if (ret < 0) {
			pr_err("%s: slim_connect_src failed ret[%d]\n",
			       __func__, ret);
			goto err;
		}
	}
	/* slim_control_ch */
	ret = slim_control_ch(wcd->slim, *grph, SLIM_CH_ACTIVATE,
			      true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err;
	}
	return 0;
err:
	/* release all acquired handles */
	wcd_slim_close_slim_sch_tx(wcd, wcd_slim_ch_list, *grph);
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_cfg_slim_sch_tx);

int wcd_slim_close_slim_sch_rx(struct wcd_slim_data *wcd,
			      struct list_head *wcd_slim_ch_list, u16 grph)
{
	u32 sph[SLIM_MAX_RX_PORTS] = {0};
	int ch_cnt = 0 ;
	int ret = 0;
	struct wcd_slim_ch *rx;

	list_for_each_entry(rx, wcd_slim_ch_list, list)
		sph[ch_cnt++] = rx->sph;

	pr_debug("%s ch_cht %d, sph[0] %d sph[1] %d\n", __func__, ch_cnt,
		sph[0], sph[1]);

	/* slim_control_ch (REMOVE) */
	pr_debug("%s before slim_control_ch grph %d\n", __func__, grph);
	ret = slim_control_ch(wcd->slim, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n", __func__, ret);
		goto err;
	}
err:
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_close_slim_sch_rx);

int wcd_slim_close_slim_sch_tx(struct wcd_slim_data *wcd,
			      struct list_head *wcd_slim_ch_list,
			      u16 grph)
{
	u32 sph[SLIM_MAX_TX_PORTS] = {0};
	int ret = 0;
	int ch_cnt = 0 ;
	struct wcd_slim_ch *tx;

	pr_debug("%s\n", __func__);
	list_for_each_entry(tx, wcd_slim_ch_list, list)
		sph[ch_cnt++] = tx->sph;

	pr_debug("%s ch_cht %d, sph[0] %d sph[1] %d\n",
		__func__, ch_cnt, sph[0], sph[1]);
	/* slim_control_ch (REMOVE) */
	ret = slim_control_ch(wcd->slim, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err;
	}
err:
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_close_slim_sch_tx);

int wcd_slim_get_slave_port(unsigned int ch_num)
{
	int ret = 0;

	ret = (ch_num - BASE_CH_NUM);
	if (ret < 0) {
		pr_err("%s: Error:- Invalid slave port found = %d\n",
			__func__, ret);
		return -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_get_slave_port);

int wcd_slim_disconnect_port(struct wcd_slim_data *wcd,
			    struct list_head *wcd_slim_ch_list, u16 grph)
{
	u32 sph[SLIM_MAX_TX_PORTS + SLIM_MAX_RX_PORTS] = {0};
	int ch_cnt = 0 ;
	int ret = 0;
	struct wcd_slim_ch *slim_ch;

	list_for_each_entry(slim_ch, wcd_slim_ch_list, list)
		sph[ch_cnt++] = slim_ch->sph;

	/* slim_disconnect_port */
	ret = slim_disconnect_ports(wcd->slim, sph, ch_cnt);
	if (ret < 0) {
		pr_err("%s: slim_disconnect_ports failed ret[%d]\n",
			__func__, ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_disconnect_port);

/* This function is called with mutex acquired */
int wcd_slim_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list)
{
	struct wcd_slim_ch *ch;
	int ret = 0;

	list_for_each_entry(ch,
		codec_dai_list, list) {
		if (ch->port == port_id) {
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(wcd_slim_rx_vport_validation);
