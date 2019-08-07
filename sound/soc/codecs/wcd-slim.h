/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#ifndef __WCD_SLIMSLAVE_H_
#define __WCD_SLIMSLAVE_H_

#define WCD_SLIM_CH(xport, xshift) \
	{.port = xport, .shift = xshift}

struct wcd_slim_ch {
	u32 sph;
	u32 ch_num;
	u16 ch_h;
	u16 port;
	u16 shift;
	struct list_head list;
};

struct wcd_slim_data {
	struct slim_device *slim;
	struct slim_device *slim_slave;
	u16 rx_port_ch_reg_base;
	u16 port_tx_cfg_reg_base;
	u16 port_rx_cfg_reg_base;
	struct regmap *regmap;
	struct regmap *if_regmap;
	struct wcd_slim_ch *rx_chs;
	struct wcd_slim_ch *tx_chs;
	u32 num_rx_port;
	u32 num_tx_port;
};

#define WCD9335_RX_SLAVE_PORTS	16
#define WCD9335_TX_SLAVE_PORTS	16
#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 16

/* below details are taken from SLIMBUS slave SWI */
#define SB_PGD_PORT_BASE 0x000

#define SB_PGD_PORT_CFG_BYTE_ADDR(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (1 * port_num))

#define SB_PGD_TX_PORT_MULTI_CHANNEL_0(port_num) \
		(SB_PGD_PORT_BASE + 0x100 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_START_PORT_ID   0
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_END_PORT_ID     7

#define SB_PGD_TX_PORT_MULTI_CHANNEL_1(port_num) \
		(SB_PGD_PORT_BASE + 0x101 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_1_START_PORT_ID   8

#define SB_PGD_RX_PORT_MULTI_CHANNEL_0(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (4 * port_num))

/* slave port water mark level
 *   (0: 6bytes, 1: 9bytes, 2: 12 bytes, 3: 15 bytes)
 */
#define SLAVE_PORT_WATER_MARK_6BYTES  0
#define SLAVE_PORT_WATER_MARK_9BYTES  1
#define SLAVE_PORT_WATER_MARK_12BYTES 2
#define SLAVE_PORT_WATER_MARK_15BYTES 3
#define SLAVE_PORT_WATER_MARK_SHIFT 1
#define SLAVE_PORT_ENABLE           1
#define SLAVE_PORT_DISABLE          0
#define WATER_MARK_VAL \
	((SLAVE_PORT_WATER_MARK_12BYTES << SLAVE_PORT_WATER_MARK_SHIFT) | \
	 (SLAVE_PORT_ENABLE))
#define BASE_CH_NUM 128


struct wcd9335_priv;

extern int wcd_slim_init_slimslave(struct wcd_slim_data *sd, u8 wcd_slim_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot);

int wcd_slim_deinit_slimslave(struct wcd_slim_data *sd);

int wcd_slim_cfg_slim_sch_rx(struct wcd_slim_data *sd,
			    struct list_head *wcd_slim_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph);
int wcd_slim_cfg_slim_sch_tx(struct wcd_slim_data *sd,
			    struct list_head *wcd_slim_ch_list,
			    unsigned int rate, unsigned int bit_width,
				u16 *grph);
int wcd_slim_close_slim_sch_rx(struct wcd_slim_data *sd,
			      struct list_head *wcd_slim_ch_list, u16 grph);
int wcd_slim_close_slim_sch_tx(struct wcd_slim_data *sd,
			      struct list_head *wcd_slim_ch_list, u16 grph);
int wcd_slim_get_channel(struct wcd_slim_data *sd,
			unsigned int *rx_ch,
			unsigned int *tx_ch);
int wcd_slim_get_slave_port(unsigned int ch_num);
int wcd_slim_disconnect_port(struct wcd_slim_data *sd,
			    struct list_head *wcd_slim_ch_list, u16 grph);
int wcd_slim_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list);
#endif /* __WCD_SLIMSLAVE_H_ */
