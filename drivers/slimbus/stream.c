// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited
// TODO:
// 	Add support to natural frequencies like 11.025KHz and 44.1KHz.
// 	Add support to PUSH/PULL transport protocol
//	Bandwidth Management
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/slimbus.h>
#include "slimbus.h"

struct slim_stream_runtime *slim_stream_allocate(struct slim_device *dev,
						 struct slim_stream_config *cfg)
{
	struct slim_stream_runtime *rt;
	struct slim_controller *ctrl = dev->ctrl;
	int num_ports, i, port_id;

	num_ports = hweight32(cfg->port_mask);
	rt = kzalloc(sizeof(*rt) + num_ports * sizeof(*rt->ports), GFP_KERNEL);
	if (!rt)
		return ERR_PTR(-ENOMEM);

	rt->dev = dev;
	rt->num_ports = num_ports;
	rt->rate = cfg->rate;
	rt->prot = cfg->prot;
	rt->bps = cfg->bps;
	rt->direction = cfg->direction;

	i = 0;
	//FIXME ONLY Natural frequencies of 8KHz to 48KHz are supported!!
	rt->ratem = cfg->rate/ctrl->a_framer->superfreq;

	for_each_set_bit(port_id, &cfg->port_mask, SLIM_MAX_PORTS) {
		rt->ports[i].state = SLIM_PORT_DISCONNECTED;
		rt->ports[i].id = port_id;
		rt->ports[i].direction = SLIM_PORT_SINK;
		rt->ports[i].ch.id = cfg->chs[i];
		rt->ports[i].ch.state = SLIM_CH_ALLOCATED;

		i++;
	}

	return rt;
}
EXPORT_SYMBOL_GPL(slim_stream_allocate);

static int slim_connect_sink(struct slim_stream_runtime *stream,
			     struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[2];

	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_CONNECT_SINK;
	txn.rl = 6;
	txn.msg = &msg;
	txn.msg->num_bytes = 2;
	txn.msg->wbuf = wbuf;
	
	wbuf[0] = port->id;
	wbuf[1] = port->ch.id;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_connect_source(struct slim_stream_runtime *stream,
			       struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[2];

	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_CONNECT_SOURCE;
	txn.rl = 6;
	txn.msg = &msg;
	txn.msg->num_bytes = 2;
	txn.msg->wbuf = wbuf;
	
	wbuf[0] = port->id;
	wbuf[1] = port->ch.id;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_disconnect_port(struct slim_stream_runtime *stream,
				struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[1];

	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_DISCONNECT_PORT;
	txn.rl = 5;
	txn.msg = &msg;
	txn.msg->num_bytes = 1;
	txn.msg->wbuf = wbuf;
	wbuf[0] = port->id;
	return slim_do_transfer(sdev->ctrl, &txn);
}

int slim_stream_prepare(struct slim_stream_runtime *stream)
{
	int i;

	for (i = 0; i < stream->num_ports; i++) {
		if (stream->direction == SLIM_STREAM_PLAYBACK)
			slim_connect_sink(stream, &stream->ports[i]);
		else if (stream->direction == SLIM_STREAM_CAPTURE)
			slim_connect_source(stream, &stream->ports[i]);
		else 
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_prepare);

/* 
 * Presense Rate
 * Table 66 from SLIMbus 2.0 Specs
 */
static int prrate_table[] = 
{
	12000,	0x01,
	24000,	0x02,
	48000,	0x03,
	96000,	0x04,
	192000,	0x05,
	384000,	0x06,
	768000,	0x07,
	110250,	0x09,
	220500,	0x0a,
	441000,	0x0b,
	882000,	0x0c,
	176400,	0x0d,
	352800,	0x0e,
	705600,	0x0f,
	4000, 0x10,
	8000, 0x11,
	16000, 0x12,
	32000, 0x13,
	64000, 0x14,
	128000, 0x15,
	256000, 0x16,
	512000, 0x17,
};

int slim_find_prrate(int rate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(prrate_table);) {
		if (rate == prrate_table[i])
			return prrate_table[i+1];
		i += 2;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(slim_find_prrate);

static int slim_define_channel_content(struct slim_stream_runtime *stream,
				       struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[4];
	int prrate;
	int fl = 1; /* Frequency Locked for ISO Protocol */
	prrate = slim_find_prrate(stream->rate);

	port->ch.prrate = prrate;
	
	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_NEXT_DEFINE_CONTENT;
	txn.rl = 8;
	txn.msg = &msg;
	txn.msg->num_bytes = 4;
	txn.msg->wbuf = wbuf;
	wbuf[0] = port->ch.id;
	wbuf[1] = prrate | (fl << 7);
	wbuf[2] = 0;
	// FIXME.. slc->prop.dataf | (slc->prop.auxf << 4);
	wbuf[3] = stream->bps/SLIM_SLOT_LEN_BITS;

	return slim_do_transfer(sdev->ctrl, &txn);
}

struct segdist_code {
	/* Channel rate multiplier */
	int ratem;
	int segdist_code;
	u32 seg_offset_mask;

};

/* Table 20 from SLIMbus Specs Version 2.0 */
static struct segdist_code segdist_codes[] = {
	{1,	 0x200,	 0xdff},	
	{2,	 0x100,	 0xcff},	
	{4,	 0x080,	 0xc7f},	
	{8,	 0x040,	 0xc3f},	
	{16,	 0x020,	 0xc1f},	
	{32,	 0x010,	 0xc0f},	
	{64,	 0x008,	 0xc07},	
	{128,	 0x004,	 0xc03},	
	{256,	 0x002,	 0xc01},	
	{512,	 0x001,	 0xc00},	
	{3,	 0xe00,	 0x1ff},	
	{6,	 0xd00,	 0x0ff},	
	{12,	 0xc80,	 0x07f},	
	{24,	 0xc40,	 0x03f},	
	{48,	 0xc20,	 0x01f},	
	{96,	 0xc10,	 0x00f},	
	{192,	 0xc08,	 0x007},	
	{364,	 0xc04,	 0x003},	
	{768,	 0xc02,	 0x001},	
};

static int slim_get_segdist_code(int ratem)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(segdist_codes); i++) {
		if (segdist_codes[i].ratem == ratem)
			return segdist_codes[i].segdist_code;
	}

	return -EINVAL;
}

static int slim_define_channel(struct slim_stream_runtime *stream,
				       struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[4];
	int sd;
	int tp = 0;

	txn.dt = SLIM_MSG_DEST_BROADCAST;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_NEXT_DEFINE_CHANNEL;
	txn.rl = 8;
	txn.msg = &msg;
	txn.msg->num_bytes = 4;
	txn.msg->wbuf = wbuf;
	sd = slim_get_segdist_code(stream->ratem);
	wbuf[0] = port->ch.id;

	port->ch.seg_dist = sd;
	wbuf[1] = sd & 0xFF;
	/* Only Isochronous Protocol supported */
	wbuf[2] = tp | ((sd & 0xF00) >> 8);

	/* Only one data line is supported */
	wbuf[3] = stream->bps/SLIM_SLOT_LEN_BITS;


	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_activate_channel(struct slim_stream_runtime *stream,
				 struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	struct slim_val_inf msg = {0, 0, NULL, NULL};
	struct slim_msg_txn txn = {0,};
	u8 wbuf[1];

	txn.dt = SLIM_MSG_DEST_BROADCAST;
	txn.la = stream->dev->laddr;
	txn.ec = 0;
	txn.mc = SLIM_MSG_MC_NEXT_ACTIVATE_CHANNEL;
	txn.rl = 5;
	txn.msg = &msg;
	txn.msg->num_bytes = 1;
	txn.msg->wbuf = wbuf;
	wbuf[0] = port->ch.id;

	return slim_do_transfer(sdev->ctrl, &txn);
}

int slim_stream_enable(struct slim_stream_runtime *stream)
{
	DEFINE_SLIM_BCAST_TXN(txn, SLIM_MSG_MC_BEGIN_RECONFIGURATION,
				3, SLIM_LA_MANAGER, NULL);
	struct slim_controller *ctrl = stream->dev->ctrl;
	int ret, i;

	if (ctrl->enable_stream)
		return ctrl->enable_stream(stream);

	ret = slim_do_transfer(ctrl, &txn);
	if (ret)
		return ret;

	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];
		slim_define_channel(stream, port);
		slim_define_channel_content(stream, port);
	}

	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];
		slim_activate_channel(stream, port);
	}
	txn.mc = SLIM_MSG_MC_RECONFIGURE_NOW;
	txn.rl = 3;

	return slim_do_transfer(ctrl, &txn);
}
EXPORT_SYMBOL_GPL(slim_stream_enable);

int slim_stream_disable(struct slim_stream_runtime *stream)
{
	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_disable);

int slim_stream_unprepare(struct slim_stream_runtime *stream)
{
	int i;
	for (i = 0; i < stream->num_ports; i++)
		slim_disconnect_port(stream, &stream->ports[i]);

	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_unprepare);

int slim_stream_free(struct slim_stream_runtime *stream)
{
	kfree(stream);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_free);
