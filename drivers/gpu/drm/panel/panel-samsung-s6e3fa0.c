// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2022, Adam Honse <calcprogrammer1@gmail.com>
// Copyright (c) 2021, Alexey Minnekhanov <alexeymin@postmarketos.org>
// Copyright (c) 2021, The Linux Foundation. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

/* Panel ID */
#define LCD_ID_S6E3FA0_1	0x414024
#define LCD_ID_S6E3FA0_2	0x404024

/* Manufacturer Command Set */
#define MCS_AID_CONTROL		0xb2  /* Samsung AMOLED Impulsive Driving */
#define MCS_ELVSS_CONTROL	0xb6  /* Amoled negative power supply */
#define MCS_GAMMA		0xca
/* Read ID commands */
#define MCS_READ_ID1		0xda
#define MCS_READ_ID2		0xdb
#define MCS_READ_ID3		0xdc

/* Number of brightness levels */
#define S6E3FA0_NUM_GAMMA_LEVELS	62
#define S6E3FA0_MAX_BRIGHTNESS		(S6E3FA0_NUM_GAMMA_LEVELS - 1)

#define NUM_AID_SEQUENCES	45
#define AID_SEQUENCE_LEN	5

/*
 * Which AID sequence to use for each candela level.
 * This lookup table is same for both panels.
 */
static const u8 map_candela_to_aid[S6E3FA0_NUM_GAMMA_LEVELS] = {
	43, 42, 41, 40, 39,
	38, 37, 36, 35, 34,
	33, 32, 31, 30, 29,
	28, 27, 26, 25, 24,
	23, 22, 21, 20, 19,
	18, 17, 16, 15, 14,
	13, 12, 11, 10, 9,
	8,  7,  6,  5,  4,
	3,  2,  1,  1,  1,
	1,  1,  1,  1,  1,
	0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,
	0,  0
};

/* AID (Amoled Impulsive Driving) tables */
static const u8 seq_s6e3fa0_aid[NUM_AID_SEQUENCES][AID_SEQUENCE_LEN] = {
	{MCS_AID_CONTROL, 0x00, 0x06, 0x00, 0x06}, /* 0 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x03, 0x06}, /* 1 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x00, 0x08}, /* 2 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x00, 0x77}, /* 3 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x00, 0xE5}, /* 4 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x01, 0x50}, /* 5 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x01, 0xB3}, /* 6 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x02, 0x15}, /* 7 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x02, 0x76}, /* 8 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x02, 0xB2}, /* 9 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x03, 0x01}, /* 10 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x03, 0x65}, /* 11 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x03, 0xAE}, /* 12 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x03, 0xE5}, /* 13 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x04, 0x1C}, /* 14 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x04, 0x54}, /* 15 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x04, 0x8C}, /* 16 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x04, 0xC5}, /* 17 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x04, 0xE9}, /* 18 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0x0C}, /* 19 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0x41}, /* 20 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0x64}, /* 21 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0x88}, /* 22 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0x99}, /* 23 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0xBC}, /* 24 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0xE0}, /* 25 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x05, 0xF1}, /* 26 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x14}, /* 27 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x26}, /* 28 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x38}, /* 29 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x48}, /* 30 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x69}, /* 31 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x7A}, /* 32 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x8B}, /* 33 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0x9B}, /* 34 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0xAC}, /* 35 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0xBC}, /* 36 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0xCD}, /* 37 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0xDE}, /* 38 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x06, 0xEF}, /* 39 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x07, 0x00}, /* 40 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x07, 0x11}, /* 41 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x07, 0x22}, /* 42 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x07, 0x31}, /* 43 */
	{MCS_AID_CONTROL, 0x00, 0x06, 0x00, 0x08}  /* 44 */
};

/*
 * Which ELVSS sequence to use for which candela level.
 * This lookup table is the same for both panels.
 */
static const u8 map_candela_to_elvss[S6E3FA0_NUM_GAMMA_LEVELS] = {
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	0, 0, 1, 2, 2,
	3, 4, 4, 5, 6,
	5, 6, 6, 7, 7,
	8, 8, 8, 8, 9,
	9, 10
};

/* ELVSS (Amoled negative power supply) tables */
#define NUM_ELVSS_SEQUENCES	11
#define ELVSS_SEQUENCE_LEN	3
static const u8 seq_s6e3fa0_elvss[NUM_ELVSS_SEQUENCES][ELVSS_SEQUENCE_LEN] = {
	{MCS_ELVSS_CONTROL, 0x88, 0x18},
	{MCS_ELVSS_CONTROL, 0x88, 0x17},
	{MCS_ELVSS_CONTROL, 0x88, 0x16},
	{MCS_ELVSS_CONTROL, 0x88, 0x15},
	{MCS_ELVSS_CONTROL, 0x88, 0x14},
	{MCS_ELVSS_CONTROL, 0x88, 0x13},
	{MCS_ELVSS_CONTROL, 0x88, 0x12},
	{MCS_ELVSS_CONTROL, 0x88, 0x11},
	{MCS_ELVSS_CONTROL, 0x88, 0x10},
	{MCS_ELVSS_CONTROL, 0x88, 0x0F},
	{MCS_ELVSS_CONTROL, 0x88, 0x0E}
};

/* Gamma (lux) smart dimming tables */
#define GAMMA_SEQ_LEN	34
static const u8 seq_s6e3fa0_lux[S6E3FA0_NUM_GAMMA_LEVELS][GAMMA_SEQ_LEN] = {
{MCS_GAMMA, 0,   175, 0,   191, 0,   165, 139, 140, 138, 140, 144, 139, 144, 155, 144, 144, 155, 145, 145, 151, 143, 149, 155, 149, 171, 171, 165, 187, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   176, 0,   191, 0,   165, 139, 140, 138, 139, 143, 138, 143, 153, 144, 142, 154, 143, 146, 151, 144, 146, 153, 147, 170, 171, 165, 186, 158, 174, 0,   0,   0  },
{MCS_GAMMA, 0,   177, 0,   191, 0,   166, 139, 140, 139, 138, 142, 137, 143, 152, 143, 141, 151, 142, 147, 152, 146, 144, 151, 145, 168, 169, 162, 188, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   177, 0,   191, 0,   167, 138, 139, 138, 139, 142, 138, 142, 152, 142, 141, 151, 142, 145, 150, 143, 143, 152, 144, 167, 168, 160, 188, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   167, 139, 139, 138, 138, 141, 137, 142, 150, 142, 140, 149, 142, 140, 146, 139, 146, 154, 146, 166, 166, 157, 187, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 139, 139, 138, 138, 141, 137, 141, 149, 140, 140, 149, 142, 140, 146, 139, 146, 153, 146, 168, 169, 159, 185, 156, 172, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 139, 139, 138, 138, 141, 137, 141, 149, 141, 137, 147, 140, 140, 145, 139, 144, 153, 144, 165, 166, 157, 187, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 139, 139, 138, 138, 141, 138, 140, 148, 141, 138, 146, 140, 142, 147, 141, 141, 150, 141, 165, 166, 157, 187, 159, 175, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 138, 138, 137, 139, 142, 139, 140, 148, 141, 138, 146, 140, 140, 145, 139, 141, 150, 140, 167, 168, 159, 184, 156, 172, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 138, 138, 137, 140, 142, 139, 139, 146, 140, 137, 145, 140, 140, 145, 138, 140, 148, 138, 167, 168, 159, 184, 156, 172, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 138, 138, 137, 139, 141, 138, 140, 147, 141, 135, 143, 138, 142, 146, 140, 140, 148, 137, 167, 168, 159, 184, 156, 172, 0,   0,   0  },
{MCS_GAMMA, 0,   178, 0,   191, 0,   168, 138, 138, 137, 139, 141, 138, 139, 146, 139, 134, 142, 138, 142, 146, 140, 140, 148, 137, 166, 168, 159, 183, 155, 171, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   168, 137, 137, 136, 140, 142, 139, 139, 146, 140, 135, 142, 138, 139, 144, 138, 142, 150, 139, 163, 165, 156, 182, 155, 170, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   168, 137, 137, 136, 140, 142, 139, 138, 145, 139, 134, 142, 138, 142, 146, 140, 136, 144, 133, 166, 168, 159, 180, 154, 169, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 140, 142, 139, 138, 145, 139, 134, 142, 138, 139, 143, 137, 139, 147, 136, 162, 166, 156, 179, 153, 168, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 138, 139, 137, 138, 144, 139, 134, 142, 139, 138, 142, 136, 139, 147, 136, 160, 165, 155, 178, 153, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 138, 139, 137, 138, 144, 139, 131, 141, 137, 141, 144, 139, 139, 146, 136, 160, 165, 155, 178, 153, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 138, 139, 137, 138, 144, 139, 132, 141, 137, 138, 142, 137, 139, 146, 136, 158, 164, 153, 178, 153, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 138, 139, 137, 139, 144, 139, 130, 139, 135, 141, 144, 139, 136, 143, 133, 156, 163, 152, 178, 153, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 137, 138, 136, 140, 145, 141, 130, 139, 135, 138, 141, 136, 139, 145, 135, 153, 159, 148, 178, 152, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 137, 138, 136, 139, 144, 139, 130, 138, 136, 141, 143, 138, 139, 145, 135, 153, 159, 148, 178, 152, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 137, 138, 136, 137, 142, 138, 135, 141, 139, 136, 138, 133, 139, 144, 135, 156, 161, 151, 178, 152, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 137, 138, 136, 137, 142, 138, 133, 139, 137, 138, 140, 135, 139, 144, 135, 151, 157, 146, 177, 152, 167, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 137, 136, 137, 138, 136, 137, 142, 138, 133, 139, 137, 136, 138, 133, 138, 144, 136, 149, 155, 145, 177, 152, 166, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 138, 137, 137, 136, 137, 135, 138, 142, 139, 132, 139, 138, 138, 140, 135, 135, 140, 132, 149, 155, 145, 177, 152, 166, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 138, 137, 137, 136, 137, 135, 138, 142, 139, 132, 139, 138, 137, 137, 132, 138, 143, 136, 150, 155, 145, 177, 152, 166, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 138, 137, 137, 136, 137, 135, 136, 141, 137, 133, 140, 139, 136, 137, 133, 133, 139, 132, 154, 159, 149, 177, 152, 166, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 138, 137, 137, 136, 137, 135, 136, 141, 137, 133, 140, 139, 136, 137, 133, 133, 139, 132, 149, 155, 144, 177, 152, 166, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 136, 136, 135, 136, 139, 137, 133, 140, 140, 136, 136, 133, 132, 138, 131, 149, 154, 144, 173, 151, 163, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 137, 136, 135, 136, 139, 137, 133, 140, 140, 136, 136, 132, 132, 138, 131, 149, 154, 144, 173, 151, 163, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 137, 136, 135, 136, 139, 137, 133, 139, 140, 136, 136, 133, 132, 138, 131, 149, 154, 144, 173, 151, 163, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 136, 135, 134, 135, 139, 136, 134, 138, 140, 135, 136, 133, 131, 137, 130, 148, 153, 144, 169, 149, 161, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 136, 135, 134, 135, 139, 136, 134, 138, 140, 135, 136, 133, 132, 137, 131, 148, 152, 144, 169, 149, 161, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 136, 135, 134, 135, 139, 136, 134, 138, 140, 135, 136, 133, 132, 137, 131, 143, 148, 139, 165, 148, 158, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 137, 136, 136, 136, 135, 134, 135, 139, 136, 134, 138, 140, 133, 133, 130, 131, 136, 131, 149, 152, 144, 165, 148, 158, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 137, 136, 135, 135, 139, 136, 131, 136, 138, 136, 136, 133, 131, 136, 131, 143, 148, 141, 162, 146, 155, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 137, 136, 135, 136, 140, 137, 131, 136, 138, 136, 135, 133, 131, 136, 131, 143, 148, 141, 162, 146, 155, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 137, 136, 135, 136, 139, 137, 131, 136, 138, 137, 135, 133, 126, 131, 127, 149, 152, 145, 162, 146, 155, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 137, 136, 135, 136, 139, 137, 131, 136, 138, 134, 132, 130, 130, 134, 130, 149, 152, 145, 153, 142, 148, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 136, 135, 134, 136, 139, 137, 131, 134, 138, 134, 132, 130, 128, 133, 130, 136, 143, 134, 160, 146, 154, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   191, 0,   169, 136, 135, 135, 136, 135, 134, 136, 139, 137, 131, 134, 138, 134, 132, 130, 129, 133, 130, 146, 149, 143, 161, 146, 154, 0,   0,   0  },
{MCS_GAMMA, 0,   179, 0,   192, 0,   171, 136, 134, 134, 136, 135, 134, 136, 139, 137, 131, 134, 138, 137, 134, 132, 126, 130, 127, 148, 149, 143, 161, 146, 154, 0,   0,   0  },
{MCS_GAMMA, 0,   208, 0,   216, 0,   202, 134, 133, 133, 133, 133, 133, 134, 135, 134, 131, 134, 133, 128, 130, 128, 133, 137, 132, 142, 147, 139, 149, 137, 144, 0,   0,   0  },
{MCS_GAMMA, 0,   212, 0,   220, 0,   207, 134, 133, 133, 133, 133, 133, 133, 134, 133, 130, 133, 131, 130, 130, 128, 135, 138, 134, 137, 143, 134, 157, 141, 150, 0,   0,   0  },
{MCS_GAMMA, 0,   215, 0,   222, 0,   211, 133, 132, 132, 133, 132, 132, 133, 134, 132, 132, 134, 134, 130, 131, 130, 133, 135, 132, 138, 143, 134, 157, 141, 150, 0,   0,   0  },
{MCS_GAMMA, 0,   219, 0,   226, 0,   215, 133, 132, 132, 133, 133, 133, 132, 133, 131, 130, 133, 133, 131, 132, 131, 130, 133, 130, 138, 143, 135, 157, 141, 150, 0,   0,   0  },
{MCS_GAMMA, 0,   222, 0,   228, 0,   219, 132, 131, 132, 133, 132, 131, 132, 132, 131, 132, 133, 133, 130, 130, 129, 130, 133, 130, 138, 143, 135, 157, 141, 150, 0,   0,   0  },
{MCS_GAMMA, 0,   226, 0,   232, 0,   224, 132, 131, 131, 132, 131, 131, 132, 132, 131, 132, 133, 133, 130, 130, 129, 131, 133, 131, 135, 139, 133, 149, 137, 144, 0,   0,   0  },
{MCS_GAMMA, 0,   230, 0,   235, 0,   228, 132, 132, 132, 131, 131, 129, 132, 132, 131, 131, 132, 132, 131, 131, 130, 128, 131, 128, 135, 139, 133, 149, 137, 144, 0,   0,   0  },
{MCS_GAMMA, 0,   234, 0,   238, 0,   232, 131, 131, 131, 130, 130, 130, 131, 132, 131, 130, 131, 131, 127, 129, 128, 131, 133, 132, 131, 137, 128, 145, 135, 141, 0,   0,   0  },
{MCS_GAMMA, 0,   209, 0,   217, 0,   203, 132, 131, 131, 133, 133, 133, 132, 134, 133, 131, 132, 134, 130, 128, 128, 129, 131, 130, 136, 136, 135, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   213, 0,   221, 0,   208, 132, 130, 131, 133, 132, 132, 132, 134, 133, 130, 132, 133, 131, 129, 129, 131, 132, 131, 131, 133, 132, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   218, 0,   224, 0,   213, 131, 130, 130, 132, 131, 131, 132, 133, 133, 132, 133, 134, 129, 128, 128, 128, 129, 128, 131, 133, 132, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   221, 0,   227, 0,   217, 131, 130, 130, 131, 131, 131, 131, 132, 131, 131, 132, 133, 129, 128, 128, 128, 129, 128, 131, 133, 132, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   225, 0,   230, 0,   221, 131, 130, 130, 130, 130, 130, 131, 132, 131, 128, 129, 130, 129, 128, 128, 128, 129, 128, 131, 133, 132, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   229, 0,   234, 0,   226, 130, 129, 129, 130, 130, 130, 130, 131, 130, 129, 130, 131, 130, 128, 128, 129, 130, 130, 127, 129, 128, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   234, 0,   238, 0,   231, 130, 129, 129, 130, 130, 130, 130, 131, 131, 129, 130, 131, 128, 127, 127, 127, 128, 127, 127, 129, 128, 134, 132, 133, 0,   0,   0  },
{MCS_GAMMA, 0,   238, 0,   241, 0,   236, 129, 129, 129, 130, 130, 130, 130, 130, 130, 129, 129, 130, 129, 128, 128, 128, 129, 129, 132, 131, 131, 127, 127, 127, 0,   0,   0  },
{MCS_GAMMA, 0,   244, 0,   246, 0,   242, 128, 128, 128, 129, 129, 129, 130, 130, 130, 128, 129, 129, 129, 129, 129, 130, 130, 130, 127, 127, 127, 127, 127, 127, 0,   0,   0  },
{MCS_GAMMA, 0,   247, 0,   249, 0,   246, 129, 129, 129, 129, 129, 129, 129, 129, 129, 128, 128, 128, 130, 129, 129, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   0  },
{MCS_GAMMA, 0,   252, 0,   252, 0,   251, 128, 128, 128, 129, 129, 129, 129, 129, 129, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   0  },
{MCS_GAMMA, 1,   0,   1,   0,   1,   0,   128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 0,   0,   0  },
};

/* sequences to lock/unlock Manufacture Time Programming (MTP) commands */
static const u8 seq_s6e3fa0_test_key_en[6] = {
	0xf0, 0x5a, 0x5a,  /* level 1 */
	0xfc, 0x5a, 0x5a   /* level 2 */
};

static const u8 seq_s6e3fa0_test_key_dis[6] = {
	0xf0, 0xa5, 0xa5,  /* level 1 */
	0xfc, 0xa5, 0xa5   /* level 2 */
};

struct s6e3fa0_sequences_data {
	const u8 *test_key_enable_seq;
	const u8 *test_key_disable_seq;

	const u8 (*aid_seq)[NUM_AID_SEQUENCES][AID_SEQUENCE_LEN];
	const u8 (*elvss_seq)[NUM_ELVSS_SEQUENCES][ELVSS_SEQUENCE_LEN];
	const u8 (*gamma_seq)[S6E3FA0_NUM_GAMMA_LEVELS][GAMMA_SEQ_LEN];
};

static const struct s6e3fa0_sequences_data seqdata_s6e3fa0 = {
	.test_key_enable_seq = seq_s6e3fa0_test_key_en,
	.test_key_disable_seq = seq_s6e3fa0_test_key_dis,
	.aid_seq = &seq_s6e3fa0_aid,
	.elvss_seq = &seq_s6e3fa0_elvss,
	.gamma_seq = &seq_s6e3fa0_lux
};

struct s6e3fa0_ctx {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	const struct s6e3fa0_sequences_data *seq_data;
};

/* helper to simplify code */
#define dsi_generic_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_generic_write(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static inline
struct s6e3fa0_ctx *panel_to_s6e3fa0(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3fa0_ctx, panel);
}

enum s6e3fa0_test_key_level {
	TK_LEVEL_1,
	TK_LEVEL_2
};

static int s6e3fa0_test_key_toggle(struct s6e3fa0_ctx *ctx, bool enable,
				   enum s6e3fa0_test_key_level level)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;
	const u8 *seq;

	if (!ctx->seq_data)
		return -ENODEV;

	if (enable)
		seq = ctx->seq_data->test_key_enable_seq;
	else
		seq = ctx->seq_data->test_key_disable_seq;

	/* First 3 bytes - level 1 lock/unlock */
	ret = mipi_dsi_generic_write(dsi, seq, 3);
	if (ret < 0)
		return ret;

	/*
	 * During init lock/unlock 2nd level MTP commands (send bytes 4,5,6)
	 */
	if (level == TK_LEVEL_2)
		ret = mipi_dsi_generic_write(dsi, seq + 3, 3);
	return ret;
}

static int s6e3fa0_write_aid_control(struct s6e3fa0_ctx *ctx, int candela)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int idx;
	const u8 *seq;

	if (!ctx->seq_data)
		return -ENODEV;

	idx = map_candela_to_aid[candela];
	seq = (*ctx->seq_data->aid_seq)[idx];
	return mipi_dsi_generic_write(dsi, seq, AID_SEQUENCE_LEN);
}

static int s6e3fa0_write_elvss(struct s6e3fa0_ctx *ctx, int candela)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int idx;
	const u8 *seq;

	if (!ctx->seq_data)
		return -ENODEV;

	idx = map_candela_to_elvss[candela];
	seq = (*ctx->seq_data->elvss_seq)[idx];
	return mipi_dsi_generic_write(dsi, seq, ELVSS_SEQUENCE_LEN);
}

static int s6e3fa0_write_gamma(struct s6e3fa0_ctx *ctx, int candela)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const u8 *seq;

	if (!ctx->seq_data)
		return -ENODEV;

	/* Gamma sequence is directly indexed by candela value */
	seq = (*ctx->seq_data->gamma_seq)[candela];
	return mipi_dsi_generic_write(dsi, seq, GAMMA_SEQ_LEN);
}

static int s6e3fa0_write_gamma_apply(struct s6e3fa0_ctx *ctx)
{
	/* From vendor driver: Gamma, LTPS(AID) update */
	dsi_generic_write_seq(ctx->dsi, 0xf7, 0x03);

	return 0;
}

static int s6e3fa0_set_brightness(struct backlight_device *bldev)
{
	struct s6e3fa0_ctx *ctx = bl_get_data(bldev);
	const int candela = backlight_get_brightness(bldev);
	int r;

	r = s6e3fa0_test_key_toggle(ctx, true, TK_LEVEL_1);
	if (r < 0)
		return r;
	r = s6e3fa0_write_aid_control(ctx, candela);
	if (r < 0)
		return r;
	r = s6e3fa0_write_elvss(ctx, candela);
	if (r < 0)
		return r;
	r = s6e3fa0_write_gamma(ctx, candela);
	if (r < 0)
		return r;
	r = s6e3fa0_write_gamma_apply(ctx);
	if (r < 0)
		return r;
	r = s6e3fa0_test_key_toggle(ctx, false, TK_LEVEL_1);
	if (r < 0)
		return r;

	return 0;
}

static int s6e3fa0_verify_panel_id(struct s6e3fa0_ctx *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	u8 id1, id2, id3;
	unsigned int lcd_id;
	int ret;

	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID1, &id1, 1);
	if (ret < 0)
		goto read_id_fail;
	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID2, &id2, 1);
	if (ret < 0)
		goto read_id_fail;
	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID3, &id3, 1);
	if (ret < 0)
		goto read_id_fail;

	lcd_id = id1 << 16 | id2 << 8 | id3;

	switch (lcd_id) {
	case LCD_ID_S6E3FA0_1:
	case LCD_ID_S6E3FA0_2:
		dev_info(&dsi->dev, "detected S6E3FA0 panel (ID: 0x%x)\n",
			 lcd_id);
		ctx->seq_data = &seqdata_s6e3fa0;
		break;
	default:
		dev_warn(&dsi->dev, "unsupported panel ID: 0x%x\n", lcd_id);
		return -ENODEV;
	}
	return 0;

read_id_fail:
	dev_err(&dsi->dev, "could not read panel MTP ID\n");
	return -ENODEV;
}

static int s6e3fa0_init(struct s6e3fa0_ctx *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const int candela = backlight_get_brightness(ctx->panel.backlight);
	u8 cmd;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	s6e3fa0_test_key_toggle(ctx, true, TK_LEVEL_2);

	/* Configure MIPI Interface (Single DSI) */
	cmd = 0xf2;
	ret = mipi_dsi_dcs_write_buffer(dsi, &cmd, 1);
	if (ret < 0)
		return ret;

	usleep_range(5000, 6000);

	s6e3fa0_write_gamma(ctx, candela);
	s6e3fa0_write_aid_control(ctx, candela);

	/* ELVSS Setting for 300Cd  */
	dsi_generic_write_seq(dsi, MCS_ELVSS_CONTROL, 0x88, 0x0a);

	s6e3fa0_write_gamma_apply(ctx);

	/* Vsync Enable(Tear On) : 0x00, Hsync Enable : 0x01  */
	mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	/* RE Condition Setting for EVT1 */
	dsi_generic_write_seq(dsi, 0xc0, 0x00, 0x02, 0x03, 0x32, 0x03, 0x44, 0x44, 0xc0, 0x00, 0x1c, 0x20, 0xe8);
	dsi_generic_write_seq(dsi, 0xe3, 0xff, 0xff, 0xff, 0xff);
	dsi_generic_write_seq(dsi, 0xfe, 0x00, 0x03);
	dsi_generic_write_seq(dsi, 0xb0, 0x2b);
	dsi_generic_write_seq(dsi, 0xfe, 0xe4);

	/* ERR_FG, Hsync On */
	dsi_generic_write_seq(dsi, 0xbd, 0xed, 0x0c, 0x04);
	dsi_generic_write_seq(dsi, 0xff, 0x01);

	s6e3fa0_test_key_toggle(ctx, false, TK_LEVEL_2);

	return 0;
}

static int s6e3fa0_power_on(struct s6e3fa0_ctx *ctx)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to enable regulators: %d\n",
			ret);
		return ret;
	}
	msleep(30);

	/* send reset pulse */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(7000, 8000);

	/* Panel init sequence does not work without this delay! */
	msleep(60);

	return 0;
}

static int s6e3fa0_power_off(struct s6e3fa0_ctx *ctx)
{
	int ret;

	gpiod_set_value(ctx->reset_gpio, 1);

	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	return 0;
}

static int s6e3fa0_enable(struct drm_panel *panel)
{
	struct s6e3fa0_ctx *ctx = panel_to_s6e3fa0(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	};
	msleep(60);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(50);

	return 0;
}

static int s6e3fa0_disable(struct drm_panel *panel)
{
	struct s6e3fa0_ctx *ctx = panel_to_s6e3fa0(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static int s6e3fa0_prepare(struct drm_panel *panel)
{
	struct s6e3fa0_ctx *ctx = panel_to_s6e3fa0(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = s6e3fa0_power_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to power on the panel!\n");
		return ret;
	}

	ret = s6e3fa0_verify_panel_id(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to verify panel ID!\n");
		return ret;
	}

	ret = s6e3fa0_init(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		s6e3fa0_power_off(ctx);
		return ret;
	}

	return 0;
}

static int s6e3fa0_unprepare(struct drm_panel *panel)
{
	struct s6e3fa0_ctx *ctx = panel_to_s6e3fa0(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = s6e3fa0_power_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to power off panel: %d\n", ret);

	return 0;
}

/* Resolution, size and timing values */
static const struct drm_display_mode s6e3fa0_display_mode = {
	.clock = (1080 + 162 + 10 + 36) * (1920 + 13 + 2 + 3) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 162,
	.hsync_end = 1080 + 162 + 10,
	.htotal = 1080 + 162 + 10 + 36,
	.vdisplay = 1920,
	.vsync_start = 1920 + 13,
	.vsync_end = 1920 + 13 + 2,
	.vtotal = 1920 + 13 + 2 + 3,
	.width_mm = 71,
	.height_mm = 126,
};

static int s6e3fa0_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &s6e3fa0_display_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.panel_orientation =
			DRM_MODE_PANEL_ORIENTATION_NORMAL;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs s6e3fa0_panel_funcs = {
	.prepare = s6e3fa0_prepare,
	.enable = s6e3fa0_enable,
	.disable = s6e3fa0_disable,
	.unprepare = s6e3fa0_unprepare,
	.get_modes = s6e3fa0_get_modes,
};

static const struct backlight_ops s6e3fa0_bl_ops = {
	.update_status	= s6e3fa0_set_brightness,
};

static int s6e3fa0_backlight_register(struct s6e3fa0_ctx *ctx)
{
	const struct backlight_properties props = {
		.type		= BACKLIGHT_RAW,
		.scale		= BACKLIGHT_SCALE_LINEAR,
		.brightness	= S6E3FA0_MAX_BRIGHTNESS,
		.max_brightness = S6E3FA0_MAX_BRIGHTNESS,
	};
	struct device *dev = &ctx->dsi->dev;
	int ret = 0;

	ctx->panel.backlight = devm_backlight_device_register(dev, "panel", dev,
							      ctx,
							      &s6e3fa0_bl_ops,
							      &props);
	if (IS_ERR(ctx->panel.backlight)) {
		ret = PTR_ERR(ctx->panel.backlight);
		dev_err(dev, "error registering backlight device (%d)\n", ret);
	}

	return ret;
}

static int s6e3fa0_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3fa0_ctx *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "iovdd";
	ctx->supplies[1].supply = "vddr";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->hs_rate = 898000000;
	/* lp_rate is unknown */

	drm_panel_init(&ctx->panel, dev, &s6e3fa0_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = s6e3fa0_backlight_register(ctx);
	if (ret < 0)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void s6e3fa0_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fa0_ctx *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

/*
 * This driver supports 1 type of panel
 */
static const struct of_device_id samsung_s6e3fa0_of_match[] = {
	{ .compatible = "samsung,s6e3fa0" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, samsung_s6e3fa0_of_match);

static struct mipi_dsi_driver samsung_s6e3fa0_driver = {
	.probe = s6e3fa0_probe,
	.remove = s6e3fa0_remove,
	.driver = {
		.name = "panel-samsung-s6e3fa0",
		.of_match_table = samsung_s6e3fa0_of_match,
	},
};
module_mipi_dsi_driver(samsung_s6e3fa0_driver);

MODULE_AUTHOR("Adam Honse <calcprogrammer1@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung S6E3FA0 DSI panel");
MODULE_LICENSE("GPL v2");
