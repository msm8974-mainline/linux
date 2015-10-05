/*
 * Copyright (C) 2015 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OCMEM_H__
#define __OCMEM_H__

enum ocmem_client {
	/* GMEM clients */
	OCMEM_GRAPHICS = 0x0,
	/*
	 * TODO add more once ocmem_allocate() is clever enough to
	 * deal with multiple clients.
	 */
	OCMEM_CLIENT_MAX,
};

struct ocmem_buf {
	unsigned long offset;
	unsigned long addr;
	unsigned long len;
};

struct ocmem_buf *ocmem_allocate(enum ocmem_client client, unsigned long size);
void ocmem_free(enum ocmem_client client, struct ocmem_buf *buf);

#endif /* __OCMEM_H__ */
