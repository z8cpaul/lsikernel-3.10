/*
 * asm/lsi/acp_ncr.h
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#ifndef __DRIVERS_LSI_ACP_NCR_H
#define __DRIVERS_LSI_ACP_NCR_H

#ifndef NCP_REGION_ID
#define NCP_REGION_ID(node, target) \
(unsigned long)((((node) & 0xffff) << 16) | ((target) & 0xffff))
#endif

#ifndef NCP_NODE_ID
#define NCP_NODE_ID(region) (((region) >> 16) & 0xffff)
#endif

#ifndef NCP_TARGET_ID
#define NCP_TARGET_ID(region) ((region) & 0xffff)
#endif

int ncr_read(unsigned long, unsigned long, int, void *);
int ncr_write(unsigned long, unsigned long, int, void *);

int is_asic(void);

extern int acp_mdio_read(unsigned long, unsigned long, unsigned short *, int);
extern int acp_mdio_write(unsigned long, unsigned long, unsigned short, int);

#endif /*  __DRIVERS_LSI_ACP_NCR_H */
