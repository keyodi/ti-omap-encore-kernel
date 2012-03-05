/*
 * Copyright (C) 2011 Barnes & Noble
 *
 * Written by David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_OMAP_ENCORE_H
#define __ASM_ARCH_OMAP_ENCORE_H

#define ENCORE_RAM_CONSOLE_START   0x8E000000
#define ENCORE_RAM_CONSOLE_SIZE    0x20000

#define BOARD_FEATURE_3G	0x08
#define BOARD_FEATURE_1GHz	0x10
#define BOARD_FEATURE_EINK	0x20

#define BOARD_ENCORE_REV_EVT1A      0x1
#define BOARD_ENCORE_REV_EVT1B      0x2
#define BOARD_ENCORE_REV_EVT2       0x3

#define EVT0  0
#define EVT1A 1
#define EVT1B 2
#define EVT2  3
#define DVT   4
#define PVT   5

#define MACH_TYPE_OMAP3621_EVT1A       3003

extern int has_3G_support(void);
extern int has_1GHz_support(void);

static inline int encore_board_type(void)
{
	return system_rev;  // This is set by U-Boot
}

static inline int is_encore_board_evt2(void)
{
    return (system_rev >= BOARD_ENCORE_REV_EVT2);
}

static inline int is_encore_board_evt1b(void)
{
    return (system_rev == BOARD_ENCORE_REV_EVT1B);
}

#endif /* __ASM_ARCH_OMAP_ENCORE_H */
