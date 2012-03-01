/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/bootmem.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board-encore.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/timer-gp.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"

void __init evt_peripherals_init(void);

static struct omap_board_config_kernel evt_config[] __initdata = {
};

static void __init omap_evt_init_irq(void)
{
	omap_board_config = evt_config;
	omap_board_config_size = ARRAY_SIZE(evt_config);
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			h8mbx00u0mer0em_sdrc_params);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	omap_init_irq();	
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static inline void omap2_ramconsole_reserve_sdram(void)
{
    reserve_bootmem(ENCORE_RAM_CONSOLE_START, ENCORE_RAM_CONSOLE_SIZE, 0);
}
#else
static inline void omap2_ramconsole_reserve_sdram(void) {}
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

static void __init omap_evt_init(void)
{
	evt_peripherals_init();
}

static void __init omap_evt_map_io(void)
{
        omap2_ramconsole_reserve_sdram();
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

#ifdef CONFIG_OMAP_MUX
  #error "EVT1A port relies on the bootloader for MUX configuration."
#endif

MACHINE_START(OMAP3621_EVT1A, "encore")
	.phys_io        = L4_34XX_PHYS,
	.io_pg_offst    = ((L4_34XX_VIRT) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_evt_map_io,
	.init_irq	= omap_evt_init_irq,
	.init_machine	= omap_evt_init,
	.timer		= &omap_timer,
MACHINE_END

