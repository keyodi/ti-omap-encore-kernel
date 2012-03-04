/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/usb/android_composite.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#include <plat/mux.h>
#include <plat/dmtimer.h>
#include <plat/omap-serial.h>
#include <plat/mmc.h>
#include <plat/opp_twl_tps.h>

#include <mach/board-encore.h>

#include "mux.h"
#include "hsmmc.h"
#include "twl4030.h"
#include "smartreflex-class1p5.h"

#include <linux/cyttsp.h>
#include <linux/ft5x06.h>
#include <linux/kxtf9.h>
#include <linux/max17042.h>
#include <linux/max8903.h>

#ifdef CONFIG_BT_WILINK
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#endif

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34
#define KXTF9_GPIO_FOR_IRQ              113

#define CYTTSP_I2C_SLAVEADDRESS  34
#define OMAP_CYTTSP_GPIO         99
#define OMAP_CYTTSP_RESET_GPIO   46
#define LCD_EN_GPIO              36

#define FT5x06_I2C_SLAVEADDRESS  (0x70 >> 1)
#define OMAP_FT5x06_GPIO         99
#define OMAP_FT5x06_RESET_GPIO   46

#define MAX17042_GPIO_FOR_IRQ   100

extern void evt_lcd_panel_init(void);

int  ft5x06_dev_init(int resource)
{
    if (resource)
    {
        if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0)
        {
            printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
            return -1;
        }

        if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0)
        {
            printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
            return -1;
        }

        gpio_direction_input(OMAP_FT5x06_GPIO);
    }
    else
    {
        gpio_free(OMAP_FT5x06_GPIO);
        gpio_free(OMAP_FT5x06_RESET_GPIO);
    }

    return 0;
}

static struct ft5x06_platform_data ft5x06_platform_data = {
    .maxx = 1024,
    .maxy = 600,
    .flags = FLIP_DATA_FLAG | REVERSE_Y_FLAG | REVERSE_X_FLAG,
    .reset_gpio = OMAP_FT5x06_RESET_GPIO,
    .use_st = FT_USE_ST,
    .use_mt = FT_USE_MT,
    .use_trk_id = 1, //FT_USE_TRACKING_ID,
    .use_sleep = FT_USE_SLEEP,
    .use_gestures = 0,
};


int  cyttsp_dev_init(int resource)
{
        if (resource)
        {
                if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
                        printk(KERN_ERR "can't get tma340 xreset GPIO\n");
                        return -1;
                }

                if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
                        printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
                        return -1;
                }

                gpio_direction_input(OMAP_CYTTSP_GPIO);
        }
        else
        {
                gpio_free(OMAP_CYTTSP_GPIO);
                gpio_free(OMAP_CYTTSP_RESET_GPIO);
        }
    return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
        .maxx = 480,
        .maxy = 800,
        .flags = FLIP_DATA_FLAG | REVERSE_Y_FLAG,
        .gen = CY_GEN3,
        .use_st = CY_USE_ST,
        .use_mt = CY_USE_MT,
        .use_hndshk = CY_SEND_HNDSHK,
        .use_trk_id = 1, //CY_USE_TRACKING_ID,
        .use_sleep = CY_USE_SLEEP,
        .use_gestures = 0,
        /* activate up to 4 groups
         * and set active distance
         */
        .gest_set = 0,
        /* change act_intrvl to customize the Active power state 
         * scanning/processing refresh interval for Operating mode
         */
        .act_intrvl = CY_ACT_INTRVL_DFLT,
        /* change tch_tmout to customize the touch timeout for the
         * Active power state for Operating mode
         */
        .tch_tmout = CY_TCH_TMOUT_DFLT,
        /* change lp_intrvl to customize the Low Power power state 
         * scanning/processing refresh interval for Operating mode
         */
        .lp_intrvl = CY_LP_INTRVL_DFLT,
};


static void kxtf9_dev_init(void)
{
	printk("board-3621_evt1a.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-3621_evt1a.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n", KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
	gpio_set_debounce(KXTF9_GPIO_FOR_IRQ, 0);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	// Map the axes from the sensor to the device.

	//. SETTINGS FOR THE EVT1A:
	.axis_map_x     = 0,
	.axis_map_y     = 1,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 1,
	.negate_z       = 0,

	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = KXTF9_GPIO_FOR_IRQ,
};

static struct regulator_consumer_supply encore_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data encore_lcd_tp_vinit = {
    .constraints = {
        .min_uV = 3300000,
        .max_uV = 3300000,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = 2,
    .consumer_supplies = encore_lcd_tp_supply,
};

static struct fixed_voltage_config encore_lcd_touch_reg_data = {
    .supply_name = "vdd_lcdtp",
    .microvolts = 3300000,
    .gpio = LCD_EN_GPIO,
    .enable_high = 1,
    .enabled_at_boot = 0,
    .init_data = &encore_lcd_tp_vinit,
};

static struct platform_device encore_lcd_touch_regulator_device = {
    .name   = "reg-fixed-voltage",
    .id     = -1,
    .dev    = {
        .platform_data = &encore_lcd_touch_reg_data,
    },
};

static int board_keymap[] = {
	KEY(0, 0, KEY_HOME),
	KEY(1, 0, KEY_VOLUMEUP),
	KEY(2, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data evt_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
//	.rep		= 1,
};

static struct gpio_keys_button evt_gpio_buttons[] = {
	{
		.code			= KEY_POWER,
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "HOME",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data evt_gpio_key_info = {
	.buttons	= evt_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(evt_gpio_buttons),
};

static struct platform_device evt_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
	.platform_data	= &evt_gpio_key_info,
	},
};

static struct twl4030_power_data evt_t2scripts_data;

static struct regulator_consumer_supply evt_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply evt_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply evt_vmmc2_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply evt_vdda_dac_supply = {
	.supply		= "vdda_dac",
};

static struct regulator_consumer_supply evt_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
};


/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data evt_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data evt_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data evt_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vsim_supply,
};


static struct regulator_init_data evt_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vdda_dac_supply,
};

static struct regulator_init_data evt_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vdds_dsi_supply,
};

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_CHARGER_MAX8903
static struct resource max8903_gpio_resources_evt1a[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};

static inline void max8903_charger_init(void)
{
	max8903_charger_device.resource = max8903_gpio_resources_evt1a;
	max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_evt1a);

	platform_device_register(&max8903_charger_device);
}
#endif

/*--------------------------------------------------------------------------*/

static struct platform_device *evt_board_devices[] __initdata = {	
	&encore_lcd_touch_regulator_device,
	&evt_keys_gpio,
};

/* The order is reverted in this table so that internal eMMC is presented
 * as first mmc card for compatibility with existing android installations */
static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.name		= "internal",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{}      /* Terminator */
};

static int evt_hsmmc_card_detect(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* Encore board EVT2 and later has pin high when card is present) */
	return gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

static int evt_twl4030_hsmmc_late_init(struct device *dev)
{
        int ret = 0;
        struct platform_device *pdev = container_of(dev,
                                struct platform_device, dev);
        struct omap_mmc_platform_data *pdata = dev->platform_data;

	if(is_encore_board_evt2()) {
		/* Setting MMC1 (external) Card detect */
		if (pdev->id == 0) {
			pdata->slots[0].card_detect = evt_hsmmc_card_detect;
		}
	}
        return ret;
}

static __init void evt_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = evt_twl4030_hsmmc_late_init;
}

static int evt_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
struct omap2_hsmmc_info *c;
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
printk("******IN boxer_twl_gpio_setup********\n");
	mmc[1].gpio_cd = gpio + 0;
	mmc[0].gpio_cd = gpio + 1;
	omap2_hsmmc_init(mmc);
	for (c = mmc; c->mmc; c++)
                evt_hsmmc_set_late_init(c->dev);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	evt_vmmc1_supply.dev = mmc[1].dev;
	evt_vsim_supply.dev = mmc[1].dev;
	evt_vmmc2_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_usb_data evt_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data evt_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= evt_twl_gpio_setup,
};

static struct twl4030_madc_platform_data evt_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data evt_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &evt_madc_data,
	.usb		= &evt_usb_data,
	.gpio		= &evt_gpio_data,
	.keypad		= &evt_kp_twl4030_data,
//	.power		= &evt_t2scripts_data,
	.vmmc1		= &evt_vmmc1,
        .vmmc2		= &evt_vmmc2,
	.vsim		= &evt_vsim,
        .vdac		= &evt_vdac,
	.vpll2		= &evt_vdsi,
};

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

        .gpio = MAX17042_GPIO_FOR_IRQ,
};
#endif

static struct i2c_board_info __initdata evt_i2c_boardinfo[] = {
#ifdef CONFIG_BATTERY_MAX17042
        {
                I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
                .platform_data = &max17042_platform_data_here,
                .irq = OMAP_GPIO_IRQ(MAX17042_GPIO_FOR_IRQ),
        },
#endif  /*CONFIG_BATTERY_MAX17042*/
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags		= I2C_CLIENT_WAKE,
		.irq		= INT_34XX_SYS_NIRQ,
		.platform_data	= &evt_twldata,
	},
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},

};

#ifdef CONFIG_TI_ST
/* wl128x BT, FM, GPS connectivity chip */
int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{ 
    return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
    return 0;
}

struct ti_st_plat_data wilink_pdata = {
        .nshutdown_gpio = 60,
        .dev_name = "/dev/ttyO1",
        .flow_cntrl = 1,
        .baud_rate = 3000000,
        .suspend = plat_kim_suspend,
        .resume = plat_kim_resume,
};

static struct platform_device kim_wl127x_device = {
        .name           = "kim",
        .id             = -1,
        .dev.platform_data = &wilink_pdata,
};

#endif

#ifdef CONFIG_BT_WILINK
static struct platform_device btwilink_device = {
       .name = "btwilink",
       .id = -1,
};
#endif

#define AUDIO_CODEC_IRQ_GPIO             59
#define AIC3100_NAME			"tlv320dac3100"
#define AIC3100_I2CSLAVEADDRESS		0x18

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC3100)
#define AUDIO_CODEC_POWER_ENABLE_GPIO    103
#define AUDIO_CODEC_RESET_GPIO           37

static void audio_dac_3100_dev_init(void)
{
        printk("board-3621_evt1a.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_RESET_GPIO, "AUDIO_CODEC_RESET_GPIO") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_RESET_GPIO \n");
                return;
        }

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output Low!\n");
        gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 0);

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_POWER_ENABLE_GPIO, "AUDIO DAC3100 POWER ENABLE") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_POWER_ENABLE_GPIO \n");
                return;
        }

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_POWER_ENABLE_GPIO to output and value high!\n");
        gpio_direction_output(AUDIO_CODEC_POWER_ENABLE_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_POWER_ENABLE_GPIO, 1);

	/* 1 msec delay needed after PLL power-up */
        mdelay (1);

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output and value high!\n");
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);
}
#endif

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
        printk("board-3621_evt1a.c: max17042_dev_init ...\n");

        if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
                printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
                return;
        }

        printk("board-3621_evt1a.c: max17042_dev_init > Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
        gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
        gpio_set_debounce(MAX17042_GPIO_FOR_IRQ, 0);
        printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

static struct i2c_board_info __initdata evt_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
		.platform_data = &cyttsp_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
        {
                I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
                .platform_data = &ft5x06_platform_data,
                .irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
        },
	{
		I2C_BOARD_INFO(AIC3100_NAME,  AIC3100_I2CSLAVEADDRESS),
                .irq = OMAP_GPIO_IRQ(AUDIO_CODEC_IRQ_GPIO),
	},
};

static int __init omap_i2c_init(void)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}
	omap_register_i2c_bus(1, 100, NULL, evt_i2c_boardinfo,
			ARRAY_SIZE(evt_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, evt_i2c_bus2_info,
			ARRAY_SIZE(evt_i2c_bus2_info));
	return 0;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.power			= 100,
};

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

static struct twl4030_ins sleep_on_seq[] __initdata = {
#if 0
	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
#endif	       	
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},

	{MSG_SINGULAR(DEV_GRP_P1, 0xe, RES_STATE_ACTIVE), 0xe},
	//{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_OFF), 0xe},

        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_SLEEP), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_SLEEP), 0x37},	
};

static struct twl4030_ins wakeup_p12_seq[] __initdata = {

	//{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_ACTIVE), 0xe},

	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},

        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},	
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {

	{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_ACTIVE), 0xe},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},

        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},	
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};


static struct twl4030_resconfig twl4030_rconfig[] = {
	{.resource = RES_HFCLKOUT,.devgroup = DEV_GRP_P3,.type = -1,
	 .type2 = -1},
	{.resource = RES_VDD1,.devgroup = DEV_GRP_P1,.type = -1,
	 .type2 = -1},
	{.resource = RES_VDD2,.devgroup = DEV_GRP_P1,.type = -1,
	 .type2 = -1},
	{.resource = RES_CLKEN,.devgroup = DEV_GRP_P3,.type = -1,
	 .type2 = 1},
	{0, 0},
};

static struct twl4030_script sleep_on_script = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_script wakeup_p12_script = {
	.script = wakeup_p12_seq,
	.size   = ARRAY_SIZE(wakeup_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_script wakeup_p3_script = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_script wrst_script  = {
	.script = wrst_seq,
	.size = ARRAY_SIZE(wrst_seq),
	.flags = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] = {
	&wakeup_p12_script,
	&sleep_on_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_power_data boxer_t2scripts_data = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};


static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
	}
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
        { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

#ifdef CONFIG_PM
static struct omap_volt_vc_data vc_config = {
	/* MPU */
	.vdd0_on	= 1200000, /* 1.2v */
	.vdd0_onlp	= 1000000, /* 1.0v */
	.vdd0_ret	=  975000, /* 0.975v */
	.vdd0_off	=  600000, /* 0.6v */
	/* CORE */
	.vdd1_on	= 1150000, /* 1.15v */
	.vdd1_onlp	= 1000000, /* 1.0v */
	.vdd1_ret	=  975000, /* 0.975v */
	.vdd1_off	=  600000, /* 0.6v */


	.clksetup	= 0x14A,
	.voltoffset	= 0x118,
	.voltsetup2	= 0x32,
	.voltsetup_time1 = 0x00B3,
	.voltsetup_time2 = 0x00A0,
};
#ifdef CONFIG_TWL4030_CORE
static struct omap_volt_pmic_info omap_pmic_mpu = { /* and iva */
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x0, /* (vdd0) VDD1 -> VDD1_CORE -> VDD_MPU */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x14,
	.vp_vlimitto_vddmax = 0x44,
};

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x1, /* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x42,
};
#endif /* CONFIG_TWL4030_CORE */
#endif /* CONFIG_PM */

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
    .start  = ENCORE_RAM_CONSOLE_START,
    .end    = (ENCORE_RAM_CONSOLE_START + ENCORE_RAM_CONSOLE_SIZE - 1),
    .flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
    .name           = "ram_console",
    .id             = 0,
    .num_resources  = 1,
    .resource       = &ram_console_resource,
};

static inline void ramconsole_init(void)
{
    platform_device_register(&ram_console_device);
}
#else
static inline void ramconsole_init(void) {}
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

void __init evt_peripherals_init(void)
{
	/* Use custom Encore scripts */
	evt_t2scripts_data = boxer_t2scripts_data;

        ramconsole_init();

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC3100)
	audio_dac_3100_dev_init();
#endif

	/* NOTE: Please deselect CONFIG_MACH_OMAP_USE_UART3 in order
	 * to init only UART1 and UART2, all in the name of saving some
	 * power.
	 */
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	omap_i2c_init();
	platform_add_devices(evt_board_devices,
			ARRAY_SIZE(evt_board_devices));
	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);
	enable_board_wakeup_source();

	sr_class1p5_init();

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
        omap_voltage_register_pmic(&omap_pmic_core, "core");
        omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif
	
	evt_lcd_panel_init();

	max8903_charger_init();
	kxtf9_dev_init();
        max17042_dev_init();
        
#ifdef CONFIG_TI_ST
    printk("encore: registering wl127x device.\n");
        platform_device_register(&kim_wl127x_device);
#endif

#ifdef CONFIG_BT_WILINK
    printk("encore: registering btwilink device.\n");
        platform_device_register(&btwilink_device);
#endif
}
