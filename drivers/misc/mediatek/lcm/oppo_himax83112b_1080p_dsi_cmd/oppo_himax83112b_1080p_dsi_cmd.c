/******************************************************************
** Copyright (C), 2004-2018, OPPO Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oppo_himax83112b_1080p_dsi_cmd.c
** Description: Source file for lcd drvier.
** lcd driver including parameter and power control.
** Version: 1.0
** Date : 2017/12/06
** Author: LIPING-M@PSW.MultiMedia.Display.LCD.Machine
**
** ------------------------------- Revision History:---------------
** liping 2017/12/06 1.0 build this module
*******************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_boot_common.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#include <platform/boot_mode.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include <linux/slab.h>
#include <linux/string.h>
#include <soc/oppo/device_info.h>

#include "ddp_hal.h"


#define DEBUG_INTERFACE

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static LCM_UTIL_FUNCS *lcm_util = NULL;

#define SET_RESET_PIN(v) (lcm_util->set_reset_pin((v)))
#define MDELAY(n) (lcm_util->mdelay(n))
#define UDELAY(n) (lcm_util->udelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util->dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util->dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util->dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util->dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util->dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util->dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)	lcm_util->dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)

#define LCM_DSI_CMD_MODE 1
#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (2280)
#define PHYSICAL_WIDTH (68)
#define PHYSICAL_HEIGHT (136)

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


extern int lcm_power_write_byte(unsigned char addr,  unsigned char value);
extern int lm3697_write_byte(unsigned char addr,  unsigned char value);
extern int lm3697_setbacklight(unsigned int level);

#define REGFLAG_DELAY      0xFC
#define REGFLAG_UDELAY     0xFB

#define REGFLAG_END_OF_TABLE   0xFD
#define REGFLAG_RESET_LOW  0xFE
#define REGFLAG_RESET_HIGH  0xFF

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_video_setting[] = {
//from set fw code
#if 0
    {0xB9,3, {0x83,0x11,0x2B}},
    {0xBD,1, {0x01}},
    {0xC2,2, {0x08,0xE8}},
    {0xBD,1, {0x00}},
    {0xBD,1, {0x03}},
    {0xB2,4, {0x04,0x38,0x08,0xE8}},
    {0xBD,1, {0x00}},
    {0xB1,10,{0xF8,0x27,0x27,0x00,0x00,0x0B,0x0E,0x0B,0x0E,0x33}},
    {0xD2,2, {0x2D,0x2D}},
    {0xB2,11,{0x00,0x02,0x18,0x80,0xE8,0x00,0x08,0x1C,0xFB,0x11,0x01}},
    {0xE9,1, {0xD1}},
    {0xB2,2, {0x00,0x08}},
    {0xE9,1, {0x00}},
    {0xDD,5, {0x00,0x00,0x08,0x1C,0xFB}},
    {0xB4,24,{0x30,0x30,0x00,0x00,0xD0,0xD4,0x4C,0xCF,0x00,0xDD,0x00,0xD8,0x00,0x00,0x00,0x07,0x00,0x3D,0x09,0x03,0x07,0x00,0x06,0x86}},
    {0xBC,1, {0x05}},
    {0xBD,1, {0x00}},
    {0xC2,1, {0xC8}},
    {0xE9,1, {0xC5}},
    {0xC7,1, {0x33}},
    {0xE9,1, {0x00}},
    {0xCC,1, {0x08}},
    {0xD3,37,{0x81,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x03,0x37,0x20,0x04,0x0B,0x0B,0x0B,0x0B,0x32,0x10,0x09,0x00,0x0B,0x32,0x10,0x08,0x00,0x08,0x32,0x10,0x08,0x00,0x08,0x00,0x00,0x0A,0x08,0xF3}},
    {0xE9,1, {0xD4}},
    {0xC6,1, {0xEE}},
    {0xE9,1, {0x00}},
    {0xE9,1, {0xEF}},
    {0xD3,1, {0x0C}},
    {0xE9,1, {0x00}},
    {0xBD,1, {0x01}},
    {0xE9,1, {0xC8}},
    {0xD3,1, {0xA1}},
    {0xE9,1, {0x00}},
    {0xBD,1, {0x00}},
    {0xD5,56,{0x18,0x18,0x19,0x18,0x18,0x18,0x18,0x20,0x18,0x18,0x10,0x10,0x18,0x18,0x18,0x18,0x03,0x03,0x02,0x02,0x01,0x01,0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x31,0x31,0x2F,0x2F,0x30,0x30,0x18,0x18,0x35,0x35,0x36,0x36,0x37,0x37,0xE0,0xE0,0x0E,0x0E,0xE0,0xE0,0x0E,0x0E}},
    {0xD8,24,{0xBA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0x55,0xAA,0xBF,0xAA,0xAA,0xAA,0x55,0xAA,0xBF,0xAA}},
    {0xBD,1, {0x01}},
    {0xD8,12,{0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA}},
    {0xBD,1, {0x02}},
    {0xD8,12,{0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA}},
    {0xBD,1, {0x03}},
    {0xD8,24,{0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA,0xAA,0xAA,0xFF,0xAA,0xBF,0xAA}},
    {0xBD,1, {0x00}},
    {0xE9,1, {0xE4}},
    {0xE7,2, {0x17,0x69}},
    {0xE9,1, {0x00}},
    {0xE7,25,{0x09,0x09,0x00,0x06,0xE1,0x00,0x20,0x00,0x07,0x00,0x00,0xE1,0x4A,0x00,0xDF,0x0A,0x0A,0x00,0x00,0x00,0x01,0x01,0x00,0x12,0x04}},
    {0xBD,1, {0x01}},
    {0xE7,25,{0x09,0x09,0x00,0x06,0xE1,0x00,0x20,0x00,0x07,0x00,0x00,0xE1,0x4A,0x00,0xDF,0x0A,0x0A,0x00,0x00,0x00,0x01,0x01,0x00,0x12,0x04}},
    {0xBD,1, {0x01}},
    {0xE7,9, {0x02,0x00,0x11,0x20,0x01,0x18,0x08,0x20,0x09}},
    {0xBD,1, {0x02}},
    {0xE7,3, {0x20,0x20,0x00}},
    {0xBD,1, {0x03}},
    {0xE7,6, {0x00,0xDC,0x11,0x70,0x00,0x20}},
    {0xE9,1, {0xC9}},
    {0xE7,6, {0x34,0xD8,0x02,0x70,0x01,0x04}},
    {0xE9,1, {0x00}},
    {0xBD,1, {0x00}},
    {0xD1,1, {0x27}},
    {0xC0,1, {0x44}},
    {0xC6,1, {0xFE}},
#endif
    {0xB9,3, {0x83,0x11,0x2B}},
    {0xBD,1, {0x00}},
    {0xB2,1, {0x80}},
    {0xB4,24, {0x30,0x30,0x00,0x00,0xD0,0xD4,0x4C,0xCF,0x00,0xDD,0x00,0xD8,0x00,0x00,0x00,0x07,0x00,0x3D,0x09,0x03,0x07,0x00,0x00,0x3D}},
    {0xBD,1, {0x01}},
    {0xB1,2, {0x64,0x00}},
    {0xBD,1, {0x00}},
    {0x35,1, {0x00}},
    {0xC9,3, {0x04,0x09,0x20}},   // pwm 30khz
    {0x51,2, {0x0F,0xFF}},
    {0x53,1, {0x24}},
    {0xE9,1, {0xCD}},
    {0xBB,1, {0x23}},
    {0xE9,1, {0x00}},
    {0x55,1, {0x02}},
    {0x11,0, {}},
    {REGFLAG_DELAY, 50, {}},
    {0xC1,1, {0x01}},
    {0xB9,3, {0x00,0x00,0x00}},
    {0x29,0, {}},
    {REGFLAG_DELAY, 75, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 40, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_cabc_enter_setting[] = {
	{0x51,2,{0x0F,0xFF}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
	{0x51,2,{0x0F,0xFF}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			if (table[i].count <= 10) {
				MDELAY(table[i].count);
			} else {
				MDELAY(table[i].count);
			}
			break;
		case REGFLAG_UDELAY :
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

#if (LCM_DSI_CMD_MODE)
static void push_table22(void *handle,struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			if (table[i].count <= 10) {
				MDELAY(table[i].count);
			} else {
				MDELAY(table[i].count);
			}
		break;

		case REGFLAG_UDELAY :
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		default:
			dsi_set_cmdq_V22(handle, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}
#endif
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	if (lcm_util == NULL) {
		lcm_util = kmalloc(sizeof(LCM_UTIL_FUNCS),GFP_KERNEL);
	}
	memcpy(lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	int boot_mode = 0;

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode   = CMD_MODE;//BURST_VDO_MODE;

	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active  = 5;
	params->dsi.vertical_backporch    = 5;
	params->dsi.vertical_frontporch   = 35;
	params->dsi.vertical_active_line  = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 16;
	params->dsi.horizontal_backporch   = 16;
	params->dsi.horizontal_frontporch  = 16;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 514;
	//params->dsi.data_rate = 1029;

	params->dsi.ssc_disable = 1;
	params->dsi.CLK_TRAIL = 9;

	/* clk continuous video mode */
	params->dsi.cont_clock = 0;

	params->dsi.clk_lp_per_line_enable = 0;
	if (get_boot_mode() == META_BOOT) {
		boot_mode++;
		LCD_DEBUG("META_BOOT\n");
	}
	if (get_boot_mode() == ADVMETA_BOOT) {
		boot_mode++;
		LCD_DEBUG("ADVMETA_BOOT\n");
	}
	if (get_boot_mode() == ATE_FACTORY_BOOT) {
		boot_mode++;
		LCD_DEBUG("ATE_FACTORY_BOOT\n");
	}
	if (get_boot_mode() == FACTORY_BOOT) {
		boot_mode++;
		LCD_DEBUG("FACTORY_BOOT\n");
	}
#ifndef BUILD_LK
	if (boot_mode == 0) {
		LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
		//params->dsi.lcm_esd_check_table[0].cmd = 0x09;
		//params->dsi.lcm_esd_check_table[0].count = 3;
        //params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		//params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
        //params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;
	}
#endif
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 32;
#endif

#ifndef BUILD_LK
	register_device_proc("lcd", "hx83112b", "jdi cmd mode");
#endif
}

static void poweron_before_ulps(void)
{
	LCD_DEBUG("[himax] poweron_before_ulps 1.8v\n");
	lcd_1p8_en_setting(1);
	MDELAY(5);
}

static void lcm_init_power(void)
{
	LCD_DEBUG("[lcd] lcm_init_power\n");

	lcm_power_write_byte(0x00, 0x0F);
	lcm_power_write_byte(0x03, 0x0F);
	lcm_power_write_byte(0x01, 0x0F);
	lcm_power_write_byte(0xFF, 0xF0);

	lcd_enp_bias_setting(1);

	MDELAY(5);

	lcd_enn_bias_setting(1);
	MDELAY(5);

	lcd_rst_setting(1);
	MDELAY(5);
	lcd_rst_setting(0);
	MDELAY(5);
	lcd_rst_setting(1);
	MDELAY(50);
}

static void poweroff_after_ulps(void)
{
	LCD_DEBUG("[lcd] poweroff_after_ulps jdi td4330 1.8\n");
}

static void lcm_suspend_power(void)
{
	LCD_DEBUG("[lcd] lcm_suspend_power\n");
	lcd_rst_setting(0);
	MDELAY(6);

	lcd_enn_bias_setting(0);
	MDELAY(5);

	lcd_enp_bias_setting(0);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
	LCD_DEBUG("[lcd] lcm_resume_power \n");
	lcm_init_power();
}


static void lcm_init(void)
{
	LCD_DEBUG("[lcd]lcm_initialization_setting\n");

	push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);

	lcd_bl_en_setting(1);
	MDELAY(5);

	if (is_lm3697) {
		lm3697_write_byte(0x10,0x04);
		if (LM3697_EXPONENTIAL) {
			lm3697_write_byte(0x16,0x00);
		} else {
			lm3697_write_byte(0x16,0x01);
		}
		lm3697_write_byte(0x17, 0x13);
		lm3697_write_byte(0x18, 0x13);
		lm3697_write_byte(0x19, 0x03);
		lm3697_write_byte(0x1A, 0x04);
		lm3697_write_byte(0x1C, 0x0D);
	} else {
		lm3697_write_byte(0x10, 0x0c);
		if (MP3188_EXPONENTIAL) {
			lm3697_write_byte(0x11, 0xc9);
		} else {
			lm3697_write_byte(0x11, 0x49);
		}
		lm3697_write_byte(0x12, 0x72);
		lm3697_write_byte(0x13, 0x79);
		lm3697_write_byte(0x15, 0xe1);
		lm3697_write_byte(0x16, 0xa8);
		lm3697_write_byte(0x17, 0x50);
		lm3697_write_byte(0x1e, 0x64);
	}
}


static void lcm_suspend(void)
{
	lcd_bl_en_setting(0);
	LCD_DEBUG("[lcd] lcm_suspend BL has disabled\n");

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

	LCD_DEBUG("[lcd] lcm_suspend \n");
}

static void lcm_resume(void)
{
	LCD_DEBUG("[lcd] lcm_resume\n");
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB<<8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16)|(y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static void lcm_setbacklight(unsigned int level)
{
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s [lcd] level is %d\n", __func__, level);
#else
	printk("%s [lcd] level is %d\n", __func__, level);
#endif

	lm3697_setbacklight(level);
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s [lcd] level is %d\n", __func__, level);
#else
	/* printk("%s [soso] level is %d\n", __func__, level); */
#endif

	lm3697_setbacklight(level);
}

static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
	printk("%s [lcd] cabc_mode is %d \n", __func__, level);
#if (LCM_DSI_CMD_MODE)
	if (level) {
		push_table22(handle,lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table22(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
	}
#else
	if (level) {
		push_table(lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
	}
#endif
}

LCM_DRIVER oppo_himax83112b_1080p_dsi_cmd_lcm_drv=
{
	.name = "oppo_himax83112b_1080p_dsi_cmd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight = lcm_setbacklight,
	.set_cabc_mode_cmdq = lcm_set_cabc_mode_cmdq,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.update = lcm_update,
	.poweron_before_ulps = poweron_before_ulps,
	.poweroff_after_ulps = poweroff_after_ulps,
};
