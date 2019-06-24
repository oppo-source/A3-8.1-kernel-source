/******************************************************************
** Copyright (C), 2004-2017, OPPO Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oppo_auo_nt36672_1080p_dsi_vdo.c
** Description: Source file for lcd drvier.
** lcd driver including parameter and power control.
** Version: 1.0
** Date : 2017/05/06
** Author: LiPing-M@PSW.MultiMedia.Display.LCD.Machine
**
** ------------------------------- Revision History:---------------
** liping 2018/01/20 1.0 build this module
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


#define SYSFS_FOLDER "dsi_access"
#define BUFFER_LENGTH 128


enum read_write {
	CMD_READ = 0,
	CMD_WRITE = 1,
};

/*
struct dsi_debug {
	bool long_rpkt;
	unsigned char length;
	unsigned char rlength;
	unsigned char buffer[BUFFER_LENGTH];
	unsigned char read_buffer[BUFFER_LENGTH];
	unsigned char command_len;
	unsigned char *command_buf;
	unsigned char cmds[64];
};

static struct dsi_debug *debug = NULL;
static struct dsi_debug *debug_read = NULL;
*/

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

#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (2280)
#define PHYSICAL_WIDTH (68)
#define PHYSICAL_HEIGHT (143)
#define PHYSICAL_WIDTH_UM (68000)
#define PHYSICAL_HEIGHT_UM (143000)

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

extern int lcm_power_write_byte(unsigned char addr,  unsigned char value);
extern int lm3697_write_byte(unsigned char addr,  unsigned char value);
extern int lm3697_setbacklight(unsigned int level);

extern int tp_control_reset_gpio(bool enable);

#define REGFLAG_DELAY      0xA0
#define REGFLAG_UDELAY     0xA1

#define REGFLAG_END_OF_TABLE   0xA2
#define REGFLAG_RESET_LOW  0xA3
#define REGFLAG_RESET_HIGH  0xA4

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_video_setting[] = {
	{0XFF,1,  {0X20}},
	{0XFB,1,  {0X01}},
	{0X0E,1,  {0XB0}},
	{0X0F,1,  {0XAE}},
	{0X62,1,  {0XBD}},
	{0X69,1,  {0X99}},
	{0X6D,1,  {0X44}},
	{0X78,1,  {0X01}},
	{0XFF,1,  {0X24}},
	{0XFB,1,  {0X01}},
	{0X00,1,  {0X1C}},
	{0X01,1,  {0X1C}},
	{0X02,1,  {0X1C}},
	{0X03,1,  {0X1C}},
	{0X04,1,  {0X20}},
	//G6L_SWAP[5:0]: 0'd->13'd
	{0X05,1,  {0X0D}},
	{0X06,1,  {0X09}},
	{0X07,1,  {0X0A}},
	{0X08,1,  {0X1E}},
	{0X09,1,  {0X0D}},
	{0X0A,1,  {0X0D}},
	{0X0B,1,  {0X25}},
	{0X0C,1,  {0X24}},
	{0X0D,1,  {0X01}},
	{0X0E,1,  {0X04}},
	{0X0F,1,  {0X04}},
	{0X10,1,  {0X03}},
	{0X11,1,  {0X03}},
	{0X12,1,  {0X14}},
	{0X13,1,  {0X14}},
	{0X14,1,  {0X12}},
	{0X15,1,  {0X12}},
	{0X16,1,  {0X10}},
	{0X17,1,  {0X1C}},
	{0X18,1,  {0X1C}},
	{0X19,1,  {0X1C}},
	{0X1A,1,  {0X1C}},
	{0X1B,1,  {0X20}},
	//G6R_SWAP[5:0]: 0'd->13'd
	{0X1C,1,  {0X0D}},
	{0X1D,1,  {0X09}},
	{0X1E,1,  {0X0A}},
	{0X1F,1,  {0X1E}},
	{0X20,1,  {0X0D}},
	{0X21,1,  {0X0D}},
	{0X22,1,  {0X25}},
	{0X23,1,  {0X24}},
	{0X24,1,  {0X01}},
	{0X25,1,  {0X04}},
	{0X26,1,  {0X04}},
	{0X27,1,  {0X03}},
	{0X28,1,  {0X03}},
	{0X29,1,  {0X14}},
	{0X2A,1,  {0X14}},
	{0X2B,1,  {0X12}},
	{0X2D,1,  {0X12}},
	{0X2F,1,  {0X10}},
	{0X31,1,  {0X02}},
	{0X32,1,  {0X03}},
	{0X33,1,  {0X04}},
	{0X34,1,  {0X02}},
	{0X37,1,  {0X09}},
	{0X38,1,  {0X74}},
	{0X39,1,  {0X74}},
	{0X3F,1,  {0X74}},
	{0X41,1,  {0X02}},
	{0X42,1,  {0X03}},
	{0X4C,1,  {0X10}},
	{0X4D,1,  {0X10}},
	{0X61,1,  {0XE8}},
	{0X72,1,  {0X00}},
	{0X73,1,  {0X00}},
	{0X74,1,  {0X00}},
	{0X75,1,  {0X00}},
	{0X79,1,  {0X23}},
	{0X7A,1,  {0X13}},
	{0X7B,1,  {0X99}},
	{0X7C,1,  {0X80}},
	{0X7D,1,  {0X09}},
	{0X80,1,  {0X42}},
	{0X82,1,  {0X11}},
	{0X83,1,  {0X22}},
	{0X84,1,  {0X33}},
	{0X85,1,  {0X00}},
	{0X86,1,  {0X00}},
	{0X87,1,  {0X00}},
	{0X88,1,  {0X11}},
	{0X89,1,  {0X22}},
	{0X8A,1,  {0X33}},
	{0X8B,1,  {0X00}},
	{0X8C,1,  {0X00}},
	{0X8D,1,  {0X00}},
	{0X92,1,  {0X77}},
	{0XB3,1,  {0X02}},
	{0XB4,1,  {0X00}},
	{0XDC,1,  {0X64}},
	{0XDD,1,  {0X03}},
	{0XDF,1,  {0X3E}},
	{0XE0,1,  {0X3E}},
	{0XE1,1,  {0X22}},
	{0XE2,1,  {0X24}},
	{0XE3,1,  {0X09}},
	{0XE4,1,  {0X09}},
	{0XEB,1,  {0X13}},
	{0XFF,1,  {0X25}},
	{0XFB,1,  {0X01}},
	{0X21,1,  {0X19}},
	{0X22,1,  {0X19}},
	{0X24,1,  {0X77}},
	{0X25,1,  {0X77}},
	{0X2F,1,  {0X10}},
	{0X30,1,  {0X30}},
	{0X38,1,  {0X30}},
	{0X40,1,  {0XC7}},
	{0X4C,1,  {0XC7}},
	{0X58,1,  {0X22}},
	{0X59,1,  {0X05}},
	{0X5A,1,  {0X09}},
	{0X5B,1,  {0X09}},
	//RST_PON_SW[2:0]: 3->4
	{0X5C,1,  {0X25}},
	{0X5E,1,  {0XF0}},
	{0X5F,1,  {0X28}},
	{0X66,1,  {0XD8}},
	{0X67,1,  {0X2B}},
	{0X68,1,  {0X58}},
	{0X6B,1,  {0X00}},
	{0X6C,1,  {0X6D}},
	{0X77,1,  {0X72}},
	{0XBF,1,  {0X00}},
	{0XC2,1,  {0X5A}},
	//GOFF1_ABOFF_LEVEL: 1->0
	{0XC3,1,  {0X01}},
	{0XFF,1,  {0X26}},
	{0XFB,1,  {0X01}},
	{0X06,1,  {0XFF}},
	{0X0C,1,  {0X09}},
	{0X0F,1,  {0X05}},
	{0X10,1,  {0X06}},
	{0X12,1,  {0XE4}},
	{0X19,1,  {0X16}},
	{0X1A,1,  {0X38}},
	{0X1C,1,  {0XAA}},
	{0X1D,1,  {0X15}},
	{0X1E,1,  {0XBB}},
	{0X98,1,  {0XF1}},
	{0XAE,1,  {0X6A}},
	{0XFF,1,  {0X27}},
	{0XFB,1,  {0X01}},
	{0X13,1,  {0X00}},
	{0XFF,1,  {0XF0}},
	{0XFB,1,  {0X01}},
	{0XA2,1,  {0X00}},
	{0xFF,1,  {0x23}},
	{0xFB,1,  {0x01}},
	{0x11,1,  {0x01}},
	{0x12,1,  {0x7E}},
	{0x15,1,  {0x6E}},
	{0x16,1,  {0x0B}},
	{0x45,1,  {0xC3}},
	{0x46,1,  {0xC0}},
	{0x47,1,  {0xBD}},
	{0x48,1,  {0xBA}},
	{0x49,1,  {0xB7}},
	{0x4A,1,  {0xB4}},
	{0x4B,1,  {0xB1}},
	{0x4C,1,  {0xAE}},
	{0x4D,1,  {0xAB}},
	{0x4E,1,  {0xA8}},
	{0x4F,1,  {0xA5}},
	{0x50,1,  {0xA2}},
	{0x51,1,  {0x9F}},
	{0x52,1,  {0x9C}},
	{0x53,1,  {0x99}},
	{0x54,1,  {0x96}},
	{0x6F,1,  {0x00}},
	{0x70,1,  {0x44}},
	{0x71,1,  {0x9A}},
	{0x05,1,  {0x24}},
	{0x07,1,  {0x20}},
	{0x08,1,  {0x04}},
	{0x09,1,  {0x00}},
	{0xFF,1,  {0x10}},
	{0xFB,1,  {0x01}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 130, {}},
	{0x51,1,  {0xFF}},
	{0x53,1,  {0x2c}},
	{0x55,1,  {0x02}},
	/* TE ON */
	{0x35,1,{0x00}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},
	{0x4F,1,{0x01}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_cabc_enter_setting[] = {
	{0x51,1,{0xFF}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
	{0x51,1,{0xFF}},
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

	//params->physical_width = PHYSICAL_WIDTH;
	//params->physical_height = PHYSICAL_HEIGHT;
	params->physical_width_um = PHYSICAL_WIDTH_UM;
	params->physical_height_um = PHYSICAL_HEIGHT_UM;

	params->dsi.mode   = BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;

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

	params->dsi.vertical_sync_active                = 2;
	params->dsi.vertical_backporch                    = 10;
	params->dsi.vertical_frontporch                    = 25;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active                = 2;
	params->dsi.horizontal_backporch                = 30;
	params->dsi.horizontal_frontporch                = 40;
	params->dsi.horizontal_active_pixel         = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 514;
	params->dsi.data_rate = 1029;

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

	if (boot_mode == 0) {
		LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
		//params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
		//params->dsi.lcm_esd_check_table[0].count = 1;
		//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
	}
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 1;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 91;
	params->corner_pattern_height_bot = 72;
#endif
	/*
	if (debug == NULL) {
		debug = kmalloc(sizeof(dsi_debug),GFP_KERNEL);
	}
	if (debug_read == NULL) {
		debug_read = kmalloc(sizeof(dsi_debug),GFP_KERNEL);
	}
	*/
#ifndef BUILD_LK
	register_device_proc("lcd", "nt36672", "truly vdo mode");
#endif
}

static void poweron_before_ulps(void)
{
	LCD_DEBUG("[himax] poweron_before_ulps 1.8v\n");
	lcd_1p8_en_setting(1);
	tp_control_reset_gpio(true);
	MDELAY(5);
	lcm_power_write_byte(0x00, 0x0F);
	lcm_power_write_byte(0x03, 0x0F);
	lcm_power_write_byte(0x01, 0x0F);
	lcm_power_write_byte(0xFF, 0xF0);
	MDELAY(5);
	lcd_enp_bias_setting(1);

	MDELAY(10);

	lcd_enn_bias_setting(1);
	MDELAY(10);
}

static void lcm_init_power(void)
{
	LCD_DEBUG("[auo] lcm_init_power\n");

	lcd_rst_setting(1);
	MDELAY(5);
	lcd_rst_setting(0);
	MDELAY(5);
	lcd_rst_setting(1);
	MDELAY(110);

}

static void lcm_suspend_power(void)
{
	LCD_DEBUG("[auo] lcm_suspend_power\n");
	lcd_rst_setting(0);
	MDELAY(6);

	tp_control_reset_gpio(false);
	MDELAY(5);

	lcd_enn_bias_setting(0);
	MDELAY(10);

	lcd_enp_bias_setting(0);
	MDELAY(10);
}

static void lcm_resume_power(void)
{
	LCD_DEBUG("[auo] lcm_resume_power \n");
	lcm_init_power();
}

static void poweroff_after_ulps(void)
{
	LCD_DEBUG("[lcd] poweroff_after_ulps auo nt36672 1.8\n");
	lcd_1p8_en_setting(0);
	MDELAY(10);
}

static void lcm_init(void)
{
	LCD_DEBUG("[auo]lcm_initialization_setting\n");

	push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);

	lcd_bl_en_setting(1);
	MDELAY(5);

	/*
	 * Guoqiang.jiang@MM.Display.LCD.Machine, 2018/03/13,
	 * add for backlight IC KTD3136
	 */
	if (is_lm3697 == 2) {   /*KTD3136*/
		lm3697_write_byte(0x02, 0x98);
		if (KTD3136_EXPONENTIAL) {
			lm3697_write_byte(0x03, 0x28);   /*32V500KHz exp*/
		} else {
			lm3697_write_byte(0x03, 0x2a);   /*32V500KHz Liner*/
		}
		lm3697_write_byte(0x06, 0x1B);
		lm3697_write_byte(0x07, 0x00);
		lm3697_write_byte(0x08, 0x00);
	} else if (is_lm3697 == 1) {
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
	LCD_DEBUG(" lcm_suspend BL has disabled\n");

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

/*
int parse_input(void *handle,const char *buf)
{
	int retval = 0;
	int index = 0;
	int cmdindex = 0;
	int ret = 0;
	char *input = (char *)buf;
	char *token = NULL;
	unsigned long value = 0;

	input[strlen(input)] = '\0';

	while (input != NULL && index < BUFFER_LENGTH) {
		token = strsep(&input, " ");
		retval = kstrtoul(token, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert from string (%s) to hex number\n", __func__, token);
			continue;
		}
		debug->buffer[index] = (unsigned char)value;
		index++;
	}

	if (index > 1) {
		debug->length = index - 1;
	}

	if (debug->length <= 0) {
		return 0;
	}

	while (cmdindex < debug->length) {
		debug->cmds[cmdindex] = debug->buffer[cmdindex+1];
		cmdindex++;
	}
	printk("%s [soso] debug->buffer[0] is 0x%x debug->length = %d \n", __func__, debug->buffer[0],debug->length);

	while (ret < debug->length) {
		printk("[soso] debug->cmds is 0x%x\n",  debug->cmds[ret]);
		ret++;
	}
	dsi_set_cmdq_V22(handle, debug->buffer[0], debug->length, debug->cmds, 1);

	return 1;
}

extern unsigned char read_buffer[128];
extern int reg_rlengh;

int parse_reg_output(void *handle,const char *buf)
{
	int retval = 0;
	int index = 0;
	int ret = 0;
	char *input = (char *)buf;
	char *token = NULL;
	unsigned long value = 0;

	input[strlen(input)] = '\0';

	while (input != NULL && index < BUFFER_LENGTH) {
		token = strsep(&input, " ");
		retval = kstrtoul(token, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert from string (%s) to hex number\n", __func__, token);
			continue;
		}
		debug_read->buffer[index] = (unsigned char)value;
		index++;
	}

	if (index > 1) {
		debug_read->length = debug_read->buffer[1];
	}

	if (debug_read->length <= 0) {
		return 0;
	}

	reg_rlengh = debug_read->length;

	printk("%s [soso] debug->buffer[0] is 0x%x debug->length = %d \n", __func__, debug_read->buffer[0],debug_read->length);

	retval = read_reg_v2(debug_read->buffer[0],debug_read->read_buffer,debug_read->length);

	if (retval<0) {
		printk("%s [soso] error can not read the reg 0x%x \n", __func__,debug_read->buffer[0]);
		return -1;
	}

	while (ret < debug_read->length) {
		printk("[soso] reg cmd 0x%x read_buffer is 0x%x \n", debug_read->buffer[0], debug_read->read_buffer[ret]);
		read_buffer[ret] = debug_read->read_buffer[ret];
		ret++;
	}
	return 1;
}
*/

LCM_DRIVER oppo_auo_nt36672_1080p_dsi_vdo_lcm_drv=
{
	.name = "oppo17331_auo_nt36672_1080p_dsi_vdo",
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
