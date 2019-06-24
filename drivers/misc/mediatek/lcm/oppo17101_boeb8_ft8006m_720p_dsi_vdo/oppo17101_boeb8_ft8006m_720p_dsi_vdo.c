/******************************************************************
** Copyright (C), 2004-2017, OPPO Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oppo_tianma_td4310_fhd1080_dsi_vdo.c
** Description: Source file for lcd drvier.
** lcd driver including parameter and power control.
** Version: 1.0
** Date : 2017/05/06
** Author: Rongchun.Zhang@EXP.MultiMedia.Display.LCD.Machine
**
** ------------------------------- Revision History:---------------
** zhangrongchun 2017/05/06 1.0 build this module
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
#endif

#include <linux/slab.h>
#include <linux/string.h>

#include "ddp_hal.h"

#include <soc/oppo/device_info.h>

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
#define FRAME_WIDTH (720)
/*
 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/07/07,
 * add for 2160 height lcd
 */
#define FRAME_HEIGHT (1440)
#define PHYSICAL_WIDTH (65)
#define PHYSICAL_HEIGHT (130)

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

/*
 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/06/23,
 * add for Lcd cabc default enable
 */
static struct LCM_setting_table lcm_initialization_video_setting[] = {
	/* video MODE*/
	{0x11,0,{}},
	{REGFLAG_DELAY, 120, {}},

	{0x50,2,{0x5A,0x0D}},
	{0x80,1,{0xF1}},

	//config te and cabc
	{0x50,2,{0x5A,0x0E}},
	{0x83,3,{0xAC,0xB4,0x6D}},

	//config cabc 02
	{0x50,2,{0x5A,0x19}},
	{0x80,16,{0xc5,0xb3,0xa5,0x9a,0x92,0x8b,0x85,0x81,0x80,0x80,0x80,0x80,0x77,0xf7,0xff,0xff}},
	{0x90,7,{0xff,0x0f,0xf0,0xbf,0x7f,0x4f,0x0f}},
	{0xA1,2,{0x00,0x44}},

	{0x50,2,{0x5A,0x23}},
	{0x90,2,{0xff,0x0f}},
	{0x94,2,{0x24,0x02}},

	//Bank 20 TE setting
	{0x50,2,{0x5A,0x14}},
	{0xF6,5,{0x18,0x01,0xBF,0x5F,0x08}},
	{0x50,2,{0x5A,0x15}},
	{0x8C,4,{0x52,0xD4,0xE0,0xE7}},
	{0x90,1,{0xF3}},
	{0x50,2,{0x00,0x2F}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 30, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},
	{0x04,1,{0x5A}},
	{0x05,1,{0x5A}}
};


static struct LCM_setting_table lcm_cabc_enter_setting[] = {
	{0x50,2,{0x5A,0x23}},
	{0x94,2,{0x2C,0x02}},
	{0x50,2,{0x00,0x2F}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
	{0x50,2,{0x5A,0x23}},
	{0x94,2,{0x2C,0x00}},
	{0x50,2,{0x00,0x2F}},
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

	params->dsi.mode   = BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	/*
	 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/07/07,
	 * add for 2160 height lcd
	 */
	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 37;
	params->dsi.vertical_frontporch = 270;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 14;
	params->dsi.horizontal_backporch = 36;
	params->dsi.horizontal_frontporch = 45;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 360;
	params->dsi.ssc_disable = 1;
	params->dsi.CLK_HS_PRPR = 7;
	params->dsi.HS_PRPR = 7;
	params->dsi.CLK_TRAIL = 7;

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

	/*
	 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/06/28,
	 * add for ESD check
	 */
	if (boot_mode == 0) {
		LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
	}
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 14;
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
	/*
	 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/06/22,
	 * add for lcd proc node
	 */
	register_device_proc("lcd", "ft8006m", "boe vdo b8");
#endif
}

/*
 * Ling.Guo@PSW.MM.Display.LCD.Stability, 2017/10/20,
 * add for 17101 tp rst
 */
//extern void focal_rst_pin_output(bool enable);
static void poweron_before_ulps(void) {
	LCD_DEBUG("[soso] poweron_before_ulps\n");
	MDELAY(50);
	lcd_ldo_setting(1);
	MDELAY(4);
	lcm_power_write_byte(0x00, 0x14);
	lcm_power_write_byte(0x01, 0x14);
	lcm_power_write_byte(0x03, 0x03);
	//lcm_power_write_byte(0xFF, 0xF0);

	lcd_enp_bias_setting(1);

	lcd_enn_bias_setting(1);
	MDELAY(2);
	if (!((get_boot_mode() == META_BOOT) || (get_boot_mode() == FACTORY_BOOT))) {
		//focal_rst_pin_output(1);
	}
	lcd_rst_setting(1);
}

static void lcm_init_power(void)
{
	LCD_DEBUG("[soso] lcm_init_power\n");
	if (!((get_boot_mode() == META_BOOT) || (get_boot_mode() == FACTORY_BOOT))) {
		MDELAY(10);
		//focal_rst_pin_output(0);
	}
	MDELAY(5);
	lcd_rst_setting(0);
	MDELAY(5);

	if (!((get_boot_mode() == META_BOOT) || (get_boot_mode() == FACTORY_BOOT))) {
		//focal_rst_pin_output(1);
	}
	MDELAY(2);
	lcd_rst_setting(1);
	MDELAY(35);
}

static void lcm_suspend_power(void)
{
	LCD_DEBUG("[soso] lcm_suspend_power\n");

	lcd_enn_bias_setting(0);

	lcd_enp_bias_setting(0);

	MDELAY(5);
	if (!((get_boot_mode() == META_BOOT) || (get_boot_mode() == FACTORY_BOOT))) {
		//focal_rst_pin_output(0);
	}
	MDELAY(5);
	lcd_rst_setting(0);
	lcd_ldo_setting(0);
}

static void lcm_resume_power(void)
{
	LCD_DEBUG("[soso] lcm_resume_power \n");
	lcm_init_power();
}


static void lcm_init(void)
{
	LCD_DEBUG("[soso]lcm_initialization_setting\n");

	push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);

	lcd_bl_en_setting(1);
	MDELAY(5);

	/*
	 * YongPeng.Yi@PSW.MM.Display.LCD.Stability, 2018/02/23,
	 * add for backlight KTD3136
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

	LCD_DEBUG("[soso] lcm_suspend \n");
}

static void lcm_resume(void)
{
	LCD_DEBUG("[soso] lcm_resume\n");
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
	dprintf(CRITICAL, "%s [soso] level is %d\n", __func__, level);
#else
	printk("%s [soso] level is %d\n", __func__, level);
#endif
	lm3697_setbacklight(level);
}
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s [soso] level is %d\n", __func__, level);
#else
	/* printk("%s [soso] level is %d\n", __func__, level); */
#endif
	lm3697_setbacklight(level);
}

/*
 * YongPeng.Yi@PSW.MultiMedia.Display.LCD.Stability, 2017/06/23,
 * add for Lcd cabc config
 */
int boeb8_ft8006m_lcm_set_cabc_mode(void *handle,int mode)
{
	int mapped_level = 0;

	if (mode == 1) {
		mapped_level = 0x01;
	} else if (mode == 2) {
		mapped_level = 0x02;
	} else if ( mode==3 ){
		mapped_level = 0x03;
	} else {
		mapped_level = 0x00;
	}
	printk("%s [soso] cabc_mode is %d \n", __func__, mode);
	if (mode) {
		push_table22(handle,lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table22(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
	}
	return 0;
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

LCM_DRIVER oppo17101_boeb8_ft8006m_720p_dsi_vdo_lcm_drv=
{
	.name = "oppo17101_boeb8_ft8006m_720p_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight = lcm_setbacklight,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.update = lcm_update,
	.poweron_before_ulps = poweron_before_ulps,
};
