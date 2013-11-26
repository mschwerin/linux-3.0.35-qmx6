/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_qmx6.h"
#include "board-mx6dl_qmx6.h"

#define QMX_BOARD_REV_Y 0x0000
#define QMX_BOARD_REV_A 0x0100
#define board_is_qmx6_revy() \
	board_is_rev(QMX_BOARD_REV_Y)
#define board_is_qmx6_reva() \
	board_is_rev(QMX_BOARD_REV_A)
#define board_is_rev_or_above(rev)      (((system_rev & 0x0F00) >= rev) ? 1 : 0)

#define MX6Q_QMX6_CONN_X4_GPIO0		IMX_GPIO_NR(5,  2)
#define MX6Q_QMX6_CONN_X4_GPIO1		IMX_GPIO_NR(3, 22) // USB_OTG_PWR
#define MX6Q_QMX6_CONN_X4_GPIO2		IMX_GPIO_NR(4, 26)
#define MX6Q_QMX6_CONN_X4_GPIO3		IMX_GPIO_NR(4, 27)
#define MX6Q_QMX6_CONN_X4_GPIO4		IMX_GPIO_NR(1,  0)
#define MX6Q_QMX6_CONN_X4_GPIO5		IMX_GPIO_NR(4, 15)
#define MX6Q_QMX6_CONN_X4_GPIO6		IMX_GPIO_NR(7, 11)
#define MX6Q_QMX6_CONN_X4_GPIO7		IMX_GPIO_NR(4, 14)

#define MX6Q_QMX6_CONN_X5_1		IMX_GPIO_NR(3, 29) // PWROFF
#define MX6Q_QMX6_CONN_X5_2		IMX_GPIO_NR(7, 13) // VOLUME_UP
#define MX6Q_QMX6_CONN_X5_3		IMX_GPIO_NR(2,  4) // HOME
#define MX6Q_QMX6_CONN_X5_4		IMX_GPIO_NR(2,  3) // SEARCH
#define MX6Q_QMX6_CONN_X5_5		IMX_GPIO_NR(2,  2) // BACK
#define MX6Q_QMX6_CONN_X5_6		IMX_GPIO_NR(2,  1) // MENU
#define MX6Q_QMX6_CONN_X5_7		IMX_GPIO_NR(4,  5) // VOLUME_DOWN

#define MX6Q_QMX6_VOLUME_UP_KEY		MX6Q_QMX6_CONN_X5_2
#define MX6Q_QMX6_VOLUME_DOWN_KEY	MX6Q_QMX6_CONN_X5_7
#define MX6Q_QMX6_MENU_KEY		MX6Q_QMX6_CONN_X5_6
#define MX6Q_QMX6_BACK_KEY		MX6Q_QMX6_CONN_X5_5
#define MX6Q_QMX6_SEARCH_KEY		MX6Q_QMX6_CONN_X5_4
#define MX6Q_QMX6_HOME_KEY		MX6Q_QMX6_CONN_X5_3
#define MX6Q_QMX6_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define MX6Q_QMX6_USB_HUB_RESET		IMX_GPIO_NR(7, 12)

#define MX6Q_QMX6_SD4_CD		IMX_GPIO_NR(2,  6)
#define MX6Q_QMX6_SD4_WP		IMX_GPIO_NR(2,  7)
#define MX6Q_QMX6_SD2_CD		IMX_GPIO_NR(1,  4)
#define MX6Q_QMX6_USB_OTG_PWR		MX6Q_QMX6_CONN_X4_GPIO1
#define MX6Q_QMX6_PCIE_WAKE_B		IMX_GPIO_NR(1,  2)
#define MX6Q_QMX6_BLT_EN		IMX_GPIO_NR(1,  9)
#define MX6Q_QMX6_LCD_EN		IMX_GPIO_NR(1,  7)

#define MX6Q_QMX6_TCH_INT1		IMX_GPIO_NR(7, 11)
#define MX6Q_QMX6_CSI0_RST		IMX_GPIO_NR(1,  8)
#define MX6Q_QMX6_PCIE_RST_B		IMX_GPIO_NR(1, 20)
#define MX6Q_QMX6_CSI0_PWN		IMX_GPIO_NR(6, 10)
#define MX6Q_QMX6_PFUZE_INT		IMX_GPIO_NR(5, 16)

#define MX6Q_QMX6_PWROFF		IMX_GPIO_NR(3, 29)
#define MX6Q_QMX6_GPIO_28		IMX_GPIO_NR(4, 28)
#define MX6Q_QMX6_GPIO_31		IMX_GPIO_NR(4, 31)
#define MX6Q_QMX6_WAKE_B		IMX_GPIO_NR(5, 13)
#define MX6Q_QMX6_THERM_B		IMX_GPIO_NR(5, 14)
#define MX6Q_QMX6_TRIP_B		IMX_GPIO_NR(5, 15)
#define MX6Q_QMX6_SUS_STAT_B		IMX_GPIO_NR(5, 17)
#define MX6Q_QMX6_RTC_INT		IMX_GPIO_NR(2,  5)

void __init early_console_setup(unsigned long base, struct clk *clk);
extern int mx6q_qmx6_init_pfuze100(u32 int_gpio);

static struct clk *sata_clk;
static int mma8451_position = 1;
static int enable_lcd_ldb;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_qmx6_sd2_data __initconst = {
	.cd_gpio = MX6Q_QMX6_SD2_CD,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_qmx6_sd3_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6q_qmx6_sd4_data __initconst = {
	.cd_gpio = MX6Q_QMX6_SD4_CD,
	.wp_gpio = MX6Q_QMX6_SD4_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct anatop_thermal_platform_data
	mx6q_qmx6_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_qmx6_init_uart(void)
{
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(4, NULL);
}

static int mx6q_qmx6_fec_phy_init(struct phy_device *phydev)
{
	/* adjust KSZ9031 ethernet phy */

	phy_write(phydev, 0x0d, 0x2);
	phy_write(phydev, 0x0e, 0x4);
	phy_write(phydev, 0x0d, 0xc002);
	phy_write(phydev, 0x0e, 0x0000);

	phy_write(phydev, 0x0d, 0x2);
	phy_write(phydev, 0x0e, 0x5);
	phy_write(phydev, 0x0d, 0xc002);
	phy_write(phydev, 0x0e, 0x0000);

	phy_write(phydev, 0x0d, 0x2);
	phy_write(phydev, 0x0e, 0x6);
	phy_write(phydev, 0x0d, 0xc002);
	phy_write(phydev, 0x0e, 0xffff);

	phy_write(phydev, 0x0d, 0x2);
	phy_write(phydev, 0x0e, 0x8);
	phy_write(phydev, 0x0d, 0xc002);
	phy_write(phydev, 0x0e, 0x3fff);

	/* fix KSZ9031 link up issue */

	phy_write(phydev, 0x0d, 0x0);
	phy_write(phydev, 0x0e, 0x4);
	phy_write(phydev, 0x0d, 0x4000);
	phy_write(phydev, 0x0e, 0x6);
	phy_write(phydev, 0x0d, 0x0000);
	phy_write(phydev, 0x0e, 0x3);
	phy_write(phydev, 0x0d, 0x4000);
	phy_write(phydev, 0x0e, 0x1A80);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_qmx6_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_qmx6_spi_cs[] = {
	MX6Q_QMX6_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_qmx6_spi_data __initconst = {
	.chipselect = mx6q_qmx6_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_qmx6_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_qmx6_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "user",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 0x002FC000,
	},
	{
	 /* this 16KB area is used for congatec manufacturing purposes */
	 /* we strongly recommend not to modify or destroy this area */
	 .name = "reserved",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 0x00004000,
	 .mask_flags = MTD_WRITEABLE,
	},
};

static struct flash_platform_data imx6_qmx6__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_qmx6_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_qmx6_spi_nor_partitions),
	.type = "sst25vf032b",
};
#endif /* CONFIG_MTD_M25P80 */

static struct spi_board_info imx6_qmx6_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
	 .modalias = "m25p80",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 0,
	 .chip_select = 0,
	 .platform_data = &imx6_qmx6__spi_flash_data,
	},
#endif
};

static struct imx_ssi_platform_data mx6_qmx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_qmx6_audio_device = {
	.name = "imx-sgtl5000",
};

static struct mxc_audio_platform_data mx6_qmx6_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 6,
	.hp_gpio = -1,
};

static int mx6_qmx6_sgtl5000_init(void)
{
	mx6_qmx6_audio_data.sysclk = 24000000;
	printk("SGLT5000: audio sysclk fix configured to: %d\n",
	       mx6_qmx6_audio_data.sysclk);
	return 0;
}

static int mx6_qmx6_clko_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error: mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qmx6_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_qmx6_mipi_sensor_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_qmx6_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_qmx6_mipi_sensor_pads));

	/* Camera power down - active high */
	gpio_request(MX6Q_QMX6_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6Q_QMX6_CSI0_PWN, 1);
	gpio_set_value(MX6Q_QMX6_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX6Q_QMX6_CSI0_PWN, 0);

	/* Camera reset - active low */
	gpio_request(MX6Q_QMX6_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6Q_QMX6_CSI0_RST, 1);
	gpio_set_value(MX6Q_QMX6_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(MX6Q_QMX6_CSI0_RST, 1);

	/* for mx6dl, mipi virtual channel 0 connect to csi 0 */
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 0);
}

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown) {
		gpio_set_value(MX6Q_QMX6_CSI0_RST, 0);
		gpio_set_value(MX6Q_QMX6_CSI0_PWN, 1);
	} else {
		gpio_set_value(MX6Q_QMX6_CSI0_RST, 0);
		gpio_set_value(MX6Q_QMX6_CSI0_PWN, 0);
		msleep(1);	/* t3 >= 1ms */
		gpio_set_value(MX6Q_QMX6_CSI0_RST, 1);
		msleep(20);	/* t4 >= 20ms */
	}
}

static struct imxi2c_platform_data mx6q_qmx6_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_camera_platform_data ov5640_mipi_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	 I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
	 I2C_BOARD_INFO("mma8451", 0x1c),
	 .platform_data = (void *)&mma8451_position,
	},
	{
	 I2C_BOARD_INFO("ov5640_mipi", 0x3c),
	 .platform_data = (void *)&ov5640_mipi_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	 I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
	},
	{
	 I2C_BOARD_INFO("m41t62", 0x68),
	 .platform_data = (void *)0,
	 .irq = gpio_to_irq(MX6Q_QMX6_RTC_INT),
	},
};

static void imx6q_qmx6_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6Q_QMX6_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_QMX6_USB_OTG_PWR, 0);
}

static void __init imx6q_qmx6_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_QMX6_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_QMX6_USB_OTG_PWR: %d\n", ret);
		return;
	}
	gpio_direction_output(MX6Q_QMX6_USB_OTG_PWR, 0);

	mxc_iomux_set_gpr_register(1, 13, 1, 1);
	mx6_set_otghost_vbus_func(imx6q_qmx6_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_qmx6_sata_init(struct device *dev, void __iomem * addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_qmx6_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_qmx6_sata_data = {
	.init = mx6q_qmx6_sata_init,
	.exit = mx6q_qmx6_sata_exit,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data qmx6_fb_data[] = {
	{ /*fb0 */
	 .disp_dev = "ldb",
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 .mode_str = "LDB-XGA",
	 .default_bpp = 16,
	 .int_clk = false,
	},
	{
	 .disp_dev = "ldb",
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 .mode_str = "LDB-SVGA",
	 .default_bpp = 16,
	 .int_clk = false,
	},
	{
	 .disp_dev = "ldb",
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 .mode_str = "LDB-SVGA",
	 .default_bpp = 16,
	 .int_clk = false,
	},
	{
	 .disp_dev = "ldb",
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 .mode_str = "LDB-VGA",
	 .default_bpp = 16,
	 .int_clk = false,
	},
};

#if defined(CONFIG_MFD_MXC_HDMI) || defined(CONFIG_MFD_MXC_HDMI_MODULE)

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabresd board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qmx6_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_qmx6_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_qmx6_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_qmx6_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qmx6_i2c2_pads,
			ARRAY_SIZE(mx6dl_qmx6_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_qmx6_i2c2_pads,
			ARRAY_SIZE(mx6q_qmx6_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

#endif /* CONFIG_MFD_MXC_HDMI */

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	 .rev = 4,
	 .csi_clk[0] = "clko2_clk",
	 .bypass_reset = false,
	},
	{
	 .rev = 4,
	 .csi_clk[0] = "clko2_clk",
	 .bypass_reset = false,
	},
};

static void qmx6_suspend_enter(void)
{
	/* suspend preparation */

	/* disable backlight */
	gpio_set_value(MX6Q_QMX6_BLT_EN, 0);
}

static void qmx6_suspend_exit(void)
{
	/* resume restore */

	/* enable backlight */
	gpio_set_value(MX6Q_QMX6_BLT_EN, 1);
}

static const struct pm_platform_data mx6q_qmx6_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = qmx6_suspend_enter,
	.suspend_exit = qmx6_suspend_exit,
};

static struct regulator_consumer_supply qmx6_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data qmx6_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(qmx6_vmmc_consumers),
	.consumer_supplies = qmx6_vmmc_consumers,
};

static struct fixed_voltage_config qmx6_vmmc_reg_config = {
	.supply_name = "vmmc",
	.microvolts = 3300000,
	.gpio = -1,
	.init_data = &qmx6_vmmc_init,
};

static struct platform_device qmx6_vmmc_reg_devices = {
	.name = "reg-fixed-voltage",
	.id = 3,
	.dev = {
		.platform_data = &qmx6_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_qmx6_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_qmx6_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_qmx6_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_qmx6_consumer_vdda,
};

static struct regulator_init_data sgtl5000_qmx6_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_qmx6_consumer_vddio,
};

static struct fixed_voltage_config sgtl5000_qmx6_vdda_reg_config = {
	.supply_name = "VDDA",
	.microvolts = 2500000,
	.gpio = -1,
	.init_data = &sgtl5000_qmx6_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_qmx6_vddio_reg_config = {
	.supply_name = "VDDIO",
	.microvolts = 3300000,
	.gpio = -1,
	.init_data = &sgtl5000_qmx6_vddio_reg_initdata,
};

static struct platform_device sgtl5000_qmx6_vdda_reg_devices = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data = &sgtl5000_qmx6_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_qmx6_vddio_reg_devices = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev = {
		.platform_data = &sgtl5000_qmx6_vddio_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_qmx6_audio_device, &mx6_qmx6_audio_data);
	imx6q_add_imx_ssi(1, &mx6_qmx6_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_qmx6_vdda_reg_devices);
	platform_device_register(&sgtl5000_qmx6_vddio_reg_devices);
	mx6_qmx6_sgtl5000_init();
#endif
	return 0;

}

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
	GPIO_BUTTON(MX6Q_QMX6_PWROFF, KEY_POWER, 1, "key-power", 1, 1),
	GPIO_BUTTON(MX6Q_QMX6_MENU_KEY, KEY_MENU, 1, "key-menu", 0, 1),
	GPIO_BUTTON(MX6Q_QMX6_HOME_KEY, KEY_HOME, 1, "key-home", 0, 1),
	GPIO_BUTTON(MX6Q_QMX6_BACK_KEY, KEY_BACK, 1, "key-back", 0, 1),
	GPIO_BUTTON(MX6Q_QMX6_SEARCH_KEY, KEY_SEARCH, 1, "key-search", 0, 1),
	GPIO_BUTTON(MX6Q_QMX6_VOLUME_UP_KEY, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(MX6Q_QMX6_VOLUME_DOWN_KEY, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons = imx6q_buttons,
	.nbuttons = ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name = "gpio-keys",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}

#endif

static struct platform_pwm_backlight_data mx6_qmx6_pwm_backlight_data = {
	.pwm_id = 3,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static const struct flexcan_platform_data mx6_qmx6_flexcan_pdata[] __initconst = {
	{
	 .transceiver_switch = NULL,
	},
};

static struct mxc_dvfs_platform_data qmx6_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id = "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = qmx6_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(qmx6_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			break;
		}
	}
}

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{
	void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit */
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_qmx6_pcie_data __initconst = {
	.pcie_pwr_en = -1,
	.pcie_rst = MX6Q_QMX6_PCIE_RST_B,
	.pcie_wake_up = MX6Q_QMX6_PCIE_WAKE_B,
	.pcie_dis = -1,
};

static int __init early_enable_lcd_ldb(char *p)
{
	enable_lcd_ldb = 1;
	return 0;
}

early_param("enable_lcd_ldb", early_enable_lcd_ldb);

static struct mipi_csi2_platform_data mipi_csi2_data = {
	.ipu_id = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static struct fsl_mxc_capture_platform_data capture_data = {
	.csi = 0,
	.ipu = 0,
	.mclk_source = 0,
	.is_mipi = 1
};

/*!
 * Board specific initialization.
 */
static void __init mx6_qmx6_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *revA_pads = NULL;
	iomux_v3_cfg_t *flexcan_pads = NULL;

	int common_pads_cnt;
	int flexcan_pads_cnt;
	int revA_pads_cnt;

	if (cpu_is_mx6q()) {
		common_pads = mx6q_qmx6_pads;
		flexcan_pads = mx6q_qmx6_can_pads;
		revA_pads = mx6q_qmx6_revA_pads;

		common_pads_cnt = ARRAY_SIZE(mx6q_qmx6_pads);
		flexcan_pads_cnt = ARRAY_SIZE(mx6q_qmx6_can_pads);
		revA_pads_cnt = ARRAY_SIZE(mx6q_qmx6_revA_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = mx6dl_qmx6_pads;
		flexcan_pads = mx6dl_qmx6_can_pads;
		revA_pads = mx6dl_qmx6_revA_pads;

		common_pads_cnt = ARRAY_SIZE(mx6dl_qmx6_pads);
		flexcan_pads_cnt = ARRAY_SIZE(mx6dl_qmx6_can_pads);
		revA_pads_cnt = ARRAY_SIZE(mx6dl_qmx6_revA_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);
	if (board_is_rev_or_above(QMX_BOARD_REV_A))
		mxc_iomux_v3_setup_multiple_pads(revA_pads, revA_pads_cnt);

	gp_reg_id = qmx6_dvfscore_data.reg_id;
	soc_reg_id = qmx6_dvfscore_data.soc_id;
	pu_reg_id = qmx6_dvfscore_data.pu_id;
	mx6q_qmx6_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 1;
		if (enable_lcd_ldb) {
			ldb_data.disp_id = 1;
			ldb_data.mode = LDB_SIN1;
		}
	}

#if defined(CONFIG_MFD_MXC_HDMI) || defined(CONFIG_MFD_MXC_HDMI_MODULE)
	if (cpu_is_mx6dl()) {
		hdmi_core_data.disp_id = 1;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);
#endif /* CONFIG_MFD_MXC_HDMI */

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(qmx6_fb_data); i++)
			imx6q_add_ipuv3fb(i, &qmx6_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(qmx6_fb_data); i++)
			imx6q_add_ipuv3fb(i, &qmx6_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data);
	imx6q_add_mipi_csi2(&mipi_csi2_data);

	if (board_is_qmx6_revy()) {
		/* onboard SNVS RTC */
		imx6q_add_imx_snvs_rtc();
	} else {
		/* external I2C RTC */
		BUG_ON(gpio_request(MX6Q_QMX6_RTC_INT, "rtc"));
		BUG_ON(gpio_direction_input(MX6Q_QMX6_RTC_INT));
	}

	imx6q_add_imx_i2c(0, &mx6q_qmx6_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_qmx6_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_qmx6_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(MX6Q_QMX6_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR "request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(MX6Q_QMX6_PFUZE_INT);
		mx6q_qmx6_init_pfuze100(MX6Q_QMX6_PFUZE_INT);
	}

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_qmx6_spi_data);
	spi_register_board_info(imx6_qmx6_board_info,
			ARRAY_SIZE(imx6_qmx6_board_info));

#if defined(CONFIG_MFD_MXC_HDMI) || defined(CONFIG_MFD_MXC_HDMI_MODULE)
	imx6q_add_mxc_hdmi(&hdmi_data);
#endif /* CONFIG_MFD_MXC_HDMI */

	imx6q_add_anatop_thermal_imx(1, &mx6q_qmx6_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_qmx6_pm_data);

	/* Move sd3 to first because sd3 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_qmx6_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_qmx6_sd3_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_qmx6_sd4_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_qmx6_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q())
		imx6q_add_ahci(0, &mx6q_qmx6_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	mx6_qmx6_clko_init();
	platform_device_register(&qmx6_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* release USB Hub reset */
	gpio_set_value(MX6Q_QMX6_USB_HUB_RESET, 1);

	/* fan & backlight PWM */
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(3, &mx6_qmx6_pwm_backlight_data);

	/* switch on lcd vcc */
	gpio_request(MX6Q_QMX6_LCD_EN, "lcden");
	gpio_direction_output(MX6Q_QMX6_LCD_EN, 1);
	gpio_set_value(MX6Q_QMX6_LCD_EN, 1);

	/* switch on backlight */
	gpio_request(MX6Q_QMX6_BLT_EN, "backlight");
	gpio_direction_output(MX6Q_QMX6_BLT_EN, 1);
	gpio_set_value(MX6Q_QMX6_BLT_EN, 1);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&qmx6_dvfscore_data);

	/* register GPIO keys */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	imx6q_add_device_buttons();
#endif /* CONFIG_KEYBOARD_GPIO */

#if defined(CONFIG_MFD_MXC_HDMI) || defined(CONFIG_MFD_MXC_HDMI_MODULE)
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
#endif /* CONFIG_MFD_MXC_HDMI */

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	mxc_iomux_v3_setup_multiple_pads(flexcan_pads, flexcan_pads_cnt);
	imx6q_add_flexcan0(&mx6_qmx6_flexcan_pdata[0]);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	imx6q_add_pcie(&mx6_qmx6_pcie_data);
	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	if (board_is_rev_or_above(QMX_BOARD_REV_A))
		printk("board_init: board is rev A or above\n");
	else
		printk("board_init: board is rev Y\n");
}

extern void __iomem *twd_base;
static void __init mx6_qmx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_qmx6_timer = {
	.init = mx6_qmx6_timer_init,
};

static void __init mx6q_qmx6_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_QMX6 data structure.
 */
MACHINE_START(MX6Q_QMX6, "Congatec i.MX 6Quad QMX6 Board")
	/* Maintainer: congatec */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_qmx6_board_init,
	.timer = &mx6_qmx6_timer,
	.reserve = mx6q_qmx6_reserve,
MACHINE_END
