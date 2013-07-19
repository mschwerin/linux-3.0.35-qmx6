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

#ifndef _BOARD_MX6Q_QMX6_H
#define _BOARD_MX6Q_QMX6_H
#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_qmx6_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_DI0_PIN4__AUDMUX_AUD6_RXD,
	MX6Q_PAD_DI0_PIN15__AUDMUX_AUD6_TXC,
	MX6Q_PAD_DI0_PIN2__AUDMUX_AUD6_TXD,
	MX6Q_PAD_DI0_PIN3__AUDMUX_AUD6_TXFS,

	MX6Q_PAD_GPIO_2__GPIO_1_2,		/* PCIE_WAKE_B */

	/* CCM  */
	MX6Q_PAD_GPIO_0__GPIO_1_0,		/* GPIO_0/Audio Ref. CLK */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */

	/* GPIO1 */
	MX6Q_PAD_ENET_RX_ER__SPDIF_IN1,		/* SPDIF_IN */

	/* GPIO2 */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* GPIO4 */
	MX6Q_PAD_GPIO_19__GPIO_4_5,	/* Volume Down */

	/* GPIO5 */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A24__GPIO_5_4,	/* J12 - Boot Mode Select */

	/* GPIO6 */
	MX6Q_PAD_EIM_A23__GPIO_6_6,	/* J12 - Boot Mode Select */

	/* GPIO7 */
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* Volume Up */

	/* I2C1 - PRIMARY */
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 - PMIC SDVO */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 - Unused */
	MX6Q_PAD_GPIO_3__I2C3_SCL,
	MX6Q_PAD_GPIO_6__I2C3_SDA,

	/* SUS_S3 */
	MX6Q_PAD_GPIO_5__GPIO_1_5,	/* GPIO1[5] */

	MX6Q_PAD_GPIO_16__GPIO_7_11,	/* GPIO7[11] */

	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* Display Connector GP */
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* Unused */

	/* PWM1 */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* PCIe RESET */
	MX6Q_PAD_SD1_DAT2__GPIO_1_19,		/* GPIO1[19] */

	/* PWM4 */
	MX6Q_PAD_SD1_CMD__PWM4_PWMO,		/* GPIO1[18] */

	/* UART2 for debug */
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* UART3 */
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,
	MX6Q_PAD_EIM_D30__UART3_CTS,
	MX6Q_PAD_EIM_D31__UART3_RTS,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* WATCHDOG */
	MX6Q_PAD_KEY_COL4__GPIO_4_14,

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_GPIO_4__GPIO_1_4,		/* Card Detect */

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
	MX6Q_PAD_NANDF_D6__GPIO_2_6,		/* SD4_CD */
	MX6Q_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */
};

static iomux_v3_cfg_t mx6q_qmx6_revA_pads[] = {

	/* PWM3 */
	MX6Q_PAD_SD1_DAT1__PWM3_PWMO,		/* GPIO1[17] */

	MX6Q_PAD_EIM_D29__GPIO_3_29,		/* PWROFF    */
	MX6Q_PAD_DISP0_DAT7__GPIO_4_28,		/* GPIO4[28] */
	MX6Q_PAD_DISP0_DAT10__GPIO_4_31,	/* GPIO4[31] */
	MX6Q_PAD_DISP0_DAT19__GPIO_5_13,	/* WAKE#	 */
	MX6Q_PAD_DISP0_DAT20__GPIO_5_14,	/* THERM#    */
	MX6Q_PAD_DISP0_DAT21__GPIO_5_15,	/* THERMTRIP#*/
	MX6Q_PAD_DISP0_DAT23__GPIO_5_17,	/* SUS_STAT# */
	MX6Q_PAD_KEY_COL0__GPIO_4_6,		/* LID_BTN#  */
	MX6Q_PAD_KEY_ROW0__GPIO_4_7,		/* SLB_BTN#	 */
	MX6Q_PAD_NANDF_CLE__GPIO_6_7,		/* SMB Alert#*/
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* GPIO2[0]  */

	/* UART5 */
	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,
};

static iomux_v3_cfg_t mx6q_qmx6_can_pads[] = {
	/* CAN1 */
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
};

static iomux_v3_cfg_t mx6q_qmx6_hdmi_ddc_pads[] = {
	MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6q_qmx6_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

static iomux_v3_cfg_t mx6q_qmx6_mipi_sensor_pads[] = {
	MX6Q_PAD_CSI0_MCLK__CCM_CLKO,           /* camera CLK (soic output at 24MHz) */
	MX6Q_PAD_NANDF_RB0__GPIO_6_10,          /* camera PWDN */
	MX6Q_PAD_GPIO_8__GPIO_1_8               /* camera RST */
};
#endif
