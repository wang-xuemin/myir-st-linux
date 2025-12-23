/*
 * drivers/net/phy/motorcomm.c
 *
 * Driver for Motorcomm PHYs
 *
 * Author: Leilei Zhao <leilei.zhao@motorcomm.com>
 *
 * Copyright (c) 2019 Motorcomm, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support : Motorcomm Phys:
 *		Giga phys: yt8511, yt8521
 *		100/10 Phys : yt8512, yt8512b, yt8510
 *		Automotive 100Mb Phys : yt8010
 *		Automotive 100/10 hyper range Phys: yt8510
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/motorcomm_phy.h>
#include <linux/of.h>
#include <linux/clk.h>
#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif
/*for wol, 20210604*/
#include <linux/netdevice.h>

#include "yt8614-phy.h"

/**** configuration section begin ***********/

/* if system depends on ethernet packet to restore from sleep, please define this macro to 1
 * otherwise, define it to 0.
 */
#define SYS_WAKEUP_BASED_ON_ETH_PKT 	0

/* to enable system WOL of phy, please define this macro to 1
 * otherwise, define it to 0.
 */
#define YTPHY_ENABLE_WOL 		0

/* some GMAC need clock input from PHY, for eg., 125M, please enable this macro
 * by degault, it is set to 0
 * NOTE: this macro will need macro SYS_WAKEUP_BASED_ON_ETH_PKT to set to 1
 */
#define GMAC_CLOCK_INPUT_NEEDED  1


#define YT8521_PHY_MODE_FIBER	1 //fiber mode only
#define YT8521_PHY_MODE_UTP		2 //utp mode only
#define YT8521_PHY_MODE_POLL	3 //fiber and utp, poll mode

/* please make choice according to system design
 * for Fiber only system, please define YT8521_PHY_MODE_CURR 1
 * for UTP only system, please define YT8521_PHY_MODE_CURR 2
 * for combo system, please define YT8521_PHY_MODE_CURR 3 
 */
#define YT8521_PHY_MODE_CURR	3

/**** configuration section end ***********/


/* no need to change below */

#if (YTPHY_ENABLE_WOL)
#undef SYS_WAKEUP_BASED_ON_ETH_PKT
#define SYS_WAKEUP_BASED_ON_ETH_PKT 	1
#endif

/* workaround for 8521 fiber 100m mode */
static int link_mode_8521 = 0; //0: no link; 1: utp; 32: fiber. traced that 1000m fiber uses 32.
static int link_mode_8614[4] = {0}; //0: no link; 1: utp; 32: fiber. traced that 1000m fiber uses 32.

/* for multiple port phy, base phy address */
static unsigned int yt_mport_base_phy_addr = 0xff; //0xff: invalid; for 8618
static unsigned int yt_mport_base_phy_addr_8614 = 0xff; //0xff: invalid;

#if ( LINUX_VERSION_CODE > KERNEL_VERSION(5,0,0) )
int genphy_config_init(struct phy_device *phydev)
{
	return  genphy_read_abilities(phydev);
}
#endif

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
static int ytphy_config_init(struct phy_device *phydev)
{
	return 0;
}
#endif

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;
	int val;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_DEBUG_DATA);

	return val;
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, REG_DEBUG_DATA, val);

	return ret;
}

static int yt8010_config_aneg(struct phy_device *phydev)
{
	phydev->speed = SPEED_100;
	return 0;
}

static int yt8521_delaysel_init(struct phy_device *phydev) {
  int ret;
  int val, tmp;

  val = ytphy_read_ext(phydev, YT8521_EXTREG_RGMII_CONFIG1);
  if (val < 0)
    return val;

  // modify tx delay sel , 1000m
  tmp = (val & 0xF0) | 0x08;
  ret = ytphy_write_ext(phydev, YT8521_EXTREG_RGMII_CONFIG1, tmp);
  return ret;
}

static int yt8512_clk_init(struct phy_device *phydev)
{
	int ret;
	int val;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_AFE_PLL);
	if (val < 0)
		return val;

	val |= YT8512_CONFIG_PLL_REFCLK_SEL_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_AFE_PLL, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_EXTEND_COMBO);
	if (val < 0)
		return val;

	val |= YT8512_CONTROL1_RMII_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_EXTEND_COMBO, val);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, MII_BMCR);
	if (val < 0)
		return val;

	val |= YT_SOFTWARE_RESET;
	ret = phy_write(phydev, MII_BMCR, val);

	return ret;
}

static int yt8512_led_init(struct phy_device *phydev)
{
	int ret;
	int val;
	int mask;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED0);
	if (val < 0)
		return val;

	val |= YT8512_LED0_ACT_BLK_IND;

	mask = YT8512_LED0_DIS_LED_AN_TRY | YT8512_LED0_BT_BLK_EN |
		YT8512_LED0_HT_BLK_EN | YT8512_LED0_COL_BLK_EN |
		YT8512_LED0_BT_ON_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED0, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED1);
	if (val < 0)
		return val;

	val |= YT8512_LED1_BT_ON_EN;

	mask = YT8512_LED1_TXACT_BLK_EN | YT8512_LED1_RXACT_BLK_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED1, val);

	return ret;
}

static int yt8521_led_init(struct phy_device *phydev)
{
    int ret;
    int val;

    val = ytphy_read_ext(phydev, YT8521_EXTREG_LED1);
    if (val < 0)
        return val;

    /* set when link up and speed is 10/100/1000 make led on  as link led */
    val = 0x70;
    ret = ytphy_write_ext(phydev, YT8521_EXTREG_LED2, val);
    if (ret < 0)
        return ret;

    val = ytphy_read_ext(phydev, YT8521_EXTREG_LED2);
    if (val < 0)
        return val;

    /* when rx and tx send or recive msg make led link  as stats led*/
    val = 0x180;
    ret = ytphy_write_ext(phydev, YT8521_EXTREG_LED1, val);

    return ret;
}

static int yt8512_config_init(struct phy_device *phydev)
{
	int ret;
	int val;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	ret = ytphy_config_init(phydev);
#else
	ret = genphy_config_init(phydev);
#endif
	if (ret < 0)
		return ret;

	ret = yt8512_clk_init(phydev);
	if (ret < 0)
		return ret;

	ret = yt8512_led_init(phydev);

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8512_EN_SLEEP_SW_BIT));

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	return ret;
}

static int yt8512_read_status(struct phy_device *phydev)
{
	int ret;
	int val;
	int speed, speed_mode, duplex;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	duplex = (val & YT8512_DUPLEX) >> YT8512_DUPLEX_BIT;
	speed_mode = (val & YT8512_SPEED_MODE) >> YT8512_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
	case 3:
	default:
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		speed = -1;
#else
		speed = SPEED_UNKNOWN;
#endif
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	return 0;
}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
#if 0
int yt8521_soft_reset(struct phy_device *phydev)
{
	int ret;

	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	ytphy_write_ext(phydev, 0xa000, 2);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		ytphy_write_ext(phydev, 0xa000, 0);
		return ret;
	}

	return 0;
}
#else
/* qingsong feedback 2 genphy_soft_reset will cause problem.
 * and this is the reduction version
 */
int yt8521_soft_reset(struct phy_device *phydev)
{
	int ret, val;

	val = ytphy_read_ext(phydev, 0xa001);
	ytphy_write_ext(phydev, 0xa001, (val & ~0x8000));

	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	return 0;
}
#endif

#endif

#if GMAC_CLOCK_INPUT_NEEDED
static int ytphy_mii_rd_ext(struct mii_bus *bus, int phy_id, u32 regnum)
{
	int ret;
	int val;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = bus->read(bus, phy_id, REG_DEBUG_DATA);

	return val;
}

static int ytphy_mii_wr_ext(struct mii_bus *bus, int phy_id, u32 regnum, u16 val)
{
	int ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_DATA, val);

	return ret;
}

int yt8511_config_dis_txdelay(struct mii_bus *bus, int phy_id)
{
    int ret;
    int val;

    /* disable auto sleep */
    val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
    if (val < 0)
            return val;

    val &= (~BIT(15));

    ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
    if (ret < 0)
            return ret;

    /* enable RXC clock when no wire plug */
    val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
    if (val < 0)
            return val;

    /* ext reg 0xc b[7:4]
	Tx Delay time = 150ps * N - 250ps
    */
    val &= ~(0xf << 4);
    ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
    printk("yt8511_config_dis_txdelay..phy txdelay, val=%#08x\n",val);

    return ret;
}


int yt8521_config_dis_txdelay(struct mii_bus *bus, int phy_id)
{
    int ret;
    int val;

    /* disable auto sleep */
    val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
    if (val < 0)
            return val;

    val &= (~BIT(15));

    ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
    if (ret < 0)
            return ret;

    /* enable RXC clock when no wire plug */
    val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
    if (val < 0)
            return val;

    /* ext reg 0xc b[7:4]
	Tx Delay time = 150ps * N - 250ps
    */
    val &= ~(0xf << 4);
    ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
    printk("yt8511_config_dis_txdelay..phy txdelay, val=%#08x\n",val);

    return ret;
}

#if 0
int yt8511_config_out_125m(struct mii_bus *bus, int phy_id)
{
	int ret;
	int val;

	/* disable auto sleep */
	val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
	if (val < 0)
	        return val;

	val &= (~BIT(15));

	ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
	if (ret < 0)
	        return ret;

	/* enable RXC clock when no wire plug */
	val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
	if (val < 0)
	        return val;

	/* ext reg 0xc.b[2:1]
	00-----25M from pll;
	01---- 25M from xtl;(default)
	10-----62.5M from pll;
	11----125M from pll(here set to this value)
	*/
	val |= (3 << 1);
	ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
	printk("yt8511_config_out_125m, phy clk out, val=%#08x\n",val);

#if 0
	/* for customer, please enable it based on demand.
	 * configure to master
	 */	
	val = bus->read(bus, phy_id, 0x9/*master/slave config reg*/);
	val |= (0x3<<11); //to be manual config and force to be master
	ret = bus->write(bus, phy_id, 0x9, val); //take effect until phy soft reset
	if (ret < 0)
		return ret;

	printk("yt8511_config_out_125m, phy to be master, val=%#08x\n",val);
#endif

    return ret;
}
#endif

static int yt8511_config_init(struct phy_device *phydev)
{
	int ret;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	ret = ytphy_config_init(phydev);
#else
	ret = genphy_config_init(phydev);
#endif

	return ret;
}
#endif /*GMAC_CLOCK_INPUT_NEEDED*/

#if (YTPHY_ENABLE_WOL)
static int ytphy_switch_reg_space(struct phy_device *phydev, int space)
{
	int ret;

	if (space == YTPHY_REG_SPACE_UTP){
		ret = ytphy_write_ext(phydev, 0xa000, 0);
	}else{
		ret = ytphy_write_ext(phydev, 0xa000, 2);
	}
	
	return ret;
}

static int ytphy_wol_en_cfg(struct phy_device *phydev, ytphy_wol_cfg_t wol_cfg)
{
	int ret=0;
	int val=0;

	val = ytphy_read_ext(phydev, YTPHY_WOL_CFG_REG);
	if (val < 0)
		return val;

	if(wol_cfg.enable) {
		val |= YTPHY_WOL_CFG_EN;

		if(wol_cfg.type == YTPHY_WOL_TYPE_LEVEL) {
			val &= ~YTPHY_WOL_CFG_TYPE;
			val &= ~YTPHY_WOL_CFG_INTR_SEL;
		} else if(wol_cfg.type == YTPHY_WOL_TYPE_PULSE) {
			val |= YTPHY_WOL_CFG_TYPE;
			val |= YTPHY_WOL_CFG_INTR_SEL;

			if(wol_cfg.width == YTPHY_WOL_WIDTH_84MS) {
				val &= ~YTPHY_WOL_CFG_WIDTH1;
				val &= ~YTPHY_WOL_CFG_WIDTH2;
			} else if(wol_cfg.width == YTPHY_WOL_WIDTH_168MS) {
				val |= YTPHY_WOL_CFG_WIDTH1;
				val &= ~YTPHY_WOL_CFG_WIDTH2;
			} else if(wol_cfg.width == YTPHY_WOL_WIDTH_336MS) {
				val &= ~YTPHY_WOL_CFG_WIDTH1;
				val |= YTPHY_WOL_CFG_WIDTH2;
			} else if(wol_cfg.width == YTPHY_WOL_WIDTH_672MS) {
				val |= YTPHY_WOL_CFG_WIDTH1;
				val |= YTPHY_WOL_CFG_WIDTH2;
			}
		}
	} else {
		val &= ~YTPHY_WOL_CFG_EN;
		val &= ~YTPHY_WOL_CFG_INTR_SEL;
	}

	ret = ytphy_write_ext(phydev, YTPHY_WOL_CFG_REG, val);
	if (ret < 0)
		return ret;

	return 0;
}

static void ytphy_get_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	int val = 0;

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	val = ytphy_read_ext(phydev, YTPHY_WOL_CFG_REG);
	if (val < 0)
		return;

	if (val & YTPHY_WOL_CFG_EN)
		wol->wolopts |= WAKE_MAGIC;

	return;
}

static int ytphy_set_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	int ret, pre_page, val;
	ytphy_wol_cfg_t wol_cfg;
	struct net_device *p_attached_dev = phydev->attached_dev;

	memset(&wol_cfg,0,sizeof(ytphy_wol_cfg_t));
	pre_page = ytphy_read_ext(phydev, 0xa000);
	if (pre_page < 0)
		return pre_page;

	/* Switch to phy UTP page */
	ret = ytphy_switch_reg_space(phydev, YTPHY_REG_SPACE_UTP);
	if (ret < 0)
		return ret;

	if (wol->wolopts & WAKE_MAGIC) {
		
		/* Enable the WOL interrupt */
		val = phy_read(phydev, YTPHY_UTP_INTR_REG);
		val |= YTPHY_WOL_INTR;
		ret = phy_write(phydev, YTPHY_UTP_INTR_REG, val);
		if (ret < 0)
			return ret;

		/* Set the WOL config */
		wol_cfg.enable = 1; //enable
		wol_cfg.type= YTPHY_WOL_TYPE_PULSE;
		wol_cfg.width= YTPHY_WOL_WIDTH_672MS;
		ret = ytphy_wol_en_cfg(phydev, wol_cfg);
		if (ret < 0)
			return ret;

		/* Store the device address for the magic packet */
		ret = ytphy_write_ext(phydev, YTPHY_MAGIC_PACKET_MAC_ADDR2,
				((p_attached_dev->dev_addr[0] << 8) |
				 p_attached_dev->dev_addr[1]));
		if (ret < 0)
			return ret;
		ret = ytphy_write_ext(phydev, YTPHY_MAGIC_PACKET_MAC_ADDR1,
				((p_attached_dev->dev_addr[2] << 8) |
				 p_attached_dev->dev_addr[3]));
		if (ret < 0)
			return ret;
		ret = ytphy_write_ext(phydev, YTPHY_MAGIC_PACKET_MAC_ADDR0,
				((p_attached_dev->dev_addr[4] << 8) |
				 p_attached_dev->dev_addr[5]));
		if (ret < 0)
			return ret;
	} else {
		wol_cfg.enable = 0; //disable
		wol_cfg.type= YTPHY_WOL_TYPE_MAX;
		wol_cfg.width= YTPHY_WOL_WIDTH_MAX;
		ret = ytphy_wol_en_cfg(phydev, wol_cfg);
		if (ret < 0)
			return ret;
	}

	/* Recover to previous register space page */
	ret = ytphy_switch_reg_space(phydev, pre_page);
	if (ret < 0)
		return ret;

	return 0;
}

#endif /*(YTPHY_ENABLE_WOL)*/

/*
static void yt8531s_dump_clkout(struct phy_device *phydev)
{
	int v;

	v = ytphy_read_ext(phydev, 0xA012);
	if (v < 0) {
		phydev_warn(phydev, "A012 read failed\n");
		return;
	}

	phydev_info(phydev,
		"A012 CLK_OUT = 0x%04x\n"
		"  CLK_OUT_EN      : %s\n"
		"  LinkDown CLK    : %s\n"
		"  CLK Frequency   : %s\n"
		"  CLK Source      : %s\n",
		v,
		(v & BIT(6)) ? "enable" : "disable",
		(v & BIT(5)) ? "keep on linkdown" : "disable on linkdown",
		(v & BIT(4)) ? "125MHz" : "25MHz",
		({ const char *s;
		   switch ((v >> 1) & 0x7) {
		   case 0: s = "PLL 125M"; break;
		   case 1: s = "UTP RX recovered"; break;
		   case 2: s = "SDS RX recovered"; break;
		   case 4: s = "25M reference"; break;
		   case 5: s = "25M SSC"; break;
		   default: s = "reserved"; break;
		   }
		   s;
		})
	);
}
*/

static int yt8531s_hw_init(struct phy_device *phydev)
{
	int ret, val;

	// phydev_info(phydev, "YT8531S: 开始执行针对 3.3V RGMII 优化的初始化流程\n");

	/* --- 步骤 1: 优化 SerDes/CDR 性能 (切换至 Page 1) --- */
	ret = ytphy_write_ext(phydev, 0xa000, 0x0001);
	if (ret < 0) return ret;

	ytphy_write_ext(phydev, 0x03, 0x1434); // 优化 CDR 锁定，解决数据偏移/Martian source
	ytphy_write_ext(phydev, 0x0e, 0x0163); // 增强信号均衡

	/* --- 步骤 2: 优化 UTP 性能与配置 (切换至 Page 0) --- */
	ret = ytphy_write_ext(phydev, 0xa000, 0x0000);
	if (ret < 0) return ret;

	/* 2.1 厂家 UTP 性能优化项 */
	ytphy_write_ext(phydev, 0xa071, 0x9007); // 10M 模板优化
	ytphy_write_ext(phydev, 0x0052, 0x231d); // UTP 长线性能
	ytphy_write_ext(phydev, 0x0051, 0x04a9); // 电口指标优化
	ytphy_write_ext(phydev, 0x0057, 0x274c); // 电口指标优化
	ytphy_write_ext(phydev, 0xa006, 0x010d); // Combo 模式优化
	ytphy_write_ext(phydev, 0xa023, 0x4031); // Combo 模式优化

	/* 2.2 关键：RGMII 驱动强度与延时校准 */
	/* 针对 3.3V IO 电压，降低驱动强度以减少信号过冲 */
	ytphy_write_ext(phydev, 0xa001, 0x0002);

	/* 设置 RGMII 延时 (寄存器 0xA003)
	 * 设置 TX/RX 内部延时约 2ns，消除数据对齐引起的 Martian source
	 * 0x1010 是平衡值，如果仍有报错可尝试 0xF0F0
	 */
	ytphy_write_ext(phydev, 0xa003, 0x1010);

	/* --- 步骤 3: 强制时钟输出使能 (寄存器 0xA012) --- */
	val = ytphy_read_ext(phydev, 0xA012);
	if (val < 0) return val;

	/* 针对 YT8531S 寄存器定义：
	 */
	// 1. 开启使能并设为 125M
	val |= (BIT(6) | BIT(4));
	// 2. 切换时钟源为 PLL 125M (根据手册：Bit 3:1 设为 000)
	val &= ~(BIT(3) | BIT(2) | BIT(1));
	ytphy_write_ext(phydev, 0xA012, val);

	/* --- 步骤 4: 执行软复位以应用配置 --- */
	/* 使用厂家建议的软复位值 0x9140 (Restart AN + Soft Reset) */
	ret = phy_write(phydev, MII_BMCR, 0x9140);

	/* 给予硬件充足时间稳定时钟信号 */
	msleep(100);

	// yt8531s_dump_clkout(phydev);

	// phydev_info(phydev, "YT8531S: 初始化完成，CLK_OUT 已开启 (125M)\n");

	return 0;
}

static int yt8521_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	phydev->irq = PHY_POLL;

	ytphy_write_ext(phydev, 0xa000, 0);
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	ret = ytphy_config_init(phydev);
#else
	ret = genphy_config_init(phydev);
#endif
	if (ret < 0)
		return ret;

	/* LED 初始化 */
	ret = yt8521_led_init(phydev);
	if (ret)
		phydev_warn(phydev, "LED init failed\n");

	/* 识别是否为 YT8531S 芯片 */
	if ((phydev->phy_id & phydev->drv->phy_id_mask) == PHY_ID_YT8531S) {
		ret = yt8531s_hw_init(phydev);
		if (ret)
			phydev_warn(phydev, "YT8531S hw init failed\n");
	} else {
		/* YT8521 逻辑保持不变 */
		ret = yt8521_delaysel_init(phydev);
		if (ret)
			phydev_warn(phydev, "YT8521 delay select init failed\n");
	}

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8521_EN_SLEEP_SW_BIT));

	ret = ytphy_write_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	/* enable RXC clock when no wire plug */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, 0xc);
	if (val < 0)
		return val;
	val &= ~(1 << 12);
	ret = ytphy_write_ext(phydev, 0xc, val);
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * for fiber mode, there is no 10M speed mode and
 * this function is for this purpose.
 */
static int yt8521_adjust_status(struct phy_device *phydev, int val, int is_utp)
{
	int speed_mode, duplex;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	int speed = -1;
#else
	int speed = SPEED_UNKNOWN;
#endif

	duplex = (val & YT8512_DUPLEX) >> YT8521_DUPLEX_BIT;
	speed_mode = (val & YT8521_SPEED_MODE) >> YT8521_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		if (is_utp)
			speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
		speed = SPEED_1000;
		break;
	case 3:
		break;
	default:
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		speed = -1;
#else
		speed = SPEED_UNKNOWN;
#endif
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;
	//printk (KERN_INFO "yt8521_adjust_status call out,regval=0x%04x,mode=%s,speed=%dm...\n", val,is_utp?"utp":"fiber", phydev->speed);

	return 0;
}

/*
 * for fiber mode, when speed is 100M, there is no definition for autonegotiation, and
 * this function handles this case and return 1 per linux kernel's polling.
 */
int yt8521_aneg_done (struct phy_device *phydev)
{

	//printk("yt8521_aneg_done callin,speed=%dm,linkmoded=%d\n", phydev->speed,link_mode_8521);

	if((32 == link_mode_8521) && (SPEED_100 == phydev->speed))
	{
		return 1/*link_mode_8521*/;
	}

	return genphy_aneg_done(phydev);
}

static int yt8521_read_status(struct phy_device *phydev)
{
	int ret;
	volatile int val, yt8521_fiber_latch_val, yt8521_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	/* reading UTP */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		link_mode_8521 = 1;
		yt8521_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)
	/* reading Fiber */
	ret = ytphy_write_ext(phydev, 0xa000, 2);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	//note: below debug information is used to check multiple PHy ports.
	//printk (KERN_INFO "yt8521_read_status, fiber status=%04x,macbase=0x%08lx\n", val,(unsigned long)phydev->attached_dev);

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case
	 * this is important for Linux kernel for that, missing linkdown event will cause problem.
	 */
	yt8521_fiber_latch_val = phy_read(phydev, MII_BMSR);
	yt8521_fiber_curr_val = phy_read(phydev, MII_BMSR);
	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if((link) && (yt8521_fiber_latch_val != yt8521_fiber_curr_val))
	{
		link = 0;
		printk (KERN_INFO "yt8521_read_status, fiber link down detect,latch=%04x,curr=%04x\n", yt8521_fiber_latch_val,yt8521_fiber_curr_val);
	}

	if (link) {
		link_fiber = 1;
		yt8521_adjust_status(phydev, val, 0);
		link_mode_8521 = 32; //fiber mode


	} else {
		link_fiber = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
		link_mode_8521 = 0;
	}

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	if (link_utp) {
		ytphy_write_ext(phydev, 0xa000, 0);
	}
#endif

	//printk (KERN_INFO "yzhang..8521 read status call out,link=%d,linkmode=%d\n", phydev->link, link_mode_8521 );
	return 0;
}

int yt8521_suspend(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}

int yt8521_resume(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;
	int ret;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	/* disable auto sleep */
	value = ytphy_read_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1);
	if (value < 0)
		return value;

	value &= (~BIT(YT8521_EN_SLEEP_SW_BIT));
	ret = ytphy_write_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1, value);
	if (ret < 0)
		return ret;

	/* enable RXC clock when no wire plug */
	value = ytphy_read_ext(phydev, 0xc);
	if (value < 0)
		return value;
	value &= ~(1 << 12);
	ret = ytphy_write_ext(phydev, 0xc, value);
	if (ret < 0)
		return ret;

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	ytphy_write_ext(phydev, 0xa000, 0);
#endif

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}


#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
int yt8618_soft_reset(struct phy_device *phydev)
{
	int ret;

	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	return 0;
}

int yt8614_soft_reset(struct phy_device *phydev)
{
	int ret;

	/* utp */
	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	/* qsgmii */
	ytphy_write_ext(phydev, 0xa000, 2);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		ytphy_write_ext(phydev, 0xa000, 0); //back to utp mode
		return ret;
	}

	/* sgmii */
	ytphy_write_ext(phydev, 0xa000, 3);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		ytphy_write_ext(phydev, 0xa000, 0); //back to utp mode
		return ret;
	}

	return 0;
}

#endif

static int yt8618_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	phydev->irq = PHY_POLL;

	if(0xff == yt_mport_base_phy_addr)
		/* by default, we think the first phy should be the base phy addr. for mul */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	{
		yt_mport_base_phy_addr = phydev->addr;
	}else if (yt_mport_base_phy_addr > phydev->addr) {
		printk (KERN_INFO "yzhang..8618 init, phy address mismatch, base=%d, cur=%d\n", yt_mport_base_phy_addr, phydev->addr);
	}
#else
	{
		yt_mport_base_phy_addr = phydev->mdio.addr;
	}else if (yt_mport_base_phy_addr > phydev->mdio.addr) {
		printk (KERN_INFO "yzhang..8618 init, phy address mismatch, base=%d, cur=%d\n", yt_mport_base_phy_addr, phydev->mdio.addr);
	}
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	ret = ytphy_config_init(phydev);
#else
	ret = genphy_config_init(phydev);
#endif
	if (ret < 0)
		return ret;

	/* for utp to optimize signal */
	ret = ytphy_write_ext(phydev, 0x41, 0x33);
	if (ret < 0)
		return ret;
	ret = ytphy_write_ext(phydev, 0x42, 0x66);
	if (ret < 0)
		return ret;
	ret = ytphy_write_ext(phydev, 0x43, 0xaa);
	if (ret < 0)
		return ret;
	ret = ytphy_write_ext(phydev, 0x44, 0xd0d);
	if (ret < 0)
		return ret;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	if((phydev->addr > yt_mport_base_phy_addr) && ((2 == phydev->addr - yt_mport_base_phy_addr) || (5 == phydev->addr - yt_mport_base_phy_addr)))
#else
	if((phydev->mdio.addr > yt_mport_base_phy_addr) && ((2 == phydev->mdio.addr - yt_mport_base_phy_addr) || (5 == phydev->mdio.addr - yt_mport_base_phy_addr)))
#endif
	{
		ret = ytphy_write_ext(phydev, 0x44, 0x2929);
		if (ret < 0)
			return ret;
	}

	val = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, val | BMCR_RESET);

	printk (KERN_INFO "yt8618_config_init call out.\n");
	return ret;
}

static int yt8614_config_init(struct phy_device *phydev)
{
	int ret = 0;

	phydev->irq = PHY_POLL;

	if(0xff == yt_mport_base_phy_addr_8614)
		/* by default, we think the first phy should be the base phy addr. for mul */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	{
		yt_mport_base_phy_addr_8614 = (unsigned int)phydev->addr;
	}else if (yt_mport_base_phy_addr_8614 > (unsigned int)phydev->addr) {
		printk (KERN_INFO "yzhang..8618 init, phy address mismatch, base=%u, cur=%d\n", yt_mport_base_phy_addr_8614, phydev->addr);
	}
#else
	{
		yt_mport_base_phy_addr_8614 = (unsigned int)phydev->mdio.addr;
	}else if (yt_mport_base_phy_addr_8614 > (unsigned int)phydev->mdio.addr) {
		printk (KERN_INFO "yzhang..8618 init, phy address mismatch, base=%u, cur=%d\n", yt_mport_base_phy_addr_8614, phydev->mdio.addr);
	}
#endif
	return ret;
}

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#define yt8614_get_port_from_phydev(phydev) ((0xff == yt_mport_base_phy_addr_8614) && (yt_mport_base_phy_addr_8614 <= (phydev)->addr) ? 0 : (unsigned int)((phydev)->addr) - yt_mport_base_phy_addr_8614)
#else
#define yt8614_get_port_from_phydev(phydev) ((0xff == yt_mport_base_phy_addr_8614) && (yt_mport_base_phy_addr_8614 <= (phydev)->mdio.addr) ? 0 : (unsigned int)((phydev)->mdio.addr) - yt_mport_base_phy_addr_8614)
#endif

int yt8618_aneg_done (struct phy_device *phydev)
{

	return genphy_aneg_done(phydev);
}

int yt8614_aneg_done (struct phy_device *phydev)
{
	int port = yt8614_get_port_from_phydev(phydev);

	/*it should be used for 8614 fiber*/
	if((32 == link_mode_8614[port]) && (SPEED_100 == phydev->speed))
	{
		return 1;
	}

	return genphy_aneg_done(phydev);
}

static int yt8614_read_status(struct phy_device *phydev)
{
        //int i;
	int ret;
	volatile int val, yt8614_fiber_latch_val, yt8614_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;
	int port = yt8614_get_port_from_phydev(phydev);

#if (YT8614_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	/* switch to utp and reading regs  */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		// here is same as 8521 and re-use the function;
		yt8521_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}
#endif //(YT8614_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)

#if (YT8614_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)
	/* reading Fiber/sgmii */
	ret = ytphy_write_ext(phydev, 0xa000, 3);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	//printk (KERN_INFO "yzhang..8614 read fiber status=%04x,macbase=0x%08lx\n", val,(unsigned long)phydev->attached_dev);

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case */
	yt8614_fiber_latch_val = phy_read(phydev, MII_BMSR);
	yt8614_fiber_curr_val = phy_read(phydev, MII_BMSR);
	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if((link) && (yt8614_fiber_latch_val != yt8614_fiber_curr_val))
	{
		link = 0;
		printk (KERN_INFO "yt8614_read_status, fiber link down detect,latch=%04x,curr=%04x\n", yt8614_fiber_latch_val,yt8614_fiber_curr_val);
	}

	if (link) {
		link_fiber = 1;
		yt8521_adjust_status(phydev, val, 0);
		link_mode_8614[port] = 32; //fiber mode


	} else {
		link_fiber = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
		link_mode_8614[port] = 0;
	}

#if (YT8614_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	if (link_utp) {
		ytphy_write_ext(phydev, 0xa000, 0);
	}
#endif
	//printk (KERN_INFO "yt8614_read_status call out,link=%d,linkmode=%d\n", phydev->link, link_mode_8614[port] );

	return 0;
}

static int yt8618_read_status(struct phy_device *phydev)
{
	int ret;
	volatile int val; //maybe for 8614 yt8521_fiber_latch_val, yt8521_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;

	/* switch to utp and reading regs  */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		yt8521_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
	}

	return 0;
}

int yt8618_suspend(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}

int yt8618_resume(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}

int yt8614_suspend(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 3);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}

int yt8614_resume(struct phy_device *phydev)
{
#if !(SYS_WAKEUP_BASED_ON_ETH_PKT)
	int value;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_lock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 3);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif
#endif /*!(SYS_WAKEUP_BASED_ON_ETH_PKT)*/

	return 0;
}


static struct phy_driver ytphy_drvs[] = {
	{
		.phy_id         = PHY_ID_YT8010,
		.name           = "YT8010 Automotive Ethernet",
		.phy_id_mask    = MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features       = PHY_BASIC_FEATURES,
		.flags          = PHY_HAS_INTERRUPT,
#endif
		.config_aneg    = yt8010_config_aneg,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		.config_init	= ytphy_config_init,
#else
		.config_init	= genphy_config_init,
#endif
		.read_status    = genphy_read_status,
	}, {
		.phy_id		= PHY_ID_YT8510,
		.name		= "YT8510 100/10Mb Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features	= PHY_BASIC_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
#endif
		.config_aneg	= genphy_config_aneg,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		.config_init	= ytphy_config_init,
#else
		.config_init	= genphy_config_init,
#endif
		.read_status	= genphy_read_status,
	}, {
		.phy_id		= PHY_ID_YT8511,
		.name		= "YT8511 Gigabit Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features	= PHY_GBIT_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
#endif
		.config_aneg	= genphy_config_aneg,
#if GMAC_CLOCK_INPUT_NEEDED
		.config_init	= yt8511_config_init,
#else
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		.config_init	= ytphy_config_init,
#else
		.config_init	= genphy_config_init,
#endif
#endif
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= PHY_ID_YT8512,
		.name		= "YT8512 Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features	= PHY_BASIC_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
#endif
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= PHY_ID_YT8512B,
		.name		= "YT8512B Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features	= PHY_BASIC_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
#endif
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
        .phy_id         = PHY_ID_YT8521,
        .name           = "YT8521 Ethernet",
        .phy_id_mask    = MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
        .features       = PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
#endif
        .flags          = PHY_POLL,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
		.soft_reset	= yt8521_soft_reset,
#endif
        .config_aneg    = genphy_config_aneg,
        .aneg_done	= yt8521_aneg_done,
        .config_init    = yt8521_config_init,
        .read_status    = yt8521_read_status,
        .suspend        = yt8521_suspend,
        .resume         = yt8521_resume,
#if (YTPHY_ENABLE_WOL)
		.get_wol		= &ytphy_get_wol,
		.set_wol		= &ytphy_set_wol,
#endif
        },{
		/* same as 8521 */
        .phy_id         = PHY_ID_YT8531S,
        .name           = "YT8531S Ethernet",
        .phy_id_mask    = MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
        .features       = PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
#endif
        .flags          = PHY_POLL,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
		.soft_reset	= yt8521_soft_reset,
#endif
        .config_aneg    = genphy_config_aneg,
        .aneg_done	= yt8521_aneg_done,
        .config_init    = yt8521_config_init,
        .read_status    = yt8521_read_status,
        .suspend        = yt8521_suspend,
        .resume         = yt8521_resume,
#if (YTPHY_ENABLE_WOL)
		.get_wol		= &ytphy_get_wol,
		.set_wol		= &ytphy_set_wol,
#endif
        }, {
        /* same as 8511 */
		.phy_id		= PHY_ID_YT8531,
		.name		= "YT8531 Gigabit Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features	= PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
#endif
		.config_aneg	= genphy_config_aneg,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
		.config_init	= ytphy_config_init,
#else
		.config_init	= genphy_config_init,
#endif
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
#if (YTPHY_ENABLE_WOL)
		.get_wol		= &ytphy_get_wol,
		.set_wol		= &ytphy_set_wol,
#endif
	}, {
        .phy_id         = PHY_ID_YT8618,
        .name           = "YT8618 Ethernet",
        .phy_id_mask    = MOTORCOMM_MPHY_ID_MASK,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
        .features       = PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
#endif
        .flags          = PHY_POLL,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
		.soft_reset	= yt8618_soft_reset,
#endif
        .config_aneg    = genphy_config_aneg,
        .aneg_done		= yt8618_aneg_done,
        .config_init    = yt8618_config_init,
        .read_status    = yt8618_read_status,
        .suspend        = yt8618_suspend,
        .resume         = yt8618_resume,
    }, {
		.phy_id 		= PHY_ID_YT8614,
		.name			= "YT8614 Ethernet",
		.phy_id_mask	= MOTORCOMM_MPHY_ID_MASK_8614,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0) )
		.features		= PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
#endif
		.flags			= PHY_POLL,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
#else
		.soft_reset = yt8614_soft_reset,
#endif
		.config_aneg	= genphy_config_aneg,
		.aneg_done		= yt8614_aneg_done,
		.config_init	= yt8614_config_init,
		.read_status	= yt8614_read_status,
		.suspend		= yt8614_suspend,
		.resume 		= yt8614_resume,
		},
};

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) )
static int ytphy_drivers_register(struct phy_driver* phy_drvs, int size)
{
	int i, j;
	int ret;

	for (i = 0; i < size; i++) {
		ret = phy_driver_register(&phy_drvs[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	for (j = 0; j < i; j++)
		phy_driver_unregister(&phy_drvs[j]);

	return ret;
}

static void ytphy_drivers_unregister(struct phy_driver* phy_drvs, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		phy_driver_unregister(&phy_drvs[i]);
	}
}

static int __init ytphy_init(void)
{
	printk("motorcomm phy register\n");
	return ytphy_drivers_register(ytphy_drvs, ARRAY_SIZE(ytphy_drvs));
}

static void __exit ytphy_exit(void)
{
	printk("motorcomm phy unregister\n");
	ytphy_drivers_unregister(ytphy_drvs, ARRAY_SIZE(ytphy_drvs));
}

module_init(ytphy_init);
module_exit(ytphy_exit);
#else
/* for linux 4.x */
module_phy_driver(ytphy_drvs);
#endif

MODULE_DESCRIPTION("Motorcomm PHY driver");
MODULE_AUTHOR("Leilei Zhao");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused motorcomm_tbl[] = {
	{ PHY_ID_YT8010, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8510, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8511, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512B, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8521, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8531S, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8531, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8618, MOTORCOMM_MPHY_ID_MASK },
	{ PHY_ID_YT8614, MOTORCOMM_MPHY_ID_MASK_8614 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, motorcomm_tbl);