/*
 * linux/mfd/bu26154.h
 *
 * Functions to access ROHM BU26154 chip.
 *
 * Copyright (C) 2016 SATO Corporation
 *
 *
 *  For licencing details see kernel-base/COPYING
 */

#ifndef __LINUX_MFD_BU26154_H
#define __LINUX_MFD_BU26154_H

#include <linux/regmap.h>
#include <linux/mutex.h>


#define BU26154_MAX_REGISTER			0xf5

/*
 * List of registers specific to BU26154.
 * This depends on the MAPCON value which change
 * the register map.
 *
 * NOTE:
 *   The register stride is 2, but start with 0
 *   for Read Register, 1 for Write Register.
 */

/* MAPCON = <ANY> */
#define BU26154_MAPCON_REG			0x1d

/* MAPCON = 0x0 */
#define BU26154_SRATE_REG			0x00
#define BU26154_CLKEN_REG			0x0c
#define BU26154_CLKCR_REG			0x0e
#define BU26154_RESET_REG			0x10
#define BU26154_RECPLAY_REG			0x12
#define BU26154_VMID_REG			0x20
#define BU26154_DAC_POW_REG			0x24
#define BU26154_SPAMP_POW_REG			0x26
#define BU26154_THERMAL_REG			0x2c
#define BU26154_AVOL_CR_REG			0x3a
#define BU26154_PDIG_VOL_REG			0x3e
#define BU26154_DACCLK_REG			0x58
#define BU26154_EFC_MODE_REG			0x5c
#define BU26154_SAI_TX_REG			0x60
#define BU26154_SAI_RX_REG			0x62
#define BU26154_SAI_MODE_REG			0x64
#define BU26154_DSPEN_REG			0x66
#define BU26154_DVCR_REG			0x68
#define BU26154_MUXVOL_REG			0x6a
#define BU26154_EFCVOL_REG			0x70
#define BU26154_ZDSR_REG			0xdc

/* MAPCON = 0x1 */
#define BU26154_FPLL_M_REG			0x02
#define BU26154_FPLL_NL_REG			0x04
#define BU26154_FPLL_NH_REG			0x06
#define BU26154_FPLL_D_REG			0x08
#define BU26154_FPLL_FL_REG			0x0a
#define BU26154_FPLL_FH_REG			0x0c
#define BU26154_FPLL_FDL_REG			0x0e
#define BU26154_FPLL_FDH_REG			0x10
#define BU26154_FPLL_V_REG			0x12
#define BU26154_TS_ADCCR_REG			0x60
#define BU26154_TS_TOUTCHAD1			0x62
#define BU26154_TS_TOUTCHAD2			0x64
#define BU26154_SPAMP_ICR_REG			0x84
#define BU26154_LPFCR_REG			0xa0

/* MAPCON = 0x2 */
#define BU26154_AUDCR2_REG			0x04
#define BU26154_AUDCR1_REG			0x12

/*
 * List of register bitfields for component BU26154
 */

/* Clock Input/Output Control Register bitfields */
#define BU26154_CLKCR_PLLISEL			(BIT(4)|BIT(3))
#define BU26154_CLKCR_CLKSEL			(BIT(2)|BIT(1)|BIT(0))

/* Clock Enable Register bitfields */
#define BU26154_CLKEN_TCLKEN			BIT(7)
#define BU26154_CLKEN_PLLOE			BIT(2)
#define BU26154_CLKEN_PLLEN			BIT(1)
#define BU26154_CLKEN_MCLKEN			BIT(0)

/* Touch ADC Control Register bitfields */
#define BU26154_TADC_CR_TCHEN			BIT(7)
#define BU26154_TADC_CR_TCHA2			BIT(6)
#define BU26154_TADC_CR_TCHA1			BIT(5)
#define BU26154_TADC_CR_TCHA0			BIT(4)
#define BU26154_TADC_CR_TCHRSEL			BIT(2)
#define BU26154_TADC_CR_TCHMODE			BIT(1)

/* DAC Clock Setting Register bitfields */
#define BU26154_DACCLK_OSRSEL			(BIT(5)|BIT(4))

/* SAI Transmitter Control Register bitfields */
#define BU26154_SAI_TX_PCMFO24			(BIT(7)|BIT(6))
#define BU26154_SAI_TX_FMTO			BIT(5)
#define BU26154_SAI_TX_MSBO			BIT(4)
#define BU26154_SAI_TX_ISSCKO			BIT(3)
#define BU26154_SAI_TX_AFOO			BIT(2)
#define BU26154_SAI_TX_DLYO			BIT(1)
#define BU26154_SAI_TX_WSLO			BIT(0)

/* SAI Receiver Control Register bitfields */
#define BU26154_SAI_RX_PCMFI24			(BIT(7)|BIT(6))
#define BU26154_SAI_RX_FMTI			BIT(5)
#define BU26154_SAI_RX_MSBI			BIT(4)
#define BU26154_SAI_RX_ISSCKI			BIT(3)
#define BU26154_SAI_RX_AFOI			BIT(2)
#define BU26154_SAI_RX_DLYI			BIT(1)
#define BU26154_SAI_RX_WSLI			BIT(0)

/* SAI Mode select Register bitfields */
#define BU26154_SAI_MODE_BSWP			BIT(4)
#define BU26154_SAI_MODE_MST			BIT(0)

/* Digital Volume Control Function Enable Register bitfields */
#define BU26154_DVCR_DVMUTE			BIT(4)
#define BU26154_DVCR_DVFADE			BIT(3)
#define BU26154_DVCR_RALCEN			BIT(1)
#define BU26154_DVCR_PALCEN			BIT(0)

/* Mixer & Volume Control Register bitfields */
#define BU26154_MUXVOL_DVFCON			(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define BU26154_MUXVOL_RMCON			(BIT(3)|BIT(2))
#define BU26154_MUXVOL_LMCON			(BIT(1)|BIT(0))

/* Zero Detection Setting Register bitfields */
#define BU26154_ZDSR_ZDTIME			(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define BU26154_ZDSR_ZDEN			BIT(0)

/* Analog Reference Power Management Register bitfields */
#define BU26154_VMID_HPREN			BIT(7)
#define BU26154_VMID_HPLEN			BIT(6)
#define BU26154_VMID_HPVDDEN			BIT(3)
#define BU26154_VMID_MICBEN			BIT(2)
#define BU26154_VMID_VMIDCON			(BIT(1)|BIT(0))

/* DAC Power Management Register */
#define BU26154_DAC_POW_DACREN			BIT(2)
#define BU26154_DAC_POW_DACLEN			BIT(1)

/* Speaker Amplifier Power Management Register bitfields */
#define BU26154_SPAMP_POW_SPMDSEL		BIT(7)
#define BU26154_SPAMP_POW_AVREN			BIT(4)
#define BU26154_SPAMP_POW_COEFSEL		BIT(3)
#define BU26154_SPAMP_POW_SPEN			BIT(1)
#define BU26154_SPAMP_POW_AVLEN			BIT(0)

/* Thermal Shutdown Control Register bitfields */
#define BU26154_THERMAL_TSDEN			BIT(0)

/* Speaker Amplifier Input Control Register bitfields */
#define BU26154_SPAMP_ICR_SPIN2EN		BIT(3)
#define BU26154_SPAMP_ICR_SPIN1EN		BIT(2)
#define BU26154_SPAMP_ICR_SPVOL			(BIT(1)|BIT(0))

/* Play Programable LPF Setting Register bitfields */
#define BU26154_LPFCR_PLPFOD			BIT(1)
#define BU26154_LPFCR_PLPFEN			BIT(0)

/* Audio Analog Control2 Register bitfields */
#define BU26154_AUDCR2_HPLSEN			BIT(2)

/* Audio Analog Control1 Register bitfields */
#define BU26154_AUDCR1_AREF1EN			BIT(0)

/*
 * List of register values for component BU26154
 */

/* MAPCON Register values */
#define BU26154_MAPCON_MAP0			0x0
#define BU26154_MAPCON_MAP1			0x1
#define BU26154_MAPCON_MAP2			0x2


/**
 * struct bu26154_dev - bu26154 sub-driver chip access routines
 *
 * Device data may be used to access the BU26154 chip
 */
struct bu26154 {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct rt_mutex map_lock;
};

static __always_inline
int bu26154_mask_and_set(struct bu26154 *bu26154, u8 reg, u8 mask, u8 val)
{
	int ret;
	unsigned int tmp, orig;
	u8 r_reg = !(reg%2) ? reg : (reg-0x1);
	u8 w_reg = (reg%2) ? reg : (reg+0x1);

	ret = regmap_read(bu26154->regmap, r_reg, &orig);
	if (ret != 0)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = regmap_write(bu26154->regmap, w_reg, tmp);

	return ret;
}

static __always_inline
void bu26154_map_lock(struct bu26154 *bu26154)
{
	rt_mutex_lock(&bu26154->map_lock);
}

static __always_inline
int bu26154_set_map(struct bu26154 *bu26154, u8 val)
{
	return regmap_write(bu26154->regmap, BU26154_MAPCON_REG, val);
}

static __always_inline
void bu26154_map_unlock(struct bu26154 *bu26154)
{
	rt_mutex_unlock(&bu26154->map_lock);
}

static inline int bu26154_set_map_lock(struct bu26154 *bu26154, u8 val)
{
	int ret;
	bu26154_map_lock(bu26154);
	ret = bu26154_set_map(bu26154, val);
	if (ret)
		bu26154_map_unlock(bu26154);
	return ret;
}

static inline void bu26154_set_map_unlock(struct bu26154 *bu26154, u8 val)
{
	bu26154_set_map(bu26154, val);
	bu26154_map_unlock(bu26154);
}

static inline int bu26154_reg_read(struct bu26154 *bu26154, u8 reg,
		unsigned int *val)
{
	return regmap_read(bu26154->regmap, !(reg%2) ? reg : (reg-0x1), val);
}

static inline int bu26154_reg_write(struct bu26154 *bu26154, u8 reg,
		unsigned int val)
{
	return regmap_write(bu26154->regmap, (reg%2) ? reg : (reg+0x1), val);
}

static inline int bu26154_block_read(struct bu26154 *bu26154, u8 reg,
		int count, u8 *buf)
{
	return regmap_raw_read(bu26154->regmap, !(reg%2) ? reg : (reg-0x1),
			       buf, count);
}

static inline int bu26154_block_write(struct bu26154 *bu26154, u8 reg,
		int count, u8 *data)
{
	return regmap_raw_write(bu26154->regmap, (reg%2) ? reg : (reg+0x1),
				data, count);
}

static inline int bu26154_reg_set_bits(struct bu26154 *bu26154, u8 reg,
		u8 mask)
{
	return bu26154_mask_and_set(bu26154, reg, mask, mask);
}

static inline int bu26154_reg_clear_bits(struct bu26154 *bu26154, u8 reg,
		u8 mask)
{
	return bu26154_mask_and_set(bu26154, reg, mask, 0);
}

static inline int bu26154_reg_update_bits(struct bu26154 *bu26154, u8 reg,
		u8 mask, u8 val)
{
	return bu26154_mask_and_set(bu26154, reg, mask, val);
}

#endif /*  __LINUX_MFD_BU26154_H */
