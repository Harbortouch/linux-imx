/*
 * Copyright (C) 2016 SATO Corporation
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/bu26154.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/bu26154.h>

#include "bu26154.h"

/* codec private data */
struct bu26154_audio_priv {
	struct snd_soc_codec *codec;
	struct bu26154_audio_data *pdata;

	int sysclk;
	int sysclk_rate;

	int bclk;  /* Desired BCLK */
	int lrclk;

	struct completion fll_lock;
	int fll_src;
	int fll_fref;
	int fll_fout;
};

static struct clk_coeff {
	u32 mclk;
	u32 rate;
	u8 pll[9];
} coeff_div[] = {
	/*
	 * PLLOutputFreq(Hz) = InputMclkFreq(Hz) / FPLLM *
	 *   (FPLLN + FPLLD/16 + FPLLF/FPLLF_D/16) * 2/FPLLV
	 */
	{
		.mclk = 48000000,
		.rate = 16000,
		.pll = {0x3, 0x3, 0x0, 0x1, 0x13, 0x0, 0x7d, 0x0, 0x6},
	},
	{
		.mclk = 48000000,
		.rate = 32000,
		.pll = {0x3, 0x3, 0x0, 0x1, 0x13, 0x0, 0x7d, 0x0, 0x3},
	},
	{
		.mclk = 48000000,
		/* FIXME
		 *   Generate 44.28KHz for now, because we can't
		 *   generate 44.1KHz from 48MHz
		 */
		.rate = 44100,
		.pll = {0x3, 0x4, 0x0, 0x4, 0x2c, 0x0, 0x35, 0xc, 0x3},
	},
	{
		.mclk = 48000000,
		.rate = 48000,
		.pll = {0x3, 0x3, 0x0, 0x1, 0x13, 0x0, 0x7d, 0x0, 0x2},
	},
};

static inline struct clk_coeff *get_coeff(int mclk, int rate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return &coeff_div[i];
	}
	return NULL;
}

#ifdef CONFIG_OF
static const struct bu26154_audio_data *
bu26154_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct bu26154_audio_data *pdata;

	if (!np) {
		dev_err(dev, "no device tree or platform data\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	//TODO

	return pdata;
}
#else
static const struct bu26154_audio_data *
bu26154_parse_dt(struct device *dev)
{
	dev_err(dev, "no platform data available\n");
	return ERR_PTR(-EINVAL);
}
#endif

/* locks should be held by the caller */
static inline void __start_fpll(struct bu26154 *bu26154)
{
	/* Ensure MAPCON register points to MAP0 */
	bu26154_set_map(bu26154, BU26154_MAPCON_MAP0);
	bu26154_reg_set_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_PLLEN);
	mdelay(10);
	bu26154_reg_set_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_PLLOE);
}

/* locks should be held by the caller */
static inline void __stop_fpll(struct bu26154 *bu26154)
{
	/* Ensure MAPCON register points to MAP0 */
	bu26154_set_map(bu26154, BU26154_MAPCON_MAP0);
	bu26154_reg_clear_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_PLLOE);
	bu26154_reg_clear_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_PLLEN);
}

/* locks should be held by the caller */
static inline int __vmid_power(struct bu26154 *bu26154, bool on)
{
	int ret;

	/* Ensure MAPCON register points to MAP0 */
	bu26154_set_map(bu26154, BU26154_MAPCON_MAP0);

	/* Power off VMID */
	bu26154_reg_clear_bits(bu26154, BU26154_VMID_REG,
			       BU26154_VMID_VMIDCON);
	if (!on)
		return 0;

	/* Power on VMID using high-speed-mode */
	if (ret = bu26154_reg_update_bits(bu26154, BU26154_VMID_REG,
					  BU26154_VMID_VMIDCON, 0x1)) {
		return ret;
	}
	mdelay(5);

	/* Set VMID to normal-mode */
	if (ret = bu26154_reg_update_bits(bu26154, BU26154_VMID_REG,
					  BU26154_VMID_VMIDCON, 0x2)) {
		return ret;
	}
	mdelay(5);

	return 0;
}

/* locks should be held by the caller */
static inline int __configure_fpll(struct snd_soc_codec *codec)
{
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);
	struct bu26154_audio_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct clk_coeff *c;

	c = get_coeff(priv->sysclk_rate, priv->lrclk);
	if (!c)
		return -EINVAL;

	__stop_fpll(bu26154);

	/* Change MAPCON to MAP1 */
	bu26154_set_map(bu26154, BU26154_MAPCON_MAP1);
	bu26154_block_write(bu26154, BU26154_FPLL_M_REG, 9, c->pll);

	__start_fpll(bu26154);
	/* __start_fpll returns pointing to MAP0 */
	return 0;
}

static int playback_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
		/* TODO */
		bu26154_reg_write(bu26154, BU26154_RECPLAY_REG, 0x2);
		bu26154_map_unlock(bu26154);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
		bu26154_reg_write(bu26154, BU26154_RECPLAY_REG, 0x0);
		bu26154_map_unlock(bu26154);
		break;
	}
	return 0;
}

static const DECLARE_TLV_DB_MINMAX_MUTE(digital_tlv, -7200, 0);

static const struct snd_kcontrol_new bu26154_snd_controls[] = {
	SOC_SINGLE_RANGE_TLV("Digital Playback Volume", BU26154_PDIG_VOL_REG,
			     0, 0x6f, 0xff, 0, digital_tlv),
};

static const struct snd_soc_dapm_widget bu26154_dapm_widgets[] = {
	SND_SOC_DAPM_PRE("Pre Playback", playback_event),
	SND_SOC_DAPM_POST("Post Playback", playback_event),
	SND_SOC_DAPM_OUTPUT("SPKOUTL"),
	SND_SOC_DAPM_OUTPUT("SPKOUTR"),
};

static int vmid_power(struct bu26154 *bu26154, bool on)
{
	bu26154_map_lock(bu26154);
	__vmid_power(bu26154, on);
	bu26154_map_unlock(bu26154);
}

static int bu26154_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);
	struct bu26154_audio_priv *priv = snd_soc_codec_get_drvdata(codec);
	int ret;

	priv->bclk = snd_soc_params_to_bclk(params);
	if (params_channels(params) == 1)
		priv->bclk *= 2;

	priv->lrclk = params_rate(params);

	bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);

	/* PCM RX format settings */
	bu26154_reg_update_bits(bu26154, BU26154_SAI_RX_REG,
				BU26154_SAI_RX_PCMFI24,
				params_width(params) == 24 ? 0xc0 : 0x80);

	if (ret = __configure_fpll(codec))
		goto out;

	/* DAC Clock setting */
	bu26154_reg_write(bu26154, BU26154_DACCLK_REG,
			  priv->lrclk == 16000 ? 0x10 : 0x20);

	/* Sampling Rate setting */
	bu26154_reg_write(bu26154, BU26154_SRATE_REG,
			  priv->lrclk == 48000 ? 0x8 :
			  priv->lrclk == 44100 ? 0x7 :
			  priv->lrclk == 32000 ? 0x6 : 0x3);
out:
	bu26154_map_unlock(bu26154);
	return ret;
}

static int bu26154_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
					unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);
	struct bu26154_audio_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int src;

	printk(KERN_INFO "%s: clk_id=%d, freq=%d, dir=%d",
			 __func__, clk_id, freq, dir);

	switch (clk_id) {
	case BU26154_SYSCLK_MCLK:
		src = 1 << 3;
		break;
	case BU26154_SYSCLK_BCLK:
		src = 1 << 4;
		break;
	default:
		return -EINVAL;
	}
	priv->sysclk = clk_id;
	priv->sysclk_rate = freq;

	bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	bu26154_reg_update_bits(bu26154, BU26154_CLKCR_REG,
				BU26154_CLKCR_PLLISEL, src);
	bu26154_map_unlock(bu26154);

	return 0;
}

static int bu26154_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);
	int mode = 0;

	//FIXME
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		mode |= BU26154_SAI_MODE_MST;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	printk(KERN_INFO "%s: fmt=%d\n", __func__, fmt);

	bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	bu26154_reg_update_bits(bu26154, BU26154_SAI_MODE_REG,
				BU26154_SAI_MODE_BSWP | BU26154_SAI_MODE_MST,
				mode);
	bu26154_map_unlock(bu26154);

	return 0;
}

static int bu26154_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);

	bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	if (mute) {
		bu26154_reg_set_bits(bu26154, BU26154_DVCR_REG,
				     BU26154_DVCR_DVMUTE);
	} else {
		bu26154_reg_clear_bits(bu26154, BU26154_DVCR_REG,
				       BU26154_DVCR_DVMUTE);
	}
	bu26154_map_unlock(bu26154);

	return 0;
}

static const struct snd_soc_dai_ops bu26154_dai_ops = {
	.hw_params = bu26154_hw_params,
	.set_sysclk = bu26154_set_dai_sysclk,
	.set_fmt = bu26154_set_dai_fmt,
	.digital_mute = bu26154_mute,
};

#define BU26154_RATES	SNDRV_PCM_RATE_8000_48000

#define BU26154_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |\
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE)

static struct snd_soc_dai_driver bu26154_dai = {
	.name = "bu26154",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = BU26154_RATES,
		.formats = BU26154_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = BU26154_RATES,
		.formats = BU26154_FORMATS,
	},
	.ops = &bu26154_dai_ops,
	.symmetric_rates = 1,
};

static int bu26154_codec_probe(struct snd_soc_codec *codec)
{
	struct bu26154 *bu26154 = dev_get_drvdata(codec->dev->parent);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	int ret;

	/* Ensure MAPCON register points to MAP0 */
	ret = bu26154_set_map_lock(bu26154, BU26154_MAPCON_MAP0);
	if (ret) {
		dev_err(codec->dev, "%s: BU26154_MAPCON reg write failed(MAP0)\n",
			__func__);
		return ret;
	}

	/* Enable MCLK clock */
	bu26154_reg_set_bits(bu26154, BU26154_CLKEN_REG, BU26154_CLKEN_MCLKEN);

	/* Power on DAC-LR */
	bu26154_reg_set_bits(bu26154, BU26154_DAC_POW_REG,
			     BU26154_DAC_POW_DACREN | BU26154_DAC_POW_DACLEN);

	/* Power on Speaker-amp */
	bu26154_reg_write(bu26154, BU26154_SPAMP_POW_REG,
			  BU26154_SPAMP_POW_SPEN |
			  BU26154_SPAMP_POW_AVREN |
			  BU26154_SPAMP_POW_AVLEN |
			  BIT(2));

	/* Initial Volume setting */
	bu26154_reg_write(bu26154, BU26154_PDIG_VOL_REG, 0xa0);

	/* Change MAPCON to MAP1 */
	bu26154_set_map(bu26154, BU26154_MAPCON_MAP1);

	/* Connect L/Rch volume(12dB) to a speaker amplifier */
	bu26154_reg_write(bu26154, BU26154_SPAMP_ICR_REG, 0x0e);

	bu26154_set_map_unlock(bu26154, BU26154_MAPCON_MAP0);

	/* Power on VMID */
	vmid_power(bu26154, true);

	snd_soc_dapm_new_controls(dapm, bu26154_dapm_widgets,
				  ARRAY_SIZE(bu26154_dapm_widgets));

	return 0;
}

static int bu26154_codec_remove(struct snd_soc_codec *codec)
{
	//TODO
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static struct regmap *bu26154_get_regmap(struct device *dev)
{
	struct bu26154 *bu26154 = dev_get_drvdata(dev->parent);
	return bu26154->regmap;
}

static struct snd_soc_codec_driver soc_codec_dev_bu26154 = {
	.probe = bu26154_codec_probe,
	.remove = bu26154_codec_remove,
	.get_regmap = bu26154_get_regmap,
	.idle_bias_off = true,

	.component_driver = {
		.controls = bu26154_snd_controls,
		.num_controls = ARRAY_SIZE(bu26154_snd_controls),
	},
};

static int bu26154_audio_probe(struct platform_device *pdev)
{
	struct bu26154 *bu26154 = dev_get_drvdata(pdev->dev.parent);
	struct bu26154_audio_priv *priv;
	struct bu26154_audio_data *pdata;
	int ret;

	pdata = dev_get_platdata(bu26154->dev);
	if (!pdata) {
		pdata = bu26154_parse_dt(&pdev->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdata = pdata;
	platform_set_drvdata(pdev, priv);

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_bu26154,
				      &bu26154_dai, 1);
}

static int bu26154_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bu26154_audio_ids[] = {
	{ .compatible = "rohm,bu26154-audio", },
	{ },
};
#endif

static struct platform_driver bu26154_audio_driver = {
	.driver	= {
		.name = "bu26154-audio",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bu26154_audio_ids),
#endif
	},
	.probe = bu26154_audio_probe,
	.remove = bu26154_audio_remove,
};
module_platform_driver(bu26154_audio_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASoc BU26154 driver");
