/*
 * Copyright (c) 2020 Google LLC.
 * Copyright (c) 2024 Gerson Fernando Budke <nandojve@gmail.com>
 * Copyright (c) 2025 GP Orcullo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PWM driver using the SAM0 Timer/Counter (TCC) in Normal PWM (NPWM) mode.
 * Supports the SAMD21 and SAMD5x series.
 */

#define DT_DRV_COMPAT atmel_sam0_tcc_pwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>

#include "pwm_sam0_tcc.h"

/* clang-format off */

/* Static configuration */
struct pwm_sam0_config {
	uintptr_t regs;
	const struct pinctrl_dev_config *pcfg;
	uint8_t channels;
	uint8_t counter_size;
	uint16_t prescaler;
	uint32_t freq;
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint32_t gclk_gen;
	uint16_t gclk_id;
};

/* Wait for the peripheral to finish all commands */
static void wait_synchronization(uintptr_t regs)
{
	while (sys_read32(regs + SYNCBUSY_OFFSET) != 0) {
	}
}

static int pwm_sam0_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_sam0_config *const cfg = dev->config;

	if (channel >= cfg->channels) {
		return -EINVAL;
	}
	*cycles = cfg->freq;

	return 0;
}

static int pwm_sam0_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct pwm_sam0_config *const cfg = dev->config;
	uintptr_t regs = cfg->regs;
	uint32_t top = 1 << cfg->counter_size;
	bool invert = ((flags & PWM_POLARITY_INVERTED) != 0);
	uint32_t invert_mask;
	bool inverted;

	/* Invert all outputs belonging to the same channel */
	invert_mask = FIELD_PREP(DRVCTRL_INVEN_MASK, (BIT(cfg->channels) | 1) << channel);

	inverted = (sys_read32(regs + DRVCTRL_OFFSET) & invert_mask) != 0;

	if (channel >= cfg->channels) {
		return -EINVAL;
	}
	if (period_cycles >= top || pulse_cycles >= top) {
		return -EINVAL;
	}

	/*
	 * Update the buffered width and period.  These will be automatically
	 * loaded on the next cycle.
	 */
	sys_write32(CCBUF_CCBUF(pulse_cycles), regs + CCBUF_OFFSET + (4 * channel));
	sys_write32(PERBUF_PERBUF(period_cycles), regs + PERBUF_OFFSET);

	if (invert != inverted) {
		/* Wait until previous update is done */
		wait_synchronization(regs);

		sys_clear_bit(regs + CTRLA_OFFSET, CTRLA_ENABLE_BIT);
		wait_synchronization(regs);

		invert_mask ^= sys_read32(regs + DRVCTRL_OFFSET);

		sys_write32(invert_mask, regs + DRVCTRL_OFFSET);
		sys_set_bit(regs + CTRLA_OFFSET, CTRLA_ENABLE_BIT);
		wait_synchronization(regs);
	}

	return 0;
}

static int pwm_sam0_init(const struct device *dev)
{
	const struct pwm_sam0_config *const cfg = dev->config;
	const uintptr_t gclk = DT_REG_ADDR(DT_INST(0, atmel_sam0_gclk));
	uintptr_t regs = cfg->regs;
	int retval;

	*cfg->mclk |= cfg->mclk_mask;

#if !defined(CONFIG_SOC_SERIES_SAMD21) && !defined(CONFIG_SOC_SERIES_SAMR21)
	sys_write32(PCHCTRL_CHEN | PCHCTRL_GEN(cfg->gclk_gen),
		    gclk + PCHCTRL_OFFSET + (4 * cfg->gclk_id));
#else
	sys_write16(CLKCTRL_CLKEN | CLKCTRL_GEN(cfg->gclk_gen) | CLKCTRL_ID(cfg->gclk_id),
		    gclk + CLKCTRL_OFFSET);
#endif

	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	sys_set_bit(regs + CTRLA_OFFSET, CTRLA_SWRST_BIT);
	wait_synchronization(regs);

	sys_write32(cfg->prescaler, regs + CTRLA_OFFSET);
	uint32_t tmp = sys_read32(regs + WAVE_OFFSET) & ~WAVE_WAVEGEN_MASK;

	sys_write32(tmp | WAVE_WAVEGEN_NPWM, regs + WAVE_OFFSET);
	sys_write32(PER_PER(1), regs + PER_OFFSET);

	sys_set_bit(regs + CTRLA_OFFSET, CTRLA_ENABLE_BIT);
	wait_synchronization(regs);

	return 0;
}

static DEVICE_API(pwm, pwm_sam0_driver_api) = {
	.set_cycles = pwm_sam0_set_cycles,
	.get_cycles_per_sec = pwm_sam0_get_cycles_per_sec,
};

#ifndef ATMEL_SAM0_DT_INST_CELL_REG_ADDR_OFFSET
#define ATMEL_SAM0_DT_INST_CELL_REG_ADDR_OFFSET(n, cell)			\
	(volatile uint32_t *)							\
	(DT_REG_ADDR(DT_INST_PHANDLE_BY_NAME(n, clocks, cell)) +		\
	 DT_INST_CLOCKS_CELL_BY_NAME(n, cell, offset))
#endif

#ifndef ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET
#define ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n)				\
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(mclk)),		\
		(ATMEL_SAM0_DT_INST_CELL_REG_ADDR_OFFSET(n, mclk)),		\
		(ATMEL_SAM0_DT_INST_CELL_REG_ADDR_OFFSET(n, pm)))
#endif

#ifndef ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK
#define ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, cell)				\
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(mclk)),		\
		(BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, cell))),		\
		(BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, pm, cell))))
#endif

#define PWM_SAM0_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
	static const struct pwm_sam0_config pwm_sam0_config_##inst = {		\
		.regs = DT_INST_REG_ADDR(inst),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.channels = DT_INST_PROP(inst, channels),			\
		.counter_size = DT_INST_PROP(inst, counter_size),		\
		.prescaler = UTIL_CAT(CTRLA_PRESCALER_DIV,			\
				      DT_INST_PROP(inst, prescaler)),		\
		.freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /			\
			DT_INST_PROP(inst, prescaler),				\
		.gclk_gen = DT_PHA_BY_NAME(DT_DRV_INST(inst), atmel_assigned_clocks, gclk, gen), \
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, id),		\
		.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(inst),	\
		.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(inst, bit),	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, &pwm_sam0_init, NULL,			\
			      NULL, &pwm_sam0_config_##inst,			\
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,		\
			      &pwm_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SAM0_INIT)

/* clang-format on */
