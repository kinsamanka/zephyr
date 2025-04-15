/*
 * Copyright (c) 2024 Daikin Comfort Technologies North America, Inc.
 *
 * Heavily based on pwm_sam0_tcc.c, which is:
 * Copyright (c) 2020 Google LLC.
 * Copyright (c) 2025 GP Orcullo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PWM driver using the SAM0 Timer/Counter (TC) Supports the SAMD21 and SAMD5x series,
 * 8 and 16 bit counter size is supported.
 *
 * The 8-bit counter operates in Normal PWM (NPWM) mode, it supports pulse width and period
 * values between 0 and 255. It is ideal for applications requiring moderate frequency PWM,
 * however, it is not suitable for high-precision or low-frequency applications.
 *
 * The 16-bit counter operates in Match PWM (MPWM) mode to generate the PWM signal.
 * this mode sacrifices the timer's CC0 channel in order to achieve pulse width modulation.
 */

#define DT_DRV_COMPAT atmel_sam0_tc_pwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>

#include "pwm_sam0_tc.h"

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

#define COUNTER_8BITS 8U

/* Wait for the peripheral to finish all commands */
static void wait_synchronization(uintptr_t regs)
{
#if defined(CONFIG_SOC_SERIES_SAMD20) || defined(CONFIG_SOC_SERIES_SAMD21) ||                      \
	defined(CONFIG_SOC_SERIES_SAMR21)
	/* SYNCBUSY is a bit */
	while (sys_read8(regs + STATUS_OFFSET) & STATUS_SYNCBUSY) {
	}
#else
	/* SYNCBUSY is a register */
	while (sys_read32(regs + SYNCBUSY_OFFSET) != 0) {
	}
#endif
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

static int pwm_sam0_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_sam0_config *const cfg = dev->config;
	uintptr_t regs = cfg->regs;
	uint8_t counter_size = cfg->counter_size;
	uint32_t top = 1 << counter_size;
	uint32_t invert_mask = 1 << channel;
	bool invert = ((flags & PWM_POLARITY_INVERTED) != 0);
	bool inverted = (sys_read8(regs + DRVCTRL_OFFSET) & invert_mask) != 0;

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
	if (COUNTER_8BITS == counter_size) {
		sys_write8(pulse_cycles, regs + CC_OFFSET + channel);
		sys_write8(period_cycles, regs + PER_OFFSET);
	} else {
		sys_write16(period_cycles, regs + CC_OFFSET);
		sys_write16(pulse_cycles, regs + CC_OFFSET + 2);
	}

	if (invert != inverted) {
		/* Wait until previous update is done */
		wait_synchronization(regs);

		/* On some devices, CTRLA register is 16bits */
		uint16_t ctrla = sys_read16(regs + CTRLA_OFFSET);

		WRITE_BIT(ctrla, CTRLA_ENABLE_BIT, 0);
		sys_write16(ctrla, regs + CTRLA_OFFSET);
		wait_synchronization(regs);

		invert_mask ^= sys_read8(regs + DRVCTRL_OFFSET);

		sys_write8(invert_mask, regs + DRVCTRL_OFFSET);

		WRITE_BIT(ctrla, CTRLA_ENABLE_BIT, 1);
		sys_write16(ctrla, regs + CTRLA_OFFSET);
		wait_synchronization(regs);
	}

	return 0;
}

static int pwm_sam0_init(const struct device *dev)
{
	const struct pwm_sam0_config *const cfg = dev->config;
	const uintptr_t gclk = DT_REG_ADDR(DT_INST(0, atmel_sam0_gclk));
	uint8_t counter_size = cfg->counter_size;
	uintptr_t regs = cfg->regs;
	int retval;

	*cfg->mclk |= cfg->mclk_mask;

#if !defined(CONFIG_SOC_SERIES_SAMD20) && !defined(CONFIG_SOC_SERIES_SAMD21) &&                    \
	!defined(CONFIG_SOC_SERIES_SAMR21)
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

	/* On some devices, CTRLA register is 16bits */
	uint16_t ctrla = sys_read16(regs + CTRLA_OFFSET);

	WRITE_BIT(ctrla, CTRLA_SWRST_BIT, 1);
	sys_write16(ctrla, regs + CTRLA_OFFSET);
	wait_synchronization(regs);

	uint8_t wave = sys_read8(regs + WAVE_OFFSET);

	if (COUNTER_8BITS == counter_size) {
		sys_write16(cfg->prescaler | CTRLA_MODE_COUNT8 | CTRLA_PRESCSYNC_PRESC,
			    regs + CTRLA_OFFSET);

		wave &= ~WAVE_WAVEGEN_MASK;
		sys_write8(wave | WAVE_WAVEGEN_NPWM, regs + WAVE_OFFSET);
		sys_write8(1, regs + PER_OFFSET);
	} else {
		sys_write16(cfg->prescaler | CTRLA_MODE_COUNT16 | CTRLA_PRESCSYNC_PRESC,
			    regs + CTRLA_OFFSET);

		wave &= ~WAVE_WAVEGEN_MASK;
		sys_write8(wave | WAVE_WAVEGEN_MPWM, regs + WAVE_OFFSET);
		sys_write16(1, regs + CC_OFFSET);
	}

	ctrla = sys_read16(regs + CTRLA_OFFSET);
	WRITE_BIT(ctrla, CTRLA_ENABLE_BIT, 1);
	sys_write16(ctrla, regs + CTRLA_OFFSET);
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
										\
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
			      POST_KERNEL, CONFIG_PWM_TC_INIT_PRIORITY,		\
			      &pwm_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SAM0_INIT)

/* clang-format on */
