/*
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>
 * Copyright (c) 2024 Gerson Fernando Budke <nandojve@gmail.com>
 * Copyright (c) 2025 GP Orcullo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_tc32

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(counter_sam0_tc32, CONFIG_COUNTER_LOG_LEVEL);

#include "counter_sam0_tc32.h"

/* clang-format off */

struct counter_sam0_tc32_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_sam0_tc32_data {
	counter_top_callback_t top_cb;
	void *top_user_data;

	struct counter_sam0_tc32_ch_data ch;
};

struct counter_sam0_tc32_config {
	struct counter_config_info info;
	uintptr_t regs;
	const struct pinctrl_dev_config *pcfg;
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint32_t gclk_gen;
	uint16_t gclk_id;
	uint16_t prescaler;
	void (*irq_config_func)(const struct device *dev);
};

static void wait_synchronization(uintptr_t regs)
{
#if !defined(CONFIG_SOC_SERIES_SAMD20) && !defined(CONFIG_SOC_SERIES_SAMD21) &&                    \
	!defined(CONFIG_SOC_SERIES_SAMR21)
	/* SYNCBUSY is a register */
	while ((sys_read32(regs + SYNCBUSY_OFFSET) & SYNCBUSY_MASK) != 0) {
	}
#else
	/* SYNCBUSY is a bit */
	while ((sys_read8(regs + STATUS_OFFSET) & STATUS_SYNCBUSY) != 0) {
	}
#endif
}

static void read_synchronize_count(uintptr_t regs)
{
#if defined(READREQ_OFFSET)
	sys_write16(READREQ_RREQ | READREQ_ADDR(COUNT_OFFSET), regs + READREQ_OFFSET);
#else
	uint8_t ctrlbset = sys_read8(regs + CTRLBSET_OFFSET) & ~CTRLBSET_CMD_MASK;

	sys_write8(ctrlbset | CTRLBSET_CMD_READSYNC, regs + CTRLBSET_OFFSET);
#endif
	wait_synchronization(regs);
}

static int counter_sam0_tc32_start(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	/*
	 * This will also reset the current counter value if it's
	 * already running.
	 */
	uint8_t ctrlbset = sys_read8(tc + CTRLBSET_OFFSET) & ~CTRLBSET_CMD_MASK;

	sys_write8(ctrlbset | CTRLBSET_CMD_RETRIGGER, tc + CTRLBSET_OFFSET);
	wait_synchronization(tc);
	return 0;
}

static int counter_sam0_tc32_stop(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	/*
	 * The older (pre SAML1x) manuals claim the counter retains its
	 * value on stop, but this doesn't actually seem to happen.
	 * The SAML1x manual says it resets, which is what the SAMD21
	 * counter actually appears to do.
	 */
	uint8_t ctrlbset = sys_read8(tc + CTRLBSET_OFFSET) & ~CTRLBSET_CMD_MASK;

	sys_write8(ctrlbset | CTRLBSET_CMD_STOP, tc + CTRLBSET_OFFSET);
	wait_synchronization(tc);
	return 0;
}

static uint32_t counter_sam0_tc32_read(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	read_synchronize_count(tc);
	return sys_read32(tc + COUNT_OFFSET);
}

static int counter_sam0_tc32_get_value(const struct device *dev,
				       uint32_t *ticks)
{
	*ticks = counter_sam0_tc32_read(dev);
	return 0;
}

static void counter_sam0_tc32_relative_alarm(const struct device *dev,
					     uint32_t ticks)
{
	struct counter_sam0_tc32_data *data = dev->data;
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;
	uint32_t before;
	uint32_t target;
	uint32_t after;
	uint32_t max;

	read_synchronize_count(tc);
	before = sys_read32(tc + COUNT_OFFSET);

	target = before + ticks;
	max = sys_read32(tc + CC0_OFFSET);
	if (target > max) {
		target -= max;
	}

	sys_write32(target, tc + CC1_OFFSET);
	wait_synchronization(tc);
	sys_write8(INTFLAG_MC1, tc + INTFLAG_OFFSET);

	read_synchronize_count(tc);
	after = sys_read32(tc + COUNT_OFFSET);

	/* Pending now, so no further checking required */
	if (sys_read8(tc + INTFLAG_OFFSET) & INTFLAG_MC1) {
		goto out_future;
	}

	/*
	 * Check if we missed the interrupt and call the handler
	 * immediately if we did.
	 */
	if (after < target) {
		goto out_future;
	}

	/* Check wrapped */
	if (target < before && after >= before) {
		goto out_future;
	}

	counter_alarm_callback_t cb = data->ch.callback;

	sys_write8(INTENCLR_MC1, tc + INTENCLR_OFFSET);
	sys_write8(INTFLAG_MC1, tc + INTFLAG_OFFSET);
	data->ch.callback = NULL;

	cb(dev, 0, target, data->ch.user_data);

	return;

out_future:
	sys_write8(INTENSET_MC1, tc + INTENSET_OFFSET);
}

static int counter_sam0_tc32_set_alarm(const struct device *dev,
				       uint8_t chan_id,
				       const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_sam0_tc32_data *data = dev->data;
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	ARG_UNUSED(chan_id);

	if (alarm_cfg->ticks > sys_read32(tc + CC0_OFFSET)) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (data->ch.callback) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->ch.callback = alarm_cfg->callback;
	data->ch.user_data = alarm_cfg->user_data;

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0) {
		sys_write32(alarm_cfg->ticks, tc + CC1_OFFSET);
		wait_synchronization(tc);
		sys_write8(INTFLAG_MC1, tc + INTFLAG_OFFSET);
		sys_write8(INTENSET_MC1, tc + INTENSET_OFFSET);
	} else {
		counter_sam0_tc32_relative_alarm(dev, alarm_cfg->ticks);
	}

	irq_unlock(key);

	return 0;
}

static int counter_sam0_tc32_cancel_alarm(const struct device *dev,
					  uint8_t chan_id)
{
	struct counter_sam0_tc32_data *data = dev->data;
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	unsigned int key = irq_lock();

	ARG_UNUSED(chan_id);

	data->ch.callback = NULL;
	sys_write8(INTENCLR_MC1, tc + INTENCLR_OFFSET);
	sys_write8(INTFLAG_MC1, tc + INTFLAG_OFFSET);

	irq_unlock(key);
	return 0;
}

static int counter_sam0_tc32_set_top_value(const struct device *dev,
					   const struct counter_top_cfg *top_cfg)
{
	struct counter_sam0_tc32_data *data = dev->data;
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;
	int err = 0;
	unsigned int key = irq_lock();

	if (data->ch.callback) {
		irq_unlock(key);
		return -EBUSY;
	}

	if (top_cfg->callback) {
		data->top_cb = top_cfg->callback;
		data->top_user_data = top_cfg->user_data;
		sys_write8(INTENSET_MC0, tc + INTENSET_OFFSET);
	} else {
		sys_write8(INTENCLR_MC0, tc + INTENCLR_OFFSET);
	}

	sys_write32(top_cfg->ticks, tc + CC0_OFFSET);
	uint8_t ctrlbset = sys_read8(tc + CTRLBSET_OFFSET) & ~CTRLBSET_CMD_MASK;

	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/*
		 * Top trigger is on equality of the rising edge only, so
		 * manually reset it if the counter has missed the new top.
		 */
		if (counter_sam0_tc32_read(dev) >= top_cfg->ticks) {
			err = -ETIME;
			if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				sys_write8(ctrlbset | CTRLBSET_CMD_RETRIGGER, tc + CTRLBSET_OFFSET);
			}
		}
	} else {
		sys_write8(ctrlbset | CTRLBSET_CMD_RETRIGGER, tc + CTRLBSET_OFFSET);
	}

	wait_synchronization(tc);

	sys_write8(INTFLAG_MC0, tc + INTFLAG_OFFSET);
	irq_unlock(key);
	return err;
}

static uint32_t counter_sam0_tc32_get_pending_int(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	return sys_read8(tc + INTFLAG_OFFSET) & (INTFLAG_MC0 | INTFLAG_MC1);
}

static uint32_t counter_sam0_tc32_get_top_value(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;

	/*
	 * Unsync read is safe here because we're not using
	 * capture mode, so things are only set from the CPU
	 * end.
	 */
	return sys_read32(tc + CC0_OFFSET);
}

static void counter_sam0_tc32_isr(const struct device *dev)
{
	struct counter_sam0_tc32_data *data = dev->data;
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	uintptr_t tc = cfg->regs;
	uint8_t status = sys_read8(tc + INTFLAG_OFFSET);

	/* Acknowledge all interrupts */
	sys_write8(status, tc + INTFLAG_OFFSET);

	if (status & INTFLAG_MC1) {
		if (data->ch.callback) {
			counter_alarm_callback_t cb = data->ch.callback;

			sys_write8(INTENCLR_MC1, tc + INTENCLR_OFFSET);
			data->ch.callback = NULL;

			cb(dev, 0, sys_read32(tc + CC1_OFFSET), data->ch.user_data);
		}
	}

	if (status & INTFLAG_MC0) {
		if (data->top_cb) {
			data->top_cb(dev, data->top_user_data);
		}
	}
}

static int counter_sam0_tc32_initialize(const struct device *dev)
{
	const struct counter_sam0_tc32_config *const cfg = dev->config;
	const uintptr_t gclk = DT_REG_ADDR(DT_INST(0, atmel_sam0_gclk));
	uintptr_t tc = cfg->regs;
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

	/*
	 * In 32 bit mode, NFRQ mode always uses MAX as the counter top, so
	 * use MFRQ mode which uses CC0 as the top at the expense of only
	 * having CC1 available for alarms.
	 */
	sys_write16(CTRLA_MODE_COUNT32 | CTRLA_WAVEGEN_MFRQ | cfg->prescaler, tc + CTRLA_OFFSET);
	wait_synchronization(tc);

#ifdef WAVE_WAVEGEN_MFRQ
	sys_write8(WAVE_WAVEGEN_MFRQ, tc + WAVE_OFFSET);
#endif

	/* Disable all interrupts */
	sys_write8(INTENCLR_MASK, tc + INTENCLR_OFFSET);

	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Set the initial top as the maximum */
	sys_write32(UINT32_MAX, tc + CC0_OFFSET);

	cfg->irq_config_func(dev);

	uint16_t ctrla = sys_read16(tc + CTRLA_OFFSET);

	WRITE_BIT(ctrla, CTRLA_ENABLE_BIT, 1);
	sys_write16(ctrla, tc + CTRLA_OFFSET);

	wait_synchronization(tc);

	/* Stop the counter initially */
	uint8_t ctrlbset = sys_read8(tc + CTRLBSET_OFFSET) & ~CTRLBSET_CMD_MASK;

	sys_write8(ctrlbset | CTRLBSET_CMD_STOP, tc + CTRLBSET_OFFSET);
	wait_synchronization(tc);

	return 0;
}

static DEVICE_API(counter, counter_sam0_tc32_driver_api) = {
	.start = counter_sam0_tc32_start,
	.stop = counter_sam0_tc32_stop,
	.get_value = counter_sam0_tc32_get_value,
	.set_alarm = counter_sam0_tc32_set_alarm,
	.cancel_alarm = counter_sam0_tc32_cancel_alarm,
	.set_top_value = counter_sam0_tc32_set_top_value,
	.get_pending_int = counter_sam0_tc32_get_pending_int,
	.get_top_value = counter_sam0_tc32_get_top_value,
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

#define SAM0_TC32_PRESCALER(n)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prescaler),			\
		    (DT_INST_PROP(n, prescaler)), (1))

#define COUNTER_SAM0_TC32_DEVICE(n)						\
	PINCTRL_DT_INST_DEFINE(n);						\
	static void counter_sam0_tc32_config_##n(const struct device *dev);	\
	static const struct counter_sam0_tc32_config				\
										\
	counter_sam0_tc32_dev_config_##n = {					\
		.info = {							\
			.max_top_value = UINT32_MAX,				\
			.freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /		\
				SAM0_TC32_PRESCALER(n),				\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			\
			.channels = 1						\
		},								\
		.regs = DT_INST_REG_ADDR(n),					\
		.gclk_gen = DT_PHA_BY_NAME(DT_DRV_INST(n), atmel_assigned_clocks, gclk, gen), \
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),		\
		.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),		\
		.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),	\
		.prescaler = UTIL_CAT(CTRLA_PRESCALER_DIV,			\
				      SAM0_TC32_PRESCALER(n)),			\
		.irq_config_func = &counter_sam0_tc32_config_##n,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	};									\
										\
	static struct counter_sam0_tc32_data counter_sam0_tc32_dev_data_##n;	\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			    &counter_sam0_tc32_initialize,			\
			    NULL,						\
			    &counter_sam0_tc32_dev_data_##n,			\
			    &counter_sam0_tc32_dev_config_##n,			\
			    PRE_KERNEL_1,					\
			    CONFIG_COUNTER_INIT_PRIORITY,			\
			    &counter_sam0_tc32_driver_api);			\
										\
	static void counter_sam0_tc32_config_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    counter_sam0_tc32_isr,				\
			    DEVICE_DT_INST_GET(n), 0);				\
		irq_enable(DT_INST_IRQN(n));					\
	}

DT_INST_FOREACH_STATUS_OKAY(COUNTER_SAM0_TC32_DEVICE)

/* clang-format on */
