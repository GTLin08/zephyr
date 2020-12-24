/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_pwm

#include <logging/log.h>

LOG_MODULE_REGISTER(pwm_ite_it8xxx2);

#include <sys/sys_io.h>
#include <device.h>
#include <drivers/pwm.h>
#include <soc.h>

#include "pwm_ite_it8xxx2.h"

/* Macros */
#define DEV_CFG(dev) \
	((const struct pwm_it8xxx2_cfg * const)(dev)->config)

#define ABS(x) ((x) >= 0 ? (x) : -(x))

#define PWM_CTRX_MIN 100
#define PWM_EC_FREQ  8000000
#define PWM_CH_COUNT 8

const struct pwm_ctrl_t2 pwm_clock_ctrl_regs[] = {
	{ &IT83XX_PWM_CTR, &IT83XX_PWM_C0CPRS, &IT83XX_PWM_C0CPRS,
		&IT83XX_PWM_PCFSR, 0x01},
	{ &IT83XX_PWM_CTR1, &IT83XX_PWM_C4CPRS, &IT83XX_PWM_C4MCPRS,
		&IT83XX_PWM_PCFSR, 0x02},
	{ &IT83XX_PWM_CTR2, &IT83XX_PWM_C6CPRS, &IT83XX_PWM_C6MCPRS,
		&IT83XX_PWM_PCFSR, 0x04},
	{ &IT83XX_PWM_CTR3, &IT83XX_PWM_C7CPRS, &IT83XX_PWM_C7MCPRS,
		&IT83XX_PWM_PCFSR, 0x08},
};

/* Structure Declarations */
struct pwm_it8xxx2_cfg {
	uint32_t base;
	uint32_t f_sys;
	enum pwm_pcfsr_sel pcfsr_sel;
	uint32_t flags;
	int channel;
};

static void pwm_enable(const struct device *dev, int enabled)
{
	const struct pwm_it8xxx2_cfg *config = DEV_CFG(dev);

	/*
	 * enabled : pin to PWM function.
	 * disabled : pin to GPIO input function.
	 */
	if (enabled) {
		IT83XX_GPIO_GPCRA(config->channel) = 0x00;
	} else {
		IT83XX_GPIO_GPCRA(config->channel) = 0x80 |
			((config->flags & PWM_CONFIG_ACTIVE_LOW) ?
			4 : 2);
	}
}

static int pwm_it8xxx2_pin_set(const struct device *dev,
			      uint32_t pwm, uint32_t period_cycles,
			      uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_it8xxx2_cfg *config = DEV_CFG(dev);
	int pcs_shift;
	int pcs_mask;
	int pcs_reg;
	int cycle_time_setting;
	uint8_t duty_cycle;
	ARG_UNUSED(flags);

	if (pwm >= PWM_CH_COUNT) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	pwm_enable(dev, 1);

	if ((period_cycles == 0) || (pulse_cycles > period_cycles)) {
		LOG_ERR("Requested pulse %d is longer than period %d\n",
			pulse_cycles, period_cycles);
		return -EINVAL;
	}

	if (period_cycles > UINT16_MAX) {
		/* 16-bit resolution */
		LOG_ERR("Too long period (%u), adjust pwm prescaler!",
			period_cycles);
		/* TODO: dynamically adjust prescaler */
		return -EINVAL;
	}

	duty_cycle = 100 * pulse_cycles / period_cycles;

	if (config->flags & PWM_CONFIG_ACTIVE_LOW) {
		duty_cycle = 100 - duty_cycle;
	}

	/* bit shift for "Prescaler Clock Source Select Group" register. */
	pcs_shift = (pwm % 4) * 2;

	/* setting of "Prescaler Clock Source Select Group" register.*/
	if (pwm  > 3) {
		pcs_reg = IT83XX_PWM_PCSSGH;
	} else {
		pcs_reg = IT83XX_PWM_PCSSGL;
	}

	/* only bit0 bit1 information. */
	pcs_mask = (pcs_reg >> pcs_shift) & 0x03;

	/* get cycle time setting of PWM channel x. */
	cycle_time_setting = *pwm_clock_ctrl_regs[pcs_mask].pwm_cycle_time;

	/* to update PWM DCRx depend on CTRx setting. */
	if (duty_cycle == 100) {
		IT83XX_PWM_DCR(config->base) = cycle_time_setting;
	} else {
		IT83XX_PWM_DCR(config->base) = 
			((cycle_time_setting + 1) * duty_cycle) / 100;
	}

	return 0;
}

static int pwm_it8xxx2_get_cycles_per_sec(const struct device *dev,
					 uint32_t pwm,
					 uint64_t *cycles)
{
	const struct pwm_it8xxx2_cfg *config = DEV_CFG(dev);
	int pcs_mask;
	int pcs_reg;
	int cycle_time_setting;

	/* setting of "Prescaler Clock Source Select Group" register. */
	if (pwm > 3) {
		pcs_reg = IT83XX_PWM_PCSSGH;
	} else {
		pcs_reg = IT83XX_PWM_PCSSGL;
	}

	/* only bit0 bit1 information. */
	pcs_mask = (pcs_reg >> ((pwm % 4) * 2)) & 0x03;

	/* get cycle time setting of PWM channel x. */
	cycle_time_setting = *pwm_clock_ctrl_regs[pcs_mask].pwm_cycle_time;

	*cycles = IT83XX_PWM_DCR(config->base) * 100 / cycle_time_setting;

	if (config->flags & PWM_CONFIG_ACTIVE_LOW) {
		*cycles = 100 - *cycles;
	}

	return 0;
}

static int pwm_ch_freq(const struct device *dev)
{
	const struct pwm_it8xxx2_cfg *config = DEV_CFG(dev);
	int actual_freq = -1, targe_freq, deviation;
	int pcfsr, ctr, pcfsr_sel, pcs_shift, pcs_mask;
	int pwm_clk_src = (config->flags & PWM_CONFIG_DSLEEP) ?
							32768 : PWM_EC_FREQ;

	targe_freq = config->f_sys;
	deviation = (targe_freq / 100) + 1;

	for (ctr = 0xFF; ctr >= PWM_CTRX_MIN; ctr--) {
		pcfsr = (pwm_clk_src / (ctr + 1) / targe_freq) - 1;
		if (pcfsr >= 0) {
			actual_freq = pwm_clk_src / (ctr + 1) / (pcfsr + 1);
			if (ABS(actual_freq - targe_freq) < deviation)
				break;
		}
	}

	if (ctr < PWM_CTRX_MIN) {
		actual_freq = -1;
	} else {
		pcfsr_sel = config->pcfsr_sel;
		*pwm_clock_ctrl_regs[pcfsr_sel].pwm_cycle_time = ctr;

		if (config->flags & PWM_CONFIG_DSLEEP) {
			/*
			 * Select 32.768KHz as PWM clock source.
			 *
			 * NOTE:
			 * For pwm_channels[], the maximum supported pwm output
			 * signal frequency is 324 Hz (32768/(PWM_CTRX_MIN+1)).
			 */
			*pwm_clock_ctrl_regs[pcfsr_sel].pwm_pcfsr_reg &=
				~pwm_clock_ctrl_regs[pcfsr_sel].pwm_pcfsr_ctrl;
		} else {
			/* ec clock 8MHz */
			*pwm_clock_ctrl_regs[pcfsr_sel].pwm_pcfsr_reg |=
				pwm_clock_ctrl_regs[pcfsr_sel].pwm_pcfsr_ctrl;
		}

		/*
		 * bit shift for "Prescaler Clock Source Select Group"
		 * register.
		 */
		pcs_shift = (config->channel % 4) * 2;
		pcs_mask = pcfsr_sel << pcs_shift;

		if (config->channel > 3) {
			IT83XX_PWM_PCSSGH &= ~(0x3 << pcs_shift);
			IT83XX_PWM_PCSSGH |= pcs_mask;
		}  else {
			IT83XX_PWM_PCSSGL &= ~(0x3 << pcs_shift);
			IT83XX_PWM_PCSSGL |= pcs_mask;
		}

		*pwm_clock_ctrl_regs[pcfsr_sel].pwm_cpr_lsb = pcfsr & 0xFF;
		*pwm_clock_ctrl_regs[pcfsr_sel].pwm_cpr_msb = (pcfsr >> 8) & 0xFF;
	}

	return actual_freq;
}

static int pwm_it8xxx2_init(const struct device *dev)
{

	pwm_ch_freq(dev);

	/*
	 * The cycle timer1 of chip 8320 later series was enhanced from
	 * 8bits to 10bits resolution, and others are still 8bit resolution.
	 * Because the cycle timer1 high byte default value is not zero,
	 * we clear cycle timer1 high byte at init and use it as 8-bit
	 * resolution like others.
	 */
	IT83XX_PWM_CTR1M = 0;
	/* enable PWMs clock counter. */
	IT83XX_PWM_ZTIER |= 0x02;

	return 0;
}

/* Device Instantiation */

static const struct pwm_driver_api pwm_it8xxx2_api = {
	.pin_set = pwm_it8xxx2_pin_set,
	.get_cycles_per_sec = pwm_it8xxx2_get_cycles_per_sec,
};

#define PWM_IT8XXX2_INIT(idx)						\
	static const struct pwm_it8xxx2_cfg pwm_it8xxx2_cfg_##idx = {	\
			.base = DT_INST_REG_ADDR(idx),			\
			.f_sys = DT_INST_PROP(idx, clock_frequency),	\
			.pcfsr_sel = DT_INST_PROP(idx, pwm_prescaler),	\
			.flags = DT_PHA_BY_IDX(DT_NODELABEL(led##idx), pwms, 0, flags),		\
			.channel = DT_PHA_BY_IDX(DT_NODELABEL(led##idx), pwms, 0, channel),	\
		};							\
	DEVICE_DT_INST_DEFINE(idx,					\
			    pwm_it8xxx2_init,				\
			    device_pm_control_nop,			\
			    NULL,					\
			    &pwm_it8xxx2_cfg_##idx,			\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &pwm_it8xxx2_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_IT8XXX2_INIT)
