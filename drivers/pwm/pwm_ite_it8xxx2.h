/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header file for the PCA9685 PWM driver.
 */

#ifndef ZEPHYR_DRIVERS_PWM_PWM_ITE_IT8XXX2_H_
#define ZEPHYR_DRIVERS_PWM_PWM_ITE_IT8XXX2_H_

enum pwm_pcfsr_sel {
	PWM_PRESCALER_C4 = 1,
	PWM_PRESCALER_C6 = 2,
	PWM_PRESCALER_C7 = 3,
};

/* Data structure to define PWM channel control registers part 2. */
struct pwm_ctrl_t2 {
	/* PWM cycle time register. */
	volatile uint8_t *pwm_cycle_time;
	/* PWM channel clock prescaler register (LSB). */
	volatile uint8_t *pwm_cpr_lsb;
	/* PWM channel clock prescaler register (MSB). */
	volatile uint8_t *pwm_cpr_msb;
	/* PWM prescaler clock frequency select register. */
	volatile uint8_t *pwm_pcfsr_reg;
	/* PWM prescaler clock frequency select register setting. */
	uint8_t pwm_pcfsr_ctrl;
};

/* Flags for PWM config table */
/**
 * PWM output signal is inverted, so 100% duty means always low
 */
#define PWM_CONFIG_ACTIVE_LOW		BIT(0)
/**
 * PWM channel has a fan controller with a tach input and can
 * auto-adjust its duty cycle to produce a given fan RPM.
 */
#define PWM_CONFIG_HAS_RPM_MODE		BIT(1)
/**
 * PWM clock select alternate source.  The actual clock and alternate
 * source are chip dependent.
 */
#define PWM_CONFIG_ALT_CLOCK		BIT(2)
/**
 * PWM channel has a complementary output signal which should
 * be enabled in addition to the primary output.
 */
#define PWM_CONFIG_COMPLEMENTARY_OUTPUT	BIT(3)
/**
 * PWM channel must stay active in low-power idle, if enabled.
 */
#define PWM_CONFIG_DSLEEP		BIT(4)
/**
 * PWM channel's IO type is open-drain, if enabled. (default IO
 * is push-pull.)
 */
#define PWM_CONFIG_OPEN_DRAIN		BIT(5)


#endif /* ZEPHYR_DRIVERS_PWM_PWM_ITE_IT8XXX2_H_ */
