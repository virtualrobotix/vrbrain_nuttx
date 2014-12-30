/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file board_pwm_input.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/stm32/drv_pwm_input.h>
#include <drivers/drv_rc_input.h>

#include "board_config.h"

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)

/* PWM Input
* RC1 PE9  Timer 1 Channel 1 (AF1)
* RC2 PE11 Timer 1 Channel 2 (AF1)
* RC3 PE13 Timer 1 Channel 3 (AF1)
* RC4 PE14 Timer 1 Channel 4 (AF1)
* RC5 PE5  Timer 9 Channel 1 (AF3)
* RC6 PE6  Timer 9 Channel 2 (AF3)
* RC7 PC8  Timer 8 Channel 3 (AF3)
* RC8 PC9  Timer 8 Channel 4 (AF3)
*/

__EXPORT const struct pwm_input_timer pwm_input_timers[PWM_INPUT_MAX_TIMERS] = {
	{
		.base	= STM32_TIM1_BASE,
		.clock_register	= STM32_RCC_APB2ENR,
		.clock_bit	= RCC_APB2ENR_TIM1EN,
		.vector	= STM32_IRQ_TIM1CC,
		.clock_freq	= STM32_APB2_TIM1_CLKIN
	},
	{
		.base = STM32_TIM9_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM9EN,
		.vector	= STM32_IRQ_TIM9,
		.clock_freq = STM32_APB2_TIM9_CLKIN
	},
	{
		.base = STM32_TIM8_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM8EN,
		.vector	= STM32_IRQ_TIM8CC,
		.clock_freq = STM32_APB2_TIM8_CLKIN
	}
};

__EXPORT const struct pwm_input_channel pwm_input_channels[PWM_INPUT_MAX_CHANNELS] = {
	{
		.gpio = GPIO_TIM1_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
	},
	{
		.gpio = GPIO_TIM1_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
	},
	{
		.gpio = GPIO_TIM1_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
	},
	{
		.gpio = GPIO_TIM1_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
	},
	{
		.gpio = GPIO_TIM9_CH1IN,
		.timer_index = 1,
		.timer_channel = 1,
	},
	{
		.gpio = GPIO_TIM9_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
	},
	{
		.gpio = GPIO_TIM8_CH3IN,
		.timer_index = 2,
		.timer_channel = 3,
	},
	{
		.gpio = GPIO_TIM8_CH4IN,
		.timer_index = 2,
		.timer_channel = 4,
	}
};

#endif
