/****************************************************************************
 *
 *   Copyright (C) 2012 VRX Development Team. All rights reserved.
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
 * 3. Neither the name VRX nor the names of its contributors may be
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

/**
 * @file pwm_input.h
 *
 * PPM input decoder.
 */

#pragma once

#include <stdint.h>

#include <arch/board/board.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

/* configuration limits */
#define PWM_INPUT_MAX_TIMERS    4
#define PWM_INPUT_MAX_CHANNELS	8

/* array of timers dedicated to PWM input use */
struct pwm_input_timer {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	vector;
	uint32_t	clock_freq;
};

/* array of channels in logical order */
struct pwm_input_channel {
	uint32_t	gpio;
	uint8_t		timer_index;
	uint8_t		timer_channel;

};

/* supplied by board-specific code */
__EXPORT extern const struct pwm_input_timer pwm_input_timers[PWM_INPUT_MAX_TIMERS];
__EXPORT extern const struct pwm_input_channel pwm_input_channels[PWM_INPUT_MAX_CHANNELS];

__BEGIN_DECLS

__EXPORT extern int up_pwm_input_init(uint32_t channel_mask);

__END_DECLS

