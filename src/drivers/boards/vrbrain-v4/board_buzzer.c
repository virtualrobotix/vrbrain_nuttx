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

/**
 * @file vrbrain_buzzer.c
 *
 * VRBRAIN BUZZER backend.
 */

#include <nuttx/config.h>

#include <stdbool.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

__BEGIN_DECLS
extern void buzzer_init();
extern void buzzer_on(int buzzer);
extern void buzzer_off(int buzzer);
extern void buzzer_toggle(int buzzer);
__END_DECLS

__EXPORT void buzzer_init()
{
	stm32_configgpio(GPIO_BUZZER);
}

__EXPORT void buzzer_on(int buzzer)
{
	if (buzzer == 0)
	{
		stm32_gpiowrite(GPIO_BUZZER, true);
	}
}

__EXPORT void buzzer_off(int buzzer)
{
	if (buzzer == 0)
	{
		stm32_gpiowrite(GPIO_BUZZER, false);
	}
}

__EXPORT void buzzer_toggle(int buzzer)
{
	if (buzzer == 0)
	{
		if (stm32_gpioread(GPIO_BUZZER))
			stm32_gpiowrite(GPIO_BUZZER, false);
		else
			stm32_gpiowrite(GPIO_BUZZER, true);
	}
}
