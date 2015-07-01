/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file buzzer.cpp
 *
 * BUZZER driver.
 */

#include <px4_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_buzzer.h>

__BEGIN_DECLS
extern void buzzer_init();
extern void buzzer_on(int buzzer);
extern void buzzer_off(int buzzer);
extern void buzzer_toggle(int buzzer);
__END_DECLS

class BUZZER : device::CDev
{
public:
	BUZZER();
	virtual ~BUZZER();

	virtual int		init();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
};

BUZZER::BUZZER() :
	CDev("buzzer", BUZZER_DEVICE_PATH)
{
	init();
}

BUZZER::~BUZZER()
{
}

int
BUZZER::init()
{
	CDev::init();
	buzzer_init();

	return 0;
}

int
BUZZER::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case BUZZER_ON:
		buzzer_on(arg);
		break;

	case BUZZER_OFF:
		buzzer_off(arg);
		break;

	case BUZZER_TOGGLE:
		buzzer_toggle(arg);
		break;


	default:
		result = CDev::ioctl(filp, cmd, arg);
	}
	return result;
}

namespace
{
BUZZER	*gBUZZER;
}

void
drv_buzzer_start(void)
{
	if (gBUZZER == nullptr) {
		gBUZZER = new BUZZER;
		if (gBUZZER != nullptr)
			gBUZZER->init();
	}
}
