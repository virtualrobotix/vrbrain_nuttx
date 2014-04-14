/****************************************************************************
 * px4/sensors/test_gpio.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <drivers/drv_buzzer.h>

#include "tests.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_led
 ****************************************************************************/

int test_buzzer(int argc, char *argv[])
{
	int		fd;
	int		ret = 0;

	fd = open(BUZZER_DEVICE_PATH, 0);

	if (fd < 0) {
		printf("\tBUZZER: open fail\n");
		return ERROR;
	}

	if (ioctl(fd, BUZZER_ON, BUZZER_EXT)) {

		printf("\tBUZZER: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, BUZZER_OFF, BUZZER_EXT);

	int i;
	uint8_t buzzeron = 0;

	for (i = 0; i < 10; i++) {
		if (buzzeron) {
			ioctl(fd, BUZZER_OFF, BUZZER_EXT);

		} else {
			ioctl(fd, BUZZER_ON, BUZZER_EXT);
		}

		buzzeron = !buzzeron;
		usleep(1000000);
	}
	ioctl(fd, BUZZER_OFF, BUZZER_EXT);

	/* Go back to default */
	ioctl(fd, BUZZER_OFF, BUZZER_EXT);

	printf("\t BUZZER test completed, no errors.\n");

	return ret;
}
