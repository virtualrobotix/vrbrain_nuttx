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

#include <drivers/drv_led.h>

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

int test_led(int argc, char *argv[])
{
	int		fd;
	int		ret = 0;

	fd = open(LED_DEVICE_PATH, 0);

	if (fd < 0) {
		printf("\tLED: open fail\n");
		return ERROR;
	}

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
	if (ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_AMBER)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */

	int i;
	uint8_t ledon = 1;

	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_ON, LED_BLUE);
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_OFF, LED_BLUE);
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(60000);
	}

	/* Go back to default */
	ioctl(fd, LED_ON, LED_BLUE);
	ioctl(fd, LED_OFF, LED_AMBER);
	
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2) ||
	    ioctl(fd, LED_ON, LED_EXT3)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	ledon = 0;
	printf("\tTest LED EXT 3\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT3);

		} else {
			ioctl(fd, LED_ON, LED_EXT3);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT3);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2) ||
	    ioctl(fd, LED_ON, LED_EXT3)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	ledon = 0;
	printf("\tTest LED EXT 3\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT3);

		} else {
			ioctl(fd, LED_ON, LED_EXT3);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT3);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2) ||
	    ioctl(fd, LED_ON, LED_EXT3)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	ledon = 0;
	printf("\tTest LED EXT 3\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT3);

		} else {
			ioctl(fd, LED_ON, LED_EXT3);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT3);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2) ||
	    ioctl(fd, LED_ON, LED_EXT3)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	ledon = 0;
	printf("\tTest LED EXT 3\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT3);

		} else {
			ioctl(fd, LED_ON, LED_EXT3);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT3);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);
	
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2) ||
	    ioctl(fd, LED_ON, LED_EXT3)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	ledon = 0;
	printf("\tTest LED EXT 3\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT3);

		} else {
			ioctl(fd, LED_ON, LED_EXT3);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT3);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);
	ioctl(fd, LED_OFF, LED_EXT3);

#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);

#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN) ||
	    ioctl(fd, LED_ON, LED_EXT1) ||
	    ioctl(fd, LED_ON, LED_EXT2)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	ledon = 0;
	printf("\tTest LED EXT 1\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT1);

		} else {
			ioctl(fd, LED_ON, LED_EXT1);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT1);

	ledon = 0;
	printf("\tTest LED EXT 2\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_EXT2);

		} else {
			ioctl(fd, LED_ON, LED_EXT2);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_EXT2);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
	ioctl(fd, LED_OFF, LED_EXT1);
	ioctl(fd, LED_OFF, LED_EXT2);

#elif defined(CONFIG_ARCH_BOARD_VRHERO_V10)

	if (ioctl(fd, LED_ON, LED_AMBER) ||
	    ioctl(fd, LED_ON, LED_BLUE) ||
	    ioctl(fd, LED_ON, LED_GREEN)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);

	int i;
	uint8_t ledon = 0;

	printf("\tTest LED AMBER\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_AMBER);

	ledon = 0;
	printf("\tTest LED BLUE\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_BLUE);

		} else {
			ioctl(fd, LED_ON, LED_BLUE);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_BLUE);

	ledon = 0;
	printf("\tTest LED GREEN\n");
	for (i = 0; i < 10; i++) {
		if (ledon) {
			ioctl(fd, LED_OFF, LED_GREEN);

		} else {
			ioctl(fd, LED_ON, LED_GREEN);
		}

		ledon = !ledon;
		usleep(1000000);
	}
	ioctl(fd, LED_OFF, LED_GREEN);

	/* Go back to default */
	ioctl(fd, LED_OFF, LED_AMBER);
	ioctl(fd, LED_OFF, LED_BLUE);
	ioctl(fd, LED_OFF, LED_GREEN);
#endif

	printf("\t LED test completed, no errors.\n");

	return ret;
}
