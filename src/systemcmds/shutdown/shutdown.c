/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file shutdown.c
 * Tool similar to UNIX reboot command
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/config.h>

#include "board_config.h"

__EXPORT int shutdown_main(int argc, char *argv[]);

int shutdown_main(int argc, char *argv[])
{
	//stm32_gpiowrite(GPIO_SHUTDOWN, 0);

	bool bKeepOn = false;
/*
	int ch;
	while ((ch = getopt(argc, argv, "ny")) != EOF) {
		switch (ch) {
		case 'n':
			bKeepOn = true;
			break;
		case 'y':
			break;
		}
	}*/

	if (argc >= 2)
	{
		if (strcmp(argv[1], "-n") == 0)
			bKeepOn = true;
	}

	stm32_gpiowrite(GPIO_SHUTDOWN, bKeepOn);

	if (bKeepOn)
		printf("SHDN DISABLED\n");
	else
		printf("SHDN ENABLED\n");

	exit(0);
}


