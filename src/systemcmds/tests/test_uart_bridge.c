/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file test_uart_bridge.c
 * Tests the uart outputs
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "tests.h"

#include <math.h>
#include <float.h>

#include <termios.h>

static int set_baudrate(int fd, unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	printf("try baudrate: %d\n", speed);

	default:
		printf("ERROR: Unsupported baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		printf("ERROR setting config: %d (cfsetispeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		printf("ERROR setting config: %d (cfsetospeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		printf("ERROR setting baudrate (tcsetattr)\n");
		return -1;
	}

	/* XXX if resetting the parser here, ensure it does exist (check for null pointer) */
	return 0;
}

int test_uart_bridge(int argc, char *argv[])
{
	char *device_uart = "/dev/ttyS0";
	unsigned baudrate_uart = 38400;
	char *device_usb = "/dev/ttyACM0";

	if (argc > 2) {
		if (!strcmp(argv[1], "-a")) {
			device_uart = argv[2];
		} else {
			printf("tests uart_bridge -a /dev/ttyS0 -b 38400 -c /dev/ttyACM0\n");
			return 0;
		}
	}

	if (argc > 4) {
		if (!strcmp(argv[3], "-b")) {
			if (!strcmp(argv[4], "9600")) {
				baudrate_uart = 9600;
			} else if (!strcmp(argv[4], "19200")) {
				baudrate_uart = 19200;
			} else if (!strcmp(argv[4], "38400")) {
				baudrate_uart = 38400;
			} else if (!strcmp(argv[4], "57600")) {
				baudrate_uart = 57600;
			} else if (!strcmp(argv[4], "115200")) {
				baudrate_uart = 115200;
			} else {
				printf("tests uart_bridge -a /dev/ttyS0 -b 38400 -c /dev/ttyACM0\n");
				return 0;
			}
		} else {
			printf("tests uart_bridge -a /dev/ttyS0 -b 38400 -c /dev/ttyACM0\n");
			return 0;
		}
	}

	if (argc > 6) {
		if (!strcmp(argv[5], "-c")) {
			device_usb = argv[6];
		} else {
			printf("tests uart_bridge -a /dev/ttyS0 -b 38400 -c /dev/ttyACM0\n");
			return 0;
		}
	}

	printf("Opening %s at %u\n", device_uart, baudrate_uart);
	int uart = open(device_uart, O_RDWR | O_NONBLOCK | O_NOCTTY);
	set_baudrate(uart, baudrate_uart);

	printf("Opening %s \n", device_usb);
	int uartusb = open(device_usb, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (uart < 0) {
		printf("ERROR opening %s, aborting..\n", device_uart);
		return uart;
	}

	if (uartusb < 0) {
		printf("ERROR opening %s, aborting..\n", device_usb);
		exit(uartusb);
	}

	int r;

	while (1) {
		uint8_t sample_stdout[1];

		r = read(uart, sample_stdout, 1);
		if (r > 0)
			write(uartusb, sample_stdout, 1);

		r = read(uartusb, sample_stdout, 1);
		if (r > 0)
			write(uart, sample_stdout, 1);
	}

	close(uart);
	close(uartusb);

	return 0;
}
