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
 * @file batt_mon.c
 * daemon application for battery monitoring and LED signalling
 * 
 * @author Matteo Murtas
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <nuttx/analog/adc.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_led.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#ifdef BATT_MON_USE_PARAMS
#include "systemlib/param/param.h"
#endif

#include "board_config.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int batt_mon_task;				/**< Handle of daemon task / thread */


#define BATT_MON_DEFAULT_PIN 	10
#define BATT_MON_DEFAULT_SCALER (5.7f * 3.3f) / 4096.0f
#define BATT_MON_DEFAULT_MULTIPLIER 1.75f
#define BATT_MON_DEFAULT_MIN 	3.2f


//alert and verbosity level
#define BATT_ALERT_NORMAL	0
#define BATT_ALERT_WARNING	1
#define BATT_ALERT_DANGER	2

#define BATT_ALERT_MAX		2
#define BATT_ALERT_DISABLE_VERBOSE	(BATT_ALERT_MAX + 1)

#ifdef BATT_MON_USE_PARAMS
PARAM_DEFINE_INT32(batt_mon_p, BATT_MON_DEFAULT_PIN);
PARAM_DEFINE_FLOAT(batt_mon_m, BATT_MON_DEFAULT_MULTIPLIER);
PARAM_DEFINE_FLOAT(batt_mon_t, BATT_MON_DEFAULT_MIN);
PARAM_DEFINE_INT32(batt_mon_v, BATT_ALERT_DISABLE_VERBOSE);
#endif

/**
 * daemon management function.
 */
__EXPORT int batt_mon_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int batt_mon_thread_main(int argc, char *argv[]);

int batt_mon_test(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

#ifdef BATT_MON_USE_PARAMS
int save_params(uint8_t pin, float converter, float min_batt_volt, int verboseLevel);
#endif

int decode_params(uint8_t *pin, float *converter, float *min_batt_volt, int * verboseLevel, int argc, char *argv[]);
float read_batt(uint8_t pin, float conv);
int batt_status(uint8_t pin, float converter, float min_batt_volt, int verboseLevel);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: batt_mon {start|stop|status|test} [-p <pin>][-m <multiplier>][-t <min voltage threshold>][-v <verbose level>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int batt_mon_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("batt_mon already running\n");
			/* this is not an error */
			exit(0);
		}

		//param_set_default_file("/fs/microsd/APM/batt_mon.par");

		thread_should_exit = false;
		batt_mon_task = task_spawn_cmd("batt_mon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 batt_mon_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "test")) {
		batt_mon_test(argc - 2, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}




int batt_mon_thread_main(int argc, char *argv[]) {

	warnx("[batt_mon] starting\n");
	thread_running = true;

	uint8_t pin = BATT_MON_DEFAULT_PIN;
	float converter = BATT_MON_DEFAULT_MULTIPLIER;
	float min_batt_volt = BATT_MON_DEFAULT_MIN;
	int verb_level = BATT_ALERT_DISABLE_VERBOSE;

	decode_params(&pin, &converter, &min_batt_volt, &verb_level, argc, argv);
	warnx("pin: %u  mult: %2.2f  thresh: %2.2f\n", pin, (double) converter, (double) min_batt_volt);

	while (!thread_should_exit) {

		led_on(LED_RED);
		led_on(LED_GREEN);
		led_on(LED_BLUE);
		usleep(800000);

		int level = batt_status(pin, converter, min_batt_volt, verb_level);
		if (level == BATT_ALERT_NORMAL)
		{
		  led_off(LED_RED);
			led_on(LED_GREEN);
			led_off(LED_BLUE);
		}
		else if (level == BATT_ALERT_WARNING)
		{
		  led_on(LED_RED);
		  led_on(LED_GREEN);
		  led_off(LED_BLUE);
		} else {
		  led_on(LED_RED);
		  led_off(LED_GREEN);
		  led_off(LED_BLUE);
		}
		usleep(200000);
	}

	led_on(LED_RED);
	led_on(LED_GREEN);
	led_on(LED_BLUE);
	warnx("[batt_mon] exiting.\n");

	thread_running = false;

	return 0;
}

int batt_mon_test(int argc, char *argv[]) {

	//ci metto un errore per vedere se compila

	warnx("[batt_mon] test\n");

	uint8_t pin = BATT_MON_DEFAULT_PIN;
	float converter = BATT_MON_DEFAULT_MULTIPLIER;
	float min_batt_volt = BATT_MON_DEFAULT_MIN;
	int verb_level = BATT_ALERT_NORMAL;

	decode_params(&pin, &converter, &min_batt_volt, &verb_level, argc, argv);
	warnx("pin: %u  mult: %2.2f  thresh: %2.2f\n", pin, (double) converter, (double) min_batt_volt);

	int level = batt_status(pin, converter, min_batt_volt, verb_level);

	warnx("[batt_mon] test end.\n");

	return 0;
}

int batt_status(uint8_t pin, float converter, float min_batt_volt, int verboseLevel)
{
	int ret = 0;

	float batt_v = read_batt(pin, converter);
	if (batt_v > (1.1f * min_batt_volt))
	{
		ret = BATT_ALERT_NORMAL;
		if (ret >= verboseLevel)
			warnx("#BATTERY_OK#");
	} else if (batt_v > min_batt_volt)
	{
		ret = BATT_ALERT_WARNING;
		if (ret >= verboseLevel)
			warnx("#BATTERY_WARNING#");
	} else {
		ret = BATT_ALERT_DANGER;
		if (ret >= verboseLevel)
			warnx("#BATTERY_DANGER#");
	}
	if (ret >= verboseLevel)
		warnx(" BattV = %2.2f \r\n", (double)batt_v);


	return ret;
}

#ifdef BATT_MON_USE_PARAMS

int32_t read_param_int(const char * name, int32_t def)
{
	int32_t v = def;
	param_t		p;
	p = param_find(name);
	if (p != PARAM_INVALID)
	{
		param_type_t t = param_type(p);
		if (t == PARAM_TYPE_INT32)
		{
			int32_t	val;
			if (param_get(p, &val) != 0)
				v = val;
		}
	}
	return v;
}

float read_param_float(const char * name, float def)
{
	float v = def;
	param_t		p;
	p = param_find(name);
	if (p != PARAM_INVALID)
	{
		param_type_t t = param_type(p);
		if (t == PARAM_TYPE_FLOAT)
		{
			float	val;
			if (param_get(p, &val) != 0)
				v = val;
		}
	}
	return v;
}


int32_t save_param_int(const char * name, int32_t def)
{
	int32_t v = def;
	param_t		p;
	p = param_find(name);
	if (p != PARAM_INVALID)
	{
		param_type_t t = param_type(p);
		if (t == PARAM_TYPE_INT32)
		{
			param_set(p, &v);
		}
	}
	return v;
}

float save_param_float(const char * name, float def)
{
	float v = def;
	param_t		p;
	p = param_find(name);
	if (p != PARAM_INVALID)
	{
		param_type_t t = param_type(p);
		if (t == PARAM_TYPE_FLOAT)
		{
			param_set(p, &v);
		}
	}
	return v;
}

int save_params(uint8_t pin, float converter, float min_batt_volt, int verboseLevel)
{
	save_param_int("batt_mon_p", (int32_t) pin);
	save_param_float("batt_mon_m", (float) converter);
	save_param_float("batt_mon_t", (float) min_batt_volt);
	save_param_int("batt_mon_v", (int32_t) verboseLevel);
}
#endif

int decode_params(uint8_t *pin, float *converter, float *min_batt_volt, int * verboseLevel, int argc, char *argv[]) {

	*pin = BATT_MON_DEFAULT_PIN;
	*converter = BATT_MON_DEFAULT_MULTIPLIER;
	*min_batt_volt = BATT_MON_DEFAULT_MIN;

#ifdef BATT_MON_USE_PARAMS

	//provo a leggere i parametri dal file dei parametri
	*pin = (uint8_t) read_param_int("batt_mon_p", (int32_t) (*pin));
	*converter = (float) read_param_float("batt_mon_m", (float) (*converter));
	*min_batt_volt = (float) read_param_float("batt_mon_t", (float) (*min_batt_volt));

	if (verboseLevel < 0)
	{
		//voglio che venga reimpostato il valore di default
		*verboseLevel = BATT_ALERT_DISABLE_VERBOSE;
		*verboseLevel = (int) read_param_int("batt_mon_v", (int32_t) (*verboseLevel));
	}

#endif

	//decodifico i parametri
	int ch;
	char *ep;
	float mult = 0.0f;
	while ((ch = getopt(argc, argv, "p:m:t:v:")) != EOF) {
		switch (ch) {
		case 'p':
			*pin = (uint8_t) strtoul(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad pin value");
			break;
		case 'm':
			mult = (float) strtod(optarg, &ep);
			if (*ep != '\0')
				usage("bad multiplier value");
			else
				*converter = mult;
			break;

		case 't':
			*min_batt_volt = (float) strtod(optarg, &ep);
			if (*ep != '\0')
				usage("bad threshold value");
			break;
		case 'v':
			*verboseLevel = (uint8_t) strtol(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad verbose level value");
			break;

		}
	}
	argc -= optind;
	argv += optind;

	return 0;
}

float read_batt(uint8_t pin, float conv)
{
	float ret = 0.0f;

	int fd = open(ADC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		//warnx("ERROR: can't open ADC device");
		return -1.0f;
	}
	/* make space for a maximum of eight channels */
	struct adc_msg_s data[8];
	/* read all channels available */
	ssize_t count = read(fd, data, sizeof(data));

	if (count < 0)
		goto errout_with_dev;

	unsigned channels = count / sizeof(data[0]);
    bool bFound =  false;
	for (unsigned j = 0; j < channels; j++) {
		//printf("%d: %u  ", data[j].am_channel, data[j].am_data);
		if (data[j].am_channel == pin)
		{
			bFound = true;
			//printf("<-- ");
			ret = conv * (float) (data[j].am_data) * BATT_MON_DEFAULT_SCALER; // / 4096.0f;
		}
	}

	//printf("\n");

	//if (bFound)
	//	warnx("\t read_batt successful.\n");

errout_with_dev:

	if (fd != 0) close(fd);

	return ret;
}
