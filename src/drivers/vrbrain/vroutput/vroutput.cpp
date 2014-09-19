/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file vroutput.cpp
 *
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/board_serial.h>

#include <drivers/drv_rc_input.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>






/*
 * This is the analog to FMU_INPUT_DROP_LIMIT_US on the IO side
 */

#define CONTROL_INPUT_DROP_LIMIT_MS		20

class VROUTPUT : public device::CDev
{
public:
	enum Mode {
		MODE_NONE,
		MODE_2PWM,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_10PWM,
		MODE_12PWM
	};
	VROUTPUT();
	virtual ~VROUTPUT();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

private:






#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
	static const unsigned _max_actuators = 8;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
	static const unsigned _max_actuators = 12;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
	static const unsigned _max_actuators = 8;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
	static const unsigned _max_actuators = 12;
#endif
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
	static const unsigned _max_actuators = 8;
#endif
#if defined(CONFIG_ARCH_BOARD_VRHERO_V10)
	static const unsigned _max_actuators = 4;
#endif

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	int		_task;
	int		_t_actuators;
	int		_t_actuator_armed;
	orb_advert_t	_t_outputs;
	unsigned	_num_outputs;
	bool		_primary_pwm_device;

	volatile bool	_task_should_exit;
	bool		_armed;
	bool		_pwm_on;



	actuator_controls_s _controls;

	pwm_limit_t	_pwm_limit;
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main() __attribute__((noreturn));

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		sensor_reset(int ms);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

};

const VROUTPUT::GPIOConfig VROUTPUT::_gpio_tab[] = {

























#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
#endif
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
#endif
#if defined(CONFIG_ARCH_BOARD_VRHERO_V10)
	{GPIO_SHUTDOWN_INT, 0,			   0},
#ifdef AUX_IO1_OUT
	{0,       			GPIO_AUX_IO1, 0},
#else
	{GPIO_AUX_IO1,      0 			, 0},
#endif
#ifdef AUX_IO2_OUT
	{0,       			GPIO_AUX_IO2, 0},
#else
	{GPIO_AUX_IO2,      0 			, 0},
#endif
#ifdef AUX_IO3_OUT
	{0,       			GPIO_AUX_IO3, 0},
#else
	{GPIO_AUX_IO3,      0 			, 0},
#endif
#ifdef AUX_IO4_OUT
	{0,       			GPIO_AUX_IO4, 0},
#else
	{GPIO_AUX_IO4,      0 			, 0},
#endif
#ifdef AUX_IO5_OUT
	{0,       			GPIO_AUX_IO5, 0},
#else
	{GPIO_AUX_IO5,      0 			, 0},
#endif
#ifdef AUX_IO6_OUT
  {0,             GPIO_AUX_IO6, 0},
#else
  {GPIO_AUX_IO6,      0       , 0},
#endif
#endif
};

const unsigned VROUTPUT::_ngpio = sizeof(VROUTPUT::_gpio_tab) / sizeof(VROUTPUT::_gpio_tab[0]);

namespace
{

VROUTPUT	*g_dev;

} // namespace

VROUTPUT::VROUTPUT() :
	CDev("vroutput", VROUTPUT_DEVICE_PATH),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_task(-1),
	_t_actuators(-1),
	_t_actuator_armed(-1),
	_t_outputs(0),
	_num_outputs(0),
	_primary_pwm_device(false),
	_task_should_exit(false),
	_armed(false),
	_pwm_on(false),

	_failsafe_pwm({0}),
	      _disarmed_pwm({0}),
	      _num_failsafe_set(0),
	      _num_disarmed_set(0)
{
	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
	}

	_debug_enabled = false;
}

VROUTPUT::~VROUTPUT()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	g_dev = nullptr;
}

int
VROUTPUT::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK)
		return ret;

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		log("default PWM output device");
		_primary_pwm_device = true;
	}

	/* reset GPIOs */
	gpio_reset();

	/* start the IO interface task */
	_task = task_spawn_cmd("vroutput",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_DEFAULT,
			       2048,
			       (main_t)&VROUTPUT::task_main_trampoline,
			       nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
VROUTPUT::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

int
VROUTPUT::set_mode(Mode mode)
{
	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		debug("MODE_2PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0x3);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_4PWM: // v1 multi-port as 4 PWM outs
		debug("MODE_4PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0xf);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_6PWM: // v2 PWMs as 6 PWM outs
		debug("MODE_6PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0x3f);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_8PWM: // v2 PWMs as 8 PWM outs
		debug("MODE_8PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0xff);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;
	case MODE_10PWM: // v2 PWMs as 8 PWM outs
		debug("MODE_12PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0x3ff);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;
	case MODE_12PWM: // v2 PWMs as 8 PWM outs
		debug("MODE_12PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0xfff);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;
	case MODE_NONE:
		debug("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;

		/* disable servo outputs - no need to set rates */
		up_pwm_servo_deinit();

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
VROUTPUT::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	debug("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0)
				continue;

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

int
VROUTPUT::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
VROUTPUT::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

void
VROUTPUT::task_main()
{
	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));
	/* force a reset of the update rate */
	_current_update_rate = 0;

	_t_actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_actuator_armed, 200);		/* 5Hz update rate */

	/* advertise the mixed control outputs */
	actuator_outputs_s outputs;
	memset(&outputs, 0, sizeof(outputs));
	/* advertise the mixed control outputs */
	_t_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &outputs);

	pollfd fds[2];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_actuator_armed;
	fds[1].events = POLLIN;










	/* initialize PWM limit lib */
	pwm_limit_init(&_pwm_limit);

	log("starting");

	/* loop until killed */
	while (!_task_should_exit) {

		/*
		 * Adjust actuator topic update rate to keep up with
		 * the highest servo update rate configured.
		 *
		 * We always mix at max rate; some channels may update slower.
		 */
		unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

		if (_current_update_rate != max_rate) {
			_current_update_rate = max_rate;
			int update_rate_in_ms = int(1000 / _current_update_rate);

			/* reject faster than 500 Hz updates */
			if (update_rate_in_ms < 2) {
				update_rate_in_ms = 2;
			}

			/* reject slower than 10 Hz updates */
			if (update_rate_in_ms > 100) {
				update_rate_in_ms = 100;
			}

			debug("adjusted actuator update interval to %ums", update_rate_in_ms);
			orb_set_interval(_t_actuators, update_rate_in_ms);

			// set to current max rate, even if we are actually checking slower/faster
			_current_update_rate = max_rate;
		}

		/* sleep waiting for data, stopping to check for PPM
		 * input at 100Hz */
		int ret = ::poll(&fds[0], 2, CONTROL_INPUT_DROP_LIMIT_MS);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			usleep(1000000);
			continue;

		} else if (ret == 0) {
			/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe");

		} else {

			/* do we have a control update? */
			if (fds[0].revents & POLLIN) {

				/* get controls - must always do this to avoid spinning */
				orb_copy(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS : ORB_ID(actuator_controls_1), _t_actuators, &_controls);
























































			}

			/* how about an arming update? */
			if (fds[1].revents & POLLIN) {
				actuator_armed_s aa;

				/* get new value */
				orb_copy(ORB_ID(actuator_armed), _t_actuator_armed, &aa);

				/* update the armed status and check that we're not locked down */
				bool set_armed = aa.armed && !aa.lockdown;

				if (_armed != set_armed)
					_armed = set_armed;

				/* update PWM status if armed or if disarmed PWM values are set */
				bool pwm_on = (aa.armed || _num_disarmed_set > 0);

				if (_pwm_on != pwm_on) {
					_pwm_on = pwm_on;
					up_pwm_servo_arm(pwm_on);
				}
			}
		}





































	}

	::close(_t_actuators);
	::close(_t_actuator_armed);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	log("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
VROUTPUT::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

int
VROUTPUT::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	// XXX disabled, confusing users
	//debug("ioctl 0x%04x 0x%08x", cmd, arg);

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY)
		return ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
	case MODE_6PWM:
	case MODE_8PWM:
	case MODE_10PWM:
	case MODE_12PWM:
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		debug("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY)
		ret = CDev::ioctl(filp, cmd, arg);

	return ret;
}

int
VROUTPUT::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0)
					_num_failsafe_set++;
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0)
					_num_disarmed_set++;
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}
	case PWM_SERVO_SET(11):
	case PWM_SERVO_SET(10):
		if (_mode < MODE_12PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(9):
	case PWM_SERVO_SET(8):
		if (_mode < MODE_10PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(7):
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
	case PWM_SERVO_SET(2):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;
	case PWM_SERVO_GET(11):
	case PWM_SERVO_GET(10):
		if (_mode < MODE_12PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(9):
	case PWM_SERVO_GET(8):
		if (_mode < MODE_10PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:

		switch (_mode) {
		case MODE_12PWM:
			*(unsigned *)arg = 12;
			break;
		case MODE_10PWM:
			*(unsigned *)arg = 10;
			break;
		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;

		case MODE_4PWM:
			*(unsigned *)arg = 4;
			break;

		case MODE_2PWM:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_COUNT: {
		/* change the number of outputs that are enabled for
		 * PWM. This is used to change the split between GPIO
		 * and PWM under control of the flight config
		 * parameters. Note that this does not allow for
		 * changing a set of pins to be used for serial on
		 * FMUv1 
		 */
		switch (arg) {
		case 0:
			set_mode(MODE_NONE);
			break;

		case 2:
			set_mode(MODE_2PWM);
			break;

		case 4:
			set_mode(MODE_4PWM);
			break;


		case 6:
			set_mode(MODE_6PWM);
			break;



		case 8:
			set_mode(MODE_8PWM);
			break;

		case 10:
			set_mode(MODE_10PWM);
			break;

		case 12:
			set_mode(MODE_12PWM);
			break;

		default:
			ret = -EINVAL;
			break;
		}
		break;
	}























































	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
VROUTPUT::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[6];

	if (count > 6) {
		// we have at most 6 outputs
		count = 6;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		up_pwm_servo_set(i, values[i]);
	}

	return count * 2;
}

void
VROUTPUT::sensor_reset(int ms)
{











































































}


void
VROUTPUT::gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, input if
	 * possible otherwise output if possible.
	 */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (_gpio_tab[i].input != 0) {
			stm32_configgpio(_gpio_tab[i].input);

		} else if (_gpio_tab[i].output != 0) {
			stm32_configgpio(_gpio_tab[i].output);
		}
	}






}

void
VROUTPUT::gpio_set_function(uint32_t gpios, int function)
{
















	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (gpios & (1 << i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				stm32_configgpio(_gpio_tab[i].input);
				break;

			case GPIO_SET_OUTPUT:
				stm32_configgpio(_gpio_tab[i].output);
				break;

			case GPIO_SET_ALT_1:
				if (_gpio_tab[i].alt != 0)
					stm32_configgpio(_gpio_tab[i].alt);

				break;
			}
		}
	}








}

void
VROUTPUT::gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (gpios & (1 << i))
			stm32_gpiowrite(_gpio_tab[i].output, value);
}

uint32_t
VROUTPUT::gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (stm32_gpioread(_gpio_tab[i].input))
			bits |= (1 << i);

	return bits;
}

int
VROUTPUT::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SENSOR_RAIL_RESET:
		sensor_reset(arg);
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		gpio_set_function(arg, cmd);
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = gpio_read();
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
};

PortMode g_port_mode;

int
vroutput_new_mode(PortMode new_mode)
{
	uint32_t gpio_bits;
	VROUTPUT::Mode servo_mode;

	/* reset to all-inputs */
	g_dev->ioctl(0, GPIO_RESET, 0);

	gpio_bits = 0;
	servo_mode = VROUTPUT::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		/* nothing more to do here */
		break;

	case PORT_FULL_PWM:







#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
		/* select 8-pin PWM mode */
		servo_mode = VROUTPUT::MODE_8PWM;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
		/* select 12-pin PWM mode */
		servo_mode = VROUTPUT::MODE_12PWM;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
		/* select 8-pin PWM mode */
		servo_mode = VROUTPUT::MODE_8PWM;
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
		/* select 8-pin PWM mode */
		servo_mode = VROUTPUT::MODE_12PWM;
#endif
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
		/* select 8-pin PWM mode */
		servo_mode = VROUTPUT::MODE_8PWM;
#endif
#if defined(CONFIG_ARCH_BOARD_VRHERO_V10)
		/* select 4-pin PWM mode */
		servo_mode = VROUTPUT::MODE_4PWM;
#endif
		break;

		/* mixed modes supported on v1 board only */

























	default:
		return -1;
	}

	/* adjust GPIO config for serial mode(s) */
	if (gpio_bits != 0)
		g_dev->ioctl(0, GPIO_SET_ALT_1, gpio_bits);

	/* (re)set the PWM output mode */
	g_dev->set_mode(servo_mode);

	return OK;
}

int
vroutput_start(void)
{
	int ret = OK;

	if (g_dev == nullptr) {

		g_dev = new VROUTPUT;

		if (g_dev == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_dev->init();

			if (ret != OK) {
				delete g_dev;
				g_dev = nullptr;
			}
		}
	}

	return ret;
}

int
vroutput_stop(void)
{
	int ret = OK;

	if (g_dev != nullptr) {

		delete g_dev;
		g_dev = nullptr;
	}

	return ret;
}

void
sensor_reset(int ms)
{
	int	 fd;

	fd = open(VROUTPUT_DEVICE_PATH, O_RDWR);

	if (fd < 0)
		errx(1, "open fail");

	if (ioctl(fd, GPIO_SENSOR_RAIL_RESET, ms) < 0)
		err(1, "servo arm failed");

}

void
test(void)
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int	 ret;

	fd = open(VROUTPUT_DEVICE_PATH, O_RDWR);

	if (fd < 0)
		errx(1, "open fail");

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0)       err(1, "servo arm failed");

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
	}

	warnx("Testing %u servos", (unsigned)servo_count);

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++)
			servos[i] = pwm_value;

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					err(1, "servo %u set failed", i);
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos))
				err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value))
				err(1, "error reading PWM servo %d", i);

			if (value != servos[i])
				errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				break;
			}
		}
	}

	close(fd);

	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5)
		errx(1, "vroutput fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle < 0)
		errx(1, "advertise failed");

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle < 0)
		errx(1, "advertise failed 2");

	exit(0);
}

} // namespace

extern "C" __EXPORT int vroutput_main(int argc, char *argv[]);

int
vroutput_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	if (!strcmp(verb, "stop")) {
		vroutput_stop();
		errx(0, "FMU driver stopped");
	}

	if (!strcmp(verb, "id")) {
		uint8_t id[12];
		(void)get_board_serial(id);

		errx(0, "Board serial:\n %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
		     (unsigned)id[0], (unsigned)id[1], (unsigned)id[2], (unsigned)id[3], (unsigned)id[4], (unsigned)id[5],
		     (unsigned)id[6], (unsigned)id[7], (unsigned)id[8], (unsigned)id[9], (unsigned)id[10], (unsigned)id[11]);
	}


	if (vroutput_start() != OK)
		errx(1, "failed to start the FMU driver");

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;















	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode)
			return OK;

		/* switch modes */
		int ret = vroutput_new_mode(new_mode);
		exit(ret == OK ? 0 : 1);
	}

	if (!strcmp(verb, "test"))
		test();

	if (!strcmp(verb, "fake"))
		fake(argc - 1, argv + 1);

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			sensor_reset(reset_time);

		} else {
			sensor_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}


	fprintf(stderr, "FMU: unrecognised command %s, try:\n", verb);




#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
#if defined(CONFIG_ARCH_BOARD_VRHERO_V10)
	fprintf(stderr, "  mode_gpio, mode_pwm, test\n");
#endif
	exit(1);
}
