/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file vrinput.cpp
 * Driver for the VRBRAIN board.
 *
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <crc32.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>

#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>



#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>


#include <drivers/vrbrain/vrinput/controls/controls.h>










#include <uORB/topics/rc_channels.h>

























/**
 * The VRBRAIN class.
 *
 *
 */
class VRINPUT : public device::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	VRINPUT(device::Device *interface);

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~VRINPUT();

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	virtual int		init();

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 *
	 * @param disable_rc_handling set to true to forbid override / RC handling on IO
	 */
	int			init(bool disable_rc_handling);

	/**
	 * Detect if a PX4IO is connected.
	 *
	 * Only validate if there is a PX4IO to talk to.
	 */
	virtual int		detect();

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * write handler.
	 *
	 * Handle writes to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] buffer pointer to the data buffer to be written
	 * @param[in] len size in bytes to be written
	 * @return number of bytes written
	 */
	virtual ssize_t		write(file *filp, const char *buffer, size_t len);

	/**
	* Set the update rate for actuator outputs from FMU to IO.
	*
	* @param[in] rate		The rate in Hz actuator outpus are sent to IO.
	* 			Min 10 Hz, max 400 Hz
	*/
	int      		set_update_rate(int rate);

	/**
	* Set the battery current scaling and bias
	*
	* @param[in] amp_per_volt
	* @param[in] amp_bias
	*/
	void      		set_battery_current_scaling(float amp_per_volt, float amp_bias);

	/**
	 * Push failsafe values to IO.
	 *
	 * @param[in] vals	Failsafe control inputs: in us PPM (900 for zero, 1500 for centered, 2100 for full)
	 * @param[in] len	Number of channels, could up to 8
	 */
	int			set_failsafe_values(const uint16_t *vals, unsigned len);

	/**
	 * Disable RC input handling
	 */
	int			disable_rc_handling();

	/**
	 * Print IO status.
	 *
	 * Print all relevant IO status information
	 *
	 * @param extended_status Shows more verbose information (in particular RC config)
	 */
	void			print_status(bool extended_status);

	/**
	 * Fetch and print debug console output.
	 */
	int			print_debug();





















	inline uint16_t		system_status() const {return _status;}

private:
	device::Device		*_interface;

	// XXX
	unsigned		_hardware;		///< Hardware revision


	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO




	bool			_rc_handling_disabled;	///< If set, IO does not evaluate, but only forward the RC values
	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp

	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag







	/* cached IO state */
	uint16_t		_status;		///< Various IO status flags
	uint16_t		_alarms;		///< Various IO alarms

	/* subscribed topics */









	/* advertised topics */
	orb_advert_t 		_to_input_rc;		///< rc inputs from io





















	/**
	 * Trampoline to the worker task
	 */
	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Send controls for one group to IO
	 */
	int			io_set_control_state(unsigned group);

	/**
	 * Send all controls to IO
	 */
	int			io_set_control_groups();

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Push RC channel configuration to IO.
	 */
	int			io_set_rc_config();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int			io_get_status();

	/**
	 * Disable RC input handling
	 */
	int			io_disable_rc_handling();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(rc_input_values &input_rc);

	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the PWM servo outputs.
	 */
	int			io_publish_pwm_outputs();

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Send mixer definition text to IO
	 */
	int			mixer_send(const char *buf, unsigned buflen, unsigned retries = 3);

	/**
	 * Handle a status update from IO.
	 *
	 * Publish IO status information if necessary.
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
	 * Handle an alarm update from IO.
	 *
	 * Publish IO alarm information if necessary.
	 *
	 * @param alarm		The status register
	 */
	int			io_handle_alarms(uint16_t alarms);

	/**
	 * Handle issuing dsm bind ioctl to vrbrain.
	 *
	 * @param dsmMode	0:dsm2, 1:dsmx
	 */
	void			dsm_bind_ioctl(int dsmMode);

	/**
	 * Handle a battery update from IO.
	 *
	 * Publish IO battery information if necessary.
	 *
	 * @param vbatt		vbatt register
	 * @param ibatt		ibatt register
	 */
	void			io_handle_battery(uint16_t vbatt, uint16_t ibatt);

	/**
	 * Handle a servorail update from IO.
	 *
	 * Publish servo rail information if necessary.
	 *
	 * @param vservo	vservo register
	 * @param vrssi 	vrssi register
	 */
	void			io_handle_vservo(uint16_t vservo, uint16_t vrssi);

	/* do not allow to copy this class due to ptr data members */
	VRINPUT(const VRINPUT&);
	VRINPUT operator=(const VRINPUT&);
};

namespace
{

VRINPUT	*g_dev = nullptr;

}

VRINPUT::VRINPUT(device::Device *interface) :
	CDev("vrinput", VRINPUT_DEVICE_PATH),
	_interface(interface),
	_hardware(0),


	_max_rc_input(0),



	_rc_handling_disabled(false),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_task(-1),
	_task_should_exit(false),




	_status(0),
	_alarms(0),








	_to_input_rc(0)

















{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = false;

}

VRINPUT::~VRINPUT()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	if (_interface != nullptr)
		delete _interface;






	g_dev = nullptr;
}

int
VRINPUT::detect()
{
	int ret;

	if (_task == -1) {

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK)
			return ret;

		/* get some parameters */
		unsigned protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

		if (protocol != PX4IO_PROTOCOL_VERSION) {
			if (protocol == _io_reg_get_error) {
				log("IO not installed");

			} else {
				log("IO version error");

			}

			return -1;
		}
	}

	log("IO found");

	return 0;
}

int
VRINPUT::init(bool rc_handling_disabled) {
	_rc_handling_disabled = rc_handling_disabled;
	return init();
}

int
VRINPUT::init()
{
	int ret;



	ASSERT(_task == -1);







	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK)
		return ret;

	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();

	do {
		usleep(2000);
		protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */
	if (protocol == _io_reg_get_error) {
		log("Failed to communicate with IO");
		return -1;
	}

	if (protocol != PX4IO_PROTOCOL_VERSION) {
		log("IO protocol/firmware mismatch");
		return -1;
	}

	_hardware      = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION);




	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);




	if ((_max_rc_input < 1)  || (_max_rc_input > 255)) {

		log("config read error");

		return -1;
	}

	if (_max_rc_input > RC_INPUT_MAX_CHANNELS)
		_max_rc_input = RC_INPUT_MAX_CHANNELS;












































































































































		if (_rc_handling_disabled) {
			ret = io_disable_rc_handling();

			if (ret != OK) {
				log("failed disabling RC handling");
				return ret;
			}

		} else {
			/* publish RC config to IO */
			ret = io_set_rc_config();

			if (ret != OK) {
				log("IO RC config upload fail");
				return ret;
			}
		}




















	/* start the IO interface task */
	_task = task_spawn_cmd("vrinput",
					SCHED_DEFAULT,
					SCHED_PRIORITY_ACTUATOR_OUTPUTS,
					1800,
					(main_t)&VRINPUT::task_main_trampoline,
					nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
VRINPUT::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
VRINPUT::task_main()
{





	log("controls_init");
	controls_init();
	






























	/* lock against the ioctl handler */
	lock();

	/* loop talking to IO */
	while (!_task_should_exit) {

		controls_tick();

		usleep(1000);




































			/* pull status and alarms from IO */
			io_get_status();

			/* get raw R/C input from IO */
			io_publish_raw_rc();
























































































































	}

	unlock();


	debug("exiting");





	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
VRINPUT::io_set_control_groups()
{







	return 0;
}

int
VRINPUT::io_set_control_state(unsigned group)
{






















































	/* copy values to registers in IO */
	return 0;
}


int
VRINPUT::io_set_arming_state()
{





















































	return 0;
}

int
VRINPUT::disable_rc_handling()
{
	_rc_handling_disabled = true;
	return io_disable_rc_handling();
}

int
VRINPUT::io_disable_rc_handling()
{
	uint16_t set = PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED;
	uint16_t clear = 0;

	return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
}

int
VRINPUT::io_set_rc_config()
{



	int ret = OK;























































































































	return ret;
}

int
VRINPUT::io_handle_status(uint16_t status)
{
	int ret = 1;






















		ret = 0;

		/* set new status */
		_status = status;

























	return ret;
}

void
VRINPUT::dsm_bind_ioctl(int dsmMode)
{










}


int
VRINPUT::io_handle_alarms(uint16_t alarms)
{







	return 0;
}

void
VRINPUT::io_handle_battery(uint16_t vbatt, uint16_t ibatt)
{








































}

void
VRINPUT::io_handle_vservo(uint16_t vservo, uint16_t vrssi)
{













}

int
VRINPUT::io_get_status()
{
	uint16_t	regs;
	int		ret = OK;

	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI, STATUS_PRSSI
	 * in that order */
	regs = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS);




	io_handle_status(regs);










	return ret;
}

int
VRINPUT::io_get_raw_rc_input(rc_input_values &input_rc)
{
	uint32_t channel_count;
	int	ret = OK;

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;

	static const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	for (unsigned i = 0; i < prolog + 9; i++) {
		regs[i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT + i);
	}




	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > RC_INPUT_MAX_CHANNELS) {
		channel_count = RC_INPUT_MAX_CHANNELS;
	}

	_rc_chan_count = channel_count;

	input_rc.timestamp_publication = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];
	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp_publication;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		for (unsigned int i = 0; i < channel_count - 9; i++) {
			regs[prolog + 9 + i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9 + i);
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	return ret;
}

int
VRINPUT::io_publish_raw_rc()
{

	/* fetch values from IO */
	rc_input_values	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK)
		return ret;

	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = RC_INPUT_SOURCE_VRBRAIN_PPM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = RC_INPUT_SOURCE_VRBRAIN_SPEKTRUM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = RC_INPUT_SOURCE_VRBRAIN_SBUS;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_PWM) {
		rc_val.input_source = RC_INPUT_SOURCE_VRBRAIN_PWM;

	} else {
		rc_val.input_source = RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	/* lazily advertise on first publication */
	if (_to_input_rc == 0) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}

	return OK;
}

int
VRINPUT::io_publish_pwm_outputs()
{



























	return OK;
}

int
VRINPUT::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{






	registers_set(page, offset, values, num_values);






	return OK;
}

int
VRINPUT::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}




















uint32_t
VRINPUT::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t *regs;
	unsigned reg_count = 1;

	if (registers_get(page, offset, &regs, &reg_count) != 0)
		return _io_reg_get_error;

	return *regs;
}

int
VRINPUT::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{

	uint16_t value;

	value = io_reg_get(page, offset);




	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, value);
}

int
VRINPUT::print_debug()
{

	int io_fd = -1;

	if (io_fd <= 0) {
		io_fd = ::open("/dev/ttyS2", O_RDONLY | O_NONBLOCK | O_NOCTTY);
	}

	/* read IO's output */
	if (io_fd >= 0) {
		pollfd fds[1];
		fds[0].fd = io_fd;
		fds[0].events = POLLIN;

		usleep(500);
		int pret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 0);

		if (pret > 0) {
			int count;
			char buf[65];

			do {
				count = ::read(io_fd, buf, sizeof(buf) - 1);

				if (count > 0) {
					/* enforce null termination */
					buf[count] = '\0';
					warnx("IO CONSOLE: %s", buf);
				}

			} while (count > 0);
		}

		::close(io_fd);
		return 0;
	}


	return 1;

}

int
VRINPUT::mixer_send(const char *buf, unsigned buflen, unsigned retries)
{












































































































	/* load must have failed for some reason */
	return -EINVAL;
}

void
VRINPUT::print_status(bool extended_status)
{
	/* basic configuration */
	printf("protocol %u hardware %u bootloader %u buffer %uB crc 0x%04x%04x\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_BOOTLOADER_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC+1));
	printf("%u controls %u actuators %u R/C inputs %u analog inputs %u relays\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT));

	/* status */
	printf("%u bytes free\n",
	       io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FREEMEM));
	uint16_t flags = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS);
	uint16_t io_status_flags = flags;
	printf("status 0x%04x%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
	       flags,
	       ((flags & PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED) ? " OUTPUTS_ARMED" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) ? " SAFETY_OFF" : " SAFETY_SAFE"),
	       ((flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) ? " OVERRIDE" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_OK)    ? " RC_OK" : " RC_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_PPM)   ? " PPM" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_DSM)   ? " DSM" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_ST24)   ? " ST24" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_SBUS)  ? " SBUS" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_FMU_OK)   ? " FMU_OK" : " FMU_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)  ? " RAW_PWM_PASSTHROUGH" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) ? " MIXER_OK" : " MIXER_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_ARM_SYNC) ? " ARM_SYNC" : " ARM_NO_SYNC"),
	       ((flags & PX4IO_P_STATUS_FLAGS_INIT_OK)  ? " INIT_OK" : " INIT_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_FAILSAFE)  ? " FAILSAFE" : ""));
	uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);
	printf("alarms 0x%04x%s%s%s%s%s%s%s%s\n",
	       alarms,
	       ((alarms & PX4IO_P_STATUS_ALARMS_VBATT_LOW)     ? " VBATT_LOW" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_TEMPERATURE)   ? " TEMPERATURE" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_SERVO_CURRENT) ? " SERVO_CURRENT" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_ACC_CURRENT)   ? " ACC_CURRENT" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_FMU_LOST)      ? " FMU_LOST" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_RC_LOST)       ? " RC_LOST" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR)     ? " PWM_ERROR" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_VSERVO_FAULT)  ? " VSERVO_FAULT" : ""));
	/* now clear alarms */
	io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, 0xFFFF);






























	uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);
	printf("%d raw R/C inputs", raw_inputs);

	for (unsigned i = 0; i < raw_inputs; i++)
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i));

	printf("\n");

	flags = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_FLAGS);
	printf("R/C flags: 0x%04x%s%s%s%s%s\n", flags,
		(((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (!(flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11))) ? " DSM10" : ""),
		(((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11)) ? " DSM11" : ""),
		((flags & PX4IO_P_RAW_RC_FLAGS_FRAME_DROP) ? " FRAME_DROP" : ""),
		((flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) ? " FAILSAFE" : ""),
		((flags & PX4IO_P_RAW_RC_FLAGS_MAPPING_OK) ? " MAPPING_OK" : "")
	       );

	if ((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_PPM)) {
		int frame_len = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_DATA);
		printf("RC data (PPM frame len) %u us\n", frame_len);

		if ((frame_len - raw_inputs * 2000 - 3000) < 0) {
			printf("WARNING  WARNING  WARNING! This RC receiver does not allow safe frame detection.\n");
		}
	}

	uint16_t mapped_inputs = io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_VALID);
	printf("mapped R/C inputs 0x%04x", mapped_inputs);

	for (unsigned i = 0; i < _max_rc_input; i++) {
		if (mapped_inputs & (1 << i))
			printf(" %u:%d", i, REG_TO_SIGNED(io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_BASE + i)));
	}

	printf("\n");
	uint16_t adc_inputs = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT);
	printf("ADC inputs");

	for (unsigned i = 0; i < adc_inputs; i++)
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_ADC_INPUT, i));

	printf("\n");

	/* setup and state */
	uint16_t features = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES);
	printf("features 0x%04x%s%s%s%s\n", features,
		((features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) ? " S.BUS1_OUT" : ""),
		((features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) ? " S.BUS2_OUT" : ""),
		((features & PX4IO_P_SETUP_FEATURES_PWM_RSSI) ? " RSSI_PWM" : ""),
		((features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) ? " RSSI_ADC" : "")
		);
	uint16_t arming = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING);
	printf("arming 0x%04x%s%s%s%s%s%s%s%s%s%s\n",
	       arming,
	       ((arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)		? " FMU_ARMED" : " FMU_DISARMED"),
	       ((arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK)		? " IO_ARM_OK" : " IO_ARM_DENIED"),
	       ((arming & PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK)	? " MANUAL_OVERRIDE_OK" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM)		? " FAILSAFE_CUSTOM" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK)	? " INAIR_RESTART_OK" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)	? " ALWAYS_PWM_ENABLE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)		? " LOCKDOWN" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE)		? " FORCE_FAILSAFE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) ? " TERM_FAILSAFE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE) ? " OVERRIDE_IMMEDIATE" : "")
	       );























	if (extended_status) {
		for (unsigned i = 0; i < _max_rc_input; i++) {
			unsigned base = PX4IO_P_RC_CONFIG_STRIDE * i;
			uint16_t options = io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_OPTIONS);
			printf("input %u min %u center %u max %u deadzone %u assigned %u options 0x%04x%s%s\n",
			       i,
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MIN),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_CENTER),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MAX),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_DEADZONE),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_ASSIGNMENT),
			       options,
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) ? " ENABLED" : ""),
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_REVERSE) ? " REVERSED" : ""));
		}
	}












}

int
VRINPUT::ioctl(file * filep, int cmd, unsigned long arg)
{
	int ret = OK;


















































































































































































































































































































































































































































































































	return ret;
}

ssize_t
VRINPUT::write(file * /*filp*/, const char *buffer, size_t len)
/* Make it obvious that file * isn't used here */
{
	unsigned count = len / 2;














	return count * 2;
}

int
VRINPUT::set_update_rate(int rate)
{













	return 0;
}

void
VRINPUT::set_battery_current_scaling(float amp_per_volt, float amp_bias)
{


}

extern "C" __EXPORT int vrinput_main(int argc, char *argv[]);

namespace
{

device::Device *
get_interface()
{
	device::Device *interface = nullptr;




























	return interface;
}

void
start(int argc, char *argv[])
{
	if (g_dev != nullptr)
		errx(0, "already loaded");

	/* allocate the interface */
	device::Device *interface = get_interface();

	/* create the driver - it will set g_dev */
	(void)new VRINPUT(interface);

	if (g_dev == nullptr) {
		delete interface;
		errx(1, "driver alloc failed");
	}

	bool rc_handling_disabled = false;

	/* disable RC handling on request */
	if (argc > 1) {
		if (!strcmp(argv[1], "norc")) {

			rc_handling_disabled = true;

		} else {
			warnx("unknown argument: %s", argv[1]);
		}
	}

	if (OK != g_dev->init(rc_handling_disabled)) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
	int dsm_vcc_ctl;

	if (param_get(param_find("RC_RL1_DSM_VCC"), &dsm_vcc_ctl) == OK) {
		if (dsm_vcc_ctl) {
			g_dev->set_dsm_vcc_ctl(true);
			g_dev->ioctl(nullptr, DSM_BIND_POWER_UP, 0);
		}
	}

#endif
	exit(0);
}

void
detect(int argc, char *argv[])
{
























}

void
checkcrc(int argc, char *argv[])
{


























































}

void
bind(int argc, char *argv[])
{




































}

void
test(void)
{




















































































}

void
monitor(void)
{
	/* clear screen */
	printf("\033[2J");

	unsigned cancels = 2;

	for (;;) {
		pollfd fds[1];

		fds[0].fd = 0;
		fds[0].events = POLLIN;
		if (poll(fds, 1, 2000) < 0) {
			errx(1, "poll fail");
		}

		if (fds[0].revents == POLLIN) {
			/* control logic is to cancel with any key */
			char c;
			(void)read(0, &c, 1);

			if (cancels-- == 0) {
				printf("\033[2J\033[H"); /* move cursor home and clear screen */
				exit(0);
			}
		}

		if (g_dev != nullptr) {

			printf("\033[2J\033[H"); /* move cursor home and clear screen */
			(void)g_dev->print_status(false);
			(void)g_dev->print_debug();
			printf("\n\n\n[ Use 'VRINPUT debug <N>' for more output. Hit <enter> three times to exit monitor mode ]\n");

		} else {
			errx(1, "driver not loaded, exiting");
		}
	}
}

void
if_test(unsigned mode)
{











}

void
lockdown(int argc, char *argv[])
{

















































	exit(0);
}

} /* namespace */

int
vrinput_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2)
		goto out;

	if (!strcmp(argv[1], "start"))
		start(argc - 1, argv + 1);
























































































































	/* commands below here require a started driver */

	if (g_dev == nullptr)
		errx(1, "not started");





























































	if (!strcmp(argv[1], "stop")) {

		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}


	if (!strcmp(argv[1], "status")) {

		printf("[VRINPUT] loaded\n");
		g_dev->print_status(true);

		exit(0);
	}



























	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm"))
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);

	if (!strcmp(argv[1], "test"))
		test();

	if (!strcmp(argv[1], "monitor"))
		monitor();







	if (!strcmp(argv[1], "sbus1_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 1);

		if (ret != 0) {
			errx(ret, "S.BUS v1 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "sbus2_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 2);

		if (ret != 0) {
			errx(ret, "S.BUS v2 failed");
		}

		exit(0);
	}



























out:
	errx(1, "need a command, try 'start', 'stop', 'status', 'test', 'monitor', 'debug <level>',\n"
	        "'recovery', 'limit <rate>', 'current', 'bind', 'checkcrc', 'safety_on', 'safety_off',\n"
	        "'forceupdate', 'update', 'sbus1_out', 'sbus2_out', 'rssi_analog' or 'rssi_pwm'");
}
