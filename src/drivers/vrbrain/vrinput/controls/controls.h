/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file px4io.h
 *
 * General defines and structures for the PX4IO module firmware.
 */

#ifndef CONTROLS_H_
#define CONTROLS_H_

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <board_config.h>

#include "protocol.h"

#include <systemlib/pwm_limit/pwm_limit.h>

__BEGIN_DECLS

/*
 * Constants and limits.
 */
#define PX4IO_SERVO_COUNT			0
#define PX4IO_CONTROL_CHANNELS		0
#define PX4IO_CONTROL_GROUPS		0
#define PX4IO_RC_INPUT_CHANNELS		18
#define PX4IO_RC_MAPPED_CONTROL_CHANNELS		8 /**< This is the maximum number of channels mapped/used */

/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/*
 * Registers.
 */
__EXPORT extern uint16_t			r_page_status[];	/* PX4IO_PAGE_STATUS */
__EXPORT extern uint16_t			r_page_actuators[];	/* PX4IO_PAGE_ACTUATORS */
__EXPORT extern uint16_t			r_page_servos[];	/* PX4IO_PAGE_SERVOS */
__EXPORT extern uint16_t			r_page_raw_rc_input[];	/* PX4IO_PAGE_RAW_RC_INPUT */
__EXPORT extern uint16_t			r_page_rc_input[];	/* PX4IO_PAGE_RC_INPUT */
__EXPORT extern uint16_t			r_page_adc[];		/* PX4IO_PAGE_RAW_ADC_INPUT */

__EXPORT extern volatile uint16_t	r_page_setup[];		/* PX4IO_PAGE_SETUP */
__EXPORT extern volatile uint16_t	r_page_controls[];	/* PX4IO_PAGE_CONTROLS */
__EXPORT extern uint16_t			r_page_rc_input_config[]; /* PX4IO_PAGE_RC_INPUT_CONFIG */
__EXPORT extern uint16_t			r_page_servo_failsafe[]; /* PX4IO_PAGE_FAILSAFE_PWM */
__EXPORT extern uint16_t			r_page_servo_control_min[]; /* PX4IO_PAGE_CONTROL_MIN_PWM */
__EXPORT extern uint16_t			r_page_servo_control_max[]; /* PX4IO_PAGE_CONTROL_MAX_PWM */
__EXPORT extern uint16_t			r_page_servo_disarmed[];	/* PX4IO_PAGE_DISARMED_PWM */

/*
 * Register aliases.
 *
 * Handy aliases for registers that are widely used.
 */
#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]

#define r_raw_rc_count		r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT]
#define r_raw_rc_values		(&r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE])
#define r_raw_rc_flags		r_page_raw_rc_input[PX4IO_P_RAW_RC_FLAGS]
#define r_rc_valid		r_page_rc_input[PX4IO_P_RC_VALID]
#define r_rc_values		(&r_page_rc_input[PX4IO_P_RC_BASE])

#define r_setup_features	r_page_setup[PX4IO_P_SETUP_FEATURES]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]
#define r_setup_pwm_rates	r_page_setup[PX4IO_P_SETUP_PWM_RATES]
#define r_setup_pwm_defaultrate	r_page_setup[PX4IO_P_SETUP_PWM_DEFAULTRATE]
#define r_setup_pwm_altrate	r_page_setup[PX4IO_P_SETUP_PWM_ALTRATE]
#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
#define r_setup_relays		r_page_setup[PX4IO_P_SETUP_RELAYS]
#endif
#define r_setup_rc_thr_failsafe	r_page_setup[PX4IO_P_SETUP_RC_THR_FAILSAFE_US]

#define r_control_values	(&r_page_controls[0])

/*
 * System state structure.
 */
struct sys_state_s {

	volatile uint64_t	rc_channels_timestamp_received;
	volatile uint64_t	rc_channels_timestamp_valid;

	/**
	 * Last FMU receive time, in microseconds since system boot
	 */
	volatile uint64_t	fmu_data_received_time;

};

__EXPORT extern struct sys_state_s system_state;

/*
 * PWM limit structure
 */
__EXPORT extern pwm_limit_t pwm_limit;

/*
 * GPIO handling.
 */
#define LED_BLUE(_s)			stm32_gpiowrite(GPIO_LED1, !(_s))
#define LED_AMBER(_s)			stm32_gpiowrite(GPIO_LED2, !(_s))
#define LED_SAFETY(_s)			stm32_gpiowrite(GPIO_LED3, !(_s))
#define LED_RING(_s)			stm32_gpiowrite(GPIO_LED4, (_s))

#ifdef CONFIG_ARCH_BOARD_PX4IO_V1

# define PX4IO_RELAY_CHANNELS		4
# define POWER_SERVO(_s)		stm32_gpiowrite(GPIO_SERVO_PWR_EN, (_s))
# define POWER_ACC1(_s)			stm32_gpiowrite(GPIO_ACC1_PWR_EN, (_s))
# define POWER_ACC2(_s)			stm32_gpiowrite(GPIO_ACC2_PWR_EN, (_s))
# define POWER_RELAY1(_s)		stm32_gpiowrite(GPIO_RELAY1_EN, (_s))
# define POWER_RELAY2(_s)		stm32_gpiowrite(GPIO_RELAY2_EN, (_s))

# define OVERCURRENT_ACC		(!stm32_gpioread(GPIO_ACC_OC_DETECT))
# define OVERCURRENT_SERVO		(!stm32_gpioread(GPIO_SERVO_OC_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	2
# define ADC_VBATT			4
# define ADC_IN5			5

#endif

#ifdef CONFIG_ARCH_BOARD_PX4IO_V2

# define PX4IO_RELAY_CHANNELS		0
# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	2
# define ADC_VSERVO			4
# define ADC_RSSI			5

#endif

#ifdef CONFIG_ARCH_BOARD_VRBRAIN_V45

# define PX4IO_RELAY_CHANNELS		0
//# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
//# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

//# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	0
//# define ADC_VSERVO			4
//# define ADC_RSSI			5

#endif

#ifdef CONFIG_ARCH_BOARD_VRBRAIN_V51

# define PX4IO_RELAY_CHANNELS		0
//# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
//# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

//# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	0
//# define ADC_VSERVO			4
//# define ADC_RSSI			5

#endif

#ifdef CONFIG_ARCH_BOARD_VRBRAIN_V52

# define PX4IO_RELAY_CHANNELS		0
//# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
//# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

//# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	0
//# define ADC_VSERVO			4
//# define ADC_RSSI			5

#endif

#ifdef CONFIG_ARCH_BOARD_VRUBRAIN_V51

# define PX4IO_RELAY_CHANNELS		0
//# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
//# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

//# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	0
//# define ADC_VSERVO			4
//# define ADC_RSSI			5

#endif

#ifdef CONFIG_ARCH_BOARD_VRUBRAIN_V52

# define PX4IO_RELAY_CHANNELS		0
//# define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
//# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

//# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	0
//# define ADC_VSERVO			4
//# define ADC_RSSI			5

#endif

#define BUTTON_SAFETY		stm32_gpioread(GPIO_BTN_SAFETY)

#define CONTROL_PAGE_INDEX(_group, _channel) (_group * PX4IO_CONTROL_CHANNELS + _channel)

/*
 * Mixer
 */
__EXPORT void	mixer_tick(void);
__EXPORT int	mixer_handle_text(const void *buffer, size_t length);

/**
 * Safety switch/LED.
 */
__EXPORT void	safety_init(void);
__EXPORT void	failsafe_led_init(void);

/**
 * FMU communications
 */
__EXPORT void	interface_init(void);
__EXPORT void	interface_tick(void);

/**
 * Register space
 */
__EXPORT int	registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
__EXPORT int	registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values);

/**
 * Sensors/misc inputs
 */
__EXPORT int	adc_init(void);
__EXPORT uint16_t	adc_measure(unsigned channel);

/**
 * R/C receiver handling.
 *
 * Input functions return true when they receive an update from the RC controller.
 */
__EXPORT void	controls_init(void);
__EXPORT void	controls_tick(void);
__EXPORT int	dsm_init(const char *device);
__EXPORT bool	dsm_input(uint16_t *values, uint16_t *num_values, uint8_t *n_bytes, uint8_t **bytes);
__EXPORT void	dsm_bind(uint16_t cmd, int pulses);
__EXPORT int	sbus_init(const char *device);
__EXPORT bool	sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_channels);
__EXPORT void	sbus1_output(uint16_t *values, uint16_t num_values);
__EXPORT void	sbus2_output(uint16_t *values, uint16_t num_values);

/** global debug level for isr_debug() */
__EXPORT extern volatile uint8_t debug_level;

/** send a debug message to the console */
__EXPORT void	isr_debug(uint8_t level, const char *fmt, ...);

/** schedule a reboot */
__EXPORT void schedule_reboot(uint32_t time_delta_usec);

__END_DECLS

#endif /* CONTROLS_H_ */

