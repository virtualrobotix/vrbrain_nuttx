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
 * @file board_config.h
 *
 * VRBRAIN internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>
 
/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* VRBRAIN connection configuration */

#define UDID_START		0x1FFF7A10

#define APM_BUILD_APMrover2      1
#define APM_BUILD_ArduCopter     2
#define APM_BUILD_ArduPlane      3
#define APM_BUILD_AntennaTracker 4

#ifndef APM_BUILD_DIRECTORY
#define APM_BUILD_DIRECTORY		APM_BUILD_ArduCopter
#endif

#ifdef APM_BUILD_DIRECTORY
#define APM_BUILD_TYPE(type) ((type) == APM_BUILD_DIRECTORY)
#endif

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)

#endif

#define RC_INPUT_PWM 		1
#define RC_INPUT_PPMSUM 	2
#define RC_INPUT_SBUS 		4
#define RC_INPUT_DSM 		8

#ifndef CONFIG_RC_INPUTS
 #define CONFIG_RC_INPUTS  RC_INPUT_PPMSUM + RC_INPUT_SBUS
#endif

#ifdef CONFIG_RC_INPUTS
#define CONFIG_RC_INPUTS_TYPE(type) ((type) & CONFIG_RC_INPUTS)
#endif

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)

#endif
#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PPMSUM)

#endif
#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_SBUS)
#define SBUS_COM_PORT "/dev/ttyS3"
#endif
#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_DSM)
#define DSM_COM_PORT "/dev/ttyS3"
#endif

//#define MPU6000_EXTERNAL

/* VRBRAIN GPIOs ***********************************************************************************/

/* BOARD LEDs */
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* EXTERNAL LEDs */
#define GPIO_EXT_LED1   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)
#define GPIO_EXT_LED2   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)


/* BUZZER */
#define GPIO_BUZZER     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)

/* SPI chip selects */
#define GPIO_SPI_CS_DATAFLASH	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_SPI_CS_MS5611  	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_SPI_CS_MPU6000  	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#ifndef MPU6000_EXTERNAL
#define GPIO_SPI_CS_SDCARD	    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#else
#define GPIO_SPI_CS_EXP_MPU6000	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#endif

#define SPI_BUS_AT45BD		 1
#define SPI_BUS_MS5611		 1
#define SPI_BUS_MPU6000 	 2
#ifndef MPU6000_EXTERNAL
#define SPI_BUS_SDCARD		 3
#else
#define SPI_BUS_EXP_MPU6000	 3
#endif

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1
 */
#define SPIDEV_MS5611		50







/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI2
 */
#define SPIDEV_MPU6000  	51




/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3
 */
#define SPIDEV_EXP_MPU6000	52


/*
 * I2C busses
 */
#define I2C_BUS_HMC5883			2
#define I2C_BUS_EXT_HMC5883		1
#define I2C_BUS_EEPROM			2
#define I2C_BUS_MEAS_AIRSPEED	1
#define I2C_BUS_MB12XX          1
#define I2C_BUS_RGBLED          1


/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define I2CDEV_HMC5883    0x1E



/* User GPIOs ********************/
#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define GPIO_GPIO0_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#endif
#if !CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)
#define GPIO_GPIO1_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)
#endif

/* USB Present */

/* Shutdown */

/* WIFI **************************/




/* SBUS **************************/





/*
 * PWM OUTPUT
 */

#define GPIO_TIM2_CH2OUT	GPIO_TIM2_CH2OUT_1
#define GPIO_TIM2_CH3OUT	GPIO_TIM2_CH3OUT_1
#define GPIO_TIM2_CH4OUT	GPIO_TIM2_CH4OUT_1
#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_2
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_1
#define GPIO_TIM4_CH3OUT	GPIO_TIM4_CH3OUT_1
#define GPIO_TIM4_CH4OUT	GPIO_TIM4_CH4OUT_1
#if !CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)
#define GPIO_TIM1_CH1OUT	GPIO_TIM1_CH1OUT_2
#define GPIO_TIM1_CH2OUT	GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_2
#define GPIO_TIM1_CH4OUT	GPIO_TIM1_CH4OUT_2
#endif

/*
 * PWM INPUT
 */

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)
#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN		GPIO_TIM1_CH4IN_2
#define GPIO_TIM8_CH1IN		GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN		GPIO_TIM8_CH2IN_1
#define GPIO_TIM8_CH3IN		GPIO_TIM8_CH3IN_1
#define GPIO_TIM8_CH4IN		GPIO_TIM8_CH4IN_1
#endif

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTD|GPIO_PIN4)

/* High-resolution timer
 */
#define HRT_TIMER			9	/* use timer9 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel 2 */

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PPMSUM)
#define PPMSUM_TIMER		8	/* use timer8 for the PPMSUM */
#define PPMSUM_CHANNEL		3	/* use capture/compare channel 3 */
#define GPIO_PPM_IN			(GPIO_ALT|GPIO_AF3|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN8)
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/**
 * Enum for sensors bus.
 */
enum BusSensor {
	TYPE_BUS_SENSOR_NONE         = 0,
	TYPE_BUS_SENSOR_INTERNAL     = 1,
	TYPE_BUS_SENSOR_IMU          = 2,
	TYPE_BUS_SENSOR_EXTERNAL     = 3
};

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the VRBRAIN board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

#endif /* __ASSEMBLY__ */

__END_DECLS
