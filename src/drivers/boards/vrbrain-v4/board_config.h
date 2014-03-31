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








/* VRBRAIN GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* SPI chip selects */
#define GPIO_SPI_CS_DATAFLASH	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_SPI_CS_SDCARD	    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#define GPIO_SPI_CS_MS5611  	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_SPI_CS_MPU6000		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)

#define SPI_BUS_DATAFLASH	 1
#define SPI_BUS_SDCARD		 3
#define SPI_BUS_MS5611		 1
#define SPI_BUS_MPU6000		 2

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1
 */
#define SPIDEV_MS5611		50

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI2
 */
#define SPIDEV_MPU6000		51

/*
 * I2C busses
 */
#define I2C_BUS_HMC5883		 2
#define I2C_BUS_EEPROM		 2

/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define I2CDEV_HMC5883    0x1e

/* User GPIOs
 *



 */


















/*
 * Tone alarm output
 */
//#define TONE_ALARM_TIMER	3	/* timer 3 */
//#define TONE_ALARM_CHANNEL	3	/* channel 3 */
//#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)
//#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN8)

/*
 * PWM
 */

#define GPIO_TIM2_CH2OUT	GPIO_TIM2_CH2OUT_1
#define GPIO_TIM2_CH3OUT	GPIO_TIM2_CH3OUT_1
#define GPIO_TIM2_CH4OUT	GPIO_TIM2_CH4OUT_1
#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_2
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_1
#define GPIO_TIM4_CH3OUT	GPIO_TIM4_CH3OUT_1
#define GPIO_TIM4_CH4OUT	GPIO_TIM4_CH4OUT_1

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTD|GPIO_PIN4)

/* High-resolution timer
 */
#define HRT_TIMER			1	/* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL	4	/* use capture/compare channel 4 */
#define HRT_PPM_CHANNEL		1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN			(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

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
