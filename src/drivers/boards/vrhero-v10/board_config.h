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

/* BOARD LEDs */
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* EXTERNAL LEDs */

/* BUZZER */

/* SPI chip selects */
#define GPIO_SPI_CS_DATAFLASH	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_SPI_CS_EEPROM      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)
#define GPIO_SPI_CS_MS5611  	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_SPI_CS_MPU6000		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define GPIO_SPI_CS_SDCARD	    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)
#define GPIO_SPI_CS_DF_EXT1     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI_CS_DF_EXT2     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_DF_EXT3     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI_CS_DF_EXT4     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define GPIO_SPI_CS_DF_EXT5     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)

#define SPI_BUS_DATAFLASH	 1

#define SPI_BUS_MS5611		 1
#define SPI_BUS_MPU6000		 2
#define SPI_BUS_SDCARD		 3





/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1
 */
#define SPIDEV_MS5611		50
#define SPIDEV_EEPROM		52
#define SPIDEV_DF_EXT1		53
#define SPIDEV_DF_EXT2		54
#define SPIDEV_DF_EXT3		55
#define SPIDEV_DF_EXT4		56
#define SPIDEV_DF_EXT5		57

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI2
 */
#define SPIDEV_MPU6000		51




/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3
 */


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
#define I2CDEV_HMC5883    0x1E



/* User GPIOs ********************/
//#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
//#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)

/* USB Present */
#define GPIO_USB_PRESENT (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN4)

/* Shutdown */
#define GPIO_SHUTDOWN_INT	(GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN0)
//#define GPIO_SHUTDOWN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
//#define GPIO_SHUTDOWN		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
//TEO 20140226 - lascio giù il pin di shutdown per evitare reboot indesiderati...
//ci pensa poi rcS a ritirarlo su
#define GPIO_SHUTDOWN		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* AUX */
//#define GPIO_AUX_OUT1		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
//#define GPIO_AUX_OUT2		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
//#define GPIO_AUX_OUT3		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)
//#define GPIO_AUX_IN1		(GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN0)
//#define GPIO_AUX_IN2		(GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN1)


//#define AUX_IO1_OUT
//#define AUX_IO2_OUT
//#define AUX_IO3_OUT
//#define AUX_IO4_OUT
#define AUX_IO5_OUT
//#define AUX_IO6_OUT

//PWM_CH0
#ifdef AUX_IO1_OUT
#define GPIO_AUX_IO1		(GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#else
#define GPIO_AUX_IO1		(GPIO_INPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN1)
#endif

//PWM_CH1
#ifdef AUX_IO2_OUT
#define GPIO_AUX_IO2		(GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#else
#define GPIO_AUX_IO2		(GPIO_INPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN2)
#endif

//PWM_CH2
#ifdef AUX_IO3_OUT
#define GPIO_AUX_IO3		(GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#else
#define GPIO_AUX_IO3		(GPIO_INPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN3)
#endif

//SIG_CAN_RX
#ifdef AUX_IO4_OUT
#define GPIO_AUX_IO4		(GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN0)
#else
#define GPIO_AUX_IO4		(GPIO_INPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN0)
#endif

//SIG_CAN_TX
#ifdef AUX_IO5_OUT
#define GPIO_AUX_IO5		(GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN1)
#else
#define GPIO_AUX_IO5		(GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN1)
#endif

//DISP_FREE
#ifdef AUX_IO6_OUT
#define GPIO_AUX_IO6    (GPIO_OUTPUT |GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#else
#define GPIO_AUX_IO6    (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN15)
#endif


/* WIFI **************************/




/* SBUS **************************/





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
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer
 */
#define HRT_TIMER			1	/* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel 2 */
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
