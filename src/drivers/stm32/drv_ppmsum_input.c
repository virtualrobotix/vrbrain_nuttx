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

/*
 * @file drv_ppmsum_input.c
 *
 * Servo driver supporting PPMSUM inputs connected to STM32 timer blocks.
 */

#include <board_config.h>

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PPMSUM)

#include "drv_ppmsum_input.h"

/* Timer configuration */
#if   PPMSUM_TIMER == 1
# define PPMSUM_TIMER_BASE		STM32_TIM1_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define PPMSUM_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
# if CONFIG_STM32_TIM1
#  error must not set CONFIG_STM32_TIM1=y and PPMSUM_TIMER=1
# endif
#elif PPMSUM_TIMER == 2
# define PPMSUM_TIMER_BASE		STM32_TIM2_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB1ENR_TIM2EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM2
# define PPMSUM_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and PPMSUM_TIMER=2
# endif
#elif PPMSUM_TIMER == 3
# define PPMSUM_TIMER_BASE		STM32_TIM3_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB1ENR_TIM3EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM3
# define PPMSUM_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
# if CONFIG_STM32_TIM3
#  error must not set CONFIG_STM32_TIM3=y and PPMSUM_TIMER=3
# endif
#elif PPMSUM_TIMER == 4
# define PPMSUM_TIMER_BASE		STM32_TIM4_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB1ENR_TIM4EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM4
# define PPMSUM_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
# if CONFIG_STM32_TIM4
#  error must not set CONFIG_STM32_TIM4=y and PPMSUM_TIMER=4
# endif
#elif PPMSUM_TIMER == 5
# define PPMSUM_TIMER_BASE		STM32_TIM5_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB1ENR_TIM5EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM5
# define PPMSUM_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
# if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and PPMSUM_TIMER=5
# endif
#elif PPMSUM_TIMER == 8
# define PPMSUM_TIMER_BASE		STM32_TIM8_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define PPMSUM_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
# if CONFIG_STM32_TIM8
#  error must not set CONFIG_STM32_TIM8=y and PPMSUM_TIMER=8
# endif
#elif PPMSUM_TIMER == 9
# define PPMSUM_TIMER_BASE		STM32_TIM9_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB2ENR_TIM9EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM9
# define PPMSUM_TIMER_CLOCK	STM32_APB2_TIM9_CLKIN
# if CONFIG_STM32_TIM9
#  error must not set CONFIG_STM32_TIM9=y and PPMSUM_TIMER=9
# endif
#elif PPMSUM_TIMER == 10
# define PPMSUM_TIMER_BASE		STM32_TIM10_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB2ENR_TIM10EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM1UP
# define PPMSUM_TIMER_CLOCK	STM32_APB2_TIM10_CLKIN
# if CONFIG_STM32_TIM10
#  error must not set CONFIG_STM32_TIM11=y and PPMSUM_TIMER=10
# endif
#elif PPMSUM_TIMER == 11
# define PPMSUM_TIMER_BASE		STM32_TIM11_BASE
# define PPMSUM_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PPMSUM_TIMER_POWER_BIT	RCC_APB2ENR_TIM11EN
# define PPMSUM_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define PPMSUM_TIMER_CLOCK	STM32_APB2_TIM11_CLKIN
# if CONFIG_STM32_TIM11
#  error must not set CONFIG_STM32_TIM11=y and PPMSUM_TIMER=11
# endif
#else
# error PPMSUM_TIMER must be a value between 1 and 11
#endif

/*
 * PPMSUM clock must be a multiple of 1MHz greater than 1MHz
 */
#if (PPMSUM_TIMER_CLOCK % 1000000) != 0
# error PPMSUM_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if PPMSUM_TIMER_CLOCK <= 1000000
# error PPMSUM_TIMER_CLOCK must be greater than 1MHz
#endif

/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define PPMSUM_INTERVAL_MIN	50
#define PPMSUM_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
#define PPMSUM_COUNTER_PERIOD	65536

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define PPMSUM_COUNTER_SCALE(_c)	(_c)

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PPMSUM_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

/* timer-specific functions */
static int		ppmsum_timer_isr(int irq, void *context);

/*
 * Specific registers and bits used by PPM sub-functions
 */

/*
 * If the timer hardware doesn't support GTIM_CCER_CCxNP, then we will work around it.
 *
 * Note that we assume that M3 means STM32F1 (since we don't really care about the F2).
 */
# ifdef CONFIG_ARCH_CORTEXM3
#  undef GTIM_CCER_CC1NP
#  undef GTIM_CCER_CC2NP
#  undef GTIM_CCER_CC3NP
#  undef GTIM_CCER_CC4NP
#  define GTIM_CCER_CC1NP 0
#  define GTIM_CCER_CC2NP 0
#  define GTIM_CCER_CC3NP 0
#  define GTIM_CCER_CC4NP 0
#  define PPM_EDGE_FLIP
# endif

# if PPMSUM_CHANNEL == 1
#  define rCCR_PPM		rCCR1				/* capture register for PPM */
#  define DIER_PPM		GTIM_DIER_CC1IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC1IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC1OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM		((0x01 << 0) | (0x03 << 4))		/* not on TI1/TI2 */
#  define CCMR2_PPM		0					/* on TI3, not on TI4 */
#  define CCER_PPM		(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) /* CC1, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC1P
# elif PPMSUM_CHANNEL == 2
#  define rCCR_PPM		rCCR2				/* capture register for PPM */
#  define DIER_PPM		GTIM_DIER_CC2IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC2IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC2OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM		((0x01 << 8) | (0x03 << 12))		/* not on TI1/TI2 */
#  define CCMR2_PPM		0					/* on TI3, not on TI4 */
#  define CCER_PPM		(GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P
# elif PPMSUM_CHANNEL == 3
#  define rCCR_PPM		rCCR3				/* capture register for PPM */
#  define DIER_PPM		GTIM_DIER_CC3IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC3IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC3OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM		0					/* not on TI1/TI2 */
#  define CCMR2_PPM		((0x01 << 0) | (0x03 << 4))		/* on TI3, not on TI4 */
#  define CCER_PPM		(GTIM_CCER_CC3E | GTIM_CCER_CC3P | GTIM_CCER_CC3NP) /* CC3, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC3P
# elif PPMSUM_CHANNEL == 4
#  define rCCR_PPM		rCCR4				/* capture register for PPM */
#  define DIER_PPM		GTIM_DIER_CC4IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC4IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC4OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM		0					/* not on TI1/TI2 */
#  define CCMR2_PPM		((0x01 << 8) | (0x03 << 12))		/* on TI3, not on TI4 */
#  define CCER_PPM		(GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP) /* CC4, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC4P
# else
#  error PPMSUM_CHANNEL must be a value between 1 and 4
# endif

/*
 * PPM decoder tuning parameters
 */
# define PPM_MIN_PULSE_WIDTH	200		/**< minimum width of a valid first pulse */
# define PPM_MAX_PULSE_WIDTH	600		/**< maximum width of a valid first pulse */
# define PPM_MIN_CHANNEL_VALUE	800		/**< shortest valid channel signal */
# define PPM_MAX_CHANNEL_VALUE	2200		/**< longest valid channel signal */
# define PPM_MIN_START		2300		/**< shortest valid start gap (only 2nd part of pulse) */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	5
#define PPM_MAX_CHANNELS	20

/** Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK	4		/**< should be less than the input timeout */

__EXPORT uint16_t ppm_buffer[PPM_MAX_CHANNELS];
__EXPORT uint16_t ppm_frame_length = 0;
__EXPORT unsigned ppm_decoded_channels = 0;
__EXPORT uint64_t ppm_last_valid_decode = 0;

/* PPM edge history */
__EXPORT uint16_t ppm_edge_history[32];
unsigned ppm_edge_next;

/* PPM pulse history */
__EXPORT uint16_t ppm_pulse_history[32];
unsigned ppm_pulse_next;

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

/** PPM decoder state machine */
struct {
	uint16_t	last_edge;	/**< last capture time */
	uint16_t	last_mark;	/**< last significant edge */
	uint16_t	frame_start;	/**< the frame width */
	unsigned	next_channel;	/**< next channel index */
	enum {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} phase;
} ppm;

static void	ppmsum_decode(uint32_t status);

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
void
up_ppmsum_input_init(void)
{
	/* claim our interrupt vector */
	irq_attach(PPMSUM_TIMER_VECTOR, ppmsum_timer_isr);

	/* clock/power on our timer */
	modifyreg32(PPMSUM_TIMER_POWER_REG, 0, PPMSUM_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PPM;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PPM;
	rCCMR2 = CCMR2_PPM;
	rCCER = CCER_PPM;
	rDCR = 0;

	/* configure the timer to free-run at 1MHz */
	rPSC = (PPMSUM_TIMER_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

	/* run the full span of the counter */
	rARR = 0xffff;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(PPMSUM_TIMER_VECTOR);

	/* configure the PPM input pin */
	stm32_configgpio(GPIO_PPM_IN);

}

/**
 * Handle the PPM decoder state machine.
 */
static void
ppmsum_decode(uint32_t status)
{
	uint16_t count = rCCR_PPM;
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PPM)
		goto error;

	/* how long since the last edge? - this handles counter wrapping implicitely. */
	width = count - ppm.last_edge;

	ppm_edge_history[ppm_edge_next++] = width;

	if (ppm_edge_next >= 32)
		ppm_edge_next = 0;

	/*
	 * if this looks like a start pulse, then push the last set of values
	 * and reset the state machine
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++)
					ppm_buffer[i] = ppm_temp_buffer[i];

				ppm_last_valid_decode = hrt_absolute_time();

			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		ppm.last_edge = count;
		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		break;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH)
			goto error;		/* pulse was too short or too long */

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = ppm.last_edge;

		/* frame length is everything including the start gap */
		ppm_frame_length = (uint16_t)(ppm.last_edge - ppm.frame_start);
		ppm.frame_start = ppm.last_edge;
		ppm.phase = ACTIVE;
		break;

	case INACTIVE:

		/* we expect a short pulse */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH)
			goto error;		/* pulse was too short or too long */

		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;
		break;

	case ACTIVE:
		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

		ppm_pulse_history[ppm_pulse_next++] = interval;

		if (ppm_pulse_next >= 32)
			ppm_pulse_next = 0;

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE))
			goto error;

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS)
			ppm_temp_buffer[ppm.next_channel++] = interval;

		ppm.phase = INACTIVE;
		break;

	}

	ppm.last_edge = count;
	return;

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;

}

/**
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
ppmsum_timer_isr(int irq, void *context)
{
	uint32_t status;

	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

	/* was this a PPM edge? */
	if (status & (SR_INT_PPM | SR_OVF_PPM)) {
		/* if required, flip edge sensitivity */
# ifdef PPM_EDGE_FLIP
		rCCER ^= CCER_PPM_FLIP;
# endif

		ppmsum_decode(status);
	}

	return OK;
}

#endif
