/*
 * joystick.c
 *
 *  Created on: 10 Dec 2016
 *      Author: dimtass
 */

#include "joystick.h"
#include "platform_config.h"
#include <string.h>

#define JOYS_RECOGNITION_TIME_MS	200 / JOYS_UPDATE_TMR_MS

/* ADC voltages */
#define JOYS_THRESS_UP	1000
#define JOYS_THRESS_DN	3000
#define JOYS_THRESS_L	3000
#define JOYS_THRESS_R	1000
#define JOYS_BTN_PRESS	0

#define IS_SAMPLE_UP(VAL)	(VAL < JOYS_THRESS_UP)
#define IS_SAMPLE_DN(VAL)	(VAL > JOYS_THRESS_DN)
#define IS_SAMPLE_R(VAL)	(VAL < JOYS_THRESS_R)
#define IS_SAMPLE_L(VAL)	(VAL > JOYS_THRESS_L)

/**
 * Joystic attributes
 */
typedef enum {
	JOYS_SAMPLE_UP = 1 << 0,
	JOYS_SAMPLE_DN = 1 << 1,
	JOYS_SAMPLE_L = 1 << 2,
	JOYS_SAMPLE_R = 1 << 3,
	JOYS_SAMPLE_BTN = 1 << 4,
} en_joys_sample_dir;

/**
 * 8-bit flags that store the direction bits
 * for each sample. Each bit is one of the
 * en_joys_attr
 */

typedef struct {
	uint8_t		samples[JOYS_SAMPLES_MAX];
	uint16_t	sample_index;
	uint16_t	recogn_tmr;
} tp_joys_data;
static tp_joys_data joys;

void joys_debug_print_samples(uint8_t * samples, uint16_t samples_len);

void joys_update(uint16_t x, uint16_t y)
{
	uint8_t current_sample = 0;

	/* Check if sample buffer is full */
	if (joys.sample_index >= JOYS_SAMPLES_MAX)
		goto __check;	// This jump will timeout the recognition

	/* Check for up/dn */
	if (IS_SAMPLE_UP(y))
		current_sample |= JOYS_SAMPLE_UP;
	else if (IS_SAMPLE_DN(y))
		current_sample |= JOYS_SAMPLE_DN;

	/* Check for left/right */
	if (IS_SAMPLE_L(x))
		current_sample |= JOYS_SAMPLE_L;
	else if (IS_SAMPLE_R(x))
		current_sample |= JOYS_SAMPLE_R;

	/* Check for button press */
	if (!!(JOYS_BTN_GPIO_Port->IDR & JOYS_BTN_Pin) == JOYS_BTN_PRESS)
		current_sample |= JOYS_SAMPLE_BTN;

	/* Get previous sample */
	uint8_t prev_sample = (!joys.sample_index) ? 0 : joys.sample_index - 1;

	if (current_sample != joys.samples[prev_sample]) {
		TRACE(("%d/%d\n", current_sample, prev_sample));
		/* only store valid directions */
		joys.samples[joys.sample_index] = current_sample;
		joys.sample_index++;
		joys.recogn_tmr = 0;
	}

__check:
	/* Check inactivity */
	if (!current_sample) joys.recogn_tmr++;

	/* After inactivity check for gesture */
	if (joys.recogn_tmr > JOYS_RECOGNITION_TIME_MS) {
		if (joys.sample_index)
			joys_debug_print_samples(joys.samples, joys.sample_index);
		/* Reset */
		joys_reset();
		joys.recogn_tmr = 0;
	}
}

void joys_debug_print_samples(uint8_t * samples, uint16_t samples_len)
{
	TRACE(("Samples: %d -> ", samples_len));
	for (int i=0; i<samples_len; i++) {
		if (samples[i] & JOYS_SAMPLE_UP) {
			TRACE(("U"));
		}
		if (samples[i] & JOYS_SAMPLE_DN) {
			TRACE(("D"));
		}
		if (samples[i] & JOYS_SAMPLE_L) {
			TRACE(("L"));
		}
		if (samples[i] & JOYS_SAMPLE_R) {
			TRACE(("R"));
		}
		if (samples[i] & JOYS_SAMPLE_BTN) {
			TRACE(("B"));
		}
		TRACE((","));
	}
	TRACE(("\n"));
}


/* Send gesture */
void joys_send(void)
{

}

/**
 * Reset xy values
 */
void joys_reset(void)
{
	memset(joys.samples, 0, JOYS_SAMPLES_MAX);
	joys.sample_index = 0;
	joys.recogn_tmr = 0;
}

