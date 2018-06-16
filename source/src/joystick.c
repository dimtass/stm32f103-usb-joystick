/*
 * joystick.c
 *
 *  Created on: 10 Dec 2016
 *      Author: dimtass
 */

#include "joystick.h"
#include "platform_config.h"
#include <string.h>


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
	uint16_t 	prev_value;
	uint8_t		debounce_cntr;
	uint16_t	sample_index;
	uint16_t	recogn_tmr;
} tp_joys_data;

static tp_joys_data joys;
static struct joys_init_data * joys_init_data = NULL;

#define STR_RESULT_SIZE		JOYS_SAMPLES_MAX*3 + 2
char str_result[STR_RESULT_SIZE];

void joys_reset(void);
void joys_str_from_samples(uint8_t * samples, uint16_t samples_len);

void joys_init(struct joys_init_data * init)
{
	if (init) {
		if (!init->debounce_cntr)
			init->debounce_cntr = JOYS_DEBOUNCE_CNTR;
		if (!init->recognition_time_ms)
			init->recognition_time_ms = JOYS_RECOGNITION_TIME_MS;
		joys_init_data = init;
	}
}

void joys_delete(void)
{
	joys_init_data = NULL;
}

void joys_update(uint16_t x, uint16_t y)
{
	uint8_t curr_value = 0;

	/* Check if sample buffer is full */
	if (joys.sample_index >= JOYS_SAMPLES_MAX)
		goto __check;	// This jump will timeout the recognition

	/* Check for up/dn */
	if (IS_SAMPLE_UP(y))
		curr_value |= JOYS_SAMPLE_UP;
	else if (IS_SAMPLE_DN(y))
		curr_value |= JOYS_SAMPLE_DN;

	/* Check for left/right */
	if (IS_SAMPLE_L(x))
		curr_value |= JOYS_SAMPLE_L;
	else if (IS_SAMPLE_R(x))
		curr_value |= JOYS_SAMPLE_R;

	/* Check for button press */
	if (!!(JOYS_BTN_GPIO_Port->IDR & JOYS_BTN_Pin) == JOYS_BTN_PRESS)
		curr_value |= JOYS_SAMPLE_BTN;

	/* debounce logic */
	if (joys.prev_value != curr_value) {
		joys.prev_value = curr_value;
		joys.debounce_cntr = 0;
		return;
	}
	if ((joys.debounce_cntr++) >= JOYS_DEBOUNCE_CNTR) {
		joys.debounce_cntr = 0;
		/* Get previous sample */
		uint8_t prev_sample_index = (!joys.sample_index) ? 0 : joys.sample_index - 1;
		/* Do not log same samples */
		if (curr_value && curr_value != joys.samples[prev_sample_index]) {
			TRACEL(TRACE_LEVEL_JOYS, ("%d/%d\n", curr_value, prev_sample_index));
			/* only store valid directions */
			if (curr_value) {
				joys.samples[joys.sample_index] = curr_value;
				joys.sample_index++;
				joys.recogn_tmr = 0;
			}
		}
	}

__check:
	/* Check inactivity */
	if (!curr_value) joys.recogn_tmr++;

	/* After inactivity check for gesture */
	if (joys.recogn_tmr > JOYS_RECOGNITION_TIME_MS) {
		if (joys.sample_index) {
			joys_str_from_samples(joys.samples, joys.sample_index);
			joys_init_data->fp_joys_callback(str_result, strlen(str_result));
		}
		/* Reset */
		joys_reset();
		joys.recogn_tmr = 0;
		joys.debounce_cntr = 0;
	}
}

void joys_str_from_samples(uint8_t * samples, uint16_t samples_len)
{
	TRACEL(TRACE_LEVEL_JOYS, ("Samples: %d -> ", samples_len));
	// memset(str_result, 0, STR_RESULT_SIZE);
	int k = 0;
	for (int i=0; i<samples_len; i++) {
		if (samples[i] & JOYS_SAMPLE_UP) {
			str_result[k++] = 'U';
			// TRACEL(TRACE_LEVEL_JOYS, ("%s", "U"));
		}
		if (samples[i] & JOYS_SAMPLE_DN) {
			str_result[k++] = 'D';
			// TRACEL(TRACE_LEVEL_JOYS, ("%s", "D"));
		}
		if (samples[i] & JOYS_SAMPLE_L) {
			str_result[k++] = 'L';
			// TRACEL(TRACE_LEVEL_JOYS, ("%s", "L"));
		}
		if (samples[i] & JOYS_SAMPLE_R) {
			str_result[k++] = 'R';
			// TRACEL(TRACE_LEVEL_JOYS, ("%s", "R"));
		}
		if (samples[i] & JOYS_SAMPLE_BTN) {
			str_result[k++] = 'B';
			// TRACEL(TRACE_LEVEL_JOYS, ("%s", "B"));
		}
		str_result[k++] = ',';
		// TRACEL(TRACE_LEVEL_JOYS, ("%s", ","));
	}
	str_result[k++] = '\n';
	str_result[k++] = 0;
	TRACEL(TRACE_LEVEL_JOYS, ("%s", str_result));
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

