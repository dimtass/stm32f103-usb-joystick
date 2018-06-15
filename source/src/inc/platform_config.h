/*
 * platform_config.h
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "dev_uart.h"

/**
 * Trace levels for this project.
 * Have in mind that these are bit flags!
 */
typedef enum {
	TRACE_LEVEL_DEFAULT = 	(1 << 0),
} en_trace_level;

#define DEBUG_TRACE

#ifdef DEBUG_TRACE
#define TRACE(X) TRACEL(TRACE_LEVEL_DEFAULT, X)
#define TRACEL(TRACE_LEVEL, X) do { if (glb.trace_levels & TRACE_LEVEL) printf X;} while(0)
#else
#define TRACE(X)
#define TRACEL(X,Y)
#endif

/* LED patterns */
enum {
	LED_PATTERN_IDLE = 0b10100000,
};

/* GPIOA */
#define AXIS_X_Pin GPIO_Pin_0
#define AXIS_X_GPIO_Port GPIOA
#define AXIS_Y_Pin GPIO_Pin_1
#define AXIS_Y_GPIO_Port GPIOA
#define JOYS_BTN_Pin GPIO_Pin_2
#define JOYS_BTN_GPIO_Port GPIOA

/* GPIOC */
#define PIN_STATUS_LED GPIO_Pin_13
#define PORT_STATUS_LED GPIOC

struct tp_glb {
	volatile uint16_t tmr_1ms;
	volatile uint16_t tmr_250ms;
	en_trace_level trace_levels;
	uint16_t tmr_joys_update;

	/* ADC values */
	__IO uint16_t adc_axis_x;
	__IO uint16_t adc_axis_y;
};


extern struct tp_glb glb;

static inline void set_trace_level(en_trace_level level, uint8_t enable)
{
	if (enable) {
		glb.trace_levels |= (uint32_t) level;
	}
	else {
		glb.trace_levels &= ~((uint32_t) level);
	}
}

#endif /* __PLATFORM_CONFIG_H */
