/*
 * led_pattern.c
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */
#include "dev_led.h"

static uint16_t m_led_pattern_index = 0;
static uint16_t m_led_tmr = 0;

void dev_led_init(struct dev_led * led)
{
	if (led->port == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if (led->port == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (led->port == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	/* Configure LED */
	GPIO_InitStructure.GPIO_Pin = led->pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(led->port, &GPIO_InitStructure);
	led->port->ODR &= ~led->pin;	//set to 0

	dev_led_set_pattern(led, led->pattern);
}


void dev_led_set_pattern(struct dev_led * led, int pattern)
{
	if (led)
		led->pattern = (uint8_t) pattern;
}

void dev_led_update(struct dev_led * led)
{
	if (led) {
		if (led->pattern & (1 << m_led_pattern_index) ) {
			led->port->ODR &= ~led->pin;
		}
		else {
			led->port->ODR |= led->pin;
		}
	}
	if ((++m_led_pattern_index) >= 8)
		m_led_pattern_index = 0;
}

