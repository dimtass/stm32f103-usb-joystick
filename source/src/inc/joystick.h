/*
 * joystick.h
 *
 *  Created on: 10 Dec 2016
 *      Author: dimtass
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>
#include "platform_config.h"

#define JOYS_SAMPLES_MAX	24
#define JOYS_UPDATE_TMR_MS	150

void joys_reset(void);
void joys_update(uint16_t x, uint16_t y);

#endif /* JOYSTICK_H_ */
