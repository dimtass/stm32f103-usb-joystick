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
#define JOYS_UPDATE_TMR_MS	10
#define JOYS_DEBOUNCE_CNTR  5
#define JOYS_RECOGNITION_TIME_MS	300 / JOYS_UPDATE_TMR_MS

#define DECLARE_JOYS(NAME, CALLBACK) \
    struct joys_init_data NAME = { \
        .debounce_cntr = JOYS_DEBOUNCE_CNTR, \
        .recognition_time_ms = JOYS_RECOGNITION_TIME_MS, \
        .fp_joys_callback = CALLBACK, \
    }

struct joys_init_data {
    uint8_t     debounce_cntr;
    uint16_t    recognition_time_ms;
    void (*fp_joys_callback)(char * str_result, uint8_t str_len);
};

void joys_init(struct joys_init_data * init);
void joys_update(uint16_t x, uint16_t y);

#endif /* JOYSTICK_H_ */
