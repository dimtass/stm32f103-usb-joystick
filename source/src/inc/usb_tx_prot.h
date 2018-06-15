/*
 * usb_tx_prot.h
 *
 *  Created on: 11 Dec 2016
 *      Author: dimtass
 */

#ifndef USB_TX_PROT_H_
#define USB_TX_PROT_H_

#include "joystick.h"

typedef struct {
	uint16_t 	preamble;
	uint8_t		datalen;
	uint8_t		data[JOYS_SAMPLES];
};



#endif /* USB_TX_PROT_H_ */
