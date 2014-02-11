/*
 * led.h
 *
 *  Created on: 2014. 1. 9.
 *      Author: jongwonk
 */

#ifndef LED_H_
#define LED_H_

void InitLED(void);
void vLedTask(void *vParameter);
void createLEDTask(void);

#endif /* LED_H_ */
