/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"

void initAfterJump(void);
void initCorePeripherals(void);
void SystemClock_Config(void);
void initGPIO(void);
void UN_TIM_Init(void);
void enableCorePeripherals(void);

#endif /* PERIPHERALS_H_ */
