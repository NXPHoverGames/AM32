/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "main.h"

void initComp0(void);
void initComp1(void);
void enableComparator(void);
uint8_t getCompOutputLevel();
void maskPhaseInterrupts();
void enableCompInterrupts();
void changeMainComp(LPCMP_Type *CMPx);
void changeCompInput();

extern char rising;
extern char step;

#endif /* COMPARATOR_H_ */
