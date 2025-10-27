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
void disableComparators(void);
void maskPhaseInterrupts();
void enableCompInterrupts();
void changeMainComp(LPCMP_Type *CMPx);
void changeCompInput();

//#define COMP_OUT_INLINE

#ifdef COMP_OUT_INLINE

extern LPCMP_Type *MAIN_COMP;

//static inline uint8_t getCompOutputLevel()
//{
//	return ((MAIN_COMP->CSR & LPCMP_CSR_COUT_MASK) >> LPCMP_CSR_COUT_SHIFT);
//}
#else
uint8_t getCompOutputLevel();
#endif

extern char rising;
extern char step;

#endif /* COMPARATOR_H_ */
