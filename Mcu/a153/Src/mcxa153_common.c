/*
 * mcxa153_common.c
 *
 *  Created on: 13 Nov 2024
 *      Author: nxg09992
 */

#include "mcxa153_common.h"

void modifyReg32(volatile uint32_t *regAddr, uint32_t clearbits, uint32_t setbits)
{
	uint32_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

void modifyReg16(volatile uint16_t *regAddr, uint16_t clearbits, uint16_t setbits)
{
	uint16_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

void modifyReg8(volatile uint8_t *regAddr, uint8_t clearbits, uint8_t setbits)
{
	uint8_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}
