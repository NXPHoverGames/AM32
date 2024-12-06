/*
 * flexpwm.h
 *
 *  Created on: 13 Nov 2024
 *      Author: nxg09992
 */

#ifndef MCU_A153_SRC_FLEXPWM_H_
#define MCU_A153_SRC_FLEXPWM_H_

#include "main.h"

/*
 * @brief 	Sets the prescaler of the PWM submodules
 */
//#define SET_PRESCALER_PWM(presc) {
//	modifyReg16(&FLEXPWM0->SM[0].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(map(presc, 0, 127, 0, 7)));
//	modifyReg16(&FLEXPWM0->SM[1].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(map(presc, 0, 127, 0, 7)));
//	modifyReg16(&FLEXPWM0->SM[2].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(map(presc, 0, 127, 0, 7)));
//}

//TODO fix this so the prescaler is not used but the sounds are made by changing VAL1 of the flexPWM
#define SET_PRESCALER_PWM(presc) { \
	modifyReg16(&FLEXPWM0->SM[0].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); \
	modifyReg16(&FLEXPWM0->SM[1].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); \
	modifyReg16(&FLEXPWM0->SM[2].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

/*
 * @brief 	Sets the reload value of the PWM submodules
 */
#define SET_AUTO_RELOAD_PWM(relval) { \
	FLEXPWM0->SM[0].VAL1 = relval; \
	FLEXPWM0->SM[1].VAL1 = relval; \
	FLEXPWM0->SM[2].VAL1 = relval; \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

/*
 * @brief 	Sets the duty cycle of all PWM signals
 */
#define SET_DUTY_CYCLE_ALL(newdc) { \
	FLEXPWM0->SM[0].VAL3 = newdc; \
	FLEXPWM0->SM[1].VAL3 = newdc; \
	FLEXPWM0->SM[2].VAL3 = newdc; \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

void initFlexPWM(void);
void enableFlexPWM(void);

void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void generatePwmTimerEvent(void);

#endif /* MCU_A153_SRC_FLEXPWM_H_ */
