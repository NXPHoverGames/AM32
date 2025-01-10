/*
 * flexpwm.c
 *
 *  Created on: 13 Nov 2024
 *      Author: nxg09992
 */

#include "flexpwm.h"
#include "functions.h"

/*
 * @brief 	Initializes the PWM for three output phases
 * 			Runs on the main_clk which can be configured to be 12MHz, 96MHz, 192MHz or 16kHz
 * 			Is configured to run at main_clk of 96MHz and reloads at 24kHz
 */
void initFlexPWM(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);

	//Enable PWM sub-clock of sub-module 0, 1 and 2
	modifyReg32(&SYSCON->PWM0SUBCTL,
			SYSCON_PWM0SUBCTL_CLK0_EN_MASK | SYSCON_PWM0SUBCTL_CLK1_EN_MASK | SYSCON_PWM0SUBCTL_CLK2_EN_MASK,
			SYSCON_PWM0SUBCTL_CLK0_EN(1) | SYSCON_PWM0SUBCTL_CLK1_EN(1) | SYSCON_PWM0SUBCTL_CLK2_EN(1));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Configure PWM pins
//	//PORT PWMA0 and PWMB0 to pin 3.6 and 3.7
//	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
//	modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
//
//	//PORT PWMA1 and PWMB1 to pin 3.8 and 3.9
//	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
//	modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
//
//	//PORT PWMA2 and PWMB2 to pin 3.10 and 3.11
//	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
//	modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

	//Initialize submodules 0, 1 and 2 identically
	for (int submodule = 0; submodule <= 2; submodule++) {
		//Set prescaler to 1 and that registers are loaded immediately upon MCTRL[LDOK] is set
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL,
				PWM_CTRL_PRSC_MASK | PWM_CTRL_LDMOD_MASK,
				PWM_CTRL_PRSC(0) | PWM_CTRL_LDMOD(0));
		//TODO LDMOD was 1. At 0 it reloads when PWM is reinitialized by VAL1

		//Set complementary channel operation
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2, PWM_CTRL2_INDEP_MASK, PWM_CTRL2_INDEP(0));

		//Set PWM timing. PWM is complementary so PWMB is the reverse of PWMA ignoring the dead-time
		FLEXPWM0->SM[submodule].INIT = 0;					//set initial value
		FLEXPWM0->SM[submodule].VAL1 = TIM1_AUTORELOAD; 	//set reload value
		FLEXPWM0->SM[submodule].VAL2 = 0; 					//set rising flank of PWMA
		FLEXPWM0->SM[submodule].VAL3 = 4000; 					//set falling flank of PWMA

		//Set dead time
		FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(DEAD_TIME);	//PWMA deadtime
		FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(DEAD_TIME);	//PWMB deadtime

		//Enable that force initialization also re-initializes the counter to the init value
//		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2, PWM_CTRL2_FRCEN_MASK, PWM_CTRL2_FRCEN(1));

		//Set init PWM23 value to 1 after a force event
//		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2, PWM_CTRL2_PWM23_INIT_MASK, PWM_CTRL2_PWM23_INIT(1));

		//Invert PWMA and PWMB outputs
//		modifyReg16(&FLEXPWM0->SM[submodule].OCTRL,
//				PWM_OCTRL_POLA_MASK | PWM_OCTRL_POLB_MASK,
//				PWM_OCTRL_POLA(1) | PWM_OCTRL_POLB(1));
	}

	//Set that the force signal from submodule 0 also forces updates to the other submodules.
	//Note that submodule 0 should have a 0 in FORCE_SEL for this to work
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, PWM_CTRL2_FORCE_SEL(1));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, PWM_CTRL2_FORCE_SEL(1));

	//Set that master sync from submodule 0 causes timer counter initialization.
	//Note that submodule 0 must use the local sync signal.
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_INIT_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_INIT_SEL_MASK, PWM_CTRL2_INIT_SEL(2));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_INIT_SEL_MASK, PWM_CTRL2_INIT_SEL(2));

	//Set that master reload signal from submodule 0 is used to reload registers.
	//Note that submodule 0 must use the local reload signal.
//	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_RELOAD_SEL_MASK, 0);
//	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_RELOAD_SEL_MASK, PWM_CTRL2_RELOAD_SEL(1));
//	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_RELOAD_SEL_MASK, PWM_CTRL2_RELOAD_SEL(1));

	//Set that AUX_CLK from submodule 0 is used as clock source for submodule 1 and 2
	//Note that submodule 0 must not use this setting
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_CLK_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_CLK_SEL_MASK, PWM_CTRL2_CLK_SEL(2));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_CLK_SEL_MASK, PWM_CTRL2_CLK_SEL(2));

	//Set that a logic 1 on the fault input causes a fault condition
	modifyReg16(&FLEXPWM0->FCTRL, PWM_FCTRL_FLVL_MASK, PWM_FCTRL_FLVL(0xf));

	//Enable all six PWM outputs
	FLEXPWM0->OUTEN = PWM_OUTEN_PWMA_EN_MASK | PWM_OUTEN_PWMB_EN_MASK;
}

void enableFlexPWM(void)
{
	//Load prescaler, modulus and PWM values of all three submodules
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);

	//Turn on clock source of all three submodules
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_RUN_MASK);
}

inline void setPWMCompare1(uint16_t compareone)
{
	//Set PWM falling flank of submodule 0
	FLEXPWM0->SM[0].VAL3 = compareone;
}

inline void setPWMCompare2(uint16_t comparetwo)
{
	//Set PWM falling flank of submodule 1
	FLEXPWM0->SM[1].VAL3 = comparetwo;
}

inline void setPWMCompare3(uint16_t comparethree)
{
	//Set PWM falling flank of submodule 2
	FLEXPWM0->SM[2].VAL3 = comparethree;
}

inline void generatePwmTimerEvent()
{
	//Load prescaler, modulus and PWM values of all three submodules
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);
//	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK(1));

	//Force update the PWM submodules to re-initialize the counter and output pins
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
//	modifyReg16(&FLEXPWM0->SM[1].CTRL2, 0, PWM_CTRL2_FORCE(1));
//	modifyReg16(&FLEXPWM0->SM[2].CTRL2, 0, PWM_CTRL2_FORCE(1));
}
