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
 * 			Is configured to run at main_clk of 192MHz (see SystemClock_Config()) and reloads at 24kHz (defined by TIM1_AUTORELOAD value)
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

	//Initialize submodules 0, 1 and 2 identically
	for (int submodule = 0; submodule <= 2; submodule++) {
		//Set prescaler to 1 and that buffered registers are set at the next PWM reload if MCTRL[LDOK] is set
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL,
				PWM_CTRL_PRSC_MASK | PWM_CTRL_LDMOD_MASK,
				PWM_CTRL_PRSC(0) | PWM_CTRL_LDMOD(0));	//TODO set LDMOD to 0 for normal operation instead for sounds

		//Set complementary channel operation
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2, PWM_CTRL2_INDEP_MASK, PWM_CTRL2_INDEP(0));

		//Set PWM timing. PWM is complementary so PWMB is the reverse of PWMA ignoring the dead-time
		FLEXPWM0->SM[submodule].INIT = 0;					//set initial value
		FLEXPWM0->SM[submodule].VAL1 = TIM1_AUTORELOAD; 	//set reload value
		FLEXPWM0->SM[submodule].VAL2 = 0; 					//set rising flank of PWMA
		FLEXPWM0->SM[submodule].VAL3 = TIM1_AUTORELOAD / 2; //set falling flank of PWMA

		//Set dead time
		FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(DEAD_TIME);	//PWMA deadtime
		FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(DEAD_TIME);	//PWMB deadtime

		//Disable fault protection as no fault input is sourced
		modifyReg16(&FLEXPWM0->SM[submodule].DISMAP[0],
				PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0X_MASK,
				0);

		//Invert PWMA and PWMB outputs
		modifyReg16(&FLEXPWM0->SM[submodule].OCTRL,
				PWM_OCTRL_POLA_MASK | PWM_OCTRL_POLB_MASK,
				PWM_OCTRL_POLA(1) | PWM_OCTRL_POLB(1));
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

	//Set that AUX_CLK from submodule 0 is used as clock source for submodule 1 and 2
	//Note that submodule 0 must not use this setting
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_CLK_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_CLK_SEL_MASK, PWM_CTRL2_CLK_SEL(2));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_CLK_SEL_MASK, PWM_CTRL2_CLK_SEL(2));

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

	//Force update the PWM submodules to re-initialize the counter and output pins
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}
