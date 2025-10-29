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
 * 			PWM_A controls the LOW FET. PWM_B control the HIGH FET.
 */
void initFlexPWM(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_AOI0(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_AOI0(1);

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
				PWM_CTRL_PRSC(0) | PWM_CTRL_LDMOD(0));

		//Set complementary channel operation and that a force out event initializes the counter
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2,
				PWM_CTRL2_INDEP_MASK | PWM_CTRL2_FRCEN_MASK,
				PWM_CTRL2_INDEP(0) | PWM_CTRL2_FRCEN(1));

		//Set PWM timing. PWM is complementary so PWMB is the reverse of PWMA ignoring the dead-time
		FLEXPWM0->SM[submodule].INIT = 0;					//set initial value
		FLEXPWM0->SM[submodule].VAL1 = TIM1_AUTORELOAD; 	//set reload value
		FLEXPWM0->SM[submodule].VAL2 = 0; 					//set rising flank of PWMA
		FLEXPWM0->SM[submodule].VAL3 = TIM1_AUTORELOAD / 2; //set falling flank of PWMA

		//Set dead time
		FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(DEAD_TIME);	//PWMA deadtime
		FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(DEAD_TIME);	//PWMB deadtime

		//Enable map the corresponding fault input to disable PWM_A and PWM_B in each DISMAP register
		//All PWM outputs will go into fault when either of the fault inputs assert.
		modifyReg16(&FLEXPWM0->SM[submodule].DISMAP[0],
				PWM_DISMAP_DIS0X_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0A_MASK,
				PWM_DISMAP_DIS0A(0x7) | PWM_DISMAP_DIS0B(0x7));

		//Invert PWMA and PWMB outputs to comprehend for correct PWM timing. Non-invert if PWM_A controls HIGH FET and PWM_B controls LOW FET.
		//PWM_A controls the LOW FET
		//PWM_B controls the HIGH FET
		modifyReg16(&FLEXPWM0->SM[submodule].OCTRL,
				PWM_OCTRL_POLA_MASK | PWM_OCTRL_POLB_MASK,
				PWM_OCTRL_POLA(1) | PWM_OCTRL_POLB(1));
	}

	//Set that PWM23 (VAL2 and VAL3) is used for complementary PWM generation
	modifyReg16(&FLEXPWM0->MCTRL, PWM_MCTRL_IPOL_MASK, 0);

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

	//Setup fault protection to prevent FET shortage for the three phases
	//Setup AOI event 0, 1, 2 to generate event when logic A&B==1 (PWM_A & PWM_B)
	//Event 0
	modifyReg16(&AOI0->BFCRT[0].BFCRT01, 0, 0xaf00);	//Pass inverted PT0_AC and PT0_BC. Force PT0_CC and PT0_DC to 1. Force others to 0
	modifyReg16(&AOI0->BFCRT[0].BFCRT23, 0, 0x0000);	//Force all others to 0

	//Event 1
	modifyReg16(&AOI0->BFCRT[1].BFCRT01, 0, 0xaf00);	//Pass inverted PT0_AC and PT0_BC. Force PT0_CC and PT0_DC to 1. Force others to 0
	modifyReg16(&AOI0->BFCRT[1].BFCRT23, 0, 0x0000);	//Force all others to 0

	//Event 2
	modifyReg16(&AOI0->BFCRT[2].BFCRT01, 0, 0xaf00);	//Pass inverted PT0_AC and PT0_BC. Force PT0_CC and PT0_DC to 1. Force others to 0
	modifyReg16(&AOI0->BFCRT[2].BFCRT23, 0, 0x0000);	//Force all others to 0

	//Set That TRIG0 is PWM_A
	//Set that TRIG1 is PWM_B
	modifyReg16(&FLEXPWM0->SM[0].TCTRL, 0, PWM_TCTRL_PWAOT0(1) | PWM_TCTRL_PWBOT1(1));
	modifyReg16(&FLEXPWM0->SM[1].TCTRL, 0, PWM_TCTRL_PWAOT0(1) | PWM_TCTRL_PWBOT1(1));
	modifyReg16(&FLEXPWM0->SM[2].TCTRL, 0, PWM_TCTRL_PWAOT0(1) | PWM_TCTRL_PWBOT1(1));

	//Mux PWM_A and PWM_B outputs to corresponding AOI inputs
	modifyReg32(&INPUTMUX0->AOI0_MUX[0], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1b));	//Mux PWM0_SM0_OUT_TRIG0 (PWM0_A) to AOI input A0
	modifyReg32(&INPUTMUX0->AOI0_MUX[1], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1c));	//Mux PWM0_SM0_OUT_TRIG1 (PWM0_B) to AOI input B0
//	modifyReg32(&INPUTMUX0->AOI0_MUX[1], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1d));	//Mux PWM0_SM1_OUT_TRIG0 (PWM1_A) to AOI input B0

	modifyReg32(&INPUTMUX0->AOI0_MUX[4], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1d));	//Mux PWM0_SM1_OUT_TRIG0 (PWM1_A) to AOI input A1
	modifyReg32(&INPUTMUX0->AOI0_MUX[5], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1e));	//Mux PWM0_SM1_OUT_TRIG1 (PWM1_B) to AOI input B1
//	modifyReg32(&INPUTMUX0->AOI0_MUX[4], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1c));	//Mux PWM0_SM0_OUT_TRIG1 (PWM0_B) to AOI input A1

	modifyReg32(&INPUTMUX0->AOI0_MUX[8], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x1f));	//Mux PWM0_SM2_OUT_TRIG0 (PWM2_A) to AOI input A2
	modifyReg32(&INPUTMUX0->AOI0_MUX[9], INPUTMUX_AOI0_MUXA_AOI0_MUX_INP_MASK, INPUTMUX_AOI0_MUXA_AOI0_MUX_INP(0x20));	//Mux PWM0_SM2_OUT_TRIG1 (PWM2_B) to AOI input B2

	//Mux AOI event output 0, 1 and 2 to corresponding PWM fault input
	modifyReg32(&INPUTMUX0->FLEXPWM0_FAULT0, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN_MASK, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN(0x2));	//Mux AOI0_OUT0 (event 0) to PWM fault 0
	modifyReg32(&INPUTMUX0->FLEXPWM0_FAULT1, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN_MASK, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN(0x3));	//Mux AOI0_OUT1 (event 1) to PWM fault 1
	modifyReg32(&INPUTMUX0->FLEXPWM0_FAULT2, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN_MASK, INPUTMUX_FLEXPWM0_FAULT0_TRIGIN(0x4));	//Mux AOI0_OUT2 (event 2) to PWM fault 2

	//Set that a fault level of logic 1 indicates fault condition
	//Enable automatic fault clearing
	modifyReg16(&FLEXPWM0->FCTRL, PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK,
			PWM_FCTRL_FLVL(0x7) | PWM_FCTRL_FAUTO(0x7));

	//Set that PWM outputs are re-enabled at the start of a full cycle
	//Also clear the fault flags
	modifyReg16(&FLEXPWM0->FSTS, PWM_FSTS_FHALF_MASK | PWM_FSTS_FFULL_MASK,
			PWM_FSTS_FFULL(0x7) | PWM_FSTS_FFLAG_MASK);

	//Enable map the corresponding fault input to disable PWM_A and PWM_B in each DISMAP register
	//All PWM outputs will go into fault when either of the fault inputs assert.
//	modifyReg16(&FLEXPWM0->SM[0].DISMAP[0], PWM_DISMAP_DIS0X_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0A_MASK,
//		PWM_DISMAP_DIS0A(0x7) | PWM_DISMAP_DIS0B(0x0));
//	modifyReg16(&FLEXPWM0->SM[1].DISMAP[0], PWM_DISMAP_DIS0X_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0A_MASK,
//			PWM_DISMAP_DIS0A(0x0) | PWM_DISMAP_DIS0B(0x0));
//	modifyReg16(&FLEXPWM0->SM[2].DISMAP[0], PWM_DISMAP_DIS0X_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0A_MASK,
//			PWM_DISMAP_DIS0A(0x0) | PWM_DISMAP_DIS0B(0x0));

	//Enable all six PWM outputs
	FLEXPWM0->OUTEN = PWM_OUTEN_PWMA_EN_MASK | PWM_OUTEN_PWMB_EN_MASK;

	//Test if fault works
//		modifyReg16(&FLEXPWM0->SWCOUT,
//					PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM0OUT45_MASK,
//					PWM_SWCOUT_SM0OUT23(1) | PWM_SWCOUT_SM0OUT45(0));
//		modifyReg16(&FLEXPWM0->SWCOUT,
//				PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM1OUT45_MASK,
//				PWM_SWCOUT_SM1OUT23(1) | PWM_SWCOUT_SM1OUT45(0));
//
//		//Then set SM0SEL23 and SM0SEL45 to SWCOUT in DTSRCSEL
//		modifyReg16(&FLEXPWM0->DTSRCSEL,
//				PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM1SEL45_MASK,
//				PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM1SEL45(2));
//		modifyReg16(&FLEXPWM0->DTSRCSEL,
//				PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM0SEL45_MASK,
//				PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM0SEL45(2));

		//Set PWM to independent mode
//		modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_INDEP_MASK, PWM_CTRL2_INDEP(1));
//		modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_INDEP_MASK, PWM_CTRL2_INDEP(1));

		//Force update the PWM submodule 1
//		modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
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
