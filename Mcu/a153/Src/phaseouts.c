/*
 * phaseouts.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */

#include "phaseouts.h"

extern char prop_brake_active;

//TODO put our PCOR and PSOR in to set our pins accordingly
#ifdef USE_INVERTED_LOW
#pragma message("using inverted low side output")
#define LOW_BITREG_ON BRR
#define LOW_BITREG_OFF BSRR
#else
#define LOW_BITREG_ON BSRR
#define LOW_BITREG_OFF BRR
#endif

#ifdef USE_INVERTED_HIGH
#pragma message("using inverted high side output")
// #define HIGH_BITREG_ON  BRR
#define HIGH_BITREG_OFF BSRR
#else
// #define HIGH_BITREG_ON  BSRR
#define HIGH_BITREG_OFF BRR
#endif

//#define INVERT_PWM

//#define FORCE_ALL_AT_ONCE
//#define TEST_FOR_LESS_NOISE

void phaseAPWM()
{
	//Select inverted PWM23 for phase A
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK, PWM_DTSRCSEL_SM0SEL23(1));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseAFLOAT()
{
	//Mask phase A to float
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x110);

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseALOW()
{
	//Set phase A low FET to HIGH
	modifyReg16(&FLEXPWM0->SWCOUT,
			PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
			PWM_SWCOUT_SM0OUT23(1) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(0));

	//Select SWCOUT for phase A
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK, PWM_DTSRCSEL_SM0SEL23(2));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseBPWM()
{
	//Select inverted PWM23 for phase B
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM1SEL23_MASK, PWM_DTSRCSEL_SM1SEL23(1));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseBFLOAT()
{
	//Mask phase B to float
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x220);

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseBLOW()
{
	//Set phase B low FET to HIGH
	modifyReg16(&FLEXPWM0->SWCOUT,
			PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
			PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(1) | PWM_SWCOUT_SM2OUT23(0));

	//Select SWCOUT for phase B
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM1SEL23_MASK, PWM_DTSRCSEL_SM1SEL23(2));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseCPWM()
{
	//Select inverted PWM23 for phase C
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM2SEL23_MASK, PWM_DTSRCSEL_SM2SEL23(1));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseCFLOAT()
{
	//Mask phase C to float
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x440);

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

void phaseCLOW()
{
	//Set phase C low FET to HIGH
	modifyReg16(&FLEXPWM0->SWCOUT,
			PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
			PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(1));

	//Select SWCOUT for phase C
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM2SEL23_MASK, PWM_DTSRCSEL_SM2SEL23(2));

#ifndef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif
}

/*
 * @brief 	Sets high FET output pins to LOW and enables PWM on the low FET output pins.
 * 			This causes the motor to brake and the brake strength is then controlled by the duty cycle.
 */
void proportionalBrake()
{
	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, 0xfff,
			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, 0xfff,
			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

	//Mask high FETs so its LOW, PWM_B are the high FETS
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x070);

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void allOff()
{
	//A FLOAT, B FLOAT, C FLOAT

	//Mask all phases
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x770);

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void comStep(char newStep)
{
	GPIO3->PSOR = (1 << 27); 	//ENC_A

#ifdef TEST_FOR_LESS_NOISE
	//Mask all phases
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x0);

#ifdef FORCE_ALL_AT_ONCE
	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif

	switch (newStep) {
	    case 1: // A-B
	        phaseCFLOAT();
	        phaseBLOW();
	        phaseAPWM();
	        break;

	    case 2: // C-B
	        phaseAFLOAT();
	        phaseBLOW();
	        phaseCPWM();
	        break;

	    case 3: // C-A
	        phaseBFLOAT();
	        phaseALOW();
	        phaseCPWM();
	        break;

	    case 4: // B-A
	        phaseCFLOAT();
	        phaseALOW();
	        phaseBPWM();
	        break;

	    case 5: // B-C
	        phaseAFLOAT();
	        phaseCLOW();
	        phaseBPWM();
	        break;

	    case 6: // A-C
	        phaseBFLOAT();
	        phaseCLOW();
	        phaseAPWM();
	        break;
	    }

	#ifdef FORCE_ALL_AT_ONCE
		//Force out event
		modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
	#endif

#else

    switch (newStep) {
    case 1: // A-B
    	//A PWM, B LOW, C FLOAT

    	//Set phase B low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(1) | PWM_SWCOUT_SM2OUT23(0));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#endif

    	//Mask phase C to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x440);
        break;

    case 2: // C-B
    	//A FLOAT, B LOW, C PWM

    	//Set phase B low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(1) | PWM_SWCOUT_SM2OUT23(0));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

    	//Mask phase to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x110);
        break;

    case 3: // C-A
    	//A LOW, B FLOAT, C PWM

    	//Set phase A low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(1) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(0));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

    	//Mask phase to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x220);
        break;

    case 4: // B-A
    	//A LOW, B PWM, C FLOAT

    	//Set phase A low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(1) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(0));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(0));
#endif

    	//Mask phase to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x440);
        break;

    case 5: // B-C
    	//A FLOAT, B PWM, C LOW

    	//Set phase C low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(1));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

    	//Mask phase to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x110);
        break;

    case 6: // A-C
    	//A PWM, B FLOAT, C LOW

    	//Set phase C low FET to HIGH
//    	modifyReg16(&FLEXPWM0->SWCOUT,
//    				PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
//    				PWM_SWCOUT_SM0OUT23(0) | PWM_SWCOUT_SM1OUT23(0) | PWM_SWCOUT_SM2OUT23(1));

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

    	//Mask phase to float
    	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x220);
        break;
    }

    //Force out event
    modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
#endif

    //TODO remove this
    GPIO3->PCOR = (1 << 27); 	//ENC_A
}

void fullBrake()
{ // full braking shorting all low sides

	//Set all PWM_A SWCOUT to high so LOW FET is enabled
	modifyReg16(&FLEXPWM0->SWCOUT,
			PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK,
			PWM_SWCOUT_SM0OUT23(1) | PWM_SWCOUT_SM1OUT23(1) | PWM_SWCOUT_SM2OUT23(1));

	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

//TODO the following are never called
void allpwm()
{ // for stepper_sine
//    phaseAPWM();
//    phaseBPWM();
//    phaseCPWM();
}

void twoChannelForward()
{
//    phaseAPWM();
//    phaseBLOW();
//    phaseCPWM();
}

void twoChannelReverse()
{
//    phaseALOW();
//    phaseBPWM();
//    phaseCLOW();
}
