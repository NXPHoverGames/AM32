/*
 * phaseouts.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */

#include "phaseouts.h"

extern char prop_brake_active;

#ifndef PWM_ENABLE_BRIDGE

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

/*
 * @brief 	Sets high FET output pins to LOW and enables PWM on the low FET output pins.
 * 			This causes the motor to brake and the brake strength is then controlled by the duty cycle.
 */
void proportionalBrake()
{
	//Set high FET pins to GPIO
	modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));
	modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));
	modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pins as output
	modifyReg32(&PHASE_A_GPIO_HIGH->PDDR, 0, (1 << PHASE_A_PIN_HIGH));
	modifyReg32(&PHASE_B_GPIO_HIGH->PDDR, 0, (1 << PHASE_B_PIN_HIGH));
	modifyReg32(&PHASE_C_GPIO_HIGH->PDDR, 0, (1 << PHASE_C_PIN_HIGH));

	//Set high FET output pins to LOW
	PHASE_A_GPIO_HIGH->PCOR = (1 << PHASE_A_PIN_HIGH);
	PHASE_B_GPIO_HIGH->PCOR = (1 << PHASE_B_PIN_HIGH);
	PHASE_C_GPIO_HIGH->PCOR = (1 << PHASE_C_PIN_HIGH);

	//Set low FET output pins to PWM, duty cycle will now control braking
	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
}

/*
 * @brief 	If the PWM is configured as complementary it will enable complementary PWM on the low and high FET output.
 * 			If it is not configured as complementary PWM, then the low FET output is pulled LOW and the PWM is enabled
 * 			on the high FET output.
 */
void phaseBPWM()
{
    if (!eepromBuffer.comp_pwm) {
    	//Set low FET pin to GPIO
    	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

    	//Set low FET pin as output
    	modifyReg32(&PHASE_B_GPIO_LOW->PDDR, 0, (1 << PHASE_B_PIN_LOW));

    	//Set low FET output to LOW
    	PHASE_B_GPIO_LOW->PCOR = (1 << PHASE_B_PIN_LOW);
    } else {
    	//Set low FET pin to PWM
    	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
    }

    //Set high FET pin to PWM
    modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

	//Force update the PWM submodule 1
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

/*
 * @brief 	Sets the low and high FET output to LOW
 */
void phaseBFLOAT()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_B_GPIO_LOW->PDDR, 0, (1 << PHASE_B_PIN_LOW));

	//Set low FET output to LOW
	PHASE_B_GPIO_LOW->PCOR = (1 << PHASE_B_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_B_GPIO_HIGH->PDDR, 0, (1 << PHASE_B_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_B_GPIO_HIGH->PCOR = (1 << PHASE_B_PIN_HIGH);
}

/*
 * @brief 	Sets the high FET output to LOW and the low FET output to HIGH.
 * 			This causes the corresponding phase to be pulled to LOW.
 */
void phaseBLOW()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_B_GPIO_LOW->PDDR, 0, (1 << PHASE_B_PIN_LOW));

	//Set low FET output to HIGH
	PHASE_B_GPIO_LOW->PSOR = (1 << PHASE_B_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_B_GPIO_HIGH->PDDR, 0, (1 << PHASE_B_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_B_GPIO_HIGH->PCOR = (1 << PHASE_B_PIN_HIGH);
}

/*
 * @brief 	If the PWM is configured as complementary it will enable complementary PWM on the low and high FET output.
 * 			If it is not configured as complementary PWM, then the low FET output is pulled LOW and the PWM is enabled
 * 			on the high FET output.
 */
void phaseCPWM()
{
    if (!eepromBuffer.comp_pwm) {
    	//Set low FET pin to GPIO
    	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

    	//Set low FET pin as output
    	modifyReg32(&PHASE_C_GPIO_LOW->PDDR, 0, (1 << PHASE_C_PIN_LOW));

    	//Set low FET output to LOW
    	PHASE_C_GPIO_LOW->PCOR = (1 << PHASE_C_PIN_LOW);
    } else {
    	//Set low FET pin to PWM
    	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
    }

    //Set high FET pin to PWM
    modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

	//Force update the PWM submodule 2
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

/*
 * @brief 	Sets the low and high FET output to LOW
 */
void phaseCFLOAT()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_C_GPIO_LOW->PDDR, 0, (1 << PHASE_C_PIN_LOW));

	//Set low FET output to LOW
	PHASE_C_GPIO_LOW->PCOR = (1 << PHASE_C_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_C_GPIO_HIGH->PDDR, 0, (1 << PHASE_C_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_C_GPIO_HIGH->PCOR = (1 << PHASE_C_PIN_HIGH);
}

/*
 * @brief 	Sets the high FET output to LOW and the low FET output to HIGH.
 * 			This causes the corresponding phase to be pulled to LOW.
 */
void phaseCLOW()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_C_GPIO_LOW->PDDR, 0, (1 << PHASE_C_PIN_LOW));

	//Set low FET output to HIGH
	PHASE_C_GPIO_LOW->PSOR = (1 << PHASE_C_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_C_GPIO_HIGH->PDDR, 0, (1 << PHASE_C_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_C_GPIO_HIGH->PCOR = (1 << PHASE_C_PIN_HIGH);
}

/*
 * @brief 	If the PWM is configured as complementary it will enable complementary PWM on the low and high FET output.
 * 			If it is not configured as complementary PWM, then the low FET output is pulled LOW and the PWM is enabled
 * 			on the high FET output.
 */
void phaseAPWM()
{
    if (!eepromBuffer.comp_pwm) {
    	//Set low FET pin to GPIO
    	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

    	//Set low FET pin as output
    	modifyReg32(&PHASE_A_GPIO_LOW->PDDR, 0, (1 << PHASE_A_PIN_LOW));

    	//Set low FET output to LOW
    	PHASE_A_GPIO_LOW->PCOR = (1 << PHASE_A_PIN_LOW);
    } else {
    	//Set low FET pin to PWM
    	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
    }

    //Set high FET pin to PWM
    modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

	//Force update the PWM submodule 0
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

/*
 * @brief 	Sets the low and high FET output to LOW
 */
void phaseAFLOAT()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_A_GPIO_LOW->PDDR, 0, (1 << PHASE_A_PIN_LOW));

	//Set low FET output to LOW
	PHASE_A_GPIO_LOW->PCOR = (1 << PHASE_A_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_A_GPIO_HIGH->PDDR, 0, (1 << PHASE_A_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_A_GPIO_HIGH->PCOR = (1 << PHASE_A_PIN_HIGH);
}

/*
 * @brief 	Sets the high FET output to LOW and the low FET output to HIGH.
 * 			This causes the corresponding phase to be pulled to LOW.
 */
void phaseALOW()
{
	//Set low FET pin to GPIO
	modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set low FET pin as output
	modifyReg32(&PHASE_A_GPIO_LOW->PDDR, 0, (1 << PHASE_A_PIN_LOW));

	//Set low FET output to HIGH
	PHASE_A_GPIO_LOW->PSOR = (1 << PHASE_A_PIN_LOW);

	//Set high FET pin to GPIO
	modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(0));

	//Set high FET pin as output
	modifyReg32(&PHASE_A_GPIO_HIGH->PDDR, 0, (1 << PHASE_A_PIN_HIGH));

	//Set high FET output to LOW
	PHASE_A_GPIO_HIGH->PCOR = (1 << PHASE_A_PIN_HIGH);
}

#else

//////////////////////////////////PHASE 1//////////////////////
void phaseBPWM()
{
//    if (!eepromBuffer.comp_pwm) { // for future
//                     // LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW,
//                     // PHASE_B_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
//                     // PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
//    } else {
//        LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
//            LL_GPIO_MODE_OUTPUT); // enable on
//        PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
//    }
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
//        LL_GPIO_MODE_ALTERNATE); // high pwm
}

void phaseBFLOAT()
{
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable off
//    PHASE_B_GPIO_PORT_ENABLE->BRR = PHASE_B_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT); // pwm off
//    PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
}

void phaseBLOW()
{
//    // low mosfet on
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable on
//    PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT); // pwm off
//    PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
//    if (!eepromBuffer.comp_pwm) {
//        //	LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
//        // LL_GPIO_MODE_OUTPUT); PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF =
//        // PHASE_C_GPIO_LOW;
//    } else {
//        LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
//            LL_GPIO_MODE_OUTPUT); // enable on
//        PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
//    }
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
//        LL_GPIO_MODE_ALTERNATE);
}

void phaseCFLOAT()
{
    // floating
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable off
//    PHASE_C_GPIO_PORT_ENABLE->BRR = PHASE_C_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
}

void phaseCLOW()
{
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable on
//    PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
//    if (!eepromBuffer.comp_pwm) {
//        //	LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
//        // LL_GPIO_MODE_OUTPUT); PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF =
//        // PHASE_A_GPIO_LOW;
//    } else {
//        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
//            LL_GPIO_MODE_OUTPUT); // enable on
//        PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
//    }
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
//        LL_GPIO_MODE_ALTERNATE);
}

void phaseAFLOAT()
{
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable on
//    PHASE_A_GPIO_PORT_ENABLE->BRR = PHASE_A_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
}

void phaseALOW()
{
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
//        LL_GPIO_MODE_OUTPUT); // enable on
//    PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
//        LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
}

#endif

void allOff()
{
    phaseAFLOAT();
    phaseBFLOAT();
    phaseCFLOAT();
}

void comStep(char newStep)
{
//	//Toggle P3.14 output
//	GPIO3->PTOR = (1 << 14);

    switch (newStep) {
    case 1: // A-B
        phaseCFLOAT();
        phaseBLOW();
        phaseAPWM();

//        GPIO2->PSOR = (1 << 6);
//        GPIO3->PCOR = (1 << 14);
//        GPIO3->PCOR = (1 << 15);
        break;

    case 2: // C-B
        phaseAFLOAT();
        phaseBLOW();
        phaseCPWM();

//        GPIO2->PCOR = (1 << 6);
//        GPIO3->PSOR = (1 << 14);
//        GPIO3->PCOR = (1 << 15);
        break;

    case 3: // C-A
        phaseBFLOAT();
        phaseALOW();
        phaseCPWM();

//        GPIO2->PSOR = (1 << 6);
//        GPIO3->PSOR = (1 << 14);
//        GPIO3->PCOR = (1 << 15);
        break;

    case 4: // B-A
        phaseCFLOAT();
        phaseALOW();
        phaseBPWM();

//        GPIO2->PCOR = (1 << 6);
//        GPIO3->PCOR = (1 << 14);
//        GPIO3->PSOR = (1 << 15);
        break;

    case 5: // B-C
        phaseAFLOAT();
        phaseCLOW();
        phaseBPWM();

//        GPIO2->PSOR = (1 << 6);
//        GPIO3->PCOR = (1 << 14);
//        GPIO3->PSOR = (1 << 15);
        break;

    case 6: // A-C
        phaseBFLOAT();
        phaseCLOW();
        phaseAPWM();

//        GPIO2->PCOR = (1 << 6);
//        GPIO3->PSOR = (1 << 14);
//        GPIO3->PSOR = (1 << 15);
        break;
    }
}

void fullBrake()
{ // full braking shorting all low sides
    phaseALOW();
    phaseBLOW();
    phaseCLOW();
}

void allpwm()
{ // for stepper_sine
    phaseAPWM();
    phaseBPWM();
    phaseCPWM();
}

void twoChannelForward()
{
    phaseAPWM();
    phaseBLOW();
    phaseCPWM();
}

void twoChannelReverse()
{
    phaseALOW();
    phaseBPWM();
    phaseCLOW();
}
