/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "peripherals.h"

void initCorePeripherals(void)
{
    SystemClock_Config();

    initGPIO();

//    GPIO3->PTOR = (1 << 27);	//ENC_A

	initFlexPWM();

#ifndef USE_ADC_INPUT
	initDshotPWMTimer();
	initDMA_DshotPWM();
#endif
#ifndef BRUSHED_MODE
	initComTimer();
#endif
	initIntervalTimer();
	initSystickTimer();
	initTenKHzTimer();

#ifdef USE_ADC
	initADC();
	initDMA_ADC();
#endif

	initComp0();
	initComp1();

#ifdef USE_SERIAL_TELEMETRY
	telem_UART_Init();
	initDMA_UART();
#endif

//	GPIO3->PTOR = (1 << 27);	//ENC_A

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//TODO check if this is needed or what it does

}

void initAfterJump()
{
    __enable_irq();
}

/*
 * @brief 	Configures the core voltage to 1.1V. Sets the Fast Internal Reference Clock (FIRC) to 192MHz.
 * 			Sets MUX to select FIRC as MAIN_CLK. Sets system clock divider to /2, so CPU_CLK and SYSTEM_CLK are 96MHz.
 * 			Also enables SIRC so 12MHz and 1MHz can be sourced to peripherals.
 */
void SystemClock_Config(void)
{
	//Set VDD_CORE voltage level to 1.1V to set it in Standard Drive mode
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_LVL(2));

	//Set VDD_CORE drive strength to normal (1.1V)
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_DS(1));

	//Wait for the SPC to finish its transition to 1.1V, i.e. SPC is not busy
	while ((SPC0->SC & SPC_SC_BUSY_MASK) >> SPC_SC_BUSY_SHIFT) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Set flash memory to support higher voltage level and frequency
	//This sets the number of additional wait-states.
	//These need to be set according to the set FIRC frequency:
	//48MHz = 0
	//64MHz = 0
	//96MHz = 1
	//192MHz = 2
	//Set to 192MHz
	modifyReg32(&FMU0->FCTRL, FMU_FCTRL_RWSC_MASK, FMU_FCTRL_RWSC(2));

	//Set SRAM to support higher voltage levels
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_VSM_MASK, SPC_SRAMCTL_VSM(2));

	//Request SRAM voltage update
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, SPC_SRAMCTL_REQ(1));

	//Wait for the SRAM voltage change to complete
	while (!((SPC0->SRAMCTL & SPC_SRAMCTL_ACK_MASK) >> SPC_SRAMCTL_ACK_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Clear the SRAM voltage update request
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, 0);

	//Set System clock divider to 2 to make the CPU and SYSTEM clock 96MHz (this is the max clock) divider value = DIV + 1
	modifyReg32(&SYSCON->AHBCLKDIV, SYSCON_AHBCLKDIV_DIV_MASK, SYSCON_AHBCLKDIV_DIV(1));

	/* Config FIRC */
	//Set the Fast Internal Reference Clock (FIRC) to 192MHz
	modifyReg32(&SCG0->FIRCCFG, SCG_FIRCCFG_FREQ_SEL_MASK, SCG_FIRCCFG_FREQ_SEL(7));

	//Unlock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_LK_MASK, 0);

	//Enable FRO_HF clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN(1));

	//Enable FIRC 48MHz clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN(1));

	//Set that FIRC is disabled when in deep sleep mode
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCSTEN_MASK, SCG_FIRCCSR_FIRCSTEN(0));

	//Wait for the FIRC clock source to be valid
	while (!((SCG0->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Select FIRC as MAIN_CLK clock source
	modifyReg32(&SCG0->RCCR, SCG_RCCR_SCS_MASK, SCG_RCCR_SCS(3));

	//Wait for the MAIN_CLK clock source mux to be set correctly
	while (((SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Enable FIRC clock source
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCEN_MASK, SCG_FIRCCSR_FIRCEN(1));

	//Lock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, 0, SCG_FIRCCSR_LK_MASK);

	/* Config SIRC */
	//Unlock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_LK_MASK, 0);

	//Set that SIRC is disabled in deep sleep mode
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRCSTEN_MASK, 0);

	//Enable SIRC clock to peripherals
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN(1));

	//Wait for SIRC clock source to be valid
	while (!((SCG0->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Lock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, 0, SCG_SIRCCSR_LK_MASK);
}

/*
 * @brief 	Enables all GPIO and PORT peripherals
 */
void initGPIO(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable PORT peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT1(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_PORT3(1);

	//Enable GPIO peripheral clocks
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO0(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO1(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO3(1);

	//Enable INPUTMUX peripheral clock
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Release PORT peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT1(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3(1);

	//Release GPIO peripherals from reset
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO0(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO1(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO3(1);

	//Release INPUTMUX peripheral from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Enable GPIO pins for testing/debugging. P3.27, P2.17, P3.28. Set them to output
	modifyReg32(&PORT3->PCR[27],	//ENC_A
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 27));
	GPIO3->PCOR = (1 << 27);

	//pin 12, 13 and 14 cannot be used as these are occupied by USB FS on the MCXA14x and MCXA15x
//	modifyReg32(&PORT2->PCR[17],	//ENC_B
//			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
//	modifyReg32(&GPIO2->PDDR, 0, (1 << 17));
//	GPIO2->PCOR = (1 << 17);

	modifyReg32(&PORT3->PCR[28],	//ENC_I
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 28));
	GPIO3->PCOR = (1 << 28);
}

void enableCorePeripherals()
{
	//First enable SysTick timer as its used for delay functions
	enableSystickTimer();

	//Enable PWM
	enableFlexPWM();

//	//Unlock clock configuration registers access
//	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);
//
//	modifyReg32(&MRCC0->MRCC_CLKOUT_CLKSEL, MRCC_MRCC_CLKOUT_CLKSEL_MUX_MASK, MRCC_MRCC_CLKOUT_CLKSEL_MUX(0));
//
//	modifyReg32(&MRCC0->MRCC_CLKOUT_CLKDIV,
//			MRCC_MRCC_CLKOUT_CLKDIV_HALT_MASK | MRCC_MRCC_CLKOUT_CLKDIV_DIV_MASK,
//			MRCC_MRCC_CLKOUT_CLKDIV_DIV(9));
//
//	//Freeze clock configuration registers access
//	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));
//
//	//Enable CLKOUT on P3_6
//	modifyReg32(&PORT3->PCR[6],
//			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(1));
//
//	//Enable CLKOUT on P3_8
//	modifyReg32(&PORT3->PCR[8],
//			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(12));

	//Enable the timers
#ifndef USE_ADC_INPUT
	enableDMA_DshotPWM();
	enableDshotPWMTimer();
//	GPIO3->PTOR = (1 << 27);	//ENC_A
#endif

#ifndef BRUSHED_MODE
	enableComTimer();
#endif
	enableTenKHzTimer();
	enableIntervalTimer();

	//Enable ADC
#ifdef USE_ADC
	enableADC();
	enableDMA_ADC();
#endif

	//Enable UART DMA
#ifdef USE_SERIAL_TELEMETRY
	enableDMA_UART();
	enable_telem_UART();
#endif
}
