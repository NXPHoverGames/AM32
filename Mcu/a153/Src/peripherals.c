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

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//TODO check if this is needed or what it does

}

void initAfterJump()
{
    __enable_irq();
}

/*
 * @brief 	Configures the core voltage to 1.1V. Sets the Fast Internal Reference Clock (FIRC) to 96MHz.
 * 			Sets MUX to select FIRC as MAIN_CLK. Sets system clock divider to /1, so CPU_CLK and SYSTEM_CLK are 96MHz.
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

	//Set System clock divider to 2 to make the CPU and SYSTEM clock 96MHz (this is the max clock)
	modifyReg32(&SYSCON->AHBCLKDIV, SYSCON_AHBCLKDIV_DIV_MASK, SYSCON_AHBCLKDIV_DIV(2));

	//Unlock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_LK_MASK, 0);

	//Set the Fast Internal Reference Clock (FIRC) to 192MHz
	modifyReg32(&SCG0->FIRCCFG, SCG_FIRCCFG_FREQ_SEL_MASK, SCG_FIRCCFG_FREQ_SEL(7));

	//Enable FRO_HF clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN(1));

	//Enable FIRC 48MHz clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN(1));

	//Set that FIRC is disabled when in deep sleep mode
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCSTEN_MASK, SCG_FIRCCSR_FIRCSTEN(0));

	//Select FIRC as MAIN_CLK clock source
	modifyReg32(&SCG0->RCCR, SCG_RCCR_SCS_MASK, SCG_RCCR_SCS(3));

	//Wait for the MAIN_CLK clock source mux to be set correctly
	while (((SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Enable FIRC clock source
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCEN_MASK, SCG_FIRCCSR_FIRCEN(1));

	//Wait for the FIRC clock source to be valid
	while (!((SCG0->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Lock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, 0, SCG_FIRCCSR_LK_MASK);
}

/*
 * @brief 	Enables all GPIO and PORT peripherals
 */
void initGPIO(void)
{
	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT1(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_PORT3(1);

	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO0(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO1(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO3(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT1(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3(1);

	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO0(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO1(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO3(1);

	//Enable GPIO pins for testing/debugging. P3.15, P3.14, P2.6. Set them to output
	modifyReg32(&PORT2->PCR[6],
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO2->PDDR, 0, (1 << 6));
	GPIO2->PCOR = (1 << 6);

	modifyReg32(&PORT2->PCR[13],
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO2->PDDR, 0, (1 << 13));
	GPIO2->PCOR = (1 << 13);

	modifyReg32(&PORT3->PCR[14],
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 14));
	GPIO3->PCOR = (1 << 14);

	modifyReg32(&PORT3->PCR[15],
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 15));
	GPIO3->PCOR = (1 << 15);

}

/*
 * @brief 	This is the timer used for the Dshot/PWM
 */
void UN_TIM_Init(void)
{
//    // LL_TIM_InitTypeDef TIM_InitStruct = {0};
//
//    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//
//    /* Peripheral clock enable */
//#ifdef USE_TIMER_15_CHANNEL_1
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);
//  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
//    /**TIM16 GPIO Configuration
//    PA6   ------> TIM16_CH1
//    */
//    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
//    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//#endif
//
//#ifdef USE_TIMER_3_CHANNEL_1
//
//    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
//    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
//    /**TIM16 GPIO Configuration
//    PA6   ------> TIM16_CH1
//    */
//    GPIO_InitStruct.Pin = INPUT_PIN;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//#endif
//
//  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_7);
//
//  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//
//  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);
//
//  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);
//
//  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
//
//  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
//
//  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_HALFWORD);
//
//  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_WORD);
//
//#ifdef USE_TIMER_15_CHANNEL_1
//    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
//    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
//#endif
//#ifdef USE_TIMER_3_CHANNEL_1
//    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
//    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
//#endif
//
//    // TIM_InitStruct.Prescaler = 0;
//    // TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//    // TIM_InitStruct.Autoreload = 63;
//    // TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//    // TIM_InitStruct.RepetitionCounter = 0;
//    //  LL_TIM_Init(IC_TIMER_REGISTER, &TIM_InitStruct);
//    IC_TIMER_REGISTER->PSC = 0;
//    IC_TIMER_REGISTER->ARR = 63;
//
//    // LL_TIM_DisableARRPreload(IC_TIMER_REGISTER);
//    // LL_TIM_IC_SetActiveInput(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
//    // LL_TIM_ACTIVEINPUT_DIRECTTI); LL_TIM_IC_SetPrescaler(IC_TIMER_REGISTER,
//    // IC_TIMER_CHANNEL, LL_TIM_ICPSC_DIV1);
//    // LL_TIM_IC_SetFilter(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
//    // LL_TIM_IC_FILTER_FDIV1); LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER,
//    // IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_BOTHEDGE);
}

void enableCorePeripherals()
{
	//First enable SysTick timer as its used for delay functions
	enableSystickTimer();

	//Enable PWM
	enableFlexPWM();

	//Enable the timers
#ifndef USE_ADC_INPUT
	enableDshotPWMTimer();
	enableDMA_DshotPWM();
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

	//Enable comparators
//	enableComparator();

	//Enable UART DMA
#ifdef USE_SERIAL_TELEMETRY
	enableDMA_UART();
#endif
}
