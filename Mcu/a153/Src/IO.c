/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 4);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

void receiveDshotDma()
{
	//TODO put in our own dshot timer + DMA config

	out_put = 0;

	//Set prescaler
//	CTIMER0->PR = ic_timer_prescaler;

	//Resets PWM/Dshot timer to 0
	resetInputCaptureTimer();
//	enableDshotPWMTimer();

//	Set PWM/Dshot input pin to timer capture/compare input
//	And enable input buffer and pull-up resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1));

	//Enable Dshot DMA
	//enables interrupt and hardware request
	//sets buffersize and source and destination address
	enableDMA_DshotPWM();
}

void sendDshotDma()
{
	//TODO put in our own SPI dshot timer + DMA config
	//Change Dshot pin to SPI

	//Get calculated GCR data to send

	//Send GCR data to the SPI FIFO


//    out_put = 1;

//    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
//    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;

//    IC_TIMER_REGISTER->CCMR1 = 0x60;
//    IC_TIMER_REGISTER->CCER = 0x3;
//    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
//    IC_TIMER_REGISTER->ARR = 115;
//
//    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;

//    DMA1_Channel5->CMAR = (uint32_t)&gcr;
//    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
//    DMA1_Channel5->CNDTR = 23 + buffer_padding;
//    DMA1_Channel5->CCR = 0x99b;

//    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
//    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
//    IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
//    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState()
{
	//Set INPUT_PIN to a GPIO pin
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_IBE(1) | PORT_PCR_PE(1) | PORT_PCR_PS(0));

	//Set INPUT_PIN to input
	modifyReg32(&INPUT_PIN_GPIO->PDDR, (1 << INPUT_PIN), 0);

	//Read INPUT_PIN value
	uint8_t readPinData = INPUT_PIN_GPIO->PDR[INPUT_PIN];

	//Set PWM/Dshot input pin to timer capture/compare input
	//And enable input buffer and pull-up resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1));

	return readPinData;
}

//void setInputPolarityRising()
//{
//    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
//        LL_TIM_IC_POLARITY_RISING);
//}

void setInputPullDown()
{
	//Enable internal pull-down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_PE(1) | PORT_PCR_PS(0));
}

void setInputPullUp()
{
	//Enable internal pull-up resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_PE(1) | PORT_PCR_PS(1));
}

//void enableHalfTransferInt() {
//	LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
//}

void setInputPullNone()
{
	//Disable internal pull resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PE_MASK, 0);
}
