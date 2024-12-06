/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

//#include "common.h"
//#include "dshot.h"
//#include "functions.h"
//#include "serial_telemetry.h"
//#include "targets.h"
//#include "DMA.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 4);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

void receiveDshotDma()
{
	//TODO put in our own dshot timer + DMA config

	//Set pin 1.2 to CT_INP0 so it can be used as the Dshot/PWM input
	//Enable input buffer and pull-down resistor
//	modifyReg32(&PORT1->PCR[2],
//			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(5) | PORT_PCR_IBE(1) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
//
//	//Enable Dshot DMA
//	enableDMA_DshotPWM();

//	out_put = 0;
//    IC_TIMER_REGISTER->CNT = 0;
//#ifdef USE_TIMER_3_CHANNEL_1
//    DMA1_Channel4->CMAR = (uint32_t)&dma_buffer;
//    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
//    DMA1_Channel4->CNDTR = buffersize;
//    DMA1_Channel4->CCR = 0x98b;
//#endif
//#ifdef USE_TIMER_15_CHANNEL_1
//    DMA1_Channel5->CMAR = (uint32_t)&dma_buffer;
//    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
//    DMA1_Channel5->CNDTR = buffersize;
//    DMA1_Channel5->CCR = 0x98b;
//#endif
//    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
//    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
//    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma()
{
	//TODO put in our own SPI dshot timer + DMA config
	//Change Dshot pin to SPI

	//Get calculated GCR data to send

	//Send GCR data to the SPI FIFO


//    out_put = 1;
//#ifdef USE_TIMER_3_CHANNEL_1
//    //          // de-init timer 2
//    RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
//    RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
//#endif
//#ifdef USE_TIMER_15_CHANNEL_1
//    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
//    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
//#endif
//    IC_TIMER_REGISTER->CCMR1 = 0x60;
//    IC_TIMER_REGISTER->CCER = 0x3;
//    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
//    IC_TIMER_REGISTER->ARR = 115;
//
//    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
//#ifdef USE_TIMER_3_CHANNEL_1
//    DMA1_Channel4->CMAR = (uint32_t)&gcr;
//    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
//    DMA1_Channel4->CNDTR = 23 + buffer_padding;
//    DMA1_Channel4->CCR = 0x99b;
//#endif
//#ifdef USE_TIMER_15_CHANNEL_1
//    //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
//    //(uint32_t)&gcr, (uint32_t)&IC_TIMER_REGISTER->CCR1,
//    // LL_DMA_GetDataTransferDirection(DMA1,
//    // INPUT_DMA_CHANNEL));
//    DMA1_Channel5->CMAR = (uint32_t)&gcr;
//    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
//    DMA1_Channel5->CNDTR = 23 + buffer_padding;
//    DMA1_Channel5->CCR = 0x99b;
//#endif
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
	modifyReg32(&INPUT_PIN_GPIO->PDDR, GPIO_PDDR_PDD0(1 << INPUT_PIN), 0);

	//Read INPUT_PIN value
	uint8_t readPinData = INPUT_PIN_GPIO->PDR[INPUT_PIN];

	//Set pin 1.2 to CT_INP0 so it can be used as the Dshot/PWM input
	//Enable input buffer and pull-down resistor
	modifyReg32(&PORT1->PCR[2],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(5) | PORT_PCR_IBE(1) | PORT_PCR_PE(1) | PORT_PCR_PS(0));

	return readPinData;
}

//void setInputPolarityRising()
//{
//    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
//        LL_TIM_IC_POLARITY_RISING);
//}

void setInputPullDown()
{
	//Enable internal pull resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PE_MASK, PORT_PCR_PE(1));

	//Set internal resistor to pull-down
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PS_MASK, PORT_PCR_PS(0));
}

void setInputPullUp()
{
	//Enable internal pull resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PE_MASK, PORT_PCR_PE(1));

	//Set internal resistor to pull-up
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PS_MASK, PORT_PCR_PS(1));
}

//void enableHalfTransferInt() {
//	LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
//}

void setInputPullNone()
{
	//Disable internal pull resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PE_MASK, 0);
}
