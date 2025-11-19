/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 8);	//Used to detect input type (Dshot300, Dshot600 and PWM400). Divide by 8 is best for MCXA153 at 96MHz CPU
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

void receiveDshotDma()
{
	out_put = 0;

	//Set prescaler
	CTIMER0->PR = ic_timer_prescaler;

	if (buffersize > 3) {
		//Resets PWM/Dshot timer to 0. Needed for Dshot to work properly.
		resetInputCaptureTimer();

		//Set match1 value to higher then the minimum Dshot300 frame time which is around 53us, so take at least 53us.
		CTIMER0->MR[1] = 10000 / (CTIMER0->PR + 1);

		//Reset timer and enable interrupt on Match1 event
		modifyReg32(&CTIMER0->MCR, 0, CTIMER_MCR_MR1I(1) | CTIMER_MCR_MR1R(1));

	} else {
		//Disable interrupt and reset on Match1 event
		modifyReg32(&CTIMER0->MCR, CTIMER_MCR_MR1I(1) | CTIMER_MCR_MR1R(1), 0);
	}

	//Set the major loop count and addresses again to prevent unintended DMA request from CTIMER match register
	//Set current and beginning major loop count to 8
	DMA0->CH[DMA_CH_DshotPWM].TCD_CITER_ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(buffersize / 2);

	//Sets the amount of major loop counts after a DMA transfer completes
	//i.e. the CITER register gets this BITER value after DMA transfer complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_BITER_ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(buffersize / 2);

	//Set last destination address adjustment to -8 bytes * the number of DMA requests before transfer complete
	//Adds this value to the destination address when CITER reaches 0 / DMA transfer is complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_DLAST_SGA = -(8 * (buffersize / 2));

	//Set source address
	DMA0->CH[DMA_CH_DshotPWM].TCD_SADDR = (uint32_t)&CTIMER0->CR[1];

	//Set destination address
	DMA0->CH[DMA_CH_DshotPWM].TCD_DADDR = (uint32_t)&dma_buffer;

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));
}

void sendDshotDma()
{
	//TODO put in our own SPI dshot timer + DMA config
	//Change Dshot pin to SPI output
//	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
//			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(2) | PORT_PCR_IBE(0) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

	//Set SPI bus prescaler according to output_timer_prescaler. 0 for Dshot600, 1 for Dshot300

	//Get calculated GCR data to send

	//Send GCR data to the SPI FIFO by enabling DMA transfer

    out_put = 1;

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
			PORT_PCR_MUX(0) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

	//Set INPUT_PIN to input
//	modifyReg32(&INPUT_PIN_GPIO->PDDR, (1 << INPUT_PIN), 0);

	//Read INPUT_PIN value
	uint8_t readPinData = INPUT_PIN_GPIO->PDR[INPUT_PIN];

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

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
