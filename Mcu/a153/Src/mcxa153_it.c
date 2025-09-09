/*
 * mcxa153_it.c
 *
 *  Created on: 12 Nov 2024
 *      Author: nxg09992
 */

#include "mcxa153_it.h"

//TODO check what is not necessary
extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void processDshot();
extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;

char input_ready = 0;

/*
 * @brief 	Comparator 0 interrupt handler. Is called after a compare event has occurred.
 */
void CMP0_IRQHandler(void)
{
	//TODO remove this
//	GPIO3->PTOR = (1 << 27);	//ENC_A

	//Call function from main.c
	interruptRoutine();

	//Clear comparator flags
	CMP0->CSR = 0x7;
}

/*
 * @brief 	Comparator 1 interrupt handler. Is called after a compare event has occurred.
 */
void CMP1_IRQHandler(void)
{
	//TODO remove this
//	GPIO3->PTOR = (1 << 27);	//ENC_A

	//Call function from main.c
	interruptRoutine();

	//Clear comparator flags
	CMP1->CSR = 0x7;
}

/*
 * @brief COM_TIMER interrupt
 */
void CTIMER1_IRQHandler(void)
{
	uint32_t flags = CTIMER1->IR;

	if(flags & CTIMER_IR_MR0INT_MASK) {
		//Call function from main.c
		PeriodElapsedCallback();
	}

	//Clear interrupt flags
	CTIMER1->IR = flags;
}

/*
 * @brief 	TenKhzTimer interrupt
 */
void LPTMR0_IRQHandler(void)
{
	//Call loop function from main.c
	tenKhzRoutine();

	//Clear timer compare flag
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TCF_MASK, LPTMR_CSR_TCF(1));
}

/*
 * @brief 	Dshot/PWM interrupt handler. Calls when DMA0 has transfered the captured timing data
 */
void DMA_CH0_IRQHandler(void)
{
	if (armed && dshot_telemetry) {
	    if (out_put) {
	        receiveDshotDma();
	        compute_dshot_flag = 2;
	    } else {
	        sendDshotDma();
	        compute_dshot_flag = 1;
	    }
		//Set input_ready so processDshot is called in main loop
		input_ready = 1;
	    return;
	}

	//Convert to correct Dshot/PWM timing data format
	doDshotCorrection();

	//Call transfercomplete
	transfercomplete();

	//Set input_ready so processDshot is called in main loop
	input_ready = 1;

	//Clear DMA channel 0 interrupt flag
	DMA0->CH[DMA_CH_DshotPWM].CH_INT = DMA_CH_INT_INT(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, 0, DMA_CH_CSR_DONE(1));

	//Clear DMA error flag
	DMA0->CH[DMA_CH_DshotPWM].CH_ES = DMA_CH_ES_ERR(1);
}

/*
 * @brief 	ADC interrupt request called after the DMA has transferred the ADC data
 */
void DMA_CH1_IRQHandler(void)
{
	//Call adc callback
	ADC_DMA_Callback();

	//Clear DMA channel 0 interrupt flag
	DMA0->CH[DMA_CH_ADC].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_ADC].CH_ES = DMA_CH_ES_ERR(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_ADC].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));
}

/*
 * @brief 	DMA interrupt after all UART data has been transferred
 */
void DMA_CH2_IRQHandler(void)
{
	//Clear DMA channel 2 interrupt flag
	DMA0->CH[DMA_CH_UART].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_UART].CH_ES = DMA_CH_ES_ERR(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_UART].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));
}

