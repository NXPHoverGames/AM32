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
	uint32_t flags = CMP0->CSR & 0x7;

	//Rising interrupt
	if (flags & LPCMP_CSR_CFR_MASK) {
		//Call function from main.c
		interruptRoutine();
	}
	//Falling interrupt
	if (flags & LPCMP_CSR_CFF_MASK) {
		//Call function from main.c
		interruptRoutine();
	}

	//Clear comparator flags
	CMP0->CSR = flags;
}

/*
 * @brief 	Comparator 1 interrupt handler. Is called after a compare event has occurred.
 */
void CMP1_IRQHandler(void)
{
	uint32_t flags = CMP1->CSR & 0x7;

	//Rising interrupt
	if (flags & LPCMP_CSR_CFR_MASK) {
		//Call function from main.c
		interruptRoutine();
	}
	//Falling interrupt
	if (flags & LPCMP_CSR_CFF_MASK) {
		//Call function from main.c
		interruptRoutine();
	}

	//Clear comparator flags
	CMP1->CSR = flags;
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
	//Clear timer compare flag
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TCF_MASK, LPTMR_CSR_TCF(1));

	//Toggle P2.6 output
//	GPIO2->PTOR = (1 << 6);

	//Call loop function from main.c
	tenKhzRoutine();
}

/*
 * @brief 	Dshot/PWM interrupt handler. Calls when DMA0 has transfered the captured timing data
 */
void DMA_CH0_IRQHandler(void)
{
	//If there is an interrupt because DMA transfer is complete
//	if ((DMA0->CH[DMA_CH_DshotPWM].CH_INT & DMA_CH_INT_INT_MASK) | (DMA0->CH[0].CH_CSR & DMA_CH_CSR_DONE_MASK)) {
		//Clear DMA channel 0 interrupt flag
		DMA0->CH[DMA_CH_DshotPWM].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_DshotPWM].CH_ES = DMA_CH_ES_ERR(1);

		//Clear done flag
		modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));

		//Convert to correct Dshot/PWM timing data format
		doDshotCorrection();

		//TODO Add transfercomplete
		//And then make input_ready 1 so it will call processDshot in the main()
//		transfercomplete();

		input_ready = 1;


		//Process Dshot
//		processDshot();
//	}
}

/*
 * @brief 	ADC interrupt request called after the DMA has transferred the ADC data
 */
void DMA_CH1_IRQHandler(void)
{
	//Clear DMA channel 0 interrupt flag
	DMA0->CH[DMA_CH_ADC].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_ADC].CH_ES = DMA_CH_ES_ERR(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_ADC].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));

	//Call adc callback
	ADC_DMA_Callback();
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

	//Disable transmitter
	modifyReg32(&LPUART1->CTRL, LPUART_CTRL_TE_MASK, 0);
}

