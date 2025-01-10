/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"

uint8_t aTxBuffer[10];
uint8_t nbDataToTransmit = sizeof(aTxBuffer);

/*
 * @brief 	Initializes LPUART1 at a baud rate of 115200.
 *			Whenever the TX FIFO is empty it will generate a DMA request.
 *			By default the TX transmitter is disabled and the DMA hardware request is also disabled.
 *			Then by enabling the TX transmitter and the DMA hardware request it will immediately transfer all
 *			data from the aTxBuffer to the TX FIFO. The TX FIFO automatically sends the FIFO data to the TX pin.
 *			After all the data is transmit, the DMA hardware request bit is automatically cleared and
 *			the TX transmitter is disabled.
 */
void telem_UART_Init()
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
//	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_LPUART1(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_LPUART0(1) << SERIAL_TELEMETRY_MODULE;

	//Release peripherals from reset
//	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_LPUART1(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_LPUART0(1) << SERIAL_TELEMETRY_MODULE;

	//Select UART clock FRO_12M
	//Has 32-bit registers so every address increment counts as 32-bits
//	modifyReg32(&MRCC0->MRCC_LPUART1_CLKSEL, MRCC_MRCC_LPUART1_CLKSEL_MUX_MASK, MRCC_MRCC_LPUART1_CLKSEL_MUX(0));
	modifyReg32((&MRCC0->MRCC_LPUART0_CLKSEL + (2 * SERIAL_TELEMETRY_MODULE)), MRCC_MRCC_LPUART0_CLKSEL_MUX_MASK, MRCC_MRCC_LPUART0_CLKSEL_MUX(0));

	//Enable UART clock
	//Has 32-bit registers so every address increment counts as 32-bits
//	modifyReg32(&MRCC0->MRCC_LPUART1_CLKDIV, MRCC_MRCC_LPUART1_CLKDIV_HALT_MASK, 0);
	modifyReg32((&MRCC0->MRCC_LPUART0_CLKDIV + (2 * SERIAL_TELEMETRY_MODULE)), MRCC_MRCC_LPUART0_CLKDIV_HALT_MASK, 0);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set UART pins
	//Set alternative pin function and enable pull-up and input buffer
//	modifyReg32(&SERIAL_TELEMETRY_RX_PORT->PCR[SERIAL_TELEMETRY_RX_PIN],
//			PORT_PCR_MUX_MASK,
//			PORT_PCR_MUX(SERIAL_TELEMETRY_RX_ALT_FUNC) | PORT_PCR_PS(1) | PORT_PCR_PE(1) | PORT_PCR_IBE(1));
	modifyReg32(&SERIAL_TELEMETRY_TX_PORT->PCR[SERIAL_TELEMETRY_TX_PIN],
			PORT_PCR_MUX_MASK,
			PORT_PCR_MUX(SERIAL_TELEMETRY_TX_ALT_FUNC) | PORT_PCR_PS(1) | PORT_PCR_PE(1) | PORT_PCR_IBE(1));

	//Set UART1 out of reset
	modifyReg32(&SERIAL_TELEMETRY->GLOBAL, LPUART_GLOBAL_RST_MASK, 0);

	//SDK config 115200
	//CLK = 12MHz
	//SBR = 0x4, OSR = 25
	//Set OSR=25, SBR=4
	modifyReg32(&SERIAL_TELEMETRY->BAUD, \
			LPUART_BAUD_OSR_MASK | LPUART_BAUD_SBR_MASK, \
			LPUART_BAUD_OSR(25) | LPUART_BAUD_SBR(4));

	//Enable TX FIFO watermark flag to generate DMA request
	modifyReg32(&SERIAL_TELEMETRY->BAUD, LPUART_BAUD_TDMAE_MASK, LPUART_BAUD_TDMAE(1));

	//Set TX watermark to 1
	modifyReg32(&SERIAL_TELEMETRY->WATER, LPUART_WATER_TXWATER_MASK, LPUART_WATER_TXWATER(1));
}

/*
 * @brief 	Enables the UART TX transmitter
 */
void enable_telem_UART(void)
{
	//Enable transmitter
	modifyReg32(&SERIAL_TELEMETRY->CTRL, 0, LPUART_CTRL_TE(1));
}

/*
 * @brief 	Enables DMA hardware request.
 * 			This causes the aTxBuffer to be transfered to the TX FIFO.
 * 			Thus sending all aTxBuffer data over the TX pin.
 */
void send_telem_DMA()
{
	//Enable DMA hardware request
	modifyReg32(&DMA0->CH[DMA_CH_UART].CH_CSR, DMA_CH_CSR_ERQ_MASK, DMA_CH_CSR_ERQ(1));
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
uint8_t crc_u, i;
crc_u = crc;
crc_u ^= crc_seed;
for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
uint8_t crc = 0, i;
for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
return (crc);
}

/*
 * @brief 	Makes the telemetry package to send via UART
 */
void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm){
    aTxBuffer[0] = temp; // temperature

    aTxBuffer[1] = (voltage >> 8) & 0xFF; // voltage hB
    aTxBuffer[2] = voltage & 0xFF; // voltage   lowB

    aTxBuffer[3] = (current >> 8) & 0xFF; // current
    aTxBuffer[4] = current & 0xFF; // divide by 10 for Amps

    aTxBuffer[5] = (consumption >> 8) & 0xFF; // consumption
    aTxBuffer[6] = consumption & 0xFF; //  in mAh

    aTxBuffer[7] = (e_rpm >> 8) & 0xFF; //
    aTxBuffer[8] = e_rpm & 0xFF; // eRpM *100

    aTxBuffer[9] = get_crc8(aTxBuffer, 9);

//    aTxBuffer[0] = 65; // temperature
//
//    aTxBuffer[1] = 66; // voltage hB
//    aTxBuffer[2] = 67; // voltage   lowB
//
//    aTxBuffer[3] = 68; // current
//    aTxBuffer[4] = 69; // divide by 10 for Amps
//
//    aTxBuffer[5] = 70; // consumption
//    aTxBuffer[6] = 71; //  in mAh
//
//    aTxBuffer[7] = 72; //
//    aTxBuffer[8] = 73; // eRpM *100
//
//    aTxBuffer[9] = 74;
}

