/*
 * uart.c
 *
 *  Created on: Aug 21, 2025
 *      Author: yksgreen
 */


#include "uart.h"

static uint8_t UART2_buf[UART_BUFFER_MAX];
static uint32_t UART2_in = 0;
static uint32_t UART2_out = 0;

bool uartInit2(void)
{
	HAL_UART_Receive_DMA(&huart2, &UART2_buf[0], UART_BUFFER_MAX);
	return true;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
	switch(ch)
	{
		case 2 :	HAL_UART_Transmit_DMA(&huart2, p_data, length);	break;

		default : 	break;
	}
	return 0;
}

uint32_t uartAvailable(uint8_t ch)
{
	uint32_t ret;

	switch(ch)
	{
		case 2 :	UART2_in = (UART_BUFFER_MAX - huart2.hdmarx->Instance->NDTR) % UART_BUFFER_MAX;
					ret = (UART_BUFFER_MAX + UART2_in - UART2_out) % UART_BUFFER_MAX;
					break;

		default : 	break;
	}

	return ret;
}

uint8_t uartRead(uint8_t ch)
{
	uint8_t ret1;

	switch(ch)
	{
		case 2 :	if(UART2_out != UART2_in)
					{
						ret1 = UART2_buf[UART2_out];
						UART2_out = (UART2_out + 1)	% UART_BUFFER_MAX;
					}
					break;

		default : 	break;
	}
	return ret1;
}


//========================= E N D =========================
