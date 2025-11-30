/*
 * uart.h
 *
 *  Created on: Sept 6, 2025
 *      Author: user
 */

#ifndef INCLUDE_UART_H_
#define INCLUDE_UART_H_


#include "def.h"

#define UART_MAX_CH


#define UART_BUFFER_MAX		256
#define STX		0x02
#define ETX 	0x03
bool uartInit2(void);
uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length);
uint32_t uartAvailable(uint8_t ch);
uint8_t uartRead(uint8_t ch);
extern unsigned char uart2_CheckCrc(unsigned char Touch_data);
extern void comm_address_data(void);

extern uint16_t * Address_buf[300];
extern unsigned char uart2_rxd_buf[];
extern unsigned char uart2_txd_buf[];
extern unsigned int uart2_crc_data;
extern unsigned int Error_Code[14];

#endif /* INCLUDE_UART_H_ */
