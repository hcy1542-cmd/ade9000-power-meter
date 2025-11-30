/*
 * MD_RTU.h
 *
 *  Created on: Aug 27, 2025
 *      Author: user
 */

#ifndef INCLUDE_MD_RTU_H_
#define INCLUDE_MD_RTU_H_

extern void ADE_MODBUS(void);
extern void UART_SendStruct(void);
extern void Modbus_SendRegisters_NoCRC(uint8_t slave_id, uint8_t func_code,
        uint16_t start_addr, uint16_t reg_count);
#endif /* INCLUDE_MD_RTU_H_ */
