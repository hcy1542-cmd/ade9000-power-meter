/*
 * ADE9000 SPI.h
 *
 *  Created on: Aug 16, 2025
 *      Author: user
 */

#ifndef INCLUDE_ADE9000_SPI_H_
#define INCLUDE_ADE9000_SPI_H_

#include "def.h"

#define SPI_CS_ON()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_CS_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define WP_Write_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define WP_Write_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)

/* 24/32bit read/write API */
extern uint32_t ADE_Read32(uint16_t addr);
extern uint16_t ADE_Read16(uint16_t addr);

extern int ADE_Write32(uint16_t addr, uint32_t val);
extern int ADE_Write16(uint16_t addr, uint16_t val);

#endif /* INCLUDE_ADE9000_SPI_H_ */
