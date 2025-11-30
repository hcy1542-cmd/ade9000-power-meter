/*
 * def.h
 *
 *  Created on: Aug 20, 2025
 *      Author: user
 */

#ifndef INCLUDE_DEF_H_
#define INCLUDE_DEF_H_

#include <stdio.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"

#include <GP.h>
#include "easyStm32LL v11.3.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "usart.h"
#include "extern.h"
#include "iwdg.h"

#include "tim.h"
#include "spi.h"
#include "fmpi2c.h"
#include "usart.h"
#include "ADE9000_SPI.h"
#include "ADE9000.h"
#include "i2c.h"
#include "uart.h"
#include "struct.h"
#include "MD_RTU.h"
#include "utill.h"
#include "stm_i2c.h"

//#include "ADE9000_i2c.h"
//#include "ADE9000_RS485.h"

extern int SPI_Flag;
extern int PM_Flag;
extern uint64_t EEP_Index;
extern char LED_Status;

extern volatile uint8_t flag_100ms;
extern volatile uint8_t flag_1s;
extern volatile uint8_t flag_1s_2;
extern volatile uint8_t flag_5min;
extern volatile uint8_t flag_10min;

extern void System_Check_after_Power_ON(void);

void GPInit(void);
void GPMain(void);

#endif /* INCLUDE_DEF_H_ */
