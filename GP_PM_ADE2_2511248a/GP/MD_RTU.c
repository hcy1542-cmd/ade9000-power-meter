/*
 * MD_RTU.c
 *
 *  Created on: Sep 26, 2025
 *      Author: user
 */

#include "def.h"

bool send_end;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin, GPIO_PIN_RESET); // RX 모드 복귀
        send_end = 1;
    }
}

