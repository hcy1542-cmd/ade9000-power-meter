/*
 * ti.c
 *
 *  Created on: Aug 20, 2025
 *      Author: user
 */


#include "def.h"		//#include "easyStm32LL v11.3.h"

uint32_t TIM3_CNT;
uint32_t TIM_500ms_CNT;
uint8_t PM_comm_Reset_CNT;
uint8_t PM_kwh_comm_Reset_CNT;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)	//10khz
	{
		TIM3_CNT++;

		if(i2c.eeprom.enable != 1)	System_Check_after_Power_ON();

 	    UART2_tx_polling();
 	    UART2_rx_polling();

 	    if(TIM_500ms_CNT++ > REF_TIME_msec(500))
 	    {
			if(LED_Status == 0)
			{
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_14);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
			}
			else		// 에러 상황
			{
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_14);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			}
 	    	TIM_500ms_CNT = 0;
 	    }

 		if(comm_data.PM_comm_Reset == 1)
 		{
 			if(PM_comm_Reset_CNT++ > 10){
 				PM_Flag = 99;
 				PM_comm_Reset_CNT = 0;
 				comm_data.PM_comm_Reset = 0;
 			}
 		}
 		if(comm_data.PM_kwh_comm_Reset == 1)
 		{
 			if(PM_comm_Reset_CNT++ > 10){
 				PM_Flag = 12;
 				PM_comm_Reset_CNT = 0;
 				comm_data.PM_kwh_comm_Reset = 0;
 			}
 		}
	}

	if(htim->Instance == TIM4)	//100hz 0.01s
	{
 		uart2_conv_hex_dec();
	}
}
