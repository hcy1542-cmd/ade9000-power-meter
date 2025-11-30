/*
 * t.c
 *
 *  Created on: Aug 20, 2025
 *      Author: user
 */


#include "def.h"
#include <string.h>
#include "stm32f4xx.h"

void GPInit(void)
{

}

bool RS485_Check;
bool IWdg_Check = false;
uint32_t t_start, t_end, elapsed_ms;
struct i2c_val i2c;
struct Unit eep;

void GPMain(void)
{
	iwdgBegin(2000);	//iWDG Enable - 2s check	// 1/32,000 * 32 * 2000 = 2

	// B-Block Index Read
	EEPROM_Init_B();

	// Data Copy
	EEPROM_ReadEnergy(EEP_Index, &EEP_Index ,&ADE9000.VA_ACC.rd, &ADE9000.WATT_ACC.rd, &ADE9000.VAR_ACC.rd);

	easyDSP_init(USART1);
	HAL_TIM_Base_Start_IT(&htim3);							// 100us Period
	HAL_TIM_Base_Start_IT(&htim4);
//	init_UART2();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		// ADE9000 Reset PIN : PM_ADE RESET High 상태 유지 후 AVDD, REF 전압 테스트
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// RS485 EN Pin
	comm_address_data();

	uartInit2();
	HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin, GPIO_PIN_RESET); //RS485_RX
	i2c.eeprom.wr_access = 1;

//	HAL_Delay(1000);

	while(1)
	{
		if(i2c.eeprom.enable == 1)		I2C_Read_Write();

		// ADE9000 Main Function
		if(i2c_sys_init == 1)
		{
			if(flag_100ms)
			{
				flag_100ms = 0;
				ADE9000_Measurement();
			}
		}

 		init_UART2();

 		iwdgRefresh();

	}
}
