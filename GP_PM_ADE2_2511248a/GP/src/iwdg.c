/*
 * iwdg.c
 *
 *  Created on: May 15, 2023
 *      Author: user
 */


#include "iwdg.h"


static IWDG_HandleTypeDef hiwdg;

bool iwdgInit(void)
{
	return true;
}

bool iwdgBegin(uint32_t time_ms)
{
	if(time_ms >= 4095)
		return false;

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = time_ms;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		return false;
	}
	return true;
}

bool iwdgRefresh(void)
{
	HAL_IWDG_Refresh(&hiwdg);
	return true;
}
