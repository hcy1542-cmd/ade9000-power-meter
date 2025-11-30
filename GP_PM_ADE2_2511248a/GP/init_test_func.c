/*
 * init_test_func.c
 *
 *  Created on: Nov 13, 2025
 *      Author: user
 */

#include "def.h"

void System_Check_after_Power_ON(void)
{
	if(i2c.eeprom.init_cnt++ > REF_TIME_sec(3))
	{
		i2c.eeprom.init_cnt = 0;
		i2c.eeprom.enable = 1;
	}
}
