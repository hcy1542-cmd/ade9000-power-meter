/*
 * i2c_val.c
 *
 *  Created on: May 2, 2023
 *      Author: yksgreen
 */



#include "def.h"

uint8_t wr_eeprom_data[10];
uint8_t rd_eeprom_data[10];

uint8_t wr_eeprom_log_data[10];
uint8_t rd_eeprom_log_data[10];


void Write_8bit_I2C_Data(unsigned int device, unsigned int data)
{
	wr_eeprom_data[0] = data;

	HAL_FMPI2C_Mem_Write(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &wr_eeprom_data[0], 1, 10);

	HAL_Delay(5);

	i2c.eeprom.page += 1;
}

void Read_8bit_I2C_Data(unsigned int device)
{
	HAL_FMPI2C_Mem_Read(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &rd_eeprom_data[0], 1, 10);

	i2c.eeprom.tmp_data = rd_eeprom_data[0];
	i2c.eeprom.page += 1;
}


void Write_I2C_float_Data(unsigned int device, float dt)
{
	float data = (float)dt;
	wr_eeprom_data[3]= *(unsigned long *)&data >> 24;
	wr_eeprom_data[2]= *(unsigned long *)&data >> 16;
	wr_eeprom_data[1]= *(unsigned long *)&data >> 8;
	wr_eeprom_data[0]= *(unsigned long *)&data;

	HAL_FMPI2C_Mem_Write(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &wr_eeprom_data[0], 4, 10);

	HAL_Delay(5);

	i2c.eeprom.page += 4;
}

void Write_I2C_long_Data(unsigned int device, long data)
{

	wr_eeprom_data[3]= *(unsigned long *)&data >> 24;
	wr_eeprom_data[2]= *(unsigned long *)&data >> 16;
	wr_eeprom_data[1]= *(unsigned long *)&data >> 8;
	wr_eeprom_data[0]= *(unsigned long *)&data;

	HAL_FMPI2C_Mem_Write(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &wr_eeprom_data[0], 4, 10);

	HAL_Delay(5);

	i2c.eeprom.page += 4;
}

void Read_I2C_float_Data(unsigned int device)
{
	HAL_FMPI2C_Mem_Read(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &rd_eeprom_data[0], 4, 10);

	i2c.eeprom.tmp_data = *(unsigned long *)&rd_eeprom_data[3] << 24 | \
							*(unsigned long *)&rd_eeprom_data[2] << 16 | \
							*(unsigned long *)&rd_eeprom_data[1] << 8 | \
							*(unsigned long *)&rd_eeprom_data[0];

	i2c.eeprom.page += 4;
}

void Read_I2C_long_Data(unsigned int device)
{

	HAL_FMPI2C_Mem_Read(&hfmpi2c1, device, i2c.eeprom.page, FMPI2C_MEMADD_SIZE_16BIT, &rd_eeprom_data[0], 4, 10);

	i2c.eeprom.tmp_data = *(unsigned long *)&rd_eeprom_data[3] << 24 | \
							*(unsigned long *)&rd_eeprom_data[2] << 16 | \
							*(unsigned long *)&rd_eeprom_data[1] << 8 | \
							*(unsigned long *)&rd_eeprom_data[0];

	i2c.eeprom.page += 4;
}
