/*
 * i2c.h
 *
 *  Created on: Aug 28, 2025
 *      Author: user
 */

#ifndef INCLUDE_I2C_H_
#define INCLUDE_I2C_H_

#include "def.h"


typedef struct
{
    uint32_t index;      // 기록 순번
    uint64_t energy;     // 누적 전력량 (Wh 단위 등)
    uint16_t crc;        // 오류 검출용
} Energy_EEPROM;

extern HAL_StatusTypeDef EEPROM_WriteEnergy(double E1, double E2, double E3);
extern HAL_StatusTypeDef EEPROM_ReadEnergy(uint64_t index, uint64_t *idx_out, double *Es, double *Ep, double *Eq);
extern HAL_StatusTypeDef EEPROM_Format_B(void);
extern HAL_StatusTypeDef EEPROM_Init_B(void);
extern void EEPROM_Format_A(void);
extern void EEPROM_Read_A(void);
extern void EEPROM_Gain_Write(int32_t value, uint16_t addr);
extern void I2C_Read_Write(void);
extern void i2c_data_init(void);
extern uint32_t EEPROM_4Byte_Read(uint16_t addr);

//

extern HAL_StatusTypeDef EEP_FillBlockB(uint8_t fill_value);
extern int32_t EEP_Read_B(uint16_t addr);



#endif /* INCLUDE_I2C_H_ */
