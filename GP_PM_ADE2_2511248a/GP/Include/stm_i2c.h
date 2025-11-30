/*
 * stm_i2c.h
 *
 *  Created on: May 6, 2023
 *      Author: user
 */

#ifndef INCLUDE_STM_I2C_H_
#define INCLUDE_STM_I2C_H_


#include "def.h"


#define 	AT24C1024_A 0xA4		// Power meter Addr로 변경됨(0xA0 -> 0xA4)

//#define I2C_WP1_HI()		HAL_GPIO_WritePin(GPIOB, I2C_WP1_Pin, GPIO_PIN_SET);
//#define I2C_WP1_LO()		HAL_GPIO_WritePin(GPIOB, I2C_WP1_Pin, GPIO_PIN_RESET);
//#define I2C_WP2_HI()		HAL_GPIO_WritePin(GPIOB, I2C_WP2_Pin, GPIO_PIN_SET);
//#define I2C_WP2_LO()		HAL_GPIO_WritePin(GPIOB, I2C_WP2_Pin, GPIO_PIN_RESET);

void FMPI2C_data_Write(uint16_t Device_address, uint16_t Mem_address, uint8_t fmp_i2c_data, uint16_t data_size);
void FMPI2C_data_Read(uint16_t Device_address, uint16_t Mem_address, uint8_t fmp_i2c_R_data, uint16_t data_size);


#define st_i2c_data_float(dt,val){ dt##_new = val; }

#define wr_i2c_data_float(dt){\
  Write_I2C_long_Data(AT24C1024_A,(float)(dt##_new * 100));\
  tmp_wr_check_sum += (unsigned long)dt##_new;\
}

#define rd_i2c_data_float(dt){\
  Read_I2C_long_Data(AT24C1024_A);\
  dt##_new=dt=(float)(i2c.eeprom.tmp_data * 0.01);\
  tmp_rd_check_sum += (unsigned long)dt##_new;\
}


#define wr_i2c_data_long(dt){\
  Write_I2C_long_Data(AT24C1024_A,(float)dt##_new);\
  tmp_wr_check_sum += (unsigned long)dt##_new;\
}


#define rd_i2c_data_long(dt){\
  Read_I2C_long_Data(AT24C1024_A);\
  dt##_new=dt=(long)i2c.eeprom.tmp_data;\
  tmp_rd_check_sum += (unsigned long)dt##_new;\
}

#define wr_i2c_data_Bgain(dt){\
    long __scaled = (long)((dt##_new) * 1000.0); \
    Write_I2C_long_Data(AT24C1024_A, __scaled); \
    tmp_wr_check_sum += (unsigned long)__scaled; \
}

#define rd_i2c_data_Bgain(dt){\
    Read_I2C_long_Data(AT24C1024_A); \
    long __scaled = (long)i2c.eeprom.tmp_data; \
    dt = dt##_new = ((double)__scaled) / 1000.0; \
    tmp_rd_check_sum += (unsigned long)__scaled; \
}



#define ck_i2c_ad_data(dt){ if(dt##_new != dt) i2c.eeprom.PM_write = 1; }


#endif /* INCLUDE_STM_I2C_H_ */
