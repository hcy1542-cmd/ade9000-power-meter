/*
 * extern.h
 *
 *  Created on: Sep 25, 2025
 *      Author: user
 */

#ifndef INCLUDE_EXTERN_H_
#define INCLUDE_EXTERN_H_

//=== uart2.c ===
extern void init_UART2(void);
extern void UART2_tx_polling(void);
extern void UART2_rx_polling(void);
extern void UART2_rxd_data_check(void);
extern void uart2_conv_hex_dec();
extern void UART2_protocol(void);
extern void uart2_Master_Control(void);

extern unsigned int Firmware_year;
extern unsigned int Firmware_mon;
extern unsigned int Firmware_date;
extern unsigned int Firmware_Version;

//=== i2c.val.c ===
extern void Write_I2C_float_Data(unsigned int device, float dt);
extern void Write_I2C_long_Data(unsigned int device, long data);
extern void Read_I2C_float_Data(unsigned int device);
extern void Read_I2C_long_Data(unsigned int device);

extern uint8_t i2c_sys_init;

#endif /* INCLUDE_EXTERN_H_ */
