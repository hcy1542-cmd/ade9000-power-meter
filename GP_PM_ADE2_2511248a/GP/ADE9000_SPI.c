/*
 * SPI.c
 *
 *  Created on: Aug 15, 2025
 *      Author: hcy
 */
#include <stdint.h>
#include "main.h"
#include "def.h"
#include "spi.h"

//extern SPI_HandleTypeDef hspi1;
#define ADE_READ_BIT	1
#define ADE_WRITE_BIT	0
#define Data_Size_16_BIT 1
#define Data_Size_32_BIT 2
#define Data_Size_48_BIT 3

/////////////////   hspi1.Init.DataSize = SPI_DATASIZE_16BIT;	//////////////////


/* CMD_HDR 생성: addr=0x000~0x7FF, rw: 1=read, 0=write */
static inline uint16_t ADE_CMD_HDR(uint16_t addr, uint8_t rw)
{
    return (uint16_t)(((addr << 4) & 0xFFF0) | ((rw & 0x1) << 3));
}
uint16_t rxDataH;
uint16_t rxDataL;

/* 32비트 읽기 (rw=1) */
uint32_t ADE_Read32(uint16_t addr)
{
    uint16_t hdr = ADE_CMD_HDR(addr, ADE_READ_BIT);
    uint16_t rxDataH, rxDataL;
    uint32_t rxData;

    SPI_CS_ON();

    HAL_SPI_Transmit(&hspi1, (uint8_t*)&hdr, Data_Size_16_BIT, HAL_MAX_DELAY);

    // 먼저 MSB를 받는다
    HAL_SPI_Receive(&hspi1, (uint8_t*)&rxDataH, Data_Size_16_BIT, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t*)&rxDataL, Data_Size_16_BIT, HAL_MAX_DELAY);

    SPI_CS_OFF();

    rxData = ((uint32_t)rxDataH << 16) | (uint32_t)rxDataL;
    return rxData;
}



uint16_t ADE_Read16(uint16_t addr)
{
    uint16_t hdr = ADE_CMD_HDR(addr, ADE_READ_BIT);
    uint16_t rxData;

    SPI_CS_ON();

    HAL_SPI_Transmit(&hspi1, (uint8_t*)&hdr, Data_Size_16_BIT, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi1, (uint8_t*)&rxData, Data_Size_16_BIT, HAL_MAX_DELAY);

    SPI_CS_OFF();

    return rxData;
}

/* 32비트 쓰기 */
int ADE_Write32(uint16_t addr, uint32_t val)
{
    uint16_t hdr = ADE_CMD_HDR(addr, ADE_WRITE_BIT);
    uint16_t frame[3] = {
        hdr,
        (uint16_t)(val >> 16),   // 상위 16비트
        (uint16_t) val           // 하위 16비트
    };

    SPI_CS_ON();

    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, (uint8_t*)frame, Data_Size_48_BIT, HAL_MAX_DELAY);

    SPI_CS_OFF();
    return (st == HAL_OK) ? 0 : -1;
}

/* 16비트 쓰기 */
int ADE_Write16(uint16_t addr, uint16_t val)
{
    uint16_t hdr = ADE_CMD_HDR(addr, ADE_WRITE_BIT);
    uint16_t frame[2] = {hdr , val};

    SPI_CS_ON();

    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, (uint8_t*)frame, Data_Size_32_BIT, HAL_MAX_DELAY);

    SPI_CS_OFF();

    return (st == HAL_OK) ? 0 : -1;
}

