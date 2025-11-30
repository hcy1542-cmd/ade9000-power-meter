/*
 * I2C.c
 *
 *  Created on: Aug 28, 2025
 *      Author: user
 */


#include "def.h"
#include <string.h>

#define EEPROM_BASE_ADDR   0x52       // (A2=0, A1=1, A0=0 → 0x52)
#define EEPROM_BLOCK_SIZE  0x10000    // 64kB (65536)
#define EEPROM_TOTAL_SIZE  0x20000    // 128kB (131072)
#define RESERVED_SIZE      128

uint32_t current_addr = RESERVED_SIZE; // 첫 저장 위치
uint32_t last_written_addr = RESERVED_SIZE;

uint8_t i2c_sys_init = 0;

uint16_t CalcCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;  // 다항식 x^16 + x^15 + x^2 + 1
            else
                crc >>= 1;
        }
    }
    return crc;
}

void EEPROM_Gain_Write(int32_t value, uint16_t addr){
	uint8_t rec[4];

    rec[0] = (uint8_t)(value);
    rec[1] = (uint8_t)(value >> 8);
    rec[2] = (uint8_t)(value >> 16);
    rec[3] = (uint8_t)(value >> 24);

    HAL_FMPI2C_Mem_Write(&hfmpi2c1,
							(uint16_t)(EEPROM_BASE_ADDR << 1),
							addr,							//Start Add
							FMPI2C_MEMADD_SIZE_16BIT,
							rec,
							4,
							HAL_MAX_DELAY);
    HAL_Delay(10);
}

uint32_t EEPROM_4Byte_Read(uint16_t addr)
{
	uint8_t buf[4];
	uint32_t temp_Value;
	HAL_FMPI2C_Mem_Read(&hfmpi2c1,
							(uint16_t)(EEPROM_BASE_ADDR << 1),
							addr,
							FMPI2C_MEMADD_SIZE_16BIT,
							buf,
							4,
							HAL_MAX_DELAY);

	temp_Value = (uint32_t)buf[0]
                      | ((uint32_t)buf[1] << 8)
                      | ((uint32_t)buf[2] << 16)
                      | ((uint32_t)buf[3] << 24);

	HAL_Delay(10);
    return (int32_t)temp_Value;
}
// ----------------- Energy Write -----------------
#define EEPROM_BASE_ADDR   0x52
#define BLOCK_B_OFFSET     0x10000
#define PAGE_PER_BLOCK     256      // Block B 내 페이지 수
#define PAGE_SIZE		   256
#define WRITE_DELAY_MS     10

uint64_t EEP_Index = 0;

HAL_StatusTypeDef EEPROM_WriteEnergy(double Es, double Ep, double Eq)
{
    HAL_StatusTypeDef ret;
    uint8_t rec[32];
    uint64_t scaled_u;
    int64_t  scaled_s;

    // --- 직렬화 ---
    for (int i = 0; i < 8; i++)
        rec[i] = (uint8_t)(EEP_Index >> (8 * i));

    scaled_u = (uint64_t)(Es * 10);
    for (int i = 0; i < 8; i++)
        rec[8 + i] = (uint8_t)(scaled_u >> (8 * i));

    scaled_u = (uint64_t)(Ep * 10);
    for (int i = 0; i < 8; i++)
        rec[16 + i] = (uint8_t)(scaled_u >> (8 * i));

    scaled_s = (int64_t)(Eq * 10);
    for (int i = 0; i < 8; i++)
        rec[24 + i] = (uint8_t)(scaled_s >> (8 * i));

    // --- 주소 계산 (Block B 고정, 인덱스 순환) ---
    uint32_t addr = BLOCK_B_OFFSET + (EEP_Index % PAGE_PER_BLOCK) * PAGE_SIZE;
    uint8_t dev7  = EEPROM_BASE_ADDR | 0x01;   // B block
    uint16_t mem16 = (uint16_t)(addr & 0xFFFF);

    // --- EEPROM 쓰기 ---
    ret = HAL_FMPI2C_Mem_Write(&hfmpi2c1,
                               (uint16_t)(dev7 << 1),
                               mem16,
                               FMPI2C_MEMADD_SIZE_16BIT,
                               rec,
                               sizeof(rec),
                               HAL_MAX_DELAY);

    HAL_Delay(WRITE_DELAY_MS);
    EEP_Index++;  // 계속 증가
    return ret;
}

#define RECORD_SIZE        32

HAL_StatusTypeDef EEPROM_ReadEnergy(uint64_t index, uint64_t *idx_out, double *Es, double *Ep, double *Eq)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[RECORD_SIZE];

    // -------------------------------
    // 주소 계산 (Block B 고정)
    // -------------------------------
    uint32_t addr = BLOCK_B_OFFSET + (index % PAGE_PER_BLOCK) * PAGE_SIZE;
    uint8_t dev7  = EEPROM_BASE_ADDR | 0x01;   // Block B 선택
    uint16_t mem16 = (uint16_t)(addr & 0xFFFF);

    // -------------------------------
    // EEPROM에서 32 byte 읽기
    // -------------------------------
    ret = HAL_FMPI2C_Mem_Read(&hfmpi2c1,
                              (uint16_t)(dev7 << 1),
                              mem16,
                              FMPI2C_MEMADD_SIZE_16BIT,
                              buf,
                              RECORD_SIZE,
                              HAL_MAX_DELAY);
    if (ret != HAL_OK)
        return ret;

    // -------------------------------
    // 역직렬화
    // -------------------------------
    uint64_t idx_raw = 0, es_raw = 0, ep_raw = 0;
    int64_t  eq_raw  = 0;

    for (int i = 0; i < 8; i++)
        idx_raw |= ((uint64_t)buf[i] << (8 * i));

    for (int i = 0; i < 8; i++)
        es_raw  |= ((uint64_t)buf[8 + i] << (8 * i));

    for (int i = 0; i < 8; i++)
        ep_raw  |= ((uint64_t)buf[16 + i] << (8 * i));

    for (int i = 0; i < 8; i++)
        eq_raw  |= ((int64_t)((uint64_t)buf[24 + i] << (8 * i)));

    // -------------------------------
    // 스케일 복원
    // -------------------------------
    *idx_out = idx_raw;
    *Es = (double)es_raw / 10.0;
    *Ep = (double)ep_raw / 10.0;
    *Eq = (double)eq_raw / 10.0;

    return HAL_OK;
}


uint64_t Test_EEP_Ind;
HAL_StatusTypeDef EEPROM_Init_B(void)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[8];
    uint64_t idx_raw = 0, max_idx = 0;

    uint8_t dev7  = EEPROM_BASE_ADDR | 0x01;   // Block B
    for (uint16_t page = 0; page < PAGE_PER_BLOCK; page++)
    {
        uint32_t addr = BLOCK_B_OFFSET + (page * PAGE_SIZE);
        uint16_t mem16 = (uint16_t)(addr & 0xFFFF);

        // --- 페이지 첫 8 바이트(인덱스 필드) 읽기 ---
        ret = HAL_FMPI2C_Mem_Read(&hfmpi2c1,
                                  (uint16_t)(dev7 << 1),
                                  mem16,
                                  FMPI2C_MEMADD_SIZE_16BIT,
                                  buf,
                                  8,
                                  HAL_MAX_DELAY);
        if (ret != HAL_OK)
            continue; // 읽기 실패 시 건너뜀

        idx_raw = 0;
        for (int i = 0; i < 8; i++)
            idx_raw |= ((uint64_t)buf[i] << (8 * i));

        // --- 빈 페이지 (0xFF 채움) 검사 선택적 추가 ---
        if (idx_raw == 0 || idx_raw > 1000000000000ULL)
            continue;   // 비정상 or 초기화 페이지는 스킵

        // --- 최대 인덱스 갱신 ---
        if (idx_raw > max_idx)
            max_idx = idx_raw;
    }

    EEP_Index = max_idx;   // 다음 기록 인덱스로 시작
    Test_EEP_Ind = max_idx;

    return HAL_OK;
}


HAL_StatusTypeDef EEPROM_Format_B(void)
{
	HAL_StatusTypeDef ret;
    uint8_t blank[PAGE_SIZE];
    memset(blank, 0x00, sizeof(blank));   // 모두 0x00로 채움

    uint8_t dev7 = EEPROM_BASE_ADDR | 0x01; // Block B
    for (uint16_t page = 0; page < PAGE_PER_BLOCK; page++)
    {

        IWDG->KR = 0xAAAA;  // watchdog kick

        uint32_t addr = BLOCK_B_OFFSET + (page * PAGE_SIZE);
        uint16_t mem16 = (uint16_t)(addr & 0xFFFF);

        ret = HAL_FMPI2C_Mem_Write(&hfmpi2c1,
                                   (uint16_t)(dev7 << 1),
                                   mem16,
                                   FMPI2C_MEMADD_SIZE_16BIT,
                                   blank,
                                   sizeof(blank),
                                   HAL_MAX_DELAY);
        HAL_Delay(WRITE_DELAY_MS);

        if (ret != HAL_OK)
        	return ret;
    }

    return HAL_OK;



}


unsigned long tmp_wr_check_sum = 0, tmp_rd_check_sum = 0;
unsigned long wr_ad_check_sum = 0, wr_ad_check_sum_new = 0;
unsigned long rd_ad_check_sum = 0, rd_sys_check_sum = 0, rd_gain_check_sum = 0;
void i2c_PM_data_write(unsigned int page)
{
	// wr_i2c_data_long
	// write dt##_new Data in EEPROM , wr_Check_sum += dt##_new
	tmp_wr_check_sum = 0;
	wr_ad_check_sum_new = 0;

	//	I2C_WP1_LO();
	i2c.eeprom.page = page;

	/*001*/wr_i2c_data_long(eep.V_Gain.AB.nscale);
	/*002*/wr_i2c_data_long(eep.V_Gain.BC.nscale);
	/*003*/wr_i2c_data_long(eep.V_Gain.CA.nscale);
	/*004*/wr_i2c_data_long(eep.I_Gain.A.nscale);
	/*005*/rd_i2c_data_Bgain(eep.I_Gain.B.fscale);
	/*006*/wr_i2c_data_long(eep.I_Gain.C.nscale);
	/*007*/wr_i2c_data_long(eep.Set.Frequency.val);
	/*008*/wr_i2c_data_long(eep.CT.Num.val);
	/*009*/wr_i2c_data_long(eep.CT.Primary.val);
	/*010*/wr_i2c_data_long(eep.CT.Secondary.val);
	/*011*/wr_i2c_data_long(eep.CT.Ratio.val);
	/*012*/wr_i2c_data_long(eep.Set.I_rated.val);
	/*013*/wr_i2c_data_long(eep.RS485.BaudRate.val);
	/*014*/wr_i2c_data_long(eep.RS485.Word_Length.val);
	/*015*/wr_i2c_data_long(eep.RS485.Parity.val);
	/*016*/wr_i2c_data_long(eep.RS485.Stop_Bit.val);
	/*017*/wr_i2c_data_long(eep.Set.Modbus_ID.val);

//	wr_ad_check_sum_new = tmp_wr_check_sum;
//	/*018*/wr_i2c_data_long(wr_ad_check_sum);

	i2c.eeprom.PM_write = 0;
//	I2C_WP1_HI();
}

void i2c_PM_data_read(unsigned int page)
{
	// rd_i2c_data_long
	// Data Copy to dt, dt##_new, rd_Check_sum += dt##_new
	tmp_rd_check_sum = 0;
	rd_ad_check_sum = 0;

	i2c.eeprom.page = page;

	/*001*/rd_i2c_data_long(eep.V_Gain.AB.nscale);
	/*002*/rd_i2c_data_long(eep.V_Gain.BC.nscale);
	/*003*/rd_i2c_data_long(eep.V_Gain.CA.nscale);
	/*004*/rd_i2c_data_long(eep.I_Gain.A.nscale);
	/*005*/rd_i2c_data_Bgain(eep.I_Gain.B.fscale);
	/*006*/rd_i2c_data_long(eep.I_Gain.C.nscale);
	/*007*/rd_i2c_data_long(eep.Set.Frequency.val);
	/*008*/rd_i2c_data_long(eep.CT.Num.val);
	/*009*/rd_i2c_data_long(eep.CT.Primary.val);
	/*010*/rd_i2c_data_long(eep.CT.Secondary.val);
	/*011*/rd_i2c_data_long(eep.CT.Ratio.val);
	/*012*/rd_i2c_data_long(eep.Set.I_rated.val);
	/*013*/rd_i2c_data_long(eep.RS485.BaudRate.val);
	/*014*/rd_i2c_data_long(eep.RS485.Word_Length.val);
	/*015*/rd_i2c_data_long(eep.RS485.Parity.val);
	/*016*/rd_i2c_data_long(eep.RS485.Stop_Bit.val);
	/*017*/rd_i2c_data_long(eep.Set.Modbus_ID.val);

//	rd_ad_check_sum = tmp_rd_check_sum;
//	rd_i2c_data_long(wr_ad_check_sum);

//	if(rd_ad_check_sum != wr_ad_check_sum)
//	{
//		// 오류 발생
//	}

}

void Check_I2C_Data(void)
{
	// ck_i2c_ad_data
	// if(dt##_new != dt) -> dt = dt##_new, EEPROM에 데이터 write

	if(eep.CT.Ratio.val != eep.CT.Primary.val / eep.CT.Secondary.val){
		eep.CT.Ratio.val_new = eep.CT.Primary.val / eep.CT.Secondary.val;
	}

	//============== ad Data ======================
	/*001*/ck_i2c_ad_data(eep.V_Gain.AB.nscale);
	/*002*/ck_i2c_ad_data(eep.V_Gain.BC.nscale);
	/*003*/ck_i2c_ad_data(eep.V_Gain.CA.nscale);
	/*004*/ck_i2c_ad_data(eep.I_Gain.A.nscale);
	/*005*/ck_i2c_ad_data(eep.I_Gain.B.fscale);
	/*006*/ck_i2c_ad_data(eep.I_Gain.C.nscale);
	/*007*/ck_i2c_ad_data(eep.Set.Frequency.val);
	/*008*/ck_i2c_ad_data(eep.CT.Num.val);
	/*009*/ck_i2c_ad_data(eep.CT.Primary.val);
	/*010*/ck_i2c_ad_data(eep.CT.Secondary.val);
	/*011*/ck_i2c_ad_data(eep.CT.Ratio.val);
	/*012*/ck_i2c_ad_data(eep.Set.I_rated.val);
	/*013*/ck_i2c_ad_data(eep.RS485.BaudRate.val);
	/*014*/ck_i2c_ad_data(eep.RS485.Word_Length.val);
	/*015*/ck_i2c_ad_data(eep.RS485.Parity.val);
	/*016*/ck_i2c_ad_data(eep.RS485.Stop_Bit.val);
	/*017*/ck_i2c_ad_data(eep.Set.Modbus_ID.val);

	if(i2c.eeprom.PM_write==1)
	{
		i2c.eeprom.wr_access = 1;
	}

	i2c_sys_init = 1;

}
void i2c_data_init(void)
{
	// st_i2c_data_float
	// dt##_new = value 입력 --> Check_I2C_Data에서 걸림
	/*001*/st_i2c_data_float(eep.V_Gain.AB.nscale, 0);
	/*002*/st_i2c_data_float(eep.V_Gain.BC.nscale, 0);
	/*003*/st_i2c_data_float(eep.V_Gain.CA.nscale, 0);
	/*004*/st_i2c_data_float(eep.I_Gain.A.nscale, 0);
	/*005*/st_i2c_data_float(eep.I_Gain.B.fscale, 1);
	/*006*/st_i2c_data_float(eep.I_Gain.C.nscale, 0);
	/*007*/st_i2c_data_float(eep.Set.Frequency.val, 60);
	/*008*/st_i2c_data_float(eep.CT.Num.val, 2);
	/*009*/st_i2c_data_float(eep.CT.Primary.val, 50);
	/*010*/st_i2c_data_float(eep.CT.Secondary.val, 5);
	/*011*/st_i2c_data_float(eep.CT.Ratio.val, 10);
	/*012*/st_i2c_data_float(eep.Set.I_rated.val, 50);
	/*013*/st_i2c_data_float(eep.RS485.BaudRate.val, 9600);
	/*014*/st_i2c_data_float(eep.RS485.Word_Length.val, UART_WORDLENGTH_8B);
	/*015*/st_i2c_data_float(eep.RS485.Parity.val, UART_PARITY_NONE);
	/*016*/st_i2c_data_float(eep.RS485.Stop_Bit.val, UART_STOPBITS_1);
	/*017*/st_i2c_data_float(eep.Set.Modbus_ID.val, 1);
}


void I2C_Read_Write(void)
{
	if(i2c.eeprom.wr_access == 1)
	{
		if(i2c.eeprom.PM_write == 1)   i2c_PM_data_write(0x0000);

		i2c_PM_data_read(0x0000);

		i2c.eeprom.wr_access = 0;


	}
	else if(i2c.eeprom.wr_access == 99)
	{
		if(i2c.eeprom.init == 1)
		{
			i2c_data_init();
			i2c.eeprom.init = 0;
			i2c.eeprom.PM_write = 1;
			i2c.eeprom.wr_access = 1;
		}
	}
	else
	{
		Check_I2C_Data();
	}
}

///////// test ////////

HAL_StatusTypeDef EEP_FillBlockB(uint8_t fill_value)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[256];
    memset(buf, fill_value, 256);

    uint8_t dev7 = EEPROM_BASE_ADDR | 0x01; // Block B

    for (uint16_t page = 0; page < 256; page++)
    {
        uint16_t mem16 = page * 256;   // 0x0000 ~ 0xFF00

        ret = HAL_FMPI2C_Mem_Write(&hfmpi2c1,
                                   (uint16_t)(dev7 << 1),
                                   mem16,
                                   FMPI2C_MEMADD_SIZE_16BIT,
                                   buf,
                                   256,
                                   HAL_MAX_DELAY);

        HAL_Delay(10);

        while (HAL_FMPI2C_IsDeviceReady(&hfmpi2c1, (dev7 << 1), 2, HAL_MAX_DELAY) != HAL_OK)
            ;
    }

    return HAL_OK;
}



int32_t EEP_Read_B(uint16_t offset)
{
    uint8_t buf[4];
    uint32_t temp_Value;

    // --- Block B 고정 ---
    uint8_t dev7 = EEPROM_BASE_ADDR | 0x01;

    // --- Block B의 offset 주소 계산 ---
    uint16_t mem16 = (uint16_t)(BLOCK_B_OFFSET + offset);

    // --- I2C Read ---
    HAL_FMPI2C_Mem_Read(&hfmpi2c1,
                        (uint16_t)(dev7 << 1),
                        mem16,
                        FMPI2C_MEMADD_SIZE_16BIT,
                        buf,
                        4,
                        HAL_MAX_DELAY);

    // --- 4바이트 → 32bit 변환 ---
    temp_Value =  (uint32_t)buf[0]
                | ((uint32_t)buf[1] << 8)
                | ((uint32_t)buf[2] << 16)
                | ((uint32_t)buf[3] << 24);

    HAL_Delay(10);
    return (int32_t)temp_Value;
}

