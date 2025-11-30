/*
 * uart2_touch.c
 *
 *  Created on: Sep 17, 2025
 *      Author: user
 */





#include "def.h"


struct Modbus_data comm_data;

uint16_t* Address_buf[300] = {0};


void comm_address_data(void)
{
    // === Set Data ===
    Address_buf[0]  = &comm_data.Part_ID;
    Address_buf[1]  = &comm_data.Version;
    Address_buf[2]  = &comm_data.RUN;
    Address_buf[3]  = &comm_data.ACCMODE;
    Address_buf[4]  = &comm_data.EP_CFG;
    Address_buf[5]  = &comm_data.CONFIG3;
    Address_buf[6]  = &comm_data.SeqErr;			//삼상 순서 Check, Touch Display 표시 기능 추가
    Address_buf[7]  = &comm_data.PM_Flag;
    Address_buf[8]  = &comm_data.CT_Primary;
    Address_buf[9]  = &comm_data.CT_Secondary;
    Address_buf[10] = &comm_data.Init_EEP;

    // === RMS (12cycle Value) ===
    Address_buf[11] = &comm_data.AIRMS;
    Address_buf[12] = &comm_data.BIRMS;
    Address_buf[13] = &comm_data.CIRMS;
    Address_buf[14] = &comm_data.IRMS_AVG;
    Address_buf[15] = &comm_data.ABVRMS;
    Address_buf[16] = &comm_data.BCVRMS;
    Address_buf[17] = &comm_data.CAVRMS;
    Address_buf[18] = &comm_data.LINE_VRMS_AVG;
    Address_buf[19] = &comm_data.VARMS;
    Address_buf[20] = &comm_data.VBRMS;
    Address_buf[21] = &comm_data.VCRMS;
    Address_buf[22] = &comm_data.PHASE_VRMS_AVG;

    // === Peak ===
    Address_buf[23] = &comm_data.VPEAK;

    // === Power ===
    Address_buf[24] = &comm_data.AWATT;
    Address_buf[25] = &comm_data.BWATT;
    Address_buf[26] = &comm_data.CWATT;
    Address_buf[27] = &comm_data.WATT_TOT;
    Address_buf[28] = (uint16_t *)&comm_data.AVAR;
    Address_buf[29] = (uint16_t *)&comm_data.BVAR;
    Address_buf[30] = (uint16_t *)&comm_data.CVAR;
    Address_buf[31] = (uint16_t *)&comm_data.VAR_TOT;
    Address_buf[32] = &comm_data.AVA;
    Address_buf[33] = &comm_data.BVA;
    Address_buf[34] = &comm_data.CVA;
    Address_buf[35] = &comm_data.CVA_TOT;

    // === Energy (2 Word each) ===
    Address_buf[36] = &comm_data.WATT_ACC_T_HI;
    Address_buf[37] = &comm_data.WATT_ACC_T_LO;

    Address_buf[38] = (uint16_t *)&comm_data.VAR_ACC_T_HI;
    Address_buf[39] = (uint16_t *)&comm_data.VAR_ACC_T_LO;

    Address_buf[40] = &comm_data.VA_ACC_T_HI;
    Address_buf[41] = &comm_data.VA_ACC_T_LO;

    // === PF ===
    Address_buf[42] = (uint16_t *)&comm_data.APF;
    Address_buf[43] = (uint16_t *)&comm_data.BPF;
    Address_buf[44] = (uint16_t *)&comm_data.CPF;
    Address_buf[45] = (uint16_t *)&comm_data.PF_AVG;

    // === THD ===
    Address_buf[46] = &comm_data.AVTHD;
    Address_buf[47] = &comm_data.BVTHD;
    Address_buf[48] = &comm_data.CVTHD;
    Address_buf[49] = &comm_data.VTHD_AVG;
    Address_buf[50] = &comm_data.AITHD;
    Address_buf[51] = &comm_data.BITHD;
    Address_buf[52] = &comm_data.CITHD;
    Address_buf[53] = &comm_data.ITHD_AVG;

    // === TDD ===
    Address_buf[54] = &comm_data.AITDD;
    Address_buf[55] = &comm_data.BITDD;
    Address_buf[56] = &comm_data.CITDD;
    Address_buf[57] = &comm_data.ITDD_AVG;

    // === Frequency ===
    Address_buf[58] = &comm_data.AFREQ;
    Address_buf[59] = &comm_data.BFREQ;
    Address_buf[60] = &comm_data.CFREQ;
    Address_buf[61] = &comm_data.FREQ_AVG;

    // === Angle (signed) ===
    Address_buf[62] = (uint16_t *)&comm_data.ANGL_VA_VB;
    Address_buf[63] = (uint16_t *)&comm_data.ANGL_VB_VC;
    Address_buf[64] = (uint16_t *)&comm_data.ANGL_VA_VC;
    Address_buf[65] = (uint16_t *)&comm_data.ANGL_VA_IA;
    Address_buf[66] = (uint16_t *)&comm_data.ANGL_VB_IB;
    Address_buf[67] = (uint16_t *)&comm_data.ANGL_VC_IC;
    Address_buf[68] = (uint16_t *)&comm_data.ANGL_IA_IB;
    Address_buf[69] = (uint16_t *)&comm_data.ANGL_IB_IC;
    Address_buf[70] = (uint16_t *)&comm_data.ANGL_IA_IC;

    // === TDD 계산시 사용되는 전류 정격 ===
    Address_buf[71] = (uint16_t *)&comm_data.I_rated;


	Address_buf[90] = &comm_data.PM_kwh_comm_Reset;
//	Address_buf[91] =
//	Address_buf[92] =
//	Address_buf[93] =
//	Address_buf[94] =
	Address_buf[95] = &comm_data.PM_comm_Reset;
//	Address_buf[96] =
	Address_buf[97] = &comm_data.Firmware_year;
	Address_buf[98] = &comm_data.Firmware_mon;
	Address_buf[99] = &comm_data.Firmware_date;
	Address_buf[100] = &comm_data.Firmware_Version;

}




unsigned int uart2_crc_data = 0;

unsigned char uart2_CheckCrc(unsigned char Touch_data)
{
    unsigned int Read_Buff;

    Read_Buff = (unsigned int)(Touch_data & 0x00FF);

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    if(((Read_Buff ^ uart2_crc_data) & 0x0001) == 0x0001) 	uart2_crc_data = (uart2_crc_data >> 1) ^ 0xA001;
    else                                                	uart2_crc_data >>= 1;
    Read_Buff >>= 1;

    return (uart2_crc_data);
}

//========================= E N D ==========================

