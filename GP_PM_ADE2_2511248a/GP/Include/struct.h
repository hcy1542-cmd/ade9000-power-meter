/*
 * struct.h
 *
 *  Created on: Sep 25, 2025
 *      Author: user
 */

#ifndef INCLUDE_STRUCT_H_
#define INCLUDE_STRUCT_H_


struct eeprom_val{
  int wr_access;
  int enable;
  int init_cnt;
  int init;

  unsigned long tmp_data;
  unsigned long wr[12];
  unsigned long rd[12];
  unsigned int page;

  int PM_write;
//  int sys_write;
//  int gain_write;
//  int Eth_write;
//  int Lifetime_write;
};
struct i2c_val{
  unsigned int device;
//  int cnt;
//  int cnt_max;
//  int tmp;
//  int tmp_new;
//  unsigned long tmp_long[3];

  struct eeprom_val eeprom;
//  struct rtc_val    rtc;
//  struct event_val  event;
};

extern struct i2c_val i2c;

//============== EEPROM Block A Data ============

struct EEP_val{
	int32_t nscale;
	int32_t nscale_new;
	float	fscale;
	float	fscale_new;
	uint32_t val;
	uint32_t val_new;

} ;

struct Sub_Unit{
	struct EEP_val A;
	struct EEP_val B;
	struct EEP_val C;
	struct EEP_val AB;
	struct EEP_val BC;
	struct EEP_val CA;

	struct EEP_val Primary;
	struct EEP_val Secondary;
	struct EEP_val Num;
	struct EEP_val Ratio;

	struct EEP_val Frequency;
	struct EEP_val Frequency_new;
	struct EEP_val I_rated;
	struct EEP_val I_rated_new;

	struct EEP_val BaudRate;
	struct EEP_val Word_Length;
	struct EEP_val Parity;
	struct EEP_val Stop_Bit;
	struct EEP_val Modbus_ID;

};

struct Unit{
	struct Sub_Unit V_Gain;
	struct Sub_Unit I_Gain;
	struct Sub_Unit CT;
	struct Sub_Unit RS485;
	struct Sub_Unit Set;
} ;

extern struct Unit eep;

//============== ADE9000 Data ============

/* 개별 레지스터 구조 */
typedef struct {
    uint16_t addr;   // 주소
    union {
        uint32_t u32;
        uint16_t u16;
        int32_t s32;
    } DIGIT;
    double rd;
} ADE9000_Reg;

/* ADE9000 맵 */
typedef struct
{
    /* Phase A */
    ADE9000_Reg AIGAIN,  AIGAIN0,  AIGAIN1,  AIGAIN2,  AIGAIN3,  AIGAIN4;
    ADE9000_Reg APHCAL0, APHCAL1, APHCAL2, APHCAL3, APHCAL4;
    ADE9000_Reg AVGAIN, AIRMSOS, AVRMSOS, APGAIN, AWATTOS, AVAROS, AVA;
    ADE9000_Reg AFWATTOS, AFVAROS, AIFRMSOS, AVFRMSOS, AVRMSONEOS, AIRMSONEOS;
    ADE9000_Reg AIRMS1012OS, AVRMS1012OS, AIRMS1012, AVRMS1012;

    /* Phase B */
    ADE9000_Reg BIGAIN,  BIGAIN0,  BIGAIN1,  BIGAIN2,  BIGAIN3,  BIGAIN4;
    ADE9000_Reg BPHCAL0, BPHCAL1, BPHCAL2, BPHCAL3, BPHCAL4;
    ADE9000_Reg BVGAIN, BIRMSOS, BVRMSOS, BPGAIN, BWATTOS, BVAROS, BVA;
    ADE9000_Reg BFWATTOS, BFVAROS, BIFRMSOS, BVFRMSOS, BVRMSONEOS, BIRMSONEOS;
    ADE9000_Reg BIRMS1012OS, BVRMS1012OS, BIRMS1012, BVRMS1012;

    /* Phase C */
    ADE9000_Reg CIGAIN,  CIGAIN0,  CIGAIN1,  CIGAIN2,  CIGAIN3,  CIGAIN4;
    ADE9000_Reg CPHCAL0, CPHCAL1, CPHCAL2, CPHCAL3, CPHCAL4;
    ADE9000_Reg CVGAIN, CIRMSOS, CVRMSOS, CPGAIN, CWATTOS, CVAROS, CVA;
    ADE9000_Reg CFWATTOS, CFVAROS, CIFRMSOS, CVFRMSOS, CVRMSONEOS, CIRMSONEOS;
    ADE9000_Reg CIRMS1012OS, CVRMS1012OS, CIRMS1012, CVRMS1012;

    /* RMS/Power */
    ADE9000_Reg AIRMS, BIRMS, CIRMS;
    ADE9000_Reg AVRMS, BVRMS, CVRMS;
    ADE9000_Reg AIFRMS, BIFRMS, CIFRMS;
    ADE9000_Reg AVFRMS, BVFRMS, CVFRMS;
    ADE9000_Reg AWATT,  BWATT,  CWATT;
    ADE9000_Reg AVAR,   BVAR,   CVAR;
    ADE9000_Reg WATT_ACC, AWATT_ACC, BWATT_ACC, CWATT_ACC;
    ADE9000_Reg VAR_ACC, AVAR_ACC, BVAR_ACC, CVAR_ACC;
    ADE9000_Reg VA_ACC, AVA_ACC, BVA_ACC, CVA_ACC;

    /* PF, THD */
    ADE9000_Reg APF,  AVTHD,  AITHD;
    ADE9000_Reg BPF,  BVTHD,  BITHD;
    ADE9000_Reg CPF,  CVTHD,  CITHD;

    /* ITDD */
    ADE9000_Reg AITDD, BITDD, CITDD;

    /* One- Cycle */
    ADE9000_Reg AIRMSONE, AVRMSONE;
    ADE9000_Reg BIRMSONE, BVRMSONE;
    ADE9000_Reg CIRMSONE, CVRMSONE;

    /* Peak*/
    ADE9000_Reg IPEAK, VPEAK;

    /* Frequency */
    ADE9000_Reg APERIOD, BPERIOD, CPERIOD,COM_PERIOD;

    /* Angle */
    ADE9000_Reg ANGL_VA_VB, ANGL_VB_VC, ANGL_VA_VC;
    ADE9000_Reg ANGL_VA_IA, ANGL_VB_IB, ANGL_VC_IC;
    ADE9000_Reg ANGL_IA_IB, ANGL_IB_IC, ANGL_IA_IC;

    /* Initial */
    ADE9000_Reg PART_ID, RUN, VERSION, ACCMODE;
    ADE9000_Reg EP_CFG;
    ADE9000_Reg CONFIG0;
    ADE9000_Reg CONFIG3;

    /*MultiPoint Scaling*/
    ADE9000_Reg MTTHR_L0, MTTHR_L1, MTTHR_L2, MTTHR_L3, MTTHR_L4;
    ADE9000_Reg MTTHR_H0, MTTHR_H1, MTTHR_H2, MTTHR_H3, MTTHR_H4;
} ADE9000_Map;

/* 전역 선언 */
extern ADE9000_Map ADE9000;

//============== usart data ============
struct rx_tx_data{
  uint8_t buf[256];
  int enable;
  int disable;
  int pos;
  int end;
  int end_cnt;
  int end_time;
  int SEQ;
  char data;
  unsigned int add_sum;
  unsigned int chk_sum;
};
struct uart_data{
  int master_slave;
  long master_check_cnt;
  long hex_dec_ena_cnt;
  int init_port;
  long init_cnt;
  long init_cnt_ref;
  unsigned int hex_dex_ena;
  float hex_dec;

  struct rx_tx_data txd, rxd;
};
extern struct uart_data uart2, uart6;

struct Modbus_data {
    // Set Data
    uint16_t Part_ID;
    uint16_t Version;
    uint16_t RUN;
    uint16_t ACCMODE;
    uint16_t EP_CFG;
    uint16_t CONFIG3;
    uint16_t SeqErr;
    uint16_t PM_Flag;
    uint16_t CT_Primary;
    uint16_t CT_Secondary;
    uint16_t Init_EEP;

    // RMS
    uint16_t AIRMS;
    uint16_t BIRMS;
    uint16_t CIRMS;
    uint16_t IRMS_AVG;
    uint16_t ABVRMS;
    uint16_t BCVRMS;
    uint16_t CAVRMS;
    uint16_t LINE_VRMS_AVG;
    uint16_t VARMS;
    uint16_t VBRMS;
    uint16_t VCRMS;
    uint16_t PHASE_VRMS_AVG;

    // Peak
    uint16_t VPEAK;

    // Power
    uint16_t AWATT;
    uint16_t BWATT;
    uint16_t CWATT;
    uint16_t WATT_TOT;
    int16_t  AVAR;
    int16_t  BVAR;
    int16_t  CVAR;
    int16_t  VAR_TOT;
    uint16_t AVA;
    uint16_t BVA;
    uint16_t CVA;
    uint16_t CVA_TOT;

    // Energy
//    uint16_t AWATT_ACC_HI;
//    uint16_t AWATT_ACC_LO;
//    uint16_t BWATT_ACC_HI;
//    uint16_t BWATT_ACC_LO;
//    uint16_t CWATT_ACC_HI;
//    uint16_t CWATT_ACC_LO;
    uint16_t WATT_ACC_T_HI;
    uint16_t WATT_ACC_T_LO;
//    int16_t  AVAR_ACC_HI;
//    int16_t  AVAR_ACC_LO;
//    int16_t  BVAR_ACC_HI;
//    int16_t  BVAR_ACC_LO;
//    int16_t  CVAR_ACC_HI;
//    int16_t  CVAR_ACC_LO;
    int16_t  VAR_ACC_T_HI;
    int16_t  VAR_ACC_T_LO;
//    uint16_t AVA_ACC_HI;
//    uint16_t AVA_ACC_LO;
//    uint16_t BVA_ACC_HI;
//    uint16_t BVA_ACC_LO;
//    uint16_t CVA_ACC_HI;
//    uint16_t CVA_ACC_LO;
    uint16_t VA_ACC_T_HI;
    uint16_t VA_ACC_T_LO;

    // PF
    int16_t  APF;
    int16_t  BPF;
    int16_t  CPF;
    int16_t  PF_AVG;

    // THD
    uint16_t AVTHD;
    uint16_t BVTHD;
    uint16_t CVTHD;
    uint16_t VTHD_AVG;
    uint16_t AITHD;
    uint16_t BITHD;
    uint16_t CITHD;
    uint16_t ITHD_AVG;

    // TDD
    uint16_t AITDD;
    uint16_t BITDD;
    uint16_t CITDD;
    uint16_t ITDD_AVG;

    // Frequency
    uint16_t AFREQ;
    uint16_t BFREQ;
    uint16_t CFREQ;
    uint16_t FREQ_AVG;

    // Angle
    int16_t  ANGL_VA_VB;
    int16_t  ANGL_VB_VC;
    int16_t  ANGL_VA_VC;
    int16_t  ANGL_VA_IA;
    int16_t  ANGL_VB_IB;
    int16_t  ANGL_VC_IC;
    int16_t  ANGL_IA_IB;
    int16_t  ANGL_IB_IC;
    int16_t  ANGL_IA_IC;

	uint16_t I_rated;

    uint16_t  Firmware_year;
    uint16_t  Firmware_mon;
    uint16_t  Firmware_date;
    uint16_t  Firmware_Version;
    uint16_t  PM_comm_Reset;
    uint16_t  PM_kwh_comm_Reset;
};

extern struct Modbus_data comm_data;

#endif /* INCLUDE_STRUCT_H_ */
