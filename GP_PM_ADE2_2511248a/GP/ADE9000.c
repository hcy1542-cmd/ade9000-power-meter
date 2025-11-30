/*
 * ADE9000.c
 *
 *  Created on: Aug 15, 2025
 *      Author: user
 */

#include "def.h"
#include <math.h>

//struct 초기화
ADE9000_Map ADE9000 = {
	//  Phase GAIN //
	.AIGAIN       = {0x000,}, .BIGAIN       = {0x020,}, .CIGAIN       = {0x040,},
	.AVGAIN       = {0x00B,}, .BVGAIN       = {0x02B,}, .CVGAIN       = {0x04B,},

	//  RMS(Display Data)  //
	.AIRMS1012			= {0x21B,}, .AVRMS1012			= {0x21C,},
	.BIRMS1012			= {0x23B,}, .BVRMS1012			= {0x23C,},
	.CIRMS1012			= {0x25B,}, .CVRMS1012			= {0x25C,},

	// RMS - (2) //
	.AIRMS			= {0x20C,}, .BIRMS			= {0x22C,}, .CIRMS			= {0x24C,},
	.AVRMS			= {0x20D,}, .BVRMS			= {0x22D,}, .CVRMS			= {0x24D,},
	.AIFRMS			= {0x20E,}, .BIFRMS			= {0x22E,}, .CIFRMS			= {0x24E,},			// Fundamental	Current
	.AVFRMS			= {0x20F,}, .BVFRMS			= {0x22F,}, .CVFRMS			= {0x24F,},			// Fundamental	Voltage

	//  Power(Display Data)  //
	.AWATT			= {0x210,}, .BWATT			= {0x230,},	.CWATT			= {0x250,},
	.AVAR			= {0x211,}, .BVAR			= {0x231,}, .CVAR			= {0x251,},
	.AVA			= {0x212,}, .BVA			= {0x232,}, .CVA			= {0x252,},

	//  PF(Display Data) / THD(Display Data)  //
	.APF			= {0x216,}, .AVTHD			= {0x217,}, .AITHD			= {0x218,},
	.BPF			= {0x236,}, .BVTHD			= {0x237,}, .BITHD			= {0x238,},
	.CPF			= {0x256,}, .CVTHD			= {0x257,}, .CITHD			= {0x258,},

	//  Peak  //
	.IPEAK        = {0x400,}, .VPEAK        = {0x401,},

	//  Frequency(Display Data)  //
	.APERIOD      = {0x418,}, .BPERIOD      = {0x419,}, .CPERIOD      = {0x41A,}, .COM_PERIOD   = {0x41B,},

	// Angle //
	.ANGL_VA_VB			= {0x482,}, .ANGL_VB_VC			= {0x483,}, .ANGL_VA_VC			= {0x484,},
	.ANGL_VA_IA			= {0x485,}, .ANGL_VB_IB			= {0x486,}, .ANGL_VC_IC			= {0x487,},
	.ANGL_IA_IB			= {0x488,}, .ANGL_IB_IC			= {0x489,}, .ANGL_IA_IC			= {0x48A,},

	//  RUN  //
	.RUN				= {0x480,},

	//  Version Check //
	.VERSION			= {0x4FE,},

	//	Setup  //
	.ACCMODE			= {0x492,},
	.EP_CFG				= {0x4B0,},
//	.CONFIG0			= {0x060,},
	.CONFIG3			= {0x493,},

	.AIGAIN0      = {0x001,}, .AIGAIN1      = {0x002,}, .AIGAIN2      = {0x003,}, .AIGAIN3      = {0x004,}, .AIGAIN4      = {0x005,},
	.BIGAIN0      = {0x021,}, .BIGAIN1      = {0x022,}, .BIGAIN2      = {0x023,}, .BIGAIN3      = {0x024,}, .BIGAIN4      = {0x025,},
	.CIGAIN0      = {0x041,}, .CIGAIN1      = {0x042,}, .CIGAIN2      = {0x043,}, .CIGAIN3      = {0x044,}, .CIGAIN4      = {0x045,},

};

// ADE9000 Data Masking & parsing
void ADE9000_Read_Monitoring(void){

	// Voltage //
	ADE9000.AVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.AVRMS1012.addr) ;
	ADE9000.BVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.CVRMS1012.addr) ;		// ADE9000 Phase C - A상보다 120도 뒤짐
	ADE9000.CVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.BVRMS1012.addr) ;		// ADE9000 Phase B - A상보다 120도 앞섬

//	ADE9000.AVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.AVRMS1012.addr) ;
//	ADE9000.CVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.BVRMS1012.addr) ;
//	ADE9000.BVRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.CVRMS1012.addr) ;

	// Current //
	ADE9000.AIRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.AIRMS1012.addr);
	ADE9000.BIRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.BIRMS1012.addr);
	ADE9000.CIRMS1012.DIGIT.u32 = ADE_Read32(ADE9000.CIRMS1012.addr);

	ADE9000.AIFRMS.DIGIT.u32 = ADE_Read32(ADE9000.AIFRMS.addr);
	ADE9000.BIFRMS.DIGIT.u32 = ADE_Read32(ADE9000.BIFRMS.addr);
	ADE9000.CIFRMS.DIGIT.u32 = ADE_Read32(ADE9000.CIFRMS.addr);

	// Frequency //
	ADE9000.APERIOD.DIGIT.u32 = ADE_Read32(ADE9000.APERIOD.addr);
	ADE9000.BPERIOD.DIGIT.u32 = ADE_Read32(ADE9000.BPERIOD.addr);
	ADE9000.CPERIOD.DIGIT.u32 = ADE_Read32(ADE9000.CPERIOD.addr);
	ADE9000.COM_PERIOD.DIGIT.u32 = ADE_Read32(ADE9000.COM_PERIOD.addr);

	// ANGLE //
	ADE9000.ANGL_VA_VB.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VA_VB.addr);
	ADE9000.ANGL_VB_VC.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VB_VC.addr);
	ADE9000.ANGL_VA_VC.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VA_VC.addr);

	ADE9000.ANGL_VA_IA.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VA_IA.addr);
	ADE9000.ANGL_VB_IB.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VB_IB.addr);
	ADE9000.ANGL_VC_IC.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_VC_IC.addr);

	ADE9000.ANGL_IA_IB.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_IA_IB.addr);
	ADE9000.ANGL_IB_IC.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_IB_IC.addr);
	ADE9000.ANGL_IA_IC.DIGIT.u16 = ADE_Read16(ADE9000.ANGL_IA_IC.addr);

	// Power //
	// 역률이 30도 차이가 발생한 결과로 데이터가 발생하여 계산치로 대체함.
	//	ADE9000.AWATT.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.AWATT.addr);
	//	ADE9000.BWATT.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.BWATT.addr);
	//	ADE9000.CWATT.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.CWATT.addr);
	//
	//	ADE9000.AVAR.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.AVAR.addr);
	//	ADE9000.BVAR.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.BVAR.addr);
	//	ADE9000.CVAR.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.CVAR.addr);
	//
	//	ADE9000.AVA.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.AVA.addr);
	//	ADE9000.BVA.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.BVA.addr);
	//	ADE9000.CVA.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.CVA.addr);
	//
	//	ADE9000.APF.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.APF.addr);
	//	ADE9000.BPF.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.BPF.addr);
	//	ADE9000.CPF.DIGIT.s32 =  (int32_t)ADE_Read32(ADE9000.CPF.addr);

	ADE9000.AVTHD.DIGIT.u32 = ADE_Read32(ADE9000.AVTHD.addr);
	ADE9000.BVTHD.DIGIT.u32 = ADE_Read32(ADE9000.BVTHD.addr);
	ADE9000.CVTHD.DIGIT.u32 = ADE_Read32(ADE9000.CVTHD.addr);

	ADE9000.AITHD.DIGIT.u32 = ADE_Read32(ADE9000.AITHD.addr);
	ADE9000.BITHD.DIGIT.u32 = ADE_Read32(ADE9000.BITHD.addr);
	ADE9000.CITHD.DIGIT.u32 = ADE_Read32(ADE9000.CITHD.addr);

}

// Angle 범위 제한
float wrap180f(float x){
    x = fmodf(x + 180.0f, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x - 180.0f;      // -180..+180
}

// deg2rad : 삼각함수 계산할 떄 사용
static inline float deg2rad(float deg)
{
	return deg * (float)M_PI / 180.0f;
}

// Power Factor 부호
double pf_signed_lagpos_from_deg(double phase_deg){
    double phi = deg2rad(phase_deg);
    double pf = cosf(phi);              // +: lag, −: lead

    if (pf >  1.0f) pf = 1.0f;
    if (pf < -1.0f) pf = -1.0f;
    return pf;
}

// ADE9000 상수
#define ADE9000_FS_CODE		52702092.		// Full-scale code for Vrms
#define ADE9000_VFS_RMS		0.707			// Full-scale Vrms at PGA=1
#define ADE9000_IFS_RMS_2	16.0			//  0.707 / 0.044
double divRatio   = 991.0;				 // 분압비 (1/990)
double ADE_Q27  = (134217728.0);   		 /* 2^27 */
double ADE_Q27_INV = 1.0 / 134217728.0;

float ANGLE_GAIN_60Hz = 0.02109375;
float ANGLE_GAIN_50Hz = 0.017578125;
float Offset_ANGLE_1 = -30.0;
float Offset_ANGLE_2 = 0;
float Offset_ANGLE_3 = 0;

float ANGLE_VAB,ANGLE_VBC, ANGLE_VCA;
float ANGLE_VA_IA, ANGLE_VB_IB, ANGLE_VC_IC;
float ANGLE_IA_IB, ANGLE_IB_IC, ANGLE_IC_IA;
float ANGLE_VA, ANGLE_VB, ANGLE_VC;

double ANGLE_IB;

uint16_t Make_ACCMODE(uint32_t frequency, uint32_t ct_count)
{
    uint16_t reg = 0;

    // === bit 8: SELFREQ ===
    if (frequency == 60)
        reg |= (1 << 8);   // 60Hz
    else
        reg &= ~(1 << 8);  // 50Hz

    // === bit 7: ICONSEL ===
    // 2CT면 1, 3CT면 0
    if (ct_count == 2)
        reg |= (1 << 7);   // 2CT mode
    else
        reg &= ~(1 << 7);  // 3CT mode

    // === bit 6:4 - VCONSEL = 100 (3-wire delta)
    reg |= (0b100 << 4);

    // === bit 3:2 - VARACC = 00 (signed)
    reg |= (0b00 << 2);

    // === bit 1:0 - WATTACC = 00 (signed)
    reg |= 0b00;

    return reg;
}

double Theta_B_Gain = 1.0;

void ADE9000_Init_Const(void){

//	ADE9000.ACCMODE.DIGIT.u16 = 0x1C0; 	// bin : 0001 1100 0000				// 60Hz, IB = -IA - IB, Delta 2CT

	ADE9000.ACCMODE.DIGIT.u16 = Make_ACCMODE(eep.Set.Frequency.val, eep.CT.Num.val);

	ADE9000.EP_CFG.DIGIT.u16 = 0x13;	// THD 계산 : 고정값

	ADE9000.CONFIG3.DIGIT.u16 = 0x1C;	// 최대값 구하기 : 고정값

	ADE_Write32(ADE9000.AIGAIN.addr, 0x0000);
	ADE_Write32(ADE9000.CIGAIN.addr, 0x0000);

	eep.I_Gain.B.fscale_new = 1;

	eep.CT.Ratio.val = (float)eep.CT.Primary.val / (float)eep.CT.Secondary.val;
}

bool Init_EEP = 1;
void ADE9000_Init_EEP(void)
{
	// ADE9000 load시 기존에 사용했던 SCALE 적용
	if(Init_EEP == 1)
	{
		ADE_Write32(ADE9000.AVGAIN.addr, eep.V_Gain.AB.nscale);
		ADE_Write32(ADE9000.BVGAIN.addr, eep.V_Gain.BC.nscale);
		ADE_Write32(ADE9000.CVGAIN.addr, eep.V_Gain.CA.nscale);

		ADE_Write32(ADE9000.AIGAIN.addr, eep.I_Gain.A.nscale);
		ADE_Write32(ADE9000.CIGAIN.addr, eep.I_Gain.C.nscale);
	}
}

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

void Calc_IB_ADE9000(
        double IA,       // AIRMS1012.rd
        double IC,       // CIRMS1012.rd
        double thA,      // ANGLE_IA (도)
        double dtheta_AC,// ADE9000.ANGL_IA_IC.rd (도)
        double* IB_mag,
        double* thB_deg)
{
    double dth = DEG2RAD(dtheta_AC);

    // --- B상 크기 ---
    *IB_mag = sqrt(IA*IA + IC*IC + 2.0 * IA * IC * cos(dth));

    // --- B상 위상 ---
    double num = IC * sin(dth);
    double den = IA + IC * cos(dth);

    *thB_deg = thA + RAD2DEG(atan2(num, den));
}

double ADE9000_Active_Power, ADE9000_Reactive_Power, ADE9000_Apparent_Power, ADE9000_PowerFactor;
double LINE_VRMS_AVG, IRMS_AVG;
double AVRMS1012_LN, BVRMS1012_LN, CVRMS1012_LN, VRMS1012_LN_AVG;
double VTHD_AVG, ITHD_AVG, ITDD_AVG;
double temp_Mag;

void ADE9000_SCALE(void){

	// Voltage Scale //
	float VFS_RMS_SCALE = ADE9000_VFS_RMS * divRatio;

	ADE9000.AVRMS1012.rd = (double)ADE9000.AVRMS1012.DIGIT.u32 * VFS_RMS_SCALE / ADE9000_FS_CODE;
	ADE9000.BVRMS1012.rd = (double)ADE9000.BVRMS1012.DIGIT.u32 * VFS_RMS_SCALE / ADE9000_FS_CODE;
	ADE9000.CVRMS1012.rd = (double)ADE9000.CVRMS1012.DIGIT.u32 * VFS_RMS_SCALE / ADE9000_FS_CODE;
	LINE_VRMS_AVG = (ADE9000.AVRMS1012.rd + ADE9000.BVRMS1012.rd + ADE9000.CVRMS1012.rd) / 3.0;

	AVRMS1012_LN = ADE9000.AVRMS1012.rd / sqrt(3);
	BVRMS1012_LN = ADE9000.BVRMS1012.rd / sqrt(3);
	CVRMS1012_LN = ADE9000.CVRMS1012.rd / sqrt(3);
	VRMS1012_LN_AVG = (AVRMS1012_LN + BVRMS1012_LN + CVRMS1012_LN) / 3.0;

	// Current Scale //
	ADE9000.AIRMS1012.rd = (double)ADE9000.AIRMS1012.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;
//	ADE9000.BIRMS1012.rd = (double)ADE9000.BIRMS1012.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;

	ADE9000.BIRMS1012.rd = sqrt(ADE9000.AIRMS1012.rd*ADE9000.AIRMS1012.rd +
							ADE9000.CIRMS1012.rd*ADE9000.CIRMS1012.rd +
							2.0*ADE9000.AIRMS1012.rd*ADE9000.CIRMS1012.rd *
							cos(DEG2RAD(ADE9000.ANGL_IA_IC.rd) / (float)Theta_B_Gain));

	ADE9000.CIRMS1012.rd = (double)ADE9000.CIRMS1012.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;
	IRMS_AVG = (ADE9000.AIRMS1012.rd + ADE9000.BIRMS1012.rd + ADE9000.CIRMS1012.rd) / 3.0;

	ADE9000.AIFRMS.rd = (double)ADE9000.AIFRMS.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;
	ADE9000.BIFRMS.rd = (double)ADE9000.BIFRMS.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;
	ADE9000.CIFRMS.rd = (double)ADE9000.CIFRMS.DIGIT.u32 * ADE9000_IFS_RMS_2 / ADE9000_FS_CODE * eep.CT.Ratio.val;

	// Frequency Scale // Half Cycle / 2 = One Cycle
	ADE9000.APERIOD.rd = 8000. * (float)(2 << 16) / ((float)ADE9000.APERIOD.DIGIT.u32 + 1) / 2.;
	ADE9000.BPERIOD.rd = 8000. * (float)(2 << 16) / ((float)ADE9000.BPERIOD.DIGIT.u32 + 1) / 2.;
	ADE9000.CPERIOD.rd = 8000. * (float)(2 << 16) / ((float)ADE9000.CPERIOD.DIGIT.u32 + 1) / 2.;
	ADE9000.COM_PERIOD.rd = 8000. * (float)(2 << 16) / ((float)ADE9000.COM_PERIOD.DIGIT.u32 + 1) / 2.;

	// ANGLE Scale //
	ADE9000.ANGL_VA_VB.rd = wrap180f(ADE9000.ANGL_VA_VB.DIGIT.u16  * ANGLE_GAIN_60Hz + Offset_ANGLE_1);
	ADE9000.ANGL_VB_VC.rd = wrap180f(ADE9000.ANGL_VB_VC.DIGIT.u16  * ANGLE_GAIN_60Hz + Offset_ANGLE_1) * -1;
	ADE9000.ANGL_VA_VC.rd = wrap180f(ADE9000.ANGL_VA_VC.DIGIT.u16  * ANGLE_GAIN_60Hz + Offset_ANGLE_1);

	ANGLE_VA = ADE9000.ANGL_VA_VB.rd - 30.0;
	ANGLE_VB = ADE9000.ANGL_VA_VC.rd - 30.0;
	ANGLE_VC = ADE9000.ANGL_VB_VC.rd - 30.0;

	ADE9000.ANGL_VA_IA.rd = wrap180f((ADE9000.ANGL_VA_IA.DIGIT.u16  * ANGLE_GAIN_60Hz));
	ADE9000.ANGL_VB_IB.rd = wrap180f((ADE9000.ANGL_VB_IB.DIGIT.u16  * ANGLE_GAIN_60Hz));
	ADE9000.ANGL_VC_IC.rd = wrap180f((ADE9000.ANGL_VC_IC.DIGIT.u16  * ANGLE_GAIN_60Hz));

	ADE9000.ANGL_IA_IB.rd = wrap180f(ADE9000.ANGL_IA_IB.DIGIT.u16  * ANGLE_GAIN_60Hz);
	ADE9000.ANGL_IB_IC.rd = wrap180f(ADE9000.ANGL_IB_IC.DIGIT.u16  * ANGLE_GAIN_60Hz);
	ADE9000.ANGL_IA_IC.rd = wrap180f(ADE9000.ANGL_IA_IC.DIGIT.u16  * ANGLE_GAIN_60Hz);

	Calc_IB_ADE9000(ADE9000.AIRMS1012.rd, ADE9000.CIRMS1012.rd, 0.0, ADE9000.ANGL_IA_IB.rd, &temp_Mag, &ANGLE_IB);

	double VAIA_Phase_Offset = -30.0;
	double VBIB_Phase_Offset = -90.0;
	double VCIC_Phase_Offset = 30.0;

	ADE9000.ANGL_VA_IA.rd = wrap180f(ADE9000.ANGL_VA_IA.rd) + VAIA_Phase_Offset;
	ADE9000.ANGL_VB_IB.rd = wrap180f(ADE9000.ANGL_VB_IB.rd) + VBIB_Phase_Offset;
	ADE9000.ANGL_VC_IC.rd = wrap180f(ADE9000.ANGL_VC_IC.rd) + VCIC_Phase_Offset;

	// Power Factor //
	ADE9000.APF.rd = pf_signed_lagpos_from_deg((double)ADE9000.ANGL_VA_IA.rd);
	ADE9000.BPF.rd = pf_signed_lagpos_from_deg((double)ADE9000.ANGL_VB_IB.rd);
	ADE9000.CPF.rd = pf_signed_lagpos_from_deg((double)ADE9000.ANGL_VC_IC.rd);

	ADE9000_PowerFactor = (ADE9000.APF.rd + ADE9000.BPF.rd + ADE9000.CPF.rd) / 3.0;

	// Electric Power //
	ADE9000.AVA.rd = ADE9000.AVRMS1012.rd * ADE9000.AIRMS1012.rd * 0.001 / sqrt(3);	// 0.001 -> k 단위 변환
	ADE9000.BVA.rd = ADE9000.BVRMS1012.rd * ADE9000.BIRMS1012.rd * 0.001 / sqrt(3);
	ADE9000.CVA.rd = ADE9000.CVRMS1012.rd * ADE9000.CIRMS1012.rd * 0.001 / sqrt(3);
	ADE9000_Apparent_Power = ADE9000.AVA.rd + ADE9000.BVA.rd + ADE9000.CVA.rd;

	ADE9000.AWATT.rd = ADE9000.AVA.rd * fabs(ADE9000.APF.rd);
	ADE9000.BWATT.rd = ADE9000.BVA.rd * fabs(ADE9000.BPF.rd);
	ADE9000.CWATT.rd = ADE9000.CVA.rd * fabs(ADE9000.CPF.rd);
	ADE9000_Active_Power = ADE9000.AWATT.rd + ADE9000.BWATT.rd + ADE9000.CWATT.rd;

	ADE9000.AVAR.rd = ADE9000.AVA.rd * sqrt(1.0 - ADE9000.APF.rd * ADE9000.APF.rd);
	ADE9000.BVAR.rd = ADE9000.BVA.rd * sqrt(1.0 - ADE9000.BPF.rd * ADE9000.BPF.rd);
	ADE9000.CVAR.rd = ADE9000.CVA.rd * sqrt(1.0 - ADE9000.CPF.rd * ADE9000.CPF.rd);
	ADE9000_Reactive_Power = ADE9000.AVAR.rd + ADE9000.BVAR.rd + ADE9000.CVAR.rd;
	// THD //
	ADE9000.AVTHD.rd = (double)( (int32_t)ADE9000.AVTHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	ADE9000.BVTHD.rd = (double)( (int32_t)ADE9000.BVTHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	ADE9000.CVTHD.rd = (double)( (int32_t)ADE9000.CVTHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	VTHD_AVG = (ADE9000.AVTHD.rd + ADE9000.BVTHD.rd + ADE9000.CVTHD.rd) / 3.0;

	ADE9000.AITHD.rd = (double)( (int32_t)ADE9000.AITHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	ADE9000.BITHD.rd = (double)( (int32_t)ADE9000.BITHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	ADE9000.CITHD.rd = (double)( (int32_t)ADE9000.CITHD.DIGIT.u32 ) * ADE_Q27_INV * 100.0;
	ITHD_AVG = (ADE9000.AITHD.rd + ADE9000.BITHD.rd + ADE9000.CITHD.rd) / 3.0;

	ADE9000.AITDD.rd = ADE9000.AITHD.rd * ADE9000.AIFRMS.rd / (float)eep.Set.I_rated.val;
	ADE9000.BITDD.rd = ADE9000.BITHD.rd * ADE9000.BIFRMS.rd / (float)eep.Set.I_rated.val;
	ADE9000.CITDD.rd = ADE9000.CITHD.rd * ADE9000.CIFRMS.rd / (float)eep.Set.I_rated.val;
	ITDD_AVG = (ADE9000.AITDD.rd + ADE9000.BITDD.rd + ADE9000.CITDD.rd) / 3.0;
}


double BURDEN_OHM = 0.044;	// 22mohm 직렬 연결
void ADE9000_Calc_IAgain(double _IA_measured)
{

	ADE_Write32(ADE9000.AIGAIN.addr, 0x00000000);

	//ADE_Write16(ADE9000.RUN.addr, 0x0000);

	double I_Scale_Const = (ADE9000_VFS_RMS) / BURDEN_OHM;		// PGA_GAIN = 00

	// 기대 값 계산
	double AI_Expected = (_IA_measured / I_Scale_Const / eep.CT.Ratio.val) * ADE9000_FS_CODE;

	double gA = AI_Expected / ADE9000.AIRMS1012.DIGIT.u32;

	// 2^27 형식으로 변환
	double xIA_ADE = (gA - 1.0) * (double)(1 << 27);   // 실수 곱셈

	eep.I_Gain.A.nscale_new = (int32_t)(xIA_ADE);

	EEPROM_Gain_Write(eep.I_Gain.A.nscale_new, 12);

	ADE_Write32(ADE9000.AIGAIN.addr, (uint32_t)eep.I_Gain.A.nscale_new);

}
double IB_ref, IB_new;
double COS_Theta_B_ref, Theta_B_ref;
double Theta_Offset_DEG, Theta_B_ref_DEG, COS_Theta_B_ref_DEG;
double DeltaTheta ;
void hcy_Calc_IBgain(double _IB_measured)
{
    double IA = ADE9000.AIRMS1012.rd;
    double IC = ADE9000.CIRMS1012.rd;
    IB_ref = _IB_measured;

    // 1) ADE9000 angle 읽고 degree -> rad
    DeltaTheta = DEG2RAD(ADE9000.ANGL_IA_IC.rd);

    // 2) cos(theta_ref) 계산
    COS_Theta_B_ref =
        (IB_ref*IB_ref - IA*IA - IC*IC) / (2.0 * IA * IC);

    // 2-1) 범위 제한
    COS_Theta_B_ref = fmax(-1.0, fmin(1.0, COS_Theta_B_ref));
    COS_Theta_B_ref_DEG = RAD2DEG(COS_Theta_B_ref);

    // 3) 목표 위상 (라디안)
    Theta_B_ref = acos(COS_Theta_B_ref);
    Theta_B_ref_DEG = RAD2DEG(Theta_B_ref);

    // 4) Offset (라디안 단위)
    Theta_B_Gain = DeltaTheta / Theta_B_ref;

    eep.I_Gain.B.fscale_new = (Theta_B_Gain);

    EEPROM_Gain_Write((int32_t)(eep.I_Gain.B.fscale_new * 1000), 16);
}


void ADE9000_Calc_ICgain(double _IC_measured)
{
	ADE_Write32(ADE9000.CIGAIN.addr, 0x00000000);

	double I_Scale_Const = (ADE9000_VFS_RMS) / BURDEN_OHM;		// PGA_GAIN = 00

	// 기대 값 계산
	double CI_Expected = (_IC_measured / I_Scale_Const / eep.CT.Ratio.val) * ADE9000_FS_CODE;

	double gC = CI_Expected / ADE9000.CIRMS1012.DIGIT.u32;

    // 2^27 형식으로 변환
    double xIC_ADE = (gC - 1.0) * (double)(1 << 27);   // 실수 곱셈

    eep.I_Gain.C.nscale_new = (int32_t)(xIC_ADE);

    EEPROM_Gain_Write(eep.I_Gain.C.nscale_new, 20);

    ADE_Write32(ADE9000.CIGAIN.addr, (uint32_t)eep.I_Gain.C.nscale_new);
}


double hour_Const = 1.0 / 3600.0;		// 2초마다 계산할 경우 hour_Const =  1.0 / 1800.0
double prev_VA = 0, prev_WATT = 0, prev_VAR = 0;

void Calculation_Energy(){

    double cur_VA   = ADE9000.AVA.rd   + ADE9000.BVA.rd   + ADE9000.CVA.rd;
    double cur_WATT = ADE9000.AWATT.rd + ADE9000.BWATT.rd + ADE9000.CWATT.rd;
    double cur_VAR  = ADE9000.AVAR.rd  + ADE9000.BVAR.rd  + ADE9000.CVAR.rd;

    ADE9000.VA_ACC.rd += cur_VA * hour_Const;
    ADE9000.WATT_ACC.rd += cur_WATT * hour_Const;
    ADE9000.VAR_ACC.rd += cur_VAR * hour_Const;

    // 동시 초기화? 개별 초기화?
    // 0으로 초기화? or 최대값 빼기?
    if ((ADE9000.WATT_ACC.rd > U32_LIMIT) || (ADE9000.VA_ACC.rd > U32_LIMIT) || \
			(ADE9000.VAR_ACC.rd > I32_LIMIT || ADE9000.VAR_ACC.rd < -I32_LIMIT))
    {
    	ADE9000.WATT_ACC.rd = 0;
    	ADE9000.VA_ACC.rd = 0;
    	ADE9000.VAR_ACC.rd = 0;
    }

	if (flag_10min)
	{
		flag_10min = 0;

		EEPROM_WriteEnergy(ADE9000.VA_ACC.rd, ADE9000.WATT_ACC.rd, ADE9000.VAR_ACC.rd);
	}
}

// ADE9000_Measurement() Sequence Index
typedef enum {
    ADE_COM_TEST   		= 0,
	ADE_EEPROM			= 1,
    ADE_RUN				= 2,
    ADE_DATA_READ  		= 3,
	VABC_CALIBRATION 	= 7,
	IA_CALIBRATION		= 8,
    IC_CALIBRATION		= 9,
	IB_CALIBRATION		= 10,
	EEPROM_RESET_A		= 11,
	EEPROM_RESET_B		= 12,
	ADE_HW_Reset		= 99
} ADE_Mode;

int PM_Flag = 0;

bool Seq_Check;
// 상순 에러 체크
bool ADE9000_CheckSeqErr(void)
{
    uint32_t status1 = ADE_Read32(0x403);

    if (status1 & (1 << 18)) return true;   // SEQERR 비트
    else return false;
}

int32_t VAB_ADE_GAIN, VBC_ADE_GAIN, VCA_ADE_GAIN;
double AV_Expected, BV_Expected, CV_Expected;
void ADE9000_Delta_Voltage_Cal(double Vab_ref, double Vbc_ref, double Vca_ref)
{
    // --- 1. ADE에서 읽은 선간전압 (RMS) 계산 ---
    double Vab_meas = (ADE9000.AVRMS1012.DIGIT.u32 / ADE9000_FS_CODE) * ADE9000_VFS_RMS * divRatio;
    double Vbc_meas = (ADE9000.BVRMS1012.DIGIT.u32 / ADE9000_FS_CODE) * ADE9000_VFS_RMS * divRatio;
    double Vca_meas = (ADE9000.CVRMS1012.DIGIT.u32 / ADE9000_FS_CODE) * ADE9000_VFS_RMS * divRatio;

    // --- 2. 보정 비율 계산 ---
    double kab = Vab_ref / Vab_meas;
    double kbc = Vbc_ref / Vbc_meas;
    double kca = Vca_ref / Vca_meas;

    double GA = (kab + kca - kbc);
    double GB = (kbc + kab - kca);
    double GC = (kca + kbc - kab);

    // --- 4. ADE9000의 GAIN 레지스터로 변환 (2^27 스케일) ---
    eep.V_Gain.AB.nscale_new = (int32_t)((GA - 1.0) * (1 << 27));
    eep.V_Gain.BC.nscale_new = (int32_t)((GB - 1.0) * (1 << 27));
    eep.V_Gain.CA.nscale_new = (int32_t)((GC - 1.0) * (1 << 27));

    // --- 6. ADE9000에 쓰기 ---
    ADE_Write32(ADE9000.AVGAIN.addr, eep.V_Gain.AB.nscale_new);
    ADE_Write32(ADE9000.BVGAIN.addr, eep.V_Gain.BC.nscale_new);
    ADE_Write32(ADE9000.CVGAIN.addr, eep.V_Gain.CA.nscale_new);

    EEPROM_Gain_Write(eep.V_Gain.AB.nscale_new, 0);
    EEPROM_Gain_Write(eep.V_Gain.BC.nscale_new, 4);
    EEPROM_Gain_Write(eep.V_Gain.CA.nscale_new, 8);
}

double IA_measured, IB_measured ,IC_measured;
double VAB_measured, VBC_measured, VCA_measured;
double VA, WATT, VAR;
double VA2, WATT2, VAR2;
uint8_t EEP_GET_OK;

uint64_t EEP_Index_R;
char LED_Status = 0;
volatile uint8_t flag_100ms;
volatile uint8_t flag_1s;
volatile uint8_t flag_1s_2;
volatile uint8_t flag_5min;
volatile uint8_t flag_10min;

int Test_Read_Value, Test_Read_Addr;
int Test_Write_Value;
int Fill_OK;


void ADE9000_Measurement(void)
{
	switch(PM_Flag)
	{
		// ADE_COM_TEST : 0 -> ADE9000 IC SPI Communication Check
		case ADE_COM_TEST :	//Version check

			ADE9000.VERSION.DIGIT.u16 = ADE_Read16(ADE9000.VERSION.addr);

			ADE9000.VERSION.DIGIT.u16 = ADE9000.VERSION.DIGIT.u16 & 0xFFC0;		//Version Read Masking -> Register Map 참조(& 0xFFC0)


			PM_Flag = 1;
			break;

		// ADE_EEPROM : 1 -> EEPROM DATA READ
		case ADE_EEPROM :

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);		// Status LED Init
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);		// Status LED Init

			// 1. Initial Constant
			ADE9000_Init_Const();

			// 2. Gain Set
			ADE9000_Init_EEP();

			// 3. RUN Initial
			ADE_Write16(ADE9000.RUN.addr, 0x0000);

			// 4.Connection Definition : Delta Connection 적용
			ADE_Write16(ADE9000.ACCMODE.addr, ADE9000.ACCMODE.DIGIT.u16);

			// 5. Power Calc Setup
			ADE_Write16(ADE9000.EP_CFG.addr, ADE9000.EP_CFG.DIGIT.u16);							// 누적 동작(Energy, THD)

			// 6. Peak
			ADE_Write16(ADE9000.CONFIG3.addr, ADE9000.CONFIG3.DIGIT.u16);

			// 7. Flag System RUN
			PM_Flag = ADE_RUN;
			break;

		// ADE_RUN : 2 ->System RUN
		case ADE_RUN :
			// 1. RUN
			ADE_Write16(ADE9000.RUN.addr, 0x0001);

			// 2. Flag Check
			PM_Flag = ADE_DATA_READ;

			break;

		 // ADE_DATA_READ : 3 -> Data Read (Data Acquisition)
		case ADE_DATA_READ :

			// 1. Calc Data transfer ADE9000 to DSP
			ADE9000_Read_Monitoring();

			// 2.Scale
			ADE9000_SCALE();

			// 3. 상순 에러 체크
			Seq_Check = ADE9000_CheckSeqErr();
			if(Seq_Check){
				LED_Status = 1;
			}

			// 4. EEPROM Store 			// Period : 1s
		    if(flag_1s)		// 1초마다 전력량 누적 (Timer Period = 0.1s)
		    {
		    	flag_1s = 0;

				// Peak //
				ADE9000.VPEAK.DIGIT.u32 = ADE_Read32(ADE9000.VPEAK.addr);

				ADE9000.VPEAK.DIGIT.u32 &= 0x00FFFFFF;

				// 값이 튀는것 방지(Peak 1000V(2365662) 이상일 경우 계산하지 않음)
				if(ADE9000.VPEAK.DIGIT.u32 < 2365662)	ADE9000.VPEAK.rd = (double)ADE9000.VPEAK.DIGIT.u32 / (ADE9000_FS_CODE * 1.414) * 32.0 * divRatio;   // [V RMS 기준]

				// 누적 전력량 계산 //
				Calculation_Energy();	// Energy Calculation
		    }

		    //--- EEPROM Test Code - Start
//		    ADE9000.VA_ACC.rd += 1.110001;
//		    ADE9000.WATT_ACC.rd += 10.11001;
//		    ADE9000.VAR_ACC.rd -= 100.11001;
//
//			EEPROM_WriteEnergy(ADE9000.VA_ACC.rd, ADE9000.WATT_ACC.rd, ADE9000.VAR_ACC.rd);

//		    EEPROM_ReadEnergy(EEP_Index - 1, &EEP_Index_R ,&ADE9000.VA_ACC.rd, &ADE9000.WATT_ACC.rd, &ADE9000.VAR_ACC.rd);

		    //--- EEPROM Test Code - End

			break;
		case VABC_CALIBRATION : // Delta Voltage Scale

			ADE9000_Delta_Voltage_Cal(VAB_measured, VBC_measured, VCA_measured);

		    PM_Flag = ADE_DATA_READ;

			break;
		case IA_CALIBRATION : // IA Calibration

			ADE9000_Calc_IAgain(IA_measured);

		    PM_Flag = ADE_DATA_READ;
			break;

		case IC_CALIBRATION : // IC Calibration

			ADE9000_Calc_ICgain(IC_measured);

		    PM_Flag = ADE_DATA_READ;
			break;

		case IB_CALIBRATION :

			hcy_Calc_IBgain(IB_measured);

			PM_Flag = ADE_DATA_READ;
			break;

		case EEPROM_RESET_A :

//			EEPROM_Format_A();

			PM_Flag = ADE_HW_Reset;

			break;

		case EEPROM_RESET_B :

			EEPROM_Format_B();

			PM_Flag = ADE_HW_Reset;

			break;

		case ADE_HW_Reset :

			HAL_NVIC_SystemReset();

			PM_Flag = -1;

			break ;

		default : // System Stop
			// ADE9000 Stop
			ADE_Write16(ADE9000.RUN.addr, 0x0000);

			break;
	}


}


