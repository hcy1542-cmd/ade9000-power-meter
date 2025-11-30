/*
 * ADE9000.h
 *
 *  Created on: Aug 15, 2025
 *      Author: hcy
 */

#ifndef INCLUDE_ADE9000_H_
#define INCLUDE_ADE9000_H_

#include "def.h"


extern void ADE9000_Measurement(void);

void ADE9000_Read_Monitoring(void);
void ADE9000_DAC(void);

extern bool Seq_Check;
extern bool Init_EEP;

extern double LINE_VRMS_AVG;
extern double IRMS_AVG;
extern double AVRMS1012_LN, BVRMS1012_LN, CVRMS1012_LN, VRMS1012_LN_AVG;
extern double ADE9000_Active_Power, ADE9000_Reactive_Power, ADE9000_Apparent_Power, ADE9000_PowerFactor;
extern double VTHD_AVG, ITHD_AVG, ITDD_AVG;

#define U32_LIMIT 429496729UL
#define I32_LIMIT 2147483647L

#endif /* INCLUDE_ADE9000_H_ */
