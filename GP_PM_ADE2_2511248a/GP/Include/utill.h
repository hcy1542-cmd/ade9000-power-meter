/*
 * utill.h
 *
 *  Created on: Sep 29, 2025
 *      Author: user
 */

#ifndef INCLUDE_UTILL_H_
#define INCLUDE_UTILL_H_
#define	BIT_MASK(x)      (1 << (x))

#define	SetBit(val, bit)	((val) |= BIT_MASK(bit))
#define	ClrBit(val, bit)	((val) &= ~BIT_MASK(bit))

#define CPU_Clock 100.e6
#define Fsamp	10.e3	//interrupt time 10kHz, 0.0001s
#define Tsamp   (float)(1./ Fsamp)
#define Fix_Fsamp	((Fsamp * 1.e-3) + 0.5)

#define REF_TIME_usec(usec) (usec * Fix_Fsamp * 0.001)
#define REF_TIME_msec(msec) (((float)msec * Fsamp * 1.e-3 ) + 0.5)
#define REF_TIME_sec(sec)   (((float)sec * Fsamp) + 0.5)


#endif /* INCLUDE_UTILL_H_ */
