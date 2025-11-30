/*
 * iwdg.h
 *
 *  Created on: May 15, 2023
 *      Author: user
 */

#ifndef INCLUDE_IWDG_H_
#define INCLUDE_IWDG_H_


#include "def.h"


bool iwdgInit(void);
bool iwdgBegin(uint32_t time_ms);
bool iwdgRefresh(void);


#endif /* INCLUDE_IWDG_H_ */
