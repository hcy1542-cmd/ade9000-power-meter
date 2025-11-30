/***************************************************************
    easyStm32LL.h
	v10   @ May 2021 : first release
	v10.1 @ Nov 2021 : 1. address alignment check before reading from and writing to
					   2. memory access not allowed once communication error happened
					   3. max 128B bytes reading enabled
					   4. STM32U5 support
	v10.5 @ Nov 2022 : 1. use flash EMPTY bit to enter boot loader for G0x series
                       2. support dual core MCU with D caching
                       3. use FIFO if enabled
	v10.8 @ Mar 2023 : 1. STM32C0 support
	v10.9 @ Jul 2023 : 1. STM32H5 and STM32WBA support
                       2. disable error interrupt
	v11.2 @ May 2024 : 1. STM32U0 support
	v11.3 @ Aug 2024 : 1. STM32WB0 support
****************************************************************/
#ifndef _EASYSTM32LL_H_
#define _EASYSTM32LL_H_

/////////////////////////////////////////////////////////////////////////////////////////////
// Select target MCU series :
// Define 1 to target MCU. 0 to all others.
/////////////////////////////////////////////////////////////////////////////////////////////
#define STM32C0XX			0
#define STM32F0XX			0
#define STM32F1XX			0
#define STM32F2XX			0
#define STM32F3XX			0
#define STM32F4XX			1
#define STM32F7XX			0
#define STM32G0XX			0
#define STM32G4XX			0
#define STM32H5XX			0
#define STM32H7XX			0
#define STM32H7RSXX			0		// STM32H7Rx/7Sx
#define STM32L0XX			0
#define STM32L1XX			0
#define STM32L4XX			0
#define STM32L5XX			0
#define STM32U0XX			0
#define STM32U5XX			0
#define STM32WBXX			0
#define STM32WBAXX			0
#define STM32WB0XX			0
#define STM32WLXX			0

/////////////////////////////////////////////////////////////////////////////////////////////
// In case of STM32H7 only :
// Define 1 if dual core MCU such as STM32H745x, STM32H747x, STM32H755x and STM32H757x
// Define 0 if single core MCU
// It is not relevant whether you use actually dual core or not
/////////////////////////////////////////////////////////////////////////////////////////////
#if STM32H7XX
#define EZ_DUAL_CORE		0
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// In case of dual core MCU :
// Define 1 if easyDSP pod is connected to this core
// Define 0 if easyDSP pod is not connected to this core
//////////////////////////////////////////////////////////////////////////////////////////
#if EZ_DUAL_CORE
#define EASYDSP_IS_CONNECTED_TO_THIS_CORE 		1
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// In case of STM32H7 dual core MCU :
// Define 1 if CPU D-cache is used (so that the communication between cores are via SEV to avoid cache coherence issue)
// Define 0 if CPU D-cache is not used (so that the communication between cores are done by direct memory access)
//////////////////////////////////////////////////////////////////////////////////////////
#if EZ_DUAL_CORE
#define EZ_USE_SEV_INT 				0
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// In case EZ_USE_SEV_INT = 1 :
// Define the address of shared memory for the communication between cores.
// The byte size of shared memory is 32 bytes (refer to 'shared_data' structure in the source file).
// by default, the first 32 bytes of SRAM4 (starting 0x38000000) is used below.
// This SRAM4 is suggested since it is outside both domains of both CPU cores, not affecting to low-power features of each domain.
// You can change the location of shared memory accordingly to your application.
// Note : The shared memory should be not used by the application. Please make it sure in the linker script file.
//////////////////////////////////////////////////////////////////////////////////////////
#if EZ_USE_SEV_INT
#define EZ_SHARED_MEM_ADDRESS	0x38000000
#endif

/////////////////////////////////////////////
// Please don't change anything below
/////////////////////////////////////////////
#if STM32C0XX
#include "stm32c0xx_ll_usart.h"
#elif STM32G0XX
#include "stm32g0xx_ll_usart.h"
#elif STM32G4XX
#include "stm32g4xx_ll_usart.h"
#elif STM32H5XX
#include "stm32h5xx_ll_usart.h"
#elif STM32H7XX
#include "stm32h7xx_ll_usart.h"
#elif STM32H7RSXX
#include "stm32h7rsxx_ll_usart.h"
#if EZ_DUAL_CORE
#include <stm32h7xx_ll_cortex.h>
#endif
#elif STM32F0XX
#include "stm32f0xx_ll_usart.h"
#elif STM32F1XX
#include "stm32f1xx_ll_usart.h"
#elif STM32F2XX
#include "stm32f2xx_ll_usart.h"
#elif STM32F3XX
#include "stm32f3xx_ll_usart.h"
#elif STM32F4XX
#include "stm32f4xx_ll_usart.h"
#elif STM32F7XX
#include "stm32f7xx_ll_usart.h"
#elif STM32L0XX
#include "stm32l0xx_ll_usart.h"
#elif STM32L1XX
#include "stm32l1xx_ll_usart.h"
#elif STM32L4XX
#include "stm32l4xx_ll_usart.h"
#elif STM32L5XX
#include "stm32l5xx_ll_usart.h"
#elif STM32U0XX
#include "stm32u0xx_ll_usart.h"
#elif STM32U5XX
#include "stm32u5xx_ll_usart.h"
#elif STM32WBXX
#include "stm32wbxx_ll_usart.h"
#elif STM32WBAXX
#include "stm32wbaxx_ll_usart.h"
#elif STM32WB0XX
#include "stm32wb0x_ll_usart.h"
#elif STM32WLXX
#include "stm32wlxx_ll_usart.h"
#else
no MCU defined !
#endif

#if !EZ_DUAL_CORE || !defined(EZ_DUAL_CORE)
#define EASYDSP_IS_CONNECTED_TO_THIS_CORE 		1
#define EZ_USE_SEV_INT							0
#endif

void easyDSP_init(USART_TypeDef *USARTx);
#if EASYDSP_IS_CONNECTED_TO_THIS_CORE
void ez_USARTx_IRQHandler(void);
#endif
#if EZ_USE_SEV_INT && STM32H7XX
void ez_SEV_IRQHandler(void);
#endif

#endif /* _EASYSTM32LL_H_ */
