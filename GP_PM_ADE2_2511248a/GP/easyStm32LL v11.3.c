/***************************************************************
    easyStm32LL.c
****************************************************************/
// Included Files
#include "easyStm32LL v11.3.h"

// function declaration
#if EASYDSP_IS_CONNECTED_TO_THIS_CORE
static inline void easyDSP_AddRing(uint8_t u8TxData);
void easyDSP_Tx_Start(void);
#endif

// only for test purpose
// comment for easyDSP release version
//#define EASYSTM32_DEBUG
#ifdef EASYSTM32_DEBUG
uint8_t ez_u8RxBuff[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t ez_u8RxBuffCnt = 0;
uint16_t ez_u16BreakSeqCount1 = 0, ez_u16BreakSeqCount2 = 0;
uint16_t ez_u16RxAlreadyCount = 0;
uint8_t ez_u8RepeatedRxCount = 0, ez_u8RepeatedRxCountMax = 0;
#endif

//#define EASYSTM32_ERROR

////////////////////////////////////////////////////////////////////////////////////
#if EASYDSP_IS_CONNECTED_TO_THIS_CORE
////////////////////////////////////////////////////////////////////////////////////
// function declaration
void ez_USART_CharReception_Callback(void);

// easyDSP commands & statesC/C++ Build → Settings → Tool Settings → MCU GCC Compiler → Includes 에 GP/Include
#define STAT_INIT	0
#define STAT_ADDR	1
#define STAT_DATA2B	2
#define STAT_DATA4B	3
#define STAT_WRITE	4
#define STAT_DATA8B	5
#define STAT_DATA1B	6

#define CMD_ADDR			0xE7
#define CMD_ADDR_32BIT      0x41    // not used but reserved
#define CMD_READ1B			0x12
#define	CMD_READ2B			0xDB
#define CMD_READ4B			0xC3
#define CMD_READ8B			0x8B
#define CMD_READ16B			0x28	// not used but reserved
#define CMD_READ128B		0xA5	// v10.1
#define CMD_DATA1B			0x30
#define	CMD_DATA2B			0xBD
#define	CMD_DATA4B			0x99
#define	CMD_DATA8B			0x64
#define	CMD_WRITE			0x7E
#define	CMD_FB_READ			0x0D
#define	CMD_FB_WRITE_OK		0x0D
#define CMD_FB_WRITE_NG		0x3C
#define CMD_BL_AUTOBAUD		0x7F	// not used but reserved
#define CMD_SPECIAL_OP		0xF6	// v10.5

// for internal use
#define VERSION_OF_THIS_FILE	1130 // xx.y.z
uint16_t ez_u16Version;

// for easyDSP
uint8_t ez_u8Rx, ez_u8State = STAT_INIT;
uint16_t ez_u16Chksum = 0;
uint32_t ez_u32Addr = 0;	// don't change the variable name
uint8_t ez_u8AddrRdCnt = 0, ez_u8DataRdCnt = 0;
uint8_t ez_u8Data;
uint16_t ez_u16Data;
uint32_t ez_u32Data;
uint64_t ez_u64Data;
USART_TypeDef *ezUSARTx = 0;
uint32_t ez_u32FlashReg;	// v10.5 : done't change the variable name
uint8_t ez_u8CoreNow;		// v10.5 : 1 = CPU1, 2 = CPU2

#if !defined(USART_CR1_FIFOEN)
// for fast reading in chart window only for MCU series without FIFO
// uncomment if memory size is limited in your application
#define MULTI_BYTE_READING
#endif

// Tx buffer related
#ifdef MULTI_BYTE_READING
#define EZ_TX_BUFF_COUNT	(128+1)	// max 128B + 1B for CMD_FB_READ
uint8_t ez_u8MultiReadingIndex;		// the name of this variable should not be changed.
uint8_t ez_u8MultiReadingCount = EZ_TX_BUFF_COUNT - 1;
#else
#define EZ_TX_BUFF_COUNT	10			// ideally it's necessary upto 9
#endif
uint8_t ez_u8aTxData[EZ_TX_BUFF_COUNT];
uint8_t ez_u8TxDataCnt = 0, ez_u8TxDataCntMax = 0;
uint8_t ez_u8TxDataSentCnt = 0;

uint8_t ez_u8IsFifoEnabled;
#endif
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
#if EZ_DUAL_CORE
////////////////////////////////////////////////////////////////////////////////////
uint8_t ez_u8Core = 0;		// v10.5 : 1 = CPU1, 2 = CPU2
uint32_t ez_u32PartNo;
#endif
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
#if EZ_USE_SEV_INT
////////////////////////////////////////////////////////////////////////////////////
#define EZ_CMD_READ             0xEA
#define EZ_CMD_WRITE            0xAE
uint32_t ez_u32aSevIntMainCounter[3] = {0,0,0};
uint32_t ez_u32aSevIntRemoteCounter[3] = {0,0,0};
//uint32_t ez_u32SameIndexErrCount = 0;
uint32_t ez_u32OtherErrCount = 0;
uint32_t ez_u32WrongIndexErrCount = 0, ez_u32SevReEnteredCount = 0;
uint8_t ez_u8IpcIndex = 0, ez_u8IpcIndexOld = 0, ez_u8IpcIndexCount;	// do not change the name "ez_u8IpcIndex"
#define EZ_SHARED_MEMORY_SIZE	16
struct shared_data
{
	uint8_t u8aMain2Remote[EZ_SHARED_MEMORY_SIZE];
	uint8_t u8aRemote2Main[EZ_SHARED_MEMORY_SIZE];
};
volatile struct shared_data * const ez_pSharedMem = (struct shared_data *)EZ_SHARED_MEM_ADDRESS;
#endif
////////////////////////////////////////////////////////////////////////////////////

void easyDSP_init(USART_TypeDef *USARTx)
{
#if	EASYDSP_IS_CONNECTED_TO_THIS_CORE
	ezUSARTx = USARTx;

	// Polling USART initialization
#if defined(USART_ISR_TEACK) && defined(USART_ISR_REACK)
	while ((!(LL_USART_IsActiveFlag_TEACK(ezUSARTx))) || (!(LL_USART_IsActiveFlag_REACK(ezUSARTx))));
#endif

	// Clear Overrun flag, in case characters have already been sent to USART
	LL_USART_ClearFlag_ORE(ezUSARTx);

	// Enable RXNE and Error interrupts
	LL_USART_EnableIT_RXNE(ezUSARTx);	// Enable RX Not Empty and RX FIFO Not Empty Interrupt
#ifdef EASYSTM32_ERROR
	LL_USART_EnableIT_ERROR(ezUSARTx);	// FE, NE, ORE
	LL_USART_EnableIT_PE(ezUSARTx);		// PE
#endif
	// just to keep it alive
	ez_u16Version = VERSION_OF_THIS_FILE;

#if defined(USART_CR1_FIFOEN)
	ez_u8IsFifoEnabled = LL_USART_IsEnabledFIFO(ezUSARTx);
#else
	ez_u8IsFifoEnabled = 0;
#endif



#endif

#if EZ_DUAL_CORE
	ez_u32PartNo = LL_CPUID_GetParNo();
#if STM32H7XX
	if(ez_u32PartNo == 0xC27) 		ez_u8Core = 1;	// CPU1 (M7)
	else if(ez_u32PartNo == 0xC24) ez_u8Core = 2;	// CPU2 (M4)
	else ez_u8Core = 0;
//#elif STM32WLXX
//	if(ez_u32PartNo == 0xC24) 		ez_u8Core = 1;	// CPU1 (M4)
//	else if(ez_u32PartNo == 0xC60) 	ez_u8Core = 2;;	// CPU1 (M0+)
#endif
#endif

#if EZ_USE_SEV_INT
#if	EASYDSP_IS_CONNECTED_TO_THIS_CORE
	uint16_t i;
	for(i = 0; i < EZ_SHARED_MEMORY_SIZE; i++) {
		ez_pSharedMem->u8aMain2Remote[i] = 0;
		ez_pSharedMem->u8aRemote2Main[i] = 0;
	}
#endif
#endif
}

////////////////////////////////////////////////////////////////////////////////////
#if EZ_USE_SEV_INT && STM32H7XX
////////////////////////////////////////////////////////////////////////////////////
void ez_SEV_IRQHandler(void)
{
#if	EASYDSP_IS_CONNECTED_TO_THIS_CORE

    ez_u32aSevIntMainCounter[0]++;	// entered

    uint8_t index, cmd, bytecount;
    index 		= ez_pSharedMem->u8aRemote2Main[0];
    cmd 		= ez_pSharedMem->u8aRemote2Main[1];
    bytecount 	= ez_pSharedMem->u8aRemote2Main[2];

    // valid ?
    if(ez_u8IpcIndex != index) 		{ ez_u32WrongIndexErrCount++; 	return; }
    if(ez_u8IpcIndexCount++ != 0) 	{ ez_u32SevReEnteredCount++; return; } // re-entered

    ez_u32aSevIntMainCounter[1]++;	// valid

    if(cmd == EZ_CMD_READ) {
        uint64_t u64Value;          // to align to 8 bytes address
        u64Value = *(uint64_t *)(ez_pSharedMem->u8aRemote2Main + 3);     // is it ok for address alignment ?

        if(bytecount == 1) {
            easyDSP_AddRing(u64Value);
            easyDSP_AddRing(CMD_FB_READ);
            easyDSP_Tx_Start();
        }
        else if(bytecount == 2) {
            easyDSP_AddRing(u64Value >> 8);    // MSB
            easyDSP_AddRing(u64Value);         // LSB
            easyDSP_AddRing(CMD_FB_READ);
            easyDSP_Tx_Start();
        }
        else if(bytecount == 4) {
            easyDSP_AddRing(u64Value >> 8*3);   // MSB
            easyDSP_AddRing(u64Value >> 8*2);
            easyDSP_AddRing(u64Value >> 8);
            easyDSP_AddRing(u64Value);         // LSB
            easyDSP_AddRing(CMD_FB_READ);
            easyDSP_Tx_Start();
        }
        else if(bytecount == 8) {
            easyDSP_AddRing(u64Value >> 8*7);   // MSB
            easyDSP_AddRing(u64Value >> 8*6);
            easyDSP_AddRing(u64Value >> 8*5);
            easyDSP_AddRing(u64Value >> 8*4);
            easyDSP_AddRing(u64Value >> 8*3);
            easyDSP_AddRing(u64Value >> 8*2);
            easyDSP_AddRing(u64Value >> 8*1);
            easyDSP_AddRing(u64Value);         // LSB
            easyDSP_AddRing(CMD_FB_READ);
            easyDSP_Tx_Start();
        }
        else {
        	ez_u32OtherErrCount++;
            return;
        }
    }
    else if(cmd == EZ_CMD_WRITE) {
    	uint8_t done = ez_pSharedMem->u8aRemote2Main[3];
		if(done == 0xAA)    easyDSP_AddRing(CMD_FB_WRITE_OK);
		else                easyDSP_AddRing(CMD_FB_WRITE_NG);
		easyDSP_Tx_Start();
    }
    else {
    	ez_u32OtherErrCount++;
        return;
    }

    ez_u32aSevIntMainCounter[2]++;		// completed
    //if(ez_u8Core == 2)
    //	HAL_Delay(ez_u8SevDelay);			// to minimize re entrance

#else

    ez_u32aSevIntRemoteCounter[0]++;	// entered

    uint64_t u64Value;          // to align to 8 bytes address
    uint8_t index, cmd, bytecount;
    uint32_t addr;
    index 		= ez_pSharedMem->u8aMain2Remote[0];
    cmd 		= ez_pSharedMem->u8aMain2Remote[1];
    bytecount 	= ez_pSharedMem->u8aMain2Remote[2];
    addr		= *(uint32_t *)(ez_pSharedMem->u8aMain2Remote + 3);     // align ok??????????????

    // valid ?
    if(index == ez_u8IpcIndexOld) 	{ ez_u32SevReEnteredCount++;   	return; }	// re-enter

    ez_u32aSevIntRemoteCounter[1]++;	// valid

    if(cmd == EZ_CMD_READ) {

        // sanity. delete later since it is already confirmed at easyDSP_ISR()
        if(bytecount == 1);
        else if((bytecount == 2) && (addr % 2 == 0)) ;
        else if((bytecount == 4) && (addr % 4 == 0)) ;
        else if((bytecount == 8) && (addr % 8 == 0)) ;
        else { ez_u32OtherErrCount++; return; }			// 실패시 피드백 버퍼를 싹 다 비워야 하나?

        // read
        if(bytecount == 1)      u64Value = *(uint8_t*) addr;
        else if(bytecount == 2) u64Value = *(uint16_t*)addr;
        else if(bytecount == 4) u64Value = *(uint32_t*)addr;
        else if(bytecount == 8) u64Value = *(uint64_t*)addr;
        else { ez_u32OtherErrCount++; return; }

        // feedback
        uint16_t msgSize = 0;
        ez_pSharedMem->u8aRemote2Main[msgSize++] = index;		// index
        ez_pSharedMem->u8aRemote2Main[msgSize++] = cmd;			// command
        ez_pSharedMem->u8aRemote2Main[msgSize++] = bytecount;	// byte count
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*0);   // LSB first
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*1);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*2);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*3);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*4);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*5);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*6);
        ez_pSharedMem->u8aRemote2Main[msgSize++]  = u64Value >> (8*7);

        // send the event to main core
        ez_u8IpcIndexOld = index;
        __SEV();
    }
    else if(cmd == EZ_CMD_WRITE) {
		u64Value = *(uint64_t*)(ez_pSharedMem->u8aMain2Remote + 7);    // align ok??????????????

		// sanity. delete later since it is already confirmed at easyDSP_ISR()
		if(bytecount == 1);
		else if((bytecount == 2) && (addr % 2 == 0)) ;
		else if((bytecount == 4) && (addr % 4 == 0)) ;
		else if((bytecount == 8) && (addr % 8 == 0)) ;
		else { ez_u32OtherErrCount++; return; }

		// write
		if(bytecount == 1)      *(uint8_t*)addr  = u64Value;
		else if(bytecount == 2) *(uint16_t*)addr = u64Value;
		else if(bytecount == 4) *(uint32_t*)addr = u64Value;
		else if(bytecount == 8) *(uint64_t*)addr = u64Value;
		else { ez_u32OtherErrCount++; return; }

		// feedback
		uint16_t msgSize = 0;
		ez_pSharedMem->u8aRemote2Main[msgSize++] = index;		// index
		ez_pSharedMem->u8aRemote2Main[msgSize++] = cmd;			// command
		ez_pSharedMem->u8aRemote2Main[msgSize++] = bytecount;	// byte count
		ez_pSharedMem->u8aRemote2Main[msgSize++]  = 0xAA;   	// done signal

		// send the event to main core
        ez_u8IpcIndexOld = index;
        __SEV();
    }
    else {
    	ez_u32OtherErrCount++;
    	return;
    }

    ez_u32aSevIntRemoteCounter[2]++;	// completed
    //if(ez_u8Core == 2)
    //	HAL_Delay(ez_u8SevDelay);			// to minimize re entrance

#endif
}

////////////////////////////////////////////////////////////////////////////////////
#if	EASYDSP_IS_CONNECTED_TO_THIS_CORE
////////////////////////////////////////////////////////////////////////////////////
static void easyDSP_ReadRemoteCore(uint8_t u8Core, uint32_t u32Addr, uint8_t u8ByteCount)
{
    uint16_t msgSize = 0;
    ez_u8IpcIndex++;
    ez_u8IpcIndexCount = 0;

    ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u8IpcIndex;
    ez_pSharedMem->u8aMain2Remote[msgSize++] = EZ_CMD_READ;      	// read
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u8ByteCount;        	// n bytes
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (0*8);   	// address. LSB first
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (1*8);
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (2*8);
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (3*8);

    // send the event to remote core
    __SEV();
}

static inline void easyDSP_WriteRemoteCore(uint8_t u8Core, uint32_t u32Addr, uint8_t u8ByteCount)
{
    uint16_t msgSize = 0;
    ez_u8IpcIndex++;
    ez_u8IpcIndexCount = 0;

    ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u8IpcIndex;      	// index
    ez_pSharedMem->u8aMain2Remote[msgSize++] = EZ_CMD_WRITE;  		// write
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u8ByteCount;        	// n bytes
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (0*8);   	// address. LSB first
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (1*8);
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (2*8);
    ez_pSharedMem->u8aMain2Remote[msgSize++] = u32Addr >> (3*8);
    if(u8ByteCount == 1) ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u8Data;
    else if(u8ByteCount == 2) {
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u16Data;           // LSB first
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u16Data >> 8);
    }
    else if(u8ByteCount == 4) {
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u32Data;           // LSB first
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u32Data >> 8*1);
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u32Data >> 8*2);
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u32Data >> 8*3);
    }
    else if(u8ByteCount == 8) {
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = ez_u64Data;           // LSB first
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*1);
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*2);
    	ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*3);
        ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*4);
        ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*5);
        ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*6);
        ez_pSharedMem->u8aMain2Remote[msgSize++] = (ez_u64Data >> 8*7);
    }
    else ez_u32OtherErrCount++;

    // send the event to remote core
    __SEV();
}
#endif // EASYDSP_IS_CONNECTED_TO_THIS_CORE
#endif // EZ_USE_SEV_INT
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
#if EASYDSP_IS_CONNECTED_TO_THIS_CORE
////////////////////////////////////////////////////////////////////////////////////
// store bytes to be sent
static inline void easyDSP_AddRing(uint8_t u8TxData)
{
	if(ez_u8IsFifoEnabled) LL_USART_TransmitData8(ezUSARTx, u8TxData);
	else {
		if(ez_u8TxDataCnt < EZ_TX_BUFF_COUNT) ez_u8aTxData[ez_u8TxDataCnt++] = u8TxData;
#ifdef	EASYSTM32_DEBUG
		if(ez_u8TxDataCnt > ez_u8TxDataCntMax) ez_u8TxDataCntMax = ez_u8TxDataCnt;
#endif
	}
}

// Send one byte > adjust ez_u8TxDataSentCnt > enable Tx int to send next byte
void easyDSP_Tx_Start(void)
{
	if(!ez_u8IsFifoEnabled) {
		LL_USART_EnableIT_TXE(ezUSARTx);
		LL_USART_TransmitData8(ezUSARTx, ez_u8aTxData[0]);
		ez_u8TxDataSentCnt = 1;
	}
}

#ifdef EASYSTM32_ERROR
// Error related
uint16_t ez_u16PECount = 0, ez_u16NECount = 0, ez_u16FECount = 0, ez_u16ORECount = 0;
uint16_t ez_u16TotalErrorCount = 0;
#endif
uint8_t ez_u8ErrorFlag = 0;

void ez_USARTx_IRQHandler(void)
{
#ifdef EASYSTM32_DEBUG
	ez_u8RepeatedRxCount = 0;
#endif

	// receive char
	// RXNE flag will be cleared by reading of RDR register in the callback
	// v10.1 : if -> while to minimize ORE
	while (LL_USART_IsActiveFlag_RXNE(ezUSARTx)) {
		ez_u8Rx = LL_USART_ReceiveData8(ezUSARTx);
		ez_USART_CharReception_Callback();
#ifdef EASYSTM32_DEBUG
		ez_u8RepeatedRxCount++;
#endif
	}


#ifdef EASYSTM32_DEBUG
	if(ez_u8RepeatedRxCount > ez_u8RepeatedRxCountMax) ez_u8RepeatedRxCountMax = ez_u8RepeatedRxCount;
	if(LL_USART_IsActiveFlag_RXNE(ezUSARTx)) {
		ez_u16RxAlreadyCount++;
	}
#endif

	if(!ez_u8IsFifoEnabled) {
		// send char
		// TXE flag will be automatically cleared when writing new data in TDR register
		if(LL_USART_IsEnabledIT_TXE(ezUSARTx) && LL_USART_IsActiveFlag_TXE(ezUSARTx))
		{
			if(ez_u8TxDataSentCnt == ez_u8TxDataCnt) {		// if all sent
				LL_USART_DisableIT_TXE(ezUSARTx);			// Disable TXE interrupt
				ez_u8TxDataSentCnt = ez_u8TxDataCnt = 0;	// preparation for AddRing()
			}
			else {
				LL_USART_TransmitData8(ezUSARTx, ez_u8aTxData[ez_u8TxDataSentCnt]);
				ez_u8TxDataSentCnt++;
			}
		}
	}

#ifdef EASYSTM32_ERROR
	// Error
	if(LL_USART_IsActiveFlag_NE(ezUSARTx) || LL_USART_IsActiveFlag_ORE(ezUSARTx) || LL_USART_IsActiveFlag_FE(ezUSARTx) || LL_USART_IsActiveFlag_PE(ezUSARTx)) {
		ez_u16TotalErrorCount++;
		ez_u8ErrorFlag = 1;

		// error counter
		if(LL_USART_IsActiveFlag_NE(ezUSARTx)) 	ez_u16NECount++;
		if(LL_USART_IsActiveFlag_ORE(ezUSARTx))	ez_u16ORECount++;
		if(LL_USART_IsActiveFlag_FE(ezUSARTx))	ez_u16FECount++;
		if(LL_USART_IsActiveFlag_PE(ezUSARTx))	ez_u16PECount++;

		// reset error flag
		if(LL_USART_IsActiveFlag_NE(ezUSARTx)) 	LL_USART_ClearFlag_NE(ezUSARTx);
		if(LL_USART_IsActiveFlag_ORE(ezUSARTx))	LL_USART_ClearFlag_ORE(ezUSARTx);
		if(LL_USART_IsActiveFlag_FE(ezUSARTx))	LL_USART_ClearFlag_FE(ezUSARTx);
		if(LL_USART_IsActiveFlag_PE(ezUSARTx))	LL_USART_ClearFlag_PE(ezUSARTx);
	}
#endif
}

// Function called from USART IRQ Handler when RXNE flag is set
// Function is in charge of reading character received on USART RX line.
void ez_USART_CharReception_Callback(void)
{
#ifdef EASYSTM32_DEBUG
	if(ez_u8RxBuffCnt < 10) {
		ez_u8RxBuff[ez_u8RxBuffCnt] = ez_u8Rx;
		ez_u8RxBuffCnt++;
	}
#endif

	////////////////////////////////////////////
	// Parsing by state
	////////////////////////////////////////////
	if(ez_u8State == STAT_INIT) {
		if(ez_u8Rx == CMD_ADDR) {
			ez_u8State = STAT_ADDR;
#if EZ_DUAL_CORE
			ez_u8AddrRdCnt = 0;
#else
			ez_u8AddrRdCnt = 1;
#endif
		}
		else if(ez_u8Rx == CMD_SPECIAL_OP) {	// empty bit = 1
#if STM32C0XX
			ez_u32FlashReg = (FLASH->ACR & ~FLASH_ACR_PROGEMPTY);
			FLASH->ACR = (ez_u32FlashReg | FLASH_ACR_PROGEMPTY);
#elif STM32G0XX
			//ez_u32FlashReg = (FLASH->ACR & ~FLASH_ACR_PROGEMPTY);
			//FLASH->ACR = (ez_u32FlashReg | FLASH_ACR_PROGEMPTY);
			ez_u32FlashReg = ((*(uint32_t*)0x40022000) & ~(1<<16));
			ez_u32FlashReg |= (1<<16);
			*(uint32_t*)0x40022000 = ez_u32FlashReg;
#elif STM32WBXX
			ez_u32FlashReg = (FLASH->ACR & ~FLASH_ACR_EMPTY);
			FLASH->ACR = (ez_u32FlashReg | FLASH_ACR_EMPTY);
#elif STM32WLXX
			ez_u32FlashReg = (FLASH->ACR & ~FLASH_ACR_EMPTY);
			FLASH->ACR = (ez_u32FlashReg | FLASH_ACR_EMPTY);
#elif STM32L4XX
			if((FLASH->SR & (1<<17)) == 0) {			// if bit is 0
				ez_u32FlashReg = (FLASH->SR & ~(1<<17));
				FLASH->SR = (ez_u32FlashReg | (1<<17));	// toggling by writing 1
			}
#elif STM32L0XX
			// nop
#elif STM32F0XX
			// not working
			//ez_u32FlashReg = (FLASH->ACR & ~(1<<16));
			//FLASH->ACR = (ez_u32FlashReg | (1<<16));
#endif
		}
#ifdef EASYSTM32_DEBUG
		else if(ez_u8Rx != 0x00)
			ez_u16BreakSeqCount1++;	// 0x00 is dummy byte when fast reading
#endif
	}
	else if(ez_u8State == STAT_ADDR) {
		ez_u8AddrRdCnt++;
		if(ez_u8AddrRdCnt == 1)			ez_u8CoreNow = ez_u8Rx;
		else if(ez_u8AddrRdCnt == 2)	ez_u32Addr = (ez_u8Rx << 24);	// MSB
		else if(ez_u8AddrRdCnt == 3)	ez_u32Addr |= (ez_u8Rx << 16);
		else if(ez_u8AddrRdCnt == 4)	ez_u32Addr |= (ez_u8Rx << 8);
		else if(ez_u8AddrRdCnt == 5)	ez_u32Addr |= ez_u8Rx;			// LSB
		else if(ez_u8AddrRdCnt == 6) {
			if(ez_u8Rx == CMD_READ1B) {
				if(!ez_u8ErrorFlag) {	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {				// v10.5
#endif
						ez_u8Data = *(uint8_t *)ez_u32Addr;
						easyDSP_AddRing(ez_u8Data);
						easyDSP_AddRing(CMD_FB_READ);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_ReadRemoteCore(ez_u8CoreNow, ez_u32Addr, 1);
#endif
				}
				else ez_u8ErrorFlag = 0;
				ez_u8State = STAT_INIT;
			}
			else if(ez_u8Rx == CMD_READ2B) {
				if(!ez_u8ErrorFlag && (ez_u32Addr % 2 == 0)) { 	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {				// v10.5
#endif
						ez_u16Data = *(uint16_t *)ez_u32Addr;
						easyDSP_AddRing(ez_u16Data >> 8);		// MSB
						easyDSP_AddRing(ez_u16Data);			// LSB
						easyDSP_AddRing(CMD_FB_READ);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_ReadRemoteCore(ez_u8CoreNow, ez_u32Addr, 2);
#endif
				}
				else ez_u8ErrorFlag = 0;
				ez_u8State = STAT_INIT;
			}
			else if(ez_u8Rx == CMD_READ4B) {
				if(!ez_u8ErrorFlag && (ez_u32Addr % 4 == 0)) { 	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {				// v10.5
#endif
						ez_u32Data = *(uint32_t *)ez_u32Addr;
						easyDSP_AddRing(ez_u32Data >> 24);		// MSB
						easyDSP_AddRing(ez_u32Data >> 16);
						easyDSP_AddRing(ez_u32Data >> 8);
						easyDSP_AddRing(ez_u32Data);			// LSB
						easyDSP_AddRing(CMD_FB_READ);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_ReadRemoteCore(ez_u8CoreNow, ez_u32Addr, 4);
#endif
				}
				else ez_u8ErrorFlag = 0;
				ez_u8State = STAT_INIT;
			}
			else if(ez_u8Rx == CMD_READ8B) {
				if(!ez_u8ErrorFlag && (ez_u32Addr % 8 == 0)) { 	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {				// v10.5
#endif
						ez_u64Data = *(uint64_t*)ez_u32Addr;
						easyDSP_AddRing(ez_u64Data >> (8*7));	// MSB
						easyDSP_AddRing(ez_u64Data >> (8*6));
						easyDSP_AddRing(ez_u64Data >> (8*5));
						easyDSP_AddRing(ez_u64Data >> (8*4));
						easyDSP_AddRing(ez_u64Data >> (8*3));
						easyDSP_AddRing(ez_u64Data >> (8*2));
						easyDSP_AddRing(ez_u64Data >> (8*1));
						easyDSP_AddRing(ez_u64Data);			// LSB
						easyDSP_AddRing(CMD_FB_READ);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_ReadRemoteCore(ez_u8CoreNow, ez_u32Addr, 8);
#endif
				}
				else ez_u8ErrorFlag = 0;
				ez_u8State = STAT_INIT;
			}
#ifdef MULTI_BYTE_READING
			else if(ez_u8Rx == CMD_READ128B) {
				if(!ez_u8ErrorFlag) {
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {				// v10.5
#endif
						for(ez_u8MultiReadingIndex = 0; ez_u8MultiReadingIndex < ez_u8MultiReadingCount; ez_u8MultiReadingIndex++) {
							ez_u8Data = *(uint8_t*)ez_u32Addr;
							easyDSP_AddRing(ez_u8Data);	// MSB
							ez_u32Addr++;
						}
						easyDSP_AddRing(CMD_FB_READ);
						easyDSP_Tx_Start();
						ez_u8MultiReadingCount = EZ_TX_BUFF_COUNT - 1;
#if EZ_USE_SEV_INT
					}
					else easyDSP_ReadRemoteCore(ez_u8CoreNow, ez_u32Addr, 128);
#endif
				}
				else ez_u8ErrorFlag = 0;
				ez_u8State = STAT_INIT;
			}
#endif
			else if(ez_u8Rx == CMD_DATA1B) {
				ez_u8State = STAT_DATA1B;
				ez_u8DataRdCnt = 0;
			}
			else if(ez_u8Rx == CMD_DATA2B) {
				ez_u8State = STAT_DATA2B;
				ez_u8DataRdCnt = 0;
			}
			else if(ez_u8Rx == CMD_DATA4B) {
				ez_u8State = STAT_DATA4B;
				ez_u8DataRdCnt = 0;
			}
			else if(ez_u8Rx == CMD_DATA8B) {
				ez_u8State = STAT_DATA8B;
				ez_u8DataRdCnt = 0;
			}
			else {
				ez_u8State = STAT_INIT;
#ifdef EASYSTM32_DEBUG
				ez_u16BreakSeqCount2++;
#endif
			}
		}
		else ez_u8State = STAT_INIT;
	}
	else if(ez_u8State == STAT_DATA1B) {
		ez_u8DataRdCnt++;
		if(ez_u8DataRdCnt == 1)			ez_u8Data = ez_u8Rx;
		else if(ez_u8DataRdCnt == 2)	ez_u16Chksum = ez_u8Rx << 8;	// MSB
		else if(ez_u8DataRdCnt == 3)	ez_u16Chksum |= ez_u8Rx;		// LSB
		else if(ez_u8DataRdCnt == 4) {
			if(ez_u8Rx == CMD_WRITE) {
				if(ez_u16Chksum == ((ez_u32Addr + ez_u8Data) & 0xFFFF) && !ez_u8ErrorFlag) { // v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {
#endif
						*(uint8_t*)ez_u32Addr = ez_u8Data;
						easyDSP_AddRing(CMD_FB_WRITE_OK);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
                    }
                    else easyDSP_WriteRemoteCore(ez_u8CoreNow, ez_u32Addr, 1);
#endif
				}
				else {
					ez_u8ErrorFlag = 0;
					easyDSP_AddRing(CMD_FB_WRITE_NG);
					easyDSP_Tx_Start();
				}
			}
			ez_u8State = STAT_INIT;
		}
	}
	else if(ez_u8State == STAT_DATA2B) {
		ez_u8DataRdCnt++;
		if(ez_u8DataRdCnt == 1)			ez_u16Data = ez_u8Rx << 8; 		// MSB
		else if(ez_u8DataRdCnt == 2)	ez_u16Data |= ez_u8Rx; 			// LSB
		else if(ez_u8DataRdCnt == 3)	ez_u16Chksum = ez_u8Rx << 8;	// MSB
		else if(ez_u8DataRdCnt == 4)	ez_u16Chksum |= ez_u8Rx;		// LSB
		else if(ez_u8DataRdCnt == 5) {
			if(ez_u8Rx == CMD_WRITE) {
				if(ez_u16Chksum == ((ez_u32Addr + ez_u16Data) & 0xFFFF) && (ez_u32Addr % 2 == 0) && !ez_u8ErrorFlag) {	// v10.1
#if EZ_USE_SEV_INT
                    if(ez_u8Core == ez_u8CoreNow) {
#endif
						*(uint16_t*)ez_u32Addr = ez_u16Data;
						easyDSP_AddRing(CMD_FB_WRITE_OK);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
                    }
                    else easyDSP_WriteRemoteCore(ez_u8CoreNow, ez_u32Addr, 2);
#endif
				}
				else {
					ez_u8ErrorFlag = 0;
					easyDSP_AddRing(CMD_FB_WRITE_NG);
					easyDSP_Tx_Start();
				}
			}
			ez_u8State = STAT_INIT;
		}
	}
	else if(ez_u8State == STAT_DATA4B) {
		ez_u8DataRdCnt++;
		if(ez_u8DataRdCnt == 1) {
			ez_u32Data = ez_u8Rx; 		// MSB
			ez_u32Data <<= 8;
		}
		else if(ez_u8DataRdCnt == 2 || ez_u8DataRdCnt == 3) {
			ez_u32Data |= ez_u8Rx;
			ez_u32Data <<= 8;
		}
		else if(ez_u8DataRdCnt == 4) {
			ez_u32Data |= ez_u8Rx;
		}
		else if(ez_u8DataRdCnt == 5) ez_u16Chksum = ez_u8Rx << 8;	// MSB
		else if(ez_u8DataRdCnt == 6) ez_u16Chksum |= ez_u8Rx;		// LSB
		else if(ez_u8DataRdCnt == 7) {
			if(ez_u8Rx == CMD_WRITE) {
				if(ez_u16Chksum == ((ez_u32Addr + ez_u32Data) & 0xFFFF) && (ez_u32Addr % 4 == 0) && !ez_u8ErrorFlag) {	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {
#endif
						*(uint32_t*)ez_u32Addr = ez_u32Data;
						easyDSP_AddRing(CMD_FB_WRITE_OK);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_WriteRemoteCore(ez_u8CoreNow, ez_u32Addr, 4);
#endif
				}
				else {
					ez_u8ErrorFlag = 0;
					easyDSP_AddRing(CMD_FB_WRITE_NG);
					easyDSP_Tx_Start();
				}
			}
			ez_u8State = STAT_INIT;
		}
	}
	else if(ez_u8State == STAT_DATA8B) {
		ez_u8DataRdCnt++;
		if(ez_u8DataRdCnt == 1) {
			ez_u64Data = ez_u8Rx; 		// MSB
			ez_u64Data <<= 8;
		}
		else if(ez_u8DataRdCnt >= 2 && ez_u8DataRdCnt <= 7) {
			ez_u64Data |= ez_u8Rx;
			ez_u64Data <<= 8;
		}
		else if(ez_u8DataRdCnt == 8) {
			ez_u64Data |= ez_u8Rx;
		}
		else if(ez_u8DataRdCnt == 9) 	ez_u16Chksum = ez_u8Rx << 8;	// MSB
		else if(ez_u8DataRdCnt == 10)	ez_u16Chksum |= ez_u8Rx;		// LSB
		else if(ez_u8DataRdCnt == 11) {
			if(ez_u8Rx == CMD_WRITE) {
				if(ez_u16Chksum == ((ez_u32Addr + ez_u64Data) & 0xFFFF) && (ez_u32Addr % 8 == 0) && !ez_u8ErrorFlag) {	// v10.1
#if EZ_USE_SEV_INT
					if(ez_u8Core == ez_u8CoreNow) {
#endif
						*(uint64_t*)ez_u32Addr = ez_u64Data;
						easyDSP_AddRing(CMD_FB_WRITE_OK);
						easyDSP_Tx_Start();
#if EZ_USE_SEV_INT
					}
					else easyDSP_WriteRemoteCore(ez_u8CoreNow, ez_u32Addr, 8);
#endif
				}
				else {
					ez_u8ErrorFlag = 0;
					easyDSP_AddRing(CMD_FB_WRITE_NG);
					easyDSP_Tx_Start();
				}
			}
			ez_u8State = STAT_INIT;
		}
	}
	else ez_u8State = STAT_INIT;
}
#endif
////////////////////////////////////////////////////////////////////////////////////
