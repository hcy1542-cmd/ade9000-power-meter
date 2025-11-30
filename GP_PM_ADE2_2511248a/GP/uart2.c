
#include "def.h"
#include "uart.h"

extern UART_HandleTypeDef huart2;

uint8_t uart2_Function_comm = 0;
uint8_t HMI_ID = 0x01;
uint8_t	tx_CRC_L, tx_CRC_H = 0;
uint8_t RS232comm_Warning_Status = 0;

unsigned char uart2_rxd_buf[UART_BUFFER_MAX];
unsigned char uart2_txd_buf[UART_BUFFER_MAX];

unsigned int comm_temp[10] = {0};
unsigned int uart2_out_frame[256] = {0};
unsigned int uart2_in_frame[256] = {0};
unsigned int CRC_byte = 0;
unsigned int uart2_Word_count = 0;
unsigned int uart2_Start_addr = 0;
unsigned int event_log_buf[48] = {0};
unsigned int uart2_send_dt_num = 0;

struct uart_data uart2;

void init_UART2(void)
{
	if(uart2.init_port == 1)
	{
	    // USART DMA Request Line OFF
	    CLEAR_BIT(USART2->CR3, USART_CR3_DMAT | USART_CR3_DMAR);

	    // RX/TX DMA OFF
	    if (huart2.hdmarx) HAL_DMA_Abort(huart2.hdmarx);
	    if (huart2.hdmatx) HAL_DMA_Abort(huart2.hdmatx);

	    // UART 인터럽트 DISABLE
	    __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR | UART_IT_IDLE | UART_IT_RXNE);
	    __HAL_UART_CLEAR_PEFLAG(&huart2);
	    __HAL_UART_CLEAR_FEFLAG(&huart2);
	    __HAL_UART_CLEAR_NEFLAG(&huart2);
	    __HAL_UART_CLEAR_OREFLAG(&huart2);

	    HAL_UART_DeInit(&huart2);
	    __HAL_RCC_USART2_FORCE_RESET();
	    __HAL_RCC_USART2_RELEASE_RESET();

	    MX_USART2_UART_Init();   // CubeMX 생성 함수

	    uartInit2();

	    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

		uart2.init_port = 0;

		HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin, GPIO_PIN_RESET);	//RS485_TX
	}
}

void txd_buf_write(int ch, unsigned char dt)
{
 	if(ch == 2)
 	{
 		uart2_txd_buf[uart2.txd.end++] = dt;
    	if(uart2.txd.end >= UART_BUFFER_MAX) uart2.txd.end = 0;
    	uart2.txd.chk_sum += dt;
  	}
}

uint32_t GetDoubleHigh32(double value)
{
    union {
        double d;
        uint64_t u64;
    } conv;

    conv.d = value;
    return (uint32_t)(conv.u64 >> 32);
}

uint32_t GetDoubleLow32(double value)
{
    union {
        double d;
        uint64_t u64;
    } conv;

    conv.d = value;
    return (uint32_t)(conv.u64 & 0xFFFFFFFF);
}


uint16_t kwh_comm_Reset_cnt = 0;
uint16_t hcy;
void PM_Comm_data(void)
{
	comm_data.Part_ID = eep.Set.Modbus_ID.val;				// 사용자 작성 Device ID
	comm_data.Version = ADE9000.VERSION.DIGIT.u16;			// SPI 정상 동작시 0x00C0, 그외 SPI 통신 오류
	comm_data.RUN = ADE_Read16(ADE9000.RUN.addr);			// ADE9000 동작 상태 / 0 : 정지, 1: 동작
	comm_data.ACCMODE = ADE_Read16(ADE9000.ACCMODE.addr);	// Frequency, Delta, 2CT 설정
	comm_data.EP_CFG = ADE_Read16(ADE9000.EP_CFG.addr);		// THD, TDD 측정
	comm_data.CONFIG3 = ADE_Read16(ADE9000.CONFIG3.addr);	// Voltage Peak 측정
	comm_data.SeqErr = Seq_Check;							// 3상 전압 상 순서 상태, Seq_Check = 0 : 정상 연결 상태
	comm_data.PM_Flag = PM_Flag;							// ADE9000 Sequence
	comm_data.CT_Primary = eep.CT.Primary.val;				// CT 1차측 정격
	comm_data.CT_Secondary = eep.CT.Secondary.val;			// CT 2차측 정격
	comm_data.Init_EEP = Init_EEP;							// EEPROM Gain 사용 여부 (0: 사용, 1 : 미사용)

	comm_data.AIRMS = ADE9000.AIRMS1012.rd * 10;			// A상 전류
	comm_data.BIRMS = ADE9000.BIRMS1012.rd * 10;			// B상 전류
	comm_data.CIRMS = ADE9000.CIRMS1012.rd * 10;			// C상 전류
	comm_data.IRMS_AVG = IRMS_AVG;							// 전류 평균값
	comm_data.ABVRMS = ADE9000.AVRMS1012.rd * 10;			// Vab
	comm_data.BCVRMS = ADE9000.BVRMS1012.rd * 10;			// Vbc
	comm_data.CAVRMS = ADE9000.CVRMS1012.rd *10 ;			// Vca
	comm_data.LINE_VRMS_AVG = LINE_VRMS_AVG * 10;			// 선간 전압 평균치
	comm_data.VARMS = AVRMS1012_LN * 10;					// Van
	comm_data.VBRMS = BVRMS1012_LN * 10;					// Vbn
	comm_data.VCRMS = CVRMS1012_LN *10;						// Vcn
	comm_data.PHASE_VRMS_AVG = VRMS1012_LN_AVG * 10;		// 상전압 평균치

	comm_data.VPEAK = ADE9000.VPEAK.rd * 10;				// 순시 전압 피크치;

	comm_data.AWATT = ADE9000.AWATT.rd * 10;				// A상 유효 전력
	comm_data.BWATT = ADE9000.BWATT.rd * 10;				// B상 유효 전력
	comm_data.CWATT = ADE9000.CWATT.rd * 10;				// C상 유효 전력
	comm_data.WATT_TOT = ADE9000_Active_Power * 10;			// 합산 유효 전력

	comm_data.AVAR = ADE9000.AVAR.rd * 10;					// A상 무효 전력
	comm_data.BVAR = ADE9000.BVAR.rd * 10;					// B상 무효 전력
	comm_data.CVAR = ADE9000.CVAR.rd * 10;					// C상 무효 전력
	comm_data.VAR_TOT = ADE9000_Reactive_Power * 10;		// 합산 무효 전력

	comm_data.AVA = ADE9000.AVA.rd * 10;					// A상 피상 전력
	comm_data.BVA = ADE9000.BVA.rd * 10;					// B상 피상 전력
	comm_data.CVA = ADE9000.CVA.rd * 10;					// C상 피상 전력
	comm_data.CVA_TOT = ADE9000_Apparent_Power * 10;		// 합산 피상 전력

	uint64_t ADE9000_WATT_ACC_Total_64 = ADE9000.WATT_ACC.rd * 10;
	uint32_t ADE9000_WATT_ACC_Total_32 = (uint32_t)((ADE9000_WATT_ACC_Total_64) & 0xffffffff);
	comm_data.WATT_ACC_T_HI = (uint16_t)(ADE9000_WATT_ACC_Total_32 >> 16);		// 총합 유효 누적 전력 상위 비트
	comm_data.WATT_ACC_T_LO = (uint16_t)(ADE9000_WATT_ACC_Total_32 & 0xffff);	// 총합 유효 누적 전력 하위 비트

	int64_t ADE9000_VAR_ACC_Total_64 = ADE9000.VAR_ACC.rd * 10;
	int32_t ADE9000_VAR_ACC_Total_32 = (int32_t)(ADE9000_VAR_ACC_Total_64 & 0xffffffff);
	comm_data.VAR_ACC_T_HI = (int16_t)(ADE9000_VAR_ACC_Total_32 >> 16);		// 총합 무효 누적 전력 상위 비트
	comm_data.VAR_ACC_T_LO = (int16_t)(ADE9000_VAR_ACC_Total_32 & 0xffff);	// 총합 무효 누적 전력 하위 비트

	uint64_t ADE9000_VA_ACC_Total_64 = ADE9000.VAR_ACC.rd * 10;
	uint32_t ADE9000_VA_ACC_Total_32 = (uint32_t)(ADE9000_VA_ACC_Total_64 & 0xffffffff);
	comm_data.VA_ACC_T_HI = (uint16_t)(ADE9000_VA_ACC_Total_32 >> 16);		// 총합 피상 누적 전력 상위 비트
	comm_data.VA_ACC_T_LO = (uint16_t)(ADE9000_VA_ACC_Total_32 & 0xffff);	// 총합 피상 누적 전력 하위 비트

	comm_data.APF = ADE9000.APF.rd * 100;						// A상 역률
	comm_data.BPF = ADE9000.BPF.rd * 100;						// B상 역률
	comm_data.CPF = ADE9000.CPF.rd * 100;						// C상 역률
	comm_data.PF_AVG = ADE9000_PowerFactor * 100;				// 역률 평균치

	comm_data.AVTHD = ADE9000.AVTHD.rd * 10;					// A상 전압 THD
	comm_data.BVTHD = ADE9000.BVTHD.rd * 10;					// B상 전압 THD
	comm_data.CVTHD = ADE9000.CVTHD.rd * 10;					// C상 전압 THD
	comm_data.VTHD_AVG = VTHD_AVG * 10;							// 전압 THD 평균치

	comm_data.AITHD = ADE9000.AITHD.rd * 10;					// A상 전류 THD
	comm_data.BITHD = ADE9000.BITHD.rd * 10;					// B상 전류 THD
	comm_data.CITHD = ADE9000.CITHD.rd * 10;					// C상 전류 THD
	comm_data.ITHD_AVG = ITHD_AVG * 10;							// 전류 THD 평균치

	comm_data.AITDD = ADE9000.AITDD.rd * 10;					// A상 전류 TDD
	comm_data.BITDD = ADE9000.BITDD.rd * 10;					// B상 전류 TDD
	comm_data.CITDD = ADE9000.CITDD.rd * 10;					// C상 전류 TDD
	comm_data.ITDD_AVG = ITDD_AVG * 10;							// 전류 TDD 평균치

	comm_data.AFREQ = ADE9000.APERIOD.rd * 10;					// A상 전압 주파수
	comm_data.BFREQ = ADE9000.BPERIOD.rd * 10;					// B상 전압 주파수
	comm_data.CFREQ = ADE9000.APERIOD.rd * 10;					// C상 전압 주파수
	comm_data.FREQ_AVG = ADE9000.COM_PERIOD.rd *10;				// 전압 주파수 평균치

	comm_data.ANGL_VA_VB = ADE9000.ANGL_VA_VB.rd;				// A상 B상 전압 위상차
	comm_data.ANGL_VB_VC = ADE9000.ANGL_VB_VC.rd;				// B상 C상 전압 위상차
	comm_data.ANGL_VA_VC = ADE9000.ANGL_VA_VC.rd;				// A상 C상 전압 위상차

	comm_data.ANGL_VA_IA = ADE9000.ANGL_VA_IA.rd;				// A상 전압 A상 전류 위상차
	comm_data.ANGL_VB_IB = ADE9000.ANGL_VB_IB.rd;				// B상 전압 B상 전류 위상차
	comm_data.ANGL_VC_IC = ADE9000.ANGL_VC_IC.rd;				// C상 전압 C상 전류 위상차

	comm_data.ANGL_IA_IB = ADE9000.ANGL_IA_IB.rd;				// A상 전류 B상 전류 위상차
	comm_data.ANGL_IB_IC = ADE9000.ANGL_IB_IC.rd;				// B상 전류 C상 전류 위상차
	comm_data.ANGL_IA_IC = ADE9000.ANGL_IA_IC.rd;				// A상 전류 C상 전류 위상차

	// */
	comm_data.I_rated = eep.Set.I_rated.val;

	comm_data.Firmware_year = Firmware_Version;
	comm_data.Firmware_mon = Firmware_mon;
	comm_data.Firmware_date = Firmware_date;
	comm_data.Firmware_Version = Firmware_year;
}


void PM_Comm_data_Reverse(void)
{
    eep.Set.Modbus_ID.val = comm_data.Part_ID;

    // comm_data.Version   (읽기전용)
    // comm_data.RUN
    // comm_data.ACCMODE
    // comm_data.EP_CFG
    // comm_data.CONFIG3


    // ADE9000 Sequence 플래그
    PM_Flag = comm_data.PM_Flag;

    // CT 정격값
    eep.CT.Primary.val   = comm_data.CT_Primary;
    eep.CT.Secondary.val = comm_data.CT_Secondary;

    // EEPROM Gain 사용 여부
    Init_EEP = comm_data.Init_EEP;

    // 정격 전류
    eep.Set.I_rated.val = comm_data.I_rated;

    // Firmware 정보 (읽기전용 → 역반영 X)
    // comm_data.Firmware_year
    // comm_data.Firmware_mon
    // comm_data.Firmware_date
    // comm_data.Firmware_Version
}


uint16_t tx_len = 0;
uint8_t tx_buf[256];

void PM_modbus_protocol(void)
{
	//=========== Writeting data to address buf ===============
	if(uart2_Function_comm == 0x06)
	{

		*Address_buf[uart2_Start_addr] = uart2_Word_count;

	}
	else if(uart2_Function_comm == 0x10)
	{
		for(CRC_byte = 0; CRC_byte < uart2_Word_count; CRC_byte++)
		{
			*Address_buf[uart2_Start_addr+CRC_byte] = ((uart2_in_frame[CRC_byte*2+7]<<8) & 0xff00) | ((uart2_in_frame[CRC_byte*2+8]) & 0xff);
		}

	}

	//===== Data Mapping =======
	PM_Comm_data();

	/* Read Holding Registers */
	if(uart2_Function_comm == 0x03)
	{
		uart2_crc_data = 0xffff;
		for(CRC_byte=0; CRC_byte<(uart2_Word_count*2+3); CRC_byte++)
		{
			if(CRC_byte==0)         uart2_out_frame[0] = HMI_ID;
			else if(CRC_byte==1)    uart2_out_frame[1] = uart2_Function_comm;
			else if(CRC_byte==2)    uart2_out_frame[2] = uart2_Word_count * 2;
			else
			{
				if(CRC_byte & 0x01) uart2_out_frame[CRC_byte]= (*Address_buf[uart2_Start_addr+((CRC_byte-3)>>1)]>>8) & 0xff;
				else                uart2_out_frame[CRC_byte]= *Address_buf[uart2_Start_addr+((CRC_byte-3)>>1)] & 0xff;
			}

			uart2_CheckCrc(uart2_out_frame[CRC_byte]);
		}

		tx_CRC_L = uart2_crc_data & 0xff;
		tx_CRC_H = (uart2_crc_data>>8) & 0xff;

		for(CRC_byte=0; CRC_byte<(uart2_Word_count*2+3); CRC_byte++)
		{
			txd_buf_write(2, uart2_out_frame[CRC_byte]);
		}

		txd_buf_write(2, tx_CRC_L); //CRC16 _L
		txd_buf_write(2, tx_CRC_H); //CRC16 _H

		uart2_send_dt_num = uart2_Word_count*2+5;

	}

	/* Write Single Registers */
	else if(uart2_Function_comm == 0x06)
	{
		uart2_out_frame[0] = HMI_ID;
		uart2_out_frame[1] = uart2_Function_comm;
		uart2_out_frame[2] = (uart2_Start_addr>>8) & 0xff;
		uart2_out_frame[3] = uart2_Start_addr & 0xff;
		uart2_out_frame[4] = (*Address_buf[uart2_Start_addr]>>8) & 0xff;
		uart2_out_frame[5] = *Address_buf[uart2_Start_addr] & 0xff;

		uart2_crc_data=0xffff;
		for(CRC_byte=0; CRC_byte<6; CRC_byte++)
		{
			uart2_CheckCrc(uart2_out_frame[CRC_byte]);
		}

		tx_CRC_L = uart2_crc_data & 0xff;
		tx_CRC_H = (uart2_crc_data>>8) & 0xff;

		for(CRC_byte=0; CRC_byte<6; CRC_byte++)
		{
			txd_buf_write(2, uart2_out_frame[CRC_byte]);
		}

		txd_buf_write(2, tx_CRC_L); //CRC16 _L
		txd_buf_write(2, tx_CRC_H); //CRC16 _H

		uart2_send_dt_num = 8;
	}
	else if(uart2_Function_comm == 0x10)
	{
		for(CRC_byte=0; CRC_byte<uart2_Word_count;   CRC_byte++)
		{
			uart2_out_frame[CRC_byte*2+7] = (*Address_buf[uart2_Start_addr+CRC_byte]>>8) & 0xff;
			uart2_out_frame[CRC_byte*2+8] = *Address_buf[uart2_Start_addr+CRC_byte] & 0xff;
		}

		uart2_out_frame[0] = HMI_ID;
		uart2_out_frame[1] = uart2_Function_comm;
		uart2_out_frame[2] = (uart2_Start_addr>>8) & 0xff;
		uart2_out_frame[3] = uart2_Start_addr & 0xff;
		uart2_out_frame[4] = (uart2_Word_count>>8) & 0xff;
		uart2_out_frame[5] = uart2_Word_count & 0xff;

		uart2_crc_data=0xffff;
		for(CRC_byte=0; CRC_byte<6; CRC_byte++)
		{
			uart2_CheckCrc(uart2_out_frame[CRC_byte]);
		}

		tx_CRC_L = uart2_crc_data & 0xff;
		tx_CRC_H = (uart2_crc_data>>8) & 0xff;

		for(CRC_byte=0; CRC_byte<6; CRC_byte++)
		{
			txd_buf_write(2, uart2_out_frame[CRC_byte]);
		}

		txd_buf_write(2, tx_CRC_L); //CRC16 _L
		txd_buf_write(2, tx_CRC_H); //CRC16 _H

		uart2_send_dt_num = 8;
	}
}

uint16_t tx_delay_cnt = 0;

void uart2_conv_hex_dec()
{
	if(uart2.hex_dex_ena == 1)
	{
		if(++tx_delay_cnt > 2)	//20ms tx delay
		{
			tx_delay_cnt = 0;
			PM_modbus_protocol();

			uart2.hex_dex_ena = 0;
			uart2.txd.enable = 1;
		}
	}
}


void UART2_tx_polling(void)
{
	if(uart2.init_port == 0)
	{
		if(uart2.txd.enable == 1)
		{
			if(uart2.txd.pos != uart2.txd.end)	// 마지막 위치가 동일하지 않으면
			{

				HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin, GPIO_PIN_SET); //RS485_RX
				if (huart2.gState == HAL_UART_STATE_READY)
				{
					uartWrite(2, &uart2_txd_buf[uart2.txd.pos++], 1);
					if(uart2.txd.pos >= UART_BUFFER_MAX) uart2.txd.pos = 0;
				}

				uart2.txd.end_cnt = 0;
			}
			else	// 송신 버퍼 비어있음
			{
				if(uart2.txd.end_cnt++ > REF_TIME_msec(10))	//10ms : 3.5 idle time , 충분히 여유 줌 9600bps -> 1 idle time : 1.041ms
				{
					uart2.txd.end_cnt = 0;
					uart2.txd.enable = 0;

					HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin, GPIO_PIN_RESET);	//RS485_TX
				}
			}
		}
	}
}

unsigned int Error_Code[14] = {0,};

void ClrBit_Error_Code(int event)
{
	int tmp_pt = 0;
	if(event <= 64)
	{
		if(event <= 8)                     tmp_pt = 0;
		else if(event > 8 && event <= 16)  tmp_pt = 1;
		else if(event > 16 && event <= 24) tmp_pt = 2;
		else if(event > 24 && event <= 32) tmp_pt = 3;
		else if(event > 32 && event <= 40) tmp_pt = 4;
		else if(event > 40 && event <= 48) tmp_pt = 5;
		else if(event > 48 && event <= 56) tmp_pt = 6;
		else if(event > 56 && event <= 64) tmp_pt = 7;
	}
	else
	{
		if(event <= 72)                    tmp_pt = 8;
		else if(event > 72 && event <= 80) tmp_pt = 9;
		else if(event > 80 && event <= 88) tmp_pt = 10;
		else if(event > 88 && event <= 96) tmp_pt = 11;
		else if(event > 96 && event <= 104) tmp_pt = 12;
		else if(event > 104 && event <= 112) tmp_pt = 13;
		else if(event > 112 && event <= 120) tmp_pt = 14;
	}
	ClrBit(Error_Code[tmp_pt],((event - (tmp_pt * 8))- 1));
}

unsigned int uart2_comm_cnt = 0, uart2_comm_cnt_1 = 0, uart2_comm_cnt_2 = 0, uart2_interval_cnt = 0;
unsigned int uart2_comm_check_cnt_1 = 0;
unsigned int uart2_comm_check[20] = {0};
unsigned int uart2_comm_check_cnt = 0, scib_comm_check_cnt_err = 0;
unsigned int rx_CRC_check_L = 0, rx_CRC_check_H = 0;
unsigned int uart2_crc_err = 0;
unsigned long uart2_rxd_cnt = 0;
unsigned short Start_addr_H = 0, Word_count_H = 0, uart2_Byte_no = 0;
unsigned short uart2_rx_CRC_L = 0, uart2_rx_CRC_H = 0;
int event_old;
int protect_Error_Code = 0;


void UART2_rxd_data_check(void)
{
	if(uart2.rxd.pos != uart2.rxd.end)
	{
        if(uart2.rxd.pos & 0x01)	uart2_comm_cnt_1 = uart2_comm_cnt;
        else                    	uart2_comm_cnt_2 = uart2_comm_cnt;

        if(uart2_comm_cnt_1<uart2_comm_cnt_2) 	uart2_interval_cnt = uart2_comm_cnt_2 - uart2_comm_cnt_1;
        else                               		uart2_interval_cnt = uart2_comm_cnt_1 - uart2_comm_cnt_2;

		uart2.rxd.data = uart2_rxd_buf[uart2.rxd.pos++];
		if(uart2.rxd.pos >= UART_BUFFER_MAX) uart2.rxd.pos = 0;

        //Modified condition to silent interval time(3.5char)
        //1char = 10bit(1start, 8data, 1stop) = 1/baudrate(9600)*10bit = 1.04ms => 3.5char = 1.04ms * 3.5char = 3.64ms
        //1char = 10bit(1start, 8data, 1stop) = 1/baudrate(38400)*10bit = 0.26ms => 3.5char = 0.26ms * 3.5char = 0.91ms
        //scib_comm_cnt => 1/9920 = 0.1ms

        if(uart2_interval_cnt > 40)	//9600bps 기준 3.65ms
        {
            if(0 < uart2_comm_check_cnt)
            {
                scib_comm_check_cnt_err++;
                if(19 <= uart2_comm_check_cnt_1)   	uart2_comm_check_cnt_1 = 0;
                else                            	uart2_comm_check_cnt_1++;
                uart2_comm_check[uart2_comm_check_cnt_1] = uart2_comm_check_cnt_1;
            }

            uart2_comm_cnt = uart2_comm_cnt_1 = uart2_comm_cnt_2 = 0;    //reset scib_comm_cnt to avoid overflow by YJH 20160328
            HMI_ID = uart2.rxd.data;                              		//HMI ID

            uart2.rxd.SEQ = 0;
        }

        switch(uart2.rxd.SEQ)
        {
            case  1  :  uart2_Function_comm = uart2.rxd.data;
                        uart2_comm_check_cnt = 1;
                        break;
            case  2  :  Start_addr_H = (uart2.rxd.data<<8) & 0xff00;
                        break;
            case  3  :  uart2_Start_addr = Start_addr_H | (uart2.rxd.data & 0xff);
                        break;
            case  4  :  Word_count_H = (uart2.rxd.data<<8) & 0xff00;
                        break;
            case  5  :  uart2_Word_count = Word_count_H | (uart2.rxd.data & 0xff);
                        break;
            case  6  :  if(uart2_Function_comm == 0x10)  	uart2_Byte_no = uart2.rxd.data;
                        else                            	uart2_rx_CRC_L = uart2.rxd.data;
                        break;
            case  7  :  if(uart2_Function_comm == 0x10)  	uart2_in_frame[7] = uart2.rxd.data;
                        else                            	uart2_rx_CRC_H = uart2.rxd.data;
                        break;
            default  :  break;
        }

		if((uart2_Function_comm == 0x10) && (7 < uart2.rxd.SEQ))      //Preset Multiple Register(Multiple Write)
		{
			if(uart2.rxd.SEQ<=(uart2_Byte_no+6))
			{
				uart2_in_frame[uart2.rxd.SEQ] = uart2.rxd.data;
			}
			else if(uart2.rxd.SEQ == (uart2_Byte_no+7)) uart2_in_frame[uart2_Byte_no+7] = uart2_rx_CRC_L = uart2.rxd.data;
			else if(uart2.rxd.SEQ == (uart2_Byte_no+8))
			{
				//CRC16 check
				uart2_in_frame[0] = HMI_ID;
				uart2_in_frame[1] = uart2_Function_comm;
				uart2_in_frame[2] = (uart2_Start_addr>>8) & 0xff;
				uart2_in_frame[3] = uart2_Start_addr & 0xff;
				uart2_in_frame[4] = (uart2_Word_count>>8) & 0xff;
				uart2_in_frame[5] = uart2_Word_count & 0xff;
				uart2_in_frame[6] = uart2_Byte_no;
				uart2_in_frame[uart2_Byte_no+8] = uart2_rx_CRC_H = uart2.rxd.data;

				//calculate CRC16
				uart2_crc_data=0xffff;
				for(CRC_byte=0; CRC_byte<(uart2_Byte_no+7); CRC_byte++)
				{
					uart2_CheckCrc(uart2_in_frame[CRC_byte]);
				}

				rx_CRC_check_L = uart2_crc_data & 0xff;
				rx_CRC_check_H = (uart2_crc_data>>8) & 0xff;
				if((uart2_rx_CRC_L == (uart2_crc_data & 0xff)) && (uart2_rx_CRC_H == ((uart2_crc_data>>8) & 0xff))) //matching CRC16 calculation
				{
					uart2.hex_dex_ena = 1;
					uart2.init_cnt = 0;
					if(RS232comm_Warning_Status == 1)
					{
						RS232comm_Warning_Status = 0;
					}
					if(uart2_rxd_cnt > 0) uart2_rxd_cnt = 0;
				}
				else uart2_crc_err++;
			}
			uart2_comm_check_cnt=2;
		}
		else if((uart2_Function_comm != 0x10) && (uart2.rxd.SEQ == 7))    //Read Holding Register and Preset Single Register
		{
			uart2_Byte_no=0;
			//CRC16 check
			uart2_in_frame[0] = HMI_ID;
			uart2_in_frame[1] = uart2_Function_comm;
			uart2_in_frame[2] = (uart2_Start_addr>>8) & 0xff;
			uart2_in_frame[3] = uart2_Start_addr & 0xff;
			uart2_in_frame[4] = (uart2_Word_count>>8) & 0xff;
			uart2_in_frame[5] = uart2_Word_count & 0xff;
			uart2_in_frame[6] = uart2_rx_CRC_L;
			uart2_in_frame[7] = uart2_rx_CRC_H;

			//calculate CRC16
			uart2_crc_data=0xffff;
			for(CRC_byte=0; CRC_byte<6; CRC_byte++)
			{
				uart2_CheckCrc(uart2_in_frame[CRC_byte]);
			}

			rx_CRC_check_L = uart2_crc_data & 0xff;
			rx_CRC_check_H = (uart2_crc_data>>8) & 0xff;

			if((uart2_rx_CRC_L == (uart2_crc_data & 0xff)) && (uart2_rx_CRC_H == ((uart2_crc_data>>8) & 0xff))) //matching CRC16 calculation
			{
				uart2.hex_dex_ena = 1;
				uart2.init_cnt = 0;
				if(RS232comm_Warning_Status == 1)
				{
					RS232comm_Warning_Status = 0;
				}
				if(uart2_rxd_cnt > 0) uart2_rxd_cnt = 0;
			}
			else uart2_crc_err++;
			uart2_comm_check_cnt = 3;
		}
		uart2.rxd.SEQ++;
		uart2_comm_check_cnt=4;
	}
	else
	{
		uart2_rxd_cnt++;

		if(uart2.init_cnt++ > REF_TIME_sec(10))//uart2.init_cnt_ref) //5s
		{
			uart2.init_cnt = 0;
			uart2.init_port = 1;
		}

		if(RS232comm_Warning_Status == 0)
		{
			{
				if(uart2_rxd_cnt > REF_TIME_sec(30) )
				{
					uart2_rxd_cnt = 0;
					RS232comm_Warning_Status = 1;
				}
			}
		}
	}
}

void UART2_rx_polling(void)
{
	if(huart2.ErrorCode != HAL_UART_ERROR_NONE)
	{
		uart2.init_port = 1;
	}
	if(uart2.init_port == 0)
	{
		if(uartAvailable(2) > 0)
		{
			uart2_rxd_buf[uart2.rxd.end] = uartRead(2);
			if(++uart2.rxd.end >= UART_BUFFER_MAX) uart2.rxd.end = 0;
			uart2.init_cnt = 0;
		}

		UART2_rxd_data_check();
	}

	uart2_comm_cnt++;	//프레임 간 간격 체크
}


//============================= E N D ========================================
