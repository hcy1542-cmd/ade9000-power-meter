/*
 * history.c
 *
 *  Created on: Nov 4, 2025
 *      Author: user
 */

unsigned int Firmware_year = 25;
unsigned int Firmware_mon = 11;
unsigned int Firmware_date = 4;
unsigned int Firmware_Version = 100;

/*
 << 2025. 11. 06  수정 : 허철영
1. Tick timer 적용 -> ADE9000 Function 100ms마다 동작
	: stm32fxx_it.c --> void SysTick_Handler(void) 내부에서 ADE9000_Measurement()호출
	flag_100ms 사용

2. uart2.comm.c Modbus Buffer 변수 매핑 및 주석 처리

3. Tick timer 적용하여 Energy Data EEPROM Write 5분마다 Write
	flag_5min 사용

4. 기존 Timer_CNT를 사용하여 1초마다 전력량 계산하던 방식
	flag_1s으로 변경

5. 리셋 및 전원 ON/OFF시 EEPROM을 통한 전력량 복원 함수 작성
	Energy 계산 수식 변경, 상별 누적 전력 계산 삭제

6. Modbus Data에 I_rated 추가 (TDD 계산용인데 빠져 있었음)

7. 상별 전력 누적량 삭제 진행돼야 함 - 완료
	(EEPROM에 상별 전력 누적량은 저장 하지 않음으로 갱신이 안됨)

8. EEPROM 초기화 코드 작성 - 진행중
	(현재 전체 다 0으로 밀어버림)
 *
 << 2025. 11. 11  수정 : 허철영
 1. Active Power Abs 처리

 << 2025. 11. 17  수정 : 허철영
 1. IAVG 수식 오타 수정

 << 2025. 11. 28  수정 : 허철영
 1. EEPROM_B Clear에 Watch Dog Kick 추가
 2. 누적 전력량 변수 structure로 변경
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
