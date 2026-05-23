/*
 * HR_data.h
 *
 *  Created on: 23 May 2026
 *      Author: Nikhil
 */

#ifndef INC_HR_DATA_H_
#define INC_HR_DATA_H_

/* Includes */
#include "main.h"

/* Defines */
#define MODBUS_FRAME_WAIT 3.5 /* Char (10-bit) */
#define RTU_MSG_MAXLENGTH 13
#define MODBUS_DEV_ADDRESS 0x20
	/* Die temperature */
#define COLLECT_DIE_TEMP 0x03
#define DIE_TEMP_REGADD_A 0x00
#define DIE_TEMP_REGADD_B 0x0A
#define DIE_TEMP_REGNUM_A 0x00
#define DIE_TEMP_REGNUM_B 0x01
	/* HR & SPO2 */
#define COLLECT_HR_SPO2 0x03
#define HR_SPO2_REGADD_A 0x00
#define HR_SPO2_REGADD_B 0x06
#define HR_SPO2_REGNUM_A 0x00
#define HR_SPO2_REGNUM_B 0x04
#define SAMPLE_HR_COUNT 10
	/* Collect data */
#define COLLECT_DATA 0x06
#define COLLECT_DATA_REGADD_A 0x00
#define COLLECT_DATA_REGADD_B 0x10
#define COLLECT_REG_NUM_START_A 0x00 /* "Valid Bytes" (Receive) */
#define COLLECT_REG_NUM_START_B 0x01
#define COLLECT_REG_NUM_STOP_A 0x00 /* "Data" (Receive) */
#define COLLECT_REG_NUM_STOP_B 0x02

/* Externs */
extern UART_HandleTypeDef huart1;
extern volatile uint8_t RX_BUSY;
extern volatile uint8_t POINTER_RX;
extern volatile uint16_t HR_ERROR;
extern uint8_t MODBUS_WRITE_BUFFER[8];
extern uint8_t MODBUS_READ_BUFFER[RTU_MSG_MAXLENGTH];
extern volatile uint8_t HR_POINTER;
extern volatile uint8_t HR_POINTER_AVG;
extern uint8_t HEARTBEAT_BUFF[SAMPLE_HR_COUNT];
extern uint8_t HEARTBEAT_AVG_BUFF[SAMPLE_HR_COUNT];
extern volatile uint8_t TOGGLE_COLLECT;
extern volatile uint16_t CRC_VALIDATE;

/* Function Declarations */
void MODBUS_format_send(uint8_t *buffer, uint8_t funct_code, uint8_t reg_addA, uint8_t reg_addB, uint8_t reg_numA, uint8_t reg_numB);
void MAX30102_collect_TOGGLE();
void MAX30102_HR_SPO2();
void HR_BUFF_storage();
void HR_AVG();
uint16_t CRC_check(uint8_t *data, uint8_t len);



#endif /* INC_HR_DATA_H_ */
