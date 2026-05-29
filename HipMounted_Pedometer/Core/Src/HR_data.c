/*
 * HR_data.c
 *
 *  Created on: 23 May 2026
 *      Author: Nikhil
 */

/* Includes */
#include "HR_data.h"
#include "string.h"

/* Variable definitions */
volatile uint8_t RX_BUSY = 0;
volatile uint16_t HR_ERROR = 0;
volatile uint8_t POINTER_RX = 0;
uint8_t MODBUS_WRITE_BUFFER[8];
uint8_t MODBUS_READ_BUFFER[RTU_MSG_MAXLENGTH];
volatile uint8_t HR_POINTER;
volatile uint8_t HR_POINTER_AVG;
uint8_t HEARTBEAT_BUFF[SAMPLE_HR_COUNT];
uint8_t HEARTBEAT_AVG_BUFF[SAMPLE_HR_COUNT];
volatile uint8_t TOGGLE_COLLECT = 0; /* 0 "OFF" && 1 "ON" */
volatile uint16_t CRC_VALIDATE;

/* Implementations */
void MAX30102_collect_TOGGLE() {
	/* Prepare data reception (RX) */
	HAL_UART_Receive_IT(&huart1, MODBUS_READ_BUFFER, 8);
	/* Toggle */
	if (TOGGLE_COLLECT == 0) {
		/* Format START message into buffer */
		MODBUS_format_send(MODBUS_WRITE_BUFFER, COLLECT_DATA, COLLECT_DATA_REGADD_A,
				COLLECT_DATA_REGADD_B, COLLECT_REG_NUM_START_A, COLLECT_REG_NUM_START_B);
		/* Send "START collecting data" (TX) */
		if (HAL_UART_Transmit(&huart1, MODBUS_WRITE_BUFFER, 8, 100) != HAL_OK) {
			/* Error handling */
			return;
			}
		TOGGLE_COLLECT = 1;
	}
	else {
		/* Format STOP message into buffer */
		MODBUS_format_send(MODBUS_WRITE_BUFFER, COLLECT_DATA, COLLECT_DATA_REGADD_A,
				COLLECT_DATA_REGADD_B, COLLECT_REG_NUM_STOP_A, COLLECT_REG_NUM_STOP_B);
		/* Send "STOP collecting data" (TX) */
		if (HAL_UART_Transmit(&huart1, MODBUS_WRITE_BUFFER, 8, 100) != HAL_OK) {
			/* Error handling */
			return;
			}
		TOGGLE_COLLECT = 0;
	}
}

void MAX30102_HR_SPO2() {

	/* Format START message into buffer */
	MODBUS_format_send(MODBUS_WRITE_BUFFER, COLLECT_HR_SPO2, HR_SPO2_REGADD_A,
			HR_SPO2_REGADD_B, HR_SPO2_REGNUM_A, HR_SPO2_REGNUM_B);
	/* Prepare data reception (RX) */
	HAL_UART_Receive_IT(&huart1, MODBUS_READ_BUFFER, 13);
	RX_BUSY = 1;
	/* Send "Collect Data" (TX) */
	HAL_UART_Transmit(&huart1, MODBUS_WRITE_BUFFER, 8, 100);
}

void HR_BUFF_storage() {
	/* Verify & store */
	CRC_VALIDATE = CRC_check(MODBUS_READ_BUFFER, 13);
	/* HR buffer */
	if (CRC_VALIDATE == 0) {
		HEARTBEAT_BUFF[HR_POINTER] = MODBUS_READ_BUFFER[8];
		HR_POINTER++;
	}
	/* CRC error handling */
	else {
		HR_ERROR++;
	}
	/* Buffer overflow management */
	memset(MODBUS_READ_BUFFER, 0, 13);
	/* Retrieve average value */
}

void HR_AVG() {

	/* Move pointer (storage) and clear buffer */
	if (HR_POINTER == SAMPLE_HR_COUNT-1) {
		/* Shift last element to 0th position */
		HEARTBEAT_BUFF[0] = HEARTBEAT_BUFF[HR_POINTER];
		/* Clear buffer */
		memset(&HEARTBEAT_BUFF[1], 0, SAMPLE_HR_COUNT-2);
		/* Reset pointer */
		HR_POINTER = 0;

		/* Sum variable */
		float sum_HR = 0;
		for (size_t i = 0; i < sizeof(HEARTBEAT_BUFF); i++) {
			/* Check for null values */
			if (HEARTBEAT_BUFF[i] != 0) {
				/* Sum non-zero elements */
				sum_HR += HEARTBEAT_BUFF[i];
			}
		}
		/* Store */
		HEARTBEAT_AVG_BUFF[HR_POINTER_AVG] = (uint8_t)(sum_HR / sizeof(HEARTBEAT_BUFF));
		HR_POINTER_AVG++;
		/* Reset if pointer > sizeof(buffer) */
		if (HR_POINTER_AVG == SAMPLE_HR_COUNT-1) {
			/* Shift last element to 0th position */
			HEARTBEAT_AVG_BUFF[0] = HEARTBEAT_AVG_BUFF[HR_POINTER_AVG];
			/* Clear buffer */
			memset(&HEARTBEAT_BUFF[1], 0, SAMPLE_HR_COUNT-2);
			/* Reset pointer */
			HR_POINTER_AVG = 0;
			}
		}
}

void MODBUS_format_send(uint8_t *buffer, uint8_t funct_code, uint8_t reg_addA,
								uint8_t reg_addB, uint8_t reg_numA, uint8_t reg_numB) {
	/* Memory helper functions */
	uint8_t pointer_MSG = 0;
	uint8_t MACRO_memory_8 = MODBUS_DEV_ADDRESS;
	/* Device Address (8-bit) */
	memcpy(buffer, &MACRO_memory_8, 1);
	pointer_MSG++;
	/* Function Code (8-bit) */
	MACRO_memory_8 = funct_code;
	memcpy(&buffer[pointer_MSG], &MACRO_memory_8, 1);
	pointer_MSG++;

	/* Register Address or Valid Bytes (16-bit) */
		/* Address (Upper) */
	MACRO_memory_8 = reg_addA;
	memcpy(&buffer[pointer_MSG], &MACRO_memory_8, 1);
	pointer_MSG++;
		/* Address (Lower) */
	MACRO_memory_8 = reg_addB;
	memcpy(&buffer[pointer_MSG], &MACRO_memory_8, 1);
	pointer_MSG++;

	/* Register Number or Data (16-bit/N*8-bit) */
		/* Number (Upper) */
	MACRO_memory_8 = reg_numA;
	memcpy(&buffer[pointer_MSG], &MACRO_memory_8, 1);
	pointer_MSG++;
		/* Number (Lower) */
	MACRO_memory_8 = reg_numB;
	memcpy(&buffer[pointer_MSG], &MACRO_memory_8, 1);
	pointer_MSG++;

	/* Calculate CRC */
	uint16_t crc_val = CRC_check(buffer, pointer_MSG); /* Verify pointer */
	/* Append CRC (upper) */
	uint8_t crc_val_A = ((uint16_t)crc_val & (uint16_t)0xFF00) >> 8;
	memcpy(&buffer[pointer_MSG], &crc_val_A, 1);
	pointer_MSG++;
	/* Append CRC (lower) */
	uint8_t crc_val_B = (uint16_t)crc_val & ((uint16_t)0x00FF);
	memcpy(&buffer[pointer_MSG], &crc_val_B, 1);
	pointer_MSG = 0;
}

uint16_t CRC_check(uint8_t *data, uint8_t len) {

	uint16_t crc = 0xFFFF;
	  for( uint8_t pos = 0; pos < len; pos++)
	  {
	    crc ^= (uint16_t)data[ pos ];
	    for(uint8_t i = 8; i != 0; i--)
	    {
	      if((crc & 0x0001) != 0){
	        crc >>= 1;
	        crc ^= 0xA001;
	      }else{
	        crc >>= 1;
	      }
	    }
	  }
	  crc = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8);
	  return crc;

}
