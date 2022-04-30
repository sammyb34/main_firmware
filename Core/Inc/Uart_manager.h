/*
 * Uart_manager.h
 *
 * Author: Grayson Lewis
 * Date: 2/28/2021
 *
 */

#ifndef UART_MANAGER_H_
#define UART_MANAGER_H_

#include "stm32l4xx_hal.h"
#include "UartRingbuffer_multi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

enum format {
	ACC_xyz,
	ACC_quat,
	ACC_mag,
	ACC_all,
	timestamp
};

enum status {
	data_available_1,
	data_available_2,
	data_available_both,
	data_available_none,
	error,
	no_error
};

// Initialize Ring Buffer
void UART_Init(void);

// check the status of the uart modules
int check_status(UART_HandleTypeDef *uart1, UART_HandleTypeDef *uart2);

// flush a char buffer and fill with null character
void flush_buf(char* buf, unsigned int length);

// generic send string
void tx_string(UART_HandleTypeDef *uart, char* tx_buf);

// generic receive string
void rx_string(UART_HandleTypeDef *uart, char* rx_buf, enum status* s);

// transmit accelerometer data in a specified format
void tx_accelerometer_data(UART_HandleTypeDef *uart, float* data, enum format f);

// receive accelerometer data in a specified format
void rx_accelerometer_data(UART_HandleTypeDef *uart, float* data, enum format* f, enum status* s);

// check an aray and return true if it only contains characters from floating point nums and commas
int valid_float_characters(const char* data);

#endif
