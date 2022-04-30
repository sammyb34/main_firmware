/*
 * Uart_manager.c
 *
 * Author: Grayson Lewis
 * Date: 2/28/2021
 *
 */

#include "Uart_manager.h"


// Initialize Ring Buffer
void UART_Init(void) {
	Ringbuf_init();
}

// flush a char buffer and fill with null character
void flush_buf(char* buf, unsigned int length) {

	for(int i = 0; i < length; i++) {
		buf[0] = '\0';
	}

}

// generic send string
void tx_string(UART_HandleTypeDef *uart, char* tx_buf) {

	Uart_sendstring(tx_buf, uart);

}

// generic receive string
void rx_string(UART_HandleTypeDef *uart, char* rx_buf, enum status* s) {

	*s = no_error;

	int i = 0;

	while(IsDataAvailable(uart)) {

			if(i >= 512) { // make sure we don't overflow
				break;
				*s = error;
			}

			while(IsDataAvailable(uart)) {

				rx_buf[i] = Uart_read(uart);
				i++;

				if(i >= 512) { // make sure we don't overflow
					*s = error;
					break;
				}
			}

			HAL_Delay(10); // Delay 1ms and see if more data has shown up, if so keep adding to the pile
		}

}

// check the status of the uart modules
int check_status(UART_HandleTypeDef *uart1, UART_HandleTypeDef *uart2) {

	int da1 = IsDataAvailable(uart1);
	int da2 = IsDataAvailable(uart2);

	if(da1 && da2) {
		return data_available_both;
	}
	else if (da1) {
		return data_available_1;
	}
	else if (da2) {
		return data_available_2;
	}
	else {
		return data_available_none;
	}

	// Add check for error status;
}

// transmit accelerometer data in a specified format
void tx_accelerometer_data(UART_HandleTypeDef *uart, float* data, enum format f) {

	char tx_string[500] = {'\0'};

	switch (f) {
		case ACC_xyz:

			sprintf(tx_string, "ACC_xyz: %f, %f, %f\r\n", data[0], data[1], data[2]);

			break;
		case ACC_quat:

			sprintf(tx_string, "ACC_quat: %f, %f, %f, %f\r\n", data[0], data[1], data[2], data[3]);

			break;
		case ACC_mag:

			sprintf(tx_string, "ACC_mag: %f, %f, %f\r\n", data[0], data[1], data[2]);

			break;
		case ACC_all:

			sprintf(tx_string, "ACC_all: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[11]);

			break;
		case timestamp:

			sprintf(tx_string, "timestamp: %f\r\n", data[0]);

			break;

		default:
			break;

	}

	int i;

	for(i = 0; i < 500; i++) {
		Uart_write((int)tx_string[i], uart);

		if(tx_string[i] == '\0') {
			break;
		}
	}

	Uart_write((int) '\0', uart);

	return;

}

// receive accelerometer data in a specified format
void rx_accelerometer_data(UART_HandleTypeDef *uart, float* data, enum format* f, enum status* s) {

	*s = no_error; // this will get set to an error state if it finds one.

	char rx_data[512] = {'\0'};

	int i = 0;

	while(IsDataAvailable(uart)) {

		if(i >= 512) { // make sure we don't overflow
			break;
			*s = error;
		}

		while(IsDataAvailable(uart)) {

			rx_data[i] = Uart_read(uart);
			i++;

			if(i >= 512) { // make sure we don't overflow
				*s = error;
				break;
			}
		}

		HAL_Delay(10); // Delay 1ms and see if more data has shown up, if so keep adding to the pile
	}

	//Uart_sendstring(rx_data, uart); // used for debug
	//Uart_sendstring("\r\n", uart);

	char type1[] = "ACC_xyz:";
	char type2[] = "ACC_quat:";
	char type3[] = "ACC_mag:";
	char type4[] = "ACC_all:";
	char type5[] = "timestamp:";

	int parsed = 0;

	if(strstr(rx_data, type1) != NULL) {
		*f = ACC_xyz;
		parsed = 1;
	}
	else if(strstr(rx_data, type2) != NULL) {
		*f = ACC_quat;
		parsed = 1;
	}
	else if(strstr(rx_data, type3) != NULL) {
		*f = ACC_mag;
		parsed = 1;
	}
	else if(strstr(rx_data, type4) != NULL) {
		*f = ACC_all;
		parsed = 1;
	}
	else if(strstr(rx_data, type5) != NULL) {
		*f = timestamp;
		parsed = 1;
	}

	if(!parsed) { // if we have encountered an error, simply return
		*s = error;
		return;
	}

	char* start_point = strstr(rx_data, ":") + 1; // the location that data should start from

	if(!valid_float_characters(start_point)) {
		*s = error;
		return;
	}

	//Uart_sendstring(start_point, uart);

	for(int i = 0; i < 11; i++) {
		data[i] = atof(start_point);

		if(strstr(start_point, ",") != NULL) {
			start_point = strstr(start_point, ",") + 1;
		}
		else {
			break;
		}
	}

}

// check an aray and return true if it only contains characters from floating point nums and commas
int valid_float_characters(const char* data) {
	char characters[] = {' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '-', 'e', 'E', ',', '.', '\r', '\n', '\0'};

	int valid = 1;

	for(int i = 0; i < strlen(data); i++) {
		for(int j = 0; j < strlen(characters) + 1; j++) {
			if(data[i] == characters[j]) {
				break;
			}
			if(j == strlen(characters)) {
				valid = 0;
				break;
			}
		}

		if (valid == 0) {
			break;
		}
	}

	return valid;
}
