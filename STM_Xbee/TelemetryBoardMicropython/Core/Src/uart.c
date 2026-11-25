/*
 * uart.c
 *
 *  Created on: Mar 25, 2026
 *      Author: glebt
 */

#include "uart.h"

uint8_t uart_rx[256];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart_rx, sizeof(uart_rx));
}
