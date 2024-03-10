/*
 * stm32f4xx_uart_driver.h
 *
 *  Created on: Mar 8, 2024
 *      Author: jaina
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_
#include <stm32f4xx.h>
typedef struct
{
	uint8_t UART_Mode;
	uint32_t UART_Baud;
	uint8_t UART_NoOfStopBits;
	uint8_t UART_WordLength;
	uint8_t UART_ParityControl;
	uint8_t UART_HWFlowControl;
}UART_Config_t;

typedef struct
{
	UART_Regdef_t *pUARTx;
	UART_Config_t UART_Config;
}UART_Handle_t;




#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
