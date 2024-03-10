/*
 * stm32f4xx_uart_driver.c
 *
 *  Created on: Mar 8, 2024
 *      Author: jaina
 */
#include "stm32f407xx_uart_driver.h"
//Function definition to enable clocks to the chosen UART Peripheral
void UARTPeripheralClockControl(uint32_t *pUARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUARTx == UART1)
		{
			UART1_PCLK_EN();
		}else if(pUARTx == UART2)
		{
			UART2_PCLK_EN();
		}else if(pUARTx == UART3)
		{
			UART3_PCLK_EN();
		}else if(pUARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if(pUARTx == UART5)
		{
			UART5_PCLK_EN();
		}else if(pUARTx == UART6)
		{
			UART6_PCLK_EN();
		}
	}
	if(EnOrDi == DISABLE)
		{
			if(pUARTx == UART1)
			{
				UART1_PCLK_DI();
			}else if(pUARTx == UART2)
			{
				UART2_PCLK_DI();
			}else if(pUARTx == UART3)
			{
				UART3_PCLK_DI();
			}else if(pUARTx == UART4)
			{
				UART4_PCLK_DI();
			}else if(pUARTx == UART5)
			{
				UART5_PCLK_DI();
			}else if(pUARTx == UART6)
			{
				UART6_PCLK_DI();
			}
		}
}

