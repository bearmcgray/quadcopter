#ifndef __UART_LOG_H
#define __UART_LOG_H

#include "stm32f4xx_usart.h"

void usart_init(USART_TypeDef* USARTx);
void wr_str(USART_TypeDef* USARTx, void* data);
void wr_buf(USART_TypeDef* USARTx, void* data, int size);
void wr_int(USART_TypeDef* USARTx, int data);
void wr_long_group(USART_TypeDef* USARTx, int data0, int data1, int data2, int data3, int data4, int data5);
void wr_long_group9(USART_TypeDef* USARTx, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8);

#endif //__UART_LOG_H
