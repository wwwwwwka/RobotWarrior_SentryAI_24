#ifndef UART6_H
#define UART6_H
#include "main.h"

void Actuator_uart6_init(uint32_t baudRate);
extern void Uart_Flush(void);
extern int16_t Uart_Read(void);
extern void Uart_Send(uint8_t *buf , uint8_t len);

#endif
