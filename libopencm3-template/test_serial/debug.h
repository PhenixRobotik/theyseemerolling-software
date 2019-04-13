#ifndef __DEBUG_H
#define __DEBUG_H

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/***************************************
simple uart configuration for debugging
***************************************/

//DEFINES FOR DEBUG_UART
#define DEBUG_RCC_USART RCC_USART2
#define DEBUG_USART USART2
#define DEBUG_UART_SPEED 9600

#define DEBUG_PORT_TX GPIOA
#define DEBUG_PORT_TX_RCC RCC_GPIOA//clock of GPIO port
#define DEBUG_PIN_TX GPIO2
#define DEBUG_AF_TX GPIO_AF7

#define DEBUG_PORT_RX GPIOA
#define DEBUG_PORT_RX_RCC RCC_GPIOA
#define DEBUG_PIN_RX GPIO3
#define DEBUG_AF_RX GPIO_AF7

void debug_setup(void);
void echo(char *chain);
void echo_int(int integer);

//rec=usart_recv_blocking(DEBUG_USART);//to receive a byte

#endif
