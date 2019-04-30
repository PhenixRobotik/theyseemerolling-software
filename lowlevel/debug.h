#ifndef __DEBUG_H
#define __DEBUG_H

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/***************************************
simple uart configuration for debugging
***************************************/

//DEFINES FOR DEBUG_UART
#define DEBUG_RCC_USART RCC_USART1
#define DEBUG_USART USART1
#define DEBUG_UART_SPEED 19200 // Bug in libopencm3 ? Really 9600.

#define DEBUG_PORT_TX GPIOB
#define DEBUG_PORT_TX_RCC RCC_GPIOB // clock of GPIO port
#define DEBUG_PIN_TX GPIO6
#define DEBUG_AF_TX GPIO_AF7

#define DEBUG_PORT_RX GPIOB
#define DEBUG_PORT_RX_RCC RCC_GPIOB
#define DEBUG_PIN_RX GPIO7
#define DEBUG_AF_RX GPIO_AF7

void debug_setup(void);
void echo(char *chain);
void echo_int(int integer);

//rec=usart_recv_blocking(DEBUG_USART);//to receive a byte

#endif
