#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "debug.h"


//DEFINES FOR ON BOARD LED
#define LED_PORT GPIOA
#define LED_PIN GPIO5

static void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART2. */
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

int main(void)
{
	int i, j = 0, c = 0;
	uint16_t rec=0;

	clock_setup();
	gpio_setup();
	debug_setup();

	/* Blink the LED (PD12) on the board with every transmitted byte. */
	while (1) {
		gpio_toggle(LED_PORT, LED_PIN);	/* LED on/off */
		//rec=usart_recv_blocking(DEBUG_USART);
		//usart_send_blocking(DEBUG_USART, rec);
		//for (i = 0; i < 300000; i++)
		//	__asm__("NOP");
		//usart_send_blocking(DEBUG_USART, c + '0');
		echo("here!\n");
		echo_int(-12);
		echo("\n");
		echo_int(42);
		echo("\n");
		echo_int(0);
		echo("\n");
		echo_int(-10);
		echo("\n");
		c = (c == 9) ? 0 : c + 1;
		if ((j++ % 80) == 0) {
			usart_send_blocking(DEBUG_USART, '\r');
			usart_send_blocking(DEBUG_USART, '\n');
		}
		for (i = 0; i < 3000000; i++) {
			__asm__("NOP");
		}
	}

	return 0;
}
