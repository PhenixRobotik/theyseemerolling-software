//gpio_toggle(GPIOA, GPIO5);
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* Set STM32 to 64 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}

static void gpio_setup(void)
{
	/* Enable GPIOD clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set GPIO12 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,  GPIO5);
}

static void button_setup(void)
{
	/* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);
}

int main(void)
{
	int i;

	clock_setup();
	button_setup();
	gpio_setup();

	/* Blink the LED (PD12) on the board. */
	while (1) {

		/* Upon button press, blink more slowly. */
		if (gpio_get(GPIOC, GPIO13)) {
			for (i = 0; i < 1000000; i++)		/* Wait a bit. */
				__asm__("nop");

		}
		gpio_toggle(GPIOA, GPIO5);
		for (i = 0; i < 1000000; i++)		/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
