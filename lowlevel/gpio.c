#include "gpio.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define GPIO_LED_PORT GPIOF
#define GPIO_LED_PIN  GPIO0

#define GPIO_POW_PORT GPIOF
#define GPIO_POW_PIN  GPIO1

// CAN
#define GPIO_CAN_PORT   GPIOA
#define GPIO_CAN_RX_PIN GPIO11
#define GPIO_CAN_TX_PIN GPIO12
#define GPIO_CAN_AF     GPIO_AF9

void gpio_setup() {
  rcc_periph_clock_enable(RCC_GPIOF);
  rcc_periph_clock_enable(RCC_GPIOA);

  // status led
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);

  // power input
  gpio_mode_setup(GPIO_POW_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_POW_PIN);

  // CAN RX
  gpio_mode_setup(GPIO_CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_CAN_RX_PIN);
  gpio_set_af(GPIO_CAN_PORT, GPIO_CAN_AF, GPIO_CAN_RX_PIN);

  // CAN TX
  gpio_mode_setup(GPIO_CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_CAN_TX_PIN);
  gpio_set_output_options(GPIO_CAN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_CAN_TX_PIN);
  gpio_set_af(GPIO_CAN_PORT, GPIO_CAN_AF, GPIO_CAN_TX_PIN);
}

void led_toggle_status() {
  gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);
}

void led_set_status(uint32_t status) {
  if (status == 0)
    gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
  else
    gpio_set  (GPIO_LED_PORT, GPIO_LED_PIN);
}

bool about_da_power() {
  return (gpio_get(GPIO_POW_PORT, GPIO_POW_PIN) == 0);
}

