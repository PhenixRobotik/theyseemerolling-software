#pragma once

#include <stdbool.h>
#include <stdint.h>

void gpio_setup();

void led_set_status(uint32_t status); // PWM ?
void led_toggle_status();

void set_all_1();
void set_all_0();
