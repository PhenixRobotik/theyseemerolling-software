#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/eeprom.h"

int main() {
  clock_setup();

  gpio_setup();
  eeprom_setup();
  encoders_setup();

  motors_setup();

  // set_all_1();
  bool status = about_da_power();


  while(true) {
    motor_b_set(0);
    delay_ms(10);
    motor_b_set(1);
    delay_ms(1);
  }


    led_set_status(1);
    delay_ms(1000);
    led_set_status(0);
    delay_ms(100);


  while(true) {
    led_set_status(1);
    delay_ms(100);
    led_set_status(0);
    delay_ms(100);

    if (status)
      led_set_status(1);
    else
      led_set_status(0);
    delay_ms(100);
    led_set_status(0);
    delay_ms(700);
  }
  return 0;
}
