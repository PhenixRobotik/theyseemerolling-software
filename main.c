#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"

int main() {
  clock_setup();

  gpio_setup();
  encoders_setup();

  // set_all_1();
  bool status = about_da_power();



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
