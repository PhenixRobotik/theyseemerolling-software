#include "encoders.h"

void encoder_setup()
{
  rcc_periph_clock_enable(RCC_TIM1);
	timer_set_period(TIM1, 1024);
	timer_slave_set_mode(TIM1, 0x3); // encoder
	timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
	timer_enable_counter(TIM1);
}

void get_pos()
{
  return timer_get_count(TIM1);
}
