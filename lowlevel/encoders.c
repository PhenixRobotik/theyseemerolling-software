#include "encoders.h"

void encoder_left_setup(void)
{
  rcc_periph_clock_enable(ENCODER_L_RCC_TIM);
  timer_set_period(ENCODER_L_TIM, ENCODER_PERIOD);
  timer_slave_set_mode(ENCODER_L_TIM, 0x3); // encoder
  timer_ic_set_input(ENCODER_L_TIM, TIM_IC1, TIM_IC_IN_TI1);
  timer_ic_set_input(ENCODER_L_TIM, TIM_IC2, TIM_IC_IN_TI2);
  timer_direction_down(ENCODER_L_TIM);
  timer_enable_counter(ENCODER_L_TIM);

	rcc_periph_clock_enable(ENCODER_L_CH1_PORT_RCC);
	gpio_mode_setup(ENCODER_L_CH1_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_L_CH1_PIN);
	gpio_set_af(ENCODER_L_CH1_PORT, ENCODER_L_CH1_AF, ENCODER_L_CH1_PIN);

  rcc_periph_clock_enable(ENCODER_L_CH2_PORT_RCC);
	gpio_mode_setup(ENCODER_L_CH2_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_L_CH2_PIN);
	gpio_set_af(ENCODER_L_CH2_PORT, ENCODER_L_CH2_AF, ENCODER_L_CH2_PIN);
}

void encoder_right_setup(void)
{
  rcc_periph_clock_enable(ENCODER_R_RCC_TIM);
  timer_set_period(ENCODER_R_TIM, ENCODER_PERIOD);
  timer_slave_set_mode(ENCODER_R_TIM, 0x3); // encoder
  timer_ic_set_input(ENCODER_R_TIM, TIM_IC1, TIM_IC_IN_TI1);
  timer_ic_set_input(ENCODER_R_TIM, TIM_IC2, TIM_IC_IN_TI2);
  timer_enable_counter(ENCODER_R_TIM);

	rcc_periph_clock_enable(ENCODER_R_CH1_PORT_RCC);
	gpio_mode_setup(ENCODER_R_CH1_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_R_CH1_PIN);
	gpio_set_af(ENCODER_R_CH1_PORT, ENCODER_R_CH1_AF, ENCODER_R_CH1_PIN);

  rcc_periph_clock_enable(ENCODER_R_CH2_PORT_RCC);
	gpio_mode_setup(ENCODER_R_CH2_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_R_CH2_PIN);
	gpio_set_af(ENCODER_R_CH2_PORT, ENCODER_R_CH2_AF, ENCODER_R_CH2_PIN);
}
void encoders_setup() {
  encoder_left_setup();
  encoder_right_setup();
}

int encoder_left_get_counter(void)
{
  if(ENCODER_L_INVERSION)
    return ENCODER_PERIOD-timer_get_counter(ENCODER_L_TIM);
  return timer_get_counter(ENCODER_L_TIM);
}

int encoder_right_get_counter(void)
{
  if(ENCODER_R_INVERSION)
    return ENCODER_PERIOD-timer_get_counter(ENCODER_R_TIM);
  return timer_get_counter(ENCODER_R_TIM);
}
