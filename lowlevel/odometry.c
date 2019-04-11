#include "odometry.h"

volatile odometry odometry_internal;//WARNING global odometry variable for update during interruption
//do not use directly outside odometry.c

void odometry_setup(void)
{
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

  //setup the pooling timer
	rcc_periph_clock_enable(ODOM_RCC_TIM);
	rcc_periph_reset_pulse(ODOM_RST_TIM);
	timer_set_mode(ODOM_TIM, TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(ODOM_TIM, ODOM_TIM_PRESCALER);
	timer_disable_preload(ODOM_TIM);
	timer_continuous_mode(ODOM_TIM);
	timer_set_period(ODOM_TIM, ODOM_TIM_PERIOD);
	timer_set_oc_value(ODOM_TIM, TIM_OC1,0);//compare to zero
	timer_enable_counter(ODOM_TIM);

  //start the encoders
  encoder_left_setup();
  encoder_right_setup();

  odometry_internal.x=0;//init the odometry structure
  odometry_internal.y=0;
  odometry_internal.theta=0;
  odometry_internal.left_count=encoder_right_get_counter();
  odometry_internal.right_count=encoder_left_get_counter();

  //start pooling
  nvic_enable_irq(ODOM_NVIC_TIM_IRQ);
	timer_enable_irq(ODOM_TIM, TIM_DIER_CC1IE);
}

void timX_isr(void)
{
  if (timer_get_flag(ODOM_TIM, TIM_SR_CC1IF))
  {
    timer_clear_flag(ODOM_TIM, TIM_SR_CC1IF);/* Clear compare interrupt flag. */
    gpio_toggle(LED_PORT, LED_PIN);
    int dl_l=encoder_left_update(&odometry_internal.left_count);//first read the encoders, should not be different than -1, 0 or 1
    int dl_r=encoder_right_update(&odometry_internal.right_count);
    odometry_internal.x+=cosf(odometry_internal.theta)*ENCODER_STEP_DIST*(dl_l+dl_r);//update the position
    odometry_internal.y+=sinf(odometry_internal.theta)*ENCODER_STEP_DIST*(dl_l+dl_r);
    odometry_internal.theta+=DELTA_L*(dl_l-dl_r);

    if(odometry_internal.theta>PI)//limit robot angle between -PI and PI
    {
      odometry_internal.theta=odometry_internal.theta-2*PI;
    }
    else if(odometry_internal.theta<PI)
    {
      odometry_internal.theta=odometry_internal.theta+2*PI;
    }
  }
}

odometry odometry_get_position(void)
{
  return odometry_internal;
}
