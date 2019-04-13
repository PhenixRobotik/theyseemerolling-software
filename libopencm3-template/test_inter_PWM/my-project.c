#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>



static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}

static void gpio_setup(void)
{
//RCC_TIM1
	rcc_periph_clock_enable(RCC_TIM1);
	//timer_reset(RCC_TIM1);
	timer_set_mode(TIM1,TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE,TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM1, 0);
	timer_enable_preload(TIM1);
	timer_continuous_mode(TIM1);
	timer_set_repetition_counter(TIM1, 0);
	timer_set_period(TIM1, 1000);
	timer_enable_break_main_output(TIM1);

	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO0);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ, GPIO0);
	gpio_set_af(GPIOC,GPIO_AF2,GPIO0);

	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_value(TIM1, TIM_OC1, 0);

	timer_generate_event(TIM1, TIM_EGR_UG);

	timer_enable_counter(TIM1);
	timer_enable_oc_output(TIM1, TIM_OC1);

}

int main(void)
{
	clock_setup();
	gpio_setup();
	//tim_setup();
	int j=0;

	while (1) {
		for (int i = 0; i < 100000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
		j++;
		if (j>1000) j=0;
		timer_set_oc_value(TIM1, TIM_OC1, j);
	}

	return 0;
}
