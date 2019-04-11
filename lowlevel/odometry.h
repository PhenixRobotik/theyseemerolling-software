#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <math.h>//LDLIBS+= -lm

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#define PI 3.14159265359
#include "encoders.h"

#define LED_PORT GPIOA
#define LED_PIN GPIO5

//Robot mechanical caracteristics, dimensions in mm
#define ENCODER_DIST 200//distance between encoders
#define ENCODER_WHEEL_D 60//diameter of the encoder wheel
#define ENCODER_STEP_DIST ENCODER_WHEEL_D*PI/ENCODER_PERIOD//distance for one encoder step
#define DELTA_L ENCODER_STEP_DIST/ENCODER_DIST//ENCODER_STEP_DIST/ENCODER_DIST

//POOLING Timer
#define ODOM_TIM TIM3
#define timX_isr(void) tim3_isr(void)
#define ODOM_RCC_TIM RCC_TIM3
#define ODOM_NVIC_TIM_IRQ NVIC_TIM3_IRQ
#define ODOM_RST_TIM RST_TIM3
#define ODOM_TIM_PRESCALER ((rcc_apb1_frequency * 2) / 5000)//change these 2 lines to setup interruption frequency
#define ODOM_TIM_PERIOD 600

typedef struct{//positions in mm and angles in rad
  float x;
  float y;
  float theta;

  int left_count;//last encoders count
  int right_count;
}odometry;

void odometry_setup(void);
odometry odometry_get_position(void);

#endif
