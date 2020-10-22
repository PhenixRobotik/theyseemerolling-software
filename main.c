#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/motors.h"

#include "can/can.h"
#include "can/canard_link.h"
#include "can/can_defines.h"

#include "fsm/fsm_asser.h"
#include "asservissement/odometry.h"
#include "asservissement/pid.h"
#include "asservissement/calibration.h"

#include <libopencm3/stm32/can.h>
#include <stdbool.h>

#define MAIN_LOOP_PERIOD 0.07

static global_data data_g;

void asservissement();

void hard_fault_handler() {
  return; //mdr
  //while(1);
}

volatile bool enable = 0;
FSM_asser fsm_asser;

int main()
{
  clock_setup();
  gpio_setup();
  uart_setup();
  motors_setup();
  odometry_setup();

  can_setup();
  init_can_link(&data_g);


  //asser init
  double voltage_A=0,
        voltage_B=0,
        voltage_sum=0,
        voltage_diff=0; // motor control variables
  double sum_goal=0, diff_goal=0;


  PID_Status pid_sigma, pid_theta;
  pid_init(&pid_sigma, &PID_Configuration_sigma);
  pid_init(&pid_theta, &PID_Configuration_theta);

  //FSM_asser fsm_asser;
  init_FSM_asser(&fsm_asser,&pid_sigma,&pid_theta);
  FSM_Instance *fsm= (FSM_Instance*)&fsm_asser;//set the current fsm to fsm_asser

  reset_odometry();

  //set_translation_speed(&fsm_asser, 10);//in mm/s
  //set_translation(&fsm_asser, 10);




  uint32_t t0, t1;
  double dt;
  t0 = get_systicks();

  while(1)
  {
    if(data_g.odom_to_set)
    {
      data_g.odom_to_set = 0;
      set_odometry(data_g.x_set, data_g.y_set, data_g.theta_set);
    }
    data_g.odom = odometry_get_position();
    data_g.about_da_power = about_da_power();
    tx_feed_back(&data_g);
    led_toggle_status();


    enable = 1;
    if(enable)
    {
      fsm->run(fsm);
      get_order(&fsm_asser, &sum_goal, &diff_goal);

      voltage_sum = pid(
        &pid_sigma,
        sum_goal - 0.5 * (data_g.odom.left_total_distance + data_g.odom.right_total_distance)
      );
      voltage_diff = pid(
        &pid_theta,
        diff_goal - (data_g.odom.right_total_distance - data_g.odom.left_total_distance)
      );

      voltage_A = voltage_sum + voltage_diff;
      voltage_B = voltage_sum - voltage_diff;

      motor_b_set(voltage_A);
      motor_a_set(voltage_B);
    }
    else
    {
      motor_a_set(0);
      motor_b_set(0);
    }


    do{
      t1 = get_systicks();
      dt = t1 - t0;
      dt = SYSTICK_TO_MILLIS(dt)/1000.0;
    }while(dt<MAIN_LOOP_PERIOD);
    t0 = t1;
    compute_speeds(dt);
  }

  // TODO : priorités interruptions

  while (1) {};
  return 0;
}
