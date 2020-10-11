#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/motors.h"

#include "can/can.h"
#include "can/link_can.h"

#include "fsm/fsm_asser.h"
#include "asservissement/odometry.h"
#include "asservissement/pid.h"
#include "asservissement/calibration.h"

#include <libopencm3/stm32/can.h>
#include <stdbool.h>

void asservissement();

void hard_fault_handler() {
  return; //mdr
  //while(1);
}

volatile bool enable = 0;
int main() {
  clock_setup();
  gpio_setup();
  uart_setup();
  can_setup();
  setup_com(); //mdr
  motors_setup();
  odometry_setup();

  // Pour commencer dans de bonnes conditions
  hard_fault_handler();

  //test
  while(1)
  {
    delay_ms(1000);
    led_toggle_status();
    motor_a_set(0);
    motor_b_set(0);
  }

  do{
    while(!enable);
    asservissement();
  }while(true != false);

  // TODO : priorités interruptions

  while (1) {};
  return 0;
}

FSM_asser fsm_asser;
void asservissement() {
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

  odometry odom;

  double t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0,t1;
  reset_odometry();
  while(enable)
  {
    fsm->run(fsm);
    odom = odometry_get_position();
    get_order(&fsm_asser, &sum_goal, &diff_goal);

    voltage_sum = pid(
      &pid_sigma,
      sum_goal - 0.5 * (odom.left_total_distance + odom.right_total_distance)
    );
    voltage_diff = pid(
      &pid_theta,
      diff_goal - (odom.right_total_distance - odom.left_total_distance)
    );

    voltage_A = voltage_sum + voltage_diff;
    voltage_B = voltage_sum - voltage_diff;

    motor_a_set(voltage_A);
    motor_b_set(voltage_B);

    do{
      t1=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
    }while(t1-t0<pid_sigma.conf->Te);
    t0=t1;

  }

  motor_a_set(0);
  motor_b_set(0);
}
