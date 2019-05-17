#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/debug.h"
#include "lowlevel/motors.h"

#include "fsm/fsm_asser.h"
#include "asservissement/odometry.h"
#include "asservissement/pid.h"
#include "asservissement/calibration.h"

void asservissement();

int main() {
  clock_setup();
  gpio_setup();
  debug_setup();
  motors_setup();
  odometry_setup();

  asservissement();

  while (1) {};
  return 0;
}

void asservissement() {
  double voltage_A=0,
        voltage_B=0,
        voltage_sum=0,
        voltage_diff=0; // motor control variables
  double sum_goal=0, diff_goal=0;

  PID_Status pid_sigma, pid_theta;
  pid_init(&pid_sigma, &PID_Configuration_sigma);
  pid_init(&pid_theta, &PID_Configuration_theta);

  FSM_asser fsm_asser;
  init_FSM_asser(&fsm_asser,&pid_sigma,&pid_theta);
  FSM_Instance *fsm= (FSM_Instance*)&fsm_asser;//set the current fsm to fsm_asser

  odometry odom;
  odometry_get_position();

  set_theta_speed(&fsm_asser,1.57/5.0);
  /*double angle=1.57;
  set_theta(&fsm_asser,angle);
  angle*=-1;*/

  set_translation_speed(&fsm_asser,10.0);
  /*double d=-70;
  set_translation(&fsm_asser,d);
  d*=-1;*/

  set_X_Y_theta(&fsm_asser,10,-10,0);

  while(1)
  {
    fsm->run(fsm);
    odom = odometry_get_position();
    get_order(&fsm_asser, &sum_goal, &diff_goal);

    if(fsm->run==FSM_NOP)//condition for command end
    {
      print_odometry(&odom);
      while(1)
      {

      }

      //set_theta(&fsm_asser,angle);
      //angle*=-1;

      //set_translation(&fsm_asser,d);
      //d*=-1;

      //led_set_status(1);
    }
    else
    {
      led_set_status(0);
    }

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
    delay_ms(pid_sigma.conf->Te);
  }
}
