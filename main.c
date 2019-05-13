#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/debug.h"
#include "lowlevel/motors.h"

#include "asservissement/fsm_asser.h"
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

void asservisement() {
  float voltage_A=0,
        voltage_B=0,
        voltage_sum=0,
        voltage_diff=0; // motor control variables
  long int sum_goal=0, diff_goal=0;

  PID_Status pid_delta, pid_theta;
  pid_init(&pid_delta, &PID_Configuration_delta);
  pid_init(&pid_theta, &PID_Configuration_theta);

  FSM_asser fsm_asser;
  init_FSM_asser(&fsm_asser);
  FSM_Instance *fsm= (FSM_Instance*)&fsm_asser;//set the current fsm to fsm_asser

  odometry odom;
  odometry_get_position();

  set_theta_speed(&fsm_asser,1.57/2.0);
  set_theta(&fsm_asser,1.57);

  while(1)
  {
    fsm->run(fsm);
    odom = odometry_get_position();

    //print_odometry(&odom);

    get_order(&fsm_asser, &sum_goal, &diff_goal);
    echo_int(sum_goal);
    echo("|");
    echo_int(diff_goal);
    echo("\n\r");

    voltage_sum = pid(
      &pid_delta,
      sum_goal - 0.5 * (odom.left_total_count + odom.right_total_count)
    );
    voltage_diff = pid(
      &pid_theta,
      diff_goal - (odom.right_total_count - odom.left_total_count)
    );

    voltage_A = voltage_sum + voltage_diff;
    voltage_B = voltage_sum - voltage_diff;

    motor_a_set(voltage_A);
    motor_b_set(voltage_B);
    delay_ms(pid_delta.conf->Te);
  }
}
