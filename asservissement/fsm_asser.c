#include "fsm_asser.h"

void init_FSM_asser(FSM_asser *fsm_asser)//init the FSM in a stop state
{
  FSM_Instance *fsm=&(fsm_asser->instance);
  fsm->status=FSM_SUCCESS;
  set_stop(fsm_asser);
}

void set_stop(FSM_asser *fsm_asser)//stop the robot at the current position and the fsm
{
  odometry odom=odometry_get_position();
  fsm_asser->sum_goal=0.5 * (odom.left_total_count + odom.right_total_count);//stop at current position
  fsm_asser->diff_goal=odom.right_total_count - odom.left_total_count;

  FSM_Instance *fsm=&(fsm_asser->instance);//stop the fsm execution
  FSM_NEXT(fsm,FSM_NOP,0);
}




void set_translation_speed(FSM_asser *fsm_asser,double vt)//in mm/s
{
  fsm_asser->linear_speed=vt;
}

void set_theta_speed(FSM_asser *fsm_asser,double vth)
{
  fsm_asser->angular_speed=vth;
}

void set_theta(FSM_asser *fsm_asser,double theta)
{
  
}




void get_order(FSM_asser *fsm_asser,long int *sum_goal,long int *diff_goal)//just return the orders
{
  *sum_goal=fsm_asser->sum_goal;
  *diff_goal=fsm_asser->diff_goal;
}





void FSM_asser_init(FSM_Instance *fsm)
{
  fsm->status=FSM_RUNNING;
  FSM_asser *fsm_pos=(FSM_asser *) fsm;
  FSM_NEXT(fsm,FSM_asser_end,0);
  echo("init\n\r");
}

void FSM_asser_end(FSM_Instance *fsm)
{
  fsm->status=FSM_SUCCESS;
  echo("end\n\r");
}
