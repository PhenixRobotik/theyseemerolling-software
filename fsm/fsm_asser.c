#include "fsm_asser.h"

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif
#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

void init_FSM_asser(FSM_asser *fsm_asser,PID_Status *pid_sigma,PID_Status *pid_theta)//init the FSM in a stop state
{
  FSM_Instance *fsm=&(fsm_asser->instance);
  fsm->status=FSM_SUCCESS;

  fsm_asser->pid_sigma=pid_sigma;
  fsm_asser->pid_theta=pid_theta;

  set_stop(fsm_asser);
}

void set_stop(FSM_asser *fsm_asser)//stop the robot at the current position and the fsm
{
  odometry odom=odometry_get_position();
  fsm_asser->sum_goal=0.5 * (odom.left_total_distance + odom.right_total_distance);//stop at current position
  fsm_asser->diff_goal=odom.right_total_distance - odom.left_total_distance;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;

  FSM_Instance *fsm=&(fsm_asser->instance);//stop the fsm execution
  FSM_NEXT(fsm,FSM_asser_wait_end,0);
}




void set_translation_speed(FSM_asser *fsm_asser,double vt)//in mm/s
{
  fsm_asser->linear_speed=vt;
}

void set_translation(FSM_asser *fsm_asser,double t)
{
  fsm_asser->pos=t;
  fsm_asser->initial_sum=fsm_asser->sum_goal;
  fsm_asser->t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_translation,0);
  fsm->status=FSM_RUNNING;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;
}

void set_theta_speed(FSM_asser *fsm_asser,double vth)
{
  fsm_asser->angular_speed=vth;
}

void set_theta(FSM_asser *fsm_asser,double theta)//TODO: set the shortest angle, attention theta0
{
  fsm_asser->angle=theta;
  fsm_asser->initial_diff=fsm_asser->diff_goal;
  fsm_asser->t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;
}



void set_X_Y_theta(FSM_asser *fsm_asser,double x,double y,double theta)
{
  fsm_asser->X_goal=x;
  fsm_asser->Y_goal=y;
  fsm_asser->theta_goal=theta;

  fsm_asser->fsm_scheduler[0]=set_X_Y_theta_translation;
  fsm_asser->fsm_scheduler[1]=set_X_Y_theta_rotation;
  fsm_asser->fsm_pointer=0;
  fsm_asser->fsm_pointer_max=2;

  odometry odom=odometry_get_position();
  double trajectory_angle=atan2(y-odom.y,x-odom.x);
  fsm_asser->angle=-trajectory_angle+odom.theta;
  //clamp the rotation angle between -Pi and -Pi
  //in order to make the shortest rotation


  fsm_asser->initial_diff=fsm_asser->diff_goal;
  fsm_asser->t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;

}

void set_X_Y_theta_translation(FSM_asser *fsm_asser)
{
  odometry odom=odometry_get_position();

  double dx=fsm_asser->X_goal-odom.x;
  double dy=fsm_asser->Y_goal-odom.y;
  double t=sqrt(dx*dx+dy*dy);

  fsm_asser->pos=t;

  fsm_asser->initial_sum=fsm_asser->sum_goal;
  fsm_asser->t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_translation,0);
  fsm->status=FSM_RUNNING;
}

void set_X_Y_theta_rotation(FSM_asser *fsm_asser)
{
  odometry odom=odometry_get_position();
  fsm_asser->angle=-fsm_asser->theta_goal+odom.theta;
  //clamp the rotation angle between -Pi and -Pi
  //in order to make the shortest rotation

  fsm_asser->initial_diff=fsm_asser->diff_goal;
  fsm_asser->t0=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;
}



void get_order(FSM_asser *fsm_asser,double *sum_goal,double *diff_goal)//just return the orders
{
  *sum_goal=fsm_asser->sum_goal;
  *diff_goal=fsm_asser->diff_goal;
}



void FSM_asser_translation(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;
  double t=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  double pos=(t-fsm_asser->t0)*fsm_asser->linear_speed;
  if(fsm_asser->pos<0)
  {
    pos=-pos;
  }

  if(fsm_asser->pos>0 && pos>fsm_asser->pos)
  {
    pos=fsm_asser->pos;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }
  else if(fsm_asser->pos<0 && pos<fsm_asser->pos)
  {
    pos=fsm_asser->pos;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }
  fsm_asser->sum_goal=pos+fsm_asser->initial_sum;
}


void FSM_asser_angle(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;
  double t=SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  double angle=(t-fsm_asser->t0)*fsm_asser->angular_speed;
  if(fsm_asser->angle<0)
  {
    angle=-angle;
  }

  if(fsm_asser->angle>0 && angle>fsm_asser->angle)
  {
    angle=fsm_asser->angle;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }
  else if(fsm_asser->angle<0 && angle<fsm_asser->angle)
  {
    angle=fsm_asser->angle;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }
  fsm_asser->diff_goal=angle*Encoders_Axis_Distance+fsm_asser->initial_diff;
}

void FSM_asser_wait_end(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;
  if(reached(fsm_asser->pid_theta) && reached(fsm_asser->pid_sigma))
  {
    if(fsm_asser->fsm_pointer>=fsm_asser->fsm_pointer_max)
    {
      FSM_NEXT(fsm,FSM_NOP,0);
    }
    else
    {
      void (*to_run)(FSM_asser *fsm_asser)=(fsm_asser->fsm_scheduler)[fsm_asser->fsm_pointer];
      to_run(fsm_asser);//runs the next setup function
      fsm_asser->fsm_pointer++;
    }
  }
}
