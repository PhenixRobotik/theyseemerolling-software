#pragma once

#include "fsm_master.h"
#include "../lowlevel/debug.h"
#include "../asservissement/odometry.h"
#include "../asservissement/calibration.h"
#include "../asservissement/pid.h"

#include <math.h>

typedef struct FSM_asser_S{
  FSM_Instance instance;
  double angle;//in rad
  double pos;//in mm
  double linear_speed;//in mm/s positive
  double angular_speed;//in rad/s positive

  double initial_sum;
  double initial_diff;
  uint32_t t0;

  double sum_goal;
  double diff_goal;

  PID_Status *pid_sigma, *pid_theta;

  //internal variables for set X Y theta
  double X_goal;
  double Y_goal;
  double theta_goal;
  int back;

  void *fsm_scheduler[10];
  int fsm_pointer;
  int fsm_pointer_max;
}FSM_asser;

void init_FSM_asser(FSM_asser *fsm_asser,PID_Status *pid_sigma,PID_Status *pid_theta);//PIDs just to check reached or not

void set_stop(FSM_asser *fsm_asser);//stop the robot at the current position
void set_theta(FSM_asser *fsm_asser,double theta);//in rad relative angle
void set_theta_speed(FSM_asser *fsm_asser,double vth);//in rad/s
void set_translation(FSM_asser *fsm_asser,double t);//in mm
void set_translation_speed(FSM_asser *fsm_asser,double vt);//in mm/s
void set_X_Y_theta(FSM_asser *fsm_asser,double x,double y,double theta,int back);//mm, mm, rad, go backward

void set_X_Y_theta_translation(FSM_asser *fsm_asser);//internal do not use
void set_X_Y_theta_rotation(FSM_asser *fsm_asser);//internal do not use

void get_order(FSM_asser *fsm_asser,double *sum_goal,double *diff_goal);

void FSM_asser_angle(FSM_Instance *fsm);
void FSM_asser_translation(FSM_Instance *fsm);
void FSM_asser_wait_end(FSM_Instance *fsm);

double limit_angle(double angle);//returns an angle between -Pi and Pi
