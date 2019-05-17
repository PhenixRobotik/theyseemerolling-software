#pragma once

#include "fsm_master.h"
#include "../lowlevel/debug.h"
#include "../asservissement/odometry.h"
#include "../asservissement/calibration.h"

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
}FSM_asser;

void init_FSM_asser(FSM_asser *fsm_asser);

void set_stop(FSM_asser *fsm_asser);//stop the robot at the current position
void set_theta(FSM_asser *fsm_asser,double theta);//in rad relative angle
void set_theta_speed(FSM_asser *fsm_asser,double vth);//in rad/s
void set_translation(FSM_asser *fsm_asser,double t);//in mm
void set_translation_speed(FSM_asser *fsm_asser,double vt);//in mm/s
void set_X_Y_theta(FSM_asser *fsm_asser,double x,double y,double theta);//mm mm rad

void get_order(FSM_asser *fsm_asser,double *sum_goal,double *diff_goal);

void FSM_asser_angle(FSM_Instance *fsm);
void FSM_asser_translation(FSM_Instance *fsm);