#pragma once

#include "libcanard/libcanard/canard.h"
#include "asservissement/odometry.h"

typedef struct{
  CanardInstance can_ins;

  uint8_t about_da_power;
  odometry odom;

  uint8_t stream_enc_l;
  uint8_t stream_enc_r;
  uint8_t stream_power;

  float x_set;
  float y_set;
  float theta_set;
  uint8_t odom_to_set;

  float translation_to_set_value;
  uint8_t translation_to_set;
  float translation_speed_to_set_value;
  uint8_t translation_speed_to_set;

  float rotation_to_set_value;
  uint8_t rotation_to_set;
  float rotation_speed_to_set_value;
  uint8_t rotation_speed_to_set;

  uint8_t XYtheta_to_set;
  float XYtheta_value_x;
  float XYtheta_value_y;
  float XYtheta_value_theta;

  uint8_t pid_sigma_to_set;
  float Kp_sig;
  float Ki_sig;
  float Kd_sig;

  uint8_t pid_delta_to_set;
  float Kp_del;
  float Ki_del;
  float Kd_del;


}global_data;
