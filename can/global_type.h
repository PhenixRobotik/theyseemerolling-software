#pragma once

#include "libcanard/libcanard/canard.h"
#include "asservissement/odometry.h"

typedef struct{
  CanardInstance can_ins;

  odometry odom;

}global_data;
