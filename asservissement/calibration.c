#include "calibration.h"

#include "lowlevel/encoders.h"

const double Pi =  3.14159265359;

// Robot mechanical caracteristics, dimensions in mm

const double Encoders_Axis_Distance  = 192;
const double Encoders_Wheel_Diameter = 60.0;
const double Encoders_Dist_Per_Step  = Encoders_Wheel_Diameter * Pi / ENCODER_PERIOD;
const double Encoders_Theta_Per_Diff = Encoders_Dist_Per_Step / Encoders_Axis_Distance;

const PID_Configuration PID_Configuration_delta = {
  .Te = 0.01,

  .Kp = 0.01,
  .Ki = 0.00,
  .Kd = 0.000,

  .max_eps = 100000000,
  .position_tolerance = 100,
  .speed_tolerance = 0
};

const PID_Configuration PID_Configuration_theta = {
  .Te = 0.01,

  .Kp = 0.005,
  .Ki = 0.000,
  .Kd = 0.000,

  .max_eps = 350,
  .position_tolerance = 100,
  .speed_tolerance = 0
};
