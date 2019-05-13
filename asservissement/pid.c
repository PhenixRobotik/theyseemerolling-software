#include "pid.h"

#include <math.h>
#include <stdlib.h>

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif
#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

void pid_init(PID_Status *pid, PID_Configuration const* config)
{
  pid->prev_eps = 0;
  pid->integral = 0;
	pid->conf = config;
}

double pid(PID_Status *pid, double eps)
{
  eps = min(eps,  pid->conf->max_eps);
  eps = max(eps, -pid->conf->max_eps);

  pid->integral += pid->conf->Te * eps;
  double output =
      pid->conf->Kp * eps
    + pid->conf->Ki * pid->integral
    + pid->conf->Kd * (eps - pid->prev_eps) / pid->conf->Te;

  pid->prev_eps = eps;
  return output;
}

bool reached(PID_Status *pid, double eps)
{
	return (
		abs(eps) < pid->conf->position_tolerance
		// && (abs(eps-pid->prev_eps) / (pid->Te<pid->speed_tolerance))
	);
}
