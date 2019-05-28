#ifndef RC_SERVER_H
#define RC_SERVER_H

#include "remote_call.h"

extern RC_Server server;

int init_server();

typedef enum Server_Functions_E{
  GET_TOF_DATA,
  GET_US_DATA
}Server_Functions;

void test_function(RC_Server *server);
void get_tof_data(RC_Server *server);
void get_us_data(RC_Server *server);

#endif
