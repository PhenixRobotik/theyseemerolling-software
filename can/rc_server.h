#ifndef RC_SERVER_H
#define RC_SERVER_H

#include "remote_call.h"

extern RC_Server server;

int init_server();

typedef enum Server_Functions_E{
				TEST_FUNCTION,
}Server_Functions;

void test_function(RC_Server *server);

#endif
