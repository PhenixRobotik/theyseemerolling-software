#include "rc_server.h"

#include "link_can.h"

RC_Server server;

int init_server(){
  RC_Server_Init(&server, &pi_iface);

  // Test function
  int r = RC_Server_Add_Function(&server,
				 TEST_FUNCTION,
				 test_function,
				 "sbBuifFs",
				 "sbBuifFs",
				 RC_IMMEDIATE);
  
  return r;
}

void test_function(RC_Server *server){
  char str_start[RC_STR_SIZE], str_end[RC_STR_SIZE];
  uint8_t byte;
  uint16_t half_word;
  uint32_t u_word;
  int word;
  float simpleFloat;
  double doubleFloat;

  int r = RC_Server_Get_Args(server,
			     str_start,
			     &byte,
			     &half_word,
			     &u_word,
			     &word,
			     &simpleFloat,
			     &doubleFloat,
			     str_end);
  if(r != 0){
    //    led_on();
    //let the other end timeout
    return;
  }

  /* Change values and send */
  byte++;
  half_word++;
  u_word++;
  word = -(word + ((word > 0)?(1):(-1)));
  simpleFloat += 0.01;
  doubleFloat -= 0.01;

  RC_Server_Return(server, str_start, byte, half_word, u_word, word, simpleFloat, doubleFloat, str_end);
}
