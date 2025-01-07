#include "state_machine.h"



struct  state_transition{

  //hold the state from tranisition come from
  state_e from;
  //event transistion
  state_event_e event;
  //state coming to
  state_e to;

}

static const struct state_transition state_transistions[] = {
  { STATE_PAUSE, STATE_EVENT_   , STATE_WAIT };
  {};
  {};
  {};
  {};
  {};
  {};
  //add all the from_state->state evnt used -> to_state array for the whole fwmav state diagram
}






struct state_machine_data{
  state_e state;
  struct state_common_data commmon;
  struct state_wait_data wait;
  struct //all the states mentioned in the state_data
  
}


static inti



void state_machine_run(void)
{
  //allocate and initialize state machine data
  // while loop
  // process input(e.g timeout,command,pause,etc)
  //process event
}