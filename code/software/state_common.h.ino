#include "drivers/ir_remote.h"
#include <stdint.h>
#include "destination.h"
#include ""//include proper library used in below code

#define STATE_COMMON.h

typedef enum {
  STATE_PARKED,
  STATE_FLAPPING,
  STATE_LANDING,
  sTATE_MOVEMENT
  STATE_PAUSE
} state_e;

typedef enum {
  STATE_EVENT_TIMEOUT,
  STATE_EVENT_FLAPPINGCOMMAND,
  STATE_EVENT_//ADD MORE EVENTS PROPERLY

} state_event_e;


struct state_machine_data;
typedef uint32_t timer_t;
struct ring_buffer;


struct state_common_data{
  struct state_machine_data *state_machine_data;
  timer_t *timer;
  struct//add proper struct to call from any state file
  ir_emd_e cmd;
  struct ring_buffer *input_history;  //ring buffer for all the inputs not directly to the database (which causes overload and malfunctioning) acts as  filter buffer from sensors/input data to datastorage

}

#endif // STATE_COMMON.h