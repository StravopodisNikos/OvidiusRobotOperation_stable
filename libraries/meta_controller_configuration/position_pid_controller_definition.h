#ifndef position_pid_controller_definition_h
#define position_pid_controller_definition_h

#include <avr/pgmspace.h>

// P2P TASK POINTS -> Cspace
const PROGMEM int total_available_gains = 10;

const PROGMEM int16_t KPgains[total_available_gains] = {50, 100, 200, 250, 300, 350, 400, 500, 550, 600 };
const PROGMEM int16_t KIgains[total_available_gains] = {0 , 10, 50, 100, 150, 200, 225, 250, 275, 300 };
const PROGMEM int16_t KDgains[total_available_gains] = {0 , 10, 50, 100, 150, 200, 225, 250, 275, 300 };


#endif
