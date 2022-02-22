#ifndef pseudo_led_indicators_h
#define pseudo_led_indicators_h

#include <avr/pgmspace.h>
// Values are defined based on table given in: https://www.rapidtables.com/web/color/RGB_Color.html

//                    INDICATOR NAME		   	RGB VALUE		   COLOR
const PROGMEM unsigned char turn_off_led[]              = {0, 0, 0};
const PROGMEM unsigned char critical_error_led[]        = {255, 0, 0};
const PROGMEM unsigned char entered_p2pcsp[]            = {0  ,0, 255 };       // deep blue
const PROGMEM unsigned char pinged_indicator[]          = {0, 255, 0  };       // deep green
const PROGMEM unsigned char finished_indicator[]        = {128, 0, 0  };       // Maroon
const PROGMEM unsigned char completed_move_indicator[]  = {128, 0, 128};       // Purple
const PROGMEM unsigned char torque_off_indicator[]     	= {148, 0, 211};       // dark violet
const PROGMEM unsigned char motors_homing_indicator[]   = {0, 0, 255};       // BLUE
const PROGMEM unsigned char motors_moving_indicator[]  	= {0, 255, 0};     // GREEN
const PROGMEM unsigned char limit_switch_indicator[]  	= {128, 128, 0};       // Olive
const PROGMEM unsigned char access_eeprom_indicator[]  	= {210, 45, 180};      // violet light

#endif
