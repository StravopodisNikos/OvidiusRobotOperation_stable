#ifndef led_indicators_h
#define led_indicators_h

#include <avr/pgmspace.h>

//                    INDICATOR NAME		   	RGB VALUE		   COLOR
const PROGMEM unsigned char turn_off_led[]              = {0, 0, 0};
const PROGMEM unsigned char critical_error_led[]        = {250, 0, 0};
const PROGMEM unsigned char completed_move_indicator[]  = {244, 164, 96};      // sandybrown
const PROGMEM unsigned char torque_off_indicator[]     	= {148, 0, 211};       // dark violet
const PROGMEM unsigned char motors_homed_indicator[]   	= {255, 140, 0};       // orange
const PROGMEM unsigned char motors_moving_indicator[]  	= {135, 206, 235};     // skyblue
const PROGMEM unsigned char updated_force_indicator[]  	= {241, 235, 0};       // yellow
const PROGMEM unsigned char updated_imu_indicator[]  	= {210, 45, 180};      // violet light
const PROGMEM unsigned char saved_data_log_dicator[]  	= {32, 62, 142};       // blue light
const PROGMEM unsigned char access_eeprom_indicator[]  	= {210, 45, 180};      // violet light

#endif
