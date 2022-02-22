/*
 * THIS ADDRESSED ARE WRITTEN IN EEPROM MEMORY OF ARDUINO MEGA
 * CONTROLLER OF OVIDIUS ROBOT - CREATED 17-2-21
 */

// EEPROM AREA ADDRESSES [0~255]

// STEPPER JOINT 1 [0-100]
#define ID_EEPROM_ADDR		0		// int
#define MAX_POS_LIM_ADDR	10		// float
#define MIN_POS_LIM_ADDR	20		// float
#define STEP_ANGLE_ADDR		30		// float
#define CS_EEPROM_ADDR		40		// byte			// Always updated with homing
#define CP_EEPROM_ADDR		50		// byte			// 50 must be garbage now(changed in 30.6.2020)
#define CD_EEPROM_ADDR		60		// uint32_t		// ...	

// GRIPPER	[101-110]
#define CS_GRIPPER_EEPROM_ADDR	101		// 
// FORCE SENSOR [111-120]
#define CS_FORCE_EEPROM_ADDR	111		//
