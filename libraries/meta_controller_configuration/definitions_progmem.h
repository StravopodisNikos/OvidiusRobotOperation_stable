#ifndef definitions_progmem_h
#define definitions_progmem_h

#include <avr/pgmspace.h>

// sends to flash memory instead of SRAM. Contains the vars in definitions.h

// Control Table Used Items Address
const PROGMEM uint16_t ADDR_PRO_TORQUE_ENABLE         =        512;                // Control table address is different in Dynamixel model
const PROGMEM uint16_t ADDR_PRO_GOAL_POSITION         =        564;
const PROGMEM uint16_t ADDR_PRO_GOAL_VELOCITY         =        552;
const PROGMEM uint16_t ADDR_PRO_PROF_ACCEL            =        556;
const PROGMEM uint16_t ADDR_PRO_PROF_VEL              =        560;
const PROGMEM uint16_t ADDR_PRO_PRESENT_VELOCITY      =        576;
const PROGMEM uint16_t ADDR_PRO_MOVING                =        570;
const PROGMEM uint16_t ADDR_PRO_MOVING_STATUS         =        571;
const PROGMEM uint16_t ADDR_PRO_PRESENT_POSITION      =        580;
const PROGMEM uint16_t ADDR_PRO_ACCEL_LIMIT           =        40;
const PROGMEM uint16_t ADDR_PRO_VEL_LIMIT             =        44;
const PROGMEM uint16_t ADDR_PRO_MAX_POS_LIMIT         =        48;
const PROGMEM uint16_t ADDR_PRO_MIN_POS_LIMIT         =        52;
const PROGMEM uint16_t ADDR_PRO_LED_BLUE              =        515;
const PROGMEM uint16_t ADDR_PRO_LED_GREEN             =        514;
const PROGMEM uint16_t ADDR_PRO_LED_RED               =        513;
const PROGMEM uint16_t ADDR_PRO_RETURN_DELAY_TIME     =        9;
const PROGMEM uint16_t ADDR_PRO_BAUDRATE              =        8;
const PROGMEM uint16_t ADDR_PRO_ID                    =        7;
const PROGMEM uint16_t ADDR_PRO_SECONDARY_ID          =        12;
const PROGMEM uint16_t ADDR_PRO_FIRMWARE_VER          =        6;
const PROGMEM uint16_t ADDR_PRO_MODEL_INFO            =        2;

// Data Byte Length of default control table items
const PROGMEM uint8_t LEN_PRO_TORQUE_ENABLE           =       1;
const PROGMEM uint8_t LEN_PRO_GOAL_POSITION           =       4;
const PROGMEM uint8_t LEN_PRO_GOAL_VELOCITY           =       4;
const PROGMEM uint8_t LEN_PRO_PROF_ACCEL              =       4;
const PROGMEM uint8_t LEN_PRO_PROF_VEL                =       4;
const PROGMEM uint8_t LEN_PRO_PRESENT_VELOCITY        =       4;
const PROGMEM uint8_t LEN_PRO_PRESENT_POSITION        =       4;
const PROGMEM uint8_t LEN_PRO_ACCEL_LIMIT             =       4;
const PROGMEM uint8_t LEN_PRO_VEL_LIMIT               =       4;
const PROGMEM uint8_t LEN_PRO_MAX_POS_LIMIT           =       4;
const PROGMEM uint8_t LEN_PRO_MIN_POS_LIMIT           =       4;
const PROGMEM uint8_t LEN_PRO_MOVING                  =       1;
const PROGMEM uint8_t LEN_PRO_MOVING_STATUS           =       1; 
const PROGMEM uint8_t LEN_PRO_LED                     =       1;
const PROGMEM uint8_t LEN_PRO_RETURN_DELAY_TIME       =       1;
const PROGMEM uint8_t LEN_PRO_BAUDRATE                =       1;
const PROGMEM uint8_t LEN_PRO_ID                      =       1;
const PROGMEM uint8_t LEN_PRO_SECONDARY_ID            =       1;
const PROGMEM uint8_t LEN_PRO_FIRMWARE_VER            =       1;
const PROGMEM uint8_t LEN_PRO_MODEL_INFO              =       4;

// Data Byte Length of custom command items for indirect addressing
const PROGMEM uint8_t LEN_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED        =   15;                   // 4*3 for pos/vel/accel and 3*1 for leds
const PROGMEM uint8_t LEN_PRO_INDIRECTDATA_FOR_READ_PP_MV              =   5;                    // 4 for present position and 1 for dxl_moving
const PROGMEM uint8_t LEN_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL     =   20;	
const PROGMEM uint8_t LEN_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED	       =   3;	

const PROGMEM uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED    =   168;                  // EEPROM region for free Indirect Address for PH54!!!
const PROGMEM uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV          =   198;                  // ...
const PROGMEM uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL =   208;
const PROGMEM uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_RGB_LED       =   248; 

const PROGMEM uint16_t ADDR_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED       =   634;                  // RAM region for the corresponding free Indirect Data for PH54!!!
const PROGMEM uint16_t ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV             =   649;                  // ...
const PROGMEM uint16_t ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL    =   654;
const PROGMEM uint16_t ADDR_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED          =   674;

// Protocol version
#define DXL_PROTOCOL_VERSION                        2.0                 // Protocol Version of Dunamixels used

// Serial Communication Properties
#define SERIAL_BAUDRATE					115200
#define DXL_BAUDRATE1                                	57600
#define DXL_BAUDRATE2                                	1000000
#define DEVICENAME                              	"/dev/ttyACM1"
#define CMD_SERIAL                              	Serial
#define ESC_ASCII_VALUE                         	0x1b

#define DXL_RESOLUTION					501923	

// Metamorphic Structure Info	
const PROGMEM uint8_t nPseudoJoints		=	1;	

// Definitions for PseudoSPIcomm library

// PINS CONFIGURATION MASTER &
// PINS CONFIGURATION SLAVE
const PROGMEM uint8_t MOSI_NANO 		=	11;
const PROGMEM uint8_t MISO_NANO 		=	12;
const PROGMEM uint8_t SCK_NANO 			=	13;
const PROGMEM uint8_t MOSI_MASTER 		=	11;			// 11 for UNO 51 for MEGA
const PROGMEM uint8_t MISO_MASTER 		=	12;			// 12         50
const PROGMEM uint8_t SCK_MASTER		=	13;			// 13         52
#define TXled_Pin 					A2		
#define RXled_Pin 					A1		
const PROGMEM uint8_t statusLED_Pin		=	6;			// 6	
#define  dirPin_NANO				A5
#define  stepPin_NANO				A4
#define  enabPin_NANO				A3
const PROGMEM uint8_t hallSwitch_Pin		=	3;			// used for homing hall sensor
const PROGMEM uint8_t pseudoLimitSwitch_Pin 	=	2;
const PROGMEM uint8_t RELAY_lock_Pin2		=	8;
const PROGMEM uint8_t RELAY_lock_Pin		=	7;

const PROGMEM uint8_t SSpinPseudo1		=	4;
const PROGMEM uint8_t SSpinPseudo2		=	5;

const PROGMEM uint8_t PSEUDO1_ID 		=	1;
const PROGMEM uint8_t PSEUDO2_ID 		=	2;
const PROGMEM uint8_t PSEUDO3_ID 		=	3;
const PROGMEM uint8_t PSEUDO4_ID 		=	4;
const PROGMEM uint8_t PSEUDO5_ID 		=	5;
const PROGMEM uint8_t PSEUDO6_ID 		=	6;

const PROGMEM uint8_t ADDRESS_WIDTH		=	6;

const PROGMEM uint8_t PSEUDO_NUMBER1 		=	1;
const PROGMEM uint8_t PSEUDO_NUMBER2		=	2;
const PROGMEM uint8_t PSEUDO_NUMBER3		=	3;
const PROGMEM uint8_t PSEUDO_NUMBER4		=	4;
const PROGMEM uint8_t PSEUDO_NUMBER5		=	5;
const PROGMEM uint8_t PSEUDO_NUMBER6		=	6;

// ============================================================================
// SPI COMMUNICATION BYTES => MUST BE UNIQUE
// ============================================================================

// COMMANDS SENT -> in single byte transfer each ci is regarded as command!
// ci's: these are the bytes given to setGoalPositionMaster() for desired anatomy Metamorphosis
const PROGMEM uint8_t junk_command	=	0;
const PROGMEM uint8_t c1		=	1;
const PROGMEM uint8_t c2		=	2;
const PROGMEM uint8_t c3		=	3;
const PROGMEM uint8_t c4		=	4;
const PROGMEM uint8_t c5		=	5;	
const PROGMEM uint8_t c6		=	6;
const PROGMEM uint8_t c7		=	7;
const PROGMEM uint8_t c8		=	8;
const PROGMEM uint8_t c9		=	9;	
const PROGMEM uint8_t c10		=	10;
const PROGMEM uint8_t c11		=	11;
const PROGMEM uint8_t c12		=	12;
const PROGMEM uint8_t c13		=	13;
const PROGMEM uint8_t c14		=	14;
const PROGMEM uint8_t c15		=	15;
const PROGMEM uint8_t wrong_ci		=	16;
const PROGMEM uint8_t home_ci		=	8;

// COMMANDS FROM MASTER TO SLAVE
const PROGMEM uint8_t CMD_LOCK		    =	20;
const PROGMEM uint8_t CMD_UNLOCK	    =	21;
const PROGMEM uint8_t CMD_SGP	  	    =   30;		// In single byte transfer is overrided
const PROGMEM uint8_t CMD_MOVE			=40;
const PROGMEM uint8_t CMD_STOP			=41;		// Danger Stop Event!
const PROGMEM uint8_t CMD_HOME			=42;
const PROGMEM uint8_t CMD_PRE_HOME        	=43;	 
const PROGMEM uint8_t CMD_CONNECT		=60;
const PROGMEM uint8_t CMD_GIVE_IS		=70;
const PROGMEM uint8_t CMD_GIVE_CS		=71;
const PROGMEM uint8_t CMD_GIVE_CP		=72;
const PROGMEM uint8_t CMD_EXIT_META_EXEC    	=80;
const PROGMEM uint8_t CMD_CONT_META_EXEC    	=81;
const PROGMEM uint8_t CMD_GIVE_EEPROM	     	=90;		
const PROGMEM uint8_t CMD_SAVE_EEPROM	     	=91;		

// STATES RETURNED FROM SLAVE TO MASTER
const PROGMEM uint8_t STATE_LOCKED 	  	=100;
const PROGMEM uint8_t STATE_UNLOCKED 		=110;
const PROGMEM uint8_t IN_POSITION 		=111;
const PROGMEM uint8_t IS_MOVING 		=112;
const PROGMEM uint8_t TALKED_DONE		=113;
const PROGMEM uint8_t IS_TALKING 		=114;
const PROGMEM uint8_t STATE_READY		=115;
const PROGMEM uint8_t GP_SET			=116;
const PROGMEM uint8_t META_FINISHED		=117;
const PROGMEM uint8_t META_REPEAT		=118;
const PROGMEM uint8_t BLOCKED			=119;
const PROGMEM uint8_t STATE_ERROR		=120;
const PROGMEM uint8_t HOME_FINISHED		=121;

const PROGMEM uint8_t OPERATION_HOME		=150;
const PROGMEM uint8_t OPERATION_META		=151;

const PROGMEM uint8_t slave_is_at_c1		=201;
const PROGMEM uint8_t slave_is_at_c2		=202;
const PROGMEM uint8_t slave_is_at_c3		=203;
const PROGMEM uint8_t slave_is_at_c4		=204;
const PROGMEM uint8_t slave_is_at_c5		=205;	
const PROGMEM uint8_t slave_is_at_c6		=206;
const PROGMEM uint8_t slave_is_at_c7		=207;
const PROGMEM uint8_t slave_is_at_c8		=208;
const PROGMEM uint8_t slave_is_at_c9		=209;	
const PROGMEM uint8_t slave_is_at_c10		=210;
const PROGMEM uint8_t slave_is_at_c11		=211;
const PROGMEM uint8_t slave_is_at_c12		=212;
const PROGMEM uint8_t slave_is_at_c13		=213;
const PROGMEM uint8_t slave_is_at_c14		=214;
const PROGMEM uint8_t slave_is_at_c15		=215;

// ============================================================================

const PROGMEM uint8_t MASTER_DELAY_TIME		=100;
const PROGMEM uint8_t SLAVE_DELAY_TIME		=5;
const PROGMEM uint8_t SLAVE_RESPONSE_TIME	=20;

// ============================================================================

const PROGMEM uint8_t GEAR_FACTOR_PSEUDO1			=60;
const PROGMEM uint8_t SPR_PSEUDO1				=800;			// Depends on driver dip switches
const PROGMEM uint16_t METAMORPHOSIS_Ci_STEPS			=1715;		// calculated using formula: (step_angle_deg/360) * spr * GEAR_FACTOR for 15 holes
const PROGMEM uint16_t HOMING_CALIBRATION_LIMIT			=250;


#endif
