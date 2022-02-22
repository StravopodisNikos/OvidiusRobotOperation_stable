// Control Table Used Items Address
#define ADDR_PRO_TORQUE_ENABLE                  512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION                  564
#define ADDR_PRO_GOAL_VELOCITY                  552
#define ADDR_PRO_PROF_ACCEL                     556
#define ADDR_PRO_PROF_VEL                       560
#define ADDR_PRO_PRESENT_CURRENT                574
#define ADDR_PRO_PRESENT_VELOCITY               576
#define ADDR_PRO_MOVING                         570
#define ADDR_PRO_MOVING_STATUS                  571
#define ADDR_PRO_PRESENT_POSITION               580
#define ADDR_PRO_ACCEL_LIMIT                    40
#define ADDR_PRO_VEL_LIMIT                      44
#define ADDR_PRO_MAX_POS_LIMIT                  48
#define ADDR_PRO_MIN_POS_LIMIT                  52
#define ADDR_PRO_LED_BLUE                       515
#define ADDR_PRO_LED_GREEN                      514
#define ADDR_PRO_LED_RED                        513
#define ADDR_PRO_RETURN_DELAY_TIME              9
#define ADDR_PRO_BAUDRATE                       8
#define ADDR_PRO_ID                             7
#define ADDR_PRO_SECONDARY_ID                   12
#define ADDR_PRO_FIRMWARE_VER                   6
#define ADDR_PRO_MODEL_INFO                     2
#define ADDR_PRO_POSITION_P_GAIN				532
#define ADDR_PRO_POSITION_I_GAIN				530
#define ADDR_PRO_POSITION_D_GAIN				528

// Data Byte Length of default control table items
#define LEN_PRO_TORQUE_ENABLE                   1
#define LEN_PRO_GOAL_POSITION                   4
#define LEN_PRO_GOAL_VELOCITY                   4
#define LEN_PRO_PROF_ACCEL                      4
#define LEN_PRO_PROF_VEL                        4
#define LEN_PRO_PRESENT_VELOCITY                4
#define LEN_PRO_PRESENT_POSITION                4
#define LEN_PRO_ACCEL_LIMIT                     4
#define LEN_PRO_VEL_LIMIT                       4
#define LEN_PRO_MAX_POS_LIMIT                   4
#define LEN_PRO_MIN_POS_LIMIT                   4
#define LEN_PRO_PRESENT_CURRENT                 2
#define LEN_PRO_MOVING                          1
#define LEN_PRO_MOVING_STATUS                   1 
#define LEN_PRO_LED                             1
#define LEN_PRO_RETURN_DELAY_TIME               1
#define LEN_PRO_BAUDRATE                        1
#define LEN_PRO_ID                              1
#define LEN_PRO_SECONDARY_ID                    1
#define LEN_PRO_FIRMWARE_VER                    1
#define LEN_PRO_MODEL_INFO                      4
#define LEN_PRO_POSITION_P_GAIN					2
#define LEN_PRO_POSITION_I_GAIN					2
#define LEN_PRO_POSITION_D_GAIN					2

// Data Byte Length of custom command items for indirect addressing
#define LEN_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED           15                   // 4*3 for pos/vel/accel and 3*1 for leds
#define LEN_PRO_INDIRECTDATA_FOR_READ_PP_MV                 5                    // 4 for present position and 1 for dxl_moving
#define LEN_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL	    20	
#define LEN_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED		    3	

#define ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED       168                  // EEPROM region for free Indirect Address for PH54!!!
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV             198                  // ...
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL    208
#define ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_RGB_LED          248 

#define ADDR_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED          634                  // RAM region for the corresponding free Indirect Data for PH54!!!
#define ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV                649                  // ...
#define ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL       654
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED             674

// Protocol version
#define DXL_PROTOCOL_VERSION                        2.0                 // Protocol Version of Dunamixels used

// Serial Communication Properties
#define SERIAL_BAUDRATE1			57600
#define SERIAL_BAUDRATE2			115200
#define SERIAL_BAUDRATE3                        1000000

#define DXL_BAUDRATE0                           9600
#define DXL_BAUDRATE1                           57600
#define DXL_BAUDRATE2                           115200
#define DXL_BAUDRATE3                           1000000
#define DXL_BAUDRATE4                           2000000
#define DXL_BAUDRATE5                           3000000
#define DEVICENAME                              "/dev/ttyACM1"
#define CMD_SERIAL                              Serial
#define ESC_ASCII_VALUE                         0x1b

#define DXL_RESOLUTION				501923	

// Metamorphic Structure Info	
#define nPseudoJoints				1	


// Definitions for PseudoSPIcomm library

// PINS CONFIGURATION MASTER &
// PINS CONFIGURATION SLAVE
#define MOSI_NANO 			11
#define MISO_NANO 			12
#define SCK_NANO 			13
#define MOSI_MASTER 			51			// 11 for UNO 51 for MEGA
#define MISO_MASTER 			50			// 12         50
#define SCK_MASTER			52			// 13         52
#define greenLED_Pin 			A2		
#define blueLED_Pin 			A1		
#define redLED_Pin			A3			// 6	
#define dirPin_NANO			7
#define stepPin_NANO			8
#define enabPin_NANO			6
#define hallSwitch_Pin			3			// used for homing hall sensor
#define pseudoLimitSwitch_Pin 		2
#define RELAY_lock_Pin2			A5
#define RELAY_lock_Pin			A4

// SS PINS MASTER & PSEUDOS
#define SSpinMaster1			5
#define SSpinMaster2			6
#define SSpinMaster3			9

#define SSpinPseudo1			5
#define SSpinPseudo2			5
#define SSpinPseudo3			5

#define PSEUDO1_ID 			1
#define PSEUDO2_ID 			2
#define PSEUDO3_ID 			3
#define PSEUDO4_ID 			4
#define PSEUDO5_ID 			5
#define PSEUDO6_ID 			6

#define ADDRESS_WIDTH			6

#define PSEUDO_NUMBER1 			1	
#define PSEUDO_NUMBER2			2
#define PSEUDO_NUMBER3			3
#define PSEUDO_NUMBER4			4
#define PSEUDO_NUMBER5			5
#define PSEUDO_NUMBER6			6

// ============================================================================
// SPI COMMUNICATION BYTES => MUST BE UNIQUE
// ============================================================================

// COMMANDS SENT -> in single byte transfer each ci is regarded as command!
// ci's: these are the bytes given to setGoalPositionMaster() for desired anatomy Metamorphosis
#define junk_command		0

#define c1			1
#define c2			2
#define c3			3
#define c4			4
#define c5			5	
#define c6			6
#define c7			7
#define c8			8
#define c9			9	
#define c10			10
#define c11			11
#define c12			12
#define c13			13
#define c14			14
#define c15			15
#define wrong_ci		16
#define home_ci			8

// COMMANDS FROM MASTER TO SLAVE
#define CMD_LOCK		20
#define CMD_UNLOCK	    	21
#define CMD_SGP	  		30		// In single byte transfer is overrided
#define CMD_MOVE		40
#define CMD_STOP		41		// Danger Stop Event!
#define CMD_HOME		42
#define CMD_PRE_HOME        	43	 
#define CMD_CONNECT		60
#define CMD_GIVE_IS		70
#define CMD_GIVE_CS		71
#define CMD_GIVE_CP		72
#define CMD_EXIT_META_EXEC    	80
#define CMD_CONT_META_EXEC    	81
#define CMD_GIVE_EEPROM	     	90		
#define CMD_SAVE_EEPROM	     	91		

// STATES RETURNED FROM SLAVE TO MASTER
#define STATE_LOCKED 	  	100
#define STATE_UNLOCKED 		110
#define IN_POSITION 		111
#define IS_MOVING 		112
#define TALKED_DONE		113
#define IS_TALKING 		114
#define STATE_READY		115
#define GP_SET			116
#define META_FINISHED		117
#define META_REPEAT		118
#define BLOCKED			119
#define STATE_ERROR		120
#define HOME_FINISHED		121

#define OPERATION_HOME		150
#define OPERATION_META		151

#define slave_is_at_c1		201
#define slave_is_at_c2		202
#define slave_is_at_c3		203
#define slave_is_at_c4		204
#define slave_is_at_c5		205	
#define slave_is_at_c6		206
#define slave_is_at_c7		207
#define slave_is_at_c8		208
#define slave_is_at_c9		209	
#define slave_is_at_c10		210
#define slave_is_at_c11		211
#define slave_is_at_c12		212
#define slave_is_at_c13		213
#define slave_is_at_c14		214
#define slave_is_at_c15		215

// ============================================================================

#define MASTER_DELAY_TIME	100
#define SLAVE_DELAY_TIME	5
#define SLAVE_RESPONSE_TIME	20

// ============================================================================
#define FIRST_HIT					1
#define GEAR_FACTOR_PSEUDO1				60
#define GEAR_FACTOR_PSEUDO2				80
#define SPR_PSEUDO1					800	// 
#define SPR_PSEUDO2					400	// 
#define METAMORPHOSIS_Ci_STEPS				1715	// [CHINA] calculated using formula: (step_angle_deg/360) * spr * GEAR_FACTOR for 15 holes, step_angle_deg = 90/7
#define METAMORPHOSIS_Ci_STEPS2				1145	// [MOTOVARIO] calculated using formula: (step_angle_deg/360) * spr * GEAR_FACTOR for 15 holes
#define HOMING_CALIBRATION_LIMIT			250     // [HALL]
#define HOMING_CALIBRATION_LIMIT2			10	// [SWITCH]	
#define BREAK_FREE_STEPS				300
#define FIXED_PSEUDO_STEP_DELAY 			1000 	// [micros]
#define PSEUDO_MOVE_LED_BLINK_INTERVAL			500  	// [millis]
#define PSEUDO_HOME_LED_BLINK_INTERVAL			250  	// [millis]
#define MAX_MOVE_STEPS					25725	// 1715 * 15
// ============================================================================
#define CONNECT_INTERVAL	500
#define ERROR_CONNECT_INTERVAL	500
#define START_INTERVAL		1000
#define BLOCKED_INTERVAL	400
#define MOVING_INTERVAL	        500 // this is non-blocking, so no times are specified 
// ============================================================================
#define CONNECT_TIMES		2
#define ERROR_CONNECT_TIMES	2
#define START_TIMES		2
#define FINISH_TIMES		2
#define BLOCKED_TIMES		3

// ============================================================================
// EEPROM AREA ADDRESSES [0~255] -> all moved to ovidius_robot_controller_eeprom_addresses.h created 17-2-21
/*
#define ID_EEPROM_ADDR		0		// int
#define MAX_POS_LIM_ADDR	10		// float
#define MIN_POS_LIM_ADDR	20		// float
#define STEP_ANGLE_ADDR		30		// float
#define CS_EEPROM_ADDR		40		// byte			// Always updated with homing
#define CP_EEPROM_ADDR		50		// byte			// 50 must be garbage now(changed in 30.6.2020)
#define CD_EEPROM_ADDR		60		// uint32_t		// ...	
*/
