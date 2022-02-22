/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 2 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels of a serial metamorphic
 *   manipulator. 
 *
 *   ==> In order to successfully compile, the DUE BOARD must be selected in Tools, and the
 *       DXL SHIELD must be installed. Stable UPLOAD only on WINDOWS PC + Arduino IDE 1.8.15.
 *       The ROBOTIS Arduino packages MUST be installed.
 *   
 *   [21-2-22] STABLE VERSION FOR ACTIVE OPERATION. FEATURES:
 *   1.  Executes fixed vel profile(stepper). Accepts C-space commands. Fixed bug for identical {qi,qi+1} for J1. 
 *   2.  C-Space points are given in code section "INCLUDE TASK DEFINITION FILES". These files should be located 
 *       in folder: "C:\Users\Nikos\Documents\Arduino\libraries\meta_controller_configuration"
 *   3.  After file is included, search for <TASK-SET> in TABS: metaOperationMainDueDevel,traj_def
 *   4.  In order to UPLOAD, connect only PROGRAM USB port. After flash write+verify connect NATIVE, press RESET 
 *       button in DUE, select it in the Port menu and open Serial Monitor.
 *   5.  During UPLOAD the switch in DXL SHIELD must be set to UP position. After upload complete, switch to DXL.
 *   6.  For LOG, 4 sensors (position,velocity,3Dforce,current) are used. To enable user set corresponding 0 or 1.
 *   7.  Currently, this file and the corresponding libraries are set for 3DoF active operation.
 *       In order to change must search <DOF-SET> here and in libraries: 
 *       CustomStepperOvidiusDueShield, DynamixelProPlusOvidiusShield, StepperMotorSettings.
 *       Before software changes the Dynamixels should be configured using DynamixelWizard2.0 software.
 *   8.  In order to work, Control Table Items of DXLS should be configured:   
 *       [i]   ID, currently {1,2}
 *       [ii]  Baudrate, ONLY 115200 for serial comm with DUE BOARD.
 *       [iii] Set the Pos,Vel,Cur Limits same as the 2 Dxls already installed.
 *   9.  User should ONLY provide the byte numbers asked from menu prompts!   
 *   10. For optimal task execution, select only 1 log for each run! Specially when >2 Dxls are used. 
 *   
 *   [21-2-22] NEXT UPDATES:
 *   1. Integrate MCU for Joint 1 Stepper control and remove stepping loop from the DUE main processor
 *   2. Develop <Tool-Space> operation.
 *   3. Develop Constrained motion for task execution, defined in tool space.
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  March 2021
 */

/* 
 *  CONFIGURE DEBUG_SERIAL PORT COMMUNICATION > THIS IS COMPILED
 *  AND TESTED FOR ARDUINO DUE ONLY.
 */
#if defined(ARDUINO_AVR_UNO) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial       
  #define DEBUG_SERIAL soft_serial  
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(11, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#else defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

/*
 * INCLUDE LIBRARIES
 */
// Core Libraries for Robot Motors Driving
#include "DynamixelProPlusOvidiusShield.h"  
#include "CustomStepperOvidiusDueShield.h"            // This is the renamed and Due-modified version of the CustomStepperOvidiusShield
// 3rd party libraries
#include <Dynamixel2Arduino.h>
// Auxiliary Libraries
#include <definitions.h>                              // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                                 // Includes motor IDs as set using Dynamixel Wizard
#include <position_pid_controller_definition.h>       // Includes available Gain parameters  
#include <contolTableItems_LimitValues.h>             // Limit values for control table controlTableItems_LimitValues
#include <utility/StepperMotorSettings.h>             // Includes Stepper Motor/Driver pin StepperMotorSettings
#include <utility/stepper_led_indicators.h>           // Includes Stepper Motor/Driver pin StepperMotorSettings
#include <utility/dynamixel_settings.h>               // Includes settings for dynamixels
#include <utility/dynamixel_led_indicators.h>         // Includes settings for dynamixels 
//#include <led_indicators.h>                         // Led color/blinks for robot monitoring
#include <utility/OvidiusSensors_config.h>            // Configuration for sensors used in robot
#include <utility/OvidiusSensors_debug.h>             // Debug codes for sensor functions
#include <TimeLib.h>                                  // Timing functions

/*
 *  INCLUDE TASK DEFINITION FILES
 */
#include <task_definitions.h>                         // Includes task points for execution of tasks: OLD MBS MUST BE RE-EXECUTED
//#include <task3_02z_definitions.h>                  // Sim data @[4-12-21]
//#include <task3_-02z_definitions.h>                 // Sim data @[2-12-21]
//#include <task3_02x_definitions.h>                  // Sim data @[2-12-21]   
//#include <task2_R075_definitions.h>                 // Sim data @[2-12-21]
#include <task1_mbsOptimizedAnatomies.h>              // Sim data @[6-12-21]

#include <task4_ikp_rehabilitation_task.h>            // Sim data @[4-2-22]
#include <task4_opt_task_placement.h>                 // Sim data @[8-2-22]
#include <task4_ikp_rect_task.h>                      // Sim data @[19-2-22]
/* 
 *  NAMESPACE OF USED CLASSES
 */
using namespace std;

/*
 * USER DEBUG_SERIAL INPUT VARIABLES
 */
byte YES_b     = 1;
byte NO_b      = 0;
byte p2pcsp_b  = 2;
byte trajcsp_b = 3;
byte trajcsp_c = 4;
byte home_b    = 5;
byte grip_b    = 6;
byte access_PID_gains_b = 7;
byte read_session_data_b = 8;       // not used, must change data type


// Auxiliary variables for DEBUG_SERIAL inputs
unsigned char error_code_received;  // previous int
byte user_input;
byte file_id;
//bool MENU_EXIT;
//bool ACCESS_EEPROM;
unsigned char p2p_index;
unsigned char traj_index;
int traj_tot_points;
/*
 * MAIN FLAGS USED TO CONTROL LOOPS
 */
bool END_P2PCSP             = false;                      
bool END_TRAJCSP            = false;
bool END_HOME               = false;
bool END_READ_SESSION_DATA  = false;
bool END_TRAJCSP2           = false;
bool END_GRIP               = false;
bool END_ACCESS_PID         = false;                       
//bool END_FORCE;
//bool END_ACCEL;
/*
 * SECONDARY FLAGS USED TO CONTROL LOOPS
 */
//bool p2pcspExecution;
//bool trajcspExecution;
//bool homeExecution;
//bool forceExecution;
//bool accelExecution;  

/*
 * Configure IDs for Dynamixels & NEMA34 Stepper Motor -> MUST align with the IDs configured at EEPROM of each module!
 */
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID};  // <DOF-SET> Define DXLi_IDs in file: C:\Users\Nikos\Documents\Arduino\libraries\meta_controller_configuration\motorIDs.h

/*
 * Configure Stepper Motor variables defined in CustomStepperOvidiusShield.h
 */
bool homingSwitchActivated = true;          // NC connection in trigger(despite signed as NO)
//bool limit1SwitchActivated = true;        // Green LED is OFF at default(THIS WAS GREEN ON/true before)
//bool limit2SwitchActivated = false;       // Green LED is OFF at default

volatile byte currentDirStatus;
uint32_t RELATIVE_STEPS_2_MOVE;
unsigned char stp_error;
volatile bool KILL_MOTION_DUMMY = false;
/*
 * Declare extern variables defined in DynamixelProPlusOvidiusShield.h
 */
uint8_t  dxl_error; 
int32_t  dxl_present_position[sizeof(dxl_id)];
int32_t  dxl_present_velocity[sizeof(dxl_id)];
int16_t  dxl_present_current[sizeof(dxl_id)];   // must check for the data type in e-Manual 
int32_t  dxl_goal_position[sizeof(dxl_id)];
int32_t  dxl_prof_vel[sizeof(dxl_id)];
int32_t  dxl_prof_accel[sizeof(dxl_id)];
uint8_t  dxl_moving[sizeof(dxl_id)];
int16_t  dxl_pKp[sizeof(dxl_id)];
int16_t  dxl_pKi[sizeof(dxl_id)];
int16_t  dxl_pKd[sizeof(dxl_id)];
//int32_t   dxl_vel_limit[sizeof(dxl_id)];
//int32_t dxl_accel_limit[sizeof(dxl_id)];

/*
 * Timing variables 
 */
 bool STOP_WAITING = false;
 int total_time_trying = 0;         // used to count time for opening comm objects. if exceed timeout limit, aborts opening
 bool TIME_NOT_SET     = true;      // used in TimeLib
 unsigned long time_now_micros;     // used for clocking functions
 unsigned long time_now_millis;     // used for clocking functions
 unsigned long p2p_duration;        // used for measuring total task execution
/*
 * Configure Stepper/Dynamixel settings
 */
float currentConfiguration[nDoF];
float desiredConfiguration[nDoF];
float p2pcsp_joint_velocities[nDoF];
float p2pcsp_joint_currents[nDoF];
float joint_velocities_limits[] = {1, 1.57, 1.57};     // [rad/sec] must be implemented in EEPROM Stepper+Dxl!
float p2pcsp_joint_accelerations[nDoF];
float joint_accelerations_limits[] = {20, 87, 87};       // [rad/sec^2] must be implemented in EEPROM Stepper+Dxl!
float p2pcsp_Texec;
float p2pcsp_Ta;

/* 
 * Declare global variables used in CustomStepperMetamorphicManipulator 
 */
bool return_function_state = false;
float currentAbsPos_double;
float goalAbsPos_double;
float VelocityLimitStp;
float AccelerationLimitStp;
float MaxPosLimitStp;
bool finalTargetSet;

/* 
 *  SYNC WRITE structure objects for sending Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sw_data_t_gp sw_data_array_gp[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pv sw_data_array_pv[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pa sw_data_array_pa[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pKp sw_data_array_pKp[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pKi sw_data_array_pKi[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pKd sw_data_array_pKd[DXL_MOTORS]; // this type of array should be passed to the function

/* 
 *  SYNC READ structure objects for receiving Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sr_data_t_pp  sr_data_array_pp[DXL_MOTORS]; 
sr_data_t_pv  sr_data_array_pv[DXL_MOTORS]; 
sr_data_t_mov sr_data_array_mov[DXL_MOTORS]; 
sr_data_t_pc  sr_data_array_pc[DXL_MOTORS]; 
sr_data_t_pKp  sr_data_array_pKp[DXL_MOTORS]; 
sr_data_t_pKi  sr_data_array_pKi[DXL_MOTORS]; 
sr_data_t_pKd  sr_data_array_pKd[DXL_MOTORS]; 

/*
 *  Create struct objects for communicating between CustomStepper+DynamixelOvidius libraries
 */
DXL_PP_PACKET  dxl_pp_packet,  *PTR_2_dxl_pp_packet;
DXL_PV_PACKET  dxl_pv_packet,  *PTR_2_dxl_pv_packet; 
DXL_MOV_PACKET dxl_mov_packet, *PTR_2_dxl_mov_packet; 
DXL_PC_PACKET  dxl_pc_packet,  *PTR_2_dxl_pc_packet;
DXL_PKP_PACKET dxl_pKp_packet, *PTR_2_dxl_pKp_packet;
DXL_PKI_PACKET dxl_pKi_packet, *PTR_2_dxl_pKi_packet;
DXL_PKD_PACKET dxl_pKd_packet, *PTR_2_dxl_pKd_packet;

/* 
 *  Create object for handling motors
 */
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelProPlusOvidiusShield meta_dxl(dxl_id), *PTR_2_meta_dxl; // Object of custom class to access custom functions for Ovidius manipulator specific 
CustomStepperOvidiusDueShield stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, HOME_TRIGGER_SWITCH, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, RED_LED_PIN, GREEN_LED_PIN, BLUE_LED_PIN, SPR_1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

/*
 *  Create object for trajectory points in C-space
 */


/*
 *  [22-11-21] Points extracted for [Task/Robot] = [-0.15 z / 0.4kg/ massage1]
 *  ANATOMY c_i->deg formula: deg2rad(90/7) * (floor(c_i)-1) + (-1.5708)
 */
// [28-11-21] All trajectory points are loaded from task definition files
// [21-2-22]  Must change the idexes according to traj defined in traj_def tab! <TASK-SET>
traj_def_struct trajectories[2];    // index must be updated when new trajectories are given in .h
traj_def_struct * ptr2traj_struct;
traj_def_struct2 trajectories2[25];   // index must be updated when new trajectories are given in .h
traj_def_struct2 * ptr2traj_struct2;
/* 
 *  Create object for handling sensors+tools
 */
 // DataLogging
uint32_t DUE_CLOCK_Hz = DUE_CLOCK_Hz_MAX;
bool SD_INITIALIZED = false;
tools::dataLogger RobotDataLog, *PTR2RobotDataLog;
debug_error_type data_error;
bool update_sensor_measurements[TOTAL_SENSORS_USED];
//File root,dir, *PTR2ROOT, *ptr2dir;
//File POS_LOG, VEL_LOG,FORCE_LOG;
File LOGFILES[TOTAL_SENSORS_USED]; // 0->pos, 1->vel, 2->force 3->current -> this must be initialized @ start of task execution functions(i.e. p2pcsp_sm)
sensors::sensors_list FORCE_FOLDER = sensors::FORCE_3AXIS;
sensors::sensors_list POS_FOLDER   = sensors::JOINT_POS;
sensors::sensors_list VEL_FOLDER   = sensors::JOINT_VEL;
sensors::sensors_list CUR_FOLDER   = sensors::CURRENT_JOINT1;
// Create structs for logging sensor data to arrays during real time [9-4-21]
tools::real_time_log_struct SENSOR_LOG_STRUCTS[TOTAL_SENSORS_USED];
 // Gripper
Servo OvidiusGripperServo, *ptr2OvidiusGripperServo;
 // ForceSensor
//HX711 ForceSensorX, *ptr2ForceSensorX;
HX711 ForceSensorHX711[num_FORCE_SENSORS];
//HX711 SingleForceSensorHX711, *ForceSensorHX711;          // ONLY FORCE Z-AXIS!
long ForceSensorAxesOffest[num_FORCE_SENSORS];
float manual_calibration_scale_factors[num_FORCE_SENSORS];
debug_error_type sensor_error;
// Gripper Tool Globals
int offset_analog_reading;                                // Returned from setupGripper(), can be executed at setup
unsigned long grasp_force_limit_newton = MIN_GRASP_FORCE; // Max value is 10[N], must be saved at EEPROM
tools::gripper_states GripperCurrentState;
sensors::force_sensor_states ForceCurrentState;
tools::gripper OvidiusGripper(GRIPPER_SERVO_PIN,FSR_ANAL_PIN1);
//DataLogging globals
char LOG_FILES[TOTAL_SENSORS_USED];
char SESSION_MAIN_DIR[SESSION_DIR_CHAR_LEN];
//SESSION_MAIN_DIR_FORCE,SESSION_MAIN_DIR_POS,SESSION_MAIN_DIR_VEL -> SESSION_MAIN_DIR_SENSORS
char SESSION_MAIN_DIR_POS[SENSOR_DIR_CHAR_LEN];
char SESSION_MAIN_DIR_VEL[SENSOR_DIR_CHAR_LEN];
char SESSION_MAIN_DIR_FORCE[SENSOR_DIR_CHAR_LEN];
char SESSION_MAIN_DIR_CUR[SENSOR_DIR_CHAR_LEN];
char LOG_POS[LOG_FILES_DIR_CHAR_LEN];
char LOG_VEL[LOG_FILES_DIR_CHAR_LEN];
char LOG_FORCE[LOG_FILES_DIR_CHAR_LEN];
char LOG_CUR[LOG_FILES_DIR_CHAR_LEN];
// array of pointers used here!
char * LOG_FILE_PATH[TOTAL_SENSORS_USED]; 
//char * LOG_FILE_PATH = (char *)malloc(sizeof(char)*TOTAL_SENSORS_USED); // changes each time create_logfiles() is called(i.e. for each task execution) - must be expanded to nSensors!
const char * PTRS2SENSOR_DIRS[TOTAL_SENSORS_USED];                      // implemented only once in session_mkdir() - must be expanded to nSensors!
const char * FINAL_ACCESSED_FILES[TOTAL_SENSORS_USED];
char read_data_buffer[100];
int file_read_buffer_size = 100;

// 3axis force sensor was commented out because test uses single sensor
//sensors::force3axis SingleForceSensor(DOUT_PIN_Z, SCK_PIN_Z), *ForceSensor; // ONLY FORCE Z-AXIS! 
sensors::force3axis ForceSensor[num_FORCE_SENSORS] = {
    sensors::force3axis(DOUT_PIN_X, SCK_PIN_X),   //ForceSensor[0] -> ForceSensorX
    sensors::force3axis(DOUT_PIN_Y, SCK_PIN_Y),   //ForceSensor[1] -> ForceSensorY
    sensors::force3axis(DOUT_PIN_Z, SCK_PIN_Z),   //ForceSensor[2] -> ForceSensorZ
}; 
// Joint1 INA219 Current sensor
float joint1_current_measurement;
sensors::current_packet CUR_DATA_PKG, *PTR_2_CUR_DATA_PKG;
sensors::currentSensor joint1_cur_sensor, *ptr2joint1_cur_sensor;
//Adafruit_INA219 ina219, * ptr2ina219; [24-3-21] Will use only ACS712 current module

/*
 * ADAFRUIT IMU sensor 
// [21-3-21] was commented out because of memory issues in MEGA BOARD
sensors::imu9dof SingleIMUSensor(GYRO_RANGE_250DPS, ACCEL_RANGE_2G, FILTER_UPDATE_RATE_HZ, 0x0021002C, 0x8700A, 0x8700B), *IMUSensor;
Adafruit_FXAS21002C gyro;
Adafruit_FXOS8700 accelmag;
Adafruit_Madgwick filter;  // added 2/3/21 - original adafruit
//SF fusion;           // removed 2/3/21 - will implement original adafruit class
Adafruit_Sensor_Calibration_EEPROM adafr_calibr;  
sensors_event_t gyro_event, accel_event, magnet_event;
float pitch, roll, yaw;
sensors::imu_packet IMU_DATA_PKG, *PTR_2_imu_packet;
sensors::imu_sensor_states ImuCurrentState;
 */

/*
 * SETUP
 */
void setup() 
{
  DEBUG_SERIAL.begin(SERIAL_BAUDRATE3);            
  while(!DEBUG_SERIAL);
  delay(500);
  DEBUG_SERIAL.print(F("STARTING DEBUG PC SERIAL BUS..."));
  DEBUG_SERIAL.println(F("SUCCESS"));

  // INITIALIZE SD CARD // -> [28-8-21] DEPRECATED -> GAMIETAI H SD
  /*
  unsigned long started_sd_initialization = millis();
  while ( (!SD_INITIALIZED) && (total_time_trying < SD_INIT_TIMEOUT_MILLIS) )
  {
    if (!SD.begin(SD_CARD_CS_PIN))
    {
       DEBUG_SERIAL.println(F("[ SETUP ] SD INITIALIZATION FAILED"));
    }
    else
    {
      SD_INITIALIZED = true;
      DEBUG_SERIAL.println(F("[ SETUP ] SD INITIALIZATION SUCCESS"));
    }
    total_time_trying = millis() - started_sd_initialization;  
  }
  */
  
  DEBUG_SERIAL.print(F("STARTING DYNAMIXEL SERIAL BUS..."));
  dxl.begin(DXL_BAUDRATE2);                       
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  DEBUG_SERIAL.println(F("SUCCESS"));
  
  // SET TRIGGERS PINMODES
  pinMode(HOME_TRIGGER_SWITCH, INPUT_PULLUP);           // HOME TRIGGER SWITCH
  pinMode(HALL_SWITCH_PIN2, INPUT_PULLUP);              // LIMIT HALL SENSOR2 
  pinMode(HALL_SWITCH_PIN3, INPUT_PULLUP);              // LIMIT HALL SENSOR3
  
  // SET EXTERNAL INTERRUPTS: IF DXL SHIELD USED ->
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN2), changeStepperDirInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN3), changeStepperDirInterrupt1, RISING);
  // IF DXL SHIELD USED <-
  
  delay(500);
  DEBUG_SERIAL.println(F(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... "));
  delay(500);

  // SETUP TIME CLOCK FOR FOLDER MANAGEMENT OF DATA LOGGING
  /*
  setSyncProvider(requestSync);
  DEBUG_SERIAL.println(F("[ SETUP ] SET ROBOT TIME STARTED - INSERT TIME:"));
  DEBUG_SERIAL.parseInt();
  while (TIME_NOT_SET)
  {
    if (DEBUG_SERIAL.available())
    {
      processSyncMessage();
      TIME_NOT_SET = false;
      DEBUG_SERIAL.println(F("[ SETUP ] SET ROBOT TIME FINISHED"));
    }      
  }; 
  */
  // MAKE SESSION DIRECTORIES AND SENSOR LOGFILES // -> [28-8-21] DEPRECATED -> GAMIETAI H SD
  /*
  DEBUG_SERIAL.println(F("[ SETUP ] BUILDING SESSION DIRECTORIES"));
  session_mkdir();
  create_logfiles(); // [21-3-21] SUCCESSFULLY EXECUTED @DUE BOARD -> NOW CALLED IN TASK EXECUTION INOS ONLY
                     // [23-3-21] IN ORDER TO EXECUTE ALONG DXLS BAUDRATE(9600) CHANGED FILE_WRITE
                     // [26-3-21] Unitl dynamic memory re-allocation is mplemented, only a single log file for each sensor will be made for the whole session
                     // [25-8-21] doesn't work. Gamhmenh SD.
  delay(500);
  */
  
  // SETUP DEFAULT ACTIONS
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("PINGING CONNECTED DYNAMIXELS PRO+"));
  ping_motors();
  delay(500);
  
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("INITIALIZING STEPPER MOTOR GLOBAL VARIABLES"));
  init_robot_globals(); // <- this to be used!
  delay(500);
  
  // initialize data packets for libraries communication
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("BUILDING DATA PACKETS FOR LIBRARIES DATA EXCHANGE"));
  setup_libraries_packets();
  delay(500);
  
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("EXTRACTING CURRENT CONFIGURATION"));
  read_current_configuration();
  delay(500);

  // [2-8-21] Initialize the trajectories user-defined in traj_def();
  traj_def();
     
  // SETUP USER-SELECTED ACTIONS 
  bool stop_setup = false; // flag for setup menu robot loop

  while(!stop_setup) //wh1
  {
    /*
     * I. SLOW MOTOR HOMING
     */
    DEBUG_SERIAL.println(F("[ SETUP ] HOME MOTORS SLOW?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) 
    {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SLOW HOMING MOTORS"));

      slow_home_motors();
    }
    
    /*
     * II. TEST GRIPPER
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP GRIPPER?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      setup_gripper();
    }

    /*
     * III. TEST FORCE SENSOR
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP FORCE SENSOR?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      setup_force_sensor();
    }

    /*
     * IV. TEST CURRENT SENSOR
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP CURRENT SENSOR?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      setup_current_sensor();
    }

    /*
     * V. CHECK DYNAMIXELS ARE NOT MOVING
     */
    DEBUG_SERIAL.println(F("[ SETUP ] CHECK DYNAMIXEL MOVING?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
        if( stp.getDynamixelsMotionState(&meta_dxl, PTR_2_dxl_mov_packet, &stp_error) )
        {
            DEBUG_SERIAL.println(F("[  INFO  ] DYNAMIXELS NOT MOVING [  SUCCESS ]"));
            DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(stp_error);
        }
        else
        {
            DEBUG_SERIAL.println(F("[  INFO  ] DYNAMIXELS NOT MOVING [  FAILED ]"));
            DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(stp_error);              
        }
    }

    /*
     * VI. SET DYNAMIXELS POSITION CONTROLLER GAINS
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SET DYNAMIXELS POSITION CONTROLLER GAINS?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);

    DEBUG_SERIAL.println(F("[  INFO  ] READING CURRENT DYNAMIXELS POSITION CONTROLLER GAINS..."));
    read_PosControllerGains();
    
    if( user_input == YES_b ) 
    {
      change_PosControllerGains();

      read_PosControllerGains();
    }
    
    /*
     * FINISH SETUP
     */
    DEBUG_SERIAL.println(F("[ SETUP ] EXIT SETUP?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      stop_setup = true;
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETUP FINISHED"));
    }
    else
    {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETUP MENU WILL REPEAT"));
    }
    
  } // wh1

} // END SETUP

/*
 * MAIN LOOP
 */
void loop() {
  /*
   * I. User must specify operating mode
   */
  DEBUG_SERIAL.println(F("Set ROBOT OPERATION MODE [2: P2P, 3: RECT-TRAJ, 4: CIRC-TRAJ, 5:HOME, 6: GRIPPER,  7: ACCESS POS-PID 8: READ DATA ]:"));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.print(user_input);

  /*
   * II. <p2pcsp>
   */
   if( user_input == p2pcsp_b  ) //start->if1
   {
   
   while( (!END_P2PCSP) ) //start->wh1
   {
      stp.setStepperLed(stepper_entered_p2pcsp);
      delay(1000);
      
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN CONFIGURATION SPACE P2P EXECUTION..."));

      // II.1 Display to user the current configuration
      read_current_configuration(); 
      
      // II.2 Specify desired configuration
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SPECIFY TASK POINT INDEX"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      p2p_index = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(p2p_index);
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SELECTED CONFIGURATION SPACE POINT:")); 
      for (size_t i = 0; i < nDoF; i++)
      {
        DEBUG_SERIAL.print(F(" [ JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(p2p_list[p2p_index][i],4);

        desiredConfiguration[i] = p2p_list[p2p_index][i];
      }
      
      // II.3 SIMPE P2P EXECUTION FUNCTION TO BE PLACED HERE!
      p2pcsp_Texec = p2p_dur[p2p_index];
      p2pcsp2_sm(currentConfiguration, desiredConfiguration, p2pcsp_Texec);

      /*
       * FINISH p2pcsp
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <p2pcsp>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_P2PCSP    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<p2pcsp> FINISHED"));
      }
      else
      {
        END_P2PCSP    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<p2pcsp> WILL REPEAT"));
      } 

   }//end->wh1
   
   }//end->if1

  /*
   * III. <rect-trajcsp> [28-7-21] Started coding for trajectory execution
   */
   if( user_input == trajcsp_b ) //start->if2
   {
   
   while( (!END_TRAJCSP) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN CONFIGURATION SPACE TRAJECTORY EXECUTION..."));

      // II.1 Display to user the current configuration
      read_current_configuration(); 
      
      // II.2 Specify desired trajectory
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SPECIFY RECT-TRAJECTORY INDEX[10 WAYPOINTS]"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      traj_index = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(traj_index);

      ptr2traj_struct = &trajectories[traj_index];
      
      // II.2.1 [26-8-21] Print the trajectory for user-friendly monitoring
      print_rect_traj_points();
      
      // II.3 SIMPE TRAJ EXECUTION FUNCTION TO BE PLACED HERE!
      rect_trajcsp_sm();
      
      /*
       * FINISH trajcsp
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <rect-trajcsp>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_TRAJCSP    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<rect-trajcsp> FINISHED"));
      }
      else
      {
        END_TRAJCSP    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<rect-trajcsp> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2   

  /*
   * III. <trajcsp> [28-7-21] Started coding for trajectory execution
   */
   if( user_input == trajcsp_c ) //start->if2
   {
   
   while( (!END_TRAJCSP2) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN CONFIGURATION SPACE TRAJECTORY EXECUTION..."));

      // II.1 Display to user the current configuration
      read_current_configuration(); 
      
      // II.2 Specify desired trajectory
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SPECIFY CIRC-TRAJECTORY INDEX[30 WAYPOINTS]"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      traj_index = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(traj_index);

      ptr2traj_struct2 = &trajectories2[traj_index];
      
      // II.2.1 [26-8-21] Print the trajectory for user-friendly monitoring
      print_circ_traj_points();
      
      // II.3 SIMPE TRAJ EXECUTION FUNCTION TO BE PLACED HERE!
      circ_trajcsp_sm();
      
      /*
       * FINISH trajcsp
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <circ-trajcsp>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_TRAJCSP2    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<circ-trajcsp> FINISHED"));
      }
      else
      {
        END_TRAJCSP2    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<circ-trajcsp> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2 
   
  /*
   * V. <home> -> [30-3-21] this should be done if something bad happened and stepper lost position
   */
   if( user_input == home_b ) //start->if2
   {
   
   while( (!END_HOME) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN RE-CALIBRATION HOMING EXECUTION..."));

      slow_home_motors();
      
      /*
       * FINISH home
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <home>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_HOME    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<home> FINISHED"));
      }
      else
      {
        END_HOME    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<home> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2 

  /*
   * V. <grip> -> [29-11-21] Executes in order to grip load 
   */
   if( user_input == grip_b ) //start->if2
   {
   
   while( (!END_GRIP) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("CHANGE GRIPPER STATE"));

      /*
       * Asks to close gripper
       */
      DEBUG_SERIAL.println(F("[ INFO ] CLOSE GRIPPER?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        OvidiusGripper.closeGripperForce(ptr2OvidiusGripperServo,grasp_force_limit_newton, &GripperCurrentState);
        
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER CLOSED"));
      }

      /*
       * Asks to open gripper
       */
      DEBUG_SERIAL.println(F("[ INFO ] OPEN GRIPPER?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        OvidiusGripper.openGripper(ptr2OvidiusGripperServo,&GripperCurrentState); 
        
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER OPENED"));
      }
             
      /*
       * FINISH grip
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <grip>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_GRIP    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<grip> FINISHED"));
      }
      else
      {
        END_GRIP    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<grip> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2 
   
  /*
   * VII. <read_session_data_b> -> [30-3-21] after files were created & data written, executed to plot in serial monitor the loggrd data
   */
   if( user_input == read_session_data_b ) //start->if5
   {
   
   while( (!END_READ_SESSION_DATA) ) //start->wh5
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("READING LOGGED DATA FROM CURRENT SESSION..."));

      // SPECIFY LOG FILE TO READ
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SPECIFY LOG FILE ID TO READ:"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      file_id = DEBUG_SERIAL.parseInt();

      /*
      //START read_session_data
      return_function_state = stp.readSessionFileLoggedData(PTR2RobotDataLog, LOGFILES, FINAL_ACCESSED_FILES , file_id, read_data_buffer, file_read_buffer_size ,MAX_DATA_LENGTH, &error_code_received);
      
      if (return_function_state)
      {
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<read_session_data> SUCCESS"));
        DEBUG_SERIAL.print(read_data_buffer);
      }
      else
      {
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<read_session_data> FAILED"));
      }
      */
      //FINISH read_session_data
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <read_session_data>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_READ_SESSION_DATA    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<read_session_data> FINISHED"));
      }
      else
      {
        END_READ_SESSION_DATA    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<read_session_data> WILL REPEAT"));
      } 

   }//end->wh5
   
   }//end->if5 

   /*
   * VIII. <access PID gains> -> [9-2-22] Executes in order to change PID gains before trajectory execution
   */
   if( user_input == access_PID_gains_b ) //start->if2
   {
   
   while( (!END_ACCESS_PID) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("ACCESSING POSITION CONTROLLER PID GAINS..."));

      /*
       *  CHANGE 
       */
      read_PosControllerGains();
      change_PosControllerGains();
      read_PosControllerGains();
             
      /*
       * FINISH access PID gains
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <access PID gains>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_ACCESS_PID    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<access PID gains> FINISHED"));
      }
      else
      {
        END_ACCESS_PID    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<access PID gainsp> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2
   
  /*
   *  RESET FLAGS
   */
    END_P2PCSP    = false;
    END_TRAJCSP   = false;
    END_TRAJCSP2  = false;
    END_HOME      = false;
    END_READ_SESSION_DATA    = false;
    END_GRIP      = false;
    END_ACCESS_PID   = false;
    //END_FORCE     = false;
    //END_ACCEL     = false;
      
}// END LOOP 

/*
 *  INTERRUPT FUNCTIONS USED
 */
void changeStepperDirInterrupt1()
{
  currentDirStatus  = !currentDirStatus;  // [29-3-21] Only used for setStepperHomePositionSlow -> TO DO: REMOVE THEM
  KILL_MOTION_DUMMY = true;
  
  CustomStepperOvidiusDueShield::KILL_MOTION = true; // [29-3-21] This is the way! Used in execute_StpTrapzProfile3+4
}

/*
 * TIMING ARDUINO THROUGH SERIAL PORT
 */
 time_t requestSync()
{
  DEBUG_SERIAL.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(DEBUG_SERIAL.find(TIME_HEADER)) {
     pctime = DEBUG_SERIAL.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}
