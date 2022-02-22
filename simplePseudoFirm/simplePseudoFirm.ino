/*
 * ** DEVEL LOG ** VERSIONS **
 * *****************************************************************
 * [24-5-21]  MANUAL PSEUDO OPERATION
 * [20-7-21]  ADDED MENU CODE FOR OPERATION
 * [26-7-21]  HARDWARE TESTED AND VERIFIED [v1]
 * 
 * [25-10-21] ADDED FN FOR ID SET. HARDWARE TESTED PSEUDOS 3-4 [v2]
 * [9 -2 -22] SEARCH FOR <CHANGE> TO FIND LINES THAT NEED TO BE 
 *            RECONFIGURED DEPENDING PSEUDO TYPE.
 * [10-2- 22] To change the ROT DIR depending the assembly of the  
 *            structure search for <ROT-SET> in the PseudoSimpleMetamorphicManipulator.cpp
 * [21-2- 22] The Pseudos currently installed on manipulator are            
 *            configured and work properly. For each other pseudo installed 
 *            this firm should be uploaded after the corresponding setting.
 *            
 * TO BE IMPROVED [SOFTWARE]:
 * [i]  Remove the need of choosing between METAMORPHOSIS_Ci_STEPS2 \ METAMORPHOSIS_Ci_STEPS. Fix bug
 *      in the class-object constructor of the pseudo.
 * [ii] Fix bug inefficiency for c=1,15 and pseudos with hall sensors. The limit switch is 
 *      ON when anatomy is at these positions and then needs to manually remove the limit hall
 *      and execute HOMING.
 * [iii]Add current sensor, to monitor the current drawn during passive operation
 * [iv] Add RS385 asynchronous serial communication protocol to improve passive operation
 *      implementation speed and efficiency.
 *      
 * TO BE IMPROVED [HARDWARE]:
 * [i]  Manufacture metallic sleeves for motovario gearbox+nema23 torque transmission
 * 
 * *****************************************************************
 */
 
// Used Libraries
#include <stdlib.h>
#include <stdio.h>
#include "PseudoSimpleMetamorphicManipulator.h"
#include <utility/pseudo_led_indicators.h>
#include <utility/pseudo_debug.h>

// MANUAL SET THE PSEUDO ID ->
unsigned char pseudo_used_id = PSEUDO6_ID;  // <CHANGE>
// <- SET PSEUDO ID

//USER DEBUG_SERIAL INPUT VARIABLES
byte user_input;
byte YES_b     = 1;
byte NO_b      = 0;
const char * meta_exec = "M";
const char * home_exec = "H";
debug_error_type error_code;
int nl_char_shifting   = 10;
String user_input_string;
bool run4ci;

//FLAGS USED TO CONTROL LOOPS
bool END_METAMORPHOSIS;                      
bool END_HOMING;

// GLOBALS
unsigned char pseudo_eeprom_id;
byte pseudo_current_ci;
uint32_t pseudo_currentAbsPos;
uint32_t steps2move;
volatile byte pseudo_current_state;
volatile byte currentDirStatusPseudo = LOW; // [30-11-21] CHECK DIR MOTION: [PSEUDO1 = LOW, PSEUDO2 = HIGH] // [3-2-22] MOTOVARIO: LOW // <CHANGE>
byte last_executed_operation;

// CLASS OBJECT
//PseudoSimpleMetamorphicManipulator PSEUDO(pseudo_used_id,SPR_PSEUDO1,GEAR_FACTOR_PSEUDO1,stepPin_NANO,dirPin_NANO,enabPin_NANO);  // <CHANGE>
PseudoSimpleMetamorphicManipulator PSEUDO(pseudo_used_id,SPR_PSEUDO2,GEAR_FACTOR_PSEUDO2,stepPin_NANO,dirPin_NANO,enabPin_NANO);    // <CHANGE>

void setup()
{
  // I.1. Wait for Serial Bus
  Serial.begin(SERIAL_BAUDRATE2);    // Set to 115200 in order to easily Open/Close Serial Monitor during Active/Passive Operation        
  while(!Serial);
  Serial.print(F("[ INFO ] CONNECTING TO PSEUDO: [ ")); Serial.print(pseudo_used_id); Serial.println(F(" ]"));

  
  // I.2. Set the INTERRUPTS
  pinMode(hallSwitch_Pin, INPUT_PULLUP);
  pinMode(pseudoLimitSwitch_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pseudoLimitSwitch_Pin), changePseudoDirInterrupt, FALLING);  // MOTOVARIO: FALLING CHINA: RISING  // <CHANGE>
  attachInterrupt(digitalPinToInterrupt(hallSwitch_Pin), beginHomingCalibrationInterrupt, FALLING);  // MOTOVARIO: FALLING CHINA: RISING  // <CHANGE>

  // II. Blink Led(4x250) - > started setup
  PSEUDO.blinkPseudoLed_blocking(START_TIMES,1000,pinged_indicator);
  
  // III. Read EEPROM
  Serial.print(F("[ INFO ] READING EEPROM OF PSEUDO: [ ")); Serial.print(pseudo_used_id); Serial.println(F(" ]"));
  PSEUDO.readEEPROMsettingsSlave(pseudo_eeprom_id, pseudo_current_state , pseudo_current_ci,  currentDirStatusPseudo, pseudo_currentAbsPos);
  Serial.print(F("[   PSEUDO:")); Serial.print(pseudo_eeprom_id); Serial.println(F("   ]   [READ EEPROM SETTINGS]   SUCCESS"));
  
   // Print the results in serial monitor
   if (pseudo_eeprom_id == pseudo_used_id)
   {
      Serial.print(F("[         ID         ]    [   ")); Serial.print(pseudo_eeprom_id); Serial.println(F("   ]"));
      Serial.print(F("[   CURRENT_STATE    ]    [   ")); Serial.print(pseudo_current_state); Serial.println(F("   ]"));
      Serial.print(F("[   CURRENT_POS_ci   ]    [   ")); Serial.print(pseudo_current_ci); Serial.println(F("     ]"));
      Serial.print(F("[   CURRENT_DIR      ]    [   ")); Serial.print(currentDirStatusPseudo); Serial.println(F("     ]"));

      PSEUDO.blinkPseudoLed_blocking(CONNECT_TIMES,CONNECT_INTERVAL,pinged_indicator);
   }
   else
   {
      Serial.print(F("[         ID         ]    [   MISMATCH  ]"));
      pseudo_eeprom_id = pseudo_used_id;
      PSEUDO.writePseudoID(pseudo_eeprom_id);
      Serial.print(F("[   NEW   ID SET     ]    [   ")); Serial.print(pseudo_eeprom_id); Serial.println(F("   ]"));
      
      PSEUDO.blinkPseudoLed_blocking(ERROR_CONNECT_TIMES,ERROR_CONNECT_INTERVAL,critical_error_led);    
   }

   pseudo_current_state = STATE_READY;
    
  // IV. Ask if Homing needed (Pseudo Angle id)
    Serial.flush();
    Serial.println(F("[ SETUP ] HOME PSEUDO?"));
    Serial.parseInt();
    while (Serial.available() == 0) {};
    user_input = Serial.parseInt();
    Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
    Serial.flush();
    if( user_input == YES_b ) 
    {
      Serial.print(F(" [ INFO ] ")); Serial.println(F("HOMING PSEUDO"));

      PSEUDO.go2HomePositionSlave( &pseudo_current_state, &pseudo_currentAbsPos, &pseudo_current_ci, &currentDirStatusPseudo, &last_executed_operation, &error_code  );
      if( error_code == NO_ERROR)
      {
        Serial.print(F(" [ INFO ] ")); Serial.println(F("HOMING PSEUDO [SUCCESS]"));
      }
      else
      {
        Serial.print(F(" [ INFO ] ")); Serial.println(F("HOMING PSEUDO [FAILED]"));
      }
      
    }

   // V. Ask for manual change of the current ci
    Serial.flush();
    Serial.println(F("[ SETUP ] CHANGE CURRENT Ci?"));
    Serial.parseInt();
    while (Serial.available() == 0) {};
    user_input = Serial.parseInt();
    Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
    Serial.flush();
    
    if( user_input == YES_b ) 
    {
      // Runs inside a loop and breaks only for correct ci
      run4ci = true;

      while(run4ci) // while#
      {
        Serial.print(F(" [ INFO ] ")); Serial.println(F("INSERT NEW Ci(1-15)"));
        Serial.flush();
        Serial.parseInt();
        while (Serial.available() == 0) {};
        user_input = Serial.parseInt();
        Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
        Serial.flush();
        
        if (user_input >= c1 && user_input <= c15)
        {
          pseudo_current_ci = user_input;
          run4ci = false;
        }
        else
        {
          Serial.print(F(" [ INFO ] ")); Serial.println(F("INSERT NEW Ci [FAILED]"));
          run4ci = true;
        }

      }// while#
    }    
    
  // VI. Blink Led(2x1000) - > finished setup
  PSEUDO.blinkPseudoLed_blocking(FINISH_TIMES,1000,finished_indicator);
}

void loop() {
  // Inform that loop is executed
  Serial.println(F(" [ LOOP ] EXECUTING..."));
  
  // I. menu - ask for META-HOME
  Serial.flush();
  Serial.print(F(" [ LOOP ] ")); Serial.println(F("Set ROBOT OPERATION MODE: FORMAT <MODE>:"));
  while (Serial.available() == 0) {};
  user_input_string = Serial.readString();
  Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.println(user_input_string);
  Serial.flush();
  /*
   * II. <METAMORPHOSIS>
   */
   if( ( strcmp(user_input_string.c_str(),meta_exec)-nl_char_shifting == 0 ) )
   {
     END_METAMORPHOSIS = false;
    
     while( (!END_METAMORPHOSIS) ) 
     {
      Serial.print(F(" [ LOOP ] ")); Serial.println(F("BEGIN METAMORPHOSIS..."));

      // exec_metamorphosis
      exec_metamorphosis();
     }
     
   } // meta

  /*
   * III. <HOME>
   */
   if( ( strcmp(user_input_string.c_str(),home_exec)-nl_char_shifting == 0 ) )
   {
     END_HOMING = false;
    
     while( (!END_HOMING) ) 
     {
      Serial.print(F(" [ LOOP ] ")); Serial.println("BEGIN HOMING...");

      // exec_homing
      exec_homing();
     }
     
   } // home

} //loop

/*
 *  INTERRUPT FUNCTIONS USED
 */
void changePseudoDirInterrupt()
{
  PseudoSimpleMetamorphicManipulator::KILL_MOTION = true; 
  PseudoSimpleMetamorphicManipulator::LIMIT_TRIGGERED = true;
  currentDirStatusPseudo = !currentDirStatusPseudo;
}

void beginHomingCalibrationInterrupt()
{
  PseudoSimpleMetamorphicManipulator::HOME_TRIGGERED = true;
}
