#include "Arduino.h"
#include "PseudoSimpleMetamorphicManipulator.h"
#include <utility/pseudo_led_indicators.h>
#include <utility/pseudo_debug.h>
#include <definitions.h> 
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <ovidius_robot_controller_eeprom_addresses.h>

using namespace std;

const double pi              = 3.14159265359;

volatile bool PseudoSimpleMetamorphicManipulator::KILL_MOTION;
volatile bool PseudoSimpleMetamorphicManipulator::HOME_TRIGGERED;
volatile bool PseudoSimpleMetamorphicManipulator::LIMIT_TRIGGERED;

// CLASS CONSTRUCTOR
PseudoSimpleMetamorphicManipulator::PseudoSimpleMetamorphicManipulator(unsigned char ID, int SPR, int GR, int stepPin, int dirPin, int enaPin)
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enaPin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(enaPin, LOW);
    digitalWrite(dirPin, LOW);

	pinMode(redLED_Pin, OUTPUT);
	pinMode(greenLED_Pin, OUTPUT);
	pinMode(blueLED_Pin, OUTPUT);

	_id      = ID;
	_stepPin = stepPin;
	_dirPin  = dirPin;
	_enapin  = enaPin;

    _spr    = SPR;              // Steps per revolution [Found from driver dip switch configuration]
    _gr    	= GR; 
    _ag     = (float) ( ( 2.0 * pi ) / ( _gr * _spr ) ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]}

	META_STEPS = (uint32_t) 0.035714f * _spr * _gr;

	_LED_PIN_STATE = LOW;
	_STEP_PIN_STATE = LOW;

	_last_LED_STATE_update  = 0;
	_last_STEP_STATE_update = 0;
	_update_STEP_STATE_interval = FIXED_PSEUDO_STEP_DELAY;
	//_return_fn_state = false;

}
// PUBLIC
void PseudoSimpleMetamorphicManipulator::blinkPseudoLed_blocking(unsigned char times2Blink, int blinkInterval_millis, const unsigned char *led_indicator)
{
    // Is used only in ino file for status monitoring
    for (size_t i = 0; i < times2Blink; i++)
    {
        setPseudoLed(turn_off_led);
        delay(blinkInterval_millis);
        setPseudoLed(led_indicator);
        delay(blinkInterval_millis);
    }

	return;
}

void PseudoSimpleMetamorphicManipulator::writePseudoID(unsigned char pseudo_id)
{
	EEPROM.update(ID_EEPROM_ADDR,pseudo_id);
}

void PseudoSimpleMetamorphicManipulator::readEEPROMsettingsSlave(unsigned char &pseudo_id, volatile byte &CURRENT_STATE , byte &currentAbsPosPseudo_ci, volatile byte &currentDirStatusPseudo, uint32_t &currentAbsPosPseudo)
{
	// This function is executed at setup() to initialize the global variables of pseudojoint module

	currentDirStatusPseudo = EEPROM.read(CD_EEPROM_ADDR);

	currentAbsPosPseudo_ci = EEPROM.read(CP_EEPROM_ADDR);

	float step_angle = 0.000000f;
	EEPROM.get(STEP_ANGLE_ADDR, step_angle);    			

	float min_pseudo_angle = 0.000000f;
	EEPROM.get(MIN_POS_LIM_ADDR, min_pseudo_angle); 

	float currentAbsPosPseudo_float = 0.000000f;
	currentAbsPosPseudo_float = step_angle * (currentAbsPosPseudo_ci - 1) + min_pseudo_angle;

	currentAbsPosPseudo = (uint32_t) abs(currentAbsPosPseudo_float) / _ag;		

	// 3. Read the saved state
	CURRENT_STATE = EEPROM.read(CS_EEPROM_ADDR);

	// 4. Read the saved ID
	pseudo_id = EEPROM.read(ID_EEPROM_ADDR);

	return;
}

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::go2HomePositionSlave(volatile byte *CURRENT_STATE , uint32_t *currentAbsPosPseudo, byte *currentAbsPosPseudo_ci, volatile byte *currentDirStatusPseudo, byte *operation_executed, debug_error_type *pseudo_error_code)
{

	bool HOMING_PSEUDO = true;
	uint32_t motor_step = 0;

	if( (*CURRENT_STATE == STATE_READY)  )
	{

		while (HOMING_PSEUDO)
		{    
			HOME_TRIGGERED = false;
			LIMIT_TRIGGERED = false;

			*CURRENT_STATE 	= IS_MOVING;

          	Serial.println(F("[   INFO   ]	PSEUDO HOMING	"));

		    updateSingleStepFixedDelay(motor_step);		// -> executes half step if time has come!

			updateLedState(motors_homing_indicator, PSEUDO_HOME_LED_BLINK_INTERVAL); // [skyblue color]

			if (LIMIT_TRIGGERED)
			{
				// Change DIR Pin status
				//digitalWrite(_dirPin, !currentDirStatusPseudo);
				digitalWrite(_dirPin, *currentDirStatusPseudo);

				// set led indicator that motor breaks out of blocking
				setPseudoLed(limit_switch_indicator);

				// Break free -> multistep sequence
				for (size_t break_free_steps = 0; break_free_steps < BREAK_FREE_STEPS; break_free_steps++)
				{
					_time_now_micros = micros();
					digitalWrite(_stepPin, HIGH);
					while(micros() < _time_now_micros + FIXED_PSEUDO_STEP_DELAY){}   	// wait approx. [μs]
					digitalWrite(_stepPin, LOW);
					while(micros() < _time_now_micros + FIXED_PSEUDO_STEP_DELAY){}   	// wait approx. [μs]
				}

				// turn off blocking led indicator
				setPseudoLed(turn_off_led);

				LIMIT_TRIGGERED = false;
				KILL_MOTION = false;
			}

			if (HOME_TRIGGERED)
			{
				setPseudoLed(completed_move_indicator);

				for (size_t homing_calibration_steps = 0; homing_calibration_steps < HOMING_CALIBRATION_LIMIT2; homing_calibration_steps++)  // <CHANGE>
				//for (size_t homing_calibration_steps = 0; homing_calibration_steps < HOMING_CALIBRATION_LIMIT; homing_calibration_steps++)
				{
					_time_now_micros = micros();
					digitalWrite(_stepPin, HIGH);
					while(micros() < _time_now_micros + FIXED_PSEUDO_STEP_DELAY){}   	// wait approx. [μs]
					digitalWrite(_stepPin, LOW);
					while(micros() < _time_now_micros + FIXED_PSEUDO_STEP_DELAY){}   	// wait approx. [μs]
				}
				
				//setPseudoLed(pseudo_turn_off_led); -> Should remain purple after homed motor!
				HOMING_PSEUDO = false;
				HOME_TRIGGERED = false;
			}

		}

		*currentAbsPosPseudo 	= 0;
		*currentAbsPosPseudo_ci = home_ci; 			

		EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);

		*CURRENT_STATE 			= STATE_READY;

		EEPROM.update(CS_EEPROM_ADDR, *CURRENT_STATE);

		*operation_executed = OPERATION_HOME;		// this value will be given to saveEEPROMsettingsSlave to accordingly save CS to EEPROM

		pseudo_error_code = NO_ERROR;
	}
	else
	{
		pseudo_error_code = HOME_FAILED;
	}

	return;
}  

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::setGoalPositionSlave( byte *PSEUDO_GOAL_POSITION, byte *currentAbsPosPseudo_ci, uint32_t *RELATIVE_STEPS_TO_MOVE, volatile byte *currentDirStatusPseudo, volatile byte *CURRENT_STATE, debug_error_type *pseudo_error_code )
{
	// Calculates the relative steps for pseudo to move

	if ( *CURRENT_STATE == STATE_READY ) 
	{
		*RELATIVE_STEPS_TO_MOVE = (uint32_t) METAMORPHOSIS_Ci_STEPS2 * abs( *PSEUDO_GOAL_POSITION - *currentAbsPosPseudo_ci );  // <CHANGE>
		//*RELATIVE_STEPS_TO_MOVE = (uint32_t) METAMORPHOSIS_Ci_STEPS * abs( *PSEUDO_GOAL_POSITION - *currentAbsPosPseudo_ci );

		if ( (*RELATIVE_STEPS_TO_MOVE < 0) ||  (*RELATIVE_STEPS_TO_MOVE > ( (uint32_t) METAMORPHOSIS_Ci_STEPS2 * c15 ) ) )     // <CHANGE>
		//if ( (*RELATIVE_STEPS_TO_MOVE < 0) ||  (*RELATIVE_STEPS_TO_MOVE > ( (uint32_t) METAMORPHOSIS_Ci_STEPS * c15 ) ) ) 
		{
			pseudo_error_code = SET_GP_FAILED;
		}
		else
		{
			if ( *PSEUDO_GOAL_POSITION <= *currentAbsPosPseudo_ci)
			{
				*currentDirStatusPseudo = CCW;						// MOTOVARIO: CCW  // <CHANGE> <ROT-SET>
			}
			else
			{
				*currentDirStatusPseudo = CW;						// MOTOVARIO: CW   // <CHANGE> <ROT-SET> 
			}
			
			digitalWrite(_dirPin, *currentDirStatusPseudo);

			_goal_ci = *PSEUDO_GOAL_POSITION;

			EEPROM.update(CD_EEPROM_ADDR, *currentDirStatusPseudo);

			*CURRENT_STATE = STATE_READY;
			
			pseudo_error_code = NO_ERROR;			
		}
		
	}
	else
	{
		pseudo_error_code = FALSE_STATE;
	}

	return;

} 

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::movePseudoSlave(  volatile byte *CURRENT_STATE , byte *currentAbsPosPseudo_ci, uint32_t *RELATIVE_STEPS_TO_MOVE, byte *operation_executed, debug_error_type *pseudo_error_code)
{
	// Steps the motor 

	uint32_t motor_step = 0;

	uint32_t TOTAL_STEPS2MOVE = (uint32_t) 2 * (*RELATIVE_STEPS_TO_MOVE); // [5-4-21 ]because state machine is implemented now

	if( (*CURRENT_STATE == STATE_READY) )
	{
		// moves motor
		KILL_MOTION = false;

        while (	( motor_step < abs( TOTAL_STEPS2MOVE ) ) && (!KILL_MOTION) )
		{  
		    updateSingleStepFixedDelay(motor_step);		// -> executes half step if time has come!

			updateLedState(motors_moving_indicator, PSEUDO_MOVE_LED_BLINK_INTERVAL); // [skyblue color]

          	Serial.print(F("[   INFO   ]	PSEUDO MOVING	")); Serial.print(F("CURRENT STEP -> ")); Serial.println(motor_step);

			*CURRENT_STATE = IS_MOVING;

			if (KILL_MOTION)
			{
				Serial.println(F("[   INFO   ] 	MIN/MAX LIMIT HALL SENSOR ACTIVATED! KILLS MOTION..."));

				*CURRENT_STATE = BLOCKED;

				EEPROM.update(CS_EEPROM_ADDR, *CURRENT_STATE);

				KILL_MOTION = false;
			}
			
        }

		if (motor_step == TOTAL_STEPS2MOVE)
		{
			*currentAbsPosPseudo_ci = _goal_ci;

			EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);

			*CURRENT_STATE = STATE_READY;

			EEPROM.update(CS_EEPROM_ADDR, *CURRENT_STATE);

			pseudo_error_code = NO_ERROR;
		}
		else
		{
			pseudo_error_code = MOVE_FAILED;
		}
			
		
		*operation_executed = OPERATION_META;		// this value will be given to saveEEPROMsettingsSlave to accordingly save CS to EEPROM

	}
	else
	{
		pseudo_error_code = FALSE_STATE;
	}

	return;

}

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::setManualCurrentPositionSlave( byte user_given_ci, byte *currentAbsPosPseudo_ci)
{
	*currentAbsPosPseudo_ci = user_given_ci;

	EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);

	return;
}

// =========================================================================================================== //

// PRIVATE
void PseudoSimpleMetamorphicManipulator::setPseudoLed(const unsigned char *led_indicator)
{
    analogWrite(redLED_Pin   , 0);  // write RED VALUE
    analogWrite(greenLED_Pin , 0);  // write GREEN VALUE 
    analogWrite(blueLED_Pin  , 0);  // write BLUE VALUE 

    analogWrite(redLED_Pin   , led_indicator[0]);  // write RED VALUE
    analogWrite(greenLED_Pin , led_indicator[1]);  // write GREEN VALUE 
    analogWrite(blueLED_Pin  , led_indicator[2]);  // write BLUE VALUE 
 
    return;        
}

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::updateSingleStepFixedDelay(uint32_t &StpPresentPosition)
{
  // Only half step pulse is generated! Total+steps/phase must be multiplied x 2! 
    
    if (micros() - _last_STEP_STATE_update > _update_STEP_STATE_interval)
    {
      digitalWrite(_stepPin, !_STEP_PIN_STATE);

      _last_STEP_STATE_update = micros();

      _STEP_PIN_STATE = !_STEP_PIN_STATE;

      StpPresentPosition++;  
	}
    
} 

// =========================================================================================================== //

void PseudoSimpleMetamorphicManipulator::updateLedState(const unsigned char *led_indicator, unsigned long led_state_interval)
{
  // Blinks the led for the  led_state_interval[millis] for the colour given by led_indicator

    if (millis() - _last_LED_STATE_update > led_state_interval)
    {
	  // check current state
	  if (_LED_PIN_STATE == LOW)
	  {
		  setPseudoLed(led_indicator);

		  _LED_PIN_STATE = HIGH;
	  }
	  else
	  {
		  setPseudoLed(turn_off_led);

		  _LED_PIN_STATE = LOW;
	  }

      _last_LED_STATE_update = millis();

	}
    
} 
