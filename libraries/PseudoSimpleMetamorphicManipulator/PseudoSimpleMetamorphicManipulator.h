#ifndef PseudoSimpleMetamorphicManipulator_h
#define PseudoSimpleMetamorphicManipulator_h

// Default includes for driving pseudojoint steppers
#include "Arduino.h"
#include <EEPROM.h>
#include <definitions.h> 

using namespace std;

typedef unsigned char debug_error_type;
typedef bool pseudo_led_states;
enum PSEUDO_ROT_DIR{CCW, CW};

class PseudoSimpleMetamorphicManipulator
{

   public:

      static volatile bool KILL_MOTION;
      static volatile bool HOME_TRIGGERED;
      static volatile bool LIMIT_TRIGGERED;

      // Constructor
      PseudoSimpleMetamorphicManipulator(unsigned char ID, int SPR, int GR, int stepPin, int dirPin, int enaPin);

      void blinkPseudoLed_blocking(unsigned char times2Blink, int blinkInterval_millis, const unsigned char *led_indicator);

      void writePseudoID(unsigned char pseudo_id);

      void readEEPROMsettingsSlave(unsigned char &pseudo_id, volatile byte &CURRENT_STATE , byte &currentAbsPosPseudo_ci, volatile byte &currentDirStatusPseudo, uint32_t &currentAbsPosPseudo);

      void go2HomePositionSlave(volatile byte *CURRENT_STATE , uint32_t *currentAbsPosPseudo, byte *currentAbsPosPseudo_ci, volatile byte *currentDirStatusPseudo, byte *operation_executed, debug_error_type *pseudo_error_code);

      void setGoalPositionSlave( byte *PSEUDO_GOAL_POSITION, byte *currentAbsPosPseudo_ci, uint32_t *RELATIVE_STEPS_TO_MOVE, volatile byte *currentDirStatusPseudo, volatile byte *CURRENT_STATE, debug_error_type *pseudo_error_code );

      void movePseudoSlave(  volatile byte *CURRENT_STATE , byte *currentAbsPosPseudo_ci, uint32_t *RELATIVE_STEPS_TO_MOVE, byte *operation_executed, debug_error_type *pseudo_error_code);

      void setManualCurrentPositionSlave( byte user_given_ci, byte *currentAbsPosPseudo_ci);

   private:
      unsigned char _id;
      int _stepPin;
      int _dirPin;
      int _enapin;
      int _spr;
      int _gr;
      float _ag;
      byte _goal_ci;
      bool _return_fn_state;

      uint32_t META_STEPS;
      
      unsigned long _time_now_micros = 0;
      unsigned long _last_LED_STATE_update = 0;
      unsigned long _last_STEP_STATE_update = 0;

		boolean         _STEP_PIN_STATE;
		pseudo_led_states _LED_PIN_STATE;
      unsigned long _update_STEP_STATE_interval;

      void setPseudoLed(const unsigned char *led_indicator);

      void updateSingleStepFixedDelay(uint32_t &StpPresentPosition);

      void updateLedState(const unsigned char *led_indicator, unsigned long led_state_interval);

};

#endif
