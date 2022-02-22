void setup_force_sensor()
{
  // =======================================
  // CALIBRATION MUST BE EXECUTED FROM SKETCH
  // calibrate_3axis_force_sensor.ino
  // in order to determine the array:
  // float manual_calibration_scale_factors[3]
  
  manual_calibration_scale_factors[0] =  160000;      // X-AXIS - manually certifed on [9-2-22]
  manual_calibration_scale_factors[1] = -150000;      // Y-AXIS - manually certifed on [9-2-22]
  manual_calibration_scale_factors[2] =  480000;      // Z-AXIS - manually certifed on [4-2-22]
  // =======================================

// [4-2-22] Only when single axis object is defined
//ForceSensor = & SingleForceSensor;
//ForceSensorHX711 = & SingleForceSensorHX711;

DEBUG_SERIAL.println(F("[    INFO    ] SETUP 3D-FORCE SENSOR"));
DEBUG_SERIAL.println(F("[    INFO    ] REMOVE ANY LOAD ATTACHED..."));
delay(4000);
DEBUG_SERIAL.println(F("[    INFO    ] SETUP STARTED..."));
/*
 * Initializes pins for each sensor and timeout pings, sets state to FORCE_READY 
 */
 bool pinged_all_sensors = true;
 for (size_t i = 0; i < num_FORCE_SENSORS; i++) // num_FORCE_SENSORS -> 1 to match testing_state_machine
 {
  return_function_state = ForceSensor[i].setupForceSensor((ForceSensorHX711+i), manual_calibration_scale_factors[i] , &ForceCurrentState, &sensor_error);
    //return_function_state = ForceSensor->setupForceSensor(ForceSensorHX711, manual_calibration_scale_factors[2] , &ForceCurrentState, &sensor_error);
  if (!return_function_state)
  {
    pinged_all_sensors =  false;
    DEBUG_SERIAL.print(F("[    INFO    ] SENSOR AXIS ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" PINGED  [ FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
    break;
  }
  else
  {
      DEBUG_SERIAL.print(F("[    INFO    ] SENSOR AXIS ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" PINGED  [ SUCCESS ]"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
  }
 }


 if(pinged_all_sensors)
 {
    DEBUG_SERIAL.println(F("[    INFO    ] 3 AXIS FORCE SENSOR PINGED [ SUCCESS ]")); // ALL SENSOR STATES->FORCE_IDLE
 }
 else
 {
    DEBUG_SERIAL.println(F("[    INFO    ] 3 AXIS FORCE SENSOR PINGED [ FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
 }
 
/*
 * Serial prints the each axis offset found in setupForceSensor
 */
 //long Z_FORCE_OFFSET;
 //return_function_state = ForceSensor[2].getPermanentZeroOffset((ForceSensorHX711+2), &Z_FORCE_OFFSET);
 //return_function_state = ForceSensor->getPermanentZeroOffset(ForceSensorHX711, &Z_FORCE_OFFSET);
 for (size_t i = 0; i < num_FORCE_SENSORS; i++) // num_FORCE_SENSORS -> 1 to match testing_state_machine
 {
   return_function_state = ForceSensor[i].getPermanentZeroOffset((ForceSensorHX711+i), (ForceSensorAxesOffest+i));
   if (return_function_state)
   {
      DEBUG_SERIAL.print(F("[    INFO    ] AXIS: ")); DEBUG_SERIAL.print(i);   DEBUG_SERIAL.print(F(" OFFSET: "));  DEBUG_SERIAL.println(ForceSensorAxesOffest[i]); 
   }
   else
   {
      DEBUG_SERIAL.print(F("[    INFO    ] AXIS: ")); DEBUG_SERIAL.print(i);   DEBUG_SERIAL.print(F(" OFFSET:  [FAILED] "));
   }
 }

 
DEBUG_SERIAL.println(F("[    INFO    ] ATTACH LOAD ON END-EFFECTOR AND PRESS <1> TO GET SINGLE MEASUREMENT..."));
bool rightKeyPressed = false;
while (!rightKeyPressed)
{
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
  if (user_input == YES_b)
  {
    rightKeyPressed = true;
  }
  else
  {
    DEBUG_SERIAL.println(F("[    INFO    ] WRONG INPUT! PRESS <1> TO GET SINGLE MEASUREMENT..."));
  }
}
DEBUG_SERIAL.println(F("[    INFO    ] MEASUREMENT STARTED..."));

/*
 * Get single measurement
 */
 //float force_measurements_nwts[num_FORCE_SENSORS];
 float force_measurements_kgs[num_FORCE_SENSORS];
 //float force_measurements_nwts;
 //float force_measurements_kgs;
 for (size_t i = 0; i < num_FORCE_SENSORS; i++)
 {
    return_function_state = ForceSensor[i].measureForceKilos((ForceSensorHX711+i), (force_measurements_kgs+i), &sensor_error);
    //return_function_state = ForceSensor->measureForceKilos(ForceSensorHX711, &force_measurements_kgs, &sensor_error);   
    if (return_function_state)
    {
      DEBUG_SERIAL.print(F("[    INFO    ] FORCE SENSOR")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" MEASURED  [ SUCCESS ]"));
      DEBUG_SERIAL.print(F("[MEASUREMENT: "));  DEBUG_SERIAL.print(force_measurements_kgs[i]); DEBUG_SERIAL.println(F(" [N]"));
      //DEBUG_SERIAL.print(F("[MEASUREMENT: "));  DEBUG_SERIAL.print(force_measurements_kgs); DEBUG_SERIAL.println(F(" [N]"));
    }
    else
    {
      DEBUG_SERIAL.print(F("[    INFO    ] FORCE SENSOR")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" MEASURED  [ FAILED ]"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
    }
}

DEBUG_SERIAL.println(F("[    INFO    ] SETUP 3D-FORCE SENSOR FINISHED"));

return;
    
}
