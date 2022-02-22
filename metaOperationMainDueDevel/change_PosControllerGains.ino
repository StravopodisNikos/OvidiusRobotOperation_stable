void change_PosControllerGains()
{
  bool changed_gains_was_ok = true;
  
  // I. Present the available PID gain parameters && Ask id for each GAIN and assign to the PID arrays
  // I.1 Kp
  DEBUG_SERIAL.println(F("[  INFO  ] READING AVAILABLE POSITION CONTROLLER Kp GAINS[ID-PARAMETER]..."));
  for (size_t i = 0; i < total_available_gains; i++)
  {
    DEBUG_SERIAL.print(F(" [ ID: ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(F(" - ")); 
    DEBUG_SERIAL.print(F("PARAMETER: ")); DEBUG_SERIAL.print(KPgains[i]); DEBUG_SERIAL.println(F(" ] ")); 
  }

  DEBUG_SERIAL.println(F("[ INFO ] ENTER ID FOR Kp: "));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);  
  for (size_t i = 0; i < sizeof(dxl_id); i++)
  {
    dxl_pKp[i] = KPgains[user_input];
  }

  // I.2 Ki
  DEBUG_SERIAL.println(F("[  INFO  ] READING AVAILABLE POSITION CONTROLLER Ki GAINS[ID-PARAMETER]..."));
  for (size_t i = 0; i < total_available_gains; i++)
  {
    DEBUG_SERIAL.print(F(" [ ID: ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(F(" - ")); 
    DEBUG_SERIAL.print(F("PARAMETER: ")); DEBUG_SERIAL.print(KIgains[i]); DEBUG_SERIAL.println(F(" ] "));  
  }
  DEBUG_SERIAL.println(F("[ INFO ] ENTER ID FOR Ki: "));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);  
  for (size_t i = 0; i < sizeof(dxl_id); i++)
  {
    dxl_pKi[i] = KIgains[user_input];
  }

  // I.3 Kd
  DEBUG_SERIAL.println(F("[  INFO  ] READING AVAILABLE POSITION CONTROLLER Kd GAINS[ID-PARAMETER]..."));
  for (size_t i = 0; i < total_available_gains; i++)
  {
    DEBUG_SERIAL.print(F(" [ ID: ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(F(" - ")); 
    DEBUG_SERIAL.print(F("PARAMETER: ")); DEBUG_SERIAL.print(KDgains[i]); DEBUG_SERIAL.println(F(" ] ")); 
  }
  DEBUG_SERIAL.println(F("[ INFO ] ENTER ID FOR Kd: "));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);  
  for (size_t i = 0; i < sizeof(dxl_id); i++)
  {
    dxl_pKd[i] = KDgains[user_input];
  }
    
  // II. Write the desired values to the Dynamixels.
  return_function_state = meta_dxl.syncSetDynamixelsPositionKpGain(dxl_id, sizeof(dxl_id), dxl_pKp, sw_data_array_pKp, &error_code_received, dxl);
  if (!return_function_state)
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE POSITION CONTROLLER Kp GAINS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    changed_gains_was_ok = false;
  }
  
  return_function_state = meta_dxl.syncSetDynamixelsPositionKiGain(dxl_id, sizeof(dxl_id), dxl_pKi, sw_data_array_pKi, &error_code_received, dxl);
  if (!return_function_state)
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE POSITION CONTROLLER Ki GAINS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    changed_gains_was_ok = false;
  }

  return_function_state = meta_dxl.syncSetDynamixelsPositionKdGain(dxl_id, sizeof(dxl_id), dxl_pKd, sw_data_array_pKd, &error_code_received, dxl);
  if (!return_function_state)
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE POSITION CONTROLLER Kd GAINS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    changed_gains_was_ok = false;
  }

  if (changed_gains_was_ok)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] WRITING POSITION CONTROLLER GAINS FINISHED [  SUCCESS  ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  INFO  ] WRITING POSITION CONTROLLER GAINS FINISHED [  FAILED  ]"));
  }
  
  return;
}
