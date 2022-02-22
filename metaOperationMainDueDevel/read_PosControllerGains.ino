void read_PosControllerGains()
{
  return_function_state = meta_dxl.syncGetDynamixelsPositionKpGain(dxl_id, sizeof(dxl_id), dxl_pKp, sr_data_array_pKp, &error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ DYNAMIXELS Kp GAIN [  SUCCESS ]"));
    for (size_t i = 0; i < sizeof(dxl_id); i++)
    {
      DEBUG_SERIAL.print(F(" [ DXL ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(dxl_pKp[i]);
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ DYNAMIXELS Kp GAIN [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }

  return_function_state = meta_dxl.syncGetDynamixelsPositionKiGain(dxl_id, sizeof(dxl_id), dxl_pKi, sr_data_array_pKi, &error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ DYNAMIXELS Ki GAIN [  SUCCESS ]"));
    for (size_t i = 0; i < sizeof(dxl_id); i++)
    {
      DEBUG_SERIAL.print(F(" [ DXL ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(dxl_pKi[i]);
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ DYNAMIXELS Ki GAIN [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }

  return_function_state = meta_dxl.syncGetDynamixelsPositionKdGain(dxl_id, sizeof(dxl_id), dxl_pKd, sr_data_array_pKd, &error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ DYNAMIXELS Kd GAIN [  SUCCESS ]"));
    for (size_t i = 0; i < sizeof(dxl_id); i++)
    {
      DEBUG_SERIAL.print(F(" [ DXL ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(dxl_pKd[i]);
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ DYNAMIXELS Kd GAIN [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }  
  
  return;
}
