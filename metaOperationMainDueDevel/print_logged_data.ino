void print_logged_data()
{
  unsigned long timestamp_pos0;
  unsigned long timestamp_vel0;
  unsigned long timestamp_for0;
  unsigned long timestamp_cur0;
  
  timestamp_pos0 = SENSOR_LOG_STRUCTS[POS_LOG_ID].timestamps[0]; // assume that at least one point of segment is logged
  timestamp_vel0 = SENSOR_LOG_STRUCTS[VEL_LOG_ID].timestamps[0]; // assume that at least one point of segment is logged
  timestamp_for0 = SENSOR_LOG_STRUCTS[FOR_LOG_ID].timestamps[0]; // assume that at least one point of segment is logged
  timestamp_cur0 = SENSOR_LOG_STRUCTS[CUR_LOG_ID].timestamps[0]; // assume that at least one point of segment is logged

  if (update_sensor_measurements[0])
  {
    // First data for EACH sensor is printed in serial monitor
    DEBUG_SERIAL.println(F("[    INFO    ] LOGGED POSITION DATA "));
    DEBUG_SERIAL.println(F("[ TIME ]  [ JOINT1 ]  [ JOINT2 ]  [ JOINT3 ]  [ CNT ]  "));
    for  (size_t i = 0; i < MAX_LOG_DATA; i++)
    {
      // Print only if timestamp is bigger/equal than the 0
      if (SENSOR_LOG_STRUCTS[POS_LOG_ID].timestamps[i] >= timestamp_pos0)
      {
        DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[POS_LOG_ID].timestamps[i]); // write timestamp
        DEBUG_SERIAL.print(F("  "));
        for (size_t j = 0; j < TOTAL_ACTIVE_JOINTS; j++)
        {
            DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[POS_LOG_ID].logged_data[i][j],4);  // write the data in same row
            DEBUG_SERIAL.print(F("    "));
        }
        DEBUG_SERIAL.println(SENSOR_LOG_STRUCTS[POS_LOG_ID].data_cnts[i]); // write data counter number and next line
      }   
    }
  }
  // [9-4-21]... must be done for the rest sensors ... 
  // [26-8-21] Added rest sensors logged data

  if (update_sensor_measurements[1])
  {
    // Velocity
    DEBUG_SERIAL.println(F("[    INFO    ] LOGGED VELOCITY DATA "));
    DEBUG_SERIAL.println(F("[ TIME ]  [ JOINT1 ]  [ JOINT2 ]  [ JOINT3 ] [ CNT ]  "));
    for  (size_t i = 0; i < MAX_LOG_DATA; i++)
    {
      // Print only if timestamp is bigger/equal than the 0
      if (SENSOR_LOG_STRUCTS[VEL_LOG_ID].timestamps[i] >= timestamp_vel0)
      {
        DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[VEL_LOG_ID].timestamps[i]); // write timestamp
        DEBUG_SERIAL.print(F("  "));
        for (size_t j = 0; j < TOTAL_ACTIVE_JOINTS; j++)
        {
            DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[VEL_LOG_ID].logged_data[i][j],4);  // write the data in same row
            DEBUG_SERIAL.print(F("    "));
        }
        DEBUG_SERIAL.println(SENSOR_LOG_STRUCTS[VEL_LOG_ID].data_cnts[i]); // write data counter number and next line     
      }
    }
  }

  if (update_sensor_measurements[2])
  {
    // Force
    DEBUG_SERIAL.println(F("[    INFO    ] LOGGED FORCE DATA "));
    DEBUG_SERIAL.println(F("[ TIME ]  [ FORCE-X ]  [ FORCE-Y ]  [ FORCE-Z ] [ CNT ]"));
    for  (size_t i = 0; i < MAX_LOG_DATA; i++)
    {
      // Print only if timestamp is bigger/equal than the 0
      if (SENSOR_LOG_STRUCTS[FOR_LOG_ID].timestamps[i] >= timestamp_for0)
      {
        DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[FOR_LOG_ID].timestamps[i]); // write timestamp
        DEBUG_SERIAL.print(F("  "));
        for (size_t j = 0; j < num_FORCE_SENSORS; j++)
        {
            DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[FOR_LOG_ID].logged_data[i][j],4);  // write the data in same row
            DEBUG_SERIAL.print(F("    "));
        }
        DEBUG_SERIAL.println(SENSOR_LOG_STRUCTS[FOR_LOG_ID].data_cnts[i]); // write data counter number and next line  
      }   
    }
  }

  if (update_sensor_measurements[3])
  {
    // Current
    DEBUG_SERIAL.println(F("[    INFO    ] LOGGED CURRENT DATA "));
    DEBUG_SERIAL.println(F("[ TIME ]  [ JOINT1 ]  [ JOINT2 ]  [ JOINT3 ] [ CNT ]  "));
    for  (size_t i = 0; i < MAX_LOG_DATA; i++)
    {
      // Print only if timestamp is bigger/equal than the 0
      if (SENSOR_LOG_STRUCTS[CUR_LOG_ID].timestamps[i] >= timestamp_cur0)
      {
        DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[CUR_LOG_ID].timestamps[i]); // write timestamp
        DEBUG_SERIAL.print(F("  "));
        for (size_t j = 0; j < TOTAL_ACTIVE_JOINTS; j++)
        {
            DEBUG_SERIAL.print(SENSOR_LOG_STRUCTS[CUR_LOG_ID].logged_data[i][j],4);  // write the data in same row
            DEBUG_SERIAL.print(F("    "));
        }
        DEBUG_SERIAL.println(SENSOR_LOG_STRUCTS[CUR_LOG_ID].data_cnts[i]); // write data counter number and next line   
      }  
    }
  }
  
  // Write the logged data to file
  // a fn inside custom stepper that takes LOGFILES AND SENSOR_LOG_STRUCTS and
  // calls for each sensor the fn OvidiusSensors/finalWriteData
  return;
}
