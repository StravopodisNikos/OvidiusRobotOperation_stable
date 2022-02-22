void circ_trajcsp_sm()
{
  // [30-7-21] Executes trajectory using simple Melchiorri time drift technique

  // Define the parameters used at each trajectory segment
  float * Qi; 
  float * Qf;
  float Tseg;

  // Get the total number of trajectory points -> MAX_TRAJ_POINTS : Assumed ct!

  // Give which sensor to log during task execution
  set_sensor_updates();

  // Torque ON the dynamixels
  return_function_state = meta_dxl.setDynamixelsTorqueON(dxl_id, sizeof(dxl_id), dxl);
  if (!return_function_state)
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]"));
  }

  // Go to START POINT
  
  read_current_configuration();
  p2pcsp2_sm(currentConfiguration, ptr2traj_struct2->traj_cs_points2[0], ptr2traj_struct2->time_intervals2[0]);
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("REACHED INITIAL POINT. EXECUTION STARTS IN 1 sec..."));
  delay(1000);
  
  
  // Run loop for p2p between all trajectory points
  int pair_cnt = 0;
  finalTargetSet = false;
  for (size_t i = 0; i < (MAX_TRAJ_POINTS2-1); i++)
  {
    // Assign the Initial point of segment
    if (i==0) // the defined point is give since p2pcsp2_sm is assumed successful
    {
      Qi = ptr2traj_struct2->traj_cs_points2[pair_cnt];
    }
    else
    {
      Qi = Qf; 
    }
   
    // [1-1-22] Final point and Time are given as defined
    Qf = ptr2traj_struct2->traj_cs_points2[pair_cnt+1];
    Tseg = ptr2traj_struct2->time_intervals2[pair_cnt+1];

    if ( i == (MAX_TRAJ_POINTS2-2) )
    {
      finalTargetSet = true;
    }
    // Call the p2pcsp3_sm function for each pair of points
    p2pcsp3_sm(Qi, Qf, Tseg);

    // cnt for next pair
    pair_cnt++;
  }

  delay(500);
  // Print that finished trajectory execution
  DEBUG_SERIAL.print(F("[    INFO    ]")); DEBUG_SERIAL.println(F(" CIRC-TRAJECTORY EXECUTED "));
  
  return;
}
