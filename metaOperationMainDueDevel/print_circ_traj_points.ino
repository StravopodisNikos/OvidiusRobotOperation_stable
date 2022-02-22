void print_circ_traj_points()
{
  
  DEBUG_SERIAL.println(F("[    INFO    ] CIRCULAR TRAJECTORY SELECTED "));
  DEBUG_SERIAL.println(F("[ CNT ]  [ JOINT1 ]  [ JOINT2 ]  [ JOINT3 ] "));

  for (size_t i = 0; i < MAX_TRAJ_POINTS2; i++) // FOR ALL POINTS
  {
    DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(F("   "));

    //q2print = ptr2traj_struct->traj_cs_points[i];
    
    for (size_t j = 0; j < TOTAL_ACTIVE_JOINTS; j++) // FOR EACH ACTIVE JOINT
    {
       //DEBUG_SERIAL.print(q2print[j],4); DEBUG_SERIAL.print(F("   "));
       DEBUG_SERIAL.print(ptr2traj_struct2->traj_cs_points2[i][j],4); DEBUG_SERIAL.print(F("   "));
    }
    DEBUG_SERIAL.println("");
  }
  
  return;
}
