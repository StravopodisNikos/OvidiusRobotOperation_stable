void exec_homing()
{
  // [20-7-21] Executes homing operation

  PSEUDO.go2HomePositionSlave(&pseudo_current_state, &pseudo_currentAbsPos, &pseudo_current_ci, &currentDirStatusPseudo, &last_executed_operation, &error_code  );
  if (error_code == NO_ERROR)
  {
    Serial.print(F(" [ EXEC_HOME ] ")); Serial.println(F("SET HOME POSITION [ SUCCESS ]"));
  }
  else
  {
    Serial.print(F(" [ EXEC_HOME ] ")); Serial.println(F("SET HOME POSITION [ FAILED ]")); 
  }

  END_HOMING = true;
} //fn end
