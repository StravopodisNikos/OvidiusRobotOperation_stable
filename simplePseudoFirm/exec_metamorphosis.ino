void exec_metamorphosis()
{
  // [20-7-21] Reads current pseudo state, if pseudo is ready executes, else aborts.

  if (pseudo_current_state == STATE_READY)
  {
    // I. Executes metamorphosis

    // I.1. Ask for goal ci
    run4ci = true;

    while(run4ci) // while#  -->> [loop]
    {
      Serial.print(F(" [ INFO ] ")); Serial.println(F("INSERT NEW Ci(1-15)"));
      Serial.parseInt();
      while (Serial.available() == 0) {};
      user_input = Serial.parseInt();
      Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
      
      if (user_input >= c1 && user_input <= c15)
      {
        // I.2. Set goal position
        // If correct ci was given execute the fn that calculates the needed steps2move
        PSEUDO.setGoalPositionSlave( &user_input, &pseudo_current_ci, &steps2move , &currentDirStatusPseudo, &pseudo_current_state, &error_code ); 

        if (error_code == NO_ERROR)
        {
          Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("SET GOAL POSITION [ SUCCESS ]"));
          Serial.print(F(" [   INFO    ] ")); Serial.print(F("STEPS TO MOVE: [ ")); Serial.print(steps2move); Serial.println(F(" ]"));
          Serial.print(F(" [   INFO    ] ")); Serial.print(F("DIR TO MOVE[0:CCW,1:CW]: [ ")); Serial.print(currentDirStatusPseudo); Serial.println(F(" ]"));
        }
        else
        {
          Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("SET GOAL POSITION [ FAILED ]")); //while loop will terminate since correct ci was given BUT an error has occured in steps calculation
        }
        
        run4ci = false;
      }
      else
      {
        Serial.print(F(" [ INFO ] ")); Serial.println(F("INSERT NEW Ci [FAILED]"));
        run4ci = true;
      }

    }// while# <<--

      delay(2000);
      
    // I.3. Move Pseudo to goal ci
    if (error_code == NO_ERROR)
    {
      PSEUDO.movePseudoSlave(&pseudo_current_state, &pseudo_current_ci, &steps2move, &last_executed_operation, &error_code  );
      
      if (error_code == NO_ERROR)
      {
        Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("MOVE PSEUDO TO POSITION [ SUCCESS ]"));
        Serial.print(F(" [   INFO    ] ")); Serial.print(F("CURRENT Ci: [ ")); Serial.print(pseudo_current_ci); Serial.println(F(" ]"));
      }
      else
      {
        Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("MOVE PSEUDO TO POSITION [ FAILED ]"));
      } 
    }
    
    // I.4. Terminates and prints the error and  state codes 
    if (error_code == NO_ERROR)
    {
      Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("EXECUTED METAMORPHOSIS [ SUCCESS ]"));
      Serial.print(F(" [   INFO    ] ")); Serial.print(F("ERROR CODE:" )); Serial.println(error_code);
      Serial.print(F(" [   INFO    ] ")); Serial.print(F("STATE CODE:" )); Serial.println(pseudo_current_state);
    }
    else
    {
      Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("EXECUTED METAMORPHOSIS [ FAILED ]"));
      Serial.print(F(" [   INFO    ] ")); Serial.print(F("ERROR CODE:" )); Serial.println(error_code);
      Serial.print(F(" [   INFO    ] ")); Serial.print(F("STATE CODE:" )); Serial.println(pseudo_current_state);
    }
    
  }
  else
  {
    // II. Aborts and asks to execute HOMING
    error_code = FALSE_STATE;

    Serial.print(F(" [ EXEC_META ] ")); Serial.println(F("STATE NOT READY. ABORTS..."));
    Serial.print(F(" [   INFO    ] ")); Serial.println(F("MUST EXECUTE HOMING! "));
  }

  END_METAMORPHOSIS = true;
}// fn end
