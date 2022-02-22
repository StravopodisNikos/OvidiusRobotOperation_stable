 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
//#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

// Include Motor Configuration files from folder ~/Arduino/libraries/
#include <definitions.h>                            
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <utility/StepperMotorSettings.h>
#include <utility/dynamixel_settings.h>
#include <utility/dynamixel_debug.h>
#include <Array.h>

using namespace std;
using namespace ControlTableItem;

DYNAMIXEL::InfoSyncWriteInst_t sw_gp_infos;                     // write goal position
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_gp[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pv_infos;                     // write profile velocity                                        
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pv[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pa_infos;                     // write profile acceleration
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pa[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pKp_infos;                     // write position-Pgain
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pKp[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pKi_infos;                     // write position-Igain
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pKi[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pKd_infos;                     // write position-Dgain
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pKd[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pp_infos;                      // read present position
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pp[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pv_infos;                      // read present velocity
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pv[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_mov_infos;                     // read moving
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_mov[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pc_infos;                      // read present current
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pc[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pKp_infos;                      // read position-Pgain
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pKp[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pKi_infos;                      // read position-Igain
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pKi[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncReadInst_t sr_pKd_infos;                      // read position-Dgain
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pKd[DXL_ID_SIZE];

const double pi              = 3.14159265359;

// Constructor
DynamixelProPlusOvidiusShield::DynamixelProPlusOvidiusShield(uint8_t *DxlIDs){

    uint8_t _DXL1_ID = DxlIDs[0];
    uint8_t _DXL2_ID = DxlIDs[1];
    //uint8_t _DXL3_ID = DxlIDs[2]; // <DOF-SET> 
}

// =========================================================================================================== //
bool DynamixelProPlusOvidiusShield::setDynamixelsTorqueON(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl)
{
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++)
    {
        dxl.torqueOn(DxlIDs[dxl_cnt]);
    }

    return true;
}

bool DynamixelProPlusOvidiusShield::setDynamixelsTorqueOFF(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl)
{
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++)
    {
        dxl.torqueOff(DxlIDs[dxl_cnt]);
    }

    return true;
}
// =========================================================================================================== //

void DynamixelProPlusOvidiusShield::setDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, const unsigned char *led_indicator, Dynamixel2Arduino dxl)
{
 /*
  *  Sets value to LED of Dynamixels given the ID numbers and the desired color(as array 3 elements)
  */
    bool command_execution_success;
    /*
    // 1. turns off led
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++) {
            dxl.writeControlTableItem(LED_RED, DxlIDs[dxl_cnt], 0); // write RED VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_GREEN, DxlIDs[dxl_cnt], 0); // write GREEN VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_BLUE, DxlIDs[dxl_cnt], 0); // write BLUE VALUE to DXL Control Table
            unsigned long time_now_millis = millis(); while(millis() < time_now_millis + 500){} // waits 500 milliseconds
    }*/
    // 1. sets specified color value
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++) {
            dxl.writeControlTableItem(LED_RED, DxlIDs[dxl_cnt], led_indicator[0]); // write RED VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_GREEN, DxlIDs[dxl_cnt], led_indicator[1]); // write GREEN VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_BLUE, DxlIDs[dxl_cnt], led_indicator[2]); // write BLUE VALUE to DXL Control Table
            
    }
    //unsigned long time_now_millis = millis(); while(millis() < time_now_millis + 500){} // waits 500 milliseconds

return;
}
// =========================================================================================================== //


bool DynamixelProPlusOvidiusShield::blinkDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, const unsigned char *led_indicator, unsigned long interval, int times, Dynamixel2Arduino dxl)
{
 /*
  *  Blinks Dynamixels given the ID numbers and the desired color(as array 3 elements) and time interval/times of blink
  *  No timer interrupt used here. Programm will "pause"!
  */
    bool command_execution_success;
    unsigned long time_now_millis;
    unsigned char turn_off_led[] = {0, 0, 0};

    for(int times_cnt = 0; times_cnt < times; times_cnt++) {
        // leds off
        DynamixelProPlusOvidiusShield::setDynamixelLeds(DxlIDs, DxlIds_size, turn_off_led, dxl);
        // delay
        time_now_millis = millis(); while(millis() < time_now_millis + interval){} // waits interval milliseconds
        // leds on
        DynamixelProPlusOvidiusShield::setDynamixelLeds(DxlIDs, DxlIds_size, led_indicator, dxl);
        // delay
        time_now_millis = millis(); while(millis() < time_now_millis + interval){} // waits interval milliseconds
    }

    return true;
}
// =========================================================================================================== //

// Ping Dynamixels
bool DynamixelProPlusOvidiusShield::pingDynamixels(uint8_t *DxlIDs, int DxlIds_size,unsigned char *error_code, Dynamixel2Arduino dxl) {
/*
 *  Pings Connected Dynamixels given the correct ID number as set using thw Wizard Software
 */
    unsigned char ping_indicator[3];
    bool dxl_comm_result;
    //int dxl_model_number[DXL_ID_SIZE];

    for(int id_count = 0; id_count < DxlIds_size; id_count++){
            dxl_comm_result = dxl.ping(DxlIDs[id_count]);
            if (dxl_comm_result == false)
            {
                //return false;
                delay(500);
            }
            else
            {
                ping_indicator[0] = 0;    //R
                ping_indicator[1] = 255;  //G
                ping_indicator[2] = 0;    //B

                unsigned long ping_interval = 500;
                int ping_times = 2;

                dxl_comm_result = DynamixelProPlusOvidiusShield::blinkDynamixelLeds(DxlIDs, DxlIds_size,ping_indicator, ping_interval, ping_times, dxl);

            }
        
    }

  bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
  if (function_state)
  {
     return true;
  }
  else
  {
      return false;
  }
  
}

// =========================================================================================================== //
bool DynamixelProPlusOvidiusShield::syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t_gp *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl) {
/*
 *  Sends Goal Position to pinged Dynamixels and moves the motor!  Main .ino file must wait depending trajectory execution time!
 */
    // Default for Goal Position
    sw_gp_infos.packet.p_buf = nullptr;
    sw_gp_infos.packet.is_completed = false;
    sw_gp_infos.addr = ADDR_PRO_GOAL_POSITION;
    sw_gp_infos.addr_length = LEN_PRO_GOAL_POSITION;
    sw_gp_infos.p_xels = info_xels_sw_gp;
    sw_gp_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].goal_position = Desired_Goal_Position[id_count];

        info_xels_sw_gp[id_count].id = DxlIDs[id_count];
        info_xels_sw_gp[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].goal_position;
        sw_gp_infos.xel_count++;
    }
    sw_gp_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_gp_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsProfVel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PV, sw_data_t_pv *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl)
{
/*
 *  Sends ProfileVelocity to pinged Dynamixels!  Main .ino file was given trajectory execution time! Profile Velocity is computed 
 *  form MATLAB(trajectory planning) and profile_acceleration is computed based on velocity profile characteristics
 */
    // Default for Goal Position
    sw_pv_infos.packet.p_buf = nullptr;
    sw_pv_infos.packet.is_completed = false;
    sw_pv_infos.addr = ADDR_PRO_PROF_VEL;
    sw_pv_infos.addr_length = LEN_PRO_PROF_VEL;
    sw_pv_infos.p_xels = info_xels_sw_pv;
    sw_pv_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].profile_velocity = Desired_PV[id_count];

        info_xels_sw_pv[id_count].id = DxlIDs[id_count];
        info_xels_sw_pv[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].profile_velocity;
        sw_pv_infos.xel_count++;
    }
    sw_pv_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_pv_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsProfAccel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PA, sw_data_t_pa *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl)
{
/*
 *  Sends ProfileAcceleration to pinged Dynamixels!  Main .ino file was given trajectory execution time! Profile Velocity is computed 
 *  form MATLAB(trajectory planning) and profile_acceleration is computed based on velocity profile characteristics
 */
    // Default for Goal Position
    sw_pa_infos.packet.p_buf = nullptr;
    sw_pa_infos.packet.is_completed = false;
    sw_pa_infos.addr = ADDR_PRO_PROF_ACCEL;
    sw_pa_infos.addr_length = LEN_PRO_PROF_ACCEL;
    sw_pa_infos.p_xels = info_xels_sw_pa;
    sw_pa_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].profile_acceleration = Desired_PA[id_count];

        info_xels_sw_pa[id_count].id = DxlIDs[id_count];
        info_xels_sw_pa[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].profile_acceleration;
        sw_pa_infos.xel_count++;
    }
    sw_pa_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_pa_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsPositionKpGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKp, sw_data_t_pKp *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl) {
/*
 *  Sets Kp gains to all pinged Dynamixels
 */
    // Default for Position Kp
    sw_pKp_infos.packet.p_buf = nullptr;
    sw_pKp_infos.packet.is_completed = false;
    sw_pKp_infos.addr = ADDR_PRO_POSITION_P_GAIN;
    sw_pKp_infos.addr_length = LEN_PRO_POSITION_P_GAIN;
    sw_pKp_infos.p_xels = info_xels_sw_pKp;
    sw_pKp_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].position_Kp_gain = Desired_pKp[id_count];

        info_xels_sw_pKp[id_count].id = DxlIDs[id_count];
        info_xels_sw_pKp[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].position_Kp_gain;
        sw_pKp_infos.xel_count++;
    }
    sw_pKp_infos.is_info_changed = true;

    // Sends data
    dxl.syncWrite(&sw_pKp_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsPositionKiGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKi, sw_data_t_pKi *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl) {
/*
 *  Sets Ki gains to all pinged Dynamixels
 */
    // Default for Position Ki
    sw_pKi_infos.packet.p_buf = nullptr;
    sw_pKi_infos.packet.is_completed = false;
    sw_pKi_infos.addr = ADDR_PRO_POSITION_I_GAIN;
    sw_pKi_infos.addr_length = LEN_PRO_POSITION_I_GAIN;
    sw_pKi_infos.p_xels = info_xels_sw_pKi;
    sw_pKi_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].position_Ki_gain = Desired_pKi[id_count];

        info_xels_sw_pKi[id_count].id = DxlIDs[id_count];
        info_xels_sw_pKi[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].position_Ki_gain;
        sw_pKi_infos.xel_count++;
    }
    sw_pKi_infos.is_info_changed = true;

    // Sends data
    dxl.syncWrite(&sw_pKi_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsPositionKdGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKd, sw_data_t_pKd *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl) {
/*
 *  Sets Kd gains to all pinged Dynamixels
 */
    // Default for Position Kd
    sw_pKd_infos.packet.p_buf = nullptr;
    sw_pKd_infos.packet.is_completed = false;
    sw_pKd_infos.addr = ADDR_PRO_POSITION_D_GAIN;
    sw_pKd_infos.addr_length = LEN_PRO_POSITION_D_GAIN;
    sw_pKd_infos.p_xels = info_xels_sw_pKd;
    sw_pKd_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].position_Kd_gain = Desired_pKd[id_count];

        info_xels_sw_pKd[id_count].id = DxlIDs[id_count];
        info_xels_sw_pKd[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].position_Kd_gain;
        sw_pKd_infos.xel_count++;
    }
    sw_pKd_infos.is_info_changed = true;

    // Sends data
    dxl.syncWrite(&sw_pKd_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPresentPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Present_Position, sr_data_t_pp *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Position from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pp = 128;
    uint8_t user_pkt_buf_pp[user_pkt_buf_cap_pp];
    uint8_t recv_cnt;

    // Default for Present Position
    sr_pp_infos.packet.p_buf = user_pkt_buf_pp;
    sr_pp_infos.packet.buf_capacity = user_pkt_buf_cap_pp;
    sr_pp_infos.packet.is_completed = false;
    sr_pp_infos.addr = ADDR_PRO_PRESENT_POSITION;
    sr_pp_infos.addr_length = LEN_PRO_PRESENT_POSITION;
    sr_pp_infos.p_xels = info_xels_sr_pp;
    sr_pp_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pp[id_count].id = DxlIDs[id_count];
        info_xels_sr_pp[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pp_infos.xel_count++;
    }
    sr_pp_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pp_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_Position[id_count] = SR_Data_Array[id_count].present_position;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPresentVelocity(uint8_t *DxlIDs, int DxlIds_size, int32_t *Present_Velocity, sr_data_t_pv *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Velocity from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pv = 128;
    uint8_t user_pkt_buf_pv[user_pkt_buf_cap_pv];
    uint8_t recv_cnt;

    // Default for Present Velocity
    sr_pv_infos.packet.p_buf = user_pkt_buf_pv;
    sr_pv_infos.packet.buf_capacity = user_pkt_buf_cap_pv;
    sr_pv_infos.packet.is_completed = false;
    sr_pv_infos.addr = ADDR_PRO_PRESENT_VELOCITY;
    sr_pv_infos.addr_length = LEN_PRO_PRESENT_VELOCITY;
    sr_pv_infos.p_xels = info_xels_sr_pv;
    sr_pv_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pv[id_count].id = DxlIDs[id_count];
        info_xels_sr_pv[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pv_infos.xel_count++;
    }
    sr_pv_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pv_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_Velocity[id_count] = SR_Data_Array[id_count].present_velocity;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsMoving(uint8_t *DxlIDs, int DxlIds_size, uint8_t *Moving, sr_data_t_mov *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Moving byte from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_mov = 128;
    uint8_t user_pkt_buf_mov[user_pkt_buf_cap_mov];
    uint8_t recv_cnt;

    // Default for Present Velocity
    sr_mov_infos.packet.p_buf = user_pkt_buf_mov;
    sr_mov_infos.packet.buf_capacity = user_pkt_buf_cap_mov;
    sr_mov_infos.packet.is_completed = false;
    sr_mov_infos.addr = ADDR_PRO_MOVING;
    sr_mov_infos.addr_length = LEN_PRO_MOVING;
    sr_mov_infos.p_xels = info_xels_sr_mov;
    sr_mov_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_mov[id_count].id = DxlIDs[id_count];
        info_xels_sr_mov[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_mov_infos.xel_count++;
    }
    sr_mov_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_mov_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Moving[id_count] = SR_Data_Array[id_count].moving;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPresentCurrent(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_Current, sr_data_t_pc *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Current from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pc = 128;
    uint8_t user_pkt_buf_pc[user_pkt_buf_cap_pc];
    uint8_t recv_cnt;

    // Default for Present Current
    sr_pc_infos.packet.p_buf = user_pkt_buf_pc;
    sr_pc_infos.packet.buf_capacity = user_pkt_buf_cap_pc;
    sr_pc_infos.packet.is_completed = false;
    sr_pc_infos.addr = ADDR_PRO_PRESENT_CURRENT;
    sr_pc_infos.addr_length = LEN_PRO_PRESENT_CURRENT;
    sr_pc_infos.p_xels = info_xels_sr_pc;
    sr_pc_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pc[id_count].id = DxlIDs[id_count];
        info_xels_sr_pc[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pc_infos.xel_count++;
    }
    sr_pc_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pc_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_Current[id_count] = SR_Data_Array[id_count].present_current;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPositionKpGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKpGain, sr_data_t_pKp *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Position-Kp gain from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pKp = 128;
    uint8_t user_pkt_buf_pKp[user_pkt_buf_cap_pKp];
    uint8_t recv_cnt;

    // Default for Present Current
    sr_pKp_infos.packet.p_buf = user_pkt_buf_pKp;
    sr_pKp_infos.packet.buf_capacity = user_pkt_buf_cap_pKp;
    sr_pKp_infos.packet.is_completed = false;
    sr_pKp_infos.addr = ADDR_PRO_POSITION_P_GAIN;
    sr_pKp_infos.addr_length = LEN_PRO_POSITION_P_GAIN;
    sr_pKp_infos.p_xels = info_xels_sr_pKp;
    sr_pKp_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pKp[id_count].id = DxlIDs[id_count];
        info_xels_sr_pKp[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pKp_infos.xel_count++;
    }
    sr_pKp_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pKp_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_PositionKpGain[id_count] = SR_Data_Array[id_count].present_position_Kp_gain;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPositionKiGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKiGain, sr_data_t_pKi *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Position-Ki gain from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pKi = 128;
    uint8_t user_pkt_buf_pKi[user_pkt_buf_cap_pKi];
    uint8_t recv_cnt;

    // Default for Present Current
    sr_pKi_infos.packet.p_buf = user_pkt_buf_pKi;
    sr_pKi_infos.packet.buf_capacity = user_pkt_buf_cap_pKi;
    sr_pKi_infos.packet.is_completed = false;
    sr_pKi_infos.addr = ADDR_PRO_POSITION_I_GAIN;
    sr_pKi_infos.addr_length = LEN_PRO_POSITION_I_GAIN;
    sr_pKi_infos.p_xels = info_xels_sr_pKi;
    sr_pKi_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pKi[id_count].id = DxlIDs[id_count];
        info_xels_sr_pKi[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pKi_infos.xel_count++;
    }
    sr_pKi_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pKi_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_PositionKiGain[id_count] = SR_Data_Array[id_count].present_position_Ki_gain;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::syncGetDynamixelsPositionKdGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKdGain, sr_data_t_pKd *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) 
{
/*
 *  Reads Present Position-Kd gain from pinged Dynamixels
 */
    bool function_state;
    const uint16_t user_pkt_buf_cap_pKd = 128;
    uint8_t user_pkt_buf_pKd[user_pkt_buf_cap_pKd];
    uint8_t recv_cnt;

    // Default for Present Current
    sr_pKd_infos.packet.p_buf = user_pkt_buf_pKd;
    sr_pKd_infos.packet.buf_capacity = user_pkt_buf_cap_pKd;
    sr_pKd_infos.packet.is_completed = false;
    sr_pKd_infos.addr = ADDR_PRO_POSITION_D_GAIN;
    sr_pKd_infos.addr_length = LEN_PRO_POSITION_D_GAIN;
    sr_pKd_infos.p_xels = info_xels_sr_pKd;
    sr_pKd_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        info_xels_sr_pKd[id_count].id = DxlIDs[id_count];
        info_xels_sr_pKd[id_count].p_recv_buf = (uint8_t*)&SR_Data_Array[id_count];
        sr_pKd_infos.xel_count++;
    }
    sr_pKd_infos.is_info_changed = true;

    // Read from motors
    recv_cnt = dxl.syncRead(&sr_pKd_infos);
    if(recv_cnt > 0)
    {
        *error_code = 0;
        function_state = true;

        for(int id_count = 0; id_count<recv_cnt; id_count++)
        {
            Present_PositionKdGain[id_count] = SR_Data_Array[id_count].present_position_Kd_gain;
        }
    }
    else
    {
        function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(error_code, dxl);
    }

    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //
//                                              AUXILIARY FUNCTIONS
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(unsigned char *error_code, Dynamixel2Arduino dxl)
{
    *error_code = dxl.getLastLibErrCode();

    if ( *error_code == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
    
    
}

// =========================================================================================================== //

int32_t DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(float position_in_radians)
{
    int32_t position_in_dxl_pulses;
    //double position_in_radians;

    if (position_in_radians == 0)
    {
        position_in_dxl_pulses = 0;
    }
    else 
    {
        position_in_dxl_pulses = (position_in_radians * DXL_RESOLUTION) / pi;
    }

return position_in_dxl_pulses;
}
  
// =========================================================================================================== //

float DynamixelProPlusOvidiusShield::convertDxlPulses2Radian(int32_t position_in_dxl_pulses)
{
    float position_in_radians;
    
    if (position_in_dxl_pulses == 0)
    {
        position_in_radians = (float) position_in_dxl_pulses;
    }
    else
    {
        position_in_radians = (float) (position_in_dxl_pulses * pi) / DXL_RESOLUTION ;
    }
    
    return position_in_radians;
}

// =========================================================================================================== //

unsigned long DynamixelProPlusOvidiusShield::calculateDxlExecTime(int32_t PV, int32_t PA, int32_t Pos_i, int32_t Pos_f)
{
    /*
     *  Described in Par. 2.4.34 @ http://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/
     *  All units are [pulses] and time in [ms]
     */

    int32_t Dpos = abs(Pos_f - Pos_i);

    unsigned long t1 = (600*PV)/PA;

    unsigned long t2 = (6000000/(2*DXL_RESOLUTION)) * (Dpos / PV);

    unsigned long t3 = t1 + t2;

    return t3;
}

// =========================================================================================================== //
/*
//bool DynamixelProPlusOvidiusShield::calculateProfAccel_preassignedVelTexec(int32_t PV, int32_t & PA, double Ta, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code)
bool DynamixelProPlusOvidiusShield::calculateProfVelAccel_preassignedVelTexec(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, unsigned char * error_code)
{
    //  All units are [pulses] and time in [ms]
    //  Returns final PV(Profile Velocity) and PA(Profile Acceleration) for given p2p properties(Dpos,V,A,Texec) from
    //  1. function: syncPreSetStepperGoalPositionVarStep for Stepper: Ta,Texec
    //  2. user: desired Profile Velocity PV (it will change according to sync constraints)

// -> was already deprecated
    // take only absolute value
    double abs_rel_dpos_dxl[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_rel_dpos_dxl[i] = abs(rel_dpos_dxl[i]);
    }
    
    // define the array type
    Array<double> array = Array<double>(abs_rel_dpos_dxl,DXL_MOTORS);

    // convert Txec[sec] to [millis] Texec_millis!
    int32_t Texec_millis = Texec * 1000;

    max_rel_dpos = array.getMax();
    // convert relative angular displacement [rad] to [pulses]
    max_rel_dpos_pulses = DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(max_rel_dpos);

    uint64_t factor1 = 600 * DXL_RESOLUTION * PV * PV;

    uint64_t factor2 = (DXL_RESOLUTION * PV * Texec_millis ) - (6000000 * max_rel_dpos_pulses) ;

    if (factor2 == 0)
    {
        PA = 3992645;                     // exceeds by +1 the maximum accepted value of AccelerationLimit in ControlTableItem -> JUNK VALUE
        (*error_code) = 15;               // custom error code. max error code was 14 for DynamixelShield  
        return false;
    }
    else
    {
        PA =  factor1 / factor2;
        (*error_code) = 0;
        return true;
    }
// <- was already deprecated
    // Profile Velocity and Acceleration can only be >=0 !!!
    int32_t abs_PV[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_PV[i] = abs(PV[i]);
    }
    
    // define the array type
    Array<int32_t> array_pv = Array<int32_t>(abs_PV,DXL_MOTORS);

    // Define Acceleration Profile w.r.t the slowest motor
    int32_t min_PV = array_pv.getMin(); 
    int min_PV_index = array_pv.getMinIndex();
    
    // convert Ta[sec] to [millis] Ta_millis!
    int32_t Ta_millis = static_cast<int32_t> (Ta * 1000);
    int32_t min_PA = ( 600 * min_PV) / (  Ta_millis );

    // assign relative values for the rest of the dynamixels! 
    double final_PV_double[DXL_MOTORS];
    double final_PA_double[DXL_MOTORS];

    double min_PA_double;
    double min_PV_double;

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        if (i==min_PV_index)
        {
            //PA[i] = abs(min_PA);     // change acceleration of this joint
            PV[i] = min_PV;     // keep min value of velocity unchanged
        }
        else
        {
            // was commented out on 15-2-21 because TOO small acceleration values returned!
            // calculated based on eq. for (ai,vi) in Melchiorri p.66
            //final_PA_double[i] = ( rel_dpos_dxl[i] / ( Ta * (Texec - Ta) ) );       // must be converted to DxlUnits[pulses]
            //double accel =  abs(final_PA_double[i]);
            //PA[i] = DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( accel );

            final_PV_double[i] = ( rel_dpos_dxl[i] / (Texec - Ta) );               // must be converted to DxlUnits[pulses]
            double vel =  abs(final_PV_double[i]);
            PV[i] = DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits( vel );
        }
        
    }

    return true;
}
*/
// =========================================================================================================== //

/*
//bool DynamixelProPlusOvidiusShield::calculateProfAccel_preassignedVelTexec2(int32_t PV, int32_t & PA, double Ta, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code)
bool DynamixelProPlusOvidiusShield::calculateProfVelAccel_preassignedVelTexec2(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, unsigned char * error_code)
{
    //  All units are [pulses] and time in [ms]
    //  Returns final PV(Profile Velocity) and PA(Profile Acceleration) for given from
    //  1. function: syncPreSetStepperGoalPositionVarStep for Stepper: Ta,Texec
    //  2. user: desired Profile Velocity PV (it will change according to sync constraints)
    //  The difference with previous function is that calculates PV,PA based on Texec (not only Ta) and largest displacement!!!
    /// 
    //  * FOR THE TIME I SET PA UNCHANGED!!!! (edited on 12-12-20 must be revised after Xmas!)

    // take only absolute value
    double abs_rel_dpos_dxl[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_rel_dpos_dxl[i] = abs(rel_dpos_dxl[i]);
    }
    
    // define the array type
    Array<double> array = Array<double>(abs_rel_dpos_dxl,DXL_MOTORS);

    // convert Txec[sec] to [millis] Texec_millis!
    int32_t Texec_millis = static_cast<int32_t> (Texec * 1000);

    // specify motor with largest relative displacement
    double max_rel_dpos = array.getMax();
    int max_rel_dpos_index = array.getMaxIndex();

    int32_t max_rel_dpos_PA;
    int32_t max_rel_dpos_PV;

    // convert relative angular displacement [rad] to [pulses]
    int32_t max_rel_dpos_pulses = DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(max_rel_dpos);

    int64_t factor1;
    factor1 = static_cast<int64_t> ( 600 * DXL_RESOLUTION * PV[max_rel_dpos_index] * PV[max_rel_dpos_index] );
  
    int64_t factor2;
    factor2 = static_cast<int64_t> ( (DXL_RESOLUTION * PV[max_rel_dpos_index] * Texec_millis ) - (6000000 * max_rel_dpos_pulses) );

    if (factor2 == 0)
    {
        max_rel_dpos_PA = static_cast<int32_t>(3992645);        // exceeds by +1 the maximum accepted value of AccelerationLimit in ControlTableItem -> JUNK VALUE
        (*error_code) = 15;               // custom error code. max error code was 14 for DynamixelShield  
    }
    else
    {
        max_rel_dpos_PA = static_cast<int32_t> (factor1 / factor2);
        (*error_code) = 0;
    }

    double final_PV_double[DXL_MOTORS];
    double final_PA_double[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        if (i==max_rel_dpos_index)
        {
            PA[i] = abs(max_rel_dpos_PA);     // change acceleration of this joint
            PV[i] = abs(PV[i]);               // keep value of velocity unchanged
        }
        else
        {
            // calculated based on eq. for (ai,vi) in Melchiorri p.66
            final_PA_double[i] = ( abs_rel_dpos_dxl[i] / ( Ta * (Texec - Ta) ) );       // must be converted to DxlUnits[pulses]
            PA[i] = DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( abs(final_PA_double[i]));

            final_PV_double[i] = ( abs_rel_dpos_dxl[i] / (Texec - Ta) );               // must be converted to DxlUnits[pulses]
            PV[i] = DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits(abs(final_PV_double[i]));
        }
        
    }

    if ( *error_code == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
*/
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::calculateProfVelAccel_preassignedVelTexec3(float * JointVel, float * JointAccel, float * rel_dpos, float * Ta, float * Texec, float * JointVelLim, float * JointAccelLim, unsigned char * error_code)
{
    /*
     * Based on calculateProfVelAccel_preassignedVelTexec. Copied at 15/2/21. Must be hardware tested. 
     * Stp+Dxl values are used to calculate sync motion based on Melchiorri.
     * To implement this function the serial function execution senquence must change!
     * 
     * Overview:
     * 1. Compute abs(rel_dpos)
     * 2. Calculate Ai,Vi
     * 3. Check (Ai,Vi) > (Alim,Vlim)
     * 4.1.1 (NO) Calculate stepper profile steps + Calculate Dxl Pulses
     * 4.1.2      Execute p2p
     * 4.2.1 (YES) Find saturated motors
     * 4.2.2       Calculate Ta,T for saturated motors and assign the max as the new Texec
     * 4.2.3       go to 2
     */

    int vel_saturated_index = -1;
    int accel_saturated_index = -1;
    bool found_saturated_vel = false;
    bool found_saturated_accel = false;
    float Texec_new[nDoF];

    // 1.
    float abs_rel_dpos[nDoF];

    for (size_t i = 0; i < nDoF; i++)
    {
        abs_rel_dpos[i] = abs(rel_dpos[i]);
    }
    
    // 2.
    for (size_t i = 0; i < nDoF; i++)
    {
        JointAccel[i] = abs_rel_dpos[i] / ( (*Ta) * ( (*Texec) - (*Ta) ) );
        JointVel[i]   = abs_rel_dpos[i] / ( (*Texec) - (*Ta) ) ;
    }

    // 3.
    for (size_t i = 0; i < nDoF; i++)
    {
        if ( JointVel[i] > JointVelLim[i] )
        {
            vel_saturated_index  = i;
            JointVel[i] = JointVelLim[i];
            found_saturated_vel = true;
        }
        else
        {
            found_saturated_vel = false;
        }
        
        if ( JointAccel[i] > JointAccelLim[i] )
        {
            accel_saturated_index  = i;
            JointAccel[i] = JointAccelLim[i];
            found_saturated_accel = true;
        }
        else
        {
            found_saturated_accel = false;
        }  
    }
    // Now new (Vi,Ai) are computed for all motors. For all (Vi,Ai) the new (Texec) is calculated and the biggest is selected as p2p sync execution time
    for (size_t i = 0; i < nDoF; i++)
    {
        Texec_new[i] =  ( (abs_rel_dpos[i] * JointAccel[i] ) + ( JointVel[i] * JointVel[i] ) ) / ( JointAccel[i] * JointVel[i] );
    }

    Array<float> array_Texec_new = Array<float>(Texec_new,nDoF);
    float max_Texec_new          = array_Texec_new.getMax(); 
    int max_Texec_new_index       = array_Texec_new.getMaxIndex();

    if (max_Texec_new < (*Texec))
    {
        // that means wrong calculations! Saturation must lead to bigger execution time
        (*error_code) = EXPECTED_HIGHER_Texec;
        (*Texec) = max_Texec_new;
        (*Ta)    = 0.40f *  (*Texec);
    }
    else
    {
        // T,Ta changed due to saturation
        (*error_code) = NO_DXL_ERROR;
        (*Texec) = Texec_new[max_Texec_new_index];
        //(*Ta)    = (double) ( ( 1 / ACCEL_WIDTH_DENOM) * (*Texec)); // returns 0!
        (*Ta)    = 0.40f *  (*Texec);
    }

    if ( *error_code == NO_DXL_ERROR)
    {
        return true;
    }
    else
    {
        return false;
    }

}
// =========================================================================================================== //

float DynamixelProPlusOvidiusShield::convertDxlVelUnits2RadPsec(int32_t dxlVunit)
{
    // 1 [dxlVunit] -> 0.01 [rev/min] -> 0.00104719755 [rad/sec]
    float dxlVunit_radsec;

    dxlVunit_radsec = static_cast<float> (0.001047 * dxlVunit);

    return dxlVunit_radsec;
}

int32_t DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits(float dxlVunit_radsec)
{
    // 1 [rad/sec] -> 9.5493 [rev/min] -> 954.93 [0.01 rev/min]
    int32_t dxlVunit;

    dxlVunit = static_cast<int32_t> (954.93 * dxlVunit_radsec); 

    return dxlVunit;
}

float DynamixelProPlusOvidiusShield::convertDxlAccelUnits2RadPsec2(int32_t dxlAunit)
{
    // 1 [dxlAunit] -> 1 [rev/min2] ->  0.00174532925199433 [rad/sec2]
    float dxlAunit_radsec2 = static_cast<float> (0.001047 * dxlAunit);

    return dxlAunit_radsec2;
}

int32_t DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( float dxlAunit_radsec2)
{
    // 1 [rad/sec2] -> 9.5493 [rev/min2]
    int32_t dxlAunit = static_cast<int32_t> (9.5493 * dxlAunit_radsec2);

    return dxlAunit;
}
// =========================================================================================================== //
