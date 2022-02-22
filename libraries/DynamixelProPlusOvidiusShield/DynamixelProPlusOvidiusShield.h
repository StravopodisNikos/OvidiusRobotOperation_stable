 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#ifndef DynamixelProPlusOvidiusShield_h
#define DynamixelProPlusOvidiusShield_h

#include "Arduino.h"
//#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <Array.h>

using namespace std;
using namespace ControlTableItem;

extern uint8_t dxl_id[];
const int DXL_ID_SIZE = 2;                            // Must be configured based on declaration in test_metamorphic_manipulator.ino
const uint16_t user_pkt_buf_cap = 128;

extern int dxl_comm_result;                                                               // Communication result
extern bool dxl_addparam_result;                                                          // addParam result
extern bool dxl_getdata_result;                                                           // GetParam result
extern uint8_t dxl_error; 
extern uint16_t dxl_model_number[];
extern int32_t dxl_present_position[];
extern int32_t dxl_present_velocity[];
extern int32_t dxl_goal_position[];
extern uint8_t dxl_ledBLUE_value[];
extern uint8_t dxl_ledGREEN_value[];
extern uint8_t dxl_ledRED_value[];
extern int32_t position_in_dxl_pulses;
extern double position_in_radians;
extern unsigned char ping_indicator[];
extern int32_t dxl_vel_limit[DXL_ID_SIZE];
extern int32_t dxl_accel_limit[DXL_ID_SIZE];
extern int32_t dxl_prof_vel[DXL_ID_SIZE];
extern int32_t dxl_prof_accel[DXL_ID_SIZE];
extern int16_t dxl_pKp[];
extern int16_t dxl_pKi[];
extern int16_t dxl_pKd[];

// Data Packaging
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t_gp;

typedef struct sw_data_pv{
  int32_t profile_velocity;
} __attribute__((packed)) sw_data_t_pv;

typedef struct sw_data_pa{
  int32_t profile_acceleration;
} __attribute__((packed)) sw_data_t_pa;

typedef struct sw_data_pKp{
  int16_t position_Kp_gain;
} __attribute__((packed)) sw_data_t_pKp;

typedef struct sw_data_pKi{
  int16_t position_Ki_gain;
} __attribute__((packed)) sw_data_t_pKi;

typedef struct sw_data_pKd{
  int16_t position_Kd_gain;
} __attribute__((packed)) sw_data_t_pKd;

typedef struct sr_data_pp{
  int32_t present_position;
} __attribute__((packed)) sr_data_t_pp;

typedef struct sr_data_pv{
  int32_t present_velocity;
} __attribute__((packed)) sr_data_t_pv;

typedef struct sr_data_mov{
  bool moving;
} __attribute__((packed)) sr_data_t_mov;

typedef struct sr_data_pc{
  int16_t present_current;
} __attribute__((packed)) sr_data_t_pc;

typedef struct sr_data_pKp{
  int16_t present_position_Kp_gain;
} __attribute__((packed)) sr_data_t_pKp;

typedef struct sr_data_pKi{
  int16_t present_position_Ki_gain;
} __attribute__((packed)) sr_data_t_pKi;

typedef struct sr_data_pKd{
  int16_t present_position_Kd_gain;
} __attribute__((packed)) sr_data_t_pKd;

// Data packaging for communication with CustomStepperOvidiusShield
struct DXL_PP_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int32_t * dxl_pp;
  sr_data_pp * SR_pp;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_PV_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int32_t * dxl_pv;
  sr_data_pv * SR_pv;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_MOV_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  uint8_t * dxl_mov;
  sr_data_mov * SR_mov;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_PC_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int16_t * dxl_pc;
  sr_data_pc * SR_pc;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_PKP_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int16_t * dxl_pKp;
  sr_data_pKp * SR_pKp;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_PKI_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int16_t * dxl_pKi;
  sr_data_pKi * SR_pKi;
  Dynamixel2Arduino dxl2ard_obj;
};

struct DXL_PKD_PACKET
{
  uint8_t * Dxl_ids;
  int       Dxl_ids_size;
  int16_t * dxl_pKd;
  sr_data_pKd * SR_pKd;
  Dynamixel2Arduino dxl2ard_obj;
};

// Function Nomenclature

/*
 *      Functions with syncSet{Name}: Use GroupSyncWrite with Direct Addressing of only 1 control table item
 *      Functions with syncSet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncWrite with Indirect Addressing of n control table items
 *      
 *      Functions with syncGet{Name}: Use GroupSyncRead with Direct Addressing of only 1 control table item
 *      Functions with syncGet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncRead with Indirect Addressing of n control table items
 */

class DynamixelProPlusOvidiusShield
{ 
 public:
    //int led_change = 0;                         // global value to see led colours changing after each movement completes(just for simple visualization)
    
    // Primary Functions
    DynamixelProPlusOvidiusShield(uint8_t *DxlIDs);

    bool setDynamixelsTorqueON(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl);

    bool setDynamixelsTorqueOFF(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl);

    void setDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, const unsigned char *led_indicator, Dynamixel2Arduino dxl);

    bool blinkDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, const unsigned char *led_indicator, unsigned long interval, int times, Dynamixel2Arduino dxl);

    bool pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, unsigned char *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t_gp *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsProfVel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PV, sw_data_t_pv *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsProfAccel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PA, sw_data_t_pa *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

	bool syncSetDynamixelsPositionKpGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKp, sw_data_t_pKp *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

	bool syncSetDynamixelsPositionKiGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKi, sw_data_t_pKi *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

	bool syncSetDynamixelsPositionKdGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Desired_pKd, sw_data_t_pKd *SW_Data_Array,unsigned char *error_code, Dynamixel2Arduino dxl);

    bool syncGetDynamixelsPresentPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Present_Position, sr_data_t_pp *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl); 

    bool syncGetDynamixelsPresentVelocity(uint8_t *DxlIDs, int DxlIds_size, int32_t *Present_Velocity, sr_data_t_pv *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl); 

    bool syncGetDynamixelsMoving(uint8_t *DxlIDs, int DxlIds_size, uint8_t *Moving, sr_data_t_mov *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl) ;

    bool syncGetDynamixelsPresentCurrent(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_Current, sr_data_t_pc *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl); 

	bool syncGetDynamixelsPositionKpGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKpGain, sr_data_t_pKp *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl);

	bool syncGetDynamixelsPositionKiGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKiGain, sr_data_t_pKi *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl); 

	bool syncGetDynamixelsPositionKdGain(uint8_t *DxlIDs, int DxlIds_size, int16_t *Present_PositionKdGain, sr_data_t_pKd *SR_Data_Array, unsigned char *error_code, Dynamixel2Arduino dxl); 

    // AUXILIARY FUNCTIONS
    bool check_If_OK_for_Errors(unsigned char *error_code, Dynamixel2Arduino dxl);

    int32_t convertRadian2DxlPulses(float position_in_radians);

    float convertDxlPulses2Radian(int32_t position_in_dxl_pulses);

    unsigned long calculateDxlExecTime(int32_t PV, int32_t PA, int32_t Pos_i, int32_t Pos_f);

    //bool calculateProfAccel_preassignedVelTexec(int32_t PV, int32_t & PA, double Texec, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code);
    //bool calculateProfVelAccel_preassignedVelTexec(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, unsigned char * error_code);

    //bool calculateProfVelAccel_preassignedVelTexec2(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, unsigned char * error_code);

    bool calculateProfVelAccel_preassignedVelTexec3(float * JointVel, float * JointAccel, float * rel_dpos, float * Ta, float * Texec, float * JointVelLim, float * JointAccelLim, unsigned char * error_code);

    float convertDxlVelUnits2RadPsec(int32_t dxlVunit);

    int32_t convertRadPsec2DxlVelUnits(float dxlVunit_radsec);

    float convertDxlAccelUnits2RadPsec2(int32_t dxlAunit);

    int32_t convertRadPsec2_2_DxlAccelUnits( float dxlAunit_radsec2);

    //DXL_PP_PACKET dxl_pp_packet;

    //DXL_PV_PACKET dxl_pv_packet;

private:


};

 #endif
