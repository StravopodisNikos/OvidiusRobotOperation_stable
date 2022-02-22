void setup_libraries_packets()
{
  /*
   *   DYNAMIXELS
   */
  // UPDATE ROBOT CONFIGURATION PACKET
  dxl_pp_packet.Dxl_ids       = dxl_id;
  dxl_pp_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pp_packet.dxl_pp        = dxl_present_position;
  dxl_pp_packet.SR_pp         = sr_data_array_pp;
  dxl_pp_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pp_packet = &dxl_pp_packet;
  
  // UPDATE ROBOT ANGULAR VELOCITY PACKET
  dxl_pv_packet.Dxl_ids       = dxl_id;
  dxl_pv_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pv_packet.dxl_pv        = dxl_present_velocity;
  dxl_pv_packet.SR_pv         = sr_data_array_pv;
  dxl_pv_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pv_packet = &dxl_pv_packet;

  // UPDATE DXL MOVING PACKET
  dxl_mov_packet.Dxl_ids      = dxl_id;
  dxl_mov_packet.Dxl_ids_size = sizeof(dxl_id);
  dxl_mov_packet.dxl_mov      = dxl_moving;
  dxl_mov_packet.SR_mov       = sr_data_array_mov;
  dxl_mov_packet.dxl2ard_obj  = dxl;

  PTR_2_dxl_mov_packet = &dxl_mov_packet;

  // UPDATE DXL CURRENT MEASUREMENT
  dxl_pc_packet.Dxl_ids       = dxl_id;
  dxl_pc_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pc_packet.dxl_pc        = dxl_present_current;
  dxl_pc_packet.SR_pc         = sr_data_array_pc;
  dxl_pc_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pc_packet = &dxl_pc_packet;

  // UPDATE DXL Position Kp Gain
  dxl_pKp_packet.Dxl_ids       = dxl_id;
  dxl_pKp_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pKp_packet.dxl_pKp       = dxl_pKp;
  dxl_pKp_packet.SR_pKp        = sr_data_array_pKp;
  dxl_pKp_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pKp_packet = &dxl_pKp_packet;

   // UPDATE DXL Position Ki Gain
  dxl_pKi_packet.Dxl_ids       = dxl_id;
  dxl_pKi_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pKi_packet.dxl_pKi       = dxl_pKi;
  dxl_pKi_packet.SR_pKi        = sr_data_array_pKi;
  dxl_pKi_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pKi_packet = &dxl_pKi_packet;

  // UPDATE DXL Position Kd Gain
  dxl_pKd_packet.Dxl_ids       = dxl_id;
  dxl_pKd_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pKd_packet.dxl_pKd       = dxl_pKd;
  dxl_pKd_packet.SR_pKd        = sr_data_array_pKd;
  dxl_pKd_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pKd_packet = &dxl_pKd_packet;
  
  return;
}
