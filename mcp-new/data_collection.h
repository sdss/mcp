/*=============================================================================
	data_collection.h

=============================================================================*/
struct TM_M68K {
	short prev_encoder;
	short current_vel;
	long actual_position;
	unsigned short time_fractional;
	unsigned long time;
	short jerk_fractional;
	long jerk;
	short acceleration_fractional;
	long acceleration;
	short velocity_fractional;
	long velocity;
	long position_fractional;
	long position;
	short master_pos;
	short ratio;
	short ratio2;
	short ratio3;
	short ratio4;
	short ratio5;
	short ratio6;
	short cv_1;
	short error;
	short trigger;
	short trigger2;
	short trigger3;
	short trigger4;
	short trigger5;
	long latch;
	short unknown1;
	short unknown2;
	short unknown3;
	short unknown4;
	short unknown5;
	short unknown6;
	short unknown7;
	short unknown8;
	short voltage;
	short unknown9;
	short prev_encoder2;
	short current_vel2;
	long actual_position2;
	short status;
	short errcnt;
};
struct TM {
	short prev_encoder;
	short current_vel;
	short actual_position_hi;		/* ALENC1, AZENC1 (upper)	*/
	unsigned short actual_position_lo;	/* ALENC1, AZENC1 (lower)	*/
	unsigned short time_fractional;
	unsigned short time_hi;
	unsigned short time_lo;
	short jerk_fractional;
	short jerk_hi;
	unsigned short jerk_lo;
	short acceleration_fractional;
	short acceleration_hi;
	unsigned short acceleration_lo;
	short velocity_fractional;
	short velocity_hi;
	unsigned short velocity_lo;
	short position_fractional_hi;
	unsigned short position_fractional_lo;
	short position_hi;			/* AZCPOS, ALCPOS, ROCPOS (upper) */
	unsigned short position_lo;		/* AZCPOS, ALCPOS, ROCPOS (lower) */
	short master_pos;
	short ratio;
	short ratio2;
	short ratio3;
	short ratio4;
	short ratio5;
	short ratio6;
	short cv_1;
	short error;
	short trigger;
	short trigger2;
	short trigger3;
	short trigger4;
	short trigger5;
	short latch_hi;
	unsigned short latch_lo;
	short unknown1;
	short unknown2;
	short unknown3;
	short unknown4;
	short unknown5;
	short unknown6;
	short unknown7;
	short unknown8;
	short voltage;				/* AZCVOLT, ALCVOLT, ROCVOLT */
	short unknown9;
	short prev_encoder2;
	short current_vel2;
	short actual_position2_hi;		/* ALENC2, AZENC2 (upper)	*/
	unsigned short actual_position2_lo;	/* ALENC2, AZENC2 (lower)	*/
	short status;
	short errcnt;
};
struct IL {
	short strain_gage;
	short pos;
	short enable;
	short status;
};
struct CW {
	short pos;
	short status;
};
struct PVT_M68K {
	long position;
	long velocity;
	long time;
};
struct PVT {
	short position_hi;
	unsigned short position_lo;
	short velocity_hi;
	unsigned short velocity_lo;
	short time_hi;
	unsigned short time_lo;
};

typedef struct {		/* msw bit - 32 i:1/15, i.e. 0x8000 0000 */
	unsigned rack_0_grp_0_bit_15 : 1;
	unsigned az_bump_ccw : 1;		/* AZCCWC	*/
	unsigned az_bump_cw : 1;		/* AZCWC	*/
	unsigned rack_0_grp_0_bit_12 : 1;
	unsigned ops_cart_in_pos : 1;
	unsigned fiber_cart_pos2 : 1;	/* cart locked in second position - put */
	unsigned fiber_cart_pos1 : 1;	/* cart locked in first position - get */
	unsigned inst_lift_low_force : 1;	/* */
	unsigned inst_lift_high_force : 1;	/* */
	unsigned inst_lift_man : 1;	/* lift is in manual or local mode */
	unsigned inst_lift_dn : 1;	/* lift is in down position */
	unsigned inst_lift_sw4 : 1;	/* fiber cartridge on cart floor */
	unsigned inst_lift_sw3 : 1;	/* dummy cartridge on cart floor */
	unsigned inst_lift_sw2 : 1;	/* engaged with clamping force */
	unsigned inst_lift_sw1 : 1;	/* kinematic mounts engaged */
	unsigned inst_lift_pump_on : 1;	/* pump status */

	unsigned low_lvl_light_req : 1;	/* msw bit - 15 I:1/31, i.e. 0x8000 */
	unsigned rack_0_grp_1_bit_14 : 1;
	unsigned rack_0_grp_1_bit_13 : 1;
	unsigned rack_0_grp_1_bit_12 : 1;
	unsigned rack_0_grp_1_bit_11 : 1;
	unsigned rack_0_grp_1_bit_10 : 1;
	unsigned rack_0_grp_1_bit_9 : 1;
	unsigned rack_0_grp_1_bit_8 : 1;
	unsigned rack_0_grp_1_bit_7 : 1;
	unsigned optical_bench_cls : 1;
	unsigned optical_bench_opn : 1;
	unsigned ops_cart_in_house : 1;
	unsigned dog_house_door_cls : 1;
	unsigned dog_house_door_opn : 1;
	unsigned dog_house_ccw_pad : 1;
	unsigned dog_house_cw_pad : 1;
}IF1_L0;
typedef struct {
	unsigned sad_latch_opn_cmd : 1;
	unsigned sad_latch_cls_cmd : 1;
	unsigned sec_latch_opn_cmd : 1;
	unsigned sec_latch_cls_cmd : 1;
	unsigned pri_latch_opn_cmd : 1;
	unsigned pri_latch_cls_cmd : 1;
	unsigned rack_1_grp_0_bit_9 : 1;
	unsigned rack_1_grp_0_bit_8 : 1;
	unsigned sad_man_valve_cls : 1;
	unsigned sec_man_valve_cls : 1;
	unsigned inst_man_valve_cls : 1;
	unsigned ilcb_pres_good : 1;
	unsigned rot_pos_370_ccw : 1;		/* ROCCWSL	*/
	unsigned rot_neg_190_cw : 1;		/* ROCWSL	*/
	unsigned rot_inst_chg_b : 1;
	unsigned rot_inst_chg_a : 1;		/* ROICHG	*/

	unsigned : 16;
}IF1_L4;
typedef struct {
	unsigned rack_1_grp_4_bit_15 : 1;
	unsigned rack_1_grp_4_bit_14 : 1;
	unsigned rack_1_grp_4_bit_13 : 1;
	unsigned rack_1_grp_4_bit_12 : 1;
	unsigned rack_1_grp_4_bit_11 : 1;
	unsigned rack_1_grp_4_bit_10 : 1;
	unsigned rack_1_grp_4_bit_9 : 1;
	unsigned rack_1_grp_4_bit_8 : 1;
	unsigned rack_1_grp_4_bit_7 : 1;
	unsigned rack_1_grp_4_bit_6 : 1;
	unsigned rack_1_grp_4_bit_5 : 1;
	unsigned rack_1_grp_4_bit_4 : 1;
	unsigned sec_mir_force_limits : 1;
	unsigned alt_bump_dn : 1;
	unsigned alt_bump_up : 1;
	unsigned purge_air_pressur_sw : 1;

	unsigned : 16;
}IF1_L6;
typedef struct {
	unsigned rack_2_grp_0_bit_15 : 1;
	unsigned spec_autofill_on : 1;
  	unsigned spec_lens2 : 1;
  	unsigned spec_lens1 : 1;
  	unsigned inst_id3_4 : 1;
  	unsigned inst_id3_3 : 1;
  	unsigned inst_id3_2 : 1;
  	unsigned inst_id3_1 : 1;
  	unsigned inst_id2_4 : 1;
  	unsigned inst_id2_3 : 1;
  	unsigned inst_id2_2 : 1;
  	unsigned inst_id2_1 : 1;
  	unsigned inst_id1_4 : 1;
  	unsigned inst_id1_3 : 1;
  	unsigned inst_id1_2 : 1;
  	unsigned inst_id1_1 : 1;

    	unsigned safety_latch2_cls : 1;
    	unsigned safety_latch2_opn : 1;
    	unsigned safety_latch1_cls : 1;
    	unsigned safety_latch1_opn : 1;
    	unsigned sec_latch3_cls : 1;
    	unsigned sec_latch3_opn : 1;
    	unsigned sec_latch2_cls : 1;
    	unsigned sec_latch2_opn : 1;
    	unsigned sec_latch1_cls : 1;
    	unsigned sec_latch1_opn : 1;
    	unsigned pri_latch3_cls : 1;
    	unsigned pri_latch3_opn : 1;
    	unsigned pri_latch2_cls : 1;
    	unsigned pri_latch2_opn : 1;
    	unsigned pri_latch1_cls : 1;
    	unsigned pri_latch1_opn : 1;
}IF1_L8;
typedef struct {
	unsigned rack_2_grp_2_bit_15 : 1;
	unsigned rack_2_grp_2_bit_14 : 1;
	unsigned slit_head_2_in_place : 1;
	unsigned slit_head_latch2_ext : 1;
	unsigned slit_head_door2_cls : 1;
	unsigned slit_head_door2_opn : 1;
	unsigned slit_head_1_in_place : 1;
	unsigned slit_head_latch1_ext : 1;
	unsigned slit_head_door1_cls : 1;
	unsigned slit_head_door1_opn : 1;
	unsigned sad_mount2 : 1;
	unsigned sad_mount1 : 1;
	unsigned sad_latch2_cls : 1;
	unsigned sad_latch2_opn : 1;
	unsigned sad_latch1_cls : 1;
	unsigned sad_latch1_opn : 1;
	
	unsigned : 16;
}IF1_L9;
typedef struct {
	unsigned : 16;
	
	unsigned rack_3_grp_1_bit_15 : 1;
	unsigned rack_3_grp_1_bit_14 : 1;
	unsigned rack_3_grp_1_bit_13 : 1;
	unsigned rack_3_grp_1_bit_12 : 1;
	unsigned rack_3_grp_1_bit_11 : 1;
	unsigned rack_3_grp_1_bit_10 : 1;
	unsigned man_lift_dn : 4;
	unsigned man_lift_up : 4;
	unsigned inst_lift_auto : 1;
	unsigned : 1;
}IF1_L12;
typedef struct {
	unsigned leaf_8_closed_stat : 1;
	unsigned leaf_8_open_stat : 1;
	unsigned leaf_7_closed_stat : 1;
	unsigned leaf_7_open_stat : 1;
	unsigned leaf_6_closed_stat : 1;
	unsigned leaf_6_open_stat : 1;
	unsigned leaf_5_closed_stat : 1;
	unsigned leaf_5_open_stat : 1;
	unsigned leaf_4_closed_stat : 1;
	unsigned leaf_4_open_stat : 1;
	unsigned leaf_3_closed_stat : 1;
	unsigned leaf_3_open_stat : 1;
	unsigned leaf_2_closed_stat : 1;
	unsigned leaf_2_open_stat : 1;
	unsigned leaf_1_closed_stat : 1;
	unsigned leaf_1_open_stat : 1;
	
	unsigned rack_3_grp_3_bit_15 : 1;
	unsigned rack_3_grp_3_bit_14 : 1;
	unsigned rack_3_grp_3_bit_13 : 1;
	unsigned rack_3_grp_3_bit_12 : 1;
	unsigned hgcd_4_stat : 1;
	unsigned hgcd_3_stat : 1;
	unsigned hgcd_2_stat : 1;
	unsigned hgcd_1_stat : 1;
	unsigned ne_4_stat : 1;
	unsigned ne_3_stat : 1;
	unsigned ne_2_stat : 1;
	unsigned ne_1_stat : 1;
	unsigned ff_4_stat : 1;
	unsigned ff_3_stat : 1;
	unsigned ff_2_stat : 1;
	unsigned ff_1_stat : 1;
}IF1_L13;
typedef struct {
	unsigned rack_3_grp_4_bit_15 : 1;
	unsigned rack_3_grp_4_bit_14 : 1;
	unsigned rack_3_grp_4_bit_13 : 1;
	unsigned rack_3_grp_4_bit_12 : 1;
	unsigned rack_3_grp_4_bit_11 : 1;
	unsigned rack_3_grp_4_bit_10 : 1;
	unsigned rack_3_grp_4_bit_9 : 1;
	unsigned rack_3_grp_4_bit_8 : 1;
	unsigned rack_3_grp_4_bit_7 : 1;
	unsigned rack_3_grp_4_bit_6 : 1;
	unsigned ff_man_cont_on : 1;
	unsigned man_hgcd_lamp_on_cmd : 1;
	unsigned man_ne_lamp_on_cmd : 1;
	unsigned man_ff_lamp_on_cmd : 1;
	unsigned man_ff_scrn_en_cmd : 1;
	unsigned man_ff_scrn_opn_cmd : 1;
	
	unsigned : 16;
}IF1_L14;
struct I1 {
	IF1_L0 il0;
	unsigned long undefined1;
	unsigned long undefined2;
	unsigned long undefined3;
	IF1_L4 il4;
	unsigned long undefined5;
	IF1_L6 il6;
	unsigned long undefined7;
	IF1_L8 il8;
	IF1_L9 il9;
	unsigned long undefined10;
	unsigned long undefined11;
	IF1_L12 il12;
	IF1_L13 il13;
	IF1_L14 il14;
	unsigned long undefined15;
};
typedef struct {
	unsigned dcm_status : 16;

	unsigned : 10;
	unsigned wind_alt_perm : 1;
	unsigned wind_az_perm : 1;
	unsigned wind_alt1_fault : 1;
	unsigned wind_az3_fault : 1;
	unsigned wind_az2_fault : 1;
	unsigned wind_az1_fault : 1;
}IF2_L0;
struct I2 {
	IF2_L0 il0;
	short az_lvdt_error;
	short alt_lvdt_error;				/* ALWSPOS	*/
	short az_primary_drv;
	short az_feed_forward_drv;
	short alt_primary_drv;
	unsigned short short_undefined7;
	unsigned long undefined4;
	unsigned long undefined5;
	unsigned long undefined6;
	unsigned long undefined7;
	unsigned long undefined8;
	unsigned long undefined9;
	unsigned long undefined10;
	unsigned long undefined11;
	unsigned long undefined12;
	unsigned long undefined13;
	unsigned long undefined14;
	unsigned long undefined15;
};
struct I3 {
	short az_1_voltage;					/* AZMTRV1  */
	short az_1_current;					/* AZMTRC1  */
	short az_2_voltage;					/* AZMTRV2  */
	short az_2_current;					/* AZMTRC2  */
	short alt_1_voltage;				/* ALMTRV1	*/
	short alt_1_current;				/* ALMTRC1	*/
	short alt_2_voltage;				/* ALMTRV2	*/
	short alt_2_current;				/* ALMTRC2	*/
};
struct I4 {
	short alt_position;
	short rot_1_voltage;				/* ROMTRV	*/
	short rot_1_current;				/* ROMTRC	*/
	short umbilical_dist;
	short inst_lift_force;
	short inst_lift_dist;
	short i_4_analog_6_spare;
	short i_4_analog_7_spare;
};
struct I5 {
	short counterweight_1_pos;
	short counterweight_2_pos;			/* ROMTRV	*/
	short counterweight_3_pos;			/* ROMTRC	*/
	short counterweight_4_pos;
	short i_5_analog_4_spare;
	short i_5_analog_5_spare;
	short i_5_analog_6_spare;
	short i_5_analog_7_spare;
};
typedef struct {
	unsigned spare_s2_c7 : 1;
	unsigned nw_fork_stop : 1;
	unsigned s_wind_stop : 1;
	unsigned w_lower_stop : 1;
	unsigned e_lower_stop : 1;
	unsigned s_lower_stop : 1;
	unsigned n_lower_stop : 1;
	unsigned w_rail_stop : 1;
	unsigned s_rail_stop : 1;
	unsigned n_rail_stop : 1;
	unsigned n_fork_stop : 1;
	unsigned n_wind_stop : 1;
	unsigned fiber_signal_loss : 1;
	unsigned spare_s1_c2 : 1;
	unsigned cr_stop : 1;
	unsigned tcc_stop : 1;
	
	unsigned az_stow_3b : 1;
	unsigned wind_az_plc_perm_in : 1;
	unsigned az_plc_perm_in : 1;
	unsigned wind_az_mtr_perm_in : 1;
	unsigned az_mtr2_perm_in : 1;
	unsigned az_mtr1_perm_in : 1;
	unsigned az_mtr_ccw_perm_in : 1;
	unsigned az_mtr_cw_perm_in : 1;
	unsigned az_stow_3a : 1;
	unsigned wind_alt_plc_perm_in : 1;
	unsigned alt_plc_perm_in : 1;
	unsigned wind_alt_mtr_perm_in : 1;
	unsigned alt_mtr2_perm_in : 1;
	unsigned alt_mtr1_perm_in : 1;
	unsigned alt_mtr_dn_perm_in : 1;
	unsigned alt_mtr_up_perm_in : 1;
} IF6_L0;
struct I6 {
	IF6_L0	il0;
};
typedef struct {
	unsigned az_stow_1b : 1;
	unsigned az_stow_1a : 1;
	unsigned alt_grt_18d6_limit_1 : 1;		/* ALP20 */
	unsigned az_109_131_limit_1 : 1;
	unsigned bldg_on_alt : 1;				/* AZCWHL */
	unsigned alt_les_90d5_limit : 1;		/* ALP100 */
	unsigned alt_locking_pin_out : 1;		/* ALSTOW */
	unsigned alt_grt_0d3_limit : 1;			/* ALP_2 */
	unsigned alt_les_2d5_limit : 1;			/* AZCWHL */
	unsigned hatch_cls : 1;					/* AZCCWSL */
	unsigned rot_plc_perm_in : 1;			/* AZCWSL */
	unsigned bldg_perm_in : 1;
	unsigned spare_s5_c3 : 1;
	unsigned rot_mtr_perm_in : 1;
	unsigned rot_mtr_ccw_perm_in : 1;
	unsigned rot_mtr_cw_perm_in : 1;

	unsigned spare_s8_c7 : 1;
	unsigned spare_s8_c6 : 1;
	unsigned az_pos_410b_ccw : 1;			/* AZCCWHL	*/
	unsigned az_neg_170b_cw : 1;			/* AZCWHL	*/
	unsigned az_pos_410a_ccw : 1;			/* AZCCWSL	*/
	unsigned az_neg_170a_cw : 1;			/* AZCWSL	*/
	unsigned az_dir_ccw : 1;
	unsigned az_dir_cw : 1;
	unsigned alt_velocity_limit : 1;
	unsigned alt_slip : 1;
	unsigned alt_grt_18d6_limit_2 : 1;		/* AZCWHL */
	unsigned deg_15_stop_ext : 1;
	unsigned az_stow_2b : 1;
	unsigned az_stow_2a : 1;
	unsigned bldg_clear_alt : 1;
	unsigned alt_grt_83_limit_1 : 1;		/* AZCWHL */
} IF7_L0;
struct I7 {
	IF7_L0	il0;
};
typedef struct {
	unsigned rot_velocity_limit : 1;
	unsigned rot_slip : 1;
	unsigned rot_pos_380b_ccw : 1;			/* ROCCWHL	*/
	unsigned rot_neg_200b_cw : 1;			/* ROCWHL	*/
	unsigned rot_pos_380a_ccw : 1;			/* ROCCWSL	*/
	unsigned rot_neg_200a_cw : 1;			/* ROCWSL	*/
	unsigned rot_dir_ccw : 1;
	unsigned rot_dir_cw : 1;
	unsigned az_velocity_limit : 1;
	unsigned az_slip : 1;
	unsigned spare_s9_c5 : 1;
	unsigned bldg_clear_az : 1;
	unsigned alt_grt_83_limit_2 : 1;		/* ALP-2	*/
	unsigned az_109_131_limit_2 : 1;		/* ALP-2	*/
	unsigned bldg_on_az : 1;
	unsigned alt_grt_18d6_limit_3 : 1;		/* ALP-2	*/
	
	unsigned t_bar_latch_stat : 1;
	unsigned in_8_bit_30_spare : 1;
	unsigned in_8_bit_29_spare : 1;
	unsigned in_8_bit_28_spare : 1;
	unsigned deg_15_stop_ret : 1;
	unsigned e_stop_byp_sw : 1;
	unsigned umbilical_strain_sw : 1;
	unsigned rot_mtr_rdy : 1;
	unsigned alt_mtr2_rdy : 1;
	unsigned alt_mtr1_rdy : 1;
	unsigned az_mtr2_rdy : 1;
	unsigned az_mtr1_rdy : 1;
	unsigned az_pos_400_ccw : 1;
	unsigned az_neg_165_cw : 1;
	unsigned az_110_130_limit : 1;
	unsigned az_stow_cntr_sw : 1;
} IF8_L0;
struct I8 {
	IF8_L0	il0;
};
typedef struct {
	unsigned in_9_bit_15_spare: 1;
	unsigned in_9_bit_14_spare: 1;
	unsigned in_9_bit_13_spare: 1;
	unsigned in_9_bit_12_spare: 1;
	unsigned in_9_bit_11_spare: 1;
	unsigned in_9_bit_10_spare: 1;
	unsigned in_9_bit_9_spare: 1;
	unsigned solenoid_engage_sw : 1;
	unsigned low_lvl_lighting_req : 1;
	unsigned alt_brake_dis_stat : 1;
	unsigned alt_brake_en_stat : 1;
	unsigned az_brake_dis_stat : 1;
	unsigned az_brake_en_stat : 1;
	unsigned clamp_dis_stat : 1;
	unsigned clamp_en_stat : 1;
	unsigned t_bar_unlatch_stat : 1;

	unsigned s2_c7_bypass_sw : 1;
	unsigned s2_c6_bypass_sw : 1;
	unsigned s2_c5_bypass_sw : 1;
	unsigned s2_c4_bypass_sw : 1;
	unsigned s2_c3_bypass_sw : 1;
	unsigned s2_c2_bypass_sw : 1;
	unsigned s2_c1_bypass_sw : 1;
	unsigned s2_c0_bypass_sw : 1;
	unsigned s1_c7_bypass_sw : 1;
	unsigned s1_c6_bypass_sw : 1;
	unsigned s1_c5_bypass_sw : 1;
	unsigned s1_c4_bypass_sw : 1;
	unsigned s1_c3_bypass_sw : 1;
	unsigned s1_c2_bypass_sw : 1;
	unsigned s1_c1_bypass_sw : 1;
	unsigned s1_c0_bypass_sw : 1;
} IF9_L0;
struct I9 {
	IF9_L0	il0;
};
typedef struct {
	unsigned in_10_bit_15_spare: 1;
	unsigned in_10_bit_14_spare: 1;
	unsigned in_10_bit_13_spare: 1;
	unsigned in_10_bit_12_spare: 1;
	unsigned in_10_bit_11_spare: 1;
	unsigned in_10_bit_10_spare: 1;
	unsigned in_10_bit_9_spare: 1;
	unsigned in_10_bit_8_spare: 1;
	unsigned in_10_bit_7_spare: 1;
	unsigned in_10_bit_6_spare: 1;
	unsigned in_10_bit_5_spare: 1;
	unsigned in_10_bit_4_spare: 1;
	unsigned in_10_bit_3_spare: 1;
	unsigned in_10_bit_2_spare: 1;
	unsigned in_10_bit_1_spare: 1;
	unsigned in_10_bit_0_spare: 1;

	unsigned in_10_bit_31_spare: 1;
	unsigned in_10_bit_30_spare: 1;
	unsigned in_10_bit_29_spare: 1;
	unsigned in_10_bit_28_spare: 1;
	unsigned in_10_bit_27_spare: 1;
	unsigned in_10_bit_26_spare: 1;
	unsigned in_10_bit_25_spare: 1;
	unsigned in_10_bit_24_spare: 1;
	unsigned in_10_bit_23_spare: 1;
	unsigned in_10_bit_22_spare: 1;
	unsigned in_10_bit_21_spare: 1;
	unsigned in_10_bit_20_spare: 1;
	unsigned in_10_bit_19_spare: 1;
	unsigned in_10_bit_18_spare: 1;
	unsigned in_10_bit_17_spare: 1;
	unsigned in_10_bit_16_spare: 1;
} IF10_L0;
struct I10 {
	IF10_L0	il0;
};
typedef struct {
	unsigned low_lvl_light_2 : 1;
	unsigned low_lvl_light_1 : 1;
	unsigned az_stow_light : 1;
	unsigned stop_bypass_strobe : 1;
	unsigned az_stow_center_light : 1;
	unsigned rack_0_grp_2_bit10 : 1;
	unsigned inst_lift_perm : 1;
  	unsigned inst_lift_dn_4 : 1;
  	unsigned inst_lift_dn_3 : 1;
  	unsigned inst_lift_dn_2 : 1;
  	unsigned inst_lift_dn_1 : 1;
  	unsigned inst_lift_up_1 : 1;
  	unsigned inst_lift_up_2 : 1;
  	unsigned inst_lift_up_3 : 1;
  	unsigned inst_lift_up_4 : 1;
  	unsigned inst_lift_high_psi : 1;
	
	unsigned : 16;
} OF1_L1;
typedef struct {
	unsigned : 16;
	
	unsigned sad_latch2_cls_led : 1;
	unsigned sad_latch2_opn_led : 1;
	unsigned sad_latch1_cls_led : 1;
	unsigned sad_latch1_opn_led : 1;
	unsigned rack_1_grp_1_bit11 : 1;
	unsigned rack_1_grp_1_bit10 : 1;
	unsigned sad_man_req : 1;
	unsigned sad_latch_perm : 1;
	unsigned sad_unlatch_perm : 1;
	unsigned sec_man_req : 1;
	unsigned sec_latch_perm : 1;
	unsigned sec_unlatch_perm : 1;
	unsigned inst_man_req : 1;
	unsigned inst_latch_perm : 1;
	unsigned inst_unlatch_perm : 1;
	unsigned ilcb_pres_led : 1;
} OF1_L4;
typedef struct {
	unsigned safety_latch2_cls_led : 1;
	unsigned safety_latch2_opn_led : 1;
	unsigned safety_latch1_cls_led : 1;
	unsigned safety_latch1_opn_led : 1;
	unsigned sec_latch3_cls_led : 1;
	unsigned sec_latch3_opn_led : 1;
	unsigned sec_latch2_cls_led : 1;
	unsigned sec_latch2_opn_led : 1;
      	unsigned sec_latch1_cls_led : 1;
	unsigned sec_latch1_opn_led : 1;
	unsigned inst_latch3_cls_led : 1;
	unsigned inst_latch3_opn_led : 1;
	unsigned inst_latch2_cls_led : 1;
	unsigned inst_latch2_opn_led : 1;
	unsigned inst_latch1_cls_led : 1;
	unsigned inst_latch1_opn_led : 1;
	
	unsigned : 16;
} OF1_L5;
typedef struct {
	unsigned : 16;
	
	unsigned audio_warning_2 : 1;
	unsigned rack_1_grp_5_bit14 : 1;
	unsigned rack_1_grp_5_bit13 : 1;
	unsigned rack_1_grp_5_bit12 : 1;
	unsigned rack_1_grp_5_bit11 : 1;
	unsigned rack_1_grp_5_bit10 : 1;
	unsigned rack_1_grp_5_bit9 : 1;
	unsigned rack_1_grp_5_bit8 : 1;
	unsigned rack_1_grp_5_bit7 : 1;
	unsigned rack_1_grp_5_bit6 : 1;
	unsigned rack_1_grp_5_bit5 : 1;
	unsigned rack_1_grp_5_bit4 : 1;
	unsigned rack_1_grp_5_bit3 : 1;
	unsigned rack_1_grp_5_bit2 : 1;
	unsigned rack_1_grp_5_bit1 : 1;
	unsigned purge_valve_permit : 1;
} OF1_L6;
typedef struct {
	unsigned : 16;
	
	unsigned rack_2_grp_3_bit15 : 1;
	unsigned rack_2_grp_3_bit14 : 1;
	unsigned rack_2_grp_3_bit13 : 1;
	unsigned rack_2_grp_3_bit12 : 1;
	unsigned rack_2_grp_3_bit11 : 1;
	unsigned rack_2_grp_3_bit10 : 1;
	unsigned rack_2_grp_3_bit9 : 1;
	unsigned rack_2_grp_3_bit8 : 1;
	unsigned rack_2_grp_3_bit7 : 1;
	unsigned rack_2_grp_3_bit6 : 1;
	unsigned slit_latch2_ext_perm : 1;
	unsigned slit_dr2_opn_perm : 1;
	unsigned slit_dr2_cls_perm : 1;
	unsigned slit_latch1_ext_perm : 1;
	unsigned slit_dr1_opn_perm : 1;
	unsigned slit_dr1_cls_perm : 1;
} OF1_L9;
typedef struct {
	short flex_analog_config;
	short unused_short1;
} OF1_L12;
typedef struct {
	unsigned : 16;
	
	unsigned audio_warning_1 : 1;
	unsigned rack_4_grp_5_bit14 : 1;
	unsigned rack_4_grp_5_bit13 : 1;
	unsigned rack_4_grp_5_bit12 : 1;
	unsigned rack_4_grp_5_bit11 : 1;
	unsigned rack_4_grp_5_bit10 : 1;
	unsigned rack_4_grp_5_bit9 : 1;
	unsigned rack_4_grp_5_bit8 : 1;
	unsigned rack_4_grp_5_bit7 : 1;
	unsigned rack_4_grp_5_bit6 : 1;
	unsigned rack_4_grp_5_bit5 : 1;
	unsigned hgcd_lamps_on_pmt : 1;
	unsigned ne_lamps_on_pmt : 1;
	unsigned ff_lamps_on_pmt : 1;
	unsigned ff_screen_enable_pmt : 1;
	unsigned ff_screen_open_pmt : 1;
} OF1_L14;
struct O1 {
	unsigned long undefined0;
	OF1_L1 ol1;
	unsigned long undefined2;
	unsigned long undefined3;
	OF1_L4 ol4;
	OF1_L5 ol5;
	OF1_L6 ol6;
	unsigned long undefined7;
	unsigned long undefined8;
	OF1_L9 ol9;
	unsigned long undefined10;
	unsigned long undefined11;
	OF1_L12 ol12;
	unsigned long undefined13;
	OF1_L14 ol14;
	unsigned long undefined15;
};
typedef struct {
	unsigned : 16;
	
	unsigned : 12;
	unsigned wind_mtr_dn_perm : 1;
	unsigned wind_mtr_up_perm : 1;
	unsigned wind_mtr_ccw_perm : 1;
	unsigned wind_mtr_cw_perm : 1;
} OF2_L0;
struct O2 {
	OF2_L0 ol0;
	unsigned long undefined1;
	unsigned long undefined2;
	unsigned long undefined3;
	unsigned long undefined4;
	unsigned long undefined5;
	unsigned long undefined6;
	unsigned long undefined7;
	unsigned long undefined8;
	unsigned long undefined9;
	unsigned long undefined10;
	unsigned long undefined11;
	unsigned long undefined12;
	unsigned long undefined13;
	unsigned long undefined14;
	unsigned long undefined15;
};
typedef struct {
	unsigned s_ll_led: 1;
	unsigned n_ll_led: 1;
	unsigned w_rail_led: 1;
	unsigned s_rail_led: 1;
	unsigned n_rail_led: 1;
	unsigned rot_plc_perm : 1;
	unsigned rot_mtr_ccw_perm : 1;
	unsigned rot_mtr_cw_perm : 1;
	unsigned wind_az_plc_perm : 1;
	unsigned az_plc_perm : 1;
	unsigned az_mtr_ccw_perm : 1;
	unsigned az_mtr_cw_perm : 1;
	unsigned wind_alt_plc_perm : 1;
	unsigned alt_plc_perm : 1;
	unsigned alt_mtr_dn_perm : 1;
	unsigned alt_mtr_up_perm : 1;

	unsigned clamp_en : 1;
	unsigned clamp_dis : 1;
	unsigned t_bar_unlatch_perm : 1;
	unsigned t_bar_latch_perm : 1;
	unsigned out_11_bit_27_spare : 1;
	unsigned lift_pump_on : 1;
	unsigned out_11_bit_25_spare : 1;
	unsigned out_11_bit_24_spare : 1;
	unsigned deg_15_stop_ret_perm : 1;
	unsigned deg_15_stop_ext_perm : 1;
	unsigned lift_solenoid_en: 1;
	unsigned s_wind_led: 1;
	unsigned n_fork_led: 1;
	unsigned n_wind_led: 1;
	unsigned w_ll_led: 1;
	unsigned e_ll_led: 1;
} OF11_L0;
struct O11 {
	OF11_L0	ol0;
};
typedef struct {
	unsigned out_12_bit_15_spare : 1;
	unsigned out_12_bit_14_spare : 1;
	unsigned out_12_bit_13_spare : 1;
	unsigned out_12_bit_12_spare : 1;
	unsigned out_12_bit_11_spare : 1;
	unsigned out_12_bit_10_spare : 1;
	unsigned out_12_bit_9_spare : 1;
	unsigned stow_pos_light : 1;
	unsigned inst_chg_pos_light : 1;
	unsigned nw_fork_led : 1;
	unsigned umbilical_up_dn : 1;
	unsigned umbilical_on_off : 1;
	unsigned alt_brake_en : 1;
	unsigned alt_brake_dis : 1;
	unsigned az_brake_en : 1;
	unsigned az_brake_dis : 1;

	unsigned out_12_bit_31_spare : 1;
	unsigned out_12_bit_30_spare : 1;
	unsigned out_12_bit_29_spare : 1;
	unsigned out_12_bit_28_spare : 1;
	unsigned out_12_bit_27_spare : 1;
	unsigned out_12_bit_26_spare : 1;
	unsigned out_12_bit_25_spare : 1;
	unsigned out_12_bit_24_spare : 1;
	unsigned out_12_bit_23_spare : 1;
	unsigned out_12_bit_22_spare : 1;
	unsigned out_12_bit_21_spare : 1;
	unsigned out_12_bit_20_spare : 1;
	unsigned out_12_bit_19_spare : 1;
	unsigned out_12_bit_18_spare : 1;
	unsigned out_12_bit_17_spare : 1;
	unsigned out_12_bit_16_spare : 1;
} OF12_L0;
struct O12 {
	OF12_L0	ol0;
};

typedef struct {
   unsigned rot_mtr_iv_good : 1;	/* Rotator motor resistance check OK */
   unsigned alt_mtr_iv_good : 1;	/* Altitude motor resistance check OK*/
   unsigned az_mtr_iv_good : 1;		/* Azimuth motor resistance check OK */
   unsigned hgcd_lamp_on_request : 1;	/* Turn HgCd lamps on */
   unsigned ne_lamp_on_request : 1;	/* Turn Ne lamps on */
   unsigned lift_speed_man_ovrid : 1;	/* override MCP lift speed. */
   unsigned ilcb_led_on : 1;		/* Turn on instr latch ctrl box light*/
   unsigned ff_screens_closed : 1;	/* all flat field screens are closed */
   unsigned e_stop_flash_reset : 1;	/* reset the e-stop LED flash timer */
   unsigned led_flash : 1;		/* E-stop led flash bit */
   unsigned flip_flop_5 : 1;		/* create e-stop led flasher */
   unsigned flip_flop_4 : 1;		/* create e-stop led flasher */
   unsigned flip_flop_3 : 1;		/* create e-stop led flasher */
   unsigned flip_flop_2 : 1;		/* debounce the */
   unsigned flip_flop_1 : 1;		/*    low level lighting switch and */
   unsigned flip_flop_0 : 1;		/*       latch the light on */

   unsigned spare31 : 1;
   unsigned spare30 : 1;
   unsigned spare29 : 1;
   unsigned spare28 : 1;
   unsigned spare27 : 1;
   unsigned spare26 : 1;
   unsigned spare25 : 1;
   unsigned spare24 : 1;
   unsigned spare23 : 1;
   unsigned spare22 : 1;
   unsigned spare21 : 1;
   unsigned dn_inhibit_latch_4 : 1;	/* Altitude Down inhibit latch bit 4 */
   unsigned dn_inhibit_latch_3 : 1;	/* Altitude Down inhibit latch bit 3 */
   unsigned dn_inhibit_latch_2 : 1;	/* Altitude Down inhibit latch bit 2 */
   unsigned dn_inhibit_latch_1 : 1;	/* Altitude Down inhibit latch bit 1 */
   unsigned up_inhibit_latch : 1;	/* Altitude Up inhibit latch bit */
} B3_W0;

struct B3 {
   B3_W0 w0;
   unsigned long undefined1;
};

typedef struct {
   unsigned mcp_clamp_engage_cmd : 1;	/* Clamp engage command */
   unsigned mcp_alt_brk_en_cmd : 1;	/* Alt brake engage command bit */
   unsigned mcp_alt_brk_dis_cmd : 1;	/* Alt brake disengage command bit */
   unsigned mcp_az_brk_en_cmd : 1;	/* Az brake engage command bit */
   unsigned mcp_az_brk_dis_cmd : 1;	/* Az brake disengage command bit */
   unsigned mcp_solenoid_engage : 1;	/* turn on the inst. lift solenoid */
   unsigned mcp_pump_on : 1;		/* turn on the instrument lift pump */
   unsigned mcp_lift_dn_4 : 1;		/* Part of the */
   unsigned mcp_lift_dn_3 : 1;		/*    binary code */
   unsigned mcp_lift_dn_2 : 1;		/*       for down lift */
   unsigned mcp_lift_dn_1 : 1;		/*          speed */
   unsigned mcp_lift_up_1 : 1;		/* Part of the */
   unsigned mcp_lift_up_2 : 1;		/*    binary code */
   unsigned mcp_lift_up_3 : 1;		/*       for up lift */
   unsigned mcp_lift_up_4 : 1;		/*          speed */
   unsigned mcp_lift_high_psi : 1;	/* turn on the high pressure for lift*/

   unsigned mcp_ff_screen_enable : 1;	/* Enable flat field screen motion */
   unsigned mcp_hgcd_lamp_on_cmd : 1;	/* turn on the HgCd lamps */
   unsigned mcp_ne_lamp_on_cmd : 1;	/* turn the Ne lamps on */
   unsigned mcp_ff_lamp_on_cmd : 1;	/* turn the ff lamps on */
   unsigned mcp_ff_scrn_opn_cmd : 1;	/* open the ff screen */
   unsigned mcp_slit_latch2_cmd : 1;	/* slithead latch2 control 1=en 0=dis*/
   unsigned mcp_slit_dr2_cls_cmd : 1;	/* slithead door 2 close command bit */
   unsigned mcp_slit_dr2_opn_cmd : 1;	/* slithead door 2 open command bit */
   unsigned mcp_slit_latch1_cmd : 1;	/* slithead latch1 control 1=en 0=dis*/
   unsigned mcp_slit_dr1_cls_cmd : 1;	/* slithead door 1 close command bit */
   unsigned mcp_slit_dr1_opn_cmd : 1;	/* slithead door 1 open command bit */
   unsigned mcp_umbilical_on_off : 1;	/* umbilical motor 1=on 0=off */
   unsigned mcp_umbilical_up_dn : 1;	/* umbilical motor 0=up 1=down */
   unsigned mcp_15deg_stop_ret_c : 1;	/* 15 degree stop remove command */
   unsigned mcp_15deg_stop_ext_c : 1;	/* 15 degree stop insert command */
   unsigned mcp_clamp_disen_cmd : 1;	/* Clamp disengage command */
} B10_W0;

typedef struct {
   unsigned spare15 : 1;
   unsigned spare14 : 1;
   unsigned spare13 : 1;
   unsigned spare12 : 1;
   unsigned spare11 : 1;
   unsigned mcp_inst_chg_alert : 1;	/* alert the observers of malfunction*/
   unsigned mcp_inst_chg_prompt : 1;	/* prompt the observers */
   unsigned mcp_sad_latch_opn_cm : 1;	/* open the saddle latches */
   unsigned mcp_sad_latch_cls_cm : 1;	/* close the saddle latches */
   unsigned mcp_sec_latch_opn_cm : 1;	/* open the secondary latches */
   unsigned mcp_sec_latch_cls_cm : 1;	/* close the secondary latches */
   unsigned mcp_pri_latch_opn_cm : 1;	/* open the primary latches */
   unsigned mcp_pri_latch_cls_cm : 1;	/* close the primary latches */
   unsigned mcp_purge_cell_on : 1;	/* start the purge cell */
   unsigned mcp_t_bar_latch : 1;	/* latch the t-bar latches */
   unsigned mcp_t_bar_unlatch : 1;	/* unlatch the t-bar latches */

   unsigned spare31 : 1;
   unsigned spare30 : 1;
   unsigned spare29 : 1;
   unsigned spare28 : 1;
   unsigned spare27 : 1;
   unsigned spare26 : 1;
   unsigned spare25 : 1;
   unsigned spare24 : 1;
   unsigned spare23 : 1;
   unsigned spare22 : 1;
   unsigned spare21 : 1;
   unsigned spare20 : 1;
   unsigned spare19 : 1;
   unsigned spare18 : 1;
   unsigned spare17 : 1;
   unsigned velocity_trp_rst_in : 1;	/* reset slip detection velocity trip*/
} B10_W1;

struct B10 {
   B10_W0 w0;
   B10_W1 w1;
};

struct AB_SLC500 {
	short status;
	short errcnt;
	struct I1 i1;
	struct O1 o1;
	struct I2 i2;
	struct O2 o2;
	struct I3 i3;
	struct I4 i4;
	struct I5 i5;
	struct I6 i6;
	struct I7 i7;
	struct I8 i8;
	struct I9 i9;
	struct I10 i10;
	struct O11 o11;
	struct O12 o12;
	struct B3 b3;
	struct B10 b10;
};

struct SDSS_FRAME {
	unsigned char vers;
#define SDSS_FRAME_VERSION	11
	unsigned char type;
#define DATA_TYPE	1
	unsigned short CRC;		/* CRC for sdssdc */
	time_t ctime;
	struct TM axis[3];
	struct IL inst;
	struct CW weight[4];
	struct AB_SLC500 status;
	struct PVT_M68K tccmove[3];	/* AZTCCPOS, ALTCCPOS, ROTCCPOS,
					   AZTCCVEL, ALTCCVEL, ROTCCVEL,
					   AZTCCTIM, ALTCCTIM, ROTCCTIM */
	struct PVT_M68K tccpmove[3];
	struct PVT_M68K pvt[3];		/* AZMCPPOS, ALMCPPOS, ROMCPPOS,
					   AZMCPVEL, ALMCPVEL, ROMCPVEL,
					   AZMCPTIM, ALMCPTIM, ROMCPTIM */
	unsigned long axis_state[3];	/* AZSTATE, ALSTATE, ROSTATE */
	unsigned long sdsstime;
	unsigned long ascii_len;
#define ASCII_LEN	80
	unsigned char ascii[ASCII_LEN];
};

#define STATUS_MASK 0x00000700		/* mask of bits not to pass to TCC */

struct AXIS_STAT {
	unsigned always_zero : 1;
	unsigned bump_up_ccw_sticky : 1;
	unsigned bump_dn_cw_sticky : 1;
	unsigned  : 4;
	unsigned ms_on_correction_too_large : 1;

	unsigned  : 6;
	unsigned clock_slow_signal : 1;
	unsigned clock_loss_signal : 1;

        unsigned : 1;
	unsigned semCmdPort_taken : 1;
	unsigned stop_in : 1;
	unsigned amp_bad : 1;
	unsigned out_closed_loop : 1;
	unsigned stop_ok : 1;
	unsigned amp_ok : 1;
	unsigned closed_loop : 1;

	unsigned max_limit: 1;
	unsigned min_limit: 1;
	unsigned max_acc: 1;
	unsigned max_vel: 1;
	unsigned max_pos: 1;
	unsigned min_pos: 1;
	unsigned pvt_time_late : 1;
	unsigned pvt_empty : 1;
};

/*
 * Global variables
 */
extern struct SDSS_FRAME sdssdc;
extern struct TM_M68K *tmaxis[];
extern int rawtick;
extern struct TM_M68K *tmaxis[];
#if defined(__INCsemLibh)
   extern SEM_ID semSLCDC;
#endif
