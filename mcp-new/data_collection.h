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
	short actual_position_hi;
	short actual_position_lo;
	unsigned short time_fractional;
	unsigned short time_hi;
	unsigned short time_lo;
	short jerk_fractional;
	short jerk_hi;
	short jerk_lo;
	short acceleration_fractional;
	short acceleration_hi;
	short acceleration_lo;
	short velocity_fractional;
	short velocity_hi;
	short velocity_lo;
	short position_fractional_hi;
	short position_fractional_lo;
	short position_hi;
	short position_lo;
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
	short latch_lo;
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
	short actual_position2_hi;
	short actual_position2_lo;
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

struct B10_1 {
	unsigned : 11;
	unsigned mcp_umbilical_off_on_cmd: 1;	/* bit 20 on=1, off=0 */
	unsigned mcp_umbilical_dn_up_cmd: 1;	/* bit 19 dn=1, up=0 */
	unsigned mcp_15_deg_stop_out_cmd: 1;
	unsigned mcp_15_deg_stop_in_cmd: 1;
	unsigned mcp_clamp_dis_cmd: 1;	/* bit 16 */
};
struct B10 {
	unsigned mcp_clamp_en_cmd: 1;
	unsigned mcp_alt_brk_en_cmd: 1;
	unsigned mcp_alt_brk_dis_cmd: 1;
	unsigned mcp_az_brk_en_cmd: 1;
	unsigned mcp_az_brk_dis_cmd: 1;
	unsigned mcp_solenoid_enable : 1;
	unsigned mcp_pump_on : 1;
	unsigned mcp_lift_dn: 4;
	unsigned mcp_lift_up: 4;
	unsigned mcp_lift_high_psi: 1;
};
typedef struct {
	unsigned : 1;			/* msw bit - 32, i.e. 0x8000 0000 */
	unsigned az_bump_ccw : 1; 
	unsigned az_bump_cw : 1;
	unsigned imager_serv_cart : 1;
	unsigned imager_ops_cart : 1;
	unsigned fiber_cart_pos2 : 1;	/* cart locked in second position - put */
	unsigned fiber_cart_pos1 : 1;	/* cart locked in first position - get */
	unsigned inst_lift_low : 1;	/* */
	unsigned inst_lift_high : 1;	/* */
	unsigned inst_lift_man : 1;	/* lift is in manual or local mode */
	unsigned inst_lift_dn : 1;	/* lift is in down position */
	unsigned inst_lift_sw4 : 1;	/* fiber cartridge on cart floor */
	unsigned inst_lift_sw3 : 1;	/* dummy cartridge on cart floor */
	unsigned inst_lift_sw2 : 1;	/* engaged with clamping force */
	unsigned inst_lift_sw1 : 1;	/* kinematic mounts engaged */
	unsigned inst_lift_pump_on : 1;	/* pump status */

	unsigned low_lvl_light_req : 1;	/* msw bit - 15, i.e. 0x8000 */
	unsigned : 8;
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
	unsigned alt_bump_dn : 1;
	unsigned alt_bump_up : 1;
	unsigned sad_man_valve_cls : 1;
	unsigned sec_man_valve_cls : 1;
	unsigned inst_man_valve_cls : 1;
	unsigned ilcb_pres_good : 1;
	unsigned rot_280_ccw : 1;
	unsigned rot_280_cw : 1;
	unsigned rot_inst_chg_b : 1;
	unsigned rot_inst_chg_a : 1;

	unsigned : 16;
}IF1_L4;
typedef struct {
	unsigned : 1;
  	unsigned spec_lens3 : 1;
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

    	unsigned ter_latch2_cls : 1;
    	unsigned ter_latch2_opn : 1;
    	unsigned ter_latch1_cls : 1;
    	unsigned ter_latch1_opn : 1;
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
	unsigned : 4;
	unsigned cart_latch2_opn : 1;
	unsigned slit_door2_cls : 1;
	unsigned slit_door2_opn : 1;
	unsigned cart_latch1_opn : 1;
	unsigned slit_door1_cls : 1;
	unsigned slit_door1_opn : 1;
	unsigned sad_mount2 : 1;
	unsigned sad_mount1 : 1;
	unsigned sad_latch2_cls : 1;
	unsigned sad_latch2_opn : 1;
	unsigned sad_latch1_cls : 1;
	unsigned sad_latch1_opn : 1;
	
	unsigned : 16;
}IF1_L9;
struct I1 {
	IF1_L0 il0;
	unsigned long undefined1;
	unsigned long undefined2;
	unsigned long undefined3;
	IF1_L4 il4;
	unsigned long undefined5;
	unsigned long undefined6;
	unsigned long undefined7;
	IF1_L8 il8;
	IF1_L9 il9;
	unsigned long undefined10;
	unsigned long undefined11;
	unsigned long undefined12;
	unsigned long undefined13;
	unsigned long undefined14;
	unsigned long undefined15;
};
typedef struct {
	unsigned dcm_status : 16;
	unsigned low_lvl_lighting_req : 1;
	unsigned : 9;
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
	short alt_lvdt_error;
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
	short az_1_voltage;
	short az_1_current;
	short az_2_voltage;
	short az_2_current;
	short alt_1_voltage;
	short alt_1_current;
	short alt_2_voltage;
	short alt_2_current;
};
struct I4 {
	short alt_position;
	short rot_1_voltage;
	short rot_1_current;
	short umbilical_dist;
	short inst_lift_force;
	short inst_lift_dist;
	unsigned short undefined3;
	unsigned short undefined4;
};
typedef struct {
	unsigned dog_house_cls: 1;
	unsigned hatch_closed : 1;
	unsigned bldg_clear : 1;
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
	unsigned interlock_reset : 1;
	unsigned cr_stop : 1;
	unsigned tcc_stop : 1;
	
	unsigned deg_15_stop_out : 1;
	unsigned deg_15_stop_in : 1;
	unsigned az_mtr2_perm : 1;
	unsigned az_mtr1_perm : 1;
	unsigned az_mtr_ccw : 1;
	unsigned az_mtr_cw : 1;
	unsigned : 1;
	unsigned az_plc_permit_in : 1;
	unsigned az_mtr2_rdy : 1;
	unsigned az_mtr1_rdy : 1;
	unsigned az_290_ccw : 1;
	unsigned az_290_cw : 1;
	unsigned az_280_ccw : 1;
	unsigned az_280_cw : 1;
	unsigned az_dir_ccw : 1;
	unsigned az_dir_cw : 1;
} IF56_L0;
struct I56 {
	IF56_L0	il0;
};
typedef struct {
	unsigned alt_brake_engaged: 1;
	unsigned az_brake_disengaged: 1;
	unsigned az_brake_engaged: 1;
	unsigned az_stow_b : 1;
	unsigned az_stow_a : 1;
	unsigned alt_stow : 1;
	unsigned alt_mtr2_perm : 1;
	unsigned alt_mtr1_perm : 1;
	unsigned alt_mtr_dn : 1;
	unsigned alt_mtr_up : 1;
	unsigned alt_plc_permit_in : 1;
	unsigned alt_mtr2_rdy : 1;
	unsigned alt_mtr1_rdy : 1;
	unsigned alt_90_5_limit : 1;
	unsigned alt_20_limit : 1;
	unsigned alt_0_6_limit : 1;
	
	unsigned s_wind_e_stop : 1;
	unsigned lift_estop_sw : 1;
	unsigned lift_dn_sw : 1;
	unsigned lift_up_sw : 1;
	unsigned deg_15_in_permit : 1;
	unsigned : 1;
	unsigned alt_brake_disengaged: 1;
	unsigned rot_mtr_perm : 1;
	unsigned rot_mtr_ccw : 1;
	unsigned rot_mtr_cw : 1;
	unsigned rot_plc_permit_in : 1;
	unsigned rot_mtr_rdy : 1;
	unsigned rot_290_ccw : 1;
	unsigned rot_290_cw : 1;
	unsigned rot_dir_ccw : 1;
	unsigned rot_dir_cw : 1;
} IF78_L0;
struct I78 {
	IF78_L0	il0;
};
typedef struct {
	unsigned low_lvl_light_2 : 1;
	unsigned low_lvl_light_1 : 1;
	unsigned : 2;
	unsigned opt_bench_cls_perm : 1;
	unsigned opt_bench_opn_perm : 1;
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
	unsigned : 2;
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
	unsigned ter_latch2_cls_led : 1;
	unsigned ter_latch2_opn_led : 1;
	unsigned ter_latch1_cls_led : 1;
	unsigned ter_latch1_opn_led : 1;
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
	
	unsigned : 10;
	unsigned slit_latch2_opn_perm : 1;
	unsigned slit_dr2_opn_perm : 1;
	unsigned slit_dr2_cls_perm : 1;
	unsigned slit_latch1_opn_perm : 1;
	unsigned slit_dr1_opn_perm : 1;
	unsigned slit_dr1_cls_perm : 1;
} OF1_L9;
struct O1 {
	unsigned long undefined0;
	OF1_L1 ol1;
	unsigned long undefined2;
	unsigned long undefined3;
	OF1_L4 ol4;
	OF1_L5 ol5;
	unsigned long undefined6;
	unsigned long undefined7;
	unsigned long undefined8;
	OF1_L9 ol9;
	unsigned long undefined10;
	unsigned long undefined11;
	unsigned long undefined12;
	unsigned long undefined13;
	unsigned long undefined14;
	unsigned long undefined15;
};
typedef struct {
	unsigned : 16;
	
	unsigned : 10;
	unsigned inst_chg_pos_light : 1;
	unsigned stow_pos_light : 1;
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
	unsigned wind_dog_house_dr_cl : 1;
	unsigned : 1;
	unsigned wind_alt_plc_perm : 1;
	unsigned wind_az_plc_perm : 1;
	unsigned deg_15_stop_in_perm : 1;
	unsigned deg_15_stop_out_perm : 1;
	unsigned : 1;
	unsigned rot_plc_ccw_perm : 1;
	unsigned rot_plc_cw_perm : 1;
	unsigned rot_plc_perm : 1;
	unsigned alt_plc_dn_perm : 1;
	unsigned alt_plc_up_perm : 1;
	unsigned alt_plc_perm : 1;
	unsigned az_plc_ccw_perm : 1;
	unsigned az_plc_cw_perm : 1;
	unsigned az_plc_perm : 1;

	unsigned : 3;
	unsigned umbilical_dir_cmd : 1;
	unsigned umbilical_motor_cmd : 1;
	unsigned alt_brake_engage_cmd : 1;
	unsigned alt_brake_disen_cmd : 1;
	unsigned az_brake_engage_cmd : 1;
	unsigned az_brake_disen_cmd : 1;
	unsigned clamp_engage_cmd : 1;
	unsigned clamp_disen_cmd : 1;
	unsigned solenoid_enable : 1;
	unsigned pump_on_cmd : 1;
	unsigned lift_estop_light : 1;
	unsigned lift_dn_light : 1;
	unsigned lift_up_light : 1;
} OF910_L0;
struct O910 {
	OF910_L0	ol0;
};
typedef struct {
	unsigned : 13;
	unsigned umbilical_strain_sw : 1;
	unsigned clamp_disengaged_st : 1;
	unsigned clamp_engaged_st : 1;

	unsigned : 14;
	unsigned tbar_latch_opn_perm : 1;
	unsigned tbar_latch_cls_perm : 1;
} I11O12_L0;
struct I11O12 {
	I11O12_L0	ol0;
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
	struct I56 i56;
	struct I78 i78;
	struct O910 o910;
	struct I11O12 i11o12;
};

struct SDSS_FRAME {
	unsigned char vers;
#define SDSS_FRAME_VERSION	3
	unsigned char type;
#define DATA_TYPE	1
	unsigned short binary_len;
	time_t ctime;
	struct TM axis[3];
	struct IL inst;
	struct CW weight[4];
	struct AB_SLC500 status;
	unsigned long ascii_len;
#define ASCII_LEN	80
	unsigned char ascii[ASCII_LEN];
};
