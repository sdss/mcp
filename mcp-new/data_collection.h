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
/*
 * Start of machine generated code
 */
typedef struct {
   unsigned rot_mtr_iv_good : 1;             
   unsigned alt_mtr_iv_good : 1;             
   unsigned az_mtr_iv_good : 1;              
   unsigned hgcd_lamp_on_request : 1;        
   unsigned ne_lamp_on_request : 1;          
   unsigned lift_speed_man_ovrid : 1;         /* Used to overide MCP lift speed from the lift controls. */
   unsigned ilcb_led_on : 1;                  /* Turns on the instrument latch control box lights when at the zenith. */
   unsigned ff_screens_closed : 1;            /* This bit indicates that all flat field screens are closed. */
   unsigned e_stop_flash_reset : 1;           /* OSR used to reset the e-stop LED flash timer. */
   unsigned led_flash : 1;                    /* E-stop led flash bit. */
   unsigned flip_flop_5 : 1;                  /* Used to create e-stop led flasher. */
   unsigned flip_flop_4 : 1;                  /* Used to create e-stop led flasher. */
   unsigned flip_flop_3 : 1;                  /* Used to create e-stop led flasher. */
   unsigned flip_flop_2 : 1;                  /* Used to debounce the low level lighting switch and latch the light on. */
   unsigned flip_flop_1 : 1;                  /* Used to debounce the low level lighting switch and latch the light on. */
   unsigned flip_flop_0 : 1;                  /* Used to debounce the low level lighting switch and latch the light on. */
   unsigned plc_cont_t_bar_latch : 1;        
   unsigned mcp_cont_t_bar_latch : 1;        
   unsigned : 2;                             
   unsigned plc_cont_slit_hd : 1;            
   unsigned mcp_cont_slit_hd : 1;            
   unsigned : 2;                             
   unsigned plc_cont_slit_dr : 1;            
   unsigned mcp_cont_slit_dr : 1;            
   unsigned e_stop_permit : 1;               
   unsigned dn_inhibit_latch_4 : 1;           /* Altitude Down inhibit latch bit 4 */
   unsigned dn_inhibit_latch_3 : 1;           /* Altitude Down inhibit latch bit 3 */
   unsigned dn_inhibit_latch_2 : 1;           /* Altitude Down inhibit latch bit 2 */
   unsigned dn_inhibit_latch_1 : 1;           /* Altitude Down inhibit latch bit 1 */
   unsigned up_inhibit_latch : 1;             /* Altitude Up inhibit latch bit */
} B3_L0;

typedef struct {
   unsigned img_cam_in_place : 1;             /* Imaging Camera on the telescope */
   unsigned undefined_2 : 1;                  /* Undefined instrument on the telescope */
   unsigned undefined_1 : 1;                  /* Undefined instrument on the telescope */
   unsigned undefined_3 : 1;                  /* Undefined instrument on the telescope */
   unsigned eng_cam_in_place : 1;             /* Engineering Camera on the telescope */
   unsigned cartridge_9 : 1;                  /* Cartridge 9 on the telescope */
   unsigned cartridge_8 : 1;                  /* Cartridge 8 on the telescope */
   unsigned cartridge_7 : 1;                  /* Cartridge 7 on the telescope */
   unsigned cartridge_6 : 1;                  /* Cartridge 6 on the telescope */
   unsigned cartridge_5 : 1;                  /* Cartridge 5 on the telescope */
   unsigned cartridge_4 : 1;                  /* Cartridge 4 on the telescope */
   unsigned cartridge_3 : 1;                  /* Cartridge 3 on the telescope */
   unsigned cartridge_2 : 1;                  /* Cartridge 2 on the telescope */
   unsigned cartridge_1 : 1;                  /* Cartridge 1 on the telescope */
   unsigned no_inst_in_place : 1;             /* No Instrument on the telescope */
   unsigned disc_cable : 1;                   /* Instrument ID Cable Disconnected */
   unsigned version_id : 16;                  /* Version 21  $Name$ */
} B3_L1;

typedef struct {
   unsigned sad_not_in_place : 1;             /* Saddle not in place on the telescope */
   unsigned sad_in_place : 1;                 /* Saddle in place on the telescope */
   unsigned plc_cont_slit_dr_cls : 1;        
   unsigned plc_cont_slit_dr_opn : 1;        
   unsigned mcp_cont_slit_dr_os4 : 1;        
   unsigned mcp_cont_slit_dr_os3 : 1;        
   unsigned mcp_cont_slit_dr_os2 : 1;        
   unsigned mcp_cont_slit_dr_os1 : 1;        
   unsigned plc_t_bar_tel : 1;               
   unsigned plc_t_bar_xport : 1;             
   unsigned plc_cont_t_bar_osr : 1;          
   unsigned mcp_cont_t_bar_osr : 1;          
   unsigned plc_cont_slit_hd_osr : 1;        
   unsigned : 1;                             
   unsigned plc_cont_slit_dr_osr : 1;        
   unsigned : 1;                             
   unsigned lift_empty : 1;                   /* Instrument lift empty */
   unsigned sad_latch_cls : 1;                /* Saddle latches closed */
   unsigned sad_latch_opn : 1;                /* Saddle latches open */
   unsigned sec_latch_cls : 1;                /* Secondary latches closed */
   unsigned sec_latch_opn : 1;                /* Secondary latches open */
   unsigned pri_latch_cls : 1;                /* Primary latches closed */
   unsigned pri_latch_opn : 1;                /* Primary latches open */
   unsigned cartg_in_place : 1;               /* Cartridge in place on the telescope */
   unsigned cor_not_in_place : 1;             /* Corrector lens not in place */
   unsigned cor_in_place : 1;                 /* Corrector lens in place */
   unsigned plc_cont_slit_hd_lth : 1;        
   unsigned plc_cont_slit_hd_unl : 1;        
   unsigned mcp_cont_slit_hd_os4 : 1;        
   unsigned mcp_cont_slit_hd_os3 : 1;        
   unsigned mcp_cont_slit_hd_os2 : 1;        
   unsigned mcp_cont_slit_hd_os1 : 1;        
} B3_L2;

typedef struct {
   unsigned speed_4 : 1;                      /* Lift Speed bit */
   unsigned speed_3 : 1;                      /* Lift Speed bit */
   unsigned speed_2 : 1;                      /* Lift Speed bit */
   unsigned speed_1 : 1;                      /* Lift Speed bit */
   unsigned lift_down_enable : 1;             /* Instrument lift down enable */
   unsigned lift_up_enable : 1;               /* Instrument lift up enable */
   unsigned lift_force_dn_enable : 1;         /* Instrument lift down force enable */
   unsigned lift_force_up_enable : 1;         /* Instrument lift up force enable */
   unsigned eng_cam_on_lift_comp : 1;         /* Engineering camera on lift and compressed to telescope */
   unsigned eng_cam_on_lift : 1;              /* Engineering camera on lift */
   unsigned cartg_on_lift_comp : 1;           /* Cartridge on lift and compressed to telescope */
   unsigned cartg_on_lift : 1;                /* Cartridge on lift */
   unsigned cam_on_lift_w_j_hok : 1;          /* Camera on lift with umbilical on j hook */
   unsigned cam_on_lift_wo_j_hok : 1;         /* Camera on lift without umbilical on j hook */
   unsigned cor_on_lift : 1;                  /* Corrector lens on lift */
   unsigned auto_mode_enable : 1;             /* Instrument change auto mode enabled */
   unsigned : 10;                            
   unsigned flex_io_fault : 1;                /* Flex I/O read/write fault */
   unsigned empty_plate_on_lift : 1;          /* Empty plate on lift */
   unsigned eng_cam_up_in_place : 1;          /* Engineering camera up in place on the telescope */
   unsigned cor_up_in_place : 1;              /* Corrector lens up in place on the telescope */
   unsigned cartg_up_in_place : 1;            /* Cartridge up in place on the telescope */
   unsigned img_cam_up_in_place : 1;          /* Imaging camera up in place on the telescope */
} B3_L3;

typedef struct {
   unsigned lh_grt_17d5 : 1;                  /* Lift height greater than 17.5 inches */
   unsigned lh_lim_1d95_18d0 : 1;             /* Lift height between 1.95 and 18.0 inches */
   unsigned lh_les_2d0 : 1;                   /* Lift height less than 2.0 inches */
   unsigned lh_lim_18d0_22d2 : 1;             /* Lift height between 18.0 and 22.2 inches */
   unsigned lh_lim_18d0_22d0 : 1;             /* Lift height between 18.0 and 22.0 inches */
   unsigned lh_lim_18d0_20d99 : 1;            /* Lift height between 18.0 and 20.99 inches */
   unsigned lh_lim_2d5_18d5 : 1;              /* Lift height between 2.5 and 18.5 inches */
   unsigned lh_lim_18d0_22d75 : 1;            /* Lift height between 18.0 and 22.75 inches */
   unsigned lh_lim_18d0_22d5 : 1;             /* Lift height between 18.0 and 22.5 inches */
   unsigned lh_lim_18d0_21d74 : 1;            /* Lift height between 18.0 and 21.74 inches */
   unsigned lh_lim_2d2_18d5 : 1;              /* Lift height between 2.2 and 18.5 inches */
   unsigned lh_lim_18d0_23d0 : 1;             /* Lift height between 18.0 and 23.0 inches */
   unsigned lh_lim_18d0_22d8 : 1;             /* Lift height between 18.0 and 22.8 inches */
   unsigned lh_lim_18d0_21d89 : 1;            /* Lift height between 18.0 and 21.89 inches */
   unsigned lh_les_18d5 : 1;                  /* Lift height less than 18.5 inches */
   unsigned altitude_at_inst_chg : 1;         /* Altitude at instrument change position */
   unsigned lh_lim_20d0_21d89 : 1;            /* Lift height between 20.0 and 21.89 inches */
   unsigned lf_les_350_3 : 1;                 /* Lift force less than 350 lbs */
   unsigned lf_les_150 : 1;                   /* Lift force less than 150 lbs */
   unsigned lf_les_200 : 1;                   /* Lift force less than 200 lbs */
   unsigned lf_les_450 : 1;                   /* Lift force less than 450 lbs */
   unsigned lf_les_350_2 : 1;                 /* Lift force less than 350 lbs */
   unsigned lf_les_1400 : 1;                  /* Lift force less than 1400 lbs */
   unsigned lf_les_350 : 1;                   /* Lift force less than 350 lbs */
   unsigned lh_lim_2d0_20d0 : 1;              /* Lift height between 2.0 and 20.0 inches */
   unsigned lh_lim_0d75_2d0 : 1;              /* Lift height between 0.75 and 2.0 inches */
   unsigned lf_les_500 : 1;                   /* Lift force less than 500 lbs. */
   unsigned lh_les_0d75 : 1;                  /* Lift height less than 0.75 inches */
   unsigned lh_les_18d5_2 : 1;                /* Lift height less than 18.5 inches */
   unsigned lh_lim_18d0_22d3 : 1;             /* Lift height between 18.0 and 22.3 inches */
   unsigned lh_lim_22d3_23d1 : 1;             /* Lift height between 22.3 and 23.1 inches */
   unsigned lh_grt_23d1 : 1;                  /* Lift height greater than 23.1 inches */
} B3_L4;

typedef struct {
   unsigned lh_lim_23d1_23d3 : 1;             /* Lift height between 23.1 and 23.3 inches */
   unsigned lf_les_1100 : 1;                  /* Lift force less than 1100 lbs */
   unsigned lh_lim_22d3_23d1_2 : 1;           /* Lift height between 22.3 and 23.1 inches */
   unsigned lf_les_400_2 : 1;                 /* Lift force less than 400 lbs */
   unsigned lf_les_150_3 : 1;                 /* Lift force less than 150 lbs */
   unsigned lf_les_200_3 : 1;                 /* Lift force less than 200 lbs */
   unsigned lf_les_500_3 : 1;                 /* Lift force less than 500 lbs */
   unsigned lf_les_400 : 1;                   /* Lift force less than 400 lbs */
   unsigned lf_les_1700 : 1;                  /* Lift force less than 1700 lbs */
   unsigned lh_lim_21d89_22d3 : 1;            /* Lift height between 21.89 and 22.3 inches */
   unsigned lf_les_350_5 : 1;                 /* Lift force less than 350 lbs */
   unsigned lf_les_150_2 : 1;                 /* Lift force less than 150 lbs */
   unsigned lf_les_200_2 : 1;                 /* Lift force less than 200 lbs */
   unsigned lf_les_500_2 : 1;                 /* Lift force less than 500 lbs */
   unsigned lf_les_350_4 : 1;                 /* Lift force less than 350 lbs */
   unsigned lf_les_1650 : 1;                  /* Lift force less than 1650 lbs */
   unsigned lf_grt_310_2 : 1;                 /* Lift force greater than 310 lbs */
   unsigned lf_grt_220_2 : 1;                 /* Lift force greater than 220 lbs */
   unsigned lf_grt_150_2 : 1;                 /* Lift force greater than 150 lbs */
   unsigned lh_lim_20d0_21d89_2 : 1;          /* Lift height between 20.0 and 21.89 inches */
   unsigned lf_grt_125 : 1;                   /* Lift force greater than 125 lbs */
   unsigned lf_grt_0d0_2 : 1;                 /* Lift force greater than 0.0 lbs */
   unsigned lf_grt_0d0 : 1;                   /* Lift force greater than 0.0 lbs */
   unsigned lf_grt_310 : 1;                   /* Lift force greater than 310 lbs */
   unsigned lf_grt_220 : 1;                   /* Lift force greater than 220 lbs */
   unsigned lf_grt_1100 : 1;                  /* Lift force greater than 1100 lbs */
   unsigned lf_grt_150 : 1;                   /* Lift force greater than 150 lbs */
   unsigned lh_lim_2d0_20d0_2 : 1;            /* Lift height between 2.0 and 20.0 inches */
   unsigned lh_lim_0d75_3d0 : 1;              /* Lift height between 0.75 and 3.0 inches */
   unsigned lf_grt_neg_125 : 1;               /* Lift force greater than negative 125 lbs */
   unsigned lh_les_0d75_2 : 1;                /* Lift height less than 0.75 inches */
   unsigned lf_les_800 : 1;                   /* Lift force less than 800 lbs */
} B3_L5;

typedef struct {
   unsigned lh_lim_22d89_23d09 : 1;           /* Lift height between 22.89 and 23.09 inches */
   unsigned lf_grt_950 : 1;                   /* Lift force greater than 950 lbs */
   unsigned lh_lim_22d85_23d05 : 1;           /* Lift height between 22.85 and 23.05 inches */
   unsigned lf_grt_1400 : 1;                  /* Lift force greater than 1400 lbs */
   unsigned lh_lim_21d8_22d15 : 1;            /* Lift height between 21.8 and 22.15 inches */
   unsigned lh_lim_22d3_24d0 : 1;             /* Lift height between 22.3 and 24.0 inches */
   unsigned lf_grt_125_3 : 1;                 /* Lift force greater than 125 lbs */
   unsigned lf_grt_0d0_6 : 1;                 /* Lift force greater than 0.0 lbs */
   unsigned lf_grt_0d0_5 : 1;                 /* Lift force greater than 0.0 lbs */
   unsigned lf_grt_310_3 : 1;                 /* Lift force greater than 310 lbs */
   unsigned lf_grt_220_3 : 1;                 /* Lift force greater than 220 lbs */
   unsigned lf_grt_150_3 : 1;                 /* Lift force greater than 150 lbs */
   unsigned lh_lim_21d89_22d3_2 : 1;          /* Lift height between 21.89 and 22.3 inches */
   unsigned lf_grt_125_2 : 1;                 /* Lift force greater than 125 lbs */
   unsigned lf_grt_0d0_4 : 1;                 /* Lift force greater than 0.0 lbs */
   unsigned lf_grt_0d0_3 : 1;                 /* Lift force greater than 0.0 lbs */
   unsigned spare_b3_13_15 : 1;              
   unsigned im_ff_uv_on_req : 1;             
   unsigned im_ff_wht_on_req : 1;            
   unsigned alt_bump_dn_delay : 1;            /* Alt Down bump signal */
   unsigned alt_bump_up_delay : 1;            /* Alt Up Bump signal */
   unsigned az_bump_ccw_delay : 1;            /* Az CCW Bump signal */
   unsigned az_bump_cw_delay : 1;             /* Az CW Bump signal */
   unsigned lh_les_6d0_5 : 1;                 /* Lift height less than 6.0 */
   unsigned lh_les_6d0_4 : 1;                 /* Lift height less than 6.0 */
   unsigned lh_les_6d0_3 : 1;                 /* Lift height less than 6.0 */
   unsigned lh_les_6d0_2 : 1;                 /* Lift height less than 6.0 */
   unsigned lh_les_6d0_1 : 1;                 /* Lift height less than 6.0 */
   unsigned lh_les_6d0 : 1;                   /* Lift height less than 6.0 */
   unsigned lf_grt_750 : 1;                   /* Lift force greater then 750 lbs */
   unsigned lh_lim_23d04_23d24 : 1;           /* Lift height between 23.04 and 23.24 inches */
   unsigned lf_grt_950_1 : 1;                 /* Lift force greater than 950 lbs */
} B3_L6;

typedef struct {
   unsigned spare_b3_14_15 : 1;              
   unsigned spare_b3_14_14 : 1;              
   unsigned spare_b3_14_13 : 1;              
   unsigned spare_b3_14_12 : 1;              
   unsigned spare_b3_14_11 : 1;              
   unsigned spare_b3_14_10 : 1;              
   unsigned spare_b3_14_9 : 1;               
   unsigned spare_b3_14_8 : 1;               
   unsigned spare_b3_14_7 : 1;               
   unsigned spare_b3_14_6 : 1;               
   unsigned spare_b3_14_5 : 1;               
   unsigned spare_b3_14_4 : 1;               
   unsigned spare_b3_14_3 : 1;               
   unsigned spare_b3_14_2 : 1;               
   unsigned spare_b3_14_1 : 1;               
   unsigned spare_b3_14_0 : 1;               
   unsigned spare_b3_15_15 : 1;              
   unsigned spare_b3_15_14 : 1;              
   unsigned spare_b3_15_13 : 1;              
   unsigned spare_b3_15_12 : 1;              
   unsigned spare_b3_15_11 : 1;              
   unsigned spare_b3_15_10 : 1;              
   unsigned spare_b3_15_9 : 1;               
   unsigned spare_b3_15_8 : 1;               
   unsigned spare_b3_15_7 : 1;               
   unsigned spare_b3_15_6 : 1;               
   unsigned spare_b3_15_5 : 1;               
   unsigned spare_b3_15_4 : 1;               
   unsigned spare_b3_15_3 : 1;               
   unsigned spare_b3_15_2 : 1;               
   unsigned spare_b3_15_1 : 1;               
   unsigned spare_b3_15_0 : 1;               
} B3_L7;

typedef struct {
   unsigned spare_b3_16_15 : 1;              
   unsigned spare_b3_16_14 : 1;              
   unsigned spare_b3_16_13 : 1;              
   unsigned spare_b3_16_12 : 1;              
   unsigned spare_b3_16_11 : 1;              
   unsigned spare_b3_16_10 : 1;              
   unsigned spare_b3_16_9 : 1;               
   unsigned spare_b3_16_8 : 1;               
   unsigned spare_b3_16_7 : 1;               
   unsigned spare_b3_16_6 : 1;               
   unsigned spare_b3_16_5 : 1;               
   unsigned spare_b3_16_4 : 1;               
   unsigned spare_b3_16_3 : 1;               
   unsigned spare_b3_16_2 : 1;               
   unsigned spare_b3_16_1 : 1;               
   unsigned spare_b3_16_0 : 1;               
   unsigned spare_b3_17_15 : 1;              
   unsigned spare_b3_17_14 : 1;              
   unsigned spare_b3_17_13 : 1;              
   unsigned spare_b3_17_12 : 1;              
   unsigned spare_b3_17_11 : 1;              
   unsigned spare_b3_17_10 : 1;              
   unsigned spare_b3_17_9 : 1;               
   unsigned spare_b3_17_8 : 1;               
   unsigned spare_b3_17_7 : 1;               
   unsigned spare_b3_17_6 : 1;               
   unsigned spare_b3_17_5 : 1;               
   unsigned spare_b3_17_4 : 1;               
   unsigned spare_b3_17_3 : 1;               
   unsigned spare_b3_17_2 : 1;               
   unsigned spare_b3_17_1 : 1;               
   unsigned spare_b3_17_0 : 1;               
} B3_L8;

typedef struct {
   unsigned spare_b3_18_15 : 1;              
   unsigned spare_b3_18_14 : 1;              
   unsigned spare_b3_18_13 : 1;              
   unsigned spare_b3_18_12 : 1;              
   unsigned spare_b3_18_11 : 1;              
   unsigned spare_b3_18_10 : 1;              
   unsigned spare_b3_18_9 : 1;               
   unsigned spare_b3_18_8 : 1;               
   unsigned spare_b3_18_7 : 1;               
   unsigned spare_b3_18_6 : 1;               
   unsigned spare_b3_18_5 : 1;               
   unsigned spare_b3_18_4 : 1;               
   unsigned spare_b3_18_3 : 1;               
   unsigned spare_b3_18_2 : 1;               
   unsigned spare_b3_18_1 : 1;               
   unsigned spare_b3_18_0 : 1;               
   unsigned spare_b3_19_15 : 1;              
   unsigned spare_b3_19_14 : 1;              
   unsigned spare_b3_19_13 : 1;              
   unsigned spare_b3_19_12 : 1;              
   unsigned spare_b3_19_11 : 1;              
   unsigned spare_b3_19_10 : 1;              
   unsigned spare_b3_19_9 : 1;               
   unsigned spare_b3_19_8 : 1;               
   unsigned spare_b3_19_7 : 1;               
   unsigned spare_b3_19_6 : 1;               
   unsigned spare_b3_19_5 : 1;               
   unsigned spare_b3_19_4 : 1;               
   unsigned spare_b3_19_3 : 1;               
   unsigned spare_b3_19_2 : 1;               
   unsigned spare_b3_19_1 : 1;               
   unsigned spare_b3_19_0 : 1;               
} B3_L9;

typedef struct {
   unsigned mcp_clamp_engage_cmd : 1;         /* MCP Clamp engage command */
   unsigned mcp_alt_brk_en_cmd : 1;           /* MCP Altitude brake engage command bit from VME */
   unsigned mcp_alt_brk_dis_cmd : 1;          /* MCP Altitude brake disengage command bit from VME */
   unsigned mcp_az_brk_en_cmd : 1;            /* MCP Azimuth brake engage command bit from VME */
   unsigned mcp_az_brk_dis_cmd : 1;           /* MCP Azimuth brake disengage command bit from VME */
   unsigned mcp_solenoid_engage : 1;          /* MCP command to turn on the instrument lift solenoid.  The solenoid must be on for the lift to work */
   unsigned mcp_pump_on : 1;                  /* MCP command to turn on the instrument lift pump. */
   unsigned mcp_lift_dn_4 : 1;                /* Part of the binary code for down lift speed. */
   unsigned mcp_lift_dn_3 : 1;                /* Part of the binary code for down lift speed. */
   unsigned mcp_lift_dn_2 : 1;                /* Part of the binary code for down lift speed. */
   unsigned mcp_lift_dn_1 : 1;                /* Part of the binary code for down lift speed. */
   unsigned mcp_lift_up_1 : 1;                /* Part of the binary code for up lift speed. */
   unsigned mcp_lift_up_2 : 1;                /* Part of the binary code for up lift speed. */
   unsigned mcp_lift_up_3 : 1;                /* Part of the binary code for up lift speed. */
   unsigned mcp_lift_up_4 : 1;                /* Part of the binary code for up lift speed. */
   unsigned mcp_lift_high_psi : 1;            /* MCP command to turn on the high lift pressure for camera exchange for the instrument lift. */
   unsigned mcp_ff_screen_enable : 1;         /* Enable motion of the flat field screens */
   unsigned mcp_hgcd_lamp_on_cmd : 1;         /* MCP command to turn on the hgcd lamps */
   unsigned mcp_ne_lamp_on_cmd : 1;           /* MCP command to turn the ne lamps on */
   unsigned mcp_ff_lamp_on_cmd : 1;           /* MCP command to turn the ff lamps on */
   unsigned mcp_ff_scrn_opn_cmd : 1;          /* MCP command to open the ff screen. */
   unsigned mcp_slit_latch2_cmd : 1;          /* MCP slithead latch2 control 1=engage 0=disengage */
   unsigned mcp_slit_dr2_cls_cmd : 1;         /* MCP slithead door 2 close command bit from VME */
   unsigned mcp_slit_dr2_opn_cmd : 1;         /* MCP slithead door 2 open command bit from VME */
   unsigned mcp_slit_latch1_cmd : 1;          /* MCP slithead latch 1 control bit from VME 1=engage 0=disengage */
   unsigned mcp_slit_dr1_cls_cmd : 1;         /* MCP slithead door 1 close command bit from VME */
   unsigned mcp_slit_dr1_opn_cmd : 1;         /* MCP slithead door 1 open command bit from VME */
   unsigned mcp_umbilical_on_off : 1;         /* MCP camera umbilical motor on / off command line. 1=on 0=off */
   unsigned mcp_umbilical_up_dn : 1;          /* MCP camera umbilical motor up / down command line. 0=up 1=down */
   unsigned mcp_15deg_stop_ret_c : 1;         /* MCP 15 degree stop remove command */
   unsigned mcp_15deg_stop_ext_c : 1;         /* MCP 15 degree stop insert command */
   unsigned mcp_clamp_disen_cmd : 1;          /* MCP Clamp disengage command */
} B10_L0;

typedef struct {
   unsigned mcp_im_ff_uv_req : 1;            
   unsigned mcp_im_ff_wht_req : 1;           
   unsigned mcp_umbilical_fast : 1;           /* MCP umbilical fast speed command */
   unsigned mcp_ff_screen2_enabl : 1;         /* MCP command to enable flat field screen 2 */
   unsigned mcp_ff_scrn2_opn_cmd : 1;         /* MCP command to open flat field screen 2 */
   unsigned mcp_inst_chg_alert : 1;           /* MCP command to alert the observers for an instrument change malfunction. */
   unsigned mcp_inst_chg_prompt : 1;          /* MCP command to alert the observers for an instrument change prompt. */
   unsigned mcp_sad_latch_opn_cm : 1;         /* MCP command to open the saddle latches. */
   unsigned mcp_sad_latch_cls_cm : 1;         /* MCP command to close the saddle latches. */
   unsigned mcp_sec_latch_opn_cm : 1;         /* MCP command to open the secondary latches. */
   unsigned mcp_sec_latch_cls_cm : 1;         /* MCP command to close the secondary latches. */
   unsigned mcp_pri_latch_opn_cm : 1;         /* MCP command to open the primary latches. */
   unsigned mcp_pri_latch_cls_cm : 1;         /* MCP command to close the primary latches. */
   unsigned mcp_purge_cell_on : 1;            /* MCP command to start the purge cell. */
   unsigned mcp_t_bar_tel : 1;                /* MCP command to latch the t-bar latches */
   unsigned mcp_t_bar_xport : 1;              /* MCP command to unlatch the t-bar latches */
   unsigned : 15;                            
   unsigned velocity_trp_rst_in : 1;          /* MCP command to reset a velocity trip in the slip detection module. */
} B10_L1;

typedef struct {
   unsigned rack_0_grp_0_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned az_bump_ccw : 1;                  /* Azimuth telescope to windscreen counter-clockwise bump switch. */
   unsigned az_bump_cw : 1;                   /* Azimuth telescope to windscreen clockwise bump switch. */
   unsigned rack_0_grp_0_bit_12 : 1;          /* Spare input bit */
   unsigned ops_cart_in_pos : 1;              /* Imager cart in position status bit. */
   unsigned fiber_cart_pos2 : 1;              /* Fiber cartridge cart in position 2 status bit. */
   unsigned fiber_cart_pos1 : 1;              /* Fiber cartridge cart in position 1 status bit. */
   unsigned inst_lift_low_force : 1;          /* Instrument lift in low force status bit. */
   unsigned inst_lift_high_force : 1;         /* Instrument lift in high force status bit. */
   unsigned inst_lift_man : 1;                /* Instrument lift in the manual mode status bit. */
   unsigned inst_lift_dn : 1;                 /* Instrument lift plate in the down position status bit. */
   unsigned inst_lift_sw4 : 1;                /* Instrument lift plate switch 4 status bit. */
   unsigned inst_lift_sw3 : 1;                /* Instrument lift plate switch 3 status bit. */
   unsigned inst_lift_sw2 : 1;                /* Instrument lift plate switch 2 status bit. */
   unsigned inst_lift_sw1 : 1;                /* Instrument lift plate switch 1 status bit. */
   unsigned inst_lift_pump_on : 1;            /* Instrument lift pump on status bit. */
   unsigned low_lvl_light_req : 1;            /* Low lever lighting request to change state of low level lighting from off to on or on to off. */
   unsigned rack_0_grp_1_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_11 : 1;          /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_10 : 1;          /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_9 : 1;           /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_8 : 1;           /* Spare PLC input bit. */
   unsigned rack_0_grp_1_bit_7 : 1;           /* Spare PLC input bit. */
   unsigned optical_bench_cls : 1;            /* T Bar latches closed or on t bars. CCD's unsafe to move camera. */
   unsigned optical_bench_opn : 1;            /* T Bar latches open or off t bars. CCD's safe to move camera. */
   unsigned ops_cart_in_house : 1;            /* Imager operations cart in dog house status bit. */
   unsigned dog_house_door_cls : 1;           /* Dog house door closed status bit. */
   unsigned dog_house_door_opn : 1;           /* Dog house door open status bit. */
   unsigned dog_house_ccw_pad : 1;            /* Dog house ccw bump switch.  Stops telescope and windscreen motion if dog house hits obj. */
   unsigned dog_house_cw_pad : 1;             /* Dog house cw bump switch.  Stops telescope and windscreen motion if dog house hits obj. */
} I1_L0;

typedef struct {
   unsigned spare_i1_l1 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L1;

typedef struct {
   unsigned spare_i1_l2 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L2;

typedef struct {
   unsigned spare_i1_l3 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L3;

typedef struct {
   unsigned open_slit_doors : 1;              /* Latch control box Slithead door Open switch */
   unsigned close_slit_doors : 1;             /* Latch control box Slithead door Close switch */
   unsigned inst_chg_remove_sw : 1;           /* Latch control box Instrument change Remove switch */
   unsigned inst_chg_install_sw : 1;          /* Latch control box Instrument change Install switch */
   unsigned man_mode_switch : 1;              /* Latch control box Manual mode switch */
   unsigned auto_mode_sw : 1;                 /* Latch control box Auto mode switch */
   unsigned off_mode_sw : 1;                  /* Latch control box Off mode switch */
   unsigned iclb_leds_on_cmd : 1;             /* Latch control box LEDs on switch */
   unsigned sad_man_valve_cls : 1;            /* Saddle latch manual valve closed status switch. */
   unsigned sec_man_valve_cls : 1;            /* Secondary latch manual valve closed status switch. */
   unsigned inst_man_valve_cls : 1;           /* Imager latch manual valve closed status switch. */
   unsigned ilcb_pres_good : 1;               /* Instrument latch control box air pressure status switch. */
   unsigned rot_pos_370_ccw : 1;              /* First ccw inhibit switch for the rotator.  Causes a directional inhibit. */
   unsigned rot_neg_190_cw : 1;               /* First cw inhibit switch for the rotator.  Causes a directional inhibit. */
   unsigned rot_inst_chg_b : 1;               /* Second of two switches used to determine rotator is at the instrument change position. */
   unsigned rot_inst_chg_a : 1;               /* First of two switches used to determine the rotator is at the instrument change po */
   unsigned rack_1_grp_1_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_11 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_10 : 1;          /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_9 : 1;           /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_8 : 1;           /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_7 : 1;           /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_6 : 1;           /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_5 : 1;           /* Spare PLC input bit. */
   unsigned rack_1_grp_1_bit_4 : 1;           /* Spare PLC input bit. */
   unsigned tbar_latch_tel_cmd : 1;           /* Tbar latch telescope command input */
   unsigned tbar_latch_xport_cmd : 1;         /* Tbar latch transport command input */
   unsigned slit_latch_lth_cmd : 1;           /* Slit head latch command input */
   unsigned slit_latch_unlth_cmd : 1;         /* Slit head unlatch command input */
} I1_L4;

typedef struct {
   unsigned spare_i1_l5 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L5;

typedef struct {
   unsigned spare_i1_l6 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L6;

typedef struct {
   unsigned spare_i1_l7 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L7;

typedef struct {
   unsigned rack_2_grp_0_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned spec_autofill_on : 1;             /* Spectrograph autofill system connected.  Altitude and Rotator motion disabled. */
   unsigned spec_lens2 : 1;                   /* Spectrographic corrector lens mount position 2 switch. */
   unsigned spec_lens1 : 1;                   /* Spectrographic corrector lens mount position 1 switch. */
   unsigned inst_id3_4 : 1;                   /* Instrument ID block 3 switch 4 status. */
   unsigned inst_id3_3 : 1;                   /* Instrument ID block 3 switch 3 status. */
   unsigned inst_id3_2 : 1;                   /* Instrument ID block 3 switch 2 status. */
   unsigned inst_id3_1 : 1;                   /* Instrument ID block 3 switch 1 status. */
   unsigned inst_id2_4 : 1;                   /* Instrument ID block 2 switch 4 status. */
   unsigned inst_id2_3 : 1;                   /* Instrument ID block 2 switch 3 status. */
   unsigned inst_id2_2 : 1;                   /* Instrument ID block 2 switch 2 status. */
   unsigned inst_id2_1 : 1;                   /* Instrument ID block 2 switch 1 status. */
   unsigned inst_id1_4 : 1;                   /* Instrument ID block 1 switch 4 status. */
   unsigned inst_id1_3 : 1;                   /* Instrument ID block 1 switch 3 status. */
   unsigned inst_id1_2 : 1;                   /* Instrument ID block 1 switch 2 status. */
   unsigned inst_id1_1 : 1;                   /* Instrument ID block 1 switch 1 status. */
   unsigned safety_latch2_cls : 1;            /* Protection bolt 2 closed status. */
   unsigned safety_latch2_opn : 1;            /* Protection bolt 2 open status. */
   unsigned safety_latch1_cls : 1;            /* Protection bolt 1 closed status. */
   unsigned safety_latch1_opn : 1;            /* Protection bolt 1 open status. */
   unsigned sec_latch3_cls : 1;               /* Secondary latch 3 closed status. */
   unsigned sec_latch3_opn : 1;               /* Secondary latch 3 open status. */
   unsigned sec_latch2_cls : 1;               /* Secondary latch 2 closed status. */
   unsigned sec_latch2_opn : 1;               /* Secondary latch 2 open status. */
   unsigned sec_latch1_cls : 1;               /* Secondary latch 1 closed status. */
   unsigned sec_latch1_opn : 1;               /* Secondary latch 1 open status. */
   unsigned pri_latch3_cls : 1;               /* Primary latch 3 closed status. */
   unsigned pri_latch3_opn : 1;               /* Primary latch 3 open status. */
   unsigned pri_latch2_cls : 1;               /* Primary latch 2 closed status. */
   unsigned pri_latch2_opn : 1;               /* Primary latch 2 open status. */
   unsigned pri_latch1_cls : 1;               /* Primary latch 1 closed status. */
   unsigned pri_latch1_opn : 1;               /* Primary latch 1 open status. */
} I1_L8;

typedef struct {
   unsigned rack_2_grp_2_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_2_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned slit_head_2_in_place : 1;         /* Cartridge 2 in place status. */
   unsigned slit_head_latch2_ext : 1;         /* Cartridge latch 2 extended status. */
   unsigned slit_head_door2_cls : 1;          /* Slit head door 2 closed status. */
   unsigned slit_head_door2_opn : 1;          /* Slit head door 2 open status. */
   unsigned slit_head_1_in_place : 1;         /* Cartridge 1 in place status. */
   unsigned slit_head_latch1_ext : 1;         /* Cartridge latch 1 extended status. */
   unsigned slit_head_door1_cls : 1;          /* Slit head door 1 closed status. */
   unsigned slit_head_door1_opn : 1;          /* Slit head door 1 open status. */
   unsigned sad_mount2 : 1;                   /* Saddle mount position 2 switch. */
   unsigned sad_mount1 : 1;                   /* Saddle mount position 1 switch. */
   unsigned sad_latch2_cls : 1;               /* Saddle latch 2 closed status. */
   unsigned sad_latch2_opn : 1;               /* Saddle latch 2 open status. */
   unsigned sad_latch1_cls : 1;               /* Saddle latch 1 closed status. */
   unsigned sad_latch1_opn : 1;               /* Saddle latch 1 open status. */
   unsigned : 16;                            
} I1_L9;

typedef struct {
   unsigned rack_2_grp_4_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_11 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_10 : 1;          /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_9 : 1;           /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_8 : 1;           /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_7 : 1;           /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_6 : 1;           /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_5 : 1;           /* Spare PLC input bit. */
   unsigned rack_2_grp_4_bit_4 : 1;           /* Spare PLC input bit. */
   unsigned sec_mir_force_limits : 1;         /* Secondary Mirror Force Limit bit. */
   unsigned alt_bump_dn : 1;                  /* Altitude telescope to windscreen down bump switch. */
   unsigned alt_bump_up : 1;                  /* Altitude telescope to windscreen up bump switch. */
   unsigned purge_air_pressur_sw : 1;         /* Purge air pressure switch */
   unsigned : 16;                            
} I1_L10;

typedef struct {
   unsigned spare_i1_l11 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L11;

typedef struct {
   unsigned spare_i1_l12 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 15;                            
   unsigned rack_3_grp_1_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_1_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_1_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_1_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_1_bit_11 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_1_bit_10 : 1;          /* Spare PLC input bit. */
   unsigned man_lift_dn_4 : 1;                /* Manual control lift down 4 switch. */
   unsigned man_lift_dn_3 : 1;                /* Manual control lift down 3 switch. */
   unsigned man_lift_dn_2 : 1;                /* Manual control lift down 2 switch. */
   unsigned man_lift_dn_1 : 1;                /* Manual control lift down 1 switch. */
   unsigned man_lift_up_4 : 1;                /* Manual control lift up 4 switch. */
   unsigned man_lift_up_3 : 1;                /* Manual control lift up 3 switch. */
   unsigned man_lift_up_2 : 1;                /* Manual control lift up 2 switch. */
   unsigned man_lift_up_1 : 1;                /* Manual control lift up 1 switch. */
   unsigned inst_lift_auto : 1;               /* Instrument lift in automatic position. */
   unsigned rack_3_grp_1_bit_0 : 1;           /* Spare PLC input bit. */
} I1_L12;

typedef struct {
   unsigned leaf_8_closed_stat : 1;           /* Leaf Screen 8 closed status bit */
   unsigned leaf_8_open_stat : 1;             /* Leaf Screen 8 open status bit */
   unsigned leaf_7_closed_stat : 1;           /* Leaf Screen 7 closed status bit */
   unsigned leaf_7_open_stat : 1;             /* Leaf Screen 7 open status bit */
   unsigned leaf_6_closed_stat : 1;           /* Leaf Screen 6 closed status bit */
   unsigned leaf_6_open_stat : 1;             /* Leaf Screen 6 open status bit */
   unsigned leaf_5_closed_stat : 1;           /* Leaf Screen 5 closed status bit */
   unsigned leaf_5_open_stat : 1;             /* Leaf Screen 5 open status bit */
   unsigned leaf_4_closed_stat : 1;           /* Leaf Screen 4 closed status bit */
   unsigned leaf_4_open_stat : 1;             /* Leaf Screen 4 open status bit */
   unsigned leaf_3_closed_stat : 1;           /* Leaf Screen 3 closed status bit */
   unsigned leaf_3_open_stat : 1;             /* Leaf Screen 3 open status bit */
   unsigned leaf_2_closed_stat : 1;           /* Leaf Screen 2 closed status bit */
   unsigned leaf_2_open_stat : 1;             /* Leaf Screen 2 open status bit */
   unsigned leaf_1_closed_stat : 1;           /* Leaf Screen 1 closed status bit */
   unsigned leaf_1_open_stat : 1;             /* Leaf Screen 1 open status bit */
   unsigned rack_3_grp_3_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_3_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_3_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_3_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned hgcd_4_stat : 1;                  /* Mercury-cadmium lamp 4 status bit */
   unsigned hgcd_3_stat : 1;                  /* Mercury-cadmium lamp 3 status bit */
   unsigned hgcd_2_stat : 1;                  /* Mercury-cadmium lamp 2 status bit */
   unsigned hgcd_1_stat : 1;                  /* Mercury-cadmium lamp 1 status bit */
   unsigned ne_4_stat : 1;                    /* Neon lamp 4 status bit */
   unsigned ne_3_stat : 1;                    /* Neon lamp 3 status bit */
   unsigned ne_2_stat : 1;                    /* Neon lamp 2 status bit */
   unsigned ne_1_stat : 1;                    /* Neon lamp 1 status bit */
   unsigned ff_4_stat : 1;                    /* Flatfield lamp 4 status bit */
   unsigned ff_3_stat : 1;                    /* Flatfield lamp 3 status bit */
   unsigned ff_2_stat : 1;                    /* Flatfield lamp 2 status bit */
   unsigned ff_1_stat : 1;                    /* Flatfield lamp 1 status bit */
} I1_L13;

typedef struct {
   unsigned rack_3_grp_4_bit_15 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_14 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_13 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_12 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_11 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_10 : 1;          /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_9 : 1;           /* Spare PLC input bit. */
   unsigned rack_3_grp_4_bit_8 : 1;           /* Spare PLC input bit. */
   unsigned man_im_ff_uv_req : 1;             /* Manual input to turn on the imager ff UV lamps */
   unsigned man_im_ff_wht_req : 1;            /* Manual input to turn on the imager ff white lamps */
   unsigned ff_man_cont_enable : 1;           /* Manual flatfield control module connected to telescope.  Alt and Az motion disabled. */
   unsigned man_hgcd_lamp_on_cmd : 1;         /* Manual mercury cadmium lamps on command */
   unsigned man_ne_lamp_on_cmd : 1;           /* Manual Neon lamps on command */
   unsigned man_ff_lamp_on_cmd : 1;           /* Manual flatfield lamps on command */
   unsigned man_ff_scrn_en_cmd : 1;           /* Manual flatfield screen enable command */
   unsigned man_ff_scrn_opn_cmd : 1;          /* Manual flatfield screen open command */
   unsigned : 16;                            
} I1_L14;

typedef struct {
   unsigned spare_i1_l15 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} I1_L15;

typedef struct {
   unsigned dcm_1_status_word : 16;           /* Direct communications module 1 status word. */
   unsigned spare : 1;                        /* Spare PLC input bit. */
   unsigned : 9;                             
   unsigned wind_alt_perm : 1;                /* Windscreen altitude motion permit bit. */
   unsigned wind_az_perm : 1;                 /* Windscreen azimuth motion permit bit. */
   unsigned wind_alt1_fault : 1;              /* Windscreen altitude amplifier fault bit. */
   unsigned wind_az3_fault : 1;               /* Windscreen azimuth amplifier 3 fault bit. */
   unsigned wind_az2_fault : 1;               /* Windscreen azimuth amplifier 2 fault bit. */
   unsigned wind_az1_fault : 1;               /* Windscreen azimuth amplifier 1 fault bit. */
} I2_L0;

typedef struct {
   unsigned az_pid_status : 16;               /* Azimuth PID Status Word */
   unsigned az_lvdt_error : 16;               /* Azimuth LVDT error analog value. */
} I2_L1;

typedef struct {
   unsigned az_pri_drv : 16;                  /* Azimuth primary drive value. */
   unsigned az_feed_fwd_drv : 16;             /* Azimuth feed forward drive value. */
} I2_L2;

typedef struct {
   unsigned dcm_1_word6_spare : 16;           /* DCM 1 spare word */
   unsigned dcm_1_word7_spare : 16;           /* DCM 1 spare word */
} I2_L3;

typedef struct {
   unsigned dcm_2_status_word : 16;           /* Direct communications module 2 status word. */
   unsigned dcm_2_word1_spare : 16;           /* DCM 2 spare word */
} I2_L4;

typedef struct {
   unsigned alt_pid_status : 16;              /* Altitude PID Status Word */
   unsigned alt_lvdt_error : 16;              /* Altitude LVDT error analog value. */
} I2_L5;

typedef struct {
   unsigned alt_pri_drv : 16;                 /* Altitude primary drive value. */
   unsigned dcm_2_word5_spare : 16;           /* DCM 2 spare word */
} I2_L6;

typedef struct {
   unsigned dcm_2_word6_spare : 16;           /* DCM 2 spare word */
   unsigned dcm_2_word7_spare : 16;           /* DCM 2 spare word */
} I2_L7;

typedef struct {
   unsigned az_1_voltage : 16;                /* Azimuth motor 1 voltage. */
   unsigned az_1_current : 16;                /* Azimuth motor 1 current. */
} I3_L0;

typedef struct {
   unsigned az_2_voltage : 16;                /* Azimuth motor 2 voltage. */
   unsigned az_2_current : 16;                /* Azimuth motor 2 current. */
} I3_L1;

typedef struct {
   unsigned alt_1_voltage : 16;               /* Altitude motor 1 voltage. */
   unsigned alt_1_current : 16;               /* Altitude motor 1 current. */
} I3_L2;

typedef struct {
   unsigned alt_2_voltage : 16;               /* Altitude motor 2 voltage. */
   unsigned alt_2_current : 16;               /* Altitude motor 2 current. */
} I3_L3;

typedef struct {
   unsigned alt_position : 16;                /* Altitude clinometer raw position value. */
   unsigned rot_1_voltage : 16;               /* Rotator motor voltage. */
} I4_L0;

typedef struct {
   unsigned rot_1_current : 16;               /* Rotator motor current. */
   unsigned umbilical_dist : 16;              /* Camera umbilical distance value. */
} I4_L1;

typedef struct {
   unsigned inst_lift_force : 16;             /* Instrument lift strain gauge value. */
   unsigned inst_lift_dist : 16;              /* Instrument lift string pot distance value. Scale = .001 */
} I4_L2;

typedef struct {
   unsigned i_4_analog_6_spare : 16;          /* Spare analog channel. */
   unsigned i_4_analog_7_spare : 16;          /* Spare analog channel. */
} I4_L3;

typedef struct {
   unsigned counterweight_1_pos : 16;         /* Counterweight #1 string pot position value */
   unsigned counterweight_2_pos : 16;         /* Counterweight #2 string pot position value */
} I5_L0;

typedef struct {
   unsigned counterweight_3_pos : 16;         /* Counterweight #3 string pot position value */
   unsigned counterweight_4_pos : 16;         /* Counterweight #4 string pot position value */
} I5_L1;

typedef struct {
   unsigned i_5_analog_4_spare : 16;          /* Spare analog channel. */
   unsigned i_5_analog_5_spare : 16;          /* Spare analog channel. */
} I5_L2;

typedef struct {
   unsigned i_5_analog_6_spare : 16;          /* Spare analog channel. */
   unsigned i_5_analog_7_spare : 16;          /* Spare analog channel. */
} I5_L3;

typedef struct {
   unsigned mcp_watchdog_timer : 1;           /* MCP Watchdog Timer. Removes drive amplifier reference if MCP Fault. */
   unsigned nw_fork_stop : 1;                 /* North West Fork E-Stop */
   unsigned s_wind_stop : 1;                  /* South wind screen e-stop */
   unsigned w_lower_stop : 1;                 /* West lower level e-stop */
   unsigned e_lower_stop : 1;                 /* East lower level e-stop */
   unsigned s_lower_stop : 1;                 /* South lower level e-stop */
   unsigned n_lower_stop : 1;                 /* North lower level e-stop */
   unsigned w_rail_stop : 1;                  /* West railing e-stop */
   unsigned s_rail_stop : 1;                  /* South railing e-stop */
   unsigned n_rail_stop : 1;                  /* North railing e-stop */
   unsigned n_fork_stop : 1;                  /* North fork e-stop */
   unsigned n_wind_stop : 1;                  /* North wind screen e-stop */
   unsigned fiber_signal_loss : 1;            /* Control room e-stop fiber link signal loss indicator. Currently not implemented. */
   unsigned spare_s1_c2 : 1;                  /* Spare splitter chassis channel. */
   unsigned cr_stop : 1;                      /* Control Room e-stop switch. */
   unsigned tcc_stop : 1;                     /* TCC inhibit input hook.  Currently not implemented. */
   unsigned az_stow_3b : 1;                   /* Azimuth stow position switch status */
   unsigned wind_az_plc_perm_in : 1;          /* Wind screen PLC permit status */
   unsigned az_plc_perm_in : 1;               /* Azimuth PLC permit status */
   unsigned wind_az_mtr_perm_in : 1;          /* Wind screen azimuth motor permit status */
   unsigned az_mtr2_perm_in : 1;              /* Azimuth motor 2 permit status */
   unsigned az_mtr1_perm_in : 1;              /* Azimuth motor 1 permit status */
   unsigned az_mtr_ccw_perm_in : 1;           /* Azimuth motor CCW permit status */
   unsigned az_mtr_cw_perm_in : 1;            /* Azimuth motor CW permit status */
   unsigned az_stow_3a : 1;                   /* Azimuth stow position switch status */
   unsigned wind_alt_plc_perm_in : 1;         /* Wind screen PLC permit status */
   unsigned alt_plc_perm_in : 1;              /* Altitude PLC permit status */
   unsigned wind_alt_mtr_perm_in : 1;         /* Wind screen altitude motor permit in */
   unsigned alt_mtr2_perm_in : 1;             /* Altitude motor 2 permit status */
   unsigned alt_mtr1_perm_in : 1;             /* Altitude motor 1 permit status */
   unsigned alt_mtr_dn_perm_in : 1;           /* Altitude motor down permit status */
   unsigned alt_mtr_up_perm_in : 1;           /* Altitude motor up permit status */
} I6_L0;

typedef struct {
   unsigned az_stow_1b : 1;                   /* Azimuth stow position status */
   unsigned az_stow_1a : 1;                   /* Azimuth stow position status */
   unsigned alt_grt_18d6_limit_1 : 1;         /* Altitude greater than 18.6 degree limit status */
   unsigned az_109_131_limit_1 : 1;           /* Azimuth between 109 to 131 degrees status */
   unsigned bldg_on_alt : 1;                  /* Building on status for altitude logic */
   unsigned alt_les_90d5_limit : 1;           /* Altitude less than 90.5 degree limit status */
   unsigned alt_locking_pin_out : 1;          /* Altitude locking pin out status */
   unsigned alt_grt_0d3_limit : 1;            /* Altitude greater then 0.3 degree limit status */
   unsigned alt_les_2d5_limit : 1;            /* Altitude greater then 2.5 degree limit status */
   unsigned hatch_cls : 1;                    /* Hatch closed status switch */
   unsigned rot_plc_perm_in : 1;              /* Rotator PLC permit status */
   unsigned bldg_perm_in : 1;                 /* Building motion permit status */
   unsigned spare_s5_c3 : 1;                  /* Spare splitter chassis channel */
   unsigned rot_mtr_perm_in : 1;              /* Rotator motor permit status */
   unsigned rot_mtr_ccw_perm_in : 1;          /* Rotator motor CCW permit status */
   unsigned rot_mtr_cw_perm_in : 1;           /* Rotator motor CW permit status */
   unsigned spare_s8_c7 : 1;                  /* Spare splitter chassis channel */
   unsigned spare_s8_c6 : 1;                  /* Spare splitter chassis channel */
   unsigned az_pos_445b_ccw : 1;              /* Azimuth less than +445 degree limit status */
   unsigned az_neg_201b_cw : 1;               /* Azimuth greater than -201 degrees status */
   unsigned az_pos_445a_ccw : 1;              /* Azimuth less than +445 degree limit status */
   unsigned az_neg_201a_cw : 1;               /* Azimuth greater then -201 degrees status */
   unsigned az_dir_ccw : 1;                   /* Azimuth direction CCW status */
   unsigned az_dir_cw : 1;                    /* Azimuth direction CW status */
   unsigned alt_velocity_limit : 1;           /* Altitude velocity limit status */
   unsigned alt_slip : 1;                     /* Altitude slip detection status */
   unsigned alt_grt_18d6_limit_2 : 1;         /* Altitude greater than 18.6 degree limit status */
   unsigned deg_15_stop_ext : 1;              /* 15 degree stop extended status */
   unsigned az_stow_2b : 1;                   /* Azimuth stow position status */
   unsigned az_stow_2a : 1;                   /* Azimuth stow position status */
   unsigned bldg_clear_alt : 1;               /* Building clear status for altitude logic */
   unsigned alt_grt_83_limit_1 : 1;           /* Altitude greater than 83 degree limit status */
} I7_L0;

typedef struct {
   unsigned rot_velocity_limit : 1;           /* Rotator velocity limit status */
   unsigned rot_slip : 1;                     /* Rotator slip detection status */
   unsigned rot_pos_380b_ccw : 1;             /* Rotator less than +300 degree limit status */
   unsigned rot_neg_200b_cw : 1;              /* Rotator greater then -200 degrees status */
   unsigned rot_pos_380a_ccw : 1;             /* Rotator less than +300 degree limit status */
   unsigned rot_neg_200a_cw : 1;              /* Rotator greater then -200 degrees status */
   unsigned rot_dir_ccw : 1;                  /* Rotator direction CCW status */
   unsigned rot_dir_cw : 1;                   /* Rotator direction CW status */
   unsigned az_velocity_limit : 1;            /* Azimuth velocity limit status */
   unsigned az_slip : 1;                      /* Azimuth slip detection status */
   unsigned spare_s9_c5 : 1;                  /* Spare splitter chassis channel */
   unsigned bldg_clear_az : 1;                /* Building clear status for Azimuth logic */
   unsigned alt_grt_83_limit_2 : 1;           /* Altitude greater then 83 degree limit status */
   unsigned az_109_131_limit_2 : 1;           /* Azimuth between 109 and 131 degrees limit status */
   unsigned bldg_on_az : 1;                   /* Building on status for Azimuth logic */
   unsigned alt_grt_18d6_limit_3 : 1;         /* Altitude greater than 18.6 degree limit status */
   unsigned t_bar_xport_stat : 1;             /* Camera T-Bar latch latched status */
   unsigned in_8_bit_30_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_8_bit_29_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_8_bit_28_spare : 1;            /* Spare PLC Input Bit */
   unsigned deg_15_stop_ret : 1;              /* 15 degree stop retracted (will not stop telescope altitude motion) */
   unsigned e_stop_byp_sw : 1;                /* E-stop bypass strobe enable key switch status */
   unsigned umbilical_strain_sw : 1;          /* Camera umbilical strain switch status */
   unsigned rot_mtr_rdy : 1;                  /* Rotator motor ready status */
   unsigned alt_mtr2_rdy : 1;                 /* Altitude motor 2 ready status */
   unsigned alt_mtr1_rdy : 1;                 /* Altitude motor 1 ready status */
   unsigned az_mtr2_rdy : 1;                  /* Azimuth motor 2 ready status */
   unsigned az_mtr1_rdy : 1;                  /* Azimuth motor 1 ready status */
   unsigned az_pos_440_ccw : 1;               /* Azimuth less than +440 degree limit status */
   unsigned az_neg_196_cw : 1;                /* Azimuth greater than -196 degree limit status */
   unsigned az_110_130_limit : 1;             /* Azimuth between 110 and 130 degree limit status */
   unsigned az_stow_cntr_sw : 1;              /* Azimuth stow center switch status */
} I8_L0;

typedef struct {
   unsigned in_9_bit_15_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_9_bit_14_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_9_bit_13_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_9_bit_12_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_9_bit_11_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_9_bit_10_spare : 1;            /* Spare PLC Input Bit */
   unsigned alt_locking_pin_in : 1;           /* Altitude Locking Pin In place bit. */
   unsigned solenoid_engage_sw : 1;           /* Solenoid engage (dead mans switch) */
   unsigned low_lvl_lighting_req : 1;         /* Low lever lighting state change request bit. */
   unsigned alt_brake_dis_stat : 1;           /* Altitude brake disabled status */
   unsigned alt_brake_en_stat : 1;            /* Altitude brake enabled status */
   unsigned az_brake_dis_stat : 1;            /* Azimuth brake disabled status */
   unsigned az_brake_en_stat : 1;             /* Azimuth brake enabled status */
   unsigned clamp_dis_stat : 1;               /* Clamp disengaged status */
   unsigned clamp_en_stat : 1;                /* Clamp engaged status */
   unsigned t_bar_tel_stat : 1;               /* Camera T-Bar latch unlatched status */
   unsigned s2_c7_mcp_wtchdg_byp : 1;         /* MCP watchdog bypass monitor status */
   unsigned s2_c6_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c5_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c4_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c3_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c2_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c1_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s2_c0_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c7_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c6_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c5_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c4_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c3_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c2_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c1_bypass_sw : 1;              /* E-Stop bypass monitor status */
   unsigned s1_c0_bypass_sw : 1;              /* E-Stop bypass monitor status */
} I9_L0;

typedef struct {
   unsigned in_10_bit_15_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_14_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_13_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_12_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_11_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_10_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_9_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_8_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_7_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_6_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_5_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_4_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_3_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_2_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_1_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_0_spare : 1;            /* Spare PLC Input Bit */
   unsigned in_10_bit_31_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_30_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_29_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_28_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_27_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_26_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_25_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_24_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_23_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_22_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_21_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_20_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_19_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_18_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_17_spare : 1;           /* Spare PLC Input Bit */
   unsigned in_10_bit_16_spare : 1;           /* Spare PLC Input Bit */
} I10_L0;

typedef struct {
   unsigned spare_o1_l0 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L0;

typedef struct {
   unsigned low_lvl_light_2 : 1;              /* Low level lighting enable bit 2 */
   unsigned low_lvl_light_1 : 1;              /* Low level lighting enable bit 1 */
   unsigned az_stow_light : 1;                /* Azimuth stow indicator light output bit */
   unsigned stop_bypass_strobe : 1;           /* E-Stop bypass warning strobe enable bit */
   unsigned az_stow_center_light : 1;         /* Turns on the azimuth stow position center light. */
   unsigned rack_0_grp_2_bit_10 : 1;          /* Spare PLC Output Bit */
   unsigned lamp_on_enable : 1;               /* NE and HGCD lamp enable relay. */
   unsigned inst_lift_dn_4 : 1;               /* Instrument lift down speed bit 4 */
   unsigned inst_lift_dn_3 : 1;               /* Instrument lift down speed bit 3 */
   unsigned inst_lift_dn_2 : 1;               /* Instrument lift down speed bit 2 */
   unsigned inst_lift_dn_1 : 1;               /* Instrument lift down speed bit 1 */
   unsigned inst_lift_up_1 : 1;               /* Instrument lift up speed bit 1 */
   unsigned inst_lift_up_2 : 1;               /* Instrument lift up speed bit 2 */
   unsigned inst_lift_up_3 : 1;               /* Instrument lift up speed bit 3 */
   unsigned inst_lift_up_4 : 1;               /* Instrument lift up speed bit 4 */
   unsigned inst_lift_high_psi : 1;           /* Instrument lift high pressure enable bit */
   unsigned : 16;                            
} O1_L1;

typedef struct {
   unsigned spare_o1_l2 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L2;

typedef struct {
   unsigned spare_01_l3 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L3;

typedef struct {
   unsigned spare_o1_l4 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L4;

typedef struct {
   unsigned slit1_latched_led : 1;            /* Slit head 1  latched LED */
   unsigned slit1_unlatched_led : 1;          /* Slit head 1  unltched LED */
   unsigned lift_down_led : 1;                /* Lift plate down LED */
   unsigned cart_in_place_led : 1;            /* Cart in  place LED */
   unsigned cam_crt_in_house_led : 1;         /* Camera cart in dog house LED */
   unsigned dog_door_open_led : 1;            /* Dog house door not closed LED */
   unsigned jhook_in_place_led : 1;           /* J Hook in place LED */
   unsigned sad_in_place_led : 1;             /* Saddle in place LED */
   unsigned eng_in_place_led : 1;             /* Eng Camera in place LED */
   unsigned cartg_in_place_led : 1;           /* Cartridge in place LED */
   unsigned cam_in_place_led : 1;             /* Camera in place LED */
   unsigned cor_in_place_led : 1;             /* Corrector in place LED */
   unsigned eng_on_lift_led : 1;              /* Eng Camera on Lift LED */
   unsigned cartg_on_lift_led : 1;            /* Cartridge on Lift LED */
   unsigned cam_on_lift_led : 1;              /* Camera on Lift LED */
   unsigned cor_on_lift_led : 1;              /* Corrector on Lift LED */
   unsigned sec_latch2_cls_led : 1;           /* ILCB Secondary latch 2 closed LED */
   unsigned sec_latch2_opn_led : 1;           /* ILCB Secondary latch 2 open LED */
   unsigned sec_latch1_cls_led : 1;           /* ILCB Secondary latch 1 closed LED */
   unsigned sec_latch1_opn_led : 1;           /* ILCB Secondary latch 1 open LED */
   unsigned inst_latch_perm : 1;              /* Instrument latch latch permit */
   unsigned inst_unlatch_perm : 1;            /* Instrument latch unlatch permit */
   unsigned inst_latch3_cls_led : 1;          /* ILCB Instrument latch 3 closed LED */
   unsigned inst_latch3_opn_led : 1;          /* ILCB Instrument latch 3 open LED */
   unsigned inst_latch2_cls_led : 1;          /* ILCB Instrument latch 2 closed LED */
   unsigned inst_latch2_opn_led : 1;          /* ILCB Instrument latch 2 open LED */
   unsigned inst_latch1_cls_led : 1;          /* ILCB Instrument latch 1 closed LED */
   unsigned inst_latch1_opn_led : 1;          /* ILCB Instrument latch 1 open LED */
   unsigned slit_latch_prm_led : 1;           /* Slit head latch permit LED */
   unsigned slit_unlatch_prm_led : 1;         /* Slit head unlatch permit LED */
   unsigned slit2_latched_led : 1;            /* Slit head 2 latched LED */
   unsigned slit2_unlatched_led : 1;          /* Slit head 2 unltched LED */
} O1_L5;

typedef struct {
   unsigned slit_dr_opn_perm_led : 1;         /* Slit head door open permit LED */
   unsigned slit_dr_cls_perm_led : 1;         /* Slit head door close permit LED */
   unsigned tbar_tel_perm_led : 1;            /* T Bar latch telescope permit LED */
   unsigned tbar_xport_perm_led : 1;          /* T Bar latch transport permit LED */
   unsigned tbar_tel_led : 1;                 /* T Bar latch telescope status LED */
   unsigned tbar_xport_led : 1;               /* T Bar latch transport status LED */
   unsigned sad_latch_perm : 1;               /* Saddle latch latch permit */
   unsigned sad_unlatch_perm : 1;             /* Saddle latch unlatch permit */
   unsigned sad_latch2_cls_led : 1;           /* ILCB Saddle latch 2 closed status LED */
   unsigned sad_latch2_opn_led : 1;           /* ILCB Saddle latch 2 open status LED */
   unsigned sad_latch1_cls_led : 1;           /* ILCB Saddle latch 1 closed status LED */
   unsigned sad_latch1_opn_led : 1;           /* ILCB Saddle latch 1 open status LED */
   unsigned sec_latch_perm : 1;               /* Secondary latch latch permit */
   unsigned sec_unlatch_perm : 1;             /* Secondary latch unlatch permit */
   unsigned sec_latch3_cls_led : 1;           /* ILCB Secondary latch 3 closed LED */
   unsigned sec_latch3_opn_led : 1;           /* ILCB Secondary latch 3 open LED */
   unsigned rack_1_grp_5_bit_15 : 1;          /* Spare PLC output bit. */
   unsigned rack_1_grp_5_bit_14 : 1;          /* Spare PLC output bit. */
   unsigned rack_1_grp_5_bit_13 : 1;          /* Spare PLC output bit. */
   unsigned saf_latch2_cls_led : 1;           /* ILCB Protection bolt 2 closed LED */
   unsigned saf_latch2_opn_led : 1;           /* ILCB Protection bolt 2 open LED */
   unsigned saf_latch1_cls_led : 1;           /* ILCB Protection bolt 1 closed LED */
   unsigned saf_latch1_opn_led : 1;           /* ILCB Protection bolt 1 open LED */
   unsigned manual_mode_led : 1;              /* Manual mode status LED */
   unsigned sad_latch_air_led : 1;            /* Saddle latch air on status LED */
   unsigned sec_latch_air_led : 1;            /* Secondary latch air on status LED */
   unsigned inst_latch_air_led : 1;           /* Instrument latch air on status LED */
   unsigned ilcb_pres_led : 1;                /* ILCB air pressure good LED */
   unsigned slit_dr2_opn_led : 1;             /* Slit head door 2 open status LED */
   unsigned slit_dr2_cls_led : 1;             /* Slit head door 2 closed status LED */
   unsigned slit_dr1_opn_led : 1;             /* Slit head door 1 open status LED */
   unsigned slit_dr1_cls_led : 1;             /* Slit head door 1 closed status LED */
} O1_L6;

typedef struct {
   unsigned spare_o1_l7 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L7;

typedef struct {
   unsigned spare_o1_l8 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L8;

typedef struct {
   unsigned spare_o1_l9 : 1;                  /* MCP Place Holder. Not used in logic code. */
   unsigned : 15;                            
   unsigned rack_2_grp_3_bit_15 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_14 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_13 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_12 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_11 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_10 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_9 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_8 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_7 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_3_bit_6 : 1;           /* Spare PLC output bit. */
   unsigned slit_latch2_ext_perm : 1;         /* Slithead latch 2 extended permit */
   unsigned slit_dr2_opn_perm : 1;            /* Slithead door 2 open permit */
   unsigned slit_dr2_cls_perm : 1;            /* Slithead door 2 close permit */
   unsigned slit_latch1_ext_perm : 1;         /* Slithead latch 1 extended permit */
   unsigned slit_dr1_opn_perm : 1;            /* Slithead door 1 open permit */
   unsigned slit_dr1_cls_perm : 1;            /* Slithead door 1 close permit */
} O1_L9;

typedef struct {
   unsigned spare_o1_l10 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 15;                            
   unsigned audio_warning_2 : 1;              /* Audio warning used for instrument change error */
   unsigned rack_2_grp_5_bit_14 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_13 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_12 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_11 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_10 : 1;          /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_9 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_8 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_7 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_6 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_5 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_4 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_3 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_2 : 1;           /* Spare PLC output bit. */
   unsigned rack_2_grp_5_bit_1 : 1;           /* Spare PLC output bit. */
   unsigned purge_valve_permit : 1;           /* Purge valve permit */
} O1_L10;

typedef struct {
   unsigned spare_o1_l11 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L11;

typedef struct {
   unsigned spare_o1_l12 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L12;

typedef struct {
   unsigned spare_o1_l13 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L13;

typedef struct {
   unsigned spare_o1_l14 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 15;                            
   unsigned audio_warning_1 : 1;              /* Audio warning used for instrument change error */
   unsigned rack_4_grp_5_bit_14 : 1;          /* Spare PLC output bit. */
   unsigned rack_4_grp_5_bit_13 : 1;          /* Spare PLC output bit. */
   unsigned rack_4_grp_5_bit_12 : 1;          /* Spare PLC output bit. */
   unsigned rack_4_grp_5_bit_11 : 1;          /* Spare PLC output bit. */
   unsigned rack_4_grp_5_bit_10 : 1;          /* Spare PLC output bit. */
   unsigned rack_4_grp_5_bit_9 : 1;           /* Spare PLC output bit. */
   unsigned im_ff_uv_on_pmt : 1;              /* Imager flat field uv lamp on permit */
   unsigned im_ff_wht_on_pmt : 1;             /* Imager flat field white lamp on permit */
   unsigned ff_screen2_enable_pm : 1;         /* Spare PLC output bit. */
   unsigned ff_screen2_open_pmt : 1;          /* Spare PLC output bit. */
   unsigned hgcd_lamps_on_pmt : 1;            /* Mercury cadmium lamps on permit */
   unsigned ne_lamps_on_pmt : 1;              /* Neon lamps on permit */
   unsigned ff_lamps_on_pmt : 1;              /* Flatfield lamps on permit */
   unsigned ff_screen_enable_pmt : 1;         /* Flatfield screen enable permit */
   unsigned ff_screen_open_pmt : 1;           /* Flatfield screen open permit */
} O1_L14;

typedef struct {
   unsigned spare_o1_l15 : 1;                 /* MCP Place Holder. Not used in logic code. */
   unsigned : 31;                            
} O1_L15;

typedef struct {
   unsigned : 26;                            
   unsigned out_2_bit_21_spare : 1;           /* Spare PLC output bit */
   unsigned out_2_bit_20_spare : 1;           /* Spare PLC output bit. */
   unsigned wind_mtr_dn_perm : 1;             /* Windscreen motor down permit */
   unsigned wind_mtr_up_perm : 1;             /* Windscreen motor up permit */
   unsigned wind_mtr_ccw_perm : 1;            /* Windscreen motor CCW permit */
   unsigned wind_mtr_cw_perm : 1;             /* Windscreen motor CW permit */
} O2_L0;

typedef struct {
   unsigned az_1_voltage_config : 16;         /* Analog input module 3 configuration control word 0 */
   unsigned az_1_current_config : 16;         /* Analog input module 3 configuration control word 1 */
} O3_L0;

typedef struct {
   unsigned az_2_voltage_config : 16;         /* Analog input module 3 configuration control word 2 */
   unsigned az_2_current_config : 16;         /* Analog input module 3 configuration control word 3 */
} O3_L1;

typedef struct {
   unsigned alt_1_voltage_config : 16;        /* Analog input module 3 configuration control word 4 */
   unsigned alt_1_current_config : 16;        /* Analog input module 3 configuration control word 5 */
} O3_L2;

typedef struct {
   unsigned alt_2_voltage_config : 16;        /* Analog input module 3 configuration control word 6 */
   unsigned alt_2_current_config : 16;        /* Analog input module 3 configuration control word 7 */
} O3_L3;

typedef struct {
   unsigned alt_position_config : 16;         /* Analog input module 4 configuration control word 0 */
   unsigned rot_1_voltage_config : 16;        /* Analog input module 4 configuration control word 1 */
} O4_L0;

typedef struct {
   unsigned rot_1_current_config : 16;        /* Analog input module 4 configuration control word 2 */
   unsigned umbilical_dist_confi : 16;        /* Analog input module 4 configuration control word 3 */
} O4_L1;

typedef struct {
   unsigned inst_lift_force_conf : 16;        /* Analog input module 4 configuration control word 4 */
   unsigned inst_lift_dist_confi : 16;        /* Analog input module 4 configuration control word 5 */
} O4_L2;

typedef struct {
   unsigned i_4_analog_6_config : 16;         /* Analog input module 4 configuration control word 6 */
   unsigned i_4_analog_7_config : 16;         /* Analog input module 4 configuration control word 7 */
} O4_L3;

typedef struct {
   unsigned cntrwht_1_pos_config : 16;        /* Analog input module 5 configuration control word 0 */
   unsigned cntrwht_2_pos_config : 16;        /* Analog input module 5 configuration control word 1 */
} O5_L0;

typedef struct {
   unsigned cntrwht_3_pos_config : 16;        /* Analog input module 5 configuration control word 2 */
   unsigned cntrwht_4_pos_config : 16;        /* Analog input module 5 configuration control word 3 */
} O5_L1;

typedef struct {
   unsigned i_5_analog_4_config : 16;         /* Analog input module 5 configuration control word 4 */
   unsigned i_5_analog_5_config : 16;         /* Analog input module 5 configuration control word 5 */
} O5_L2;

typedef struct {
   unsigned i_5_analog_6_config : 16;         /* Analog input module 5 configuration control word 6 */
   unsigned i_5_analog_7_config : 16;         /* Analog input module 5 configuration control word 7 */
} O5_L3;

typedef struct {
   unsigned s_ll_led : 1;                     /* South Lower Level e-stop LED */
   unsigned n_ll_led : 1;                     /* North Lower Level e-stop LED */
   unsigned w_rail_led : 1;                   /* West Rail e-stop LED */
   unsigned s_rail_led : 1;                   /* South Rail e-stop LED */
   unsigned n_rail_led : 1;                   /* North Rail e-stop LED */
   unsigned rot_plc_perm : 1;                 /* Rotator motion PLC permit */
   unsigned rot_mtr_ccw_perm : 1;             /* Rotator motor CCW permit */
   unsigned rot_mtr_cw_perm : 1;              /* Rotator motor CW permit */
   unsigned wind_az_plc_perm : 1;             /* Windscreen azimuth motion PLC permit */
   unsigned az_plc_perm : 1;                  /* Azimuth motion PLC permit */
   unsigned az_mtr_ccw_perm : 1;              /* Azimuth motor CCW permit */
   unsigned az_mtr_cw_perm : 1;               /* Azimuth motor CW permit */
   unsigned wind_alt_plc_perm : 1;            /* Windscreen altitude motion PLC permit */
   unsigned alt_plc_perm : 1;                 /* Altitude motion PLC permit */
   unsigned alt_mtr_dn_perm : 1;              /* Altitude motor down permit */
   unsigned alt_mtr_up_perm : 1;              /* Altitude motor up permit */
   unsigned clamp_en : 1;                     /* Clamp engage permit */
   unsigned clamp_dis : 1;                    /* Clamp disengage permit */
   unsigned t_bar_xport_perm : 1;             /* Camera T-Bar latch unlatch permit */
   unsigned t_bar_tel_perm : 1;               /* Camera T-Bar latch latch permit */
   unsigned out_11_bit_27_spare : 1;          /* Spare PLC output bit. */
   unsigned lift_pump_on : 1;                 /* Instrument lift pump on permit */
   unsigned out_11_bit_25_spare : 1;          /* Spare PLC output bit. */
   unsigned out_11_bit_24_spare : 1;          /* Spare PLC output bit. */
   unsigned deg_15_stop_ret_perm : 1;         /* 15 degree stop retracted permit */
   unsigned deg_15_stop_ext_perm : 1;         /* 15 degree stop extended permit */
   unsigned lift_solenoid_en : 1;             /* Instrument lift solenoid enable */
   unsigned s_wind_led : 1;                   /* South Windscreen e-stop LED */
   unsigned n_fork_led : 1;                   /* North Telescope fork e-stop LED */
   unsigned n_wind_led : 1;                   /* North Windscreen e-stop LED */
   unsigned w_ll_led : 1;                     /* West Lower Level e-stop LED */
   unsigned e_ll_led : 1;                     /* East Lower Level e-stop LED */
} O11_L0;

typedef struct {
   unsigned out_12_bit_15_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_14_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_13_spare : 1;          /* Spare PLC Output Bit */
   unsigned umbilical_fast : 1;               /* If set the umbilical speed is set to fast */
   unsigned lift_enable : 1;                  /* Old MCP Lift Enable bit. */
   unsigned velocity_trp_rst_out : 1;         /* Output bit to reset a velocity trip in the slip detection module. */
   unsigned velocity_select_bit : 1;          /* This bit is used to select between the 1.0 degree per sec and 3.5 degree per sec velocity limits */
   unsigned stow_pos_light : 1;               /* Stow position indicator light on permit */
   unsigned inst_chg_pos_light : 1;           /* Instrument change position indicator light on permit */
   unsigned nw_fork_led : 1;                  /* North West Fork e-stop led */
   unsigned umbilical_up_dn : 1;              /* Umbilical cord lift up / down command */
   unsigned umbilical_on_off : 1;             /* Umbilical cord lift on / off command */
   unsigned alt_brake_en : 1;                 /* Altitude brake engage permit */
   unsigned alt_brake_dis : 1;                /* Altitude brake disengage permit */
   unsigned az_brake_en : 1;                  /* Azimuth brake engage permit */
   unsigned az_brake_dis : 1;                 /* Azimuth brake disengage permit */
   unsigned out_12_bit_31_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_30_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_29_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_28_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_27_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_26_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_25_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_24_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_23_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_22_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_21_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_20_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_19_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_18_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_17_spare : 1;          /* Spare PLC Output Bit */
   unsigned out_12_bit_16_spare : 1;          /* Spare PLC Output Bit */
} O12_L0;


/*
 * Version from PLC
 */
#if defined(DATA_COLLECTION_C)
   static char plcVersion[] = "Version 21  $Name$";
#endif
/*
 * End of machine generated code
 */
struct I1 {
   I1_L0 il0;
   I1_L1 il1;
   I1_L2 il2;
   I1_L3 il3;
   I1_L4 il4;
   I1_L5 il5;
   I1_L6 il6;
   I1_L7 il7;
   I1_L8 il8;
   I1_L9 il9;
   I1_L10 il10;
   I1_L11 il11;
   I1_L12 il12;
   I1_L13 il13;
   I1_L14 il14;
   I1_L15 il15;
};
struct I2 {
	I2_L0 il0;
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

struct I6 {
   I6_L0 il0;
};

struct I7 {
   I7_L0 il0;
};

struct I8 {
   I8_L0 il0;
};

struct I9 {
   I9_L0 il0;
};

struct I10 {
   I10_L0 il0;
};

struct O1 {
   O1_L0 ol0;
   O1_L1 ol1;
   O1_L2 ol2;
   O1_L3 ol3;
   O1_L4 ol4;
   O1_L5 ol5;
   O1_L6 ol6;
   O1_L7 ol7;
   O1_L8 ol8;
   O1_L9 ol9;
   O1_L10 ol10;
   O1_L11 ol11;
   O1_L12 ol12;
   O1_L13 ol13;
   O1_L14 ol14;
   O1_L15 ol15;
};

struct O2 {
	O2_L0 ol0;
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

struct O11 {
   O11_L0 ol0;
};

struct O12 {
   O12_L0 ol0;
};

struct B3 {
   B3_L0 w0;
   B3_L1 w1;
   B3_L2 w2;
   B3_L3 w3;
   B3_L4 w4;
   B3_L5 w5;
   B3_L6 w6;
   B3_L7 w7;
   B3_L8 w8;
   B3_L9 w9;
};

struct B10 {
   B10_L0 w0;
   B10_L1 w1;
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
