#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	axis.h							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/

#ifndef __AXIS_H__

#define NAXIS			3
#define NOT_SPECIFIED		-1
#define AZIMUTH			0
#define ALTITUDE		1
#define	INSTRUMENT		2

#define	ENCLOSURE		3

#define	SPECTOGRAPH1		0
#define	SPECTOGRAPH2		1

#define SDSS_STOP_RATE		200000.0
#define SDSS_E_STOP_RATE	200000.0
#define AZ_TICK		(.0140167104063)   /* (.0035053554041*4)*/
#define ALT_TICK 	(.0140091448584)   /* (.0035022862146*4)*/
#ifdef ROT_ROTARY_ENCODER
#define ROT_ROTARY_ENCODER	1
#define ROT_TICK	(.0127597662202) /* rotary encoder */
#else
#define ROT_TICK	(.0106578926333*2) /* optical encoder */
#endif
#define OPT_TICK	(.0106578926333*2) /* optical encoder */
/*#define ROT_TICK	(.0106578926333*4) *//* optical encoder */
/*#define ROT_TICK	(.0031823575754*4)*/
#define ROT4_TICK 	(.0031823575754*4)
#define ROT5_TICK       (.0031823575754*4*4.607)
#define ROT2_TICK_SF	4.607
#define AZ_TICKS_DEG	(3600/AZ_TICK)
#define ALT_TICKS_DEG	(3600/ALT_TICK)
#define ROT_TICKS_DEG	(3600/ROT_TICK)

/* function prototypes */
char *reboot_cmd(char *cmd);
char *correct_cmd(char *cmd);
char *drift_cmd(char *cmd);
char *encl_cmd(char *cmd);
char *id_cmd(char *cmd);
char *init_cmd(char *cmd);
char *maxacc_cmd(char *cmd);
char *maxvel_cmd(char *cmd);
char *mc_dump_cmd(char *cmd);
char *mc_maxacc_cmd(char *cmd);
char *mc_maxpos_cmd(char *cmd);
char *mc_maxvel_cmd(char *cmd);
char *mc_minpos_cmd(char *cmd);
char *move_cmd(char *cmd);
char *plus_move_cmd(char *cmd);
char *mr_dump_cmd(char *cmd);
char *ms_dump_cmd(char *cmd);
char *ms_map_dump_cmd(char *cmd);
char *ms_map_load_cmd(char *cmd);
char *ms_off_cmd(char *cmd);
char *ms_on_cmd(char *cmd);
char *ms_pos_dump_cmd(char *cmd);
char *remap_cmd(char *cmd);
char *rot_cmd(char *cmd);
char *set_limits_cmd(char *cmd);
char *set_position_cmd(char *cmd);
char *set_time_cmd(char *cmd);
char *stats_cmd(char *cmd);
char *status_cmd(char *cmd);
char *status_long_cmd(char *cmd);
char *tel1_cmd(char *cmd);
char *tel2_cmd(char *cmd);
char *ticklost_cmd(char *cmd);
char *time_cmd(char *cmd);
char *dummy_cmd(char *cmd);
char *brakeon_cmd(char *cmd);
char *brakeoff_cmd(char *cmd);
char *clampon_cmd(char *cmd);
char *clampoff_cmd(char *cmd);
char *cwmov_cmd(char *cmd);
char *cwpos_cmd(char *cmd);
char *cwinst_cmd(char *cmd);
char *cwabort_cmd(char *cmd);
char *cwstatus_cmd(char *cmd);
char *sp1_cmd(char *cmd);
char *sp2_cmd(char *cmd);
char *slitopen_cmd(char *cmd);
char *slitclose_cmd(char *cmd);
char *cartlatch_cmd(char *cmd);
char *cartunlatch_cmd(char *cmd);
char *slitstatus_cmd(char *cmd);
char *abstatus_cmd(char *cmd);

#define CONTROL_BUFFER_EMPTY	00000001L
#define MOVE_TMO		00000002L
#define MIN_POSITION		00000004L
#define MAX_POSITION		00000008L
#define VELOCITY_LIMITED	00000010L
#define ACCELERATION_LIMITED	00000020L
#define MIN_LIMIT		00000040L
#define MAX_LIMIT		00000080L

#define LOW_RESOLUTION_PROBLEM	00000100L
#define HIGH_RESOLUTION_PROBLEM	00000200L
#define AD_CONVERTER_PROBLEM	00000400L
#define SERVO_AMP_STOP_INPUT	00000800L
#define UNUSED_00001000		00001000L
#define TEST_00002000		00002000L
#define TEST_00004000		00004000L
#define TEST_00008000		00008000L
#define LOST_1HZ_CLOCK		00010000L
#define LIMIT_SWITCH_PROBLEM	00020000L
#define CURRENT_LIMIT_MOTOR2	00040000L
#define CURRENT_LIMIT_MOTIR100080000L
#define POWER_LOSS_SERVO_AMP2	00100000L
#define POWER_LOSS_SERVO_AMP1	00200000L
#define MOTOR2_PROBLEM		00400000L
#define MOTOR1_PROBLEM		00800000L

#define MAGNESWITCH_ERR_LARGE	01000000L
#define MAGNESWITCH_BEHIND	02000000L
#define MAGNESWITCH_AHEAD	04000000L
#define MAGNESWITCH_NEG_VELOCITY	08000000L
#define MAGNESWITCH_NEG_CORRECTION	10000000L
#define UNUSED_20000000		20000000L
#define UNUSED_40000000		40000000L
#define ALWAYS_ZERO		80000000L

#define MEI_VECTOR		27
#define MEI_IRQ			5
#define MEI_TYPE		1

#define DIO316_VECTOR		27
#define DIO316_IRQ		5
#define DIO316_TYPE		2

#define DID48_VECTOR		29
#define DID48_IRQ		5
#define DID48_TYPE		3

#define NIST_INT		0x1
#define NBS_INT			0x1
#define AZIMUTH_INT		0x2
#define ALTITUDE_INT		0x4
#define INSTRUMENT_INT		0x8
	
struct FIDUCIARY {
	int markvalid;
	long mark;
	int index;
};	
struct FIDUCIALS {
	int markvalid;
	long mark;
	long last;
	long poserr;
	long err;
};	
struct SW_COEFFS {
        short coeffs[COEFFICIENTS];
        double uplimit_deg;
        double dnlimit_deg;
        long uplimit_cts;
        long dnlimit_cts;
};
void set_rot_coeffs (int state, int index, short val);
void print_rot_coeffs ();
void set_rot_uplimit (int state, int val);
void set_rot_dnlimit (int state, int val);
void set_rot_state (int state);
int coeffs_state_deg (int axis, double degs);
int coeffs_state_cts (int axis, int cts);

#define __AXIS_H__             /* do only once */

#endif	/* End __AXIS_H__ */
