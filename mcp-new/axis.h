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

#define	SPECTOGRAPH1		0
#define	SPECTOGRAPH2		1

#define SDSS_STOP_RATE		-24000
#define SDSS_E_STOP_RATE	-24000
/* .01401665166 bill's on 22-oct-99 */
/* mulitply AZ times 2000 for FNAL test setupd */
#define AZ_TICK		(.0140167104063)   /* (.0035053554041*4)*/
#define ALT_TICK 	(.0140091448584)   /* (.0035022862146*4)*/
#if 0
#  define ROT_ROTARY_ENCODER 1
#  define ROT_TICK	(.0127597662202) /* rotary encoder */
#else
#  define ROT_TICK	(.0106578926333*2) /* optical encoder */
#endif
#define AZ_TICKS_DEG	(3600/AZ_TICK)
#define ALT_TICKS_DEG	(3600/ALT_TICK)
#define ROT_TICKS_DEG	(3600/ROT_TICK)
/*
 * Dynamically switch PID coefficients?
 */
#define SWITCH_PID_COEFFS 0

/* function prototypes */
char *reboot_cmd(char *cmd);
char *axis_status_cmd(char *cmd);
char *correct_cmd(char *cmd);
char *drift_cmd(char *cmd);
char *id_cmd(char *cmd);
char *version_cmd(char *cmd);
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
char *system_status_cmd(char *cmd);
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
char *cwinst_cmd(char *cmd);
char *cwabort_cmd(char *cmd);
char *cwstatus_cmd(char *cmd);
char *sp1_cmd(char *cmd);
char *sp2_cmd(char *cmd);
char *slitstatus_cmd(char *cmd);
char *ffsopen_cmd(char *cmd);
char *ffsclose_cmd(char *cmd);
char *fflon_cmd(char *cmd);
char *ffloff_cmd(char *cmd);
char *neon_cmd(char *cmd);
char *neoff_cmd(char *cmd);
char *hgcdon_cmd(char *cmd);
char *hgcdoff_cmd(char *cmd);
char *ffstatus_cmd(char *cmd);
char *abstatus_cmd(char *cmd);

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
#if defined(COEFFICIENTS)
   struct SW_COEFFS {
      short coeffs[COEFFICIENTS];
      double uplimit_deg;
      double dnlimit_deg;
      long uplimit_cts;
      long dnlimit_cts;
   };
#endif

void set_rot_state(int state);
#if SWITCH_PID_COEFFS
void set_rot_coeffs(int state, int index, short val);
void print_rot_coeffs(void);
void set_rot_uplimit(int state, int val);
void set_rot_dnlimit(int state, int val);
int coeffs_state_cts(int axis, int cts);
#endif
double sdss_get_time(void);
double get_time(void);
void amp_reset(int axis);
const char *getCvsTagname(void);
void restore_pos(void);
extern double sdss_get_time(void);

int mcp_set_monitor(int axis, int on_off);
int mcp_set_pos(int axis, double pos);
int mcp_set_vel(int axis, double vel);
int mcp_set_fiducial(int axis);
int mcp_set_brake(int axis);
int mcp_unset_brake(int axis);
int mcp_hold(int axis);
int mcp_amp_reset(int axis);
int mcp_cw_abort(void);
int mcp_set_cw(int inst, int cw, int cwpos, const char **ans);
int mcp_stop_axis(int axis);
int mcp_move_va(int axis, long pos, long vel, long acc);
int mcp_set_tbars(int val);

int mcp_slit_clear(int spec);
int mcp_slit_open(int spec);
int mcp_slit_close(int spec);
int mcp_slithead_latch_open(int spec);
int mcp_slithead_latch_close(int spec);

/*
 * Semaphores
 */
extern SEM_ID semMEI;
extern SEM_ID semMEIUPD;
extern SEM_ID semSLC;

/*
 * extern declarations for globals
 */
extern int axis_select;			/* 0=AZ,1=ALT,2=ROT -1=ERROR  */
extern double sec_per_tick[3];
extern double ticks_per_degree[3];
extern long fiducial_position[3];
extern struct FIDUCIARY fiducial[3];
extern int fiducialidx[3];
extern struct FIDUCIALS az_fiducial[48];
extern struct FIDUCIALS alt_fiducial[7];
extern struct FIDUCIALS rot_fiducial[156];
extern struct FRAME_QUEUE axis_queue[3];
extern struct AXIS_STAT axis_stat[3];
extern struct AXIS_STAT persistent_axis_stat[3];
extern int axis_alive;
extern int altclino_off;
extern float altclino_sf;

#define __AXIS_H__             /* do only once */

#endif	/* End __AXIS_H__ */
