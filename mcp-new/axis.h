#if !defined(AXIS_H)
#define AXIS_H

#define NAXIS			3
#define NOT_SPECIFIED		-1
#define AZIMUTH			0
#define ALTITUDE		1
#define	INSTRUMENT		2

#define	SPECTROGRAPH1		0
#define	SPECTROGRAPH2		1

#define SDSS_STOP_RATE		-24000
#define SDSS_E_STOP_RATE	-24000

/*
 * Charlie says:
 *   multiply AZ times 2000 for FNAL test setup
 * Does this refer to old ticksize of (.0035053554041*4)? RHL.
 */
#define AZ_TICK		0.0140167104063
#define ALT_TICK 	0.0140091448584
#if 0
#  define ROT_ROTARY_ENCODER 1
#  define ROT_TICK	0.0127597662202 /* rotary encoder */
#else
#  define ROT_TICK      0.0213157852666	/* optical encoder */
#endif
#define AZ_TICKS_DEG	(3600/AZ_TICK)
#define ALT_TICKS_DEG	(3600/ALT_TICK)
#define ROT_TICKS_DEG	(3600/ROT_TICK)
/*
 * Dynamically switch PID coefficients?
 */
#define SWITCH_PID_COEFFS 0
/*
 * function prototypes
 */
/*
 * Routines to deal with encoder errors; they convert between the MCP's
 * idea of the encoder position and the MEI's
 */
int get_axis_encoder_error(int axis);
void set_axis_encoder_error(int axis, int error);
int get_position_corr(int mei_axis, double *position);
int set_position_corr(int mei_axis, double position);
int get_latched_position_corr(int mei_axis, double *position);
int start_move_corr(int mei_axis, double pos, double vel, double acc);
double convert_mei_to_mcp(int axis, double pos);

const char *axis_name(int axis);
float axis_ticks_deg(int axis);

char *ms_off_cmd(char *cmd);

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
int coeffs_state_cts(int mei_axis, int cts);
#endif
double sdss_get_time(void);
double get_time(void);
void amp_reset(int mei_axis);
const char *getCvsTagname(void);
double sdss_get_time(void);
float read_clinometer(void);

int get_ffstatus(char *ffstatus_ans, int size);
int get_slitstatus(char *slitstatus_ans, int size);

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

int mcp_specdoor_clear(int spec);
int mcp_specdoor_open(int spec);
int mcp_specdoor_close(int spec);
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
extern double sec_per_tick[NAXIS];
extern double ticks_per_degree[NAXIS];
extern struct FRAME_QUEUE axis_queue[NAXIS];
extern struct AXIS_STAT axis_stat[NAXIS];
extern struct AXIS_STAT persistent_axis_stat[NAXIS];
extern int axis_alive;

#endif
