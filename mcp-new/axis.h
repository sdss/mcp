#if !defined(AXIS_H)
#define AXIS_H

#define ALLOW_AMP_UPDATE 1		/* allow *_amp_ok() to update? */

#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif

#define NAXIS			3
#define NOINST			-1
#define AZIMUTH			0
#define ALTITUDE		1
#define	INSTRUMENT		2

#define	SPECTROGRAPH1		0
#define	SPECTROGRAPH2		1

#define SDSS_STOP_RATE		-24000
#define SDSS_E_STOP_RATE	-24000

/*
 * This one is repeated in data_collection.c --- they MUST agree
 */
#define SHARE_MEMORY	0x02800000

/*
 * Charlie says:
 *   multiply AZ times 2000 for FNAL test setup
 * Does this refer to old ticksize of (.0035053554041*4)? RHL.
 * Old AZ_TICK 0.01401671040630 20-Sep-00 DL
 * New AZ_TICK 0.01401678528201 12-Dec-00 DL
 * New AZ_TICK 0.01401693078832 11-Feb-01 EN-JK
 * New AZ_TICK 0.01401666759398 10-May-01 EN-AK
 * New AZ_TICK 0.01401663645595 09-Jun-01 SS-EN
 * New ROT_TICK 0.021315787643528178 09-Jun-01 SS-EN
 */
#define AZ_TICK	0.014016669435746228
#if 1
#  define ALT_TICK	0.01400002855	/* encoder 1 */
#else
#  define ALT_TICK 	0.01400914486	/* encoder 2 */
#endif

#if 0
#  define ROT_ROTARY_ENCODER 1
#  define ROT_TICK	0.0127597662202 /* rotary encoder */
#else
#  define ROT_TICK      0.021315788702945707	/* optical encoder */
#endif
#define AZ_TICKS_DEG	(3600/AZ_TICK)
#define ALT_TICKS_DEG	(3600/ALT_TICK)
#define ROT_TICKS_DEG	(3600/ROT_TICK)
/*
 * Dynamically switch PID coefficients?
 */
#define SWITCH_PID_COEFFS 0
/*
 * Status buffers for AXIS.STATUS/SYSTEM.STATUS
 */
#define STATUS_BUFF_SIZE 200

extern char axis_status_buff[NAXIS][STATUS_BUFF_SIZE];
extern char system_status_buff[STATUS_BUFF_SIZE];
/*
 * function prototypes
 */
/*
 * Routines to deal with encoder errors; they convert between the MCP's
 * idea of the encoder position and the MEI's
 */
int get_axis_encoder_error(int axis);
void set_axis_encoder_error(int axis, int error, int write_log);
int get_position_corr(int mei_axis, double *position);
int set_position_corr(int mei_axis, double position);
int get_latched_position_corr(int mei_axis, double *position);
int start_move_corr(int mei_axis, double pos, double vel, double acc);
double convert_mei_to_mcp(int axis, double pos);
#if defined(__IDSP_H)
   int dsp_set_last_command_corr(PDSP pdsp, int16 mei_axis, double final);
   int frame_m_xvajt_corr(PFRAME frame, char *cmd_str, int mei_axis,
			  double x, double v, double a, double j, double t,
			  int new_frame);
#endif

const char *axis_name(int axis);
float axis_ticks_deg(int axis);
void enable_pvt(int axis);
void tm_TCC(int axis);
int mcp_drift(int axis, double *arcdeg, double *veldeg, double *t);
int mcp_move(int axis, double *params, int nparam);
int mcp_plus_move(int axis, double *params, int nparam);

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
float convert_clinometer(float val);
float read_clinometer(void);

int get_ffstatus(char *ffstatus_ans, int size);
int get_slitstatus(char *slitstatus_ans, int size);
void set_status(int axis, char *buff, int size);

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
extern SEM_ID semMEIDC;
extern SEM_ID semMEIUPD;
extern SEM_ID semSLC;
extern SEM_ID semSDSSDC;
extern SEM_ID semStatusCmd;

/*
 * extern declarations for globals
 */
extern double sec_per_tick[NAXIS];
extern double ticks_per_degree[NAXIS];
extern struct FRAME_QUEUE axis_queue[NAXIS];
extern struct AXIS_STAT axis_stat[NAXIS][2];
extern double max_velocity[NAXIS];
extern double max_acceleration[NAXIS];
extern double max_velocity_requested[NAXIS];
extern double max_acceleration_requested[NAXIS];
extern int axis_alive;

#endif
