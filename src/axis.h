#if defined(AXIS_H)
#warning "AXIS_H previously defined"
#else
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
 * Which encoders are plugged into the MEI
 */
#define AZ_ENCODER 1
#define ALT_ENCODER 1
#define ROT_ENCODER 1
/*
 * Default scales for encoders
 */
#if AZ_ENCODER == 1
#  define AZ_TICK0	0.014016690731950178 /* encoder 1 */
#else
#  define AZ_TICK0	0.014025846229286947 /* encoder 2 */
#endif

#if ALT_ENCODER == 1
#  define ALT_TICK0	0.01400002855	/* encoder 1 */
#else
#  define ALT_TICK0 	0.01400914486	/* encoder 2 */
#endif

#if ROT_ENCODER == 1
#  define ROT_TICK0	0.021315787526207226 /* optical encoder */
#else
#  error There is only one optical encoder; please use it
#endif
/*
 * Dynamically switch PID coefficients?
 */
#define SWITCH_PID_COEFFS 0
/*
 * Status buffers for AXIS.STATUS/SYSTEM.STATUS
 */
#define STATUS_BUFF_SIZE 440

extern char axis_status_buff[NAXIS][STATUS_BUFF_SIZE];
extern char system_status_buff[STATUS_BUFF_SIZE];
/*
 * function prototypes
 */
/*
 * Routines to deal with encoder errors; they convert between the MCP's
 * idea of the encoder position and the MEI's
 */
int get_axis_encoder_error(int axis, int encoder);
void set_axis_encoder_error(int axis, int encoder, long error, int write_log);
int get_position_corr(int mei_axis, double *position);
int set_position_corr(int mei_axis, double position);
int get_latched_position_corr(int mei_axis, double *position);
int start_move_corr(int mei_axis, double pos, double vel, double acc);
double convert_mei_to_mcp(int mei_axis, double pos);
#if defined(__IDSP_H)
   int dsp_set_last_command_corr(PDSP pdsp, int16 mei_axis, double final);
   int frame_m_xvajt_corr(PFRAME frame, char *cmd_str, int mei_axis,
			  double x, double v, double a, double j, double t,
			  int new_frame);
#endif

void set_axis_scale(int axis, double ticksize);
const char *axis_name(int axis);
const char *axis_abbrev(int axis);
float axis_ticks_deg(int axis);
void enable_pvt(int axis);
void tm_TCC(int axis);
int mcp_drift(int uid, unsigned long cid, int axis, double *arcdeg, double *veldeg, double *t);
int mcp_move(int uid, unsigned long cid, int axis, double *params, int nparam);
int mcp_plus_move(int axis, double *params, int nparam);

char *ms_off_cmd(int uid, unsigned long cid, char *cmd);

#if defined(COEFFICIENTS)
   struct SW_COEFFS {
      short coeffs[COEFFICIENTS];
      double uplimit_deg;
      double dnlimit_deg;
      long uplimit_cts;
      long dnlimit_cts;
   };
#endif

#define PID_COEFFS_TRACKING     0
#define PID_COEFFS_SLEWING      1

void select_pid_block(int uid,           /* user id */
                      unsigned long cid, /* command id */
                      int axis,          /* desired axis */
                      int block); /* desired block of coefficients */

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
int mcp_set_brake(int uid, unsigned long cid, int axis);
int mcp_unset_brake(int uid, unsigned long cid, int axis);
int mcp_hold(int axis);
int mcp_amp_reset(int axis);
int mcp_cw_abort(int uid, unsigned long cid);
int mcp_set_cw(int uid, unsigned long cid, int inst, int cw, int cwpos, char **ans);
int mcp_stop_axis(int axis);
int mcp_move_va(int axis, long pos, long vel, long acc);

int mcp_specdoor_clear(int uid, unsigned long cid, int spec);
int mcp_specdoor_open(int uid, unsigned long cid, int spec);
int mcp_specdoor_close(int uid, unsigned long cid, int spec);
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
