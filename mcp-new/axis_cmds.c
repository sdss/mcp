#include "copyright.h"
/**************************************************************************
**
**	Action routines for commands from the TCC which involve setting up
**	queues for each axis to execute motion.
**	tm_TCC is spawned for each axis to excute motion
*/
#include <assert.h>
#include <string.h>
#include <ctype.h>
#include "vxWorks.h"                            
#include "wdLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "stdio.h"
#include "tickLib.h"
#include "inetLib.h"
#include "taskLib.h"
#include "rebootLib.h"
#include "sysLib.h"
#include "ioLib.h"
#include "errno.h"
#include "usrLib.h"
#include "in.h"
#include "timers.h"
#include "time.h"
#include "frame.h"
#include "iv.h"
#include "intLib.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "gendefs.h"
#include "mv162IndPackInit.h"
#include "logLib.h"
#include "data_collection.h"
#include "io.h"
#include "math.h"
#include "tm.h"
#include "axis.h"
#include "cw.h"
#include "serial.h"
#include "mcpUtils.h"
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "abdh.h"
#include "mcpTimers.h"
#include "mcpFiducials.h"
#include "cmd.h"

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
#define	DSP_IO_BASE		(0x300)			/* in A16/D16 space. */

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
struct DIAG_Q {
   double p;
   double v;
   double a;
   double ji;
   double tim;
};

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
SEM_ID semMEI = NULL;
SEM_ID semSLC = NULL;

MSG_Q_ID msgMoveCW = NULL;		/* control the moveCW task */
MSG_Q_ID msgMoveCWAbort = NULL;		/*  "   "   "   "  "   "   */
SEM_ID semMoveCWBusy = NULL;		/*  "   "   "   "  "   "   */

int axis_select = -1;			/* 0=AZ,1=ALT,2=ROT -1=ERROR  */
int MEI_interrupt = FALSE;
int sdss_was_init = FALSE;
struct FRAME_QUEUE axis_queue[3] = {
   {0, NULL, NULL},
   {0, NULL, NULL},
   {0, NULL, NULL}
};
/*
 * Maximum allowed velocity/acceleration and arrays to keep account of
 * the maximum |values| actually requested
 */
double max_velocity[NAXIS] = {2.25, 1.75, 2.25};
double max_acceleration[NAXIS] = {4.0, 4.0, 6.0};
double max_velocity_requested[NAXIS] = {0, 0, 0};
double max_acceleration_requested[NAXIS]={0, 0, 0};

int CALC_verbose=FALSE;
int CALCOFF_verbose=FALSE;
int CALCADDOFF_verbose=FALSE;
int CALCFINAL_verbose=FALSE;
int FRAME_verbose=FALSE;
static struct DIAG_Q *diagq = NULL;
static int diagq_siz,diagq_i;
int DIAGQ_verbose=FALSE;
#define FRMHZ		20	/* 20 Hz Frames to MEI */
#define FLTFRMHZ	20.
#define MAX_CALC 20
static double tim[3][MAX_CALC],p[3][MAX_CALC],v[3][MAX_CALC],a[3][MAX_CALC],ji[3][MAX_CALC];
static double timoff[3][MAX_CALC],poff[3][MAX_CALC],voff[3][MAX_CALC],
	aoff[3][MAX_CALC],jioff[3][MAX_CALC];
double time_off[3]={0.0,0.0,0.0};
double stop_position[3]={0.0,0.0,0.0};
double drift_velocity[3]={0.0,0.0,0.0};
int frame_break[3]={FALSE,FALSE,FALSE};
int drift_break[3]={FALSE,FALSE,FALSE};
int DRIFT_verbose=FALSE;
int drift_modify_enable=FALSE;
#define OFF_MAX	6		/* upto 8 offsets at one time */
struct FRAME offset[3][OFF_MAX+2][2];
struct FRAME *offset_queue_end[3][OFF_MAX+2]={
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL},
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL},
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL}
				};
int offset_idx[3][OFF_MAX+2]={
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0}
};
double sec_per_tick[3]={AZ_TICK, ALT_TICK, ROT_TICK};
double ticks_per_degree[3]={AZ_TICKS_DEG, ALT_TICKS_DEG, ROT_TICKS_DEG};

/*------------*/
/* Prototypes */
/*------------*/

static int tm_frames_to_execute(int axis);

/*****************************************************************************/
/*
 * return an axis's name
 */
const char *
axis_name(int axis)
{
   switch (axis) {
    case AZIMUTH:    return("azimuth");
    case ALTITUDE:   return("altitude");
    case INSTRUMENT: return("rotator");
    default:
      return("(unknown axis)");
   }
}

float
axis_ticks_deg(int axis)
{
   switch (axis) {
    case AZIMUTH:    return(AZ_TICKS_DEG);
    case ALTITUDE:   return(ALT_TICKS_DEG);
    case INSTRUMENT: return(ROT_TICKS_DEG);
    default:
      fprintf(stderr,"axis_ticks_deg: unknown axis %d\n", axis);
      return(1);
   }
}

/*****************************************************************************/
/*
 * The encoders are assumed to be wrong by axis_encoder_error[] which
 * is indexed by 2*axis
 *
 * Two routines to get/set this error:
 */
/* static */ int axis_encoder_error[2*NAXIS] = { 0, 0, 0, 0, 0, 0 };

int 
get_axis_encoder_error(int axis)	/* the axis in question */
{
   assert(axis == AZIMUTH || axis == ALTITUDE || axis == INSTRUMENT);

   return(axis_encoder_error[2*axis]);
}

void
set_axis_encoder_error(int axis,	/* the axis in question */
		       int error)	/* value of error */
{
   assert(axis == AZIMUTH || axis == ALTITUDE || axis == INSTRUMENT);

   write_fiducial_log("SET_FIDUCIAL_ERROR", axis, 0, 0, 0, 0, error);

   axis_encoder_error[2*axis] += error;
   axis_encoder_error[2*axis + 1] = axis_encoder_error[2*axis];
}

/*
 * Seven routines to intermediate between the MCP and the MEIs.  These
 * routines are responsible for correcting the measured encoder readings
 * by the known errors in those readings, as determined by crossing the
 * fiducials
 *
 * ALL COMMUNICATION OF POSITIONS TO/FROM THE MEIs MUST USE THESE ROUTINES!
 */
double
convert_mei_to_mcp(int axis,
		   double pos)
{
   return(pos + axis_encoder_error[2*axis]);
}

int
get_position_corr(int mei_axis,		/* 2*(desired axis) */
		  double *position)	/* position to get */
{
   int ret = get_position(mei_axis, position); /* read encoder */
   *position += axis_encoder_error[mei_axis]; /* undo correction */

   return(ret);
}

int
get_latched_position_corr(int mei_axis,	/* 2*(desired axis) */
			  double *position) /* position of latch */
{
   int ret = get_latched_position(mei_axis, position);
   *position += axis_encoder_error[mei_axis];
   
   return(ret);
}

int
set_position_corr(int mei_axis,		/* 2*(desired axis) */
		  double position)	/* position to set */
{
   position -= axis_encoder_error[mei_axis]; /* apply correction */
   return(set_position(mei_axis, position)); /* set position */
}

int
start_move_corr(int mei_axis,
		double pos,
		double vel,
		double acc)
{
   pos -= axis_encoder_error[mei_axis]; /* apply correction */
   return(start_move(mei_axis, pos, vel, acc)); /* do move */
}


static int
frame_m_xvajt_corr(PFRAME frame,	/* frame for MEI */
		   char *cmd_str,	/* what we want the DSP to do */
		   int mei_axis,	/* 2*axis */
		   double x,		/* desired position */
		   double v,		/* desired velocity */
		   double a,		/* desired acceleration */
		   double j,		/* desired jerk */
		   double t,		/* at time t */
		   long flags,		/* describe what we have */
		   int new_frame)	/* new frame? */
{
   x -= axis_encoder_error[mei_axis];	/* apply correction */
   
   return(frame_m(frame, cmd_str, mei_axis, x, v, a, j, t, flags, new_frame));
}

static int
dsp_set_last_command_corr(PDSP pdsp,
			  int16 mei_axis, /* 2*axis */
			  double final)
{
   final -= axis_encoder_error[mei_axis]; /* apply correction */
   
   return(dsp_set_last_command(pdsp, mei_axis, final));
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: drift_cmd
**
** DESCRIPTION:
**      DRIFT -> Continue at the current velocity until new commands are issued
**
** RETURN VALUES:
**      "pos vel time"
**
** CALLS TO:
**	sdss_get_time
**
** GLOBALS REFERENCED:
**	axis_select
**	drift_velocity
**	drift_break
**
**=========================================================================
*/
static char drift_ans[] =
   "360.00000 0.500000 5040.00000                                ";

char *
drift_cmd(char *cmd)
{
   double position;
   double arcdeg, veldeg;
   float time;
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(axis_select, 0) < 0) {
      TRACE(0, "drift_cmd: failed to set MS.OFF", 0, 0);
      return("ERR: failed to set MS.OFF");
   }
/*
 * Get the MEI semaphore and proceed
 */
   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "drift_cmd: failed to get semMEI: %s (%d)",
	    strerror(errno), errno);
      return "ERR: semMEI";
   }
   
   get_velocity(axis_select<<1,&drift_velocity[axis_select]);
   semGive (semMEI);
   drift_break[axis_select]=TRUE;

   taskDelay(3);
   if(semTake(semMEI, 60) == ERROR) {
      TRACE(0, "drift_cmd: failed to retake semMEI: %s (%d)",
	    strerror(errno), errno);
      return "ERR: semMEI";
   }

   taskLock();				/* enforce coincidental data */
   get_position_corr(2*axis_select, &position);
   time = sdss_get_time();
   taskUnlock();
   semGive(semMEI);

   if(time < 0) {
      TRACE(0, "drift_cmd: bad time %g", time, 0);
      return "ERR: BAD TIME";
   }

   veldeg = sec_per_tick[axis_select]*drift_velocity[axis_select]/3600.;
   arcdeg = sec_per_tick[axis_select]*position/3600.;
   sprintf(drift_ans, "%f %f %f", arcdeg, veldeg, time);

   TRACE(3, "DRIFT %s: %s", axis_name(axis_select), drift_ans);
   printf("at end DRIFT %s: %s\n", axis_name(axis_select), drift_ans);

   return(drift_ans);
}

/*=========================================================================
**=========================================================================
**
**      ID -> MEI firmware
**
** RETURN VALUES:
**      "axis date firmware"
**=========================================================================
*/
char *
id_cmd(char *cmd)
{
   static char id_ans[] = "0 None Specified MMM DD 19YY\r\nDSP Firmware=Vxxx.xx Rxx Sx, Option=xxxx Axes=x";

   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

  sprintf(id_ans,"%d %s %s\r\n%s\r\nDSP Firmware: V%f R%d S%d, Option=%d, Axes=%d",
		axis_select,axis_name(axis_select),__DATE__, getCvsTagname(),
		dsp_version()/1600.,dsp_version()&0xF,(dsp_option()>>12)&0x7,
		dsp_option()&0xFFF,dsp_axes());
   
  return id_ans;
}

/*=========================================================================
**=========================================================================
**
**      INIT -> Init the axis
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**	tm_axis_state
**	tm_sem_controller_idle
**	tm_reset_integrator
**	amp_reset
**      tm_sp_az_brake_off
**      tm_sp_alt_brake_off
**	tm_sem_controller_run
**
** GLOBALS REFERENCED:
**	axis_select
**	sdssdc
**	semMEI
**	axis_queue
**	drift_break
**	frame_break
**
**=========================================================================
*/
char *
init_cmd(char *cmd)
{
   int i;
   int state;
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(axis_select, 0) < 0) {
      TRACE(0, "init_cmd: failed to set MS.OFF", 0, 0);
      return("ERR: failed to set MS.OFF");
   }
/*
 * I (Charlie) don't really agree with this, JEG directed...
 * I think the axis should remain in closed loop if in close loop
 */
   state = tm_axis_state(2*axis_select);

   if(state > 2) {			/* normal...NOEVENT,running, or
					   NEW_FRAME */
      printf("INIT axis %d: not running, state=0x%x\n", 2*axis_select, state);
      tm_sem_controller_idle(2*axis_select);
   }
   
   tm_reset_integrator(2*axis_select);
   drift_break[axis_select] = FALSE;
   frame_break[axis_select] = FALSE;
   
   if(semTake(semSLCDC, WAIT_FOREVER) == ERROR) {
      TRACE(0, "couldn't take semSLCDC semaphore.", 0, 0);
   } else {
      memset(&axis_stat[axis_select], '\0', sizeof(struct AXIS_STAT *));
      semGive(semSLCDC);
   }

   switch(axis_select) {
    case AZIMUTH:
      amp_reset(2*axis_select);		/* two amplifiers, */
      amp_reset(2*axis_select + 1);	/* param is index to bit field */
      if(sdssdc.status.i9.il0.az_brake_en_stat) {
	 tm_sp_az_brake_off();
      }
      break;
    case ALTITUDE:
      amp_reset(2*axis_select);		/* two amplifiers */
      amp_reset(2*axis_select + 1);	/* param is index to bit field */
      if(sdssdc.status.i9.il0.alt_brake_en_stat) {
	 tm_sp_alt_brake_off();           
      }
      break;
    case INSTRUMENT:
      amp_reset(2*axis_select);		/* one amplifier */
      break;				/* param is index to bit field */
   }
/*
 * reinitialize the queue of pvts to none
 */
   axis_queue[axis_select].active = axis_queue[axis_select].end;
   axis_queue[axis_select].active = NULL;
/*
 * zero the velocity
 */
   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "Failed to take semMEI to zero velocity for axis %s",
	    axis_name(axis_select), 0);
   } else {
      taskLock();
      set_stop(2*axis_select);
      while(!motion_done(2*axis_select)) ;
      clear_status(2*axis_select);
      taskUnlock();

      if(dsp_error != DSP_OK) {
	 TRACE(0, "failed to stop %s : %d", axis_name(axis_select), dsp_error);
      }

      semGive(semMEI);
   }
/*
 * tm_axis_state retries as well...so this is really redundant, but then the
 * brakes don't come off really quickly so this will assure closed loop for
 * at least sometime before continuing
 */
   for(i = 0; i < 4; i++) {
      tm_sem_controller_run(2*axis_select);

      if(tm_axis_state(2*axis_select) <= NEW_FRAME) { /* axis is OK */
	 break;
      }

      taskDelay(20);
   }
/*
 * If there's a known positional error on an axis take this opportunity to
 * zero it by setting the encoder's position
 */
   if(semTake(semLatch, 60) == ERROR) {
      TRACE(0, "ERR: init cannot take semLatch", 0, 0);
   } else {
      int correction;			/* how much to correct encoder pos */
      
      for(i = 0; i < NAXIS; i++) {
	 if(fiducial[i].max_correction == 0) {
	    continue;
	 }

	 correction = get_axis_encoder_error(i);
	 if(correction > 0) {
	    TRACE(3 ,"Adjusting position of %s by %d", axis_name(i), correction);
	    
	    if(abs(correction) >= fiducial[i].max_correction) {
	       TRACE(0, "    correction %ld is too large (max %ld)",
		     correction, fiducial[i].max_correction);
	       continue;
	    }
	    
	    if(tm_adjust_position(i, correction) < 0) {
	       TRACE(0 ,"Failed to adjust position for axis %s",
		     axis_name(i), 0);
	    }
	 }
      }

      semGive(semLatch);
   }
   
   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: maxacc_cmd
**
** DESCRIPTION:
**      MAXACC acc -> Set maximum acceleration for axis.
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	max_acceleration
**	axis_select
**
**=========================================================================
*/
char *
maxacc_cmd(char *cmd)
{
   printf (" MAXACC command fired\r\n");
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   if(sscanf(cmd,"%12lf", &max_acceleration[axis_select]) != 1) {
      printf("ERR: no acceleration specified\n");
   }

   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: maxvel_cmd
**
** DESCRIPTION:
**      MAXVEL vel -> Maximum velocity for axis.
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	max_velocity
**	axis_select
**
**=========================================================================
*/
char *
maxvel_cmd(char *cmd)
{
   printf (" MAXVEL command fired\r\n");

   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   if(sscanf (cmd,"%12lf",&max_velocity[axis_select]) != 1) {
      printf("ERR: no velocity specified\n");
   }

   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mc_dump_cmd
**
** DESCRIPTION:
**      MC.DUMP -> Dump the time-tagged MOVE commands
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *mc_dump_cmd(char *cmd)
{
  printf (" MC.DUMP command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mc_maxacc_cmd
**
** DESCRIPTION:
**      MC.MAX.ACC -> Display the maximum acceleration.
**
** RETURN VALUES:
**      "F@ F. acc" or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	max_acceleration
**	axis_select
**
**=========================================================================
*/
static char max_ans[] = "F@ F.             ";

char *
mc_maxacc_cmd(char *cmd)
{
   printf (" MC.MAX.ACC command fired\n");
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   sprintf(max_ans,"F@ F. %12f", max_acceleration[axis_select]);

   return max_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mc_maxvel_cmd
**
** DESCRIPTION:
**      MC.MAX.VEL -> Display the maximum velocity.
**
** RETURN VALUES:
**      "F@ F. vel" or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	max_velocity
**	axis_select
**
**=========================================================================
*/
char *
mc_maxvel_cmd(char *cmd)		/* NOTUSED */
{
   printf (" MC.MAX.VEL command fired\n");
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   sprintf(max_ans,"F@ F. %12f", max_velocity[axis_select]);
   
   return max_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: move_cmd
**
** DESCRIPTION:
**      MOVE -> Move to the current postion or very close to it...implementation
**	dependent.  I've done it both ways...operators don't like to hear
**	it reposition itself...so it just stops as quick as possible.
**      MOVE pos -> Move to the specified position.
**      MOVE pos vel -> Move to the position with this velocity
**      MOVE pos vel time -> Move to the position with this velocity at the
**	time specified...the common pvt triplet.
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**	sdss_delta_time
**	sdss_get_time
**      tm_get_position
**	malloc
**	free
**
** GLOBALS REFERENCED:
**	axis_select
**	drift_break
**	frame_break
**	axis_queue
**	sdssdc
**
**=========================================================================
*/
char *
move_cmd(char *cmd)
{
   double position,velocity,pos;
   struct FRAME *frame,*nxtque;
   struct FRAME_QUEUE *queue;
   int i;
   int cnt;
   double dt;
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(axis_select, 0) < 0) {
      TRACE(0, "move_cmd: failed to set MS.OFF", 0, 0);
      return("ERR: failed to set MS.OFF");
   }
/*
 * Proceed with the MOVE
 */
   if(sdss_get_time() < 0) {
      TRACE(0, "move_cmd(): bad time", 0, 0);
      return "ERR: BAD TIME";
   }
   
   queue = &axis_queue[axis_select];
   
   frame = (struct FRAME *)malloc(sizeof(struct FRAME));
   if(frame == NULL) {
      TRACE(0, "Cannot allocate frame (%d)", errno, 0);
      return "ERR: OUT OF MEMORY";
   }
   
   cnt = sscanf(cmd, "%12lf %12lf %12lf",
		&position, &velocity, &frame->end_time);

#if 0					/* XXX */
   TRACE(3, "MOVE %s", cmd, 0);
   TRACE(3, "     Axis = %s cnt = %d", axis_name(axis_select), cnt);
   printf("%s MOVE %s %d\n", axis_name(axis_select), cmd, cnt);
#endif

   switch (cnt) {
    case -1:
    case 0:
      tm_get_position(2*axis_select, &stop_position[axis_select]);
      frame_break[axis_select] = TRUE;

      sdssdc.tccmove[axis_select].position = 0;
      sdssdc.tccmove[axis_select].velocity = 0;
      sdssdc.tccmove[axis_select].time = 0;

      if(frame != NULL) free(frame);
      
      return "";
    case 1:
      tm_get_position(2*axis_select,&pos);
      velocity = (double)0.10;
      frame->end_time =
	fmod((double)(sdss_get_time() +
		      abs((pos/ticks_per_degree[axis_select] -
			   position)/velocity)), (double)86400.0);
      velocity=0.0;
      break;
    case 2:
      tm_get_position(2*axis_select,&pos);
      frame->end_time =
	fmod((double)(sdss_get_time() +
		      abs((pos/ticks_per_degree[axis_select] -
			   position)/velocity)), (double)86400.0);
      break;
    case 3:
      if(sdss_delta_time(frame->end_time, sdss_get_time()) < 0.0) {
	 if(frame != NULL) free(frame);
	 printf("MOVE CMD: bad time=%f real time=%f\n",
		frame->end_time, sdss_get_time());
	 TRACE(0, "MOVE CMD: bad time=%f", frame->end_time, 0);
	 TRACE(0, "          real time=%f", sdss_get_time(), 0);
	 return "ERR: BAD TIME";
      }
      
      if(drift_break[axis_select]) {
	 if(DRIFT_verbose) {
            printf("DRIFT pvt %f %f %f\n", position,velocity,frame->end_time);
	 }
	 
	 taskLock();
	 tm_get_position(2*axis_select, &pos);
	 dt = sdss_delta_time(frame->end_time, sdss_get_time());
	 taskUnlock();
/*
 * in practice, shifted around sample frequency (.05) for better
 * err transition
 */
	 dt -=.043;	/* modify time to reduce error during transition */
	 pos = (pos + drift_velocity[axis_select]*dt)/
						 ticks_per_degree[axis_select];
	 if(DRIFT_verbose) {
	    printf("DRIFT modified pvt %f %f %f, difference=%f, dt=%f\n",
		   pos, velocity, frame->end_time, position - pos, dt);
	 }
	 
	 if(drift_modify_enable) {
	    position = pos;
	 }
	 
	 drift_break[axis_select] = FALSE;
      }
      break;
   }
   
   frame->position = position;

   if(fabs(velocity) > max_velocity[axis_select]) {
      TRACE(2, "Max vel. for %s exceeded: %ld",
	    axis_name(axis_select), (long)velocity);
      velocity = (velocity > 0) ?
	max_velocity[axis_select] : -max_velocity[axis_select];
   }
   
   frame->velocity = (double)velocity;
   frame->nxt = NULL;

   sdssdc.tccmove[axis_select].position=
     (long)(frame->position*ticks_per_degree[axis_select]);
   sdssdc.tccmove[axis_select].velocity=
     (long)(frame->velocity*ticks_per_degree[axis_select]);
   sdssdc.tccmove[axis_select].time=(long)(frame->end_time*1000);
/*
 * queues are initialized with one dummy entry by axisMotionInit
 */
   taskLock();
   nxtque = queue->end;
   nxtque->nxt = frame;
   if(queue->active == NULL) {/* end of queue, and becomes active frame */
      queue->active = frame;
   }
   
   for(i = 0; i < OFF_MAX; i++) {
      if(offset_queue_end[axis_select][i] != NULL) {
	 /* still correcting offset , reduce new specifications */
	 offset_queue_end[axis_select][i] = frame;
	 frame->position -= offset[axis_select][i][1].position;
	 frame->velocity -= offset[axis_select][i][1].velocity;

	 TRACE(3, "Offsetting %s", axis_name(axis_select), 0);
	 TRACE(3, "     pos = %d, vel = %d",
	       sdssdc.tccmove[axis_select].position,
	       sdssdc.tccmove[axis_select].velocity);
	 
#if 0
	 printf("reduce offset end=%p, pos=%f,idx=%d\n",frame,
		 frame->position,offset_idx[axis_select][i]);
#endif
      }
   }
   
   queue->end = frame;
   queue->cnt++;
   taskUnlock();
/*
 * clean up queue for old entries
 */
  while(queue->cnt > MAX_FRAME_CNT && queue->top != queue->active) {
     nxtque = queue->top;
     queue->top = nxtque->nxt;
     free(nxtque);
     queue->cnt--;
  }

  return "";
}

/*=========================================================================
**=========================================================================
**
**	+MOVE does nothing
**	+MOVE pos - offsets position by specified amount
**	+MOVE pos vel - offsets position by specified amount and increases
**	velocity as well.
**	+MOVE pos vel time - unimplemented and not required
**	Builds a pvt pair and uses the same calculations to provide a bump to
**	the existing waveform.  The bump is disabled when new pvts are 
**	specified.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** GLOBALS REFERENCED:
**	sdssdc
**	axis_select
**	axis_queue
**	offset_axis_queue
**	offset
**	offset_idx
**
**=========================================================================
*/
char *
plus_move_cmd(char *cmd)
{
   double position,velocity,frame_time;
   struct FRAME_QUEUE *queue;
   int cnt;
   int i;
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   for(i = 0; i < OFF_MAX; i++) {
      if(offset_queue_end[axis_select][i] == NULL) break;
   }
   
   if(i >= OFF_MAX) {
      return "ERR: offset active";
   }

   queue = &axis_queue[axis_select];
   cnt = sscanf(cmd,"%lf %lf %lf", &position, &velocity, &frame_time);
   
   switch (cnt) {
    case -1:
    case 0:
      break;		/* NULL offset - does nothing */
    case 1:
      if(position == 0.0) break;
      
      offset[axis_select][i][0].nxt = &offset[axis_select][i][1];
      offset[axis_select][i][0].position = 0;
      offset[axis_select][i][1].position = position;
      offset[axis_select][i][0].velocity = 0;
      offset[axis_select][i][1].velocity = 0;
      offset[axis_select][i][0].end_time = 0;
/*
 * short offsets are give some extra time for smooth ramp.  long offsets
 * are spread over a time period averaging .4 degs per second
 */
      if(position < 0.15) {
	 offset[axis_select][i][1].end_time = (double)0.75;
      } else {
	 offset[axis_select][i][1].end_time =
				       (double)((int)((position/0.15)*20))/20.;
      }

      offset_idx[axis_select][i] = 0;
      offset_queue_end[axis_select][i] = queue->end;
      
      break;
    case 2:
      if(position == 0.0 && velocity == 0.0) break;
      
      offset[axis_select][i][0].nxt = &offset[axis_select][i][1];
      offset[axis_select][i][0].position = 0;
      offset[axis_select][i][1].position = position;
      offset[axis_select][i][0].velocity = 0;
      offset[axis_select][i][1].velocity = velocity;
      offset[axis_select][i][0].end_time = 0;
      
      if(position < 0.15) {
	 offset[axis_select][i][1].end_time=(double).75;
      } else {			/* average .4 degree per second */
	 offset[axis_select][i][1].end_time =
	   (double)((int)((position/.15)*20))/20.;
      }
      
      offset_idx[axis_select][i] = 0;
      offset_queue_end[axis_select][i] = queue->end;
      break;
    case 3:
      TRACE(0, "Attempt to specify +MOVE p v t", 0, 0);
      return "ERR: unimplemented";
#if 0
      if(position == 0.0 && velocity == 0.0) break;
      
      offset[axis_select][i][0].nxt = &offset[axis_select][i][1];
      offset[axis_select][i][0].position = 0;
      offset[axis_select][i][1].position = position;
      offset[axis_select][i][0].velocity = 0;
      offset[axis_select][i][1].velocity = velocity;
      offset[axis_select][i][0].end_time = 0;
      if(position < 0.2) {
	 offset[axis_select][i][1].end_time = (double).4;
      } else {
	 offset[axis_select][i][1].end_time =
	   (double)((int)((position/1.0)*20))/20.;
      }
      offset_idx[axis_select][i]=0;
      offset_queue_end[axis_select][i]=queue->end;
#endif
      break;
   }

   printf("%p: queue_end=%p, position=%f, velocity=%f, end_time=%f\n",
	  &offset[axis_select][i],offset_queue_end[axis_select][i],
	  offset[axis_select][i][1].position,
	  offset[axis_select][i][1].velocity,
	  offset[axis_select][i][1].end_time);
   
   sdssdc.tccpmove[axis_select].position =
     (long)(offset[axis_select][i][1].position*ticks_per_degree[axis_select]);
   sdssdc.tccpmove[axis_select].velocity =
     (long)(offset[axis_select][i][1].velocity*ticks_per_degree[axis_select]);
   sdssdc.tccpmove[axis_select].time =
     (long)(offset[axis_select][i][1].end_time*1000);
   
   return "";
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: rot_cmd
**	    tel1_cmd
**	    tel2_cmd
**
** DESCRIPTION:
**	IR - instrument rotator axis selected, originally described as "ROT"
**	TEL1 - azimuth axis selected
**	TEL2 - altitude axis selected
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	axis_select
**
**=========================================================================
*/
char *
rot_cmd(char *cmd)			/* NOTUSED */
{
   axis_select = INSTRUMENT;
   return "";
}

char *
tel1_cmd(char *cmd)			/* NOTUSED */
{
   axis_select = AZIMUTH;
   return "";
}

char *
tel2_cmd(char *cmd)			/* NOTUSED */
{
   axis_select = ALTITUDE;
   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: stats_cmd
**
** DESCRIPTION:
**	STATS - Tracking error statistics based on fiducial crossing.
**	Unimplemented until fiducials are better understood.  The information
**	is available via print_fiducials(axis, show_all).
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *
stats_cmd(char *cmd)			/* NOTUSED */
{
   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: status_cmd
**
** DESCRIPTION:
**	STATUS - Returns status of the axis.
**
** RETURN VALUES:
**	return "pos vel time status index" or "ERR:..."
**
** CALLS TO:
**	sdss_get_time
**
** GLOBALS REFERENCED:
**	axis_select
**	semMEIUPD
**	tmaxis
**	fiducial
**	axis_stat
**
**=========================================================================
*/
/* returns: "%position %velocity %time %status_word %index_position" */
static long status=0x40000000;

char *
status_cmd(char *cmd)
{
   double sdss_time;			/* time from sdss_get_time() */
   static char status_ans[200];		/* reply */

   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   sdss_time = sdss_get_time();
   if(sdss_time < 0) {
      sdss_time += 1.0;
   }
   
   if(semTake(semMEIUPD,60) == ERROR) {
      TRACE(0, "status_cmd: failed to get semMEIUPD: %s (%d)",
	    strerror(errno), errno);
      sprintf(status_ans, "ERR: semMEIUPD : %s", strerror(errno));
   }

   sprintf(status_ans,"%f %f %f %ld %f",
	    (*tmaxis[axis_select]).actual_position/ticks_per_degree[axis_select],
	    (*tmaxis[axis_select]).velocity/ticks_per_degree[axis_select],
	    sdss_time, *(long *)&axis_stat[axis_select],
	    fiducial[axis_select].mark/ticks_per_degree[axis_select]);
   semGive(semMEIUPD);

   return status_ans;
}

/*=========================================================================
**=========================================================================
**
**	STATUS.LONG - Returns long status of the axis.
**
** RETURN VALUES:
**	return "date time
**		pos position
**		vel velocity
**		bits status
**		deg last index" or "ERR:..."
**	currently returning only "bits status"
**
** CALLS TO:
**	sdss_get_time
**
** GLOBALS REFERENCED:
**	axis_select
**	semMEIUPD
**	tmaxis
**	fiducial
**	axis_stat
**
**=========================================================================
*/
/* returns:    "%date %time
		%f position 
		%f velocity 
		%b status 
		%f last index" */
char *
status_long_cmd(char *cmd)		/* NOTUSED */
{
   static char status_long_ans[33];
   int i;
   
   printf (" STATUS.LONG command fired\r\n");
   for(i = 0;i < 32; i++) {
      status_long_ans[i]= (status & (1 << (31 - i))) ? '1' : '0';
   }
   status_long_ans[i] = '\0';

   return status_long_ans;
}


/*****************************************************************************/
/*
 * Is an e-stop in?
 */
int
check_stop_in(void)
{
   if(sdssdc.status.i6.il0.w_lower_stop &&
      sdssdc.status.i6.il0.e_lower_stop &&
      sdssdc.status.i6.il0.s_lower_stop &&
      sdssdc.status.i6.il0.n_lower_stop &&
      sdssdc.status.i6.il0.w_rail_stop &&
      sdssdc.status.i6.il0.s_rail_stop &&
      sdssdc.status.i6.il0.n_rail_stop &&
      sdssdc.status.i6.il0.n_fork_stop &&
      sdssdc.status.i6.il0.n_wind_stop &&
      sdssdc.status.i6.il0.cr_stop &&
      sdssdc.status.i6.il0.s_wind_stop) {
      return FALSE;
   } else {
      return TRUE;
   }
}

/*****************************************************************************/
/*
 * Return the clinometer reading
 */
float
read_clinometer(void)
{
/*
 * 8857 is pinned at 0 degrees; -9504 is zenith 90 degrees 22-Aug-98
 */
   const float altclino_sf = 0.0048683116163; /* scale factor */
   const int altclino_off = 8937;	/* offset */

   return((altclino_off - sdssdc.status.i4.alt_position)*altclino_sf);
}

/*****************************************************************************/
/*
 * An axis-status command that can be used by IOP to get enough information
 * to update the MCP Menu
 */
char *
axis_status_cmd(char *cmd)
{
   const int axis = axis_select;	/* in case it changes */
   int brake_is_on = -1;		/* is the brake on for current axis? */
   long fid_mark = 0;			/* position of fiducial mark */
   static char reply_str[200 + 1];	/* desired values */

   TRACE(8, "Setting reply_str[200] axis == %d", axis, 0);
   reply_str[200] = '\a';
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   TRACE(8, "taking semMEIUPD", 0, 0);
   if (semTake(semMEIUPD, 60) == ERROR) { 
      TRACE(5, "ERR: semMEIUPD : %d", errno, 0);
      sprintf(reply_str, "ERR: semMEIUPD : %s", strerror(errno));
      return(reply_str);
   }

   switch (axis) {
    case AZIMUTH:
      brake_is_on = sdssdc.status.i9.il0.az_brake_en_stat;
      fid_mark = az_fiducial[fiducialidx[axis]].mark;
      break;
    case ALTITUDE:
      brake_is_on = sdssdc.status.i9.il0.alt_brake_en_stat;
      fid_mark = alt_fiducial[fiducialidx[axis]].mark;
      break;
    case INSTRUMENT:
      brake_is_on = -1;			/* there is no brake */
      fid_mark = rot_fiducial[fiducialidx[axis]].mark - ROT_FID_BIAS;
      break;
    default:
      semGive(semMEIUPD);
      fprintf(stderr,"axis_status_cmd: impossible instrument %d\n",
	      axis);
      abort();
      break;				/* NOTREACHED */
   }

   TRACE(8, "setting reply_str", 0, 0);
   sprintf(reply_str,
	   "%f %d %d  %ld %ld %ld %ld  %d %d %ld  %d %d 0x%lx  %.4f",
	   ticks_per_degree[axis], monitor_on[axis], axis_state(2*axis),
	   tmaxis[axis]->actual_position, tmaxis[axis]->position,
		   tmaxis[axis]->voltage, tmaxis[axis]->velocity,
	   fiducialidx[axis], fiducial[axis].seen_index, fid_mark,
	   check_stop_in(), brake_is_on, *(long *)&axis_stat[axis],
	   read_clinometer());

   TRACE(8, "giving semMEIUPD", 0, 0);
   semGive(semMEIUPD);

   TRACE(8, "Checking reply_str[200]: %d", reply_str[200], 0);
   assert(reply_str[200] == '\a');	/* check for overflow */

   return(reply_str);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: brakeon_cmd
**	    brakeoff_cmd
**
** DESCRIPTION:
**	BRAKE.ON - turn on the selected brake.
**	BRAKE.OFF - turn off the selected brake.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**	tm_sp_az_brake_on
**	tm_sp_az_brake_off
**	tm_sp_alt_brake_on
**	tm_sp_alt_brake_off
**
** GLOBALS REFERENCED:
**	axis_select
**
**=========================================================================
*/
char *brakeon_cmd(char *cmd)
{
  mcp_set_brake(axis_select);
  
  return "";
}

char *brakeoff_cmd(char *cmd)
{
   mcp_unset_brake(axis_select);
   
   return "";
}

/*****************************************************************************/
/*
 * Return the status of misc things
 */
static int
get_miscstatus(char *status,
	       int size)			/* dimen of status[] */
{
  int len;

  sprintf(status,"Misc: %d %d\n",
	  sdssdc.status.i9.il0.clamp_en_stat, /* alignment clamp */
	  sdssdc.status.i9.il0.clamp_dis_stat);
  
  len = strlen(status);
  assert(len < size);

  return(len);
}

/*****************************************************************************/
/*
 * An status command that can be used by IOP to get enough information
 * to update the MCP Menu. Returns everything except the axis status and
 * the state of the semCmdPort
 */
char *
system_status_cmd(char *cmd)
{
   int i;
   static char reply_str[200 + 1];	/* desired values */
   const int size = sizeof(reply_str) - 1;

   reply_str[size] = '\a';

   TRACE(8, "taking semMEIUPD", 0, 0);
   if (semTake(semMEIUPD, 60) == ERROR) { 
      TRACE(5, "system_status_cmd: semMEIUPD : %d", errno, 0);
      sprintf(reply_str, "ERR: semMEIUPD : %s", strerror(errno));
      return(reply_str);
   }

   i = 0;
   i += get_cwstatus(&reply_str[i], size - i);
   i += get_ffstatus(&reply_str[i], size - i);
   i += get_slitstatus(&reply_str[i], size - i);
   i += get_miscstatus(&reply_str[i], size - i);

   TRACE(8, "giving semMEIUPD", 0, 0);
   semGive(semMEIUPD);

   TRACE(8, "Checking reply_str[end]: %d", reply_str[size], 0);
   assert(reply_str[size] == '\a');	/* check for overflow */

   return(reply_str);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: abstatus_cmd
**
** DESCRIPTION:
**	AB.STATUS off len - return upto 20 words of allen-bradley status starting
**	at a specified position.
**
** RETURN VALUES:
**	return "xxxx xxxx ...."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
char abstatus_ans[5*20];
char *abstatus_cmd(char *cmd)
{
  int i,idx;
  short *dt;
  int off,len;

  printf (" AB.STATUS command fired\r\n");
  sscanf (cmd,"%d %d",&off,&len);
  if (len>20) return "ERR: bad length";
  dt=(short *)&sdssdc.status;
  dt+=off;
  idx=0;
  for (i=0;i<len;i++,dt++)
    idx+=sprintf (&abstatus_ans[idx],"%04x ",*dt);
  return &abstatus_ans[0];
}

/*=========================================================================
**=========================================================================
**
**	Calculate frames from pvt pairs starting at a specified position.
**	Up to MAX_CALC frames are returned into the global buffers for
**	specification into MEI frames.
**
**	Inputs are:
**	xi, vi, ti
**	xf, vf, tf
**	
**	p(t) = pi + vi (t-ti) + 1/2 ai (t-ti)^2 + 1/6 j (t-ti)^3
**	v(t) = vi + ai (t-ti) + 1/2 j  (t-ti)^2
**	a(t) = ai + j  (t-ti)
**	j(t) = j
**	
**	where ti <= t <= tf
**	
**	dx = xf - xi
**	dv = vf - vi
**	dt = tf - ti
**	v' = dx / dt
**	
**	ai = (2/dt) * ((3 * v') - (2 * vi) - vf)
**	j  = (6/(dt^2)) * (vi + vf - (2 * v'))
**	
**
** RETURN VALUES:
**	number of frames processed
**
** CALLS TO:
**	sdss_delta_time
**
** GLOBALS REFERENCED:
**	tim,p,v,a,ji
**	time_off
**
**=========================================================================
*/
int
calc_frames(int axis, struct FRAME *iframe, int start)
{
   double dx,dv,dt,vdot;
   double ai,j,t,lai,lj,lt,ldt;
   struct FRAME *fframe;
   struct FRAME *lframe;
   int i;
   
   /* problem............................*/
   if(iframe->nxt == NULL) {
      TRACE(0, "Initial frame has NULL ->nxt pointer", 0, 0);
      return ERROR;
   }
   
   fframe = iframe->nxt;
   dx = fframe->position - iframe->position;
   dv = fframe->velocity - iframe->velocity;
   dt = sdss_delta_time(fframe->end_time, iframe->end_time);
   vdot = dx/dt;
   ai = (2/dt)*(3*vdot - 2*iframe->velocity - fframe->velocity);
   j = (6/(dt*dt))*(iframe->velocity + fframe->velocity - 2*vdot);
/*
 * necessary if for loop not executed; calc of t
 */
   t = (start + 1)/FLTFRMHZ + time_off[axis]; /* for end condition */

   if(CALC_verbose) {
      printf("\r\n iframe=%p, fframe=%p",iframe,fframe);
      printf("\r\n iframe->end_time=%f, fframe->end_time=%f",
	      iframe->end_time,fframe->end_time);
      printf("\r\n dx=%12.8f, dv=%12.8f, dt=%12.8f, vdot=%f",dx,dv,dt,vdot);
      printf("\r\n ai=%12.8f, j=%12.8f, t=%f, start=%d, time_off=%f",
	     ai,j,t,start,time_off[axis]);
  }

   for(i = 0;
       i < (int)min(MAX_CALC-1, (int)((dt - time_off[axis])*FRMHZ) - start);
									 i++) {
      t = (i + start + 1)/FLTFRMHZ + time_off[axis];
      tim[axis][i] = 1/FLTFRMHZ;
      p[axis][i] = iframe->position + iframe->velocity*t + (1/2.)*ai*(t*t) +
	(1/6.)*j*(t*t*t);
      v[axis][i] = iframe->velocity + ai*t + (1/2.)*j*(t*t);
      a[axis][i] = ai + j*t;
      ji[axis][i] = j;
      
      if(CALC_verbose) {
	 printf ("\r\n%d @%f Secs: ti=%f, p=%12.8f, v=%12.8f, a=%12.8f",
		 i,t,tim[axis][i],p[axis][i],v[axis][i],a[axis][i]);
      }
   }
/*
 * last one with a portion remaining; needs portion of next one
 */
#if 0
   printf("Check for FINAL: time_off=%f, i=%d, start=%d, t=%f, dt=%f\n",
	  time_off[axis], i, start, t, dt);
#endif

   if((int)(i + start) != (int)((dt - time_off[axis])*FLTFRMHZ) ||
						    t == dt - time_off[axis]) {
      if(i > MAX_CALC - 1) {
	 TRACE(0, "calc_frames has problems (A) %d\n",i, 0);
      }
      return i;
   }

   ldt = ((dt - time_off[axis])*FLTFRMHZ -
			       (int)((dt - time_off[axis])*FLTFRMHZ))/FLTFRMHZ;
   t = 1/FLTFRMHZ - ldt;
   lt = dt;
   tim[axis][i] = 1/FLTFRMHZ;
   
   lframe = fframe;
   if(lframe->nxt == NULL) {
      TRACE(3, "CALC FRAME: next frame required to finish", 0, 0);
      return ERROR;
   }
    
   fframe = lframe->nxt;
   time_off[axis] = t;
   lai = ai; 
   lj = j;
   dx = fframe->position - lframe->position;
   dv = fframe->velocity - lframe->velocity;
   dt = fframe->end_time - lframe->end_time;
   vdot = dx/dt;

   if(CALCFINAL_verbose) {
      printf("time_off=%f, ldt=%f, lt=%f, t=%f\n",time_off[axis],ldt,lt,t);
      printf("dx=%12.8f, dv=%12.8f, dt=%12.8f, vdot=%12.8f\n",dx,dv,dt,vdot);
   }
   
   ai = (2/dt)*(3*vdot - 2*lframe->velocity - fframe->velocity);
   j = (6/(dt*dt))*(lframe->velocity + fframe->velocity - 2*vdot);
   
   p[axis][i] = lframe->position + lframe->velocity*t + (1/2.)*ai*(t*t) +
							      (1/6.)*j*(t*t*t);
   v[axis][i] = lframe->velocity + ai*t + (1/2.)*j*(t*t);
   a[axis][i] = FLTFRMHZ*t*(ai + j*t) + FLTFRMHZ*ldt*(lai + (lj*lt));
   ji[axis][i] = FLTFRMHZ*t*j + FLTFRMHZ*ldt*lj;
   
   if(CALCFINAL_verbose) {
      printf("Final %d @%f Secs: ti=%f, p=%12.8f, v=%12.8f, a=%12.8f\n",
	     i,t,tim[axis][i],p[axis][i],v[axis][i],a[axis][i]);
   }
   
   if(i + 1 > MAX_CALC) {
      TRACE(0, "calc_frames has problems (B) %d\n",i + 1, 0);
   }

   return(i + 1);
}

/*=========================================================================
**=========================================================================
**
**	Calculate offset frames from pvt pairs starting at a specified position
**	Up to MAX_CALC frames are returned into the global buffers for
**	specification into MEI frames.  Similar to calc_frames and applied
**	as an addition to calc_frames or a bump.
**
** RETURN VALUES:
**	number of offset frames processed
**
** CALLS TO:
**	sdss_delta_time
**
** GLOBALS REFERENCED:
**	timoff,poff,voff,aoff,jioff
**
**=========================================================================
*/
int
calc_offset(int axis, struct FRAME *iframe, int start, int cnt)
{
   double dx,dv,dt,vdot;
   double ai,j,t;
   struct FRAME *fframe;
   int i,ii;
  
   fframe = iframe->nxt;
   dx = fframe->position - iframe->position;
   dv = fframe->velocity - iframe->velocity;
   dt = sdss_delta_time(fframe->end_time, iframe->end_time);
   vdot = dx/dt;
   ai = (2/dt)*(3*vdot - 2*iframe->velocity - fframe->velocity);
   j = (6/(dt*dt))*(iframe->velocity + fframe->velocity - 2*vdot);
/*
 * neccessary if for loop not executed; calc of t
 */
   t = (start + 1)/FLTFRMHZ;		/* for end condition */
   if(CALCOFF_verbose) {
      printf("dx=%12.8f, dv=%12.8f, dt=%12.8f, vdot=%f\n",dx,dv,dt,vdot);
      printf("ai=%12.8f, j=%12.8f, t=%f, start=%d,\n",ai,j,t,start);
   }
   
   for(i = 0; i < (int)min(cnt, (int)(dt*FRMHZ)-start); i++) {
      t = (i + start + 1)/FLTFRMHZ;
      timoff[axis][i] = 1/FLTFRMHZ;
      poff[axis][i] += iframe->position + iframe->velocity*t + (1/2.)*ai*(t*t)+
							      (1/6.)*j*(t*t*t);
      voff[axis][i] += iframe->velocity + ai*t + (1/2.)*j*(t*t);
      aoff[axis][i] += ai + j*t;
      jioff[axis][i] += j;
      
      if(CALCOFF_verbose) {
	 printf ("%d @%f Secs: ti=%f, p=%12.8f, v=%12.8f, a=%12.8f\n",
		 i,t,timoff[axis][i],poff[axis][i],voff[axis][i],
		 aoff[axis][i]);
      }
   }

   for(ii = i; ii < cnt; ii++) {         
      t = (ii+start+1)/FLTFRMHZ;
      timoff[axis][ii] = 1/FLTFRMHZ;
      poff[axis][ii] += fframe->position + fframe->velocity*(t - dt);
      voff[axis][ii] += fframe->velocity;
      
      if(CALCOFF_verbose) {
	 printf("%d @%f Secs: ti=%f, p=%12.8f, v=%12.8f, a=%12.8f\n",
		ii,t,timoff[axis][ii],poff[axis][ii],voff[axis][ii],
		aoff[axis][ii]);
      }
   }
   
   return i;
}

/*=========================================================================
**=========================================================================
**
**	Clear the offset calculation.
**
** RETURN VALUES:
**	number of offset calculations cleared
**
** GLOBALS REFERENCED:
**	poff,voff,aoff,jioff
**
**=========================================================================
*/
int
clroffset(int axis,int cnt)
{
   int i;
   
   for(i = 0; i < cnt; i++) {         
      poff[axis][i] = voff[axis][i] = aoff[axis][i] = jioff[axis][i] = 0;
   }

   return cnt;
}

/*=========================================================================
**=========================================================================
**
**	Add the offset calculation to the MEI frames.
**
** RETURN VALUES:
**	number of offset calculations added
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	p,v,a,ji
**	poff,voff,aoff,jioff
**
**=========================================================================
*/
int
addoffset(int axis,int cnt)
{
   int i;

   for(i=0;i<cnt;i++) {
      p[axis][i] += poff[axis][i];
      v[axis][i] += voff[axis][i];
      a[axis][i] += aoff[axis][i];
      ji[axis][i] += jioff[axis][i];

      if(CALCADDOFF_verbose) {
	 printf("%d:  p=%12.8f, v=%12.8f, a=%12.8f\n",
		i,p[axis][i],v[axis][i],a[axis][i]);
      }
   }

   return cnt;
}

/*=========================================================================
**=========================================================================
**
**	Start the frame processing after at a specified time.  The MEI
**	controller dwells after all frames inside the MEI have been 
**	purged.
**
** RETURN VALUES:
**	number of offset calculations added
**
** CALLS TO:
**	tm_frames_to_execute
**	sdss_delta_time
**
**=========================================================================
*/
void
start_frame(int axis,double time)
{
   int lcnt;
  
   time_off[axis] = 0.0;
   while ((lcnt = tm_frames_to_execute(axis)) > 1) {
#if 0
      printf("Dwell frames left=%d\n",lcnt);
#endif
      taskDelay(3);
   }
   
   taskDelay(5);
   if(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      TRACE(0, "start_frame: Failed to get semMEI: %s (%d)",
	    strerror(errno), errno);
   } else {
      time = sdss_delta_time(time, sdss_get_time());
#if 0
      printf("time to dwell=%f\n",time);
#endif
      dsp_dwell(2*axis, time);
      semGive(semMEI);
   }
   
   printf("START axis=%d: time=%f\n", 2*axis, time);
}

/*=========================================================================
**=========================================================================
**
**	Retrieves the number of MEI frames between two pvts including the 
**	left over remants of the previous pvt pair.
**
** RETURN VALUES:
**	number of MEI frames
**
**=========================================================================
*/
int
get_frame_cnt(int axis, struct FRAME *iframe)
{
   struct FRAME *fframe;
   double dt;
   int cnt;
   
   fframe = iframe->nxt;
   dt = fframe->end_time-iframe->end_time;
   cnt = (dt - time_off[axis])*FLTFRMHZ;
#if 0
   printf("first cnt=%d,%f,%f\n",cnt,dt,(dt-time_off[axis])*FLTFRMHZ);
#endif
   if((dt - time_off[axis])*FLTFRMHZ > (int)((dt - time_off[axis])*FLTFRMHZ)) {
      cnt++;
#if 0
      printf("final cnt=%d\n",cnt);
#endif
   }
   
   return cnt;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE:
**	    set_rot_state (int state)
**
**	    set_rot_coeffs
**	    print_rot_coeffs
**	    set_rot_uplimit
**	    set_rot_dnlimit
**	    coeffs_state_cts	modify the state based on velocity in raw counts
**
** DESCRIPTION:
**	Modify the instrument rotators coeffs as a function of velocity.
**	Supporting routines for the specification of the state machine which
**	provides hysterisis to prevent undue switching.  This was used and 
**	tested, but is deprecated by reducing the sampling frequency of the
**	rotator and thus increasing the gain of the system.  I want to maintain
**	these functions.
**
**      N.b. These routines are only compiled/called if SWITCH_PID_COEFFS
**      is true, with the exception of set_rot_state()
**
** RETURN VALUES:
**	number of MEI frames
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
#if !SWITCH_PID_COEFFS

void
set_rot_state(int state)
{
   assert(state == -1);			/* called from mcp.login */
}

#else

int axis_coeffs_state[]={-1,-1,-1,-1,-1,-1};
int *rotcoeffs=&axis_coeffs_state[4];
struct SW_COEFFS rot_coeffs[]={
        {{320,48,600,0,0,32767,0,15000,-6,1000},7000/ROT_TICKS_DEG,-1.0,              7000,-1},
        {{320,44,600,0,0,32767,0,15000,-6,1200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,40,600,0,0,32767,0,15000,-6,1500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,36,600,0,0,32767,0,15000,-6,1700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,32,600,0,0,32767,0,15000,-6,2000},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,28,600,0,0,32767,0,15000,-6,2200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,24,600,0,0,32767,0,15000,-6,2500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,20,600,0,0,32767,0,15000,-6,2700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,16,600,0,0,32767,0,15000,-6,3000},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,12,600,0,0,32767,0,15000,-6,3200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{300,10,600,0,0,32767,0,15000,-6,3500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{260, 8,600,0,0,32767,0,15000,-6,3600},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{220, 6,600,0,0,32767,0,15000,-6,3700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{180, 4,600,0,0,32767,0,15000,-6,3800},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{140, 3,600,0,0,32767,0,15000,-6,3900},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{100, 2,300,0,0,32767,0,15000,-6,4000},-1.0              ,5000/ROT_TICKS_DEG,   -1,5000}
};

void
set_rot_coeffs (int state, int index, short val)
{
   rot_coeffs[state].coeffs[index] = val;
}

void
print_rot_coeffs(void)
{
   int i;
  
   printf ("State %d Active\n",axis_coeffs_state[4]);
   for (i=0;i<sizeof(rot_coeffs)/sizeof(struct SW_COEFFS);i++) {
      printf ("rot_coeffs state %d: uplimit=%d, dnlimit=%d\n",
	      i,rot_coeffs[i].uplimit_cts,rot_coeffs[i].dnlimit_cts);
      printf("P=%d,I=%d,D=%d,AFF=%d,VFF=%d,ILIM=%d,OFF=%d,OLIM=%d,S=%d,FFF=%d\n",
	     rot_coeffs[i].coeffs[0],      /* P */
	     rot_coeffs[i].coeffs[1],      /* I */
	     rot_coeffs[i].coeffs[2],      /* D */
	     rot_coeffs[i].coeffs[3],      /* AFF */
	     rot_coeffs[i].coeffs[4],      /* VFF */
	     rot_coeffs[i].coeffs[5],      /* ILIM */
	     rot_coeffs[i].coeffs[6],      /* OFF */
	     rot_coeffs[i].coeffs[7],      /* OLIM */
	     rot_coeffs[i].coeffs[8],      /* S */
	     rot_coeffs[i].coeffs[9]);     /* FFF */
   }
}

void
set_rot_uplimit (int state, int val)
{
   rot_coeffs[state].uplimit_cts=val;
   rot_coeffs[state].uplimit_deg=val/ROT_TICKS_DEG;
}

void
set_rot_dnlimit (int state, int val)
{
   rot_coeffs[state].dnlimit_cts=val;
   rot_coeffs[state].dnlimit_deg=val/ROT_TICKS_DEG;
}

void
set_rot_state(int state)
{
   axis_coeffs_state[4] = state;
}

int
coeffs_state_cts(int mei_axis,
		 int cts)
{
   int state;
   struct SW_COEFFS *coeff;
   
   state = axis_coeffs_state[mei_axis];
   if(state == -1) return FALSE;

   taskDelay(1);
   coeff = &rot_coeffs[state];
#if 0
   printf ("\r\nAXIS %d: state=%d coeff=%p",mei_axis,state,coeff);
#endif
   
   if(coeff->uplimit_cts > 0 && abs(cts) > coeff->uplimit_cts) {
#if 0
      printf ("UP cts=%d, uplimit=%d",cts,coeff->uplimit_cts);
#endif
      state++; coeff++;
   } else if(abs(cts) < coeff->dnlimit_cts) {
#if 0
      printf("DN cts=%d, dnlimit=%d", cts, coeff->dnlimit_cts);
#endif
      if(state == 0) return FALSE;
      state--; coeff--;
   } else {
      return FALSE;
   }

   dsp_set_filter(mei_axis, (P_INT)&coeff->coeffs[0]);
   axis_coeffs_state[mei_axis] = state;
   return TRUE;
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: load_frames
**	    load_test_frames - diagnostic to print but not load frames
**
** DESCRIPTION:
**	Load the MEI axis frames into the DSP.  Checks for maximum and
**	updates the largest value.  No action is taken at this time
**	if any maximum is exceeded other than a message.
**	The diagnostic queue was used to aid in trapping problems.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tim,p,v,a,ji
**	max_acceleration
**	max_velocity
**	sdssdc
**	diagq, diagq_i
**
**=========================================================================
*/
void
load_frames(int axis, int cnt, int idx, double sf)
{
   int e;
   int i;
   FRAME frame;
   
   if(FRAME_verbose) {
      printf("\nLoad %d Frames, sf=%f\n",cnt,sf);
   }
   
   for(i = idx; i < cnt + idx; i++) {
      if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = a[axis][i];
      }
      
      if(fabs(a[axis][i]) > max_acceleration[axis]) {
	 printf("AXIS %d: MAX ACC %f exceeded by %f\n",
		axis, a[axis][i], max_acceleration[axis]);
	 {
	    long acc = a[axis][i];	/* TRACE macro has a variable "a" */
	    TRACE(2, "Max accl. for %s exceeded: %ld", axis_name(axis), acc);
	 }
      }
      
      if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = v[axis][i];
      }
      
      if(fabs(v[axis][i]) > max_velocity[axis]) {
	 printf ("AXIS %d: MAX VEL %f exceeded by %f\n",
		 axis,v[axis][i], max_velocity[axis]);
	 TRACE(2, "Max vel. for %s exceeded: %ld",
	       axis_name(axis), (long)v[axis][i]);
      }
      
      if(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "load_frames: failed to get semMEI: %s (%d)",
	       strerror(errno), errno);
      } else {
	 taskLock();
	 e=frame_m_xvajt_corr(&frame,"0l xvajt un d", axis<<1,
			      p[axis][i]*sf,
			      v[axis][i]*sf,
			      a[axis][i]*sf,
			      ji[axis][i]*sf,
			      tim[axis][i],
			      FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,NEW_FRAME);
	 taskUnlock();
	 semGive (semMEI);
      }
      
      sdssdc.pvt[axis].position = (long)(p[axis][i]*sf);
      sdssdc.pvt[axis].velocity = (long)(v[axis][i]*sf);
      sdssdc.pvt[axis].time = (long)(tim[axis][i]*1000);
      
      if(DIAGQ_verbose) {
	 if(diagq != NULL && axis == DIAGQ_verbose) {
	    diagq[diagq_i].p = p[axis][i];
	    diagq[diagq_i].v = v[axis][i];
	    diagq[diagq_i].a = a[axis][i];
	    diagq[diagq_i].ji = ji[axis][i];
	    diagq[diagq_i].tim = tim[axis][i];
	    diagq_i = (diagq_i+1)%diagq_siz;
	 }
      }
      
      if(FRAME_verbose) {
	 printf("axis=%d (%d): p=%12.8f, v=%12.8f, a=%12.8f "
		"j=%12.8f,t=%12.8f\n",
		axis<<1, i,
		(double)p[axis][i]*sf, (double)v[axis][i]*sf,
		(double)a[axis][i]*sf, ji[axis][i]*sf,
		tim[axis][i]);
      }
   }
}

void
load_frames_test(int axis, int cnt, double sf)
{
   int i;
   
   printf("\r\n Load %d Frames, sf=%f",cnt,sf);
   for(i = 0; i < cnt; i++) {
      printf ("\r\n axis=%d (%d): p=%12.8f, v=%12.8f, a=%12.8f, j=%12.8f,t=%12.8f",
	      axis<<1,i,
	      (double)p[axis][i]*sf,(double)v[axis][i]*sf,
	      (double)a[axis][i]*sf,ji[axis][i]*sf,
	      tim[axis][i]);
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: stop_frame
**
** DESCRIPTION:
**	Stop motion using the stop deceleration built into the MEI.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void stop_frame(int axis,double pos,double sf)
{
  int stopped;
  int state;

  printf ("\r\nSTOP axis=%d: p=%12.8f",
	axis<<1,(double)pos);
  stopped=FALSE; state = 0;		/* get rid of warnings */
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    set_stop(axis<<1);
    stopped=motion_done(axis<<1);
    state=axis_state(axis<<1);
    semGive (semMEI);
  }
  while (!stopped)
  {  
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
      stopped=motion_done(axis<<1);
/*      state=axis_state(axis<<1);*/
      semGive (semMEI);
    }
/*    printf("\r\nStopping");*/
    taskDelay(3);
  }
  printf("\r\nStopped axis=%d, motion_done=%d, axis_state=%d",axis,stopped,state);
  taskDelay(3);
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    clear_status(axis<<1);
    semGive (semMEI);
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: stp_frame
**
** DESCRIPTION:
**	Stop motion alternative similar to end_frame
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void stp_frame(int axis,double pos,double sf)
{
  int stopped;
/*  int state;*/
  int e;
  FRAME frame;
  double position;
  double velocity;

/*  printf ("\r\nSTP axis=%d: p=%12.8f,sf=%f",
	axis<<1,(double)pos,sf);*/

  if (semTake (semMEI,WAIT_FOREVER)!=ERROR) {
     get_position_corr(2*axis, &position);
    get_velocity(axis<<1,&velocity);
    printf ("pos=%f, vel=%f\n", position, velocity);
    if (velocity>0.0)
      e=frame_m(&frame,"0l avj un d",axis<<1,
	(double)(-.5*sf),(double)0.0,(double)0.0,
	FUPD_ACCEL|FUPD_JERK|FTRG_VELOCITY,
	NEW_FRAME|TRIGGER_NEGATIVE);
    else
      e=frame_m(&frame,"0l avj un d",axis<<1,
	(double)(.5*sf),(double)0.0,(double)0.0,
	FUPD_ACCEL|FUPD_JERK|FTRG_VELOCITY,
	NEW_FRAME);
    e=frame_m(&frame,"0l va u d",axis<<1,
	(double)0.0,(double)0.0,
	FUPD_ACCEL|FUPD_VELOCITY,0);
      semGive (semMEI);
  }

  stopped=TRUE;	/* ...gets rid of warning */
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    stopped=in_motion(axis<<1);
    semGive (semMEI);
  }
  while (stopped)
  {  
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
      stopped=in_motion(axis<<1);
/*      state=axis_state(axis<<1);*/
      semGive (semMEI);
    }
/*    printf("\r\nStopping,in_motion%d,status=%x",stopped,state);*/
    taskDelay(3);
  }
/*  printf("\r\nStopped axis=%d, in_motion=%d",axis,stopped);*/
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: drift_frame
**
** DESCRIPTION:
**	Drift at the specified velocity until further commanded.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void drift_frame(int axis,double vel,double sf)
{
  int e;
  FRAME frame;
  
  printf ("\r\nDRIFT axis=%d: v=%12.8f",
	axis<<1,
	(double)vel);
/*  printf("\r\nDrift frames left=%d",tm_frames_to_execute(axis));*/
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
      e=frame_m(&frame,"0l vaj un d",axis<<1,
	(double)vel,(double).2*sf,
	(double)0.0,
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_JERK|FTRG_VELOCITY,NEW_FRAME);
      e=frame_m(&frame,"0l va u d",axis<<1,
	(double)vel,(double)0.0,
	FUPD_ACCEL|FUPD_VELOCITY,0);
      semGive (semMEI);
  }
}
/*=========================================================================
**=========================================================================
**
**	End frames sent to MEI since there are no new pvts.  This should
**	bring the motion to a position with no velocity or acceleration.
**	The controller will remain in closed-loop holding the position.
**
**=========================================================================
*/
void
end_frame(int axis, int index, double sf)
{
   int e;
   FRAME frame;
   
   if(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      TRACE(0, "end_frame: failed to get semMEI: %s (%d)",
	    strerror(errno), errno);
      return;
   } 

   e = frame_m_xvajt_corr(&frame, "0l xvajt un d", 2*axis,
			  p[axis][index]*sf,
			  0.0,
			  0.0,
			  0.0,
			  (1./FLTFRMHZ),
			  FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,0);
   dsp_set_last_command_corr(dspPtr, 2*axis, (double)p[axis][index]*sf);
   semGive(semMEI);
   
   printf("END axis=%d (%d): "
	  "p=%12.8f, v=%12.8f, a=%12.8f, j=%12.8f, t=%12.8f\n",
	  2*axis, index,
	  (double)p[axis][index]*sf,(double)v[axis][index]*sf,
	  (double)a[axis][index]*sf, (double)ji[axis][index]*sf,
	  tim[axis][index]);
}

/*=========================================================================
**
**	Returns remaining frames in MEI queue or ERROR
**
** RETURN VALUES:
**	remaining frames to execute
**
** GLOBALS REFERENCED:
**	semMEI
**
**=========================================================================
*/
static int
tm_frames_to_execute(int axis)
{
   int cnt;
   
   if(semTake(semMEI,60) == ERROR) {
      printf("tm_frames_to_execute error\n");
      return ERROR;    
   }
   
   cnt = frames_to_execute(2*axis);
   semGive(semMEI);
   
   return cnt;    
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_TCC
**	    start_tm_TCC
**
** DESCRIPTION:
**	The task which calculates and loads frames into the MEI for the axis
**	motion.  A task is spawned for each axis and runs independently.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	semMEI
**
**=========================================================================
*/
#define LOAD_MAX        20

void
tm_TCC(int axis)
{
   const char *const aname = axis_name(axis); /* name of our axis */
   int cnt, lcnt, cntoff;
   struct FRAME *frame;
   struct FRAME *frmoff;
   int i;
   int frame_cnt, frame_idx;
   double position;
   double velocity;
   long pos;
   int idx;
   int status;
   
   tm_sem_controller_run(2*axis);
   idx=0;
   printf("Axis=%s;  Ticks per degree=%f\n", aname, ticks_per_degree[axis]);
   
   for(;;) {
/*
 * task should idle here with no input pvt
 */
      while(axis_queue[axis].active == NULL) {
	 axis_alive |= (1 << axis);	/* keep alive bit */
/*
 * in case drifting, no new pvt, and need to stop
 */
	 if(frame_break[axis]) {
	    stp_frame(axis, stop_position[axis],
		      (double)ticks_per_degree[axis]);
	    frame_break[axis] = FALSE;
	 }
	 taskDelay(3);
      }

      frame=axis_queue[axis].active;
      drift_break[axis]=FALSE;
/*
 * reposition if necessary
 */
      if(semTake(semMEI,WAIT_FOREVER) == ERROR) {
	 TRACE(0, "tm_TCC: failed to take semMEI: %s (%d)",
	       strerror(errno), errno);
      }

      get_position_corr(2*axis, &position);
      get_velocity(2*axis,&velocity);
      semGive(semMEI);
      
      pos = (long)position;
      TRACE(8, "Check Params for repositioning %s", aname, 0);

      if(abs((long)((frame->position*ticks_per_degree[axis]) - position)) >
					 (long)(0.01*ticks_per_degree[axis]) &&
							 fabs(velocity) == 0) {
	 while((lcnt = tm_frames_to_execute(axis)) > 1) {
	    TRACE(8, "Frames left for %s: %d", aname, lcnt);
	    taskDelay(1);
	 }
	 tm_start_move(2*axis,
		       1*(double)ticks_per_degree[axis],
		       0.05*(double)ticks_per_degree[axis],
		       frame->position*(double)ticks_per_degree[axis]);

	 TRACE(3, "Repositioning %s by TCC vmd", aname, 0);
	 TRACE(3, "    from pos=%lf to pos=%ld",
	       (long)position, (long)(frame->position*ticks_per_degree[axis]))
	 
	 status=TRUE;
	 while((abs((frame->position*ticks_per_degree[axis]) - position) >
		(long)(0.01*ticks_per_degree[axis])) &&
	       status == TRUE) {
	    taskDelay (10);
	    if(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	       TRACE(0, "tm_TCC: failed to retake semMEI: %s (%d)",
		     strerror(errno), errno);
	    } else {
	       status = in_motion(2*axis);
	       get_position_corr(2*axis, &position);
	       semGive(semMEI);
	       pos = (long)position;
	    }
	    TRACE(8, "repositioning to %ld, status=%d", (long)pos, status);
	 }
	 TRACE(3, "Done repositioning %s to %ld", aname, (long)position);
      } else {
#if 0
	 printf("nonzero vel=%f\n", velocity);
#endif
      }
/*
 * check for time
 */
      while(frame != NULL &&
	    sdss_delta_time(frame->end_time, sdss_get_time()) < 0.0) {
	 frame = frame->nxt;
	 axis_queue[axis].active = frame;
	 TRACE(0, "%s frame deleted due to expiration time", aname, 0);
      }
      
      if(frame != NULL) {
	 start_frame(axis, frame->end_time);
	 while(frame->nxt == NULL &&
	       sdss_delta_time(frame->end_time, sdss_get_time()) > 0.02) {
	    TRACE(8, "%s waiting for second frame", aname, 0);
	    taskDelay (3);
	 }
#if 0					/* Charlie's version */
	 while ( (frame->nxt!=NULL) || (axis_queue[axis].active!=NULL) &&
		((!frame_break)&&(!drift_break)) ) ;
#else  /* re-written for clarity */
	 while(frame->nxt != NULL ||
	       (axis_queue[axis].active != NULL &&
					       !frame_break && !drift_break)) {

#endif
	    frame_cnt = get_frame_cnt(axis,frame);

	    TRACE(8, "%s frames_cnt=%d", aname, frame_cnt);
	    frame_idx = 0;
	    while(frame_cnt > 0) {
	       while((cnt = calc_frames(axis,frame,frame_idx)) == ERROR &&
		     (lcnt = tm_frames_to_execute(axis)) > 4) {
		  taskDelay(1);
	       }
	       
	       if(cnt == ERROR) {
		  frame_break[axis] = TRUE;
		  printf("frame=%p, nxt=%p, nxt=%p, frame_cnt=%d\n",
			 frame,frame->nxt,(frame->nxt)->nxt,frame_cnt);
	       } else {			/* OFFSET */
		  for(i = 0; i < OFF_MAX; i++) {
		     clroffset(axis,cnt);
		     if(offset_queue_end[axis][i] != NULL) {
			cntoff = calc_offset(axis, &offset[axis][i][0],
					     offset_idx[axis][i], cnt);
			offset_idx[axis][i] += cnt;
			
			if(offset_idx[axis][i]/20.0 >
			   offset[axis][i][1].end_time) {
			   TRACE(8, "%s shutdown offset", aname, 0);
			   frmoff=frame;
			   taskLock();
			   while(frmoff != offset_queue_end[axis][i]) {
			      frmoff->position += offset[axis][i][1].position;
			      frmoff->velocity += offset[axis][i][1].velocity;
			      frmoff = frmoff->nxt;
#if 0
			      printf("offset end=%p, pos=%f,idx=%d\n",
				     frmoff, frmoff->position,
				     offset_idx[axis][i]);
#endif
			   }
			   
			   frmoff->position += offset[axis][i][1].position;
			   frmoff->velocity += offset[axis][i][1].velocity;
#if 0
			   printf("offset end=%p, pos=%f,idx=%d\n",
				  frmoff, frmoff->position,
				  offset_idx[axis][i]);
#endif
			   offset_queue_end[axis][i] = NULL;
			   taskUnlock();
			}
		     }
		     addoffset(axis,cnt);
		  }
		  frame_idx += cnt;
		  frame_cnt -= cnt;
	       }
	       
	       if(frame_break[axis]) {
		  TRACE(8, "%s frame_break", aname, 0);
		  axis_queue[axis].active=NULL;
		  frame_cnt=0;
		  break;
	       }
	       
	       if(drift_break[axis]) {
		  TRACE(8, "%s drift_break", aname, 0);
		  axis_queue[axis].active=NULL;
		  frame_cnt=0;
		  break;
	       }
	       
	       idx = 0;
	       while(cnt > 0) {
		  if(frame_break[axis]) {
		  TRACE(8, "%s frame_break 2", aname, 0);
		     axis_queue[axis].active=NULL;
		     frame_cnt=0;
		     cnt=0;
		     break;
		  }
		  
		  if(drift_break[axis]) {
		     TRACE(8, "%s drift_break 2", aname, 0);
		     axis_queue[axis].active=NULL;
		     frame_cnt=0;
		     cnt=0;
		     break;
		  }
		  
		  if(cnt > 0) {
		     load_frames(axis, min(cnt,5), idx,
				 (double)ticks_per_degree[axis]);
		     
		     if(idx == 15 && cnt == 5) {
			TRACE(1, "Mystery message: p=%f", p[axis][19], 0);
		     }
		     
		     while((lcnt = tm_frames_to_execute(axis)) > 10) {
			taskDelay(3);
		     }
		     
		     idx += 5;
		     cnt -= 5;
		  }
	       }
	    }
	    
	    if(axis_queue[axis].active == NULL) {
	       frame=axis_queue[axis].end;
	       break;
	    }
	    
	    frame = frame->nxt;
	    axis_queue[axis].active = frame;
	    while(frame->nxt == NULL &&
		  sdss_delta_time(frame->end_time, sdss_get_time()) > 0.02) {
	       taskDelay (1);
	    }
	    
	    while(frame->nxt == NULL &&
		  (lcnt = tm_frames_to_execute(axis)) > 1) {
	       taskDelay(1);
	    }
	 }
	 
	 lcnt = tm_frames_to_execute(axis);
	 TRACE(3, "%s ran out: frames left=%d\n", aname, lcnt);
	 
	 taskLock();
	 axis_queue[axis].active = NULL;
	 frame = axis_queue[axis].end;
	 for(i = 0; i < OFF_MAX; i++) {
	    offset_idx[axis][i]=0;
	    offset_queue_end[axis][i]=NULL;
	 }
	 taskUnlock();
	 
	 if(idx <= 0) {
	    idx = 1;
	 }

	 if(frame_break[axis]) {
	    stp_frame(axis, stop_position[axis],
		      (double)ticks_per_degree[axis]);
	    frame_break[axis] = FALSE;
	 } else {
	    if(drift_break[axis]) {
	       drift_frame(axis, drift_velocity[axis],
			   (double)ticks_per_degree[axis]);
	    } else {
	       end_frame(axis, idx - 1, (double)ticks_per_degree[axis]);
	    }
	 }
      } else {
	 TRACE(3, "%s restart no frames to process", aname, 0);
      }
   }
}

#if 0
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_TCC_test
**	    start_tm_TCC_test
**
** DESCRIPTION:
**	The test task which calculates and loads frames as a diagnostic for
**	motion.  A task is spawned for each axis and runs independently.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**	calc_frames
**	load_frames_test
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void tm_TCC_test(int axis, struct FRAME *iframe, struct FRAME *fframe)
{
  int cnt;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;

  printf ("\r\n Axis=%d;  Ticks per degree=%f",axis,
        ticks_per_degree[axis]);
  CALC_verbose=TRUE;
    frame=iframe;
      while ((frame!=fframe)&&(frame->nxt!=NULL))
      {
        frame_cnt=get_frame_cnt(axis,frame);
        printf ("\r\n frames_cnt=%d",frame_cnt);
        frame_idx=0;
	frame_break[axis]=FALSE;
        for (i=0;i<((frame_cnt-1)/(LOAD_MAX-1))+1;i++)
        {
	  if (frame_break[axis]) break;
          cnt=calc_frames(axis,frame,frame_idx);
          frame_idx += cnt;
          printf ("\r\n cnt=%d, i=%d",cnt,i);
	  if (cnt>0)
            load_frames_test(axis,cnt,(double)ticks_per_degree[axis]);
        }
	frame_break[axis]=FALSE;
        frame = frame->nxt;
      }
      printf ("\r\n Ran out");

  CALC_verbose=FALSE;
}

void
start_tm_TCC_test(void)
{
  taskSpawn("tmAztest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlttest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRottest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		2,0,0,0,0,0,0,0,0,0);
}
#endif

/*=========================================================================
**=========================================================================
**
**	Diagnostic for display the last 100 pvts for a specified axis.
**
** GLOBALS REFERENCED:
**	axis_queue
**
**=========================================================================
*/
void
print_axis_queue(int axis)
{
   struct FRAME *frame;
   struct FRAME_QUEUE *queue;
   
   printf("List Axis Queue=%d: %p\n",axis,&axis_queue[axis]);
   queue = &axis_queue[axis];
   for(frame = (struct FRAME *)queue->top; frame != NULL; frame = frame->nxt) {
      if(frame == queue->top) printf("TOP, cnt=%d\n", queue->cnt);
      if(frame == queue->active) printf("ACTIVE\n");
      if(frame == queue->end) printf ("END\n");
      
      printf("%p: position=%12.8f, velocity=%12.8f, end_time=%12.8f\n",
	     frame, frame->position, frame->velocity, frame->end_time);
      
      if(frame->nxt != NULL) {
	 printf ("      "
		 "deltas position=%12.8f, velocity=%12.8f, end_time=%12.8f\n",
		 frame->nxt->position-frame->position,
		 frame->nxt->velocity-frame->velocity,
		 frame->nxt->end_time-frame->end_time);
      }
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ip_shutdown
**
** DESCRIPTION:
**	Shutdown all ip hardware with a reset to the ip slots for the
**	MVME162 and the SYSTRAN carrier.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
ip_shutdown(int type)
{
    char *ip;
/* reset the systran carrier */
    ip=(char *)0xffff4501;
    printf("IP shutdown: reset SYSTRAN carrier ip @%p\r\n",ip);
    *ip=0x1F;
    taskDelay(20);
    *ip=0x00;
/* reset MVME162 board */
    ip=(char *)0xfffbc01f;
    printf("IP shutdown: reset MVME162 ip @%p\r\n",ip);
    *ip=0x1;
    taskDelay(20);
    *ip=0x00;
    taskDelay(30);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: amp_reset
**
** DESCRIPTION:
**	Resets the amps; 2 Az, 2 Alt, and 1 Rot.  Pulses the corresponding
**	output bit.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
void
amp_reset(int mei_axis)
{
   TRACE(3, "Resetting amp for axis %s: %d", axis_name(mei_axis/2), mei_axis);
   
   DIO316_Write_Port(cw_DIO316, AMP_RESET, 1<<mei_axis);
   taskDelay (2);
   DIO316_Write_Port(cw_DIO316, AMP_RESET, 0);
}

#if 0
/*=========================================================================
**=========================================================================
**
** ROUTINE: save_firmware
**	    restore_firmware
**
** DESCRIPTION:
**	Save/restore of MEI firmware...primarily for upgrades which was
**	not part of the VME package.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
save_firmware(void)
{
  upload_firmware_file ("vx_mei_firmware.bin");
}

void
restore_firmware(void)
{
  download_firmware_file ("vx_mei_firmware.bin");
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: print_max
**
** DESCRIPTION:
**	Diagnostic routines to monitor the max vel and accel.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
print_max(void)
{
   int i;
   
   for(i = 0; i < NAXIS; i++) {
      printf("%s: MAX ACC limit %f deg/sec^2 maximum requested %f\n",
	     axis_name(i), max_acceleration[i], max_acceleration_requested[i]);
   }
   for(i = 0; i < NAXIS; i++) {
      printf("%s: MAX VEL limit %f deg/sec maximum requested %f\n",
	     axis_name(i), max_velocity[i], max_velocity_requested[i]);
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: diagq_setup
**	    diagq_print
**
** DESCRIPTION:
**	Diagnostic routines to monitor frames loaded into the MEI.
**
** RETURN VALUES:
**	return 0
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	diagq
**	diagq_i
**	diagq_siz
**
**=========================================================================
*/
void
diagq_setup(int ks)
{
  diagq_siz = ks*1024;
  diagq = malloc(diagq_siz*sizeof(struct DIAG_Q));
  assert(diagq != NULL);
  diagq_i = 0;
}

void
print_diagq(void)
{
   int i;
   
   printf("i=%d\n",diagq_i);
   for(i = 0; i < diagq_siz; i++) {
      printf("%c%d: p=%f tim=%f v=%f a=%f ji=%f",
	     ((i == diagq_i) ? '*' : ' '),
	     i, diagq[i].p,diagq[i].tim, diagq[i].v,diagq[i].a, diagq[i].ji);
   }
}

/*****************************************************************************/
/*
 * Functions called by Menu() to do various things; also accessible
 * via the serial or tcp interfaces
 */
/*
 * Set whether an axis is monitored
 */
int
mcp_set_monitor(int axis,		/* which axis? */
		int on_off)		/* set monitor on/off/toggle */
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_set_monitor: illegal axis %d", axis, 0);

      return(-1);
   }

   if(on_off == -1) {
      on_off = !monitor_on[axis];
   }

   monitor_on[axis] = on_off ? 1 : 0;

   return(0);
}

/*
 * Set the position of an axis
 */
int
mcp_set_pos(int axis,			/* the axis to set */
	    double pos)			/* the position to go to */
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_set_pos: illegal axis %d", axis, 0);

      return(-1);
   }

   tm_set_position(2*axis, pos);
   tm_set_position(2*axis + 1, pos);
   fiducial[axis].seen_index = FALSE;
   
   return 0;
}

/*
 * move to a position with specified velocity and acceleration
 */
int
mcp_move_va(int axis,			/* the axis to move */
	    long pos,			/* desired position */
	    long vel,			/* desired velocity */
	    long acc)			/* desired acceleration */
{
   int ret;				/* return code from MEI */
   
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_move_va: illegal axis %d", axis, 0);

      return(-1);
   }

   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "mcp_move_va: failed to take semMEI: %s (%d)",
	    strerror(errno), errno);

      return(-1);
   }
   
   sem_controller_run(2*axis);
   
#if SWITCH_PID_COEFFS
   if(axis == INSTRUMENT) {
      while(coeffs_state_cts(2*axis, vel) == TRUE) {
	 continue;
      }	     
   }
#endif
   
   if((ret = start_move_corr(2*axis, pos, vel, acc)) != DSP_OK) {
      TRACE(0, "start_move failed for %s : %d", axis_name(axis), ret);
      if(ret == DSP_NO_DISTANCE) {
	 double mei_pos;
	 get_position_corr(2*axis, &mei_pos);
	 TRACE(3, "positions: MCP: %ld MEI: %ld", (long)pos, (long)mei_pos);
      }
   }
   semGive(semMEI);
   
   return(0);
}

/*
 * set a velocity
 */
int
mcp_set_vel(int axis,			/* the axis to set */
	    double vel)			/* the position to go to */
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_set_vel: illegal axis %d", axis, 0);

      return(-1);
   }

   if(axis == AZIMUTH && sdssdc.status.i9.il0.az_brake_en_stat) {
      TRACE(0, "mcp_set_vel: AZ Brake is Engaged", 0, 0);
      return(-1);
   } else if(axis == ALTITUDE && sdssdc.status.i9.il0.alt_brake_en_stat) {
      TRACE(0, "mcp_set_vel: ALT Brake is Engaged", 0, 0);
      return(-1);
   }

   if(semTake(semMEI, 60) == ERROR) {
      TRACE(0, "mcp_set_val: failed to take semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }
	 
#if SWITCH_PID_COEFFS
   if(axis == INSTRUMENT) {
      while(coeffs_state_cts(2*axis, vel) == TRUE) {
	 ;
      }
   }
#endif
   set_velocity (2*axis, vel);
   semGive (semMEI); 

   return 0;
}

/*****************************************************************************/
/*
 * Put on a brake
 */
int
mcp_set_brake(int axis)
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_set_brake: illegal axis %d", axis, 0);

      return(-1);
   }

   TRACE(3, "Setting brake for axis %s", axis_name(axis), 0);

   if(axis == AZIMUTH) {
      tm_sp_az_brake_on();
   } else if(axis == ALTITUDE) {
      tm_sp_alt_brake_on();
   }
   
   tm_sem_controller_idle(2*axis);
   tm_reset_integrator(2*axis);

   return(0);
}

/*
 * Clear a brake
 */
int
mcp_unset_brake(int axis)		/* axis to set */
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_unset_brake: illegal axis %d", axis, 0);

      return(-1);
   }

   TRACE(3, "Clearing brake for axis %s", axis_name(axis), 0);

   if(axis == AZIMUTH) {
      tm_sp_az_brake_off();
   } else if(axis == ALTITUDE) {
      tm_sp_alt_brake_off();
   }
   
   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "mcp_unset_brake: failed to take semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }

   TRACE(3, "Putting axis %s into closed loop", axis_name(axis), 0);
   sem_controller_run(2*axis);

#if SWITCH_PID_COEFFS
   if(axis == INSTRUMENT) {
      while(coeffs_state_cts(2*axis, 0) == TRUE) {
	 ;
      }
   }
#endif

   TRACE(3, "Stopping axis %s", axis_name(axis), 0);
   v_move(2*axis,(double)0,(double)5000);

   semGive(semMEI);

   return(0);
}

/*****************************************************************************/
/*
 * Hold an axis
 */
int
mcp_hold(int axis)			/* desired axis */
{
   double vel;				/* velocity of axis */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_halt: illegal axis %d", axis, 0);

      return(-1);
   }

   TRACE(3, "Holding axis %s", axis_name(axis), 0);

   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "mcp_hold: %s: couldn't take semMEI: %d",
	    axis_name(axis), errno);
      return(-1);
   }
   
   if(get_velocity(2*axis, &vel) != DSP_OK) {
      TRACE(0, "Holding axis %s: failed to read velocity",
	    axis_name(axis), errno);
      vel = 0;
   }
   
#if 1
   set_stop(2*axis);
   {
      int i, ntry = 120*60;		/* two minutes */

      for(i = 0; i < ntry; i++) {
	 if(motion_done(2*axis)) {
	    break;
	 }
	 taskDelay(1);
      }
      if(i == ntry) {
	 TRACE(0, "TELL RHL. Failed to get motion_done() for %s: in_seq = %d",
	       axis_name(axis), in_sequence(2*axis));
	 TRACE(0, "           in_mot = %d, frames = %d",
	       in_motion(2*axis), frames_left(2*axis));
      }
   }
   clear_status(2*axis);
   
   if(dsp_error != DSP_OK) {
      TRACE(0, "failed to stop %s : %d", axis_name(axis_select), dsp_error);
   }

   TRACE(5, "Holding axis %s: back into closed loop", axis_name(axis), 0);
   sem_controller_run(2*axis);		/* back into closed loop */
#else
   TRACE(5, "Holding axis %s: back into closed loop", axis_name(axis), 0);
   sem_controller_run(2*axis);		/* back into closed loop */

   while(fabs(vel) > 5000) {
      vel -= (vel > 0) ? 5000 : -5000;
      TRACE(5, "Velocity = %g", vel, 0);
      set_velocity(2*axis, vel);
      
#if SWITCH_PID_COEFFS
      if(axis == INSTRUMENT) {
	 while(coeffs_state_cts(2*axis, vel) == TRUE) {
	    ;
	 }
      }
#endif
      
      taskDelay (15);
   }
   
   vel = 0;
   set_velocity(2*axis, vel);

   {
      double pos;
      get_position_corr(2*axis, &pos);
      set_position_corr(2*axis, pos);
   }
#if SWITCH_PID_COEFFS
   while(coeffs_state_cts(2*axis, vel) == TRUE) {
      ;
   }
#endif
   TRACE(5, "Velocity = %g", vel, 0);
#endif
   
   semGive (semMEI);

   return(0);
}

/*****************************************************************************/
/*
 * Reset amplifiers for an axis
 */
int
mcp_amp_reset(int axis)
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_amp_reset: illegal axis %d", axis, 0);

      return(-1);
   }

   if(axis == INSTRUMENT) {
      amp_reset(2*axis);
   } else {
      amp_reset(2*axis);
      amp_reset(2*axis + 1);
   }

   return(0);
}

/*****************************************************************************/
/*
 * Stop an axis
 */
int
mcp_stop_axis(int axis)
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "mcp_stop_axis: illegal axis %d", axis, 0);

      return(-1);
   }

   TRACE(3, "Stopping axis %s", axis_name(axis), 0);

   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "Stopping axis %s: couldn't take semMEI: %d",
	    axis_name(axis), errno);
      return(-1);
   }

   sem_controller_idle(2*axis);
   reset_integrator(2*axis);
   
   semGive (semMEI);

   return(0);
}

/*****************************************************************************/
/*
 * Wrappers for MCP commands.
 *
 * These aren't declared static so that cmdList can find them
 */
char *
amp_reset_cmd(char *cmd)		/* NOTUSED */
{
   mcp_amp_reset(axis_select);

   return("");
}

char *
hold_cmd(char *cmd)			/* NOTUSED */

{
   mcp_hold(axis_select);

   return("");
}

char *
stop_cmd(char *cmd)			/* NOTUSED */
{
   mcp_stop_axis(axis_select);

   return("");
}

char *
set_monitor_cmd(char *cmd)			/* NOTUSED */
{
   int on_off;

   if(sscanf(cmd, "%d", &on_off) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_monitor(axis_select, on_off);

   return("");
}

char *
set_pos_cmd(char *cmd)
{
   double pos;

   if(sscanf(cmd, "%lf", &pos) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_pos(axis_select, pos);

   return("");
}

char *
goto_pos_va_cmd(char *cmd)
{
   double pos, vel, acc;

   if(sscanf(cmd, "%lf %lf %lf", &pos, &vel, &acc) != 3) {
      return("ERR: malformed command argument");
   }
   
   mcp_move_va(axis_select, pos, vel, acc);

   return("");
}

char *
set_vel_cmd(char *cmd)
{
   double pos;

   if(sscanf(cmd, "%lf", &pos) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_vel(axis_select, pos);

   return("");
}

/*****************************************************************************/
/*
 * Restore positions from shared memory after a boot
 */
void
restore_pos(void)
{
   struct SDSS_FRAME *save;
   struct TM_M68K *restore;
   int i;
   
   save = (struct SDSS_FRAME *)(SHARE_MEMORY + 2);
   for(i = 0; i < NAXIS; i++) {
      restore = (struct TM_M68K *)&save->axis[i];
      
      fprintf(stderr, "Restoring encode position for %s: %d\n",
	      axis_name(i), restore->actual_position);
      TRACE(3, "Restoring encode position for %s: %d",
	    axis_name(i), restore->actual_position);
      
      tm_set_position(2*i, restore->actual_position);
      tm_set_position(2*i + 1,restore->actual_position2);
   }
}

/*****************************************************************************/
/*
 * General initialization of MEI board...called from startup script.
 *
 * 
 * Initialize queue data structures.
 *
 * Setup semaphores
 *
 * Declare commands to the command interpreter
 */
int
axisMotionInit(void)
{
   short action;
   char buffer[MAX_ERROR_LEN] ;
   short coeff[COEFFICIENTS];		/* coefficients for PID loops */
   int err;
   int i;
   double limit;
   int mei_axis;			/* counter for all 2*NAXIS "axes" */
   double rate;
/*
 * Create semaphores
 */
   if(semMEI == NULL) {
      semMEI = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
      assert(semMEI != 0);
   }
   if(semSLC == NULL) {
      semSLC = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
      assert(semSLC != NULL);
   }
/*
 * We don't really need to take semMEI here as we're the only process
 * running, but it's clearer if we do
 */
   err = semTake(semMEI, WAIT_FOREVER);
   assert(err != ERROR);

   for(i = 0; i < 3; i++) {
      axis_queue[i].top = (struct FRAME *)malloc(sizeof(struct FRAME));
      if(axis_queue[i].top == NULL) {
	 TRACE(0, "axisMotionInit: no memory for queue", 0, 0);
	 return(ERROR);
      }
      
      axis_queue[i].end = axis_queue[i].top;
      axis_queue[i].active = NULL;
      axis_queue[i].cnt = 1;
      axis_queue[i].top->nxt = NULL;
      axis_queue[i].top->position = 0;
      axis_queue[i].top->velocity = 0;
      axis_queue[i].top->end_time = 0;
   }
   sdss_was_init = TRUE;

   err = dsp_init(DSP_IO_BASE);
   if(err) {
      error_msg(err, buffer) ;	/* convert an error code to a human message */
      TRACE(0, "dsp_init failed--%s (%d)", buffer, err);
#if 0
      return(ERROR);
#endif
   } else {
      printf("dsp_init Passed!\n");
   }
   
   err = dsp_reset();
   if(err) {
      error_msg(err, buffer) ;	/* convert an error code to a human message */
      TRACE(0, "dsp_reset failed--%s (%d)", buffer, err);
#if 0
      return(ERROR);
#endif
   } else {
      printf("dsp_reset Passed!\n");
   }
   
   set_sample_rate(160);
   TRACE(3, "Sample Rate=%d", dsp_sample_rate(), 0);
   
   for(mei_axis = 0; mei_axis < dsp_axes(); mei_axis++) {
      TRACE(3, "Initialising AXIS %d", mei_axis, 0);

      get_stop_rate(mei_axis, &rate);
      TRACE(3, "old stop rate = %f", rate, 0);
      set_stop_rate(mei_axis, (double)SDSS_STOP_RATE);
      get_stop_rate(mei_axis, &rate);
      TRACE(3, "set stop rate = %f", rate, 0);
      
      get_e_stop_rate(mei_axis, &rate);
      TRACE(3, "old e_stop rate=%f", rate, 0);
      set_e_stop_rate(mei_axis, (double)SDSS_E_STOP_RATE);
      get_e_stop_rate(mei_axis, &rate);
      TRACE(3, "set e_stop rate=%f", rate, 0);
      
      get_error_limit(mei_axis, &limit, &action);
      TRACE(3, "old error limit=%ld, action=%d", (long)limit, action);
      set_error_limit(mei_axis, 24000, ABORT_EVENT);
      get_error_limit(mei_axis, &limit, &action);
      TRACE(3, "set error limit=%ld, action=%d", (long)limit, action);

      set_integration(mei_axis, IM_ALWAYS);
   }

   restore_pos();			/* restore axis positions */

   semGive(semMEI);

   VME2_pre_scaler(0xE0);  /* 256-freq, defaults to 33 MHz, but sys is 32MHz */
   init_io(2,IO_INPUT);
/*
 * Set filter coefficients for the axes.  Note that it is essential that we
 * provide values for all of the coefficients.
 */
   semTake(semMEI,WAIT_FOREVER);

   coeff[DF_P] = 160;
   coeff[DF_I] = 6;
   coeff[DF_D] = 1500;
   coeff[DF_ACCEL_FF] = 0;
   coeff[DF_VEL_FF] = 0;
   coeff[DF_I_LIMIT] = 32767;
   coeff[DF_OFFSET] = 0;
   coeff[DF_DAC_LIMIT] = 18000;
   coeff[DF_SHIFT] = -4;		/* 1/16 */
   coeff[DF_FRICT_FF] = 0;
   set_filter(2*AZIMUTH, (P_INT)coeff);

   coeff[DF_P] = 120;
   coeff[DF_I] = 6;
   coeff[DF_D] = 1200;
   coeff[DF_ACCEL_FF] = 0;
   coeff[DF_VEL_FF] = 0;
   coeff[DF_I_LIMIT] = 32767;
   coeff[DF_OFFSET] = 0;
   coeff[DF_DAC_LIMIT] = 10000;
   coeff[DF_SHIFT] = -4;		/* 1/16 */
   coeff[DF_FRICT_FF] = 0;
   set_filter(2*ALTITUDE, (P_INT)coeff);
   
   coeff[DF_P] = 120;
   coeff[DF_I] = 12;
   coeff[DF_D] = 600;
   coeff[DF_ACCEL_FF] = 0;
   coeff[DF_VEL_FF] = 0;
   coeff[DF_I_LIMIT] = 32767;
   coeff[DF_OFFSET] = 0;
   coeff[DF_DAC_LIMIT] = 12000;
   coeff[DF_SHIFT] = -5;		/* 1/32 */
   coeff[DF_FRICT_FF] = 0;
   set_filter(2*INSTRUMENT, (P_INT)coeff);

   semGive(semMEI);
/*
 * Spawn the tasks that run the axes
 */
   taskSpawn("tmAz",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, AZIMUTH,
	     0,0,0,0,0,0,0,0,0);
   taskSpawn("tmAlt",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, ALTITUDE,
	     0,0,0,0,0,0,0,0,0);
   taskSpawn("tmRot",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, INSTRUMENT,
	     0,0,0,0,0,0,0,0,0);
/*
 * Declare commands
 */
   define_cmd("+MOVE",         plus_move_cmd, 	 -1, 1, 1);
   define_cmd("AB.STATUS",     abstatus_cmd, 	  2, 0, 1);
   define_cmd("AMP.RESET",     amp_reset_cmd, 	  0, 1, 1);
   define_cmd("AXIS.STATUS",   axis_status_cmd,   0, 0, 0);
   define_cmd("BRAKE.OFF",     brakeoff_cmd, 	  0, 1, 1);
   define_cmd("BRAKE.ON",      brakeon_cmd, 	  0, 1, 1);
   define_cmd("DRIFT",         drift_cmd, 	  0, 1, 1);
   define_cmd("GOTO.POS.VA",   goto_pos_va_cmd,   3, 1, 1);
   define_cmd("HALT",          hold_cmd, 	  0, 1, 1);
   define_cmd("HOLD",          hold_cmd, 	  0, 1, 1);
   define_cmd("ID",            id_cmd, 		  0, 0, 1);
   define_cmd("INIT",          init_cmd, 	  0, 1, 1);
   define_cmd("ROT",           rot_cmd, 	  0, 0, 0);
   define_cmd("IR",            rot_cmd, 	  0, 0, 0);
   define_cmd("MAXACC",        maxacc_cmd, 	  1, 1, 1);
   define_cmd("MAXVEL",        maxvel_cmd, 	  1, 1, 1);
   define_cmd("MC.DUMP",       mc_dump_cmd, 	  0, 0, 1);
   define_cmd("MC.MAXACC",     mc_maxacc_cmd,     0, 0, 1);
   define_cmd("MC.MAXVEL",     mc_maxvel_cmd,     0, 0, 1);
   define_cmd("MOVE",          move_cmd, 	 -1, 1, 1);
   define_cmd("SET.MONITOR",   set_monitor_cmd,   1, 1, 1);
   define_cmd("SET.POS.VA",    goto_pos_va_cmd,   3, 1, 1);
   define_cmd("SET.POSITION",  set_pos_cmd, 	  1, 1, 1);
   define_cmd("SET.VELOCITY",  set_vel_cmd, 	  1, 1, 1);
   define_cmd("STATS",         stats_cmd, 	  0, 0, 1);
   define_cmd("STATUS",        status_cmd, 	  0, 0, 1);
   define_cmd("STATUS.LONG",   status_long_cmd,   0, 0, 1);
   define_cmd("STOP",          stop_cmd, 	  0, 1, 1);
   define_cmd("SYSTEM.STATUS", system_status_cmd, 0, 0, 0);
   define_cmd("AZ",            tel1_cmd,          0, 0, 0);
   define_cmd("TEL1",          tel1_cmd,          0, 0, 0);
   define_cmd("ALT",           tel2_cmd,          0, 0, 0);
   define_cmd("TEL2",          tel2_cmd,          0, 0, 0);
   
   return 0;
}
