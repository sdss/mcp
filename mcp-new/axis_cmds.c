#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      axis_cmds.c
**
** ABSTRACT:
**	Action routines for commands from the TCC which involve setting up
**	queues for each axis to execute motion.
**	tm_TCC is spawned for each axis to excute motion
**	Fiducial routines for resetting position based on known points.
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
**	tm_TCC		task	tmAz, tmAlt, tmRot
**	tm_latch	task	tmLatch
**	DIO316_interrupt int	reference crossing interrupts
**	DID48_interrupt	int	1 Hz Timer for SDSStime
**
** ENVIRONMENT:
**      ANSI C.
**
** REQUIRED PRODUCTS:
**
** AUTHORS:
**      Creation date:  Aug 30, 1999
**      Charlie Briegel
**
***************************************************************************
***************************************************************************/
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	axis_cmds.c						*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, briegel@fnal.gov	*/
/*   Program:	axis_cmds : VxWorks					*/
/*   Modules:								*/
/*									*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
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
#include "nfsDrv.h"
#include "nfsLib.h"
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
#include "dio316dr.h"
#include "dio316lb.h"
#include "did48lb.h"
#include "mv162IndPackInit.h"
#include "logLib.h"
#include "data_collection.h"
#include "io.h"
#include "math.h"
#include "tm.h"
#include "axis.h"
#include "cw.h"
/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
#define	DSP_IO_BASE		(0x300)			/* in A16/D16 space. */
/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/
#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONE_DAY	86400
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
struct LATCH_POS
{
	int axis;
	int ref;
	int data;
	double pos1;
	double pos2;
};
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
FILE *fidfp=NULL;
int latchidx=0;
int LATCH_error=FALSE;
int LATCH_verbose=FALSE;
#define MAX_LATCHED	2000
struct LATCH_POS latchpos[MAX_LATCHED];
int errmsg_max[3]={400,200,100}; /* axis fiducial max error to post msg */
SEM_ID semMEI=NULL;
SEM_ID semSLC=NULL;
SEM_ID semLATCH=NULL;
int tm_DIO316,tm_DID48;
int axis_select=-1;		/* 0=AZ,1=ALT,2=ROT -1=ERROR  */
int spectograph_select=-1;	/* 0=SP1, 1=SP2, -1=ERROR  */
int MEI_interrupt=FALSE;
int DIO316_Init=FALSE;
int DID48_Init=FALSE;
int sdss_was_init=FALSE;
struct FRAME_QUEUE axis_queue[]={
		{0,NULLPTR,NULLPTR},
		{0,NULLPTR,NULLPTR},
		{0,NULLPTR,NULLPTR}
};
double max_velocity[]={2.25,1.75,2.25,0,0,0};
double max_acceleration[]={4.,4.,6.0,0,0,0};
double max_position[]={360.,92.,320.0,0,0,0};
double min_position[]={-360.,0.,-180.0,0,0,0};
float time1[3],time2[3];
int CALC_verbose=FALSE;
int CALCOFF_verbose=FALSE;
int CALCADDOFF_verbose=FALSE;
int CALCFINAL_verbose=FALSE;
int FRAME_verbose=FALSE;
struct DIAG_Q *diagq=NULL;
int diagq_siz,diagq_i;
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
double sec_per_tick[]={AZ_TICK,ALT_TICK,ROT_TICK};
double ticks_per_degree[]={AZ_TICKS_DEG,
				ALT_TICKS_DEG,
				ROT_TICKS_DEG};
/*------------*/
/* Prototypes */
/*------------*/
void axis_DIO316_shutdown(int type);
void DIO316_interrupt(int type);
int DIO316_initialize(unsigned char *addr, unsigned short vecnum);
void axis_DID48_shutdown(int type);
void DID48_interrupt(int type);
int DID48_initialize(unsigned char *addr, unsigned short vecnum);
void save_firmware();
void restore_firmware();
int sdss_init();
void stop_frame(int axis,double pos,double sf);
void stp_frame(int axis,double pos,double sf);
void end_frame(int axis,int index,double sf);
void start_tm_TCC();
void start_tm_TCC_test();
void amp_reset(int axis);
void latchstart ();
void latchprint (char *description);
void latchexcel (int axis);
void set_primary_fiducials (int axis,int fididx,long pos);
void set_fiducials (int axis);
void save_fiducials (int axis);
void set_fiducials_all ();
void save_fiducials_all ();
void restore_fiducials (int axis);
void restore_fiducials_all ();
void print_max ();
double sdss_delta_time(double t2, double t1);
void DIO316ClearISR_delay (int delay, int bit);
int tm_frames_to_execute(int axis);

/*=========================================================================
**=========================================================================
**
** ROUTINE: reboot_cmd
**
** DESCRIPTION:
**      <break> -> Software reboot
**
** RETURN VALUES:
**      NULL string
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *reboot_cmd(char *cmd)
{
  printf (" <break> command fired\r\n");
  reboot(BOOT_NORMAL);
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: correct_cmd
**
** DESCRIPTION:
**      CORRECT -> set the fiducial
**	This will set the fiducial to the nearest valid fiducial crossed (in the
**	future), but currently there is one "trusted" fiducial per axis.  This
**	fiducial, if valid, will adjust the axis position appropriately.
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**	tm_set_position
**
** GLOBALS REFERENCED:
**	fiducial
**	fiducial_position
**	semMEI
**	axis_select
**
**=========================================================================
*/
char *correct_cmd(char *cmd)
{
  extern SEM_ID semMEI;
  extern struct FIDUCIARY fiducial[3];
  extern long fiducial_position[3];
  long pos;
  double position;
  time_t fidtim;

  printf (" CORRECT command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (fiducial[axis_select].markvalid)
  {
    if (semTake (semMEI,60)!=ERROR)
    {
      get_position(axis_select<<1,&position);
      semGive (semMEI);
/*      pos=fiducial_position[axis_select];*/
      pos=position;
/* use optical encoder for axis 4 */
      switch (axis_select)
      {
        case ALTITUDE:
          pos += (fiducial_position[axis_select]-fiducial[axis_select].mark);
/*          pos += ((long)position-fiducial[axis_select].mark);*/
          tm_set_pos((axis_select*2),pos);
/* connection of the second encoder is at best a guess/wish/dream */
          tm_set_pos((axis_select*2)+1,pos);	
	  break;

        case AZIMUTH:
          pos += (fiducial_position[axis_select]-fiducial[axis_select].mark);
/*          pos += ((long)position-fiducial[axis_select].mark);*/
          tm_set_pos((axis_select*2),pos);
/* connection of the second encoder is at best a guess/wish/dream */
          tm_set_pos((axis_select*2)+1,pos);
	  break;

        case INSTRUMENT:
          pos += (fiducial_position[axis_select]-fiducial[axis_select].mark);
/*          pos += ((long)position-fiducial[axis_select].mark);*/
          tm_set_pos((axis_select*2),pos);
          tm_set_pos((axis_select*2)+1,pos);  /* connected reliably */
	  break;
      }
      if (fidfp!=NULL)
      {
	time (&fidtim);
        fprintf (fidfp,"\nCORRECT %d\t%d\t%.24s:%f\t%ld\t%ld",
	          axis_select,fiducial[axis_select].index,
	          ctime(&fidtim),sdss_get_time(),
	          (long)fiducial_position[axis_select]-fiducial[axis_select].mark,
                  (long)position);
/*        fidfp=freopen (bldFileName(name),"a",fidfp);*/
      }
      fiducial[axis_select].mark=fiducial_position[axis_select];
      return "";
    }
    else
      return "ERR: semMEI";
  }
  else
    return "ERR: fiducial for axis not crossed";
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
static char *drift_ans={"360.00000 0.500000 5040.00000                                "};
char *drift_cmd(char *cmd)
{
  double position;
  double arcdeg, veldeg;
  float time;

/*  printf (" DRIFT command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (semTake (semMEI,60)!=ERROR)
  {
    get_velocity(axis_select<<1,&drift_velocity[axis_select]);
    semGive (semMEI);
    drift_break[axis_select]=TRUE;
  }
  else
    return "ERR: semMEI";
  taskDelay(3);
  if (semTake (semMEI,60)!=ERROR)
  {
    taskLock();	/* enforce coincidental data */
    get_position(axis_select<<1,&position);
    time=sdss_get_time();
    taskUnlock();
    semGive (semMEI);
    if (time<0) return "ERR: BAD TIME";
  }
  else
    return "ERR: semMEI";
  veldeg=(sec_per_tick[axis_select]*drift_velocity[axis_select])/3600.;
  arcdeg=(sec_per_tick[axis_select]*position)/3600.;
  sprintf (drift_ans,"%lf %lf %f",arcdeg,veldeg,time);
  return drift_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: id_cmd
**
** DESCRIPTION:
**      ID -> MEI firmware
**
** RETURN VALUES:
**      "axis date firmware"
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
static char *id_ans={"0 None Specified MMM DD 19YY\r\nDSP Firmware=Vxxx.xx Rxx Sx, Option=xxxx Axes=x"};
char *id_cmd(char *cmd)
{
  static char *axis_name[]={"None Specified","Azimuth","Altitude","Rotator"
		};

/*  printf (" ID command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (id_ans,"%d %s %s\r\n%s\r\nDSP Firmware=V%f R%d S%d, Option=%d, Axes=%d",
		axis_select,axis_name[axis_select+1],__DATE__,
		"$Name$",
		dsp_version()/1600.,dsp_version()&0xF,(dsp_option()>>12)&0x7,
		dsp_option()&0xFFF,dsp_axes());
  return id_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: init_cmd
**
** DESCRIPTION:
**      INIT -> Init the axis
**
** RETURN VALUES:
**      NULL string or "ERR:..."
**
** CALLS TO:
**	tm_axis_state
**	tm_controller_idle
**	tm_reset_integrator
**	amp_reset
**      tm_sp_az_brake_off
**      tm_sp_alt_brake_off
**	tm_controller_run
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
char *init_cmd(char *cmd)
{
  int state;
  int retry;
  extern struct SDSS_FRAME sdssdc;

/*  printf (" INIT command fired axis=%d\r\n",axis_select);*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  state=tm_axis_state(axis_select<<1);

/*  I don't really agree with this, JG directed...I think the axis should
    remain in closed loop if in close loop */
  if (state>2)		/* normal...NOEVENT,running, or NEW_FRAME */
  {
    printf ("\r\n  INIT axis %d: not running, state=%x",axis_select<<1,state);
    tm_controller_idle (axis_select<<1);
  }
  tm_reset_integrator(axis_select<<1);
  drift_break[axis_select]=FALSE;
  frame_break[axis_select]=FALSE;
  switch (axis_select)
  {
    case AZIMUTH:
      amp_reset(axis_select<<1);	/* two amplifiers, */
      amp_reset((axis_select<<1)+1);	/* param is index to bit field */
      if (sdssdc.status.i9.il0.az_brake_en_stat)
      {
        printf("Azimuth Brake Turned Off                ");
        tm_sp_az_brake_off();
      }
      break;

    case ALTITUDE:
      amp_reset(axis_select<<1);	/* two amplifiers */
      amp_reset((axis_select<<1)+1);	/* param is index to bit field */
      if (sdssdc.status.i9.il0.alt_brake_en_stat)
      {
        printf("Altitude Brake Turned Off               ");
        tm_sp_alt_brake_off();           
      }
      break;

    case INSTRUMENT:
      amp_reset(axis_select<<1);	/* one amplifiers */
      break;				/* param is index to bit field */
  }
/*  tm_axis_state retries as well...so this is really redundant, but then the
    brakes don't come off real quick so this will assure closed loop for
    at least sometime before continuing */
  retry=4;
  while ((tm_axis_state(axis_select<<1)>2)&&((retry--)>0))
  {
    tm_controller_run (axis_select<<1);
    taskDelay(20);
  }
/* zero the velocity */
  if (semTake (semMEI,60)!=ERROR)  
  {
    v_move(axis_select<<1,(double)0,(double)5000);
    semGive (semMEI);
  }
/* reinitialize the queue of pvts to none */
  axis_queue[axis_select].active=axis_queue[axis_select].end;
  axis_queue[axis_select].active=NULL;
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
char *maxacc_cmd(char *cmd)
{
  int cnt;

  printf (" MAXACC command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  cnt=sscanf (cmd,"%12lf",&max_acceleration[axis_select]);
  if (cnt!=1) printf ("ERR: no acceleration specified");
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
char *maxvel_cmd(char *cmd)
{
  int cnt;

  printf (" MAXVEL command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  cnt=sscanf (cmd,"%12lf",&max_velocity[axis_select]);
  if (cnt!=1) printf ("ERR: no velocity specified");
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
static char *max_ans={"F@ F.             "};
char *mc_maxacc_cmd(char *cmd)
{
  printf (" MC.MAX.ACC command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (max_ans,"F@ F. %12lf",&max_acceleration[axis_select]);
  return max_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mc_maxpos_cmd
**
** DESCRIPTION:
**      MC.MAX.POS -> Display the maximum position.
**
** RETURN VALUES:
**      "F@ F. pos" or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	max_position
**	axis_select
**
**=========================================================================
*/
char *mc_maxpos_cmd(char *cmd)
{
  printf (" MC.MAX.POS command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (max_ans,"F@ F. %12lf",&max_position[axis_select]);
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
char *mc_maxvel_cmd(char *cmd)
{
  printf (" MC.MAX.VEL command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (max_ans,"F@ F. %12lf",&max_velocity[axis_select]);
  return max_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mc_minpos_cmd
**
** DESCRIPTION:
**      MC.MIN.POS -> Display the minimum position.
**
** RETURN VALUES:
**      "F@ F. pos" or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	min_position
**	axis_select
**
**=========================================================================
*/
char *mc_minpos_cmd(char *cmd)
{
  printf (" MC.MIN.POS command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (max_ans,"F@ F. %12lf",&min_position[axis_select]);
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
**      tm_get_pos
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
char *move_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;

  double position,velocity,pos;
  struct FRAME *frame,*nxtque;
  struct FRAME_QUEUE *queue;
  int i;
  int cnt;
  double dt;

/*  printf (" MOVE command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (sdss_get_time()<0) return "ERR: BAD TIME";
  queue = &axis_queue[axis_select];
  frame = (struct FRAME *)malloc (sizeof(struct FRAME));
  if (frame==NULL) return "ERR: OUT OF MEMORY";
  cnt=sscanf (cmd,"%12lf %12lf %12lf",&position,&velocity,&frame->end_time);
  switch (cnt)
  {
    case -1:
    case 0:
        tm_get_pos(axis_select<<1,&stop_position[axis_select]);
        frame_break[axis_select]=TRUE;
	if (frame!=NULL) free (frame);
	sdssdc.tccmove[axis_select].position=0;
	sdssdc.tccmove[axis_select].velocity=0;
	sdssdc.tccmove[axis_select].time=0;
        return "";

    case 1:
        tm_get_pos(axis_select<<1,&pos);
	velocity = (double).10;
	frame->end_time = fmod((double)(sdss_get_time()+
          abs((pos/ticks_per_degree[axis_select]-position)/velocity)),
	  (double)86400.0);
	velocity=0.0;
        break;

    case 2:
        tm_get_pos(axis_select<<1,&pos);
	frame->end_time = fmod((double)(sdss_get_time()+
          abs((pos/ticks_per_degree[axis_select]-position)/velocity)),
	  (double)86400.0);
        break;

    case 3:
        if (sdss_delta_time(frame->end_time,sdss_get_time())<0.0)
	{
	  free (frame);
	  printf("\r\n MOVE CMD: bad time=%lf real time=%lf",frame->end_time,
		sdss_get_time());
	  return "ERR: BAD TIME";
	}
  	if (drift_break[axis_select])
	{
	  if (DRIFT_verbose)
            printf("\r\nDRIFT pvt %lf %lf %lf",
		position,velocity,frame->end_time);
	  taskLock();
          tm_get_pos(axis_select<<1,&pos);
	  dt=sdss_delta_time(frame->end_time,sdss_get_time());
	  taskUnlock();
/* in practice, shifted around sample frequency (.05) for better err transition */
	  dt -=.043;	/* modify time to reduce error during transition */
	  pos=(pos+
	    (drift_velocity[axis_select]*dt))/ticks_per_degree[axis_select];
	  if (DRIFT_verbose)
            printf("\r\nDRIFT modified pvt %lf %lf %lf, difference=%lf, dt=%lf",
		pos,velocity,frame->end_time,position-pos,dt);
	  if (drift_modify_enable)
	    position=pos;
	  drift_break[axis_select]=FALSE;
	}
	break;
  }
  frame->position=position;
  if (fabs(velocity)>max_velocity[axis_select]) 
    if (velocity>0.0)
      velocity=max_velocity[axis_select];
    else
      velocity=-max_velocity[axis_select];
  frame->velocity=(double)velocity;
  frame->nxt = NULL;
  sdssdc.tccmove[axis_select].position=
	(long)(frame->position*ticks_per_degree[axis_select]);
  sdssdc.tccmove[axis_select].velocity=
	(long)(frame->velocity*ticks_per_degree[axis_select]);
  sdssdc.tccmove[axis_select].time=(long)(frame->end_time*1000);

/* queues are initialized with one dummy entry at sdss_init time */
  taskLock();
  nxtque = queue->end;
  nxtque->nxt = frame;
  if (queue->active==NULL)/* end of queue, and becomes active frame */
    queue->active=frame;
  for (i=0;i<OFF_MAX;i++)
  {
    if (offset_queue_end[axis_select][i]!=NULL)
    {
/* still correcting offset , reduce new specifications */
      offset_queue_end[axis_select][i]=frame;
      frame->position-=offset[axis_select][i][1].position;
      frame->velocity-=offset[axis_select][i][1].velocity;
/*      printf ("\r\nreduce offset end=%p, pos=%lf,idx=%d",frame,
	frame->position,offset_idx[axis_select][i]);*/
    }
  }
  queue->end=frame;
  queue->cnt++;
  taskUnlock();
/* clean up queue for old entries */
  while ((queue->cnt>MAX_FRAME_CNT)&&(queue->top != queue->active))
  {
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
** ROUTINE: plus_move_cmd
**
** DESCRIPTION:
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
** CALLS TO:
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
char *plus_move_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  double position,velocity,frame_time;
  struct FRAME_QUEUE *queue;
  int cnt;
  int i;

/*  printf (" +MOVE command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  for (i=0;i<OFF_MAX;i++)
    if (offset_queue_end[axis_select][i]==NULL) break;
  if (i>=OFF_MAX) return "ERR: offset active";
  queue = &axis_queue[axis_select];
  cnt=sscanf (cmd,"%lf %lf %lf",&position,&velocity,&frame_time);

  switch (cnt)
  {
    case -1:
    case 0:
        break;		/* NULL offset - does nothing */
    case 1:
	if (position==0.0) break;
        offset[axis_select][i][0].nxt=&offset[axis_select][i][1];
        offset[axis_select][i][0].position=0;
        offset[axis_select][i][1].position=position;
	offset[axis_select][i][0].velocity=0;
	offset[axis_select][i][1].velocity=0;
	offset[axis_select][i][0].end_time=0;
/* short offsets are give some extra time for smooth ramp.  long offsets
are spread over a time period averaging .4 degs per second */
        if (position<.15)
	  offset[axis_select][i][1].end_time=(double).75;
	else
	  offset[axis_select][i][1].end_time=
		(double)((int)((position/.15)*20))/20.;
	offset_idx[axis_select][i]=0;
	offset_queue_end[axis_select][i]=queue->end;
        break;
    case 2:
	if ((position==0.0)&&(velocity==0.0)) break;
        offset[axis_select][i][0].nxt=&offset[axis_select][i][1];
        offset[axis_select][i][0].position=0;
        offset[axis_select][i][1].position=position;
	offset[axis_select][i][0].velocity=0;
	offset[axis_select][i][1].velocity=velocity;
	offset[axis_select][i][0].end_time=0;
        if (position<.15)
	  offset[axis_select][i][1].end_time=(double).75;
	else	/* average .4 degree per second */
	  offset[axis_select][i][1].end_time=
		(double)((int)((position/.15)*20))/20.;
	offset_idx[axis_select][i]=0;
	offset_queue_end[axis_select][i]=queue->end;
        break;
    case 3:
	return "ERR: unimplemented";
/*
	if ((position==0.0)&&(velocity==0.0)) break;
        offset[axis_select][i][0].nxt=&offset[axis_select][i][1];
        offset[axis_select][i][0].position=0;
        offset[axis_select][i][1].position=position;
	offset[axis_select][i][0].velocity=0;
	offset[axis_select][i][1].velocity=velocity;
	offset[axis_select][i][0].end_time=0;
        if (position<.2)
	  offset[axis_select][i][1].end_time=(double).4;
	else
	  offset[axis_select][i][1].end_time=(double)((int)((position/1.0)*20))/20.;
	offset_idx[axis_select][i]=0;
	offset_queue_end[axis_select][i]=queue->end;
*/
        break;
  }
	printf("\r\n%p: queue_end=%p, position=%lf, velocity=%lf, end_time=%lf",
		&offset[axis_select][i],offset_queue_end[axis_select][i],
		offset[axis_select][i][1].position,
		offset[axis_select][i][1].velocity,
		offset[axis_select][i][1].end_time);
  sdssdc.tccpmove[axis_select].position=
	(long)(offset[axis_select][i][1].position*ticks_per_degree[axis_select]);
  sdssdc.tccpmove[axis_select].velocity=
	(long)(offset[axis_select][i][1].velocity*ticks_per_degree[axis_select]);
  sdssdc.tccpmove[axis_select].time=
	(long)(offset[axis_select][i][1].end_time*1000);
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: mr_dump_cmd
**
** DESCRIPTION:
**	MR.DUMP - Displays a large buffer of positions at 20 ms resolution.
**	Remains unimplemented since better engineering tools are available.
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
char *mr_dump_cmd(char *cmd)
{
  printf (" MR.DUMP command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ms_dump_cmd
**
** DESCRIPTION:
**	MS.DUMP - Displays the last 12 fiducials passed.
**	Remains unimplemented since better engineering tools are available.
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
char *ms_dump_cmd(char *cmd)
{
/*  printf (" MS.DUMP command fired\r\n");*/
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ms_map_dump_cmd
**
** DESCRIPTION:
**	MS.MAP.DUMP - Displays the last all fiducials if fed back to the 
**	MCP will restore the positions.
**	Remains unimplemented.  Fiducials are currently saved and restored
**	from shared memory.
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
char *ms_map_dump_cmd(char *cmd)
{
  printf (" MS.MAP.DUMP command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ms_map_load_cmd
**
** DESCRIPTION:
**	MS.MAP.LOAD ms# val - Loads the specified fiducials.
**	Remains unimplemented.  Fiducials are currently restored from
**	shared memory.
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
char *ms_map_load_cmd(char *cmd)
{
  printf (" MS.MAP.LOAD?????? command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ms_off_cmd
**	    ms_on_cmd
**
** DESCRIPTION:
**	MS.OFF - turn off automatic setting of positions as fiducials are passed.
**	MS.ON - turn on automatic setting of positions as fiducials are passed.
**	Unimplemented until fiducials are better tested.
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
char *ms_off_cmd(char *cmd)
{
/*  printf (" MS.OFF command fired\r\n");*/
  return "";
}
char *ms_on_cmd(char *cmd)
{
/*  printf (" MS.ON command fired\r\n");*/
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ms_pos_dump_cmd
**
** DESCRIPTION:
**	MS.POS.DUMP - Displays the last all fiducials.
**	Remains unimplemented.  Fiducials can be viewed from 
**	print_fiducials(axis).
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
char *ms_pos_dump_cmd(char *cmd)
{
  printf (" MS.POS.DUMP command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: remap_cmd
**
** DESCRIPTION:
**	REMAP - maps all the fiducials.
**	Unimplemented until fiducials are better tested.
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
char *remap_cmd(char *cmd)
{
  printf (" REMAP command fired\r\n");
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
char *rot_cmd(char *cmd)
{
/*  printf (" IR command fired\r\n");*/
  axis_select=INSTRUMENT;
  return "";
}
char *tel1_cmd(char *cmd)
{
/*  printf (" TEL1 command fired\r\n");*/
  axis_select=AZIMUTH;
  return "";
}
char *tel2_cmd(char *cmd)
{
/*  printf (" TEL2 command fired\r\n");*/
  axis_select=ALTITUDE;
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_limits_cmd
**
** DESCRIPTION:
**	SET.LIMITS pos1 pos2 - sets maximum and minimum positions for axis.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	axis_select
**	max_position
**	min_position
**
**=========================================================================
*/
char *set_limits_cmd(char *cmd)
{
  double pos1,pos2;
  int cnt;

  printf (" SET.LIMITS command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  cnt=sscanf (cmd,"%12lf %12lf",&pos1,&pos2);
  if (cnt!=2) return "ERR: requires two positions specified";
  if(pos1>pos2)
  {
    max_position[axis_select]=pos1;
    min_position[axis_select]=pos2;
  }
  else
  {
    max_position[axis_select]=pos2;
    min_position[axis_select]=pos1;
  }
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_position_cmd
**
** DESCRIPTION:
**	SET.POSITION pos - sets position of axis.  Could cause jerk and is
**	not implemented.   This functionality is available via the Menu.
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
char *set_position_cmd(char *cmd)
{
  printf (" SET.POSITION command fired\r\n");
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_time_cmd
**	    print_time_changes - diagnostic
**
** DESCRIPTION:
**	SET.TIME date - sets time to WWVB.
**	Date can be specified as either "month day year hour minute second" or
**	"month day year seconds-in-day".
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	SDSStime
**	time1, time2
**
**=========================================================================
*/
long SDSStime=-1;
char *set_time_cmd(char *cmd)
{
  float t1,t2,t3;
  int cnt;
  struct timespec tp;
  struct tm t;
  int extrasec;

/*  printf (" SET.TIME command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  cnt=sscanf (cmd,"%d %d %d %f %f %f",&t.tm_mon,&t.tm_mday,&t.tm_year,&t1,&t2,&t3);
  switch (cnt)
  {
/* date -> m d y h m s */
    case 6:
      t.tm_hour=(int)t1;
      t.tm_min=(int)t2;
      t.tm_sec=(int)t3;
      if ((t3-(int)t3)>.75)
      {
        taskDelay(20);	/* 1/3 sec */
        extrasec=1;
      }
      else 
        extrasec=0;
      break;		

/* date -> m d y s  where s is in floating pt precision*/
    case 4:
      t.tm_hour = (int)(t1/3600);
      t.tm_min = (int)((t1-(t.tm_hour*3600))/60);
      t3 = (float)(t1-(t.tm_hour*3600)-(t.tm_min*60));
      t.tm_sec = (int)t3;
      if ((t3-(int)t3)>.75)
      {
        taskDelay(20);	/* 1/3 sec */
        extrasec=1;
      }
      else 
        extrasec=0;
      break;		

      default:
	return "ERR: wrong number of args";
  }
  t.tm_year -= 1900;
  t.tm_mon -= 1;
  tp.tv_sec=mktime(&t)+extrasec;
  tp.tv_nsec=0;
  time1[axis_select]=sdss_get_time();	/* before and after for diagnostic */
  SDSStime=tp.tv_sec%ONE_DAY;
  time2[axis_select]=sdss_get_time();
  clock_settime(CLOCK_REALTIME,&tp);
/*
  printf("\r\nt3=%f (extrasec=%d)",t3,extrasec);
  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
*/
  return "";
}
void print_time_changes()
{
  int i;

  for (i=0;i<3;i++)
    printf ("\r\nSDSS axis %d:  time1=%lf time2=%lf",i,time1[i],time2[i]);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: stats_cmd
**
** DESCRIPTION:
**	STATS - Tracking error statistics based on fiducial crossing.
**	Unimplemented until fiducials are better understood.  The information
**	is available via print_fiducials(axis).
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
char *stats_cmd(char *cmd)
{
  printf (" STATS command fired\r\n");
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
static char *status_ans=
{"1073741824                                                                   "};	/* 0x40000000 */
char *status_cmd(char *cmd)
{
  extern struct TM_M68K *tmaxis[];
  extern struct FIDUCIARY fiducial[3];
  extern SEM_ID semMEIUPD;
  extern struct AXIS_STAT axis_stat[];

/*  printf (" STATUS command fired\r\n"); */
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (sdss_get_time()<0) 
  {
   if (semTake (semMEIUPD,60)!=ERROR)
   {
    sprintf (status_ans,"%lf %lf %f %ld %lf",
	(*tmaxis[axis_select]).actual_position/ticks_per_degree[axis_select],
	(*tmaxis[axis_select]).velocity/ticks_per_degree[axis_select],
	sdss_get_time()+1.0,
	axis_stat[axis_select],
	fiducial[axis_select].mark/ticks_per_degree[axis_select]);
    semGive (semMEIUPD);
    return status_ans;
   }
  }
  else
  {
   if (semTake (semMEIUPD,60)!=ERROR)
   {
    sprintf (status_ans,"%lf %lf %f %ld %lf",
	(*tmaxis[axis_select]).actual_position/ticks_per_degree[axis_select],
	(*tmaxis[axis_select]).velocity/ticks_per_degree[axis_select],
	sdss_get_time(),
	axis_stat[axis_select],
	fiducial[axis_select].mark/ticks_per_degree[axis_select]);
    semGive (semMEIUPD);
    return status_ans;
   }
  }
  return "ERR: semMEIUPD";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: status_long_cmd
**
** DESCRIPTION:
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
char *status_long_ans={"10987654321098765432109876543210"};/* 31-0 bits */
char *status_long_cmd(char *cmd)
{
  unsigned long bit_check=0x80000000;
  int i;

  printf (" STATUS.LONG command fired\r\n");
  for (i=0;i<32;i++) 
  {
    if (status&bit_check)
      status_long_ans[i]='1';
    else
      status_long_ans[i]='0';
    bit_check >>= 1;
  }
  return status_long_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ticklost_cmd
**
** DESCRIPTION:
**	TICKLOST @ . - Returns if receiving 1 Hz ticks.
**
** RETURN VALUES:
**	return "0" or undefined if not receiving ticks so i return NULL string.
**
** CALLS TO:
**	sdss_get_time
**	sdss_delta_time
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *ticklost_cmd(char *cmd)
{
  double tick;

  printf (" TICKLOST @ . command fired\r\n");
  tick=sdss_get_time();
  taskDelay(65);
  if (sdss_delta_time(sdss_get_time(),tick)>1.0) return "0";
  else
    return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: time_cmd
**
** DESCRIPTION:
**	TIME? - gets date and time.
**
** RETURN VALUES:
**	return date as "m d y seconds-of-day"
**
** CALLS TO:
**	sdss_get_time
**
** GLOBALS REFERENCED:
**	SDSStime
**
**=========================================================================
*/
static char *time_ans={"01 31 1996 86400.000                "};	/* */
char *time_cmd(char *cmd)
{
  static struct tm *t;
  struct timespec tp;

/*  printf (" TIME? command fired\r\n");*/
  clock_gettime(CLOCK_REALTIME,&tp);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
  t = localtime(&tp.tv_sec);
/*  printf ("t=%p, mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t,t->tm_mon,t->tm_mday,t->tm_year,t->tm_hour,t->tm_min,t->tm_sec);*/
  sprintf (time_ans,"%d %d %d %f",t->tm_mon+1,t->tm_mday,t->tm_year+1900,
	sdss_get_time());
  return time_ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: cwmov_cmd
**	    cwinst_cmd
**	    cwpos_cmd
**	    cwabort_cmd
**	    cwstatus_cmd
**
** DESCRIPTION:
**	CWMOV - Move specified counter-weight to specified position in units
**	of volts*100.
**	CWINST - Move all counter-weights to the instruments specified 
**	positions.
**	CWPOS - Move all four counter-weights to specified position in units
**	of volts*100.
**	CWABORT - Abort any counter-weight motion provided by spawned tasks.
**	CWSTATUS - Return status of counter_weights including position and 
**	limit status.
**
** RETURN VALUES:
**	return NULL string except for status return
**
** CALLS TO:
**	cw_positionv
**	balance_weight
**
** GLOBALS REFERENCED:
**	sdssdc
**	semMEIUPD
**
**=========================================================================
*/
char *cwmov_cmd(char *cmd)
{
  int cw;
  int cwpos;
  extern struct SDSS_FRAME sdssdc;

  printf (" CWMOV command fired\r\n");
  sscanf (cmd,"%d %d",&cw,&cwpos);
  if ((cw<0)||(cw>4))
    return "ERR: Bad CW id (0-3)";
  if ((cwpos<10)||(cwpos>800))
    return "ERR: Position out of Range (10-800)";
  if (taskIdFigure("cw")!=ERROR)
    return "ERR: CW task still active";
  if (taskIdFigure("cwp")!=ERROR)
    return "ERR: CWP task still active";
  if (sdssdc.status.i9.il0.alt_brake_en_stat)
    taskSpawn ("cwp",60,VX_FP_TASK,4000,(FUNCPTR)cw_positionv,
			  cw,cwpos,0,0,0,0,0,0,0,0);
  else
    return "ERR: Altitude Brake NOT Engaged";
  return "";
}
char *cwinst_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  int inst;

  printf (" CWINST command fired\r\n");
  while (*cmd==' ') cmd++;
  if ((inst=cw_get_inst(cmd))==-1)
    return "ERR: Invalid Instrument";
  if ((inst<0)||(inst>16)) 
    return "ERR: Inst Out of Range (0-16)";
  if (taskIdFigure("cw")!=ERROR)
    return "ERR: CW task still active";
  if (taskIdFigure("cwp")!=ERROR)
    return "ERR: CWP task still active";
  if (sdssdc.status.i9.il0.alt_brake_en_stat)
    taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
			  (int)inst,0,0,0,0,0,0,0,0,0);
  else
    return "ERR: Altitude Brake NOT Engaged";
  return "";
}
char *cwpos_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  int cwpos;

  printf (" CWPOS command fired\r\n");
  sscanf (cmd,"%d",&cwpos);
  if ((cwpos<10)||(cwpos>800))
    return "ERR: Position out of Range (10-800)";
  if (taskIdFigure("cw")!=ERROR)
    return "ERR: CW task still active";
  if (taskIdFigure("cwp")!=ERROR)
    return "ERR: CWP task still active";
  cw_set_positionv(INST_DEFAULT,cwpos,cwpos,cwpos,cwpos);
  if (sdssdc.status.i9.il0.alt_brake_en_stat)
    taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
		  (int)INST_DEFAULT,0,0,0,0,0,0,0,0,0);
  else
    return "ERR: Altitude Brake NOT Engaged";
  return "";
}
char *cwabort_cmd(char *cmd)
{
  printf (" CWABORT command fired\r\n");
  taskDelete(taskIdFigure("cw"));
  taskDelete(taskIdFigure("cwp"));
  return "";
}
static char *limitstatus[]={"LU","L "," U","  "};
static char *cwstatus_ans=
	{"CW# 800 UL  CW# 800 UL  CW# 800 UL  CW# 800 UL       "};
char *cwstatus_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  int i, idx;
  int adc;
  int limidx;
  extern unsigned char cwLimit;
  extern SEM_ID semMEIUPD;

  printf (" CWSTATUS command fired\r\n");
  if (semTake (semMEIUPD,60)!=ERROR)
  {
    idx=0;
    for (i=0;i<4;i++)
    {
/*      printf("\r\n%d %d",sdssdc.weight[i].pos,idx);*/
	adc=sdssdc.weight[i].pos;
        if ((adc&0x800)==0x800) adc |= 0xF000;
        else adc &= 0xFFF;
	limidx = (cwLimit>>(i*2))&0x3;
        idx+=sprintf (&cwstatus_ans[idx],"CW%d %d %s",
	  i,(1000*adc)/2048,limitstatus[limidx]);
/*       printf("%s",cwstatus_ans);*/
    }
    semGive (semMEIUPD);
  }
  else
    return "ERR: semMEIUPD";
  return cwstatus_ans;
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
  printf (" BRAKEON command fired\r\n");
  if (axis_select==AZIMUTH)
    tm_sp_az_brake_on();
  else if (axis_select==ALTITUDE)
         tm_sp_alt_brake_on();
       else
         return "ERR: ILLEGAL DEVICE SELECTION";
  return "";
}
char *brakeoff_cmd(char *cmd)
{
  printf (" BRAKEOFF command fired\r\n");
  if (axis_select==AZIMUTH)
    tm_sp_az_brake_off();
  else if (axis_select==ALTITUDE)
         tm_sp_alt_brake_off();
       else
         return "ERR: ILLEGAL DEVICE SELECTION";
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: clampon_cmd
**	    clampoff_cmd
**
** DESCRIPTION:
**	CLAMP.ON - turn on the instrument change postion clamp.
**	CLAMP.OFF - turn off the instrument change position clamp.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**	tm_sp_clamp_on
**	tm_sp_clamp_off
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *clampon_cmd(char *cmd)
{
  printf (" CLAMPON command fired\r\n");
  tm_sp_clamp_on();
  return "";
}
char *clampoff_cmd(char *cmd)
{
  printf (" CLAMPOFF command fired\r\n");
  tm_sp_clamp_off();
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: sp1_cmd
**	    sp2_cmd
**
** DESCRIPTION:
**	SP1 - select spectograph1
**	SP2 - select spectograph2
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	spectograph_select
**
**=========================================================================
*/
char *sp1_cmd(char *cmd)
{
  printf (" SP1 command fired\r\n");
  spectograph_select=SPECTOGRAPH1;
  return "";
}
char *sp2_cmd(char *cmd)
{
  printf (" SP2 command fired\r\n");
  spectograph_select=SPECTOGRAPH2;
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: slitclear_cmd
**	    slitopen_cmd
**	    slitclose_cmd
**
** DESCRIPTION:
**	SLIT.CLEAR - Neither close nor open the slit door...allow it to be 
**	moved by hand or gravity
**	SLIT.OPEN - open the selected spectograph's slit door.
**	SLIT.CLOSE - close the selected spectograph's slit door.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**	tm_slit_clear
**	tm_slit_open
**	tm_slit_close
**
** GLOBALS REFERENCED:
**	spectograph_select
**
**=========================================================================
*/
char *slitclear_cmd(char *cmd)
{
  printf (" SLIT.CLEAR command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_slit_clear(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *slitopen_cmd(char *cmd)
{
  printf (" SLIT.OPEN command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_slit_open(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *slitclose_cmd(char *cmd)
{
  printf (" SLIT.CLOSE command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_slit_close(spectograph_select-SPECTOGRAPH1);
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: cartlatch_cmd
**	    cartunlatch_cmd
**
** DESCRIPTION:
**	CART.LATCH - latch the fiber cartridge
**	CART.UNLATCH - latch the fiber cartridge
**	There is currently no status as to the position of the latch.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**	tm_cart_latch
**	tm_cart_unlatch
**
** GLOBALS REFERENCED:
**	spectograph_select
**
**=========================================================================
*/
char *cartlatch_cmd(char *cmd)
{
  printf (" CART.LATCH command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_cart_latch(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *cartunlatch_cmd(char *cmd)
{
  printf (" CART.UNLATCH command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_cart_unlatch(spectograph_select-SPECTOGRAPH1);
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: slitstatus_cmd
**
** DESCRIPTION:
**	SLIT.STATUS - status of the selected slit and latch for the 
**	fiber cartridge
**	There is currently no status as to the position of the latch.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	spectograph_select
**	sdssdc
**
**=========================================================================
*/
static char *slitstatus[]={"    ","OPEN","     ","CLOSE","UNLATCH","LATCH  "};
static char *slitstatus_ans={"SP1 OPEN CLOSE UNLATCH SP2 OPEN CLOSE UNLATCH"};
char *slitstatus_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;

  printf (" SLIT.STATUS command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  slitstatus_ans[2]=0x31+spectograph_select;
  sprintf (&slitstatus_ans[4],"%s %s %s %s %s %s",
	slitstatus[sdssdc.status.i1.il9.slit_head_door1_opn],
	slitstatus[sdssdc.status.i1.il9.slit_head_door1_cls+2],
	slitstatus[sdssdc.status.i1.il9.slit_head_latch1_opn+4],
	slitstatus[sdssdc.status.i1.il9.slit_head_door2_opn],
	slitstatus[sdssdc.status.i1.il9.slit_head_door2_cls+2],
	slitstatus[sdssdc.status.i1.il9.slit_head_latch2_opn+4]);
  return slitstatus_ans;	
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ffsopen_cmd
**	    ffsclose_cmd
**
** DESCRIPTION:
**	FFS.OPEN - open the flat field screen
**	FFS.CLOSE - close the flat field screen
**
** RETURN VALUES:
**	NULL string
**
** CALLS TO:
**	tm_ffs_open
**	tm_ffs_close
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *ffsopen_cmd(char *cmd)
{
  printf (" FFS.OPEN command fired\r\n");
  tm_ffs_open();
  return "";
}
char *ffsclose_cmd(char *cmd)
{
  printf (" FFS.CLOSE command fired\r\n");
  tm_ffs_close();
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: fflon_cmd
**	    ffloff_cmd
**
** DESCRIPTION:
**	FFL.ON - turn on the flat field incandescent lamps
**	FFL.OFF - turn off the flat field incandescent lamps
**
** RETURN VALUES:
**	NULL string
**
** CALLS TO:
**	tm_ffl_on
**	tm_ffl_off
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *fflon_cmd(char *cmd)
{
  printf (" FFL.ON command fired\r\n");
  tm_ffl_on();
  return "";
}
char *ffloff_cmd(char *cmd)
{
  printf (" FFL.OFF command fired\r\n");
  tm_ffl_off();
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: neon_cmd
**	    neoff_cmd
**
** DESCRIPTION:
**	NE.ON - turn on the flat field neon lamps
**	NE.OFF - turn off the flat field neon lamps
**
** RETURN VALUES:
**	NULL string
**
** CALLS TO:
**	tm_neon_on
**	tm_neon_off
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *neon_cmd(char *cmd)
{
  printf (" NE.ON command fired\r\n");
  tm_neon_on();
  return "";
}
char *neoff_cmd(char *cmd)
{
  printf (" NE.OFF command fired\r\n");
  tm_neon_off();
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: hgcdon_cmd
**	    hgcdoff_cmd
**
** DESCRIPTION:
**	HGCD.ON - turn on the flat field incandescent lamps
**	HGCD.OFF - turn off the flat field incandescent lamps
**
** RETURN VALUES:
**	NULL string
**
** CALLS TO:
**	tm_hgcd_on
**	tm_hgcd_off
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *hgcdon_cmd(char *cmd)
{
  printf (" HGCD.ON command fired\r\n");
  tm_hgcd_on();
  return "";
}
char *hgcdoff_cmd(char *cmd)
{
  printf (" HGCD.OFF command fired\r\n");
  tm_hgcd_off();
  return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ffstatus_cmd
**
** DESCRIPTION:
**	FF.STATUS - turn on the flat field incandescent lamps
**
** RETURN VALUES:
**	NULL string
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
char ffstatus_ans[250];
char *ffstatus_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  char open[]={' ','O'};
  char close[]={' ','C'};
  char *oo[]={"Off"," On"};

  printf (" FF.STATUS command fired\r\n");
  sprintf (&ffstatus_ans[0],"\r\nLeaf 01 02 03 04 05 06 07 08");
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],
	"\r\n  FF %c%c  %c%c %c%c %c%c %c%c  %c%c %c%c %c%c",
	open[sdssdc.status.i1.il13.leaf_1_open_stat],
	close[sdssdc.status.i1.il13.leaf_1_closed_stat],
	open[sdssdc.status.i1.il13.leaf_2_open_stat],
	close[sdssdc.status.i1.il13.leaf_2_closed_stat],
	open[sdssdc.status.i1.il13.leaf_3_open_stat],
	close[sdssdc.status.i1.il13.leaf_3_closed_stat],
	open[sdssdc.status.i1.il13.leaf_4_open_stat],
	close[sdssdc.status.i1.il13.leaf_4_closed_stat],
	open[sdssdc.status.i1.il13.leaf_5_open_stat],
	close[sdssdc.status.i1.il13.leaf_5_closed_stat],
	open[sdssdc.status.i1.il13.leaf_6_open_stat],
	close[sdssdc.status.i1.il13.leaf_6_closed_stat],
	open[sdssdc.status.i1.il13.leaf_7_open_stat],
	close[sdssdc.status.i1.il13.leaf_7_closed_stat],
	open[sdssdc.status.i1.il13.leaf_8_open_stat],
	close[sdssdc.status.i1.il13.leaf_8_closed_stat]
  );
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],"\r\nLamp  01  02  03  04");
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],"\r\n  FF %s %s %s %s",
	oo[sdssdc.status.i1.il13.ff_1_stat],
	oo[sdssdc.status.i1.il13.ff_2_stat],
	oo[sdssdc.status.i1.il13.ff_3_stat],
	oo[sdssdc.status.i1.il13.ff_4_stat]
  );
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],"\r\n  Ne %s %s %s %s",
	oo[sdssdc.status.i1.il13.ne_1_stat],
	oo[sdssdc.status.i1.il13.ne_2_stat],
	oo[sdssdc.status.i1.il13.ne_3_stat],
	oo[sdssdc.status.i1.il13.ne_4_stat]
  );
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],"\r\nHgCd %s %s %s %s",
	oo[sdssdc.status.i1.il13.hgcd_1_stat],
	oo[sdssdc.status.i1.il13.hgcd_2_stat],
	oo[sdssdc.status.i1.il13.hgcd_3_stat],
	oo[sdssdc.status.i1.il13.hgcd_4_stat]
  );
/*
  sprintf(&ffstatus_ans[strlen(&ffstatus_ans[0])],"\n\rhgcd_lamps_on_pmt=%d",sdssdc.status.o1.ol14.hgcd_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(&ffstatus_ans[0])],"\n\rne_lamps_on_pmt=%d",sdssdc.status.o1.ol14.ne_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(&ffstatus_ans[0])],"\n\rff_lamps_on_pmt=%d",sdssdc.status.o1.ol14.ff_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(&ffstatus_ans[0])],"\n\rff_screen_open_pmt=%d",sdssdc.status.o1.ol14.ff_screen_open_pmt);
  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],"\r\n");
*/
  printf ("\r\nffstatus_ans length=%d",strlen(&ffstatus_ans[0]));
  return &ffstatus_ans[0];	
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
  extern struct SDSS_FRAME sdssdc;
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
** ROUTINE: calc_frames
**
** DESCRIPTION:
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
int calc_frames (int axis, struct FRAME *iframe, int start)
{
  double dx,dv,dt,vdot;
  double ai,j,t,lai,lj,lt,ldt;
  struct FRAME *fframe;
  struct FRAME *lframe;
  int i;
  
/* problem............................*/
  if (iframe->nxt==NULL) return ERROR;
/*....................................*/
  fframe=iframe->nxt;
  dx=fframe->position-iframe->position;
  dv=fframe->velocity-iframe->velocity;
  dt=sdss_delta_time(fframe->end_time,iframe->end_time);
  vdot=dx/dt;
  ai=(2/dt)*((3*vdot)-(2*iframe->velocity)-fframe->velocity);
  j=(6/(dt*dt))*(iframe->velocity+fframe->velocity-(2*vdot));
/* neccessary if for loop not executed; calc of t*/
  t=(start+1)/FLTFRMHZ+time_off[axis];	/* for end condition */
  if (CALC_verbose)
  {
    printf ("\r\n iframe=%p, fframe=%p",iframe,fframe);
    printf ("\r\n iframe->end_time=%lf, fframe->end_time=%lf",
		iframe->end_time,fframe->end_time);
    printf("\r\n dx=%12.8lf, dv=%12.8lf, dt=%12.8lf, vdot=%lf",dx,dv,dt,vdot);
    printf("\r\n ai=%12.8lf, j=%12.8lf, t=%f, start=%d, time_off=%f",ai,j,t,start,time_off[axis]);
  }
  for (i=0;i<(int)min(MAX_CALC-1,
		(int)((dt-time_off[axis])*FRMHZ)-start);i++)
  {
    t=(i+start+1)/FLTFRMHZ+time_off[axis];
    tim[axis][i]=1/FLTFRMHZ;
    p[axis][i]=iframe->position+(iframe->velocity*t)+(1/2.)*ai*(t*t)+
    			(1/6.)*j*(t*t*t);
    v[axis][i]=iframe->velocity+(ai*t)+(1/2.)*j*(t*t);
    a[axis][i]=ai+(j*t);
    ji[axis][i]=j;
    if (CALC_verbose)
      printf ("\r\n%d @%lf Secs: ti=%lf, p=%12.8lf, v=%12.8lf, a=%12.8lf",
 	i,t,tim[axis][i],p[axis][i],v[axis][i],a[axis][i]);
  }
/* last one with a portion remaining; needs portion of next one */
/*  printf ("\r\nCheck for FINAL: time_off=%f, i=%d, start=%d, t=%f, dt=%f",
		time_off[axis],i,start,t,dt);*/

  if ( ((int)(i+start)==(int)((dt-time_off[axis])*FLTFRMHZ)) &&
	(t!=(dt-time_off[axis])) )
  {
    ldt=(((dt-time_off[axis])*FLTFRMHZ)-(int)((dt-time_off[axis])*FLTFRMHZ))/FLTFRMHZ;
    t = (1/FLTFRMHZ)-ldt;
    lt = dt;
    tim[axis][i]=1/FLTFRMHZ;
    lframe=fframe;
    if (lframe->nxt==NULL) 
    {
      printf ("\r\nCALC FRAME: next frame required to finish");
      return ERROR;
    }
    fframe=lframe->nxt;
    time_off[axis]=t;
    lai=ai; 
    lj=j;
    dx=fframe->position-lframe->position;
    dv=fframe->velocity-lframe->velocity;
    dt=fframe->end_time-lframe->end_time;
    vdot=dx/dt;
    if (CALCFINAL_verbose)
    {
      printf ("\r\n time_off=%f, ldt=%f, lt=%f, t=%f",time_off[axis],ldt,lt,t);
      printf("\r\n dx=%12.8lf, dv=%12.8lf, dt=%12.8lf, vdot=%12.8lf",dx,dv,dt,vdot);
    }
    ai=(2/dt)*((3*vdot)-(2*lframe->velocity)-fframe->velocity);
    j=(6/(dt*dt))*(lframe->velocity+fframe->velocity-(2*vdot));

    p[axis][i]=lframe->position+(lframe->velocity*t)+(1/2.)*ai*(t*t)+
    			(1/6.)*j*(t*t*t);
    v[axis][i]=lframe->velocity+(ai*t)+(1/2.)*j*(t*t);
    a[axis][i]=(FLTFRMHZ*t*(ai+(j*t)))+(FLTFRMHZ*ldt*(lai+(lj*lt)));
    ji[axis][i]=(FLTFRMHZ*t*j)+(FLTFRMHZ*ldt*lj);
    if (CALCFINAL_verbose)
      printf ("\r\nFinal %d @%lf Secs: ti=%lf, p=%12.8lf, v=%12.8lf, a=%12.8lf",
	i,t,tim[axis][i],p[axis][i],v[axis][i],a[axis][i]);
    if ((i+1)>MAX_CALC) printf ("\r\n calc_frames has problems %d",i+1);
    return (i+1);
  }
  else 
  {
    if (i>(MAX_CALC-1)) printf ("\r\n calc_frames has problems %d",i);
    return i;
  }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: calc_offset
**
** DESCRIPTION:
**	Calculate offset frames from pvt pairs starting at a specified position.
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
int calc_offset (int axis, struct FRAME *iframe, int start, int cnt)
{
  double dx,dv,dt,vdot;
  double ai,j,t;
  struct FRAME *fframe;
  int i,ii;
  
  fframe=iframe->nxt;
  dx=fframe->position-iframe->position;
  dv=fframe->velocity-iframe->velocity;
  dt=sdss_delta_time(fframe->end_time,iframe->end_time);
  vdot=dx/dt;
  ai=(2/dt)*((3*vdot)-(2*iframe->velocity)-fframe->velocity);
  j=(6/(dt*dt))*(iframe->velocity+fframe->velocity-(2*vdot));
/* neccessary if for loop not executed; calc of t*/
  t=(start+1)/FLTFRMHZ;	/* for end condition */
  if (CALCOFF_verbose)
  {
    printf("\r\n dx=%12.8lf, dv=%12.8lf, dt=%12.8lf, vdot=%lf",dx,dv,dt,vdot);
    printf("\r\n ai=%12.8lf, j=%12.8lf, t=%f, start=%d, ",ai,j,t,start);
  }
  for (i=0;i<(int)min(cnt,
		(int)(dt*FRMHZ)-start);i++)
  {
    t=(i+start+1)/FLTFRMHZ;
    timoff[axis][i]=1/FLTFRMHZ;
    poff[axis][i]+=iframe->position+(iframe->velocity*t)+(1/2.)*ai*(t*t)+
    			(1/6.)*j*(t*t*t);
    voff[axis][i]+=iframe->velocity+(ai*t)+(1/2.)*j*(t*t);
    aoff[axis][i]+=ai+(j*t);
    jioff[axis][i]+=j;
    if (CALCOFF_verbose)
      printf ("\r\n%d @%lf Secs: ti=%lf, p=%12.8lf, v=%12.8lf, a=%12.8lf",
 	i,t,timoff[axis][i],poff[axis][i],voff[axis][i],aoff[axis][i]);
  }
  for (ii=i;ii<cnt;ii++)
  {         
    t=(ii+start+1)/FLTFRMHZ;
    timoff[axis][ii]=1/FLTFRMHZ;
    poff[axis][ii]+=fframe->position+(fframe->velocity*(t-dt));
    voff[axis][ii]+=fframe->velocity;
    if (CALCOFF_verbose)
      printf ("\r\n%d @%lf Secs: ti=%lf, p=%12.8lf, v=%12.8lf, a=%12.8lf",
        ii,t,timoff[axis][ii],poff[axis][ii],voff[axis][ii],aoff[axis][ii]);
  }
  return i;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: clroffset
**
** DESCRIPTION:
**	Clear the offset calculation.
**
** RETURN VALUES:
**	number of offset calculations cleared
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	poff,voff,aoff,jioff
**
**=========================================================================
*/
int clroffset(int axis,int cnt)
{
  int i;

  for (i=0;i<cnt;i++)
  {         
    poff[axis][i]=0;
    voff[axis][i]=0;
    aoff[axis][i]=0;
    jioff[axis][i]=0;
  }
  return cnt;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: addoffset
**
** DESCRIPTION:
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
int addoffset(int axis,int cnt)
{
  int i;

  for (i=0;i<cnt;i++)
  {         
    p[axis][i]+=poff[axis][i];
    v[axis][i]+=voff[axis][i];
    a[axis][i]+=aoff[axis][i];
    ji[axis][i]+=jioff[axis][i];
    if (CALCADDOFF_verbose)
      printf ("\r\n%d:  p=%12.8lf, v=%12.8lf, a=%12.8lf",
        i,p[axis][i],v[axis][i],a[axis][i]);
  }
  return cnt;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: start_frame
**
** DESCRIPTION:
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
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void start_frame(int axis,double time)
{
  int lcnt;
  
  time_off[axis]=0.0;
  while ((lcnt=tm_frames_to_execute(axis))>1)
  {
/*    printf("\r\nDwell frames left=%d",lcnt);*/
    taskDelay(3);
  }
  taskDelay(5);
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
     time = sdss_delta_time(time,sdss_get_time());
/*     printf("\r\ntime to dwell=%lf",time);*/
     dsp_dwell (axis<<1,time);
     semGive (semMEI);
  }
  printf ("\r\nSTART axis=%d: time=%lf",axis<<1,time);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: get_frame_count
**
** DESCRIPTION:
**	Retrieves the number of MEI frames between two pvts including the 
**	left over remants of the previous pvt pair.
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
int get_frame_cnt(int axis, struct FRAME *iframe)
{
  struct FRAME *fframe;
  double dt;
  int cnt;

  fframe=iframe->nxt;
  dt=fframe->end_time-iframe->end_time;
  cnt=(int)((dt-time_off[axis])*FLTFRMHZ);
/*  printf("\r\nfirst cnt=%d,%lf,%lf",cnt,dt,(dt-time_off[axis])*FLTFRMHZ);*/
  if ( ((dt-time_off[axis])*FLTFRMHZ)>(int)((dt-time_off[axis])*FLTFRMHZ) )
  {
    cnt++;
/*    printf(" final cnt=%d",cnt);*/
  }
  return cnt;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_rot_coeffs
**	    print_rot_coeffs
**	    set_rot_uplimit
**	    set_rot_dnlimit
**	    set_rot_state (int state)
**	    coeffs_state_deg	modify the state based on velocity in deg
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
** RETURN VALUES:
**	number of MEI frames
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
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
void set_rot_coeffs (int state, int index, short val)
{
  rot_coeffs[state].coeffs[index]=val;
}
void print_rot_coeffs ()
{
  int i;

  printf ("\r\nState %d Active",axis_coeffs_state[4]);
  for (i=0;i<sizeof(rot_coeffs)/sizeof(struct SW_COEFFS);i++)
  {
    printf ("\r\n  rot_coeffs state %d: uplimit=%d, dnlimit=%d",i,rot_coeffs[i].uplimit_cts,rot_coeffs[i].dnlimit_cts);
    printf ("\r\nP=%d,I=%d,D=%d,AFF=%d,VFF=%d,ILIM=%d,OFF=%d,OLIM=%d,S=%d,FFF=%d",
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
void set_rot_uplimit (int state, int val)
{
  rot_coeffs[state].uplimit_cts=val;
  rot_coeffs[state].uplimit_deg=val/ROT_TICKS_DEG;
}
void set_rot_dnlimit (int state, int val)
{
  rot_coeffs[state].dnlimit_cts=val;
  rot_coeffs[state].dnlimit_deg=val/ROT_TICKS_DEG;
}
void set_rot_state (int state)
{
  axis_coeffs_state[4]=state;
}
int coeffs_state_deg (int axis, double degs)
{
	int state;
	struct SW_COEFFS *coeff;
        
	state = axis_coeffs_state[axis];
	if (state==-1) return FALSE;
	taskDelay(1);
        coeff=(struct SW_COEFFS *)&rot_coeffs[state];
/*        printf ("\r\nAXIS %d: state=%d coeff=%p",axis,state,coeff);*/
        if ((coeff->uplimit_deg>0.0)&&(fabs(degs)>coeff->uplimit_deg))
        {
          state++; coeff++;
          dsp_set_filter (axis,(P_INT)&coeff->coeffs[0]);
          axis_coeffs_state[axis]=state;
          return TRUE;
	}
        else
        {
          if  (fabs(degs)<coeff->dnlimit_deg)
          {
            state--; coeff--;
            dsp_set_filter (axis,(P_INT)&coeff->coeffs[0]);
            axis_coeffs_state[axis]=state;
	    return TRUE;
          }
        }
	return FALSE;
 }
int coeffs_state_cts (int axis, int cts)
{
        int state;
        struct SW_COEFFS *coeff;

        state = axis_coeffs_state[axis];
        if (state==-1) return FALSE;
	taskDelay(1);
        coeff=(struct SW_COEFFS *)&rot_coeffs[state];
/*        printf ("\r\nAXIS %d: state=%d coeff=%p",axis,state,coeff);*/
        if ((coeff->uplimit_cts>0)&&(abs(cts)>coeff->uplimit_cts))
        {
/*	  printf ("\r\nUP cts=%d, uplimit=%d",cts,coeff->uplimit_cts);*/
          state++; coeff++;
          dsp_set_filter (axis,(P_INT)&coeff->coeffs[0]);
          axis_coeffs_state[axis]=state;
          return TRUE;
        }
        else
        {
          if  (abs(cts)<coeff->dnlimit_cts)
          {
/*            printf ("\r\nDN cts=%d, dnlimit=%d",cts,coeff->dnlimit_cts);*/
            if (state==0) return FALSE;
            state--; coeff--;
            dsp_set_filter (axis,(P_INT)&coeff->coeffs[0]);
            axis_coeffs_state[axis]=state;
            return TRUE;
          }
        }
        return FALSE;
}

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
void load_frames(int axis, int cnt, int idx, double sf)
{
  extern struct SDSS_FRAME sdssdc;
  int e;
  int i;
  FRAME frame;
  
  if (FRAME_verbose)
    printf("\r\n Load %d Frames, sf=%lf",cnt,sf);
  for (i=idx;i<(cnt+idx);i++)
  {
    if (fabs(a[axis][i])>fabs(max_acceleration[axis+3])) 
      max_acceleration[axis+3]=a[axis][i];
    if (fabs(a[axis][i])>max_acceleration[axis]) 
      printf ("\r\nAXIS %d: MAX ACC %lf exceeded by %lf",
	  axis,a[axis][i],max_acceleration[axis]);
    if (fabs(v[axis][i])>fabs(max_velocity[axis+3])) 
      max_velocity[axis+3]=v[axis][i];
    if (fabs(v[axis][i])>max_velocity[axis]) 
      printf ("\r\nAXIS %d: MAX VEL %lf exceeded by %lf",
	  axis,v[axis][i],max_velocity[axis]);
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
/*      if (axis==2)
        if (coeffs_state_deg (axis<<1,v[axis][i]));*/
      taskLock();
      e=frame_m(&frame,"0l xvajt un d",axis<<1,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,(double)ji[axis][i]*sf,
	tim[axis][i],
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,NEW_FRAME);
      taskUnlock();
      semGive (semMEI);
    }
    sdssdc.pvt[axis].position=(long)(p[axis][i]*sf);
    sdssdc.pvt[axis].velocity=(long)(v[axis][i]*sf);
    sdssdc.pvt[axis].time=(long)(tim[axis][i]*1000);
    if (DIAGQ_verbose)
    {
      if ((diagq!=NULL)&&(axis==DIAGQ_verbose))
      {
        (diagq+diagq_i)->p=p[axis][i];
        (diagq+diagq_i)->v=v[axis][i];
        (diagq+diagq_i)->a=a[axis][i];
        (diagq+diagq_i)->ji=ji[axis][i];
        (diagq+diagq_i)->tim=tim[axis][i];
        diagq_i = (diagq_i+1)%diagq_siz;
      }
    }
    if (FRAME_verbose)
        printf ("\r\n axis=%d (%d): p=%12.8lf, v=%12.8lf, a=%12.8lf, \r\nj=%12.8lf,t=%12.8lf",
	axis<<1,i,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,ji[axis][i]*sf,
	tim[axis][i]);    
    }
}
void load_frames_test(int axis, int cnt, double sf)
{
  int i;
   
    printf("\r\n Load %d Frames, sf=%lf",cnt,sf);
    for (i=0;i<cnt;i++)
    {
      printf ("\r\n axis=%d (%d): p=%12.8lf, v=%12.8lf, a=%12.8lf, j=%12.8lf,t=%12.8lf",
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

  printf ("\r\nSTOP axis=%d: p=%12.8lf",
	axis<<1,(double)pos);
  stopped=FALSE;	/* ...gets rid of warning */
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

/*  printf ("\r\nSTP axis=%d: p=%12.8lf,sf=%lf",
	axis<<1,(double)pos,sf);*/

  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    get_position(axis<<1,&position);
    get_velocity(axis<<1,&velocity);
    printf ("\r\npos=%lf, vel=%lf",position,velocity);
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
  
  printf ("\r\nDRIFT axis=%d: v=%12.8lf",
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
** ROUTINE: end_frame
**
** DESCRIPTION:
**	End frames sent to MEI since there are no new pvts.  This should
**	bring the motion to a position with no velocity or acceleration.
**	The controller will remain in closed-loop holding the position.
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
void end_frame(int axis,int index,double sf)
{
  int e;
  FRAME frame;
  
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
      e=frame_m(&frame,"0l xvajt un d",axis<<1,
	(double)p[axis][index]*sf,(double)0.0,(double)0.0,
	(double)0.0,
	(double)(1./FLTFRMHZ),
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,0);
      dsp_set_last_command(dspPtr,axis<<1,(double)p[axis][index]*sf);
      semGive (semMEI);
      printf ("\r\nEND axis=%d (%d): p=%12.8lf, v=%12.8lf, a=%12.8lf, j=%12.8lf,t=%12.8lf",
	axis<<1,index,
	(double)p[axis][index]*sf,(double)v[axis][index]*sf,(double)a[axis][index]*sf,
	(double)ji[axis][index]*sf,
	tim[axis][index]);    
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_frames_to_execute
**
** DESCRIPTION:
**	Returns remaining frames in MEI queue or ERROR
**
** RETURN VALUES:
**	remaining frames to execute
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	semMEI
**
**=========================================================================
*/
int tm_frames_to_execute(int axis)
{
  int cnt;
  
  if (semTake (semMEI,60)!=ERROR)
  {
    cnt=frames_to_execute(axis<<1);
    semGive (semMEI);
    return cnt;    
  }
  printf("\r\ntm_frames_to_execute error");
  return ERROR;    
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
void tm_TCC(int axis)
{
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
  extern int axis_alive;
  
  tm_controller_run (axis<<1);
  idx=0;
  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
	ticks_per_degree[axis]);
  FOREVER
  {
/* task should idle here with no input pvt */
    while (axis_queue[axis].active==NULL)
    {
      axis_alive |= (1<<axis);	/* keep alive bit */
/* in case drifting, no new pvt, and need to stop */
      if (frame_break[axis])
      {
        stp_frame(axis,stop_position[axis],(double)ticks_per_degree[axis]);
	frame_break[axis]=FALSE;
      }
      taskDelay (3);
    }
    frame=axis_queue[axis].active;
    drift_break[axis]=FALSE;

/* reposition if neccessary */
    semTake (semMEI,WAIT_FOREVER);
    get_position(axis<<1,&position);
    get_velocity(axis<<1,&velocity);
    semGive (semMEI);
    pos=(long)position;
/*      printf("\r\nCheck Params for repositioning");*/
    if ( (abs((long)((frame->position*ticks_per_degree[axis])-position))>
	(long)(.01*ticks_per_degree[axis])) && (fabs(velocity)==0) )
    {
      while ((lcnt=tm_frames_to_execute(axis))>1)
      {
        printf ("\r\n frames left=%d",lcnt);
        taskDelay(1);
      }
      tm_start_move (axis<<1,
		1*(double)ticks_per_degree[axis],
		.05*(double)ticks_per_degree[axis],
		frame->position*(double)ticks_per_degree[axis]);
      printf ("\r\nAxis %d Repositioning TCC cmd to pos=%lf from pos=%lf\r\n diff=%ld>%ld",
		axis,frame->position*ticks_per_degree[axis],position,
		abs((frame->position*ticks_per_degree[axis])-position),
		(long)(.01*ticks_per_degree[axis]) );
      status=TRUE;
      while ((abs((frame->position*ticks_per_degree[axis])-position)>
		(long)(.01*ticks_per_degree[axis]))&&status)
      {
        taskDelay (10);
        if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
        {
	  status=in_motion(axis<<1);
          get_position(axis<<1,&position);
          semGive (semMEI);
          pos=(long)position;
        }
/*        printf("\r\nrepositioning to %lf, status=%d",position,status);*/
      }
      printf("\r\nDone repositioning to %lf",position);
    }
/*    else
      printf("\r\n nonzero vel=%lf",velocity);*/

/* check for time */
    while ((frame!=NULL)&&
	  (sdss_delta_time(frame->end_time,sdss_get_time())<0.0))
    {
      frame = frame->nxt;
      axis_queue[axis].active=frame;
      printf ("\r\n Frame deleted due to time");
    }

    if (frame!=NULL)
    {
      start_frame (axis,frame->end_time);
      while ((frame->nxt==NULL)&&
	    (sdss_delta_time(frame->end_time,sdss_get_time())>0.02))
      {
/*        printf ("\r\n waiting for second frame");*/
        taskDelay (3);
      }
      while ( (frame->nxt!=NULL) || (axis_queue[axis].active!=NULL) &&
	((!frame_break)&&(!drift_break)) )
      {
        frame_cnt=get_frame_cnt(axis,frame);
/*        printf ("\r\n frames_cnt=%d",frame_cnt);*/
        frame_idx=0;
	while (frame_cnt>0)
        {
          while ( ((cnt=calc_frames(axis,frame,frame_idx))==ERROR)&&
            ((lcnt=tm_frames_to_execute(axis))>4) ) taskDelay (1);
	  if (cnt==ERROR)
	  {
            frame_break[axis]=TRUE;
            printf ("\r\n frame=%p, nxt=%p, nxt=%p, frame_cnt=%d",
	      frame,frame->nxt,(frame->nxt)->nxt,frame_cnt);
	  }
	  else
	  {
/* OFFSET */
            for (i=0;i<OFF_MAX;i++)
  	    {
	      clroffset(axis,cnt);
	      if (offset_queue_end[axis][i]!=NULL)
	      {
	        cntoff=calc_offset(axis,&offset[axis][i][0],
					offset_idx[axis][i],cnt);
	        offset_idx[axis][i]+=cnt;
  	        if ((offset_idx[axis][i]/20.)>offset[axis][i][1].end_time)
	        {
/*	          printf ("\r\nShutdown offset");*/
                  frmoff=frame;
	          taskLock();
                  while (frmoff!=offset_queue_end[axis][i])
                  {
	            frmoff->position+=offset[axis][i][1].position;
	            frmoff->velocity+=offset[axis][i][1].velocity;
	            frmoff=frmoff->nxt;
/*	            printf ("\r\noffset end=%p, pos=%lf,idx=%d",frmoff,
		      frmoff->position,offset_idx[axis][i]);*/
	          }
                  frmoff->position+=offset[axis][i][1].position;
	          frmoff->velocity+=offset[axis][i][1].velocity;
/*                  printf ("\r\noffset end=%p, pos=%lf,idx=%d",frmoff,
		      frmoff->position,offset_idx[axis][i]);*/
	          offset_queue_end[axis][i]=NULL;
	          taskUnlock();
	        }
	      }
	      addoffset(axis,cnt);
	    }
            frame_idx += cnt;
            frame_cnt -= cnt;
	  }

	  if (frame_break[axis]) 
	  {
/*	    printf ("\r\nFRAME_BREAK");*/
            axis_queue[axis].active=NULL;
	    frame_cnt=0;
            break;
	  }
	  if (drift_break[axis]) 
	  {
/*	    printf ("\r\nDRIFT_BREAK");*/
            axis_queue[axis].active=NULL;
	    frame_cnt=0;
            break;
	  }

	  idx=0;
	  while (cnt>0)
          {
	    if (frame_break[axis]) 
	    {
/*	      printf ("\r\nFRAME_BREAK");*/
              axis_queue[axis].active=NULL;
 	      frame_cnt=0;
	      cnt=0;
              break;
	    }
	    if (drift_break[axis]) 
	    {
/*	      printf ("\r\nDRIFT_BREAK");*/
              axis_queue[axis].active=NULL;
	      frame_cnt=0;
	      cnt=0;
              break;
	    }
	    if (cnt>0)
	    {
              load_frames(axis,min(cnt,5),idx,(double)ticks_per_degree[axis]);
	      if ((idx==15)&&(cnt==5)) printf ("\r\n p=%lf",p[axis][19]);
              while ((lcnt=tm_frames_to_execute(axis))>10) taskDelay (3);
	      idx+=5;
	      cnt -=5;
	    }
          }
        }
        if (axis_queue[axis].active==NULL) 
        {
	  frame=axis_queue[axis].end;
	  break;
	}
        frame = frame->nxt;
        axis_queue[axis].active=frame;    
        while ((frame->nxt==NULL)&&
	    (sdss_delta_time(frame->end_time,sdss_get_time())>.02))
          taskDelay (1);
        while ((frame->nxt==NULL)&&((lcnt=tm_frames_to_execute(axis))>1)) 
	  taskDelay(1);
      }
      lcnt=tm_frames_to_execute(axis);;
      printf ("\r\n Ran out: frames left=%d",lcnt);
      taskLock();
      axis_queue[axis].active=NULL;    
      frame=axis_queue[axis].end;
      for (i=0;i<OFF_MAX;i++)
      {
        offset_idx[axis][i]=0;
	offset_queue_end[axis][i]=NULL;
      }
      taskUnlock();
      if (idx<=0) idx=1;
      if (frame_break[axis])
      {
        stp_frame(axis,stop_position[axis],(double)ticks_per_degree[axis]);
	frame_break[axis]=FALSE;
      }
      else
      {
        if (drift_break[axis])
        {
          drift_frame(axis,drift_velocity[axis],(double)ticks_per_degree[axis]);
        }
        else
	{
          end_frame(axis,idx-1,(double)ticks_per_degree[axis]);
	}
      }
    }
    else
      printf ("\r\nRestart no frames to process");
  }
}
void start_tm_TCC()
{
  taskSpawn("tmAz",47,VX_FP_TASK,20000,(FUNCPTR)tm_TCC,
		0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlt",47,VX_FP_TASK,20000,(FUNCPTR)tm_TCC,
		1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRot",47,VX_FP_TASK,20000,(FUNCPTR)tm_TCC,
		2,0,0,0,0,0,0,0,0,0);
}
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

  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
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
void start_tm_TCC_test()
{
  taskSpawn("tmAztest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlttest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRottest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		2,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_axis_queue
**
** DESCRIPTION:
**	Diagnostic for display the last 100 pvts for a specified axis.
**
** RETURN VALUES:
**	return always zero
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	axis_queue
**
**=========================================================================
*/
int print_axis_queue(int axis)
{
  struct FRAME *frame;
  struct FRAME_QUEUE *queue;

  printf ("\r\nList Axis Queue=%d: %p",axis,&axis_queue[axis]);
  queue = &axis_queue[axis];
  frame = (struct FRAME *)queue->top;

  while (frame!=NULL)
  {		/* end of queue, and becomes active frame */
    if (frame==queue->top) printf ("\r\nTOP, cnt=%d",queue->cnt);
    if (frame==queue->active) printf ("\r\nACTIVE");
    if (frame==queue->end) printf ("\r\nEND");
    printf ("\r\n %p: position=%12.8lf, velocity=%12.8lf, end_time=%12.8f",frame,
    		frame->position,
  		frame->velocity,
		frame->end_time);
    if (frame->nxt!=NULL)
    {
      printf ("\r\n      deltas position=%12.8lf, velocity=%12.8lf, end_time=%12.8f",
                frame->nxt->position-frame->position,
                frame->nxt->velocity-frame->velocity,
                frame->nxt->end_time-frame->end_time);
    }
    frame = frame->nxt;
  }
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: axis_DIO316_shutdown
**
** DESCRIPTION:
**	Software reboot shutdown of DIO316 hardware.  Disables interrupts.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DIO316
**
**=========================================================================
*/
void axis_DIO316_shutdown(int type)
{
    printf("AXIS DIO316 Shutdown:  4 interrupts %d\r\n",tm_DIO316);
    if (tm_DIO316!=-1)
    {
      DIO316_Interrupt_Enable_Control (tm_DIO316,0,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_DIS);
    }
    taskDelay(30);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: DIO316_interrupt
**
** DESCRIPTION:
**	Interrupt handler.  Used to handle function generator driven 1 Hz
**	pulse for testing.  Used for monitoring a crossing of a fiducial
**	and triggers a task via a semaphore.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DIO316
**	semLATCH
**
**=========================================================================
*/
unsigned long int_count=0;
float latchpos4,latchpos5;
int lpos4,lpos5;
int illegal_NIST=0;
unsigned char dio316int_bit=0;
void DIO316_interrupt(int type)
{

	  int_count++;
          DIO316ReadISR (tm_DIO316,&dio316int_bit);
          if (dio316int_bit&NIST_INT)
          {
	    illegal_NIST++;
            DIO316ClearISR (tm_DIO316);
          }
	  else
	  {
            if (dio316int_bit&AZIMUTH_INT) 
	      DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_DIS);
            if (dio316int_bit&ALTITUDE_INT) 
	      DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_DIS);
            if (dio316int_bit&INSTRUMENT_INT) 
	      DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_DIS);
	    semGive (semLATCH);
	  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: init_fiducial 
**	    tm_latch
**
** DESCRIPTION:
**	Initialize the fiducials marking all invalid and restoring known
**	positions from shared memory.  Always update the fixed fiducial
**	for each axis to a known point.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**	restore_fiducials
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
/* structure for the primary set of fiducials - one per axis */
struct FIDUCIARY fiducial[3]=
/*	NotValid, mark, index */
	{{FALSE,0,9+24},
	  {FALSE,0,1},
	  {FALSE,0,40+43}};
/* stow     120:44:15.8
   fiducial 120:45:44.9
   median   000:21:59.4
*/
/*		           120:45:44.9, 14:39:23:286, 005:58:04:130  */
/*			   AZ           , ALT         , ROT      */
long fiducial_position[3]={31016188     , 3766415+58807     , 336370/2};
/*long fiducial_position[3]={31016188     , 3766415+58807     , 402702};*/

/* structure for all the fiducials */
struct FIDUCIALS az_fiducial[48];
struct FIDUCIALS alt_fiducial[7];
struct FIDUCIALS rot_fiducial[156];
long az_fiducial_position[60];
long alt_fiducial_position[7];
long rot_fiducial_position[156];
long rot_latch=0;	/* need two latches or a valid last point */
int fiducialidx[3]={-1,-1,-1};		/* last fiducial crossed */
void init_fiducial()
{
  int i;

  for (i=0;i<sizeof(az_fiducial)/sizeof(struct FIDUCIALS);i++)
  {
    az_fiducial[i].markvalid=FALSE;
    az_fiducial[i].last=0;
    az_fiducial[i].err=0;
    az_fiducial[i].poserr=0;
    az_fiducial[i].mark=0;
    az_fiducial_position[i]=0;
  }
  restore_fiducials (0);
  az_fiducial_position[fiducial[0].index]=fiducial_position[0];	/* 120:49:20:00 */
  for (i=0;i<sizeof(alt_fiducial)/sizeof(struct FIDUCIALS);i++)
  {
    alt_fiducial[i].markvalid=FALSE;
    alt_fiducial[i].last=0;
    alt_fiducial[i].err=0;
    alt_fiducial[i].poserr=0;
    alt_fiducial[i].mark=0;
    alt_fiducial_position[i]=0;
  }
  restore_fiducials (1);
	/* 14:39:23:286 */
  alt_fiducial_position[fiducial[1].index]=fiducial_position[1];
  alt_fiducial_position[0]=0x0;	/* 00:00:00:00 */
  alt_fiducial_position[6]=0x0160E6C6;	/* 090:00:00:00 */
  for (i=0;i<sizeof(rot_fiducial)/sizeof(struct FIDUCIALS);i++)
  {
    rot_fiducial[i].markvalid=FALSE;
    rot_fiducial[i].last=0;
    rot_fiducial[i].err=0;
    rot_fiducial[i].poserr=0;
    rot_fiducial[i].mark=0;
    rot_fiducial_position[i]=0;
  }
  restore_fiducials (2);
   	/* 001:13:35:373 */
  rot_fiducial_position[fiducial[2].index]=fiducial_position[2];
}
static char const thePath[] = "/mcptpm/";
/*------------------------------------------------------------------------------
  createNfsConnection

  This function sets up the NFS connection to 'sdsshost'. It also
  changes the directory.
------------------------------------------------------------------------------*/
static STATUS createNfsConnection(void)
{
/*        int const NESWOLD = 6295;*/
        int const BRIEGEL = 1131;
        int const CONTROLS = 1522;

        /* First, set up our NFS authentication parameters. We'll appear
       as 'neswold' from the 'controls' group. These IDs are obtained
       from 'sdsshost.apo.nmsu.edu'. */

        nfsAuthUnixSet("sdssmcp", BRIEGEL, CONTROLS, 0, 0);

        /* Attempt to mount the remote drive and set the default
       directory. */

        if (OK == nfsMount("sdsshost", (char*) thePath, 0))
                return OK;
        else {
	  printErrno(errno);
                return ERROR;
        }
}
/*------------------------------------------------------------------------------
  destroyNfsConnection
------------------------------------------------------------------------------*/
static STATUS destroyNfsConnection(void)
{
        return nfsUnmount((char*) thePath);
}
/* This variable holds the default path to where the log files will be
   held. There is code that expects this string to contain the
   trailing '/' */

/*------------------------------------------------------------------------------
  logFileName
------------------------------------------------------------------------------*/
static char const* bldFileName(char const* newName)
{
        static char buffer[1024] = "";

        /* If the new name isn't NULL, then we'll be modifying the static
       value. If it is NULL, we'll just return the current
       contents. */

        if (newName)

                /* If the string isn't empty, then build up a path (the file
           must reside in the /mcptpm directory!) If the string is
           empty, we clear out the static value. */

                if (*newName)
                        sprintf(buffer, "%s%.*s", thePath,
                                        (int) (sizeof(buffer) - sizeof(thePath)
- 1), newName);
                else
                        buffer[0] = '\0';
        return buffer;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: fiducial_shutdown
**
** DESCRIPTION:
**	Flush the file and close it.
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
void fiducial_shutdown(int type)
{
  printf("fiducial file shutdown: FP=%p\r\n",fidfp);
  fflush (fidfp);
  fclose (fidfp);
  destroyNfsConnection();
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_latch
**
** DESCRIPTION:
**	The azimuth has 48 entries for +-360 degrees.  The azimuth uses small
**	segments of optical tape with reference marks to interrupt the MCP.
**	A barcode reader is triggered to read a corresponding UPC which 
**	contains an absolute index.  The position must be aligned within
**	approximately 120 deg for this mechanism to work since the code
**	must determine if the azimuth is wrapped or not.
**	The altitude has 7 entries for 0-90 degrees.  The altitude also uses
**	segments of optical tape with reference marks to interrupt the MCP, 
**	but the barcode reader is disabled (failed).  The clinometer is used
**	to judge the index for the reference crossing.  The clinometer must
**	be accurate to with +-7 degrees for this to work correctly.
**	The rotator has 156 entries to exceed +-360 degrees.  The rotator
**	uses the same optical tape but it is continuous and provides an
**	absolute unique value between any two reference marks.  This value
**	is adjusted as an index into the table.
**	tm_latch is the routine triggered by the semaphore and processes
**	the reference corresponding to the axis causing the trigger.  Note:
**	when a latch occurs in the MEI, all axis are latched.  Also, if
**	numberous interrupts occur due to setting on the reference mark, then
**	the next interrupt is delayed by disabling the interrupt and not
**	enabling until a delay is expired.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**	barcode_serial
**
** GLOBALS REFERENCED:
**	semMEI
**	sdssdc
**	latchpos
**	fiducial
**	fiducialidx
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**	altclino_off, altclino_sf
**
**=========================================================================
*/
void tm_latch(char *name)
{
  int i;
  extern int barcode_serial();
  int fididx;
  int fididx1;
  int status;
  extern struct SDSS_FRAME sdssdc;
  extern int altclino_off;
  extern float altclino_sf;
  time_t fidtim;

  if ((status=createNfsConnection())==ERROR) printf ("\r\nNFSConnection error");
  fidfp=fopen (bldFileName(name),"a");  /*  */
  if (fidfp==NULL) 
    printf ("\r\nOpen file error: %s",name);
  else
  {
    printf ("\r\nOpen file %s; %p",name,fidfp);
/*    setvbuf (fidfp,NULL,_IOLBF,0);*/
/*    rebootHookAdd((FUNCPTR)fiducial_shutdown);*/
    time (&fidtim);
    fprintf (fidfp,"\n#RESTART......... %s %.24s",
	bldFileName(name),ctime(&fidtim));
    fprintf (fidfp,"\n#Axis\tIndex\tDate & Time:SDSStime\tPosition1\tPosition2");
/* should use fflush, but doesn't work */
    fclose(fidfp);
    fidfp=fopen (bldFileName(name),"a");  /*  */
/*    fidfp=freopen (bldFileName(name),"a",fidfp);*/
  }
  init_fiducial();
  if (semLATCH==NULL) semLATCH = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
  for (;;)
  {
    if (semTake(semLATCH,WAIT_FOREVER)!=ERROR)
    {
      i=15;
      status=FALSE;
      while ((!status)&&(i>0))
      {
        if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
        {
          status=(int)latch_status;
          semGive (semMEI);
        }
        taskDelay(1);
        i--;
      }
      if (status)
      {
        fididx=-1;
        latchpos[latchidx].axis=-9;
        if (dio316int_bit&AZIMUTH_INT)
        {
	  latchpos[latchidx].axis=AZIMUTH;
          if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
          {
	    get_latched_position(0,&latchpos[latchidx].pos1);
	    get_latched_position(1,&latchpos[latchidx].pos2);
	    semGive (semMEI);
	  }
          if (LATCH_verbose)
            printf ("\r\nAXIS %d: latched pos0=%f,pos1=%f",latchpos[latchidx].axis,
	    (float)latchpos[latchidx].pos1,
	    (float)latchpos[latchidx].pos2);
          fididx1 = barcode_serial(3);	/* backwards from what you would think */
	  fididx = barcode_serial(3);	/* read twice...not reliable */
	  if ((fididx>0)&&(fididx<=24))
	  {
	    if (latchpos[latchidx].pos1>0)
		fididx += 24;
	    fidtim=time(&fidtim);
            if (fidfp!=NULL)
	    {
   	      time (&fidtim);
	      fprintf (fidfp,"\n%d\t%d\t%.24s:%f\t%ld\t%ld",
	      latchpos[latchidx].axis,fididx,
	      ctime(&fidtim),sdss_get_time(),
	      (long)latchpos[latchidx].pos1,
              (long)latchpos[latchidx].pos2);
	      fprintf (fidfp,"\n#first barcode reading=%d",fididx1);
              fclose(fidfp);
              fidfp=fopen (bldFileName(name),"a");  /*  */
/*    fidfp=freopen (bldFileName(name),"a",fidfp);*/
            }
            if ((fididx<48)&&(fididx>0))
            {
              az_fiducial[fididx].last=az_fiducial[fididx].mark;
              az_fiducial[fididx].mark=latchpos[latchidx].pos1;
	      az_fiducial[fididx].err=az_fiducial[fididx].mark-
		az_fiducial[fididx].last;
	      az_fiducial[fididx].poserr=az_fiducial[fididx].mark-
		az_fiducial_position[fididx];
	      az_fiducial[fididx].markvalid=TRUE;
	      if ((abs(az_fiducial[fididx].poserr)>errmsg_max[0])&&
		      (az_fiducial_position[fididx]!=0))
                printf ("\r\nAXIS %d: ERR=%ld, latched pos0=%f,pos1=%f",
		  latchpos[latchidx].axis,
	          (long)az_fiducial[fididx].poserr,
	          (float)latchpos[latchidx].pos1,
	          (float)latchpos[latchidx].pos2);
              if (fididx==fiducial[0].index)
              {
                fiducial[0].mark=az_fiducial[fididx].mark;
	        fiducial[0].markvalid=TRUE;
              }
	      fiducialidx[0]=fididx;
	    }
	  }
	}
        if (dio316int_bit&ALTITUDE_INT)
        {
	  latchpos[latchidx].axis=ALTITUDE;
          if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
          {
	    get_latched_position(2,&latchpos[latchidx].pos1);
	    get_latched_position(3,&latchpos[latchidx].pos2);
            semGive (semMEI);
          }
          if (LATCH_verbose)
          printf ("\r\nAXIS %d: latched pos2=%f,pos3=%f",latchpos[latchidx].axis,
	    (float)latchpos[latchidx].pos1,
	    (float)latchpos[latchidx].pos2);
/*  turned off (failed hardware)
          fididx = barcode_serial(2);
	  fididx = barcode_serial(2);
*/
/* clinometer does a better job */
          fididx=((int)(abs(sdssdc.status.i4.alt_position-altclino_off)*
            altclino_sf)+7.5)/15;
	  fididx++;
	  if (fididx!=-1)
	  {
	    fididx--;
	    fidtim=time(&fidtim);
            if (fidfp!=NULL)
	    {
	      time (&fidtim);
	      fprintf (fidfp,"\n%d\t%d\t%.24s:%f\t%ld\t%ld",
	      latchpos[latchidx].axis,fididx,
	      ctime(&fidtim),sdss_get_time(),
	      (long)latchpos[latchidx].pos1,
              (long)latchpos[latchidx].pos2);
	      fprintf (fidfp,"\n#alt_position=%d",
	      sdssdc.status.i4.alt_position);
              fclose(fidfp);
              fidfp=fopen (bldFileName(name),"a");  /*  */
/*    fidfp=freopen (bldFileName(name),"a",fidfp);*/
	    }
            if ((fididx<7)&&(fididx>=0))
            {
              alt_fiducial[fididx].last=alt_fiducial[fididx].mark;
              alt_fiducial[fididx].mark=latchpos[latchidx].pos1;
	      alt_fiducial[fididx].err=alt_fiducial[fididx].mark-
		alt_fiducial[fididx].last;
	      alt_fiducial[fididx].poserr=alt_fiducial[fididx].mark-
		alt_fiducial_position[fididx];
	      alt_fiducial[fididx].markvalid=TRUE;
	      if ((abs(alt_fiducial[fididx].poserr)>errmsg_max[1])&&
		      (alt_fiducial_position[fididx]!=0))
                printf ("\r\nAXIS %d: ERR=%ld, latched pos0=%f,pos1=%f",
		  latchpos[latchidx].axis,
	          (long)alt_fiducial[fididx].poserr,
	          (float)latchpos[latchidx].pos1,
	          (float)latchpos[latchidx].pos2);
              if (fididx==fiducial[1].index)
              {
                fiducial[1].mark=alt_fiducial[fididx].mark;
                fiducial[1].markvalid=TRUE;
              }
	      fiducialidx[1]=fididx;
	    }
	  }
	}
        if (dio316int_bit&INSTRUMENT_INT)
        {
	  latchpos[latchidx].axis=INSTRUMENT;
          if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
          {
#ifdef ROT_ROTARY_ENCODER
/* switch to 5 for optical encoder, when using rotary */
	    get_latched_position(5,&latchpos[latchidx].pos1);
	    get_latched_position(4,&latchpos[latchidx].pos2);
#else
	    get_latched_position(4,&latchpos[latchidx].pos1);
	    get_latched_position(5,&latchpos[latchidx].pos2);
#endif
            semGive (semMEI);
          }
          if (LATCH_verbose)
            printf ("\r\nAXIS %d: latched pos4=%f,pos5=%f",
	      latchpos[latchidx].axis,
	      (float)latchpos[latchidx].pos1,
	      (float)latchpos[latchidx].pos2);
	  if (rot_latch!=0)
          {
            if (abs((long)latchpos[latchidx].pos1-rot_latch)>250000)
	      fididx = abs(iround((latchpos[latchidx].pos1-rot_latch)/800.) )-500;
	    else
	      fididx = 0;
            if (LATCH_verbose)
              printf ("\r\nAXIS %d: latched pos4=%ld,rot_latch=%ld,idx=%d, abspos=%d",
                latchpos[latchidx].axis,
                (long)latchpos[latchidx].pos1,
                rot_latch,fididx,
	        abs(iround((latchpos[latchidx].pos1-rot_latch)/800.) ));
	    if ((fididx<0)&&(fididx>-80))
	    {
	      fididx = -fididx;
	      if ((fididx>45)&&(latchpos[latchidx].pos1>0))
	        fididx = fididx-76;
	      else 
	        if ((fididx<35)&&(latchpos[latchidx].pos1<0))
	          fididx = 76+fididx;
	      fididx +=45;
	      fidtim=time(&fidtim);
              if (fidfp!=NULL)
	      {
	        time (&fidtim);
	        fprintf (fidfp,"\n%d\t%d\t%.24s:%f\t%ld\t%ld",
	        latchpos[latchidx].axis,fididx,
	        ctime(&fidtim),sdss_get_time(),
	        (long)latchpos[latchidx].pos1,
                (long)latchpos[latchidx].pos2);
                fclose(fidfp);
                fidfp=fopen (bldFileName(name),"a");  /*  */
/*    fidfp=freopen (bldFileName(name),"a",fidfp);*/
	      }
              if (LATCH_verbose)
	        printf ("\r\n      final fididx=%d",fididx);
	      if ((fididx<156)&&(fididx>0))
	      {
/*	        rot_fiducial[fididx].last=rot_fiducial[fididx].mark;*/
                rot_fiducial[fididx].mark=
	          max((long)latchpos[latchidx].pos1,rot_latch);
/*	        rot_fiducial[fididx].err=rot_fiducial[fididx].mark-
		  rot_fiducial[fididx].last;*/
	        rot_fiducial[fididx].poserr=rot_fiducial[fididx].mark-
		  rot_fiducial_position[fididx];
      	        rot_fiducial[fididx].markvalid=TRUE;
                fiducialidx[2]=fididx;
	        if ((abs(rot_fiducial[fididx].poserr)>errmsg_max[2])&&
		      (rot_fiducial_position[fididx]!=0))
                  printf ("\r\nAXIS %d: ERR=%ld, latched pos0=%f,pos1=%f",
	  	    latchpos[latchidx].axis,
	            (long)rot_fiducial[fididx].poserr,
	            (float)latchpos[latchidx].pos1,
	            (float)latchpos[latchidx].pos2);
	      }
	    }
	    else
            {
	      if ((fididx>0)&&(fididx<80))
              {
	        if ((fididx>45)&&(latchpos[latchidx].pos1>0))
	          fididx = fididx-76;
	        else 
		  if ((fididx<35)&&(latchpos[latchidx].pos1<0))
	            fididx = 76+fididx;
	        fididx +=45;
	        fidtim=time(&fidtim);
                if (fidfp!=NULL)
	        {
	          time (&fidtim);
	          fprintf (fidfp,"\n%d\t%d\t%.24s:%f\t%ld\t%ld",
	          latchpos[latchidx].axis,fididx,
	          ctime(&fidtim),sdss_get_time(),
	          (long)latchpos[latchidx].pos1,
                  (long)latchpos[latchidx].pos2);
                  fclose(fidfp);
                  fidfp=fopen (bldFileName(name),"a");  /*  */
/*    fidfp=freopen (bldFileName(name),"a",fidfp);*/
		}
                if (LATCH_verbose)
	          printf ("\r\n      final fididx=%d",fididx);
	        if ((fididx<156)&&(fididx>0))
	        {
	          rot_fiducial[fididx].last=rot_fiducial[fididx].mark;
                  rot_fiducial[fididx].mark=
                    min((long)latchpos[latchidx].pos1,rot_latch);
	          rot_fiducial[fididx].err=rot_fiducial[fididx].mark-
	  	    rot_fiducial[fididx].last;
	          rot_fiducial[fididx].poserr=rot_fiducial[fididx].mark-
	 	    rot_fiducial_position[fididx];
                  rot_fiducial[fididx].markvalid=TRUE;
                  fiducialidx[2]=fididx;
	          if (LATCH_error)
	            if ((abs(rot_fiducial[fididx].poserr)>200)&&
		      (rot_fiducial_position[fididx]!=0))
                      printf ("\r\nAXIS %d: ERR=%ld, latched pos0=%f,pos1=%f",
	  	      latchpos[latchidx].axis,
	              (long)rot_fiducial[fididx].poserr,
	              (float)latchpos[latchidx].pos1,
	              (float)latchpos[latchidx].pos2);
                }
              }
            }
          }
          if (LATCH_verbose)
	  {
            i=fididx;
            printf ("\r\nROT FIDUCIAL %d:  mark=%ld, pos=%ld, last=%ld",i,
                rot_fiducial[i].mark,rot_fiducial_position[i],
                rot_fiducial[i].last);
            printf ("\r\n                  err=%ld, poserr=%ld",
                rot_fiducial[i].err,rot_fiducial[i].poserr);
	  }
          rot_latch=latchpos[latchidx].pos1;
          if (fididx==fiducial[2].index)
          {
            fiducial[2].mark=rot_fiducial[fididx].mark;
            fiducial[2].markvalid=TRUE;
          }
	}
      }
      else
      {
        latchpos[latchidx].axis=-9;
        if (dio316int_bit&AZIMUTH_INT)
          latchpos[latchidx].axis=-(AZIMUTH+1);
        if (dio316int_bit&ALTITUDE_INT)
          latchpos[latchidx].axis=-(ALTITUDE+1);
        if (dio316int_bit&INSTRUMENT_INT)
          latchpos[latchidx].axis=-(INSTRUMENT+1);
        printf ("\r\n BAD LATCH: latchidx=%d",latchidx);
      }
    }
    if (latchidx<MAX_LATCHED)
      latchidx++;
    else
      latchidx=0;
/*
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
      arm_latch(TRUE);
      semGive (semMEI);
    }
*/
    DIO316ClearISR (tm_DIO316);
    taskSpawn ("tm_ClrInt",30,8,4000,(FUNCPTR)DIO316ClearISR_delay,120,
		dio316int_bit,0,0,0,0,0,0,0,0);
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: DIO316ClearISR_delay
**
** DESCRIPTION:
**	Task is spawned to delay enabling interrupt and arming latch
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DIO316
**	semMEI
**
**=========================================================================
*/
void DIO316ClearISR_delay (int delay, int bit)
{
  int status;

  taskDelay(delay);

  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    while ((status=arm_latch(TRUE))!=DSP_OK) 
	printf ("\r\n Trying to ARM Latch; status=%d",status);
    semGive (semMEI);
  }
  if (bit&AZIMUTH_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_ENA);
  if (bit&ALTITUDE_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_ENA);
  if (bit&INSTRUMENT_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_ENA);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: set_primary_fiducials
**
** DESCRIPTION:
**	Each axis has one primary fiducial and it can be adjusted or changed
**	to a new index.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	fiducial  - structure for the primary set of fiducials
**	fiducial_position
**
**=========================================================================
*/
void set_primary_fiducials (int axis,int fididx,long pos)
{
  switch (axis)
  {
    case 0:
	if ((fididx<48)&&(fididx>=0))
	{
	  fiducial[axis].index=fididx;
	  fiducial[axis].markvalid=FALSE;
	  fiducial[axis].mark=fididx;
	  fiducial_position[axis]=pos;
	}
        az_fiducial_position[fiducial[axis].index]=fiducial_position[axis];
	break;	  
    case 1:
	if ((fididx<7)&&(fididx>=0))
	{
	  fiducial[axis].index=fididx;
	  fiducial[axis].markvalid=FALSE;
	  fiducial[axis].mark=fididx;
	  fiducial_position[axis]=pos;
	}
        alt_fiducial_position[fiducial[axis].index]=fiducial_position[axis];
	break;	  
    case 2:
	if ((fididx<156)&&(fididx>=0))
	{
	  fiducial[axis].index=fididx;
	  fiducial[axis].markvalid=FALSE;
	  fiducial[axis].mark=fididx;
	  fiducial_position[axis]=pos;
	}
        rot_fiducial_position[fiducial[axis].index]=fiducial_position[axis];
	break;	  
  }  
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: set_fiducials_all
**	    set_fiducials
**
** DESCRIPTION:
**	Set the last passed position as the "known" position for the fiducial
**	if it was marked as valid.  This does not automatically save these
**	settings to shared memory and the fiducials should be calibrated
**	before executing this function.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
void set_fiducials_all ()
{
  int i;

  for (i=0;i<3;i++)
    set_fiducials (i);
}
void set_fiducials (int axis)
{
  int i;

  switch (axis)
  {
    case 0:
        for (i=0;i<48;i++)
        {
          if (az_fiducial[i].markvalid)
	  {
            az_fiducial_position[i]=az_fiducial[i].mark;
	    az_fiducial[i].poserr=0;
	  }
        }
        break;
    case 1:
        for (i=0;i<7;i++)
        {
          if (alt_fiducial[i].markvalid)
	  {
            alt_fiducial_position[i]=alt_fiducial[i].mark;
	    alt_fiducial[i].poserr=0;
          }
        }
        break;
    case 2:
        for (i=0;i<156;i++)
        {
          if (rot_fiducial[i].markvalid)
	  {
            rot_fiducial_position[i]=rot_fiducial[i].mark;
            rot_fiducial[i].poserr=0;
          }
        }
        break;
    }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: save_fiducials_all
**	    save_fiducials
**	    restore_fiducials_all
**	    restore_fiducials
**
** DESCRIPTION:
**	Save/restore to/from shared memory the fiducials.  
**	Does nothing with the primary
**	fiducials...which is an artifact of getting fiducials to work and
**	is becoming the working scenario.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
#define SM_AZ_FIDUCIALS	0x02810000
#define SM_ALT_FIDUCIALS	0x02811000
#define SM_ROT_FIDUCIALS	0x02812000
void save_fiducials_all ()
{
  int i;

  for (i=0;i<3;i++)
    save_fiducials (i);
}
void save_fiducials (int axis)
{
  int i;
  long *sm;

  switch (axis)
  {
    case 0:
	sm = (long *)SM_AZ_FIDUCIALS;
        for (i=0;i<48;i++)
          sm[i]=az_fiducial_position[i];
        break;
    case 1:
	sm = (long *)SM_ALT_FIDUCIALS;
        for (i=0;i<7;i++)
          sm[i]=alt_fiducial_position[i];
        break;
    case 2:
	sm = (long *)SM_ROT_FIDUCIALS;
        for (i=0;i<156;i++)
          sm[i]=rot_fiducial_position[i];
        break;
    }
}
void restore_fiducials_all ()
{
  int i;

  for (i=0;i<3;i++)
    restore_fiducials (i);
}
void restore_fiducials (int axis)
{
  int i;
  long *sm;

  switch (axis)
  {
    case 0:
	sm = (long *)SM_AZ_FIDUCIALS;
        for (i=0;i<48;i++)
          az_fiducial_position[i]=sm[i];
        break;
    case 1:
	sm = (long *)SM_ALT_FIDUCIALS;
        for (i=0;i<7;i++)
          alt_fiducial_position[i]=sm[i];
        break;
    case 2:
	sm = (long *)SM_ROT_FIDUCIALS;
        for (i=0;i<156;i++)
          rot_fiducial_position[i]=sm[i];
        break;
    }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_fiducials
**
** DESCRIPTION:
**	Diagnostic to see the fiducial positions and errors.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
void print_fiducials (int axis)
{
  int i;

  switch (axis)
  {
    case 0:
	for (i=0;i<48;i++)
	{
	  printf ("\r\n");
	  if (fiducial[axis].index==i)
	  {
            printf ("*");
	    if (fiducial[axis].markvalid==i) printf ("!");
	  }
          if (az_fiducial[i].markvalid)
	  {
	    printf ("AZ FIDUCIAL %d(%d degs):  mark=%ld, pos=%ld, last=%ld",i,
		(int)(az_fiducial[i].mark/AZ_TICKS_DEG),
		az_fiducial[i].mark,az_fiducial_position[i], az_fiducial[i].last);
	    printf ("\r\n                 err=%ld, poserr=%ld",
		az_fiducial[i].err,az_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("AZ FIDUCIAL %d:  pos=%ld",i,
		az_fiducial_position[i]);
	  }     
	}
	break;
    case 1:
	for (i=0;i<7;i++)
	{
	  printf ("\r\n");
	  if (fiducial[axis].index==i)
	  {
            printf ("*");
	    if (fiducial[axis].markvalid==i) printf ("!");
	  }
          if (alt_fiducial[i].markvalid)
	  {
	    printf ("ALT FIDUCIAL %d(%d degs):  mark=%ld, pos=%ld, last=%ld",i,
		(int)(alt_fiducial[i].mark/ALT_TICKS_DEG),
		alt_fiducial[i].mark,alt_fiducial_position[i],
	        alt_fiducial[i].last);
	    printf ("\r\n                 err=%ld, poserr=%ld",
		alt_fiducial[i].err,alt_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("ALT FIDUCIAL %d:  pos=%ld",i,
		alt_fiducial_position[i]);
	  }     
	}
	break;
    case 2:
	for (i=0;i<156;i++)
	{
	  printf ("\r\n");
	  if (fiducial[axis].index==i)
	  {
            printf ("*");
	    if (fiducial[axis].markvalid==i) printf ("!");
	  }
          if (rot_fiducial[i].markvalid)
	  {
	    printf ("ROT FIDUCIAL %d(%d degs):  mark=%ld, pos=%ld, last=%ld",i,
		(int)(rot_fiducial[i].mark/ROT_TICKS_DEG),
		rot_fiducial[i].mark,rot_fiducial_position[i],
	        rot_fiducial[i].last);
	    printf ("\r\n                  err=%ld, poserr=%ld",
		rot_fiducial[i].err,rot_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("ROT FIDUCIAL %d:  pos=%ld",i,
		rot_fiducial_position[i]);
	  }     
	}
	break;
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: axis_DID48_shutdown
**
** DESCRIPTION:
**	Shutdown the DID48 hardware which is used to field the 1 Hz interrupt.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DID48
**
**=========================================================================
*/
void axis_DID48_shutdown(int type)
{
    printf("AXIS DID48 shutdown: TOD interrupt %d\r\n",tm_DID48);
    if (tm_DID48!=-1)
      DID48_Interrupt_Enable_Control (tm_DID48,5,DID48_INT_DIS);
    taskDelay(30);
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
void ip_shutdown(int type)
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
** ROUTINE: DID48_interrupt
**
** DESCRIPTION:
**	Interrupt handler for the 1 Hz tick interrupt.  Monitors timeliness
**	of interrupt and restarts 1us timer for interval between interrupts.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	NIST_sec
**	NIST_cnt
**	SDSS_cnt
**	SDSStime
**	axis_stat
**	persistent_axis_stat
**
**=========================================================================
*/
unsigned long NIST_sec;
unsigned char did48int_bit;
unsigned long NIST_cnt=0;
unsigned long SDSS_cnt=0;
#define DAYINSECS	86400
void DID48_interrupt(int type)
{
  extern timer_read(int timer);
  extern timer_start(int timer);
  extern struct AXIS_STAT axis_stat[];
  extern struct AXIS_STAT persistent_axis_stat[];

  DID48_Read_Port (tm_DID48,5,&did48int_bit);
  NIST_cnt++;
  if (did48int_bit&NIST_INT)
  {
    SDSS_cnt++;
    if (SDSStime>=0)
      SDSStime=(SDSStime+1)%DAYINSECS;
    NIST_sec=timer_read(1);
    if (NIST_sec>1000100) 
    {
      axis_stat[0].clock_loss_signal=1;
      persistent_axis_stat[0].clock_loss_signal=1;
    }
    else axis_stat[0].clock_loss_signal=0;
    axis_stat[2].clock_loss_signal=axis_stat[1].clock_loss_signal=
      axis_stat[0].clock_loss_signal;
    timer_start (1);
  }
  DID48_Write_Reg (tm_DID48,4,0x20);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: DIO316_initialize
**
** DESCRIPTION:
**	Setup the DIO316 for axis motion.
**
** RETURN VALUES:
**	return 0 or ERROR
**
** CALLS TO:
**	Industry_Pack
**
** GLOBALS REFERENCED:
**	tm_DIO316
**
**=========================================================================
*/
int DIO316_initialize(unsigned char *addr, unsigned short vecnum)
{
  STATUS stat;
  int i;
  struct IPACK ip;

  Industry_Pack (addr,SYSTRAN_DIO316,&ip);
  for (i=0;i<MAX_SLOTS;i++) 
    if (ip.adr[i]!=NULL)
    {
      printf ("\r\nFound at %d, %p",i,ip.adr[i]);
      tm_DIO316=DIO316Init((struct DIO316 *)ip.adr[i], vecnum);
      break;
    }
  if (i>=MAX_SLOTS) 
  {
    printf ("\r\n****Missing DIO316 at %p****\r\n",addr);
    return ERROR;
  }
  DIO316_Init=TRUE;
  DIO316_Read_Reg(tm_DIO316,0xA,&vecnum);
  DIO316_Interrupt_Enable_Control (tm_DIO316,0,DIO316_INT_DIS);
  DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_DIS);
  DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_DIS);
  DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_DIS);
/*
  if (vecnum==0) vecnum = DIO316_VECTOR;
*/
  stat = intConnect (INUM_TO_IVEC(vecnum),
                                (VOIDFUNCPTR)DIO316_interrupt,
                                DIO316_TYPE);
  printf ("DIO316 vector = %d, interrupt address = %p, result = %8x\r\n",
              vecnum,DIO316_interrupt,stat);
  rebootHookAdd((FUNCPTR)axis_DIO316_shutdown);

/*
  DIO316_Interrupt_Configuration (tm_DIO316,0,DIO316_INT_HIGH_LVL);           
  DIO316_Interrupt_Enable_Control (tm_DIO316,0,DIO316_INT_ENA);
*/
  DIO316_Interrupt_Configuration (tm_DIO316,1,DIO316_INT_FALL_EDGE/*ON_CHANGE*/);
  DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_ENA);
  DIO316_Interrupt_Configuration (tm_DIO316,2,DIO316_INT_FALL_EDGE);
  DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_ENA);
  DIO316_Interrupt_Configuration (tm_DIO316,3,DIO316_INT_FALL_EDGE);
  DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_ENA);
  DIO316_OE_Control (tm_DIO316,3,DIO316_OE_ENA);
  DIO316_OE_Control (tm_DIO316,2,DIO316_OE_ENA);

  IP_Interrupt_Enable(&ip,DIO316_IRQ);
  sysIntEnable(DIO316_IRQ);                                
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: DID48_initialize
**
** DESCRIPTION:
**	Setup the DID48 for 1 Hz diffential interrupt (approximately +-8 volts)
**
** RETURN VALUES:
**	return 0 or ERROR
**
** CALLS TO:
**	Industry_Pack
**
** GLOBALS REFERENCED:
**	tm_DID48
**
**=========================================================================
*/
int DID48_initialize(unsigned char *addr, unsigned short vecnum)
{
  STATUS stat;
  int i;
  struct IPACK ip;

  Industry_Pack (addr,SYSTRAN_DID48,&ip);
  for (i=0;i<MAX_SLOTS;i++) 
    if (ip.adr[i]!=NULL)
    {
      printf ("\r\nFound at %d, %p",i,ip.adr[i]);
      tm_DID48=DID48Init((struct DID48 *)ip.adr[i], vecnum);
      break;
    }
  if (i>=MAX_SLOTS) 
  {
    printf ("\r\n****Missing DID48 at %p****\r\n",addr);
    return ERROR;
  }
  DID48_Init=TRUE;
/*    DID48_Read_Reg(tm_DID48,0x6,&vecnum);
    if (vecnum==0) vecnum = DID48_VECTOR;*/
  stat = intConnect (INUM_TO_IVEC(vecnum),
                                (VOIDFUNCPTR)DID48_interrupt,
                                DID48_TYPE);
  printf ("DID48 vector = %d, interrupt address = %p, result = %8x\r\n",
                vecnum,DID48_interrupt,stat);
  rebootHookAdd((FUNCPTR)axis_DID48_shutdown);

  IP_Interrupt_Enable(&ip,DID48_IRQ);
  sysIntEnable(DID48_IRQ);                                
  DID48_Write_Reg (tm_DID48,3,0x3); /* disable debounce for all byte lanes */
  DID48_Interrupt_Enable_Control (tm_DID48,5,DID48_INT_ENA);
  return 0;
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
void amp_reset(int axis)
{
  extern int cw_DIO316;

	DIO316_Write_Port (cw_DIO316,AMP_RESET,1<<axis);
        taskDelay (2);
	DIO316_Write_Port (cw_DIO316,AMP_RESET,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_init
**
** DESCRIPTION:
**	General initialization of MEI board...called from startup script.
**	Initializes queue data structures.  Setups semaphores and gets
**	tm_latch spawned.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	axis_queue
**	sdss_was_init
**	semMEI
**	semSLC
**
**=========================================================================
*/
int sdss_init()
{
  int i;
  int err;
  int axis;
  double rate;
  char buffer[MAX_ERROR_LEN] ;
  double limit;
  short action;

  for (i=0;i<3;i++)
  {
    axis_queue[i].top=(struct FRAME *)malloc (sizeof(struct FRAME));
    if (axis_queue[i].top==NULL)
    {
      printf("\r\nSDSS_INIT: no memory for queue");
      return ERROR;
    }	
    axis_queue[i].end=axis_queue[i].top;
    axis_queue[i].active=NULL;
    axis_queue[i].cnt=1;
    axis_queue[i].top->nxt=NULL;
    axis_queue[i].top->position=0;
    axis_queue[i].top->velocity=0;
    axis_queue[i].top->end_time=0;
  }
  sdss_was_init=TRUE;
  err = dsp_init(DSP_IO_BASE);
  if (err)
  {
    error_msg(err, buffer) ;	/* convert an error code to a human message */
    printf("dsp_init failed--%s (%d)\n", buffer, err);
/*    return (-1);*/
  }
  else 
    printf("dsp_init Passed!\n");
  err=dsp_reset();
  if (err)
  {
    error_msg(err, buffer) ;	/* convert an error code to a human message */
    printf("dsp_reset failed--%s (%d)\n", buffer, err);
/*    return (-1);*/
  }
  else
    printf("dsp_reset Passed!\n");
  set_sample_rate(160);
  printf("\r\n Sample Rate=%d",dsp_sample_rate());
  for (axis=0;axis<dsp_axes();axis++)
  {
    get_stop_rate(axis,&rate);
    printf ("AXIS %d:  set stop rate=%lf\r\n",axis,rate);
    set_stop_rate(axis,(double)SDSS_STOP_RATE);
    get_stop_rate(axis,&rate);
    printf ("AXIS %d:  set stop rate=%lf\r\n",axis,rate);

    get_e_stop_rate(axis,&rate);
    printf ("AXIS %d: old e_stop rate=%lf\r\n",axis,rate);
    set_e_stop_rate(axis,(double)SDSS_E_STOP_RATE);
    get_e_stop_rate(axis,&rate);
    printf ("AXIS %d:  set e_stop rate=%lf\r\n",axis,rate);

    get_error_limit(axis,&limit,&action);
    printf ("AXIS %d: error limit=%ld, action=%d\r\n",axis,(long)limit,action);
    set_error_limit(axis,24000.,ABORT_EVENT);
    get_error_limit(axis,&limit,&action);
    printf ("AXIS %d: SET error limit=%ld, action=%d\r\n",axis,(long)limit,action);
    set_integration (axis,IM_ALWAYS);
  }
  init_io(2,IO_INPUT);
  arm_latch(TRUE);
  semMEI = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
  semSLC = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
  taskSpawn ("tmLatch",49,VX_FP_TASK,10000,(FUNCPTR)tm_latch,
		(long)"latch.log",0,0,0,0,0,0,0,0,0);
  return 0;
}

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
void save_firmware()
{
  upload_firmware_file ("vx_mei_firmware.bin");
}
void restore_firmware()
{
  download_firmware_file ("vx_mei_firmware.bin");
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_get_time
**	    get_time - prints time as well; used as a diagnostic.
**
** DESCRIPTION:
**	Get SDSS time based on seconds-in-a-day plus us since last 1 Hz
**	interrupt.
**	The timer is scaled due to an incorrect setting in the BSP for
**	VME162.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**	timer_read
**
** GLOBALS REFERENCED:
**	SDSStime
**
**=========================================================================
*/
#define CLOCK_INT 1
/* turn on 1 Hz interrupt time returns else use local clock
*/
#ifdef CLOCK_INT
double sdss_get_time()
{
  extern timer_read(int timer);
  	  unsigned long micro_sec;

          micro_sec = (unsigned long)(1.0312733648*timer_read (1));
	  if (micro_sec>1000000) micro_sec=999999;
          return (double)(SDSStime+((micro_sec%1000000)/1000000.));
}
double get_time()
{
  extern timer_read(int timer);
  	  unsigned long micro_sec;

          micro_sec = (unsigned long)(1.0312733648*timer_read (1));
	  if (micro_sec>1000000) micro_sec=999999;
	  printf ("\r\nSDSS time=%lf",
		(double)(SDSStime+((micro_sec%1000000)/1000000.)));
          return (double)(SDSStime+((micro_sec%1000000)/1000000.));
}
#else
double sdss_get_time()
{
  extern timer_read(int timer);
  	  struct timespec tp;
  	  unsigned long micro_sec;

  	  clock_gettime(CLOCK_REALTIME,&tp);
          micro_sec = timer_read (1);
          return ((double)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
double get_time()
{
  extern timer_read(int timer);
  	  struct timespec tp;
  	  unsigned long micro_sec;

  	  clock_gettime(CLOCK_REALTIME,&tp);
          micro_sec = timer_read (1);
	  printf ("\r\nsec=%d, day_sec=%d, micro_sec=%d, time=%lf",
		tp.tv_sec,tp.tv_sec%ONE_DAY,micro_sec,
		(double)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
          return ((double)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_delta_time
**	    test_dt - diagnostic test
**
** DESCRIPTION:
**	Determine the delta time.  This routine takes into consideration
**	the window when the time wraps from a full day to the beginning
**	of the new day.  The logic has a limit of 400 seconds to the end
**	of the day to work across the boundary.
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
double sdss_delta_time(double t2, double t1)
{
  if ((t2>=0.0)&&(t2<400.)&&(t1>86000.)&&(t1<86400.))
    return ((86400.-t1)+t2);
  if ((t1>=0.0)&&(t1<400.)&&(t2>86000.)&&(t2<86400.))
    return ((t2-86400.)-t1);
  return (t2-t1);
}
int test_dt(int t2,int t1)
{
  printf ("\r\ndt=%f",sdss_delta_time((double)t2,(double)t1));
  return (int)sdss_delta_time((double)t2,(double)t1);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: latchstart
**	    latchverbose
**	    latchquiet
**	    latchprint
**	    latchexcel
**
** DESCRIPTION:
**	Diagnostic routines to capture and analyze the position latches.
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
void latchstart ()
{
  latchidx=0;
}
void latchverbose ()
{
  LATCH_verbose=TRUE;
}
void latchquiet ()
{
  LATCH_verbose=FALSE;
}
void latchprint (char *description)
{
  int i;
  float ratio;

  date();
  printf ("\r\n%s",description);
  printf ("\r\n");
  printf ("\r\naxis\tref\tdata\tlatch pos1\tlatch pos2\tratio");
  ratio=0.0;
  for (i=0;i<latchidx;i++)
  {
    if (i>0) ratio=(latchpos[i].pos2-latchpos[i-1].pos2)/
		(latchpos[i].data-latchpos[i-1].data);
    printf ("\r\n%d\t%d\t%d\t%12.0lf\t%12.0lf\t%f",latchpos[i].axis,
			latchpos[i].ref,latchpos[i].data,
    		(float)latchpos[i].pos1,(float)latchpos[i].pos2,ratio);
  }
  printf ("\r\n");
}
void latchexcel (int axis)
{
  int i;

  date();
  printf ("\r\naxis\tlatch pos1\tlatch pos2");
  for (i=0;i<latchidx;i++)
  {
    if (axis==latchpos[i].axis)
      printf ("\r\n%d\t%12.0lf\t%12.0lf",latchpos[i].axis,
                (float)latchpos[i].pos1,(float)latchpos[i].pos2);
  }
  printf ("\r\n");
}

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
void print_max ()
{
  int i;

  for (i=0;i<3;i++)
    printf ("\r\nAXIS %d: MAX ACC limit %lf deg/sec/sec current max %lf",
	  i,max_acceleration[i],max_acceleration[i+3]);
  for (i=0;i<3;i++)
    printf ("\r\nAXIS %d: MAX VEL limit %lf deg/sec current max %lf",
	  i,max_velocity[i],max_velocity[i+3]);
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
int diagq_setup(int ks)
{
  diagq_siz=ks*1024;
  diagq=malloc (diagq_siz*sizeof(struct DIAG_Q));
  diagq_i=0;
  return 0;
}
int print_diagq()
{
  int i;
  printf ("\r\ni=%d",diagq_i);
  for (i=0;i<diagq_siz;i++)
  {
	printf ("\r\n%d: p=%lf tim=%lf",i,(diagq+i)->p,(diagq+i)->tim);
	printf (" v=%lf a=%lf ji=%lf",(diagq+i)->v,(diagq+i)->a,
		(diagq+i)->ji);
  }
  return 0;
}
 
