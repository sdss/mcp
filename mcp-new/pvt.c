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

static int tm_frames_to_execute(int axis);

int CALC_verbose=FALSE;
int CALCOFF_verbose=FALSE;
int CALCADDOFF_verbose=FALSE;
int CALCFINAL_verbose=FALSE;
int FRAME_verbose=FALSE;
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

/*****************************************************************************/
/*
 * Frame diagnostics
 */
struct DIAG_Q {
   double p;
   double v;
   double a;
   double ji;
   double tim;
};

static struct DIAG_Q *diagq = NULL;
static int diagq_siz,diagq_i;
int DIAGQ_verbose=FALSE;


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
**
**	Load the MEI axis frames into the DSP.  Checks for maximum and
**	updates the largest value.  No action is taken at this time
**	if any maximum is exceeded other than a message.
**	The diagnostic queue was used to aid in trapping problems.
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
      drift_break[axis] = FALSE;
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
	       
	       if(drift_break[axis] || frame_break[axis]) {
		  if(frame_break[axis]) {
		     TRACE(8, "%s frame_break", aname, 0);
		  } else {
		     TRACE(8, "%s drift_break", aname, 0);
		  }
		  
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
	 } else if(drift_break[axis]) {
	    drift_frame(axis, drift_velocity[axis],
			(double)ticks_per_degree[axis]);
	 } else {
	    end_frame(axis, idx - 1, (double)ticks_per_degree[axis]);
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

/*****************************************************************************/
/*
 * Set {drift,frame}_break to allow for processing of PVT tripes
 */
void
enable_pvt(int axis)
{
   drift_break[axis] = frame_break[axis] = FALSE;
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
int
mcp_drift(int axis,			/* the axis in question */
	  double *arcdeg,		/* return the position */
	  double *veldeg,		/*            velocity */
	  double *time)			/*        and time */
{
   double position;
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(axis, 0) < 0) {
      TRACE(0, "drift_cmd: failed to set MS.OFF", 0, 0);
      return(-1);
   }
/*
 * Get the MEI semaphore and proceed
 */
   if(semTake(semMEI,60) == ERROR) {
      TRACE(0, "drift_cmd: failed to get semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }
   
   get_velocity(axis<<1,&drift_velocity[axis]);
   semGive (semMEI);
   drift_break[axis]=TRUE;

   taskDelay(3);
   if(semTake(semMEI, 60) == ERROR) {
      TRACE(0, "drift_cmd: failed to retake semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }

   taskLock();				/* enforce coincidental data */
   get_position_corr(2*axis, &position);
   *time = sdss_get_time();
   taskUnlock();
   semGive(semMEI);

   if(*time < 0) {
      TRACE(0, "drift_cmd: bad time %g", *time, 0);
      return(-1);
   }

   *veldeg = sec_per_tick[axis]*drift_velocity[axis]/3600.;
   *arcdeg = sec_per_tick[axis]*position/3600.;

   return(0);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: move_cmd
**
** DESCRIPTION:
**      MOVE -> Move to the current postion or very close to it..implementation
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
int 
mcp_move(int axis,			/* the axis to move */
	 double *params,		/* 0-3 values: p v t */
	 int nparam)			/* number of valid parameters */
{
   double position,velocity,pos;
   struct FRAME *frame,*nxtque;
   struct FRAME_QUEUE *queue;
   int i;
   double dt;
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(axis, 0) < 0) {
      TRACE(0, "move_cmd: failed to set MS.OFF", 0, 0);
      return(-1);
   }
/*
 * Proceed with the MOVE
 */
   if(sdss_get_time() < 0) {
      TRACE(0, "mcp_move(): bad time", 0, 0);
      return(-1);
   }
   
   queue = &axis_queue[axis];
   
   frame = (struct FRAME *)malloc(sizeof(struct FRAME));
   if(frame == NULL) {
      TRACE(0, "Cannot allocate frame: (%d) %s", errno, strerror(errno));
      return(-1);
   }
   
#if 0					/* XXX */
   TRACE(3, "MOVE %s", cmd, 0);
   TRACE(3, "     Axis = %s cnt = %d", axis_name(axis), cnt);
   printf("%s MOVE %s %d\n", axis_name(axis), cmd, cnt);
#endif

   switch (nparam) {
    case -1:
    case 0:
      tm_get_position(2*axis, &stop_position[axis]);
      frame_break[axis] = TRUE;

      sdssdc.tccmove[axis].position = 0;
      sdssdc.tccmove[axis].velocity = 0;
      sdssdc.tccmove[axis].time = 0;

      if(frame != NULL) free(frame);
      
      return(0);
    case 1:
      position = params[0];

      tm_get_position(2*axis,&pos);
      velocity = (double)0.10;
      frame->end_time =
	fmod((double)(sdss_get_time() +
		      abs((pos/ticks_per_degree[axis] -
			   position)/velocity)), (double)86400.0);
      velocity=0.0;
      break;
    case 2:
      position = params[0];
      velocity = params[1];

      tm_get_position(2*axis,&pos);
      frame->end_time =
	fmod((double)(sdss_get_time() +
		      abs((pos/ticks_per_degree[axis] -
			   position)/velocity)), (double)86400.0);
      break;
    case 3:
      position = params[0];
      velocity = params[1];
      frame->end_time = params[2];

      if(sdss_delta_time(frame->end_time, sdss_get_time()) < 0.0) {
	 if(frame != NULL) free(frame);
	 printf("MOVE CMD: bad time=%f real time=%f\n",
		frame->end_time, sdss_get_time());
	 TRACE(0, "MOVE CMD: bad time=%f", frame->end_time, 0);
	 TRACE(0, "          real time=%f", sdss_get_time(), 0);
	 return(-1);
      }
      
      if(drift_break[axis]) {
	 if(DRIFT_verbose) {
            printf("DRIFT pvt %f %f %f\n", position,velocity,frame->end_time);
	 }
	 
	 taskLock();
	 tm_get_position(2*axis, &pos);
	 dt = sdss_delta_time(frame->end_time, sdss_get_time());
	 taskUnlock();
/*
 * in practice, shifted around sample frequency (.05) for better
 * err transition
 */
	 dt -=.043;	/* modify time to reduce error during transition */
	 pos = (pos + drift_velocity[axis]*dt)/ticks_per_degree[axis];
	 if(DRIFT_verbose) {
	    printf("DRIFT modified pvt %f %f %f, difference=%f, dt=%f\n",
		   pos, velocity, frame->end_time, position - pos, dt);
	 }
	 
	 if(drift_modify_enable) {
	    position = pos;
	 }
	 
	 drift_break[axis] = FALSE;
      }
      break;
   }
   
   frame->position = position;

   if(fabs(velocity) > max_velocity[axis]) {
      TRACE(2, "Max vel. for %s exceeded: %ld",
	    axis_name(axis), (long)velocity);
      velocity = (velocity > 0) ? max_velocity[axis] : -max_velocity[axis];
   }
   
   frame->velocity = (double)velocity;
   frame->nxt = NULL;

   sdssdc.tccmove[axis].position=
     (long)(frame->position*ticks_per_degree[axis]);
   sdssdc.tccmove[axis].velocity=
     (long)(frame->velocity*ticks_per_degree[axis]);
   sdssdc.tccmove[axis].time=(long)(frame->end_time*1000);
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
      if(offset_queue_end[axis][i] != NULL) {
	 /* still correcting offset , reduce new specifications */
	 offset_queue_end[axis][i] = frame;
	 frame->position -= offset[axis][i][1].position;
	 frame->velocity -= offset[axis][i][1].velocity;

	 TRACE(3, "Offsetting %s", axis_name(axis), 0);
	 TRACE(3, "     pos = %d, vel = %d",
	       sdssdc.tccmove[axis].position,
	       sdssdc.tccmove[axis].velocity);
	 
#if 0
	 printf("reduce offset end=%p, pos=%f,idx=%d\n",frame,
		 frame->position,offset_idx[axis][i]);
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

  return(0);
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
**	axis_queue
**	offset_axis_queue
**	offset
**	offset_idx
**
**=========================================================================
*/
int 
mcp_plus_move(int axis,			/* the axis to move */
	      double *params,		/* 0-3 values: p v t */
	      int nparam)		/* number of valid parameters */
{
   double position,velocity,frame_time;
   struct FRAME_QUEUE *queue;
   int i;

   for(i = 0; i < OFF_MAX; i++) {
      if(offset_queue_end[axis_select][i] == NULL) break;
   }
   
   if(i >= OFF_MAX) {
      TRACE(0, "mcp_plus_move: offset active", 0, 0);
      return(-1);
   }

   queue = &axis_queue[axis];
   
   switch (nparam) {
    case -1:
    case 0:
      break;		/* NULL offset - does nothing */
    case 1:
      position = params[0];

      if(position == 0.0) break;
      
      offset[axis][i][0].nxt = &offset[axis][i][1];
      offset[axis][i][0].position = 0;
      offset[axis][i][1].position = position;
      offset[axis][i][0].velocity = 0;
      offset[axis][i][1].velocity = 0;
      offset[axis][i][0].end_time = 0;
/*
 * short offsets are give some extra time for smooth ramp.  long offsets
 * are spread over a time period averaging .4 degs per second
 */
      if(position < 0.15) {
	 offset[axis][i][1].end_time = (double)0.75;
      } else {
	 offset[axis][i][1].end_time =
				       (double)((int)((position/0.15)*20))/20.;
      }

      offset_idx[axis][i] = 0;
      offset_queue_end[axis][i] = queue->end;
      
      break;
    case 2:
      position = params[0];
      velocity = params[1];

      if(position == 0.0 && velocity == 0.0) break;
      
      offset[axis][i][0].nxt = &offset[axis][i][1];
      offset[axis][i][0].position = 0;
      offset[axis][i][1].position = position;
      offset[axis][i][0].velocity = 0;
      offset[axis][i][1].velocity = velocity;
      offset[axis][i][0].end_time = 0;
      
      if(position < 0.15) {
	 offset[axis][i][1].end_time=(double).75;
      } else {			/* average .4 degree per second */
	 offset[axis][i][1].end_time =
	   (double)((int)((position/.15)*20))/20.;
      }
      
      offset_idx[axis][i] = 0;
      offset_queue_end[axis][i] = queue->end;
      break;
    case 3:
      position = params[0];
      velocity = params[1];
      frame_time = params[2];

      TRACE(0, "Attempt to specify +MOVE p v t", 0, 0);
      return(-1);
#if 0
      if(position == 0.0 && velocity == 0.0) break;
      
      offset[axis][i][0].nxt = &offset[axis][i][1];
      offset[axis][i][0].position = 0;
      offset[axis][i][1].position = position;
      offset[axis][i][0].velocity = 0;
      offset[axis][i][1].velocity = velocity;
      offset[axis][i][0].end_time = 0;
      if(position < 0.2) {
	 offset[axis][i][1].end_time = (double).4;
      } else {
	 offset[axis][i][1].end_time =
	   (double)((int)((position/1.0)*20))/20.;
      }
      offset_idx[axis][i]=0;
      offset_queue_end[axis][i]=queue->end;
#endif
      break;
   }

   printf("%p: queue_end=%p, position=%f, velocity=%f, end_time=%f\n",
	  &offset[axis][i],offset_queue_end[axis][i],
	  offset[axis][i][1].position,
	  offset[axis][i][1].velocity,
	  offset[axis][i][1].end_time);
   
   sdssdc.tccpmove[axis].position =
     (long)(offset[axis][i][1].position*ticks_per_degree[axis]);
   sdssdc.tccpmove[axis].velocity =
     (long)(offset[axis][i][1].velocity*ticks_per_degree[axis]);
   sdssdc.tccpmove[axis].time =
     (long)(offset[axis][i][1].end_time*1000);
   
   return(0);
}