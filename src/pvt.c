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
#include "as2.h"

static int tm_frames_to_execute(int axis);

int DRIFT_verbose = FALSE;
int FRAME_verbose = FALSE;

#define FRMHZ		20	/* 20 Hz Frames to MEI */
#define FLTFRMHZ	20.
#define MAX_CALC 20

static double tim[NAXIS][MAX_CALC];
static double p[NAXIS][MAX_CALC];
static double v[NAXIS][MAX_CALC];
static double a[NAXIS][MAX_CALC];
static double ji[NAXIS][MAX_CALC];

static double timoff[NAXIS][MAX_CALC];
static double poff[NAXIS][MAX_CALC];
static double voff[NAXIS][MAX_CALC];
static double aoff[NAXIS][MAX_CALC];
static double jioff[NAXIS][MAX_CALC];

double time_off[NAXIS] = {0.0, 0.0, 0.0};
double stop_position[NAXIS] = {0.0, 0.0, 0.0};
double drift_velocity[NAXIS] = {0.0, 0.0, 0.0};

int frame_break[NAXIS] = {FALSE, FALSE, FALSE};
int drift_break[NAXIS] = {FALSE, FALSE, FALSE};
int drift_modify_enable = FALSE;

#define OFF_MAX	6		/* upto OFF_MAX + 2 offsets at one time */
struct FRAME offset[NAXIS][OFF_MAX+2][2];
struct FRAME *offset_queue_end[NAXIS][OFF_MAX+2]={
   {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}, 
   {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}, 
   {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
};
int offset_idx[NAXIS][OFF_MAX+2]={
			{0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0}
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
   int uid = 0, cid = 0;
   int bad_pvt = 0;			/* was a bad PVT detected? */
   double x0, x1, v0, v1;		/* initial/final values of {p,v} */
   double dx,dv,dt,xdot;
   double ai,j,t,lai,lj,lt,ldt;
   struct FRAME *fframe;
   struct FRAME *lframe;
   int i;
   int nframe;				/* number of frames to calculate */
   
   /* problem............................*/
   if(iframe->nxt == NULL) {
      NTRACE(0, uid, cid, "Initial frame has NULL ->nxt pointer");
      return ERROR;
   }
   
   fframe = iframe->nxt;

   x0 = iframe->position; x1 = fframe->position;
   v0 = iframe->velocity; v1 = fframe->velocity;

   dx = x1 - x0;
   dv = v1 - v0;
   dt = sdss_delta_time(fframe->end_time, iframe->end_time);
   xdot = dx/dt;
   ai = (2/dt)*(3*xdot - 2*v0 - v1);
   j = (6/(dt*dt))*(v0 + v1 - 2*xdot);
/*
 * necessary if for loop not executed; calc of t
 */
   t = (start + 1)/FLTFRMHZ + time_off[axis]; /* for end condition */

   nframe = min(MAX_CALC-1, (int)((dt - time_off[axis])*FRMHZ) - start);
#if 1
   for(i = 0; i < nframe; i++) {
      t = (i + start + 1)/FLTFRMHZ + time_off[axis];
      tim[axis][i] = 1/FLTFRMHZ;
      p[axis][i] = x0 + v0*t + (1/2.)*ai*(t*t) + (1/6.)*j*(t*t*t);
      v[axis][i] =      v0   +        ai*t     + (1/2.)*j*(t*t);
      a[axis][i] =                    ai              + j*t;
      ji[axis][i] =                                     j;
/*
 * Check if velocity/acceleration is out of bounds
 */
      if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = a[axis][i];
      }
      
      if(fabs(a[axis][i]) > max_acceleration[axis]) {
	 long acc = 1e3*a[axis][i];	/* OTRACE macro has a variable "a" */
	 OTRACE(2, "calc_frames: Max accl. for %s exceeded: %ld/1000",
	       axis_name(axis), acc);
	 {
	    char key[20];
	    sprintf(key, "%sMaxAccRequested", axis_abbrev(axis));

	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, a[axis][i]);
	 }
	 
	 bad_pvt++;

	 a[axis][i] = (a[axis][i] < 0) ? max_acceleration[axis] :
						       -max_acceleration[axis];
/*
 * Recalculate the PVTs now that we've had to deviate from the initial path
 */
	 x0 = p[axis][i];
	 v0 = v[axis][i];
	 
	 dx = x1 - x0;
	 dv = v1 - v0;
	 dt = sdss_delta_time(fframe->end_time, t);
	 xdot = dx/dt;
	 ai = (2/dt)*(3*xdot - 2*v0 - v1);
	 j = (6/(dt*dt))*(v0 + v1 - 2*xdot);
      }

      if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = v[axis][i];
      }
      
      if(fabs(v[axis][i]) > max_velocity[axis]) {
	 OTRACE(2, "calc_frames: Max vel. for %s exceeded: %ld/1000",
	       axis_name(axis), (long)(1e3*v[axis][i]));

	 {
	    char key[20];
	    sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, v[axis][i]);
	 }
	 
	 bad_pvt++;
      }
   }
#else
   for(i = 0; i < nframe; i++) {
      t = (i + start + 1)/FLTFRMHZ + time_off[axis];
      tim[axis][i] = 1/FLTFRMHZ;
      p[axis][i] = x0 + v0*t + (1/2.)*ai*(t*t) + (1/6.)*j*(t*t*t);
      v[axis][i] =      v0   +        ai*t     + (1/2.)*j*(t*t);
      a[axis][i] =                    ai              + j*t;
      ji[axis][i] =                                     j;
/*
 * Check if velocity/acceleration is out of bounds
 */
      if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = a[axis][i];
      }
      
      if(fabs(a[axis][i]) > max_acceleration[axis]) {
	 long acc = 1e3*a[axis][i];	/* OTRACE macro has a variable "a" */
	 OTRACE(2, "calc_frames: Max accl. for %s exceeded: %ld/1000",
		axis_name(axis), acc);
	 
	 {
	    char key[20];
	    sprintf(key, "%sMaxAccRequested", axis_abbrev(axis));
	    
	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, a[axis][i]);
	 }
	 bad_pvt++;
      }

      if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = v[axis][i];
      }
      
      if(fabs(v[axis][i]) > max_velocity[axis]) {
	 OTRACE(2, "calc_frames: Max vel. for %s exceeded: %ld/1000",
	       axis_name(axis), (long)(1e3*v[axis][i, 0, 0]));

	 {
	    char key[20];
	    sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, v[axis][i]);
	 }
	 
	 bad_pvt++;
      }
   }
/*
 * Do we need to fixup those velocities/accelerations and write debugging info?
 */
   if(bad_pvt) {
      int ii;
      
      OTRACE(3, "Bad PVT: time_off = %g", time_off[axis], 0);
      OTRACE(3, "Bad PVT: dt = %g", dt, 0);
      OTRACE(3, "Bad PVT: dx = %g", dx, 0);
      OTRACE(3, "Bad PVT: dv = %g", dv, 0);
      OTRACE(3, "Bad PVT: ai = %g", ai, 0);
      OTRACE(3, "Bad PVT: j = %g", j, 0);
      
      for(ii = 0; ii < nframe; ii++) {
	 OTRACE(3, "Bad PVT: p = %g", p[axis][ii], 0);
	 OTRACE(3, "Bad PVT: v = %g", v[axis][ii], 0);
	 {
	    double acc = a[axis][ii];	/* OTRACE has a variable `a' */
	    OTRACE(3, "Bad PVT: a = %g", acc, 0);
	 }
	 
	 if(fabs(v[axis][ii]) > max_velocity[axis]) {
	    v[axis][ii] = (v[axis][ii] > 0) ?
	      max_velocity[axis] : -max_velocity[axis];
	 }
	 
	 if(fabs(a[axis][ii]) > max_acceleration[axis]) {
	    a[axis][ii] = (a[axis][ii] > 0) ?
	      max_acceleration[axis] : -max_acceleration[axis];
	 }
      }

      bad_pvt = 0;
   }
#endif
/*
 * last one (frame?) with a portion remaining; needs portion of next one
 */
#if 0
   printf("Check for FINAL: time_off=%f, i=%d, start=%d, t=%f, dt=%f\n",
	  time_off[axis], i, start, t, dt);
#endif

   if((int)(i + start) != (int)((dt - time_off[axis])*FLTFRMHZ) ||
						    t == dt - time_off[axis]) {
      if(i > MAX_CALC - 1) {
	 OTRACE(0, "calc_frames has problems (A) %d\n",i, 0);
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
      OTRACE(3, "CALC FRAME: next frame required to finish", 0, 0);
      return ERROR;
   }
    
   fframe = lframe->nxt;
   time_off[axis] = t;
   lai = ai; 
   lj = j;
   dx = fframe->position - lframe->position;
   dv = fframe->velocity - lframe->velocity;
   dt = fframe->end_time - lframe->end_time;
   xdot = dx/dt;

   ai = (2/dt)*(3*xdot - 2*lframe->velocity - fframe->velocity);
   j = (6/(dt*dt))*(lframe->velocity + fframe->velocity - 2*xdot);
   
   p[axis][i] = lframe->position + lframe->velocity*t + (1/2.)*ai*(t*t) +
							      (1/6.)*j*(t*t*t);
   v[axis][i] = lframe->velocity + ai*t + (1/2.)*j*(t*t);
   a[axis][i] = FLTFRMHZ*t*(ai + j*t) + FLTFRMHZ*ldt*(lai + (lj*lt));
   ji[axis][i] = FLTFRMHZ*t*j + FLTFRMHZ*ldt*lj;
/*
 * Check if velocity/acceleration is out of bounds
 */
   if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
      max_acceleration_requested[axis] = a[axis][i];
   }
   
   if(fabs(a[axis][i]) > max_acceleration[axis]) {
      long acc = 1e3*a[axis][i];	/* OTRACE macro has a variable "a" */
      OTRACE(2, "calc_frames (end): Max accl. for %s exceeded: %ld/1000",
	    axis_name(axis), acc);
   
      {
	 char key[20];
	 sprintf(key, "%sMaxAccRequested", axis_abbrev(axis));
	 
	 sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, a[axis][i]);
      }
      
      bad_pvt++;
   }

   if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
      max_velocity_requested[axis] = v[axis][i];
   }
   
   if(fabs(v[axis][i]) > max_velocity[axis]) {
      OTRACE(2, "calc_frames (end): Max vel. for %s exceeded: %ld/1000",
	    axis_name(axis), (long)(1e3*v[axis][i]));
      
      {
	 char key[20];
	 sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	 sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, v[axis][i]);
      }
	 
      bad_pvt++;
   }

   if(i + 1 > MAX_CALC) {
      OTRACE(0, "calc_frames has problems (B) %d\n",i + 1, 0);
   }
/*
 * Do we need to fixup those velocities/accelerations and write debugging info?
 */
   if(bad_pvt) {
      int ii;
      
      OTRACE(3, "Bad PVT: dt = %g", dt, 0);
      OTRACE(3, "Bad PVT: dx = %g", dx, 0);
      OTRACE(3, "Bad PVT: dv = %g", dv, 0);
      OTRACE(3, "Bad PVT: ai = %g", ai, 0);
      OTRACE(3, "Bad PVT: j = %g", j, 0);

      for(ii = 0; ii <= i; ii++) {
	 OTRACE(3, "Bad PVT: p = %g", p[axis][ii], 0);
	 OTRACE(3, "Bad PVT: v = %g", v[axis][ii], 0);
	 {
	    double acc = a[axis][ii];	/* OTRACE has a variable `a' */
	    OTRACE(3, "Bad PVT: a = %g", acc, 0);
	 }

	 if(fabs(v[axis][ii]) > max_velocity[axis]) {
	    v[axis][ii] = (v[axis][ii] > 0) ?
	      max_velocity[axis] : -max_velocity[axis];
	 }
	 
	 if(fabs(a[axis][ii]) > max_acceleration[axis]) {
	    a[axis][ii] = (a[axis][ii] > 0) ?
	      max_acceleration[axis] : -max_acceleration[axis];
	 }
      }
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
   double dx,dv,dt,xdot;
   double ai,j,t;
   struct FRAME *fframe;
   int i,ii;
  
   fframe = iframe->nxt;
   dx = fframe->position - iframe->position;
   dv = fframe->velocity - iframe->velocity;
   dt = sdss_delta_time(fframe->end_time, iframe->end_time);
   xdot = dx/dt;
   ai = (2/dt)*(3*xdot - 2*iframe->velocity - fframe->velocity);
   j = (6/(dt*dt))*(iframe->velocity + fframe->velocity - 2*xdot);
/*
 * neccessary if for loop not executed; calc of t
 */
   t = (start + 1)/FLTFRMHZ;		/* for end condition */
   
   for(i = 0; i < (int)min(cnt, (int)(dt*FRMHZ)-start); i++) {
      t = (i + start + 1)/FLTFRMHZ;
      timoff[axis][i] = 1/FLTFRMHZ;
      poff[axis][i] += iframe->position + iframe->velocity*t + (1/2.)*ai*(t*t)+
							      (1/6.)*j*(t*t*t);
      voff[axis][i] += iframe->velocity + ai*t + (1/2.)*j*(t*t);
      aoff[axis][i] += ai + j*t;
      jioff[axis][i] += j;
   }

   for(ii = i; ii < cnt; ii++) {         
      t = (ii+start+1)/FLTFRMHZ;
      timoff[axis][ii] = 1/FLTFRMHZ;
      poff[axis][ii] += fframe->position + fframe->velocity*(t - dt);
      voff[axis][ii] += fframe->velocity;
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
   int uid = 0, cid = 0;
   int bad_pvt = 0;			/* was a bad PVT detected? */
   int i;

   for(i=0;i<cnt;i++) {
      p[axis][i] += poff[axis][i];
      v[axis][i] += voff[axis][i];
      a[axis][i] += aoff[axis][i];
      ji[axis][i] += jioff[axis][i];
/*
 * Check if velocity/acceleration is out of bounds
 */
      if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = a[axis][i];
      }
      
      if(fabs(a[axis][i]) > max_acceleration[axis]) {
	 long acc = 1e3*a[axis][i];	/* OTRACE macro has a variable "a" */
	 OTRACE(2, "addoffset: Max accl. for %s exceeded: %ld/1000",
	       axis_name(axis), acc);
	 {
	    char key[20];
	    sprintf(key, "%sMaxAccRequested", axis_abbrev(axis));
	    
	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, a[axis][i]);
	 }
	 
	 bad_pvt++;
      }

      if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = v[axis][i];
      }
      
      if(fabs(v[axis][i]) > max_velocity[axis]) {
	 OTRACE(2, "addoffset: Max vel. for %s exceeded: %ld/1000",
	       axis_name(axis), (long)(1e3*v[axis][i]));
	 
	 {
	    char key[20];
	    sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, v[axis][i]);
	 }
	 
	 bad_pvt++;
      }
   }
/*
 * Do we need to fixup those velocities/accelerations and write debugging info?
 */
   if(bad_pvt) {
      int ii;
      
      OTRACE(3, "Bad PVT: jioff = %g", jioff[axis][0], 0);
      for(ii = 0; ii < cnt; ii++) {
	 OTRACE(3, "Bad PVT: p = %g", p[axis][ii] - poff[axis][ii], 0);
	 OTRACE(3, "Bad PVT:         poff = %g", poff[axis][ii], 0);
	 OTRACE(3, "Bad PVT: v = %g", v[axis][ii] - voff[axis][ii], 0);
	 OTRACE(3, "Bad PVT:         voff = %g", voff[axis][ii], 0);
	 {
	    double acc = a[axis][ii];	/* OTRACE has a variable `a' */
	    OTRACE(3, "Bad PVT: a = %g", acc - aoff[axis][ii], 0);
	 }
	 OTRACE(3, "Bad PVT:         aoff = %g", aoff[axis][ii], 0);

	 if(fabs(v[axis][ii]) > max_velocity[axis]) {
	    v[axis][ii] = (v[axis][ii] > 0) ?
	      max_velocity[axis] : -max_velocity[axis];
	 }
	 
	 if(fabs(a[axis][ii]) > max_acceleration[axis]) {
	    a[axis][ii] = (a[axis][ii] > 0) ?
	      max_acceleration[axis] : -max_acceleration[axis];
	 }
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
start_frame(int axis, double t)
{
   int uid = 0, cid = 0;
   int lcnt;
  
   time_off[axis] = 0.0;
   while ((lcnt = tm_frames_to_execute(axis)) > 1) {
#if 0
      printf("Dwell frames left=%d\n",lcnt);
#endif
      taskDelay(3);
   }
   
   taskDelay(5);
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "start_frame: Failed to get semMEI: %s (%d)",
	       strerror(errno), errno);
      taskSuspend(0);
   }
   
   t = sdss_delta_time(t, sdss_get_time());
#if 0
   printf("time to dwell=%f\n",t);
#endif
   dsp_dwell(2*axis, t);
   semGive(semMEI);
   
   if (FRAME_verbose) {
      printf("START axis=%d: time=%f\n", 2*axis, t);
   }
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
   dt = fframe->end_time - iframe->end_time;
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
static void
load_frames(int axis,			/* which axis */
	    int idx,			/* starting index in p[], a[] etc. */
	    int cnt)			/* number of frames */
{
   int uid = 0, cid = 0;
   int e;
   int i;
   FRAME frame;
   const double sf = ticks_per_degree[axis];
   
   if(FRAME_verbose) {
      printf("\nLoad %d Frames, sf=%f\n",cnt,sf);
   }

   if(idx + cnt > MAX_CALC) {
      NTRACE_1(0, uid, cid, "Buffer overrun in load_frames: %d", idx + cnt);
      cnt = MAX_CALC - idx;
   }

   for(i = idx; i < idx + cnt; i++) {
#if 1					/* we check when we _generate_ frames*/
      if(fabs(a[axis][i]) > fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = a[axis][i];
      }
      
      if(fabs(a[axis][i]) > max_acceleration[axis]) {
	 printf("AXIS %d: MAX ACC %f exceeded; limit %f\n",
		axis, a[axis][i], max_acceleration[axis]);
	 {
	    long acc = 1e3*a[axis][i];	/* OTRACE macro has a variable "a" */
	    OTRACE(2, "Max accl. for %s exceeded: %ld/1000",
		  axis_name(axis), acc);
	 }
	 
	 {
	    char key[20];
	    sprintf(key, "%sMaxAccRequested", axis_abbrev(axis));
	    
	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, a[axis][i]);
	 }
	 a[axis][i] = (a[axis][i] > 0) ?
	    max_acceleration[axis] : -max_acceleration[axis];
      }
      
      if(fabs(v[axis][i]) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = v[axis][i];
      }
      
      if(fabs(v[axis][i]) > max_velocity[axis]) {
	 OTRACE(2, "load_frames: Max vel. for %s exceeded: %ld/1000",
	       axis_name(axis), (long)(1e3*v[axis][i]));

	 {
	    char key[20];
	    sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	    sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, v[axis][i]);
	 }

	 v[axis][i] = (v[axis][i] > 0) ?
				       max_velocity[axis] : -max_velocity[axis];
      }
#endif
      
      while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	 NTRACE_2(0, uid, cid, "load_frames: failed to get semMEI: %s (%d)",
		  strerror(errno), errno);
	 taskSuspend(0);
      }
      
      taskLock();
      e = frame_m_xvajt_corr(&frame,"0l xvajt un d", 2*axis,
			     p[axis][i]*sf,
			     v[axis][i]*sf,
			     a[axis][i]*sf,
			     ji[axis][i]*sf,
			     tim[axis][i],
			     NEW_FRAME);
      taskUnlock();
      semGive (semMEI);
      
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
void
stop_frame(int axis,
	   double pos,
	   double sf)
{
   int uid = 0, cid = 0;
   
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "stop_frame failed to take semMEI for %s: %s",
	       axis_name(axis), strerror(errno));
      taskSuspend(0);
   }
   set_stop(2*axis);
   semGive(semMEI);

   for(;;) {
      while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	 NTRACE_2(0, uid, cid, "stop_frame failed to take semMEI for %s: %s",
		  axis_name(axis), strerror(errno));
	 taskSuspend(0);
      }

      if(motion_done(2*axis)) {
	 break;
      }
      
      semGive(semMEI);

      taskDelay(3);
   }

   clear_status(2*axis);
   
   semGive(semMEI);
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
static void
stp_frame(int axis,
	  double pos)
{
   int uid = 0, cid = 0;
   double a;				/* desired acceleration */
   int e;
   FRAME frame;
   int frame_type;			/* what sort of event is desired? */
   double position;
   const double sf = ticks_per_degree[axis];
   double velocity;

   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "stp_frame failed to take semMEI for %s: %s",
	       axis_name(axis), strerror(errno));
      taskSuspend(0);
   }
   
   get_position_corr(2*axis, &position);
   get_velocity(2*axis, &velocity);

   a = sf/2;
   frame_type = NEW_FRAME;
   if(velocity > 0.0) {
      a = -a;
      frame_type |= TRIGGER_NEGATIVE;
   }

   e = frame_m(&frame,"0l avj un d", 2*axis,
	       a, (double)0.0, (double)0.0,
	       FUPD_ACCEL | FUPD_JERK | FTRG_VELOCITY, frame_type);
   e = frame_m(&frame,"0l va u d", 2*axis,
	       (double)0.0, (double)0.0,
	       FUPD_ACCEL | FUPD_VELOCITY, 0);

   semGive(semMEI);
   
   for(;;) {
      while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	 NTRACE_2(0, uid, cid, "stp_frame failed to take semMEI for %s: %s",
		  axis_name(axis), strerror(errno));
	 taskSuspend(0);
      }
   
      if(!in_motion(2*axis)) {
	 semGive (semMEI);
	 break;
      }
      semGive(semMEI);
      
      taskDelay(3);
   }
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
static void
drift_frame(int axis,
	    double vel)
{
   int uid = 0, cid = 0;
   int e;
   FRAME frame;
   const double sf = ticks_per_degree[axis];
   
   NTRACE_2(3, uid, cid, "drifting %s: v = %ld cts/sec", axis_name(axis), (long)vel);
   if (FRAME_verbose) {
      printf ("DRIFT %s: v=%12.8f\n", axis_name(axis), (double)vel);
      printf("Drift frames left=%d\n", tm_frames_to_execute(axis));
   }
   
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "drift_frame failed to take semMEI for %s: %s",
	       axis_name(axis), strerror(errno));
      taskSuspend(0);
   }
   
   e = frame_m(&frame, "0l vaj un d", 2*axis,
	       vel, 0.2*sf, (double)0.0,
	       FUPD_ACCEL|FUPD_VELOCITY|FUPD_JERK|FTRG_VELOCITY, NEW_FRAME);
   e = frame_m(&frame, "0l va u d", 2*axis,
	       vel, (double)0.0,
	       FUPD_ACCEL|FUPD_VELOCITY, 0);
   
   semGive (semMEI);
}

/*=========================================================================
**
**	End frames sent to MEI since there are no new pvts.  This should
**	bring the motion to a position with no velocity or acceleration.
**	The controller will remain in closed-loop holding the position.
**
**=========================================================================
*/
void
end_frame(int axis, int ind)
{
   int uid = 0, cid = 0;
   int e;
   FRAME frame;
   const double sf = ticks_per_degree[axis];
   
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "end_frame failed to take semMEI for %s: %s",
	       axis_name(axis), strerror(errno));
      taskSuspend(0);
   }

   e = frame_m_xvajt_corr(&frame, "0l xvajt un d", 2*axis,
			  p[axis][ind]*sf,
			  0.0,
			  0.0,
			  0.0,
			  (1./FLTFRMHZ),
			  0);

   dsp_set_last_command_corr(dspPtr, 2*axis, (double)p[axis][ind]*sf);
   semGive(semMEI);
   
   if (FRAME_verbose) {
      printf("END axis=%d (%d): "
	     "p=%12.8f, v=%12.8f, a=%12.8f, j=%12.8f, t=%12.8f\n",
	     2*axis, ind,
	     (double)p[axis][ind]*sf,(double)v[axis][ind]*sf,
	     (double)a[axis][ind]*sf, (double)ji[axis][ind]*sf,
	     tim[axis][ind]);
   }
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
   int uid = 0, cid = 0;
   const char *const aname = axis_name(axis); /* name of our axis */
   int cnt, lcnt;
   struct FRAME *frame;
   struct FRAME *frmoff;
   int i;
   int frame_cnt, frame_idx;
   int moving;				/* the axis is still moving */
   double position;
   double velocity;
   int idx;
   
   tm_sem_controller_run(2*axis);
   printf("Axis=%s;  Ticks per degree=%f\n", aname, ticks_per_degree[axis]);
   
   idx = 0;
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
	    stp_frame(axis, stop_position[axis]);
	    frame_break[axis] = FALSE;
	 }
	 taskDelay(3);
      }

      frame=axis_queue[axis].active;
      drift_break[axis] = FALSE;
/*
 * reposition if necessary, and if we aren't already moving
 */
      while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	 NTRACE_2(0, uid, cid, "tm_TCC: failed to take semMEI: %s (%d)",
		  strerror(errno), errno);
	 taskSuspend(0);
      }

      get_position_corr(2*axis, &position);
      get_velocity(2*axis, &velocity);
      semGive(semMEI);
      
      OTRACE(8, "Check Params for repositioning %s", aname, 0);

      if(fabs(velocity) < 1e-8 &&
	 fabs(frame->position*ticks_per_degree[axis] - position) >
						  0.01*ticks_per_degree[axis]) {
	 while((lcnt = tm_frames_to_execute(axis)) > 1) {
	    OTRACE(8, "Frames left for %s: %d", aname, lcnt);
	    taskDelay(1);
	 }
	 tm_start_move(2*axis, frame->position*ticks_per_degree[axis],
		       max_velocity[axis]/2*ticks_per_degree[axis],
		       max_acceleration[axis]/2*ticks_per_degree[axis]);

	 OTRACE(3, "Repositioning %s by TCC cmd", aname, 0);
	 OTRACE(3, "    from pos=%ld to pos=%ld",
		(long)position, (long)(frame->position*ticks_per_degree[axis]));
	 
	 while((fabs(frame->position*ticks_per_degree[axis] - position) >
						 0.01*ticks_per_degree[axis])) {
	    taskDelay(10);

	    while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
	       NTRACE_2(0, uid, cid, "tm_TCC: failed to retake semMEI: %s (%d)",
			strerror(errno), errno);
	       taskSuspend(0);
	    }
	    
	    moving = in_motion(2*axis);
	    get_position_corr(2*axis, &position);
	    
	    semGive(semMEI);

	    if(!moving) {	/* not in motion */
	       break;
	    }

	    OTRACE(8, "repositioning to %ld", (long)position, 0);
	 }
	 OTRACE(3, "Done repositioning %s to %ld", aname, (long)position);
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
	 OTRACE(0, "%s frame deleted due to expiration time", aname, 0);
      }
      
      if(frame == NULL) {
	 OTRACE(3, "%s restart no frames to process", aname, 0);
	 continue;
      }
      
      start_frame(axis, frame->end_time);
      while(frame->nxt == NULL &&
	    sdss_delta_time(frame->end_time, sdss_get_time()) > 0.02) {
	 OTRACE(8, "%s waiting for second frame", aname, 0);
	 taskDelay (3);
      }

      while(frame->nxt != NULL ||
	    (axis_queue[axis].active != NULL &&
				   !frame_break[axis] && !drift_break[axis])) {

	 if(frame == NULL) {
	    OTRACE(0, "%s frame == NULL", axis_name(axis), 0);
	    traceMode(traceModeGet() & ~0x1);
	    taskSuspend(0);
	 }

	 frame_cnt = get_frame_cnt(axis, frame);
	 
	 OTRACE(8, "%s frame_cnt=%d", aname, frame_cnt);
	 frame_idx = 0;
	 while(frame_cnt > 0) {
	    OTRACE(8, "%s loop: frame_cnt=%d", aname, frame_cnt);

	    while((cnt = calc_frames(axis, frame, frame_idx)) == ERROR &&
		  tm_frames_to_execute(axis) > 4) {
	       taskDelay(1);
	    }
	       
	    if(cnt == ERROR) {
	       OTRACE(5, "No frames; setting frame_break for %s",
		     axis_name(axis), 0);
#if 0
	       printf("frame=%p, nxt=%p, nxt=%p, frame_cnt=%d\n",
		      frame,frame->nxt,(frame->nxt)->nxt,frame_cnt);
#endif
	       frame_break[axis] = TRUE;
	       axis_queue[axis].active = NULL;
	       frame_cnt = 0;
	       break;
	    }
	    
	    for(i = 0; i < OFF_MAX; i++) {
	       OTRACE(8, "%s offset queue i=%d", aname, i);
	       clroffset(axis,cnt);
	       
	       if(offset_queue_end[axis][i] != NULL) {
		  calc_offset(axis,
			      &offset[axis][i][0], offset_idx[axis][i], cnt);
		  offset_idx[axis][i] += cnt;
		  
		  if(offset_idx[axis][i]/20.0 > offset[axis][i][1].end_time) {
		     OTRACE(8, "%s shutdown offset", aname, 0);
		     frmoff = frame;
		     
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
			    frmoff, frmoff->position, offset_idx[axis][i]);
#endif
		     offset_queue_end[axis][i] = NULL;
		     taskUnlock();
		  }
	       }
	       
	       addoffset(axis, cnt);
	    }
	    frame_idx += cnt;
	    frame_cnt -= cnt;
	    
	    if(drift_break[axis] || frame_break[axis]) {
	       OTRACE(8, "%s %s_break", aname, 
		      (frame_break[axis] ? "frame" : "drift"));
	       
	       axis_queue[axis].active = NULL;
	       frame_cnt = 0;
	       break;
	    }
	    
	    idx = 0;
	    while(cnt > 0) {
	       if(drift_break[axis] || frame_break[axis]) {
		  OTRACE(8, "%s %s_break 2", aname, 
			(frame_break[axis] ? "frame" : "drift"));
		  
		  axis_queue[axis].active = NULL;
		  frame_cnt = 0;
		  cnt = 0;
		  break;
	       }

	       if(cnt <= 0) {		/* replaces weird Charlie if */
		  OTRACE(0, "cnt == %d <= 0", cnt, 0); /* XXXX */
		  traceMode(traceModeGet() & ~0x1);
		  taskSuspend(0);
	       }
	       OTRACE(8, "load_frames idx=%d cnt = %d", idx, min(cnt, 5));
	       load_frames(axis, idx, min(cnt, 5));
	       
	       if(idx == MAX_CALC - 5 && cnt == 5) {
		  OTRACE(1, "Last frame in buffer: p=%f",
			 p[axis][MAX_CALC - 1], 0);
	       }
	       
	       while(tm_frames_to_execute(axis) > 10) {
		  taskDelay(3);
	       }
	       
	       idx += 5;
	       cnt -= 5;
	    }
	 }
	 
	 if(axis_queue[axis].active == NULL) {
	    frame = axis_queue[axis].end;
	    break;
	 }
	 
	 frame = frame->nxt;
	 axis_queue[axis].active = frame;
	 while(frame->nxt == NULL &&
	       sdss_delta_time(frame->end_time, sdss_get_time()) > 0.02) {
	    taskDelay (1);
	 }
	 
	 while(frame->nxt == NULL && tm_frames_to_execute(axis) > 1) {
	    taskDelay(1);
	 }
      }
      
      lcnt = tm_frames_to_execute(axis);
      if(frame_break[axis]) {
	 OTRACE(3, "%s frame_break: frames left=%d", aname, lcnt);
      } else if(drift_break[axis]) {	
	 OTRACE(3, "%s drift_break: frames left=%d", aname, lcnt);
      } else if(frame->nxt == NULL) {
	 OTRACE(3, "%s no next frame: frames left=%d", aname, lcnt);
      } else {
	 OTRACE(3, "%s no active frame: frames left=%d", aname, lcnt);
      }
      
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
	 stp_frame(axis, stop_position[axis]);
	 frame_break[axis] = FALSE;
      } else if(drift_break[axis]) {
	 drift_frame(axis, drift_velocity[axis]);
      } else {
	 end_frame(axis, idx - 1);
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
void
load_frames_test(int axis, int cnt)
{
   int i;
   const double sf = ticks_per_degree[axis];
   
   printf("Load %d Frames, sf=%f\n",cnt,sf);
   for(i = 0; i < cnt; i++) {
      printf ("axis=%d (%d): p=%12.8f, v=%12.8f, a=%12.8f, j=%12.8f,t=%12.8f\n",
	      axis<<1,i,
	      (double)p[axis][i]*sf,(double)v[axis][i]*sf,
	      (double)a[axis][i]*sf,ji[axis][i]*sf,
	      tim[axis][i]);
   }
}

void tm_TCC_test(int axis, struct FRAME *iframe, struct FRAME *fframe)
{
  int cnt;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;

  printf ("\r\n Axis=%d;  Ticks per degree=%f",axis,
        ticks_per_degree[axis]);

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
            load_frames_test(axis,cnt);
        }
	frame_break[axis]=FALSE;
        frame = frame->nxt;
      }
      printf ("\r\n Ran out");

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
   
   printf("List Axis Queue=%s: %p\n", axis_name(axis), &axis_queue[axis]);
   queue = &axis_queue[axis];
   for(frame = queue->top; frame != NULL; frame = frame->nxt) {
      if(frame == queue->top) printf("TOP, cnt=%d\n", queue->cnt);
      if(frame == queue->active) printf("ACTIVE\n");
      if(frame == queue->end) printf ("END\n");
      
      printf("%p: position=%12.8f, velocity=%12.8f, end_time=%12.8f\n",
	     frame, frame->position, frame->velocity, frame->end_time);
      
      if(frame->nxt != NULL) {
	 printf ("      "
		 "deltas position=%12.8f, velocity=%12.8f, end_time=%12.8f\n",
		 frame->nxt->position - frame->position,
		 frame->nxt->velocity - frame->velocity,
		 frame->nxt->end_time - frame->end_time);
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
**	drift_velocity
**	drift_break
**
**=========================================================================
*/
int
mcp_drift(int uid, unsigned long cid,
	  int axis,			/* the axis in question */
	  double *arcdeg,		/* return the position */
	  double *veldeg,		/*            velocity */
	  double *t)			/*        and time */
{
   double position;
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(uid, cid, axis, 0) < 0) {
      NTRACE(0, uid, cid, "drift_cmd: failed to set MS.OFF");
      return(-1);
   }
/*
 * Get the MEI semaphore and proceed
 */
   if(semTake(semMEI,60) == ERROR) {
      NTRACE_2(0, uid, cid, "drift_cmd: failed to get semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }
   
   get_velocity(axis<<1,&drift_velocity[axis]);
   semGive (semMEI);
   drift_break[axis]=TRUE;

   taskDelay(3);
   if(semTake(semMEI, 60) == ERROR) {
      NTRACE_2(0, uid, cid, "drift_cmd: failed to retake semMEI: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }

   taskLock();				/* enforce coincidental data */
   get_position_corr(2*axis, &position);
   *t = sdss_get_time();
   taskUnlock();
   semGive(semMEI);

   if(*t < 0) {
      NTRACE_1(0, uid, cid, "drift_cmd: bad time %g", *t);
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
**	drift_break
**	frame_break
**	axis_queue
**	sdssdc
**
**=========================================================================
*/
int 
mcp_move(int uid, unsigned long cid,
	 int axis,			/* the axis to move */
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
   if(nparam == 0 && set_ms_off(uid, cid, axis, 0) < 0) {
      NTRACE(0, uid, cid, "mcp_move: failed to set MS.OFF");
      return(-1);
   }
/*
 * Proceed with the MOVE
 */
   if(sdss_get_time() < 0) {
      NTRACE(0, uid, cid, "mcp_move: bad time");
      return(-1);
   }
   
   queue = &axis_queue[axis];
   
   frame = (struct FRAME *)malloc(sizeof(struct FRAME));
   if(frame == NULL) {
      NTRACE_2(0, uid, cid, "Cannot allocate frame: (%d) %s", errno, strerror(errno));
      return(-1);
   }

   /* Log this. We may not have any other way to see what parameters were passed. */
   NTRACE_2(1, uid, cid, "mcp_move: %s, nparam = %d", axis_name(axis), nparam);
   if(nparam > 0) {
      NTRACE_2(1, uid, cid, "mcp_move: MOVE p0 %f", params[0], 0);
      if(nparam > 1) {
        NTRACE_2(1, uid, cid, "mcp_move: MOVE p1 %f", params[1], 0);
        if(nparam > 2) {
          NTRACE_2(1, uid, cid, "mcp_move: MOVE p2 %f", params[2], 0);
        }
      }
   }

   switch (nparam) {
    case -1:
    case 0:
      tm_get_position(2*axis, &stop_position[axis]);
      frame_break[axis] = TRUE;

      if(semTake(semSDSSDC, NO_WAIT) != ERROR) {
	 sdssdc.tccmove[axis].position = 0;
	 sdssdc.tccmove[axis].velocity = 0;
	 sdssdc.tccmove[axis].time = 0;
	 semGive(semSDSSDC);
      }

      if(frame != NULL) free(frame);
      
      tcc_may_release_semCmdPort = 1;

      return(0);
    case 1:
      position = params[0];
      mcp_move_va(axis, position*ticks_per_degree[axis],
		  0.9*max_velocity[axis]*ticks_per_degree[axis],
		  0.9*max_acceleration[axis]*ticks_per_degree[axis]);

      return(0);
    case 2:
      position = params[0];
      velocity = params[1];

      tm_get_position(2*axis,&pos);
      frame->end_time =
         sdss_get_time()+ abs((pos/ticks_per_degree[axis] - position)/velocity);
      frame->end_time = fmod(frame->end_time, ONE_DAY);

      break;
    case 3:
      position = params[0];
      velocity = params[1];
      frame->end_time = params[2];

      if(sdss_delta_time(frame->end_time, sdss_get_time()) < 0.0) {
        if(frame != NULL) free(frame);
        NTRACE_1(0, uid, cid, "MOVE CMD: requested time=%f", frame->end_time);
        NTRACE_1(0, uid, cid, "            current time=%f", sdss_get_time());
        traceMode(traceModeGet() & ~0x1); /* XXX */
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
	 dt -= 0.043;	/* modify time to reduce error during transition */
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
      NTRACE_2(0, uid, cid, "Move: Max vel. for %s exceeded: %ld/1000",
	    axis_name(axis), (long)(1e3*velocity));

      {
	 char key[20];
	 sprintf(key, "%sMaxVelRequested", axis_abbrev(axis));

	 sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, velocity);
      }
	       
      velocity = (velocity > 0) ? max_velocity[axis] : -max_velocity[axis];
   }
   
   frame->velocity = (double)velocity;
   frame->nxt = NULL;

   if(semTake(semSDSSDC, NO_WAIT) != ERROR) {
      sdssdc.tccmove[axis].position=
	(long)(frame->position*ticks_per_degree[axis]);
      sdssdc.tccmove[axis].velocity=
	(long)(frame->velocity*ticks_per_degree[axis]);
      sdssdc.tccmove[axis].time=(long)(frame->end_time*1000);

      semGive(semSDSSDC);
   }

#if 0					/* XXX */
   printf("%s MOVE %f %f %f  %10.4f\n", axis_name(axis),
	  frame->position, frame->velocity, frame->end_time,
	  frame->end_time - sdss_get_time());
#endif
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

     NTRACE_1(0, uid, cid, "Offsetting %s", axis_name(axis));
	 NTRACE_2(0, uid, cid, "     pos = %d, vel = %d",
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
   int uid = 0, cid = 0;
   double position,velocity,frame_time;
   struct FRAME_QUEUE *queue;
   int i;

   for(i = 0; i < OFF_MAX; i++) {
      if(offset_queue_end[axis][i] == NULL) break;
   }
   
   if(i >= OFF_MAX) {
      NTRACE(0, uid, cid, "mcp_plus_move: offset active");
      return(-1);
   }

   queue = &axis_queue[axis];
   
   switch (nparam) {
    case -1:
    case 0:
      break;				/* NULL offset - does nothing */
    case 1:
      params[1] = 0;			/* velocity */
    case 2:				/* FALL THROUGH */
      position = params[0];
      velocity = params[1];

      if(position == 0.0 && (nparam == 2 && velocity == 0.0)) break;
      
      offset[axis][i][0].nxt = &offset[axis][i][1];
      offset[axis][i][0].position = 0;
      offset[axis][i][1].position = position;
      offset[axis][i][0].velocity = 0;
      offset[axis][i][1].velocity = velocity;
      offset[axis][i][0].end_time = 0;
/*
 * short offsets are give some extra time for smooth ramp.  long offsets
 * are spread over a time period averaging 0.8 degs per second
 */
      if(fabs(position) < 0.3) {
	 offset[axis][i][1].end_time = 0.75;
      } else {
	 offset[axis][i][1].end_time = ((int)(fabs(position/0.3)*20))/20.;
      }
      
      offset_idx[axis][i] = 0;
      offset_queue_end[axis][i] = queue->end;
      break;
    case 3:
      position = params[0];
      velocity = params[1];
      frame_time = params[2];

      NTRACE(0, uid, cid, "Attempt to specify +MOVE p v t");
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

#if 0
   printf("%p: queue_end=%p, position=%f, velocity=%f, end_time=%f\n",
	  &offset[axis][i],offset_queue_end[axis][i],
	  offset[axis][i][1].position,
	  offset[axis][i][1].velocity,
	  offset[axis][i][1].end_time);
#endif
   
   if(semTake(semSDSSDC, NO_WAIT) != ERROR) {
      sdssdc.tccpmove[axis].position =
	(long)(offset[axis][i][1].position*ticks_per_degree[axis]);
      sdssdc.tccpmove[axis].velocity =
	(long)(offset[axis][i][1].velocity*ticks_per_degree[axis]);
      sdssdc.tccpmove[axis].time =
	(long)(offset[axis][i][1].end_time*1000);

      semGive(semSDSSDC);
   }
   
   return(0);
}
