/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	axis_cmds.c						*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
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
#include "ioLib.h"
#include "data_collection.h"
#include "io.h"
#include "math.h"
#include "tm.h"
#include "axis.h"
#include "cw.h"

#define	DSP_IO_BASE		(0x300)			/* in A16/D16 space. */
/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/
/* Prototypes */
void axis_DIO316_shutdown(int type);
void DIO316_interrupt(int type);
int DIO316_initialize(unsigned char *addr, unsigned short vecnum);
void axis_DID48_shutdown(int type);
void DID48_interrupt(int type);
int DID48_initialize(unsigned char *addr, unsigned short vecnum);
void AZ();
void AL();
void IR();
int inc_acks(int axis);
int Read_DSP(unsigned addr);
void Write_DSP(unsigned addr, int dm);
void save_firmware();
void restore_firmware();
int sdss_init();
int sdss_home(int axis);
int init_coeffs(int axis);
int sdss_error(int error_code);
float sdss_get_time();
float get_time();
void stop_frame(int axis,double pos,double sf);
void end_frame(int axis,int index,double sf);
void start_tm_TCC();
void start_tm_TCC_test();
void test_rotfiducials_idx (int axis, double pos, int fididx);
void amp_reset(int axis);
void test_latch (int ticks);
void latch_it ();
void latchstart ();
void latchprint (char *description);
void latchexcel (int axis);
void test_rotfiducials_idx (int axis, double pos, int fididx);
void set_primary_fiducials (int axis,int fididx,long pos);
void set_fiducials (int axis);
void save_fiducials (int axis);
void set_fiducials_all ();
void save_fiducials_all ();
void restore_fiducials (int axis);
void restore_fiducials_all ();
void print_max ();
void tm_load_frame();
void load_frames_trigger(int axisnum);
void lfStart();
float sdss_delta_time(float t2, float t1);
void DIO316ClearISR_delay (int delay, int bit);
int tm_frames_to_execute(int axis);

#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONCE if (YES)
#define ONE_DAY	86400
/* Global Data */
struct LATCH_POS
{
	int axis;
	int ref;
	int data;
	double pos1;
	double pos2;
};
#define MAX_LATCHED	2000
int latchidx=0;
int LATCH_verbose=FALSE;
struct LATCH_POS latchpos[MAX_LATCHED];
int errmsg_max[3]={400,200,100};
SEM_ID semMEI=NULL;
SEM_ID semSLC=NULL;
SEM_ID semLATCH=NULL;
SEM_ID semLOADFRAME=NULL;
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
float time1[3],time2[3];
int CALC_verbose=FALSE;
int CALCOFF_verbose=FALSE;
int CALCADDOFF_verbose=FALSE;
int CALCFINAL_verbose=FALSE;
int FRAME_verbose=FALSE;
struct DIAG_Q {
	double p;
	double v;
	double a;
	double ji;
	double tim;
};
struct DIAG_Q *diagq=NULL;
int diagq_siz,diagq_i;
int DIAGQ_verbose=FALSE;
#define FRMHZ	20
#define FLTFRMHZ	20.
#define MAX_CALC 20
static double tim[3][MAX_CALC],p[3][MAX_CALC],v[3][MAX_CALC],a[3][MAX_CALC],ji[3][MAX_CALC];
static double timoff[3][MAX_CALC],poff[3][MAX_CALC],voff[3][MAX_CALC],
	aoff[3][MAX_CALC],jioff[3][MAX_CALC];
float time_off[3]={0.0,0.0,0.0};
double stop_position[3]={0.0,0.0,0.0};
double drift_velocity[3]={0.0,0.0,0.0};
int frame_break[3]={FALSE,FALSE,FALSE};
int drift_break[3]={FALSE,FALSE,FALSE};
int DRIFT_verbose=FALSE;
int drift_modify_enable=FALSE;
#define OFF_MAX	8
struct FRAME offset[3][OFF_MAX][2];
struct FRAME *offset_queue_end[3][OFF_MAX]={
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL},
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL},
				{NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL}
				};
int offset_idx[3][OFF_MAX]={
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0}
};

char *reboot_cmd(char *cmd)
{
  printf (" <break> command fired\r\n");
  reboot(BOOT_NORMAL);
  return "";
}
char *correct_cmd(char *cmd)
{
  extern SEM_ID semMEIUPD;
  extern struct TM_M68K *tmaxis[];
  extern struct FIDUCIARY fiducial[3];
  extern long fiducial_position[3];
  long pos;

  printf (" CORRECT command fired\r\n");
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (fiducial[axis_select].markvalid)
  {
    if (semTake (semMEIUPD,60)!=ERROR)
    {
      pos=fiducial_position[axis_select];
/* use optical encoder for axis 4 */
      if (axis_select==INSTRUMENT) 
      { 
#ifdef ROT_ROTARY_ENCODER
        pos += ((*tmaxis[axis_select]).actual_position2-fiducial[axis_select].mark);
#else
        pos += ((*tmaxis[axis_select]).actual_position-fiducial[axis_select].mark);
#endif
        tm_set_pos(axis_select*2+1,pos);
/*	     tm_set_pos(axis_select*2-1,pos);*/
#ifdef ROT_ROTARY_ENCODER
        pos = (pos*OPT_TICK)/ROT_TICK;
#endif
        tm_set_pos(axis_select*2,pos);
      }
      if (axis_select==ALTITUDE)
      { 
        pos += ((*tmaxis[axis_select]).actual_position-fiducial[axis_select].mark);
        tm_set_pos((axis_select*2)&0x6,pos);
        tm_set_pos((axis_select*2)+1,pos);
      }
      if (axis_select==AZIMUTH)
      { 
        pos += ((*tmaxis[axis_select]).actual_position-fiducial[axis_select].mark);
        tm_set_pos((axis_select*2)&0x6,pos);
        tm_set_pos((axis_select*2)+1,pos);
      }
      fiducial[axis_select].mark=fiducial_position[axis_select];
      semGive (semMEIUPD);
    }
    else
      return "ERR: semMEIUPD";
  }
  else
    return "ERR: fiducial for axis not crossed";
  return "";
}

double sec_per_tick[3]={AZ_TICK,ALT_TICK,ROT_TICK};
double ticks_per_degree[]={AZ_TICKS_DEG,
				ALT_TICKS_DEG,
				ROT_TICKS_DEG};
static char *drift_ans={"360.00000 0.500000 5040.00000                                "};
char *drift_cmd(char *cmd)
{
  double position,velocity;
  double arcdeg, veldeg;
  float time;
/*  extern struct TM_M68K *tmaxis[];*/

/*  printf (" DRIFT command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  if (semTake (semMEI,60)!=ERROR)
  {
    drift_break[axis_select]=TRUE;
    get_velocity(axis_select<<1,&drift_velocity[axis_select]);
    semGive (semMEI);
    velocity=drift_velocity[axis_select];
  }
  else
    return "ERR: semMEI";
/*  while ((tm_frames_to_execute(axis_select)>1)
    taskDelay(3);*/
  taskDelay(3);
  if (semTake (semMEI,60)!=ERROR)
  {
    taskLock();
    get_position(axis_select<<1,&position);
    time=sdss_get_time();
    taskUnlock();
    semGive (semMEI);
    if (time<0) return "ERR: BAD TIME";
  }
  else
    return "ERR: semMEI";
  veldeg=(sec_per_tick[axis_select]*velocity)/3600.;
  arcdeg=(sec_per_tick[axis_select]*position)/3600.;
  sprintf (drift_ans,"%lf %lf %f",arcdeg,veldeg,time);
  return drift_ans;
}

char *encl_cmd(char *cmd)
{
  printf ("ENCL command fired\r\n");
  return "";
}

char *id_ans={"0 None Specified MMM DD 19YY\r\nDSP Firmware=Vxxx.xx Rxx Sx, Option=xxxx Axes=x"};
char *id_cmd(char *cmd)
{
  static char *axis_name[]={"None Specified","Azimuth","Altitude","Rotator",
		};

/*  printf (" ID command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  sprintf (id_ans,"%d %s %s\r\nDSP Firmware=V%f R%d S%d, Option=%d, Axes=%d",
		axis_select,axis_name[axis_select+1],__DATE__,
		dsp_version()/1600.,dsp_version()&0xF,(dsp_option()>>12)&0x7,
		dsp_option()&0xFFF,dsp_axes());
  return id_ans;
}

char *init_cmd(char *cmd)
{
/*  int e;*/
/*  char buffer[MAX_ERROR_LEN];*/
  int state;
/*  extern struct SDSS_FRAME sdssdc;*/

/*  printf (" INIT command fired axis=%d\r\n",axis_select);*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  state=tm_axis_state(axis_select<<1);
/*  if (state>2)*/		/* normal...NOEVENT,running, or NEW_FRAME */
/*  {*/
    if (state>2)		/* normal...NOEVENT,running, or NEW_FRAME */
      printf ("\r\n  INIT axis %d: not running, state=%x",axis_select<<1,state);
    tm_controller_idle (axis_select<<1);
    tm_controller_idle (axis_select<<1);
    tm_controller_idle (axis_select<<1);
    tm_reset_integrator(axis_select<<1);
    taskDelay(1);
/*  }*/
  drift_break[axis_select]=FALSE;
  frame_break[axis_select]=FALSE;
  if (axis_select==2)
    amp_reset(axis_select<<1);
  else
  {
    amp_reset(axis_select<<1);
    amp_reset((axis_select<<1)+1);
  }
/*
  if (axis_select==0)
  {
    if (sdssdc.status.i7.il0.az_brake_engaged)
    {
      printf("Azimuth Brake Turned Off                ");
      tm_az_brake_off();
    }
  }
  if (axis_select==1)
  {
    if (sdssdc.status.i7.il0.alt_brake_engaged)
    {
      printf("Altitude Brake Turned Off               ");
      tm_alt_brake_off();           
    }
  }
*/
  tm_controller_run (axis_select<<1);
  tm_controller_run (axis_select<<1);
  tm_controller_run (axis_select<<1);
  if (semTake (semMEI,60)!=ERROR)
  {
    v_move(axis_select<<1,(double)0,(double)5000);
    semGive (semMEI);
  }
  axis_queue[axis_select].active=axis_queue[axis_select].end;
  axis_queue[axis_select].active=NULL;

#ifdef NOTDEFINED
  e = dsp_init(DSP_IO_BASE);
  if (e)
  {
    error_msg(e, buffer) ;	/* convert an error code to a human message */
    printf("dsp_init failed--%s (%d)\n", buffer, e);
  }
  else
    printf("dsp_init Passed!\n");
#endif
  return "";
}

char *maxacc_cmd(char *cmd)
{
  printf (" MAXACC command fired\r\n");
  return "";
}

char *maxvel_cmd(char *cmd)
{
  printf (" MAXVEL command fired\r\n");
  return "";
}

char *mc_dump_cmd(char *cmd)
{
  printf (" MC.DUMP command fired\r\n");
  return "";
}

char *mc_maxacc_cmd(char *cmd)
{
  printf (" MC.MAXACC command fired\r\n");
  return "";
}

char *mc_maxpos_cmd(char *cmd)
{
  printf (" MC.MAXPOS command fired\r\n");
  return "";
}

char *mc_maxvel_cmd(char *cmd)
{
  printf (" MC.MAXVEL command fired\r\n");
  return "";
}

char *mc_minpos_cmd(char *cmd)
{
  printf (" MC.MINPOS command fired\r\n");
  return "";
}

char *move_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;

  double position,velocity,pos;
  struct FRAME *frame,*nxtque;
/*  struct FRAME *nframe;*/
  struct FRAME_QUEUE *queue;
  int cnt;
  double dt;

/*  printf (" MOVE command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  queue = &axis_queue[axis_select];
  frame = (struct FRAME *)malloc (sizeof(struct FRAME));
  if (frame==NULL) return "ERR: OUT OF MEMORY";
  if (sdss_get_time()<0) return "ERR: BAD TIME";
  cnt=sscanf (cmd,"%12lf %12lf %12lf",&position,&velocity,&frame->end_time);
  switch (cnt)
  {
    case -1:
    case 0:
        tm_get_pos(axis_select<<1,&stop_position[axis_select]);
        frame_break[axis_select]=TRUE;
/*
	while (tm_frames_to_execute(axis_select)>1)taskDelay (1);
	printf("\r\n frames to execute=%d",tm_frames_to_execute(axis_select));
	velocity=(ticks_per_degree[axis_select]*max_velocity[axis_select])/2;
        tm_start_move (axis_select<<1,
		velocity,
		(ticks_per_degree[axis_select]*max_acceleration[axis_select])/2,
		position);
        printf ("\r\n Repositioning MOVE cmd to position=%lf @vel=%lf,@accel=%lf",
		position,velocity,
		(ticks_per_degree[axis_select]*max_acceleration[axis_select])/2);
*/
	if (frame!=NULL) free (frame);
	sdssdc.tccmove[axis_select].position=0;
	sdssdc.tccmove[axis_select].velocity=0;
	sdssdc.tccmove[axis_select].time=0;
        return "";
/*        break;*/
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
        if (sdss_delta_time((float)frame->end_time,sdss_get_time())<0.0)
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
	  dt=(double)sdss_delta_time((float)frame->end_time,sdss_get_time());
	  taskUnlock();
	  dt -=.043;
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
  sdssdc.tccmove[axis_select].position=
	(long)(frame->position*ticks_per_degree[axis_select]);
  sdssdc.tccmove[axis_select].velocity=
	(long)(frame->velocity*ticks_per_degree[axis_select]);
  sdssdc.tccmove[axis_select].time=(long)(frame->end_time*1000);
  frame->position=position;
  if (fabs(velocity)>max_velocity[axis_select]) 
      velocity=max_velocity[axis_select];
  frame->velocity=(double)velocity;
  frame->nxt = NULL;

/* queues are initialized with one dummy entry at sdss_init time */
  taskLock();
  nxtque = queue->end;
  nxtque->nxt = frame;
  if (queue->active==NULL)/* end of queue, and becomes active frame */
    queue->active=frame;
  if (cnt==-1)		/* this frame becomes active frame */
  {
/*
    queue->active=frame;
    nframe = (struct FRAME *)malloc (sizeof(struct FRAME));
    nframe->position=frame->position;
    nframe->velocity=frame->velocity;
    nframe->end_time=frame->end_time+2.;
    nxtque = queue->end;
    frame->nxt = nframe;
    nframe->nxt = NULL;
    queue->cnt++;
    frame=nframe;
    frame_break[axis_select]=TRUE;
*/
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
int calc_frames (int axis, struct FRAME *iframe, int start)
{
  double dx,dv,dt,vdot;
  double ai,j,t,lai,lj,lt,ldt;
  struct FRAME *fframe;
  struct FRAME *lframe;
  int i;
  
  while (iframe->nxt==NULL) taskDelay (3);
  taskLock();
  fframe=iframe->nxt;
  dx=fframe->position-iframe->position;
  dv=fframe->velocity-iframe->velocity;
  dt=(double)sdss_delta_time((float)fframe->end_time,(float)iframe->end_time);
  vdot=dx/dt;
  ai=(2/dt)*((3*vdot)-(2*iframe->velocity)-fframe->velocity);
  j=(6/(dt*dt))*(iframe->velocity+fframe->velocity-(2*vdot));
/* neccessary if for loop not executed; calc of t*/
  t=(start+1)/FLTFRMHZ+time_off[axis];	/* for end condition */
  if (CALC_verbose)
  {
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
/* last one with a portion remaining */
/*  printf ("\r\nCheck for FINAL: time_off=%f, i=%d, start=%d, t=%f, dt=%f",
		time_off[axis],i,start,t,dt);*/

  if ( ((int)(i+start)==(int)((dt-time_off[axis])*FLTFRMHZ)) &&
	(t!=(dt-time_off[axis])) )
  {
    ldt=(((dt-time_off[axis])*FLTFRMHZ)-(int)((dt-time_off[axis])*FLTFRMHZ))/FLTFRMHZ;
/*    lt = dt-t;*/
/*    t = (1/FLTFRMHZ)-lt;*/
    t = (1/FLTFRMHZ)-ldt;
    lt = dt;
/*    tim[axis][i]=((dt*FRMHZ)-(int)(dt*FRMHZ))/FLTFRMHZ;*/
    tim[axis][i]=1/FLTFRMHZ;
    lframe=fframe;
    if (lframe->nxt==NULL) 
    {
      printf ("\r\nCALC FRAME: next frame required to finish");
      taskUnlock();
      return i;
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
    taskUnlock();
    return (i+1);
  }
  else 
  {
    if (i>(MAX_CALC-1)) printf ("\r\n calc_frames has problems %d",i);
    taskUnlock();
    return i;
  }
}
int calc_offset (int axis, struct FRAME *iframe, int start, int cnt)
{
  double dx,dv,dt,vdot;
  double ai,j,t;
  struct FRAME *fframe;
  int i,ii;
  
  fframe=iframe->nxt;
  dx=fframe->position-iframe->position;
  dv=fframe->velocity-iframe->velocity;
  dt=(double)sdss_delta_time((float)fframe->end_time,(float)iframe->end_time);
/*  dt=fframe->end_time-iframe->end_time;*/
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
  for (i=0;i<(int)min(MAX_CALC-1,
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
/*
    aoff[axis][ii]=0;
    jioff[axis][ii]=0;
*/
    if (CALCOFF_verbose)
      printf ("\r\n%d @%lf Secs: ti=%lf, p=%12.8lf, v=%12.8lf, a=%12.8lf",
        ii,t,timoff[axis][ii],poff[axis][ii],voff[axis][ii],aoff[axis][ii]);
  }
  return i;
}
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
void start_frame(int axis,double time)
{
/*
  int e;
  FRAME frame;
  double pos;
*/
  int lcnt;
  
  time_off[axis]=0.0;
  while ((lcnt=tm_frames_to_execute(axis))>1)
  {
/*    printf("\r\nDwell frames left=%d",lcnt);*/
    taskDelay(3);
  }
  taskDelay(5);
/*
  lcnt=tm_frames_to_execute(axis);
  printf("\r\nDwell frames left=%d",lcnt);
*/
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
     time = (double)sdss_delta_time((float)time,sdss_get_time());
/*     printf("\r\ntime to dwell=%lf",time);*/
     dsp_dwell (axis<<1,time);
/*
     set_gate(axis<<1);
     e=frame_m(&frame,"0l t un d",axis<<1,
	time,
	FTRG_TIME,NEW_FRAME);    
     reset_gate(axis<<1);
*/
     semGive (semMEI);
  }
/*
  while ((lcnt=tm_frames_to_execute(axis))>1)
  {
    tm_get_pos(axis_select<<1,&pos);
    printf("\r\nAfter Dwell pos=%lf",pos);
    printf("\r\nDwell frames left=%d",lcnt);
  }
  tm_get_pos(axis_select<<1,&pos);
  printf("\r\nAfter Dwell pos=%lf",pos);
  printf("\r\nDwell frames left=%d",lcnt);
*/
  printf ("\r\nSTART axis=%d: time=%lf",axis<<1,time);
}
int get_frame_cnt(int axis, struct FRAME *iframe)
{
  struct FRAME *fframe;
  double dt;
  int cnt;

  fframe=iframe->nxt;
  dt=fframe->end_time-iframe->end_time;
  cnt=(int)((dt-time_off[axis])*FLTFRMHZ);
/*  printf("\r\nfirst cnt=%d",cnt);*/
  if ( ((dt-time_off[axis])*FLTFRMHZ)>(int)((dt-time_off[axis])*FLTFRMHZ) )
  {
    cnt++;
/*    printf(" final cnt=%d",cnt);*/
  }
  return cnt;
}

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
/*
      e=frame_m(&frame,"0l xvat un d",axis<<1,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,
	tim[axis][i],
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FTRG_TIME,NEW_FRAME);    
*/
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
void print_frames(int axis)
{
  int i;
  double sf;

  sf=ticks_per_degree[axis];
  for (i=0;i<MAX_CALC;i++)
        printf ("\r\n axis=%d (%d): p=%12.8lf, v=%12.8lf, a=%12.8lf, \r\nj=%12.8lf,t=%12.8lf",
	axis<<1,i,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,ji[axis][i]*sf,
	tim[axis][i]);    
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

void stop_frame(int axis,double pos,double sf)
{
  int stopped;

  printf ("\r\nSTOP axis=%d: p=%12.8lf",
	axis<<1,(double)pos);
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    set_stop(axis<<1);
    stopped=motion_done(axis<<1);
    semGive (semMEI);
  }
  while(!stopped)
  {  
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
      stopped=motion_done(axis<<1);
      semGive (semMEI);
    }
/*    printf("\r\nStopping");*/
    taskDelay(1);
  }
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    clear_status(axis<<1);
    semGive (semMEI);
  }
  printf("\r\nStopped");
/* do not reposition, just stop */
/*  tm_start_move (axis<<1,1*sf,.8*sf,pos);*/
}
void drift_frame(int axis,double vel,double sf)
{
  int e;
/*  int lcnt;*/
  FRAME frame;
  
  printf ("\r\nDRIFT axis=%d: v=%12.8lf",
	axis<<1,
	(double)vel);
/*  printf("\r\nDrift frames left=%d",tm_frames_to_execute(axis));*/
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
/*  this should work but instaneously to velocity */
/*    set_accel(axis<<1,(double)0.0);
    set_velocity(axis<<1,(double)vel);*/


      e=frame_m(&frame,"0l vaj un d",axis<<1,
	(double)vel,(double).8*sf,
	(double)0.0,
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_JERK|FTRG_VELOCITY,NEW_FRAME);
      e=frame_m(&frame,"0l va u d",axis<<1,
	(double)vel,(double)0.0,
	FUPD_ACCEL|FUPD_VELOCITY,0);

/*      dsp_set_last_command(dspPtr,axis<<1,(double)p[axis][index]*sf);*/
      semGive (semMEI);
  }

/*
  while ((lcnt=tm_frames_to_execute(axis))>1) 
  {
    taskDelay(3);
  }
*/
/*    printf("...%d",lcnt);*/
}
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
/*      tm_start_move (axis<<1,2*sf,4*sf,frame->position);*/
      dsp_set_last_command(dspPtr,axis<<1,(double)p[axis][index]*sf);
      semGive (semMEI);
      printf ("\r\nEND axis=%d (%d): p=%12.8lf, v=%12.8lf, a=%12.8lf, j=%12.8lf,t=%12.8lf",
	axis<<1,index,
	(double)p[axis][index]*sf,(double)v[axis][index]*sf,(double)a[axis][index]*sf,
	(double)ji[axis][index]*sf,
	tim[axis][index]);    
  }
}
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
#define LOAD_MAX        20
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

void tm_TCC(int axis)
{
  int cnt, lcnt, cntoff;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;
/*  extern struct TM_M68K *tmaxis[];*/
  double position;
  double velocity;
  long pos;
  int idx;
  int status;
  extern int axis_alive;
  
  tm_controller_run (axis<<1);
  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
	ticks_per_degree[axis]);
  FOREVER
  {
/* task should idle here with no input pvt */
    while (axis_queue[axis].active==NULL)
    {
      axis_alive |= (1<<axis);
/* in case drifting, no new pvt, and need to stop */
      if (frame_break[axis])
      {
        stop_frame(axis,stop_position[axis],(double)ticks_per_degree[axis]);
	frame_break[axis]=FALSE;
      }
      taskDelay (3);
    }
    frame=axis_queue[axis].active;
    drift_break[axis]=FALSE;

/* reposition if neccessary */
    if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
    {
      get_position(axis<<1,&position);
      get_velocity(axis_select<<1,&velocity);
      semGive (semMEI);
      pos=(long)position;
/*      printf("\r\nCheck Params for repositioning");*/
    }
    if ( (abs((frame->position*ticks_per_degree[axis])-position)>
		(.01*ticks_per_degree[axis])) && (fabs(velocity)==0) )
    {
      while ((lcnt=tm_frames_to_execute(axis))>1)
      {
        printf ("\r\n frames left=%d",lcnt);
        taskDelay(1);
      }
      printf("\r\nRepositioning");
      tm_start_move (axis<<1,
		1*(double)ticks_per_degree[axis],
		.5*(double)ticks_per_degree[axis],
		frame->position*(double)ticks_per_degree[axis]);
      printf ("\r\nAxis %d Repositioning TCC cmd to position=%lf from pos=%ld, diff=%ld>%ld",
		axis,frame->position*ticks_per_degree[axis],pos,
		abs((frame->position*ticks_per_degree[axis])-position),
		(long)(.01*ticks_per_degree[axis]) );
      status=TRUE;
      while ((abs((frame->position*ticks_per_degree[axis])-pos)>
		(.01*ticks_per_degree[axis]))&&status)
      {
        if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
        {
	  status=in_motion(axis<<1);
          get_position(axis<<1,&position);
          semGive (semMEI);
          pos=(long)position;
        }
        taskDelay (1);
        printf("\r\nStill repositioning");
      }
    }
/*    else
      printf("\r\n nonzero vel=%lf",velocity);*/

/* check for time */
    while ((frame!=NULL)&&
	  (sdss_delta_time((float)frame->end_time,sdss_get_time())<0.0))
    {
      frame = frame->nxt;
      axis_queue[axis].active=frame;
      printf ("\r\n Frame deleted due to time");
    }

    if (frame!=NULL)
    {
/*      while (sdss_delta_time((float)frame->end_time,sdss_get_time())>4.0)
	taskDelay (2*60);*/
      start_frame (axis,frame->end_time);
      while ((frame->nxt==NULL)&&
	    (sdss_delta_time((float)frame->end_time,sdss_get_time())>0.02))
      {
        printf ("\r\n waiting for second frame");
        taskDelay (3);
      }
/*
      if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
      {
        get_velocity(axis_select<<1,&velocity);
        semGive (semMEI);
      }
      printf("\r\nAfter dwell vel=%lf",velocity);
*/
      while ( (frame->nxt!=NULL) || (axis_queue[axis].active!=NULL) &&
	((!frame_break)&&(!drift_break))
			)
      {
        frame_cnt=get_frame_cnt(axis,frame);
/*        printf ("\r\n frames_cnt=%d",frame_cnt);*/
        frame_idx=0;
        for (i=0;i<OFF_MAX;i++)
	{
	  if (offset_queue_end[axis][i]==frame)
	  {
/*	    printf ("\r\nShutdown offset");*/
	    if (frame->end_time>offset[axis][i][1].end_time)
	    {
              frame->position+=(offset[axis][i][1].position+
               (offset[axis][i][1].velocity*(offset_idx[axis][i]/20.-
		offset[axis][i][1].end_time)));
	      frame->velocity+=offset[axis][i][1].velocity;
	    }
	    else
	    {
	      clroffset(axis,1);
	      cntoff=calc_offset(axis,&offset[axis][i][0],offset_idx[axis][i],1);
	      frame->position+=(poff[axis][0]+
               (voff[axis][0]*(offset[axis][i][1].end_time/20.-
		frame->end_time)) );
	      frame->velocity+=voff[axis][0];
	    }
	    offset_idx[axis][i]=0;
	    offset_queue_end[axis][i]=NULL;
	  }
	}
	while (frame_cnt>0)
        {
          cnt=calc_frames(axis,frame,frame_idx);
/* OFFSET */
          for (i=0;i<OFF_MAX;i++)
	  {
	    clroffset(axis,cnt);
	    if (offset_queue_end[axis][i]!=NULL)
	    {
/*            printf("\r\nCalc offset");*/
	      cntoff=calc_offset(axis,&offset[axis][i][0],
					offset_idx[axis][i],cnt);
	      offset_idx[axis][i]+=cnt;
	    }
/*          printf("\r\nAdd offset");*/
	    addoffset(axis,cnt);
	  }

          frame_idx += cnt;
          frame_cnt -= cnt;

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

          if (cnt==0) taskDelay(3);	/* calc is probably waiting on the next frame due to offset */
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
          if (lcnt<1)
	  {
	    printf ("\r\n frames left=%d, frame cnt=%d, cnt=%d",
		lcnt,frame_cnt,cnt);
            printf ("\r\n no new frames calc axis=%d; end_time=%lf, time=%lf",
		axis,frame->end_time,sdss_get_time());
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
	    (sdss_delta_time((float)frame->end_time,sdss_get_time())>.02))
          taskDelay (1);
        while ((frame->nxt==NULL)&&((lcnt=tm_frames_to_execute(axis))>1)) 
	  taskDelay(1);
      }
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
        stop_frame(axis,stop_position[axis],(double)ticks_per_degree[axis]);
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
void start_tm_TCC_test()
{
  taskSpawn("tmAztest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlttest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRottest",62,VX_FP_TASK,20000,(FUNCPTR)tm_TCC_test,
		2,0,0,0,0,0,0,0,0,0);
}
void tm_pos_vel(int axis,int vel, int accel)
{
  int e;
  FRAME frame;
    
  if (semTake (semMEI,60)!=ERROR)
  {
    set_gate(axis);
    e=frame_m(&frame,"0l av un d",
  	axis,(double)accel,(double)vel, FUPD_ACCEL|FTRG_VELOCITY,NEW_FRAME);
    e=frame_m(&frame,"0l avt un d",
  	axis,(double)0.0,(double)vel,(double)10.0, 
	FUPD_ACCEL|FUPD_VELOCITY|FTRG_TIME,NEW_FRAME);
    e=frame_m(&frame,"0l av un d",
  	axis,(double)-accel,(double)0.0,
	FUPD_ACCEL|FTRG_VELOCITY,TRIGGER_NEGATIVE|NEW_FRAME);
    e=frame_m(&frame,"0l av un d",
  	axis,(double)0.0,(double)0.0, FUPD_ACCEL|FUPD_VELOCITY,0);
    reset_gate(axis);
/*    e=pcdsp_set_event (dspPtr, axis, NEW_FRAME);*/
    semGive (semMEI);
  }
}	

char *plus_move_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;
  double position,velocity,frame_time;
/*  struct FRAME *nxtque;*/
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
        if (position<.2)
	  offset[axis_select][i][1].end_time=(double).4;
	else
	  offset[axis_select][i][1].end_time=(double)((int)((position/1.0)*20))/20.;
	offset_idx[axis_select][i]=0;
	offset_queue_end[axis_select][i]=queue->end;
/*	printf("\r\n%p: queue_end=%p, position=%lf, velocity=%lf, end_time=%lf",
		&offset[axis_select][i],offset_queue_end[axis_select][i],
		offset[axis_select][i][1].position,
		offset[axis_select][i][1].velocity,
		offset[axis_select][i][1].end_time);*/
        break;
    case 2:
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
        break;
    case 3:
        if (queue->active!=NULL)
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
        break;
  }
  sdssdc.tccpmove[axis_select].position=
	(long)(offset[axis_select][i][1].position*ticks_per_degree[axis_select]);
  sdssdc.tccpmove[axis_select].velocity=
	(long)(offset[axis_select][i][1].velocity*ticks_per_degree[axis_select]);
  sdssdc.tccpmove[axis_select].time=
	(long)(offset[axis_select][i][1].end_time*1000);
  return "";
}

char *mr_dump_cmd(char *cmd)
{
  printf (" MR.DUMP command fired\r\n");
  return "";
}

char *ms_dump_cmd(char *cmd)
{
  printf (" MS.DUMP command fired\r\n");
  return "";
}

char *ms_map_dump_cmd(char *cmd)
{
  printf (" MS.MAP.DUMP command fired\r\n");
  return "";
}

char *ms_map_load_cmd(char *cmd)
{
  printf (" MS.MAP.LOAD?????? command fired\r\n");
  return "";
}

char *ms_off_cmd(char *cmd)
{
  printf (" MS.OFF command fired\r\n");
  return "";
}

char *ms_on_cmd(char *cmd)
{
  printf (" MS.ON command fired\r\n");
  return "";
}

char *ms_pos_dump_cmd(char *cmd)
{
  printf (" MS.POS.DUMP command fired\r\n");
  return "";
}

char *remap_cmd(char *cmd)
{
  printf (" REMAP command fired\r\n");
  return "";
}

char *rot_cmd(char *cmd)
{
/*  printf (" IR command fired\r\n");*/
  axis_select=INSTRUMENT;
  return 0;
}

char *set_limits_cmd(char *cmd)
{
  printf (" SET.LIMITS command fired\r\n");
  return "";
}

char *set_position_cmd(char *cmd)
{
  printf (" SET.POSITION command fired\r\n");
  return "";
}

struct tm t;
long SDSStime=-1;
char *set_time_cmd(char *cmd)
{
  float t1,t2,t3;
  int cnt;
  struct timespec tp;
  int extrasec;

/*  printf (" SET.TIME command fired\r\n");*/
  if ((axis_select<AZIMUTH) ||
    (axis_select>INSTRUMENT)) return "ERR: ILLEGAL DEVICE SELECTION";
  cnt=sscanf (cmd,"%d %d %d %f %f %f",&t.tm_mon,&t.tm_mday,&t.tm_year,&t1,&t2,&t3);
/*  printf ("\r\ncnt=%d",cnt);*/
  if (cnt>4)
  {
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
  }
  else
  {
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
  }
  t.tm_year -= 1900;
  t.tm_mon -= 1;
  tp.tv_sec=mktime(&t)+extrasec;
  tp.tv_nsec=0;
  time1[axis_select]=sdss_get_time();
  SDSStime=tp.tv_sec%ONE_DAY;
/*  timer_start (1);*/
  time2[axis_select]=sdss_get_time();
/*  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);*/
  clock_settime(CLOCK_REALTIME,&tp);
  printf("\r\nt3=%f (extrasec=%d)",t3,extrasec);
/*  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);*/
  return "";
}

void print_time_changes()
{
  int i;

  for (i=0;i<3;i++)
    printf ("\r\nSDSS axis %d:  time1=%lf time2=%lf",i,time1[i],time2[i]);
}
char *stats_cmd(char *cmd)
{
  printf (" STATS command fired\r\n");
  return "";
}

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
char *tel1_cmd(char *cmd)
{
/*  printf (" TEL1 command fired\r\n");*/
  axis_select=AZIMUTH;
  return 0;
}
char *tel2_cmd(char *cmd)
{
/*  printf (" TEL2 command fired\r\n");*/
  axis_select=ALTITUDE;
  return 0;
}
char *ticklost_cmd(char *cmd)
{
  printf (" TICKLOST @ . command fired\r\n");
  return "";
}
static char *time_ans={"01 31 1996 86400.000                "};	/* */
char *time_cmd(char *cmd)
{
  static struct tm *t;
  struct timespec tp;
/*  unsigned long micro_sec;*/

/*  printf (" TIME? command fired\r\n");*/
  clock_gettime(CLOCK_REALTIME,&tp);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
  t = localtime(&tp.tv_sec);
  printf ("t=%p, mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t,t->tm_mon,t->tm_mday,t->tm_year,t->tm_hour,t->tm_min,t->tm_sec);
/*  micro_sec = timer_read (1);*/

  sprintf (time_ans,"%d %d %d %f",t->tm_mon+1,t->tm_mday,t->tm_year+1900,
	sdss_get_time()
/*
	(t->tm_hour*3600.)+(t->tm_min*60.)+
	t->tm_sec+((micro_sec%1000000)/1000000.)
*/
);
  return time_ans;
}
char *cwmov_cmd(char *cmd)
{
  int cw;
  int cwpos;
  extern struct SDSS_FRAME sdssdc;

  printf (" CWMOV command fired\r\n");
  sscanf (cmd,"%d %d",&cw,&cwpos);
/*  printf ("\r\n cw=%d, cwpos=%d",cw,cwpos);*/
  if ((cw<0)||(cw>4))
    return "ERR: Bad CW id (0-3)";
  if ((cwpos<10)||(cwpos>800))
    return "ERR: Position out of Range (10-800)";
  if (taskIdFigure("cw")!=ERROR)
    return "ERR: CW task still active";
  if (taskIdFigure("cwp")!=ERROR)
    return "ERR: CWP task still active";
  if (sdssdc.status.i7.il0.alt_brake_engaged)
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
  if (sdssdc.status.i7.il0.alt_brake_engaged)
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
  if (sdssdc.status.i7.il0.alt_brake_engaged)
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
  taskDelete(taskIdFigure("cw"));
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
char *sp1_cmd(char *cmd)
{
  printf (" SP1 command fired\r\n");
  spectograph_select=SPECTOGRAPH1;
  return 0;
}
char *sp2_cmd(char *cmd)
{
  printf (" SP2 command fired\r\n");
  spectograph_select=SPECTOGRAPH2;
  return 0;
}
char *slitopen_cmd(char *cmd)
{
  printf (" SLIT.OPEN command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_sp_slit_open(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *slitclose_cmd(char *cmd)
{
  printf (" SLIT.CLOSE command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_sp_slit_close(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *cartlatch_cmd(char *cmd)
{
  printf (" CART.LATCH command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_sp_cart_latch(spectograph_select-SPECTOGRAPH1);
  return "";
}
char *cartunlatch_cmd(char *cmd)
{
  printf (" CART.UNLATCH command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  tm_sp_cart_unlatch(spectograph_select-SPECTOGRAPH1);
  return "";
}
static char *slitstatus[]={"    ","OPEN","     ","CLOSE","UNLATCH","LATCH  "};
static char *slitstatus_ans={"SP1 OPEN CLOSE UNLATCH SP2 OPEN CLOSE UNLATCH"};
char *slitstatus_cmd(char *cmd)
{
  extern struct SDSS_FRAME sdssdc;

  printf (" SLIT.STATUS command fired\r\n");
  if ((spectograph_select<SPECTOGRAPH1) ||
    (spectograph_select>SPECTOGRAPH2)) return "ERR: ILLEGAL DEVICE SELECTION";
  slitstatus_ans[2]=0x31+spectograph_select;
  sprintf (&slitstatus_ans[4],"%s %s %s",
	slitstatus[sdssdc.status.i1.il9.slit_door1_opn],
	slitstatus[sdssdc.status.i1.il9.slit_door1_cls+2],
	slitstatus[sdssdc.status.i1.il9.cart_latch1_opn+4]);
  sprintf (&slitstatus_ans[26],"%s %s %s",
	slitstatus[sdssdc.status.i1.il9.slit_door2_opn],
	slitstatus[sdssdc.status.i1.il9.slit_door2_cls+2],
	slitstatus[sdssdc.status.i1.il9.cart_latch2_opn+4]);
  return slitstatus_ans;	
}
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
 
/**********************************************************************
NAME
        NBS_interrupt *NBS_interrupt 

        void *NBS_interrupt   (int type);


DESCRIPTION

RETURNS

KEYWORDS

FILES
        axis_cmds.c axis.h

AUTHOR
        C. Briegel

SEE ALSO

**********************************************************************/
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
/*            logMsg ("NIST_INTERRUPT,%d %d, dio316int_bit=%2x\r\n",
		type,NIST_sec,(short)dio316int_bit,0,0,0);*/
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
struct FIDUCIARY fiducial[3]=
/*	NotValid, mark, index */
	{{FALSE,0,9+24},
	  {FALSE,0,1},
	  {FALSE,0,40+43}};
/* stow     120:44:15.8
   fiducial 120:45:44.9
   medidian 000:21:59.4
*/
/*		           120:45:44.9, 14:39:23:286, 005:58:04:130  */
/*	measured AZ        120:28:03:514, 14:39:23:286, 005:58:04:130  */
/*			   AZ           , ALT         , ROT      */
long fiducial_position[3]={31016188     , 3766415+58807     , 336370/2};
/*long fiducial_position[3]={31016188     , 3766415+58807     , 402702};*/
/*long fiducial_position[3]={31016188     , 3766415+58807     , 640920};*/
/*	measured AZ */
/*long fiducial_position[3]={31016188     , 3766415     , 2015795};*/
/*long fiducial_position[3]={30940462     , 3766415     , 2015795};*/
struct FIDUCIALS az_fiducial[48];
struct FIDUCIALS alt_fiducial[7];
struct FIDUCIALS rot_fiducial[156];
long az_fiducial_position[60];
long alt_fiducial_position[7];
long rot_fiducial_position[156];
long rot_latch=0;
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
void tm_latch()
{
  int i;
  extern int barcode_serial();
  int fididx;
  int status;
  extern struct SDSS_FRAME sdssdc;
  extern int altclino_off;
  extern float altclino_sf;

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
/*        if (!(dio316int_bit&(AZIMUTH_INT|ALTITUDE_INT|INSTRUMENT_INT)))
	  printf ("\r\n dio316int_bit=%x",dio316int_bit);*/
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
          fididx = barcode_serial(3);	/* backwards from what you would think */
	  fididx = barcode_serial(3);
	  if ((fididx>0)&&(fididx<=24))
	  {
	    if (latchpos[latchidx].pos1>0)
		fididx += 24;
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
                printf ("\r\nAXIS %d: ERR=%d, latched pos0=%f,pos1=%f",
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
/*
          fididx = barcode_serial(2);
	  fididx = barcode_serial(2);
*/
          fididx=((int)(abs(sdssdc.status.i4.alt_position-altclino_off)*
            altclino_sf)+7.5)/15;
	  fididx++;
/*	    fididx=2;*/


	  if (fididx!=-1)
	  {
	    fididx--;
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
                printf ("\r\nAXIS %d: ERR=%d, latched pos0=%f,pos1=%f",
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
              printf ("\r\nAXIS %d: latched pos4=%d,rot_latch=%d,idx=%d,abspos=%d",
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
                  printf ("\r\nAXIS %d: ERR=%d, latched pos0=%f,pos1=%f",
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
	          if ((abs(rot_fiducial[fididx].poserr)>200)&&
		      (rot_fiducial_position[fididx]!=0))
                    printf ("\r\nAXIS %d: ERR=%d, latched pos0=%f,pos1=%f",
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
            printf ("\r\nROT FIDUCIAL %d:  mark=%d, pos=%d, last=%d",i,
                rot_fiducial[i].mark,rot_fiducial_position[i],
                rot_fiducial[i].last);
            printf ("\r\n                  err=%d, poserr=%d",
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
void DIO316ClearISR_delay (int delay, int bit)
{
  taskDelay(delay);

  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    arm_latch(TRUE);
    semGive (semMEI);
  }
  if (bit&AZIMUTH_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_ENA);
  if (bit&ALTITUDE_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_ENA);
  if (bit&INSTRUMENT_INT)
    DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_ENA);
}
void test_rotfiducials_idx (int axis, double pos, int fididx)
{
	    if ((fididx<0)&&(fididx>-80))
	    {
	      fididx = -fididx;
	        if ((fididx>45)&&(pos>0))
	          fididx = fididx-76;
	        else 
		  if ((fididx<35)&&(pos<0))
	            fididx = 76+fididx;
	        fididx +=45;
	    }
	    else
            {
	      if ((fididx>0)&&(fididx<80))
              {
	        if ((fididx>45)&&(pos>0))
	          fididx = fididx-76;
	        else 
		  if ((fididx<35)&&(pos<0))
	            fididx = 76+fididx;
	        fididx +=45;
	      }
	    }
	printf ("\r\n pos=%f, fididx=%d",pos,fididx);
}
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
	    printf ("AZ FIDUCIAL %d(%d degs):  mark=%d, pos=%d, last=%d",i,
		(int)(az_fiducial[i].mark/AZ_TICKS_DEG),
		az_fiducial[i].mark,az_fiducial_position[i], az_fiducial[i].last);
	    printf ("\r\n                 err=%d, poserr=%d",
		az_fiducial[i].err,az_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("AZ FIDUCIAL %d:  pos=%d",i,
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
	    printf ("ALT FIDUCIAL %d(%d degs):  mark=%d, pos=%d, last=%d",i,
		(int)(alt_fiducial[i].mark/ALT_TICKS_DEG),
		alt_fiducial[i].mark,alt_fiducial_position[i],
	        alt_fiducial[i].last);
	    printf ("\r\n                 err=%d, poserr=%d",
		alt_fiducial[i].err,alt_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("ALT FIDUCIAL %d:  pos=%d",i,
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
	    printf ("ROT FIDUCIAL %d(%d degs):  mark=%d, pos=%d, last=%d",i,
		(int)(rot_fiducial[i].mark/ROT_TICKS_DEG),
		rot_fiducial[i].mark,rot_fiducial_position[i],
	        rot_fiducial[i].last);
	    printf ("\r\n                  err=%d, poserr=%d",
		rot_fiducial[i].err,rot_fiducial[i].poserr);
          }
	  else
	  {
	    printf ("ROT FIDUCIAL %d:  pos=%d",i,
		rot_fiducial_position[i]);
	  }     
	}
	break;
  }
}
void axis_DID48_shutdown(int type)
{
    printf("AXIS DID48 shutdown: TOD interrupt %d\r\n",tm_DID48);
    if (tm_DID48!=-1)
      DID48_Interrupt_Enable_Control (tm_DID48,5,DID48_INT_DIS);
    taskDelay(30);
}
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
unsigned long NIST_sec;
unsigned char did48int_bit;
unsigned long NIST_cnt=0;
unsigned long SDSS_cnt=0;
#define DAYINSECS	86400
void DID48_interrupt(int type)
{
  extern struct AXIS_STAT axis_stat[];
  extern struct AXIS_STAT persistent_axis_stat[];

          DID48_Read_Port (tm_DID48,5,&did48int_bit);
	  NIST_cnt++;
          if (did48int_bit&NIST_INT)
          {
	    SDSS_cnt++;
	    if (SDSStime>0)
              SDSStime=(SDSStime+1)%DAYINSECS;

/*            NIST_sec=(unsigned long)(1.0312733648*timer_read (1));*/
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

/*            logMsg ("NIST_INTERRUPT,%d %d, did48int_bit=%x, cnt=%d\r\n",
		type,NIST_sec,(short)did48int_bit,NIST_cnt,0,0);*/
          }
          DID48_Write_Reg (tm_DID48,4,0x20);
}
#ifdef MEI_INTERRUPTS_WORK
int addr1=0,addr2=0;
int v1=0, v2=0;
int inc_acks(int axis)
{

    addr1 = dspPtr->global_data + axis + GD_SIZE;               /* irq_count */
    addr2 = addr1 + dspPtr->axes;				/* ack_count */
    v1 = Read_DSP(addr1);
    v2 = Read_DSP(addr2);

    if (v1 != v2)                       /* Is the DSP generating interrkupt */
                Write_DSP(addr2,v1);    /* set ack_count = irq_count */
    return (v1 - v2);
}

int Read_DSP(unsigned addr)
{
  unsigned short *out_addr;
  unsigned short *in_data;

  out_addr=dspPtr->mei_dsp_base+dspPtr->address;
  in_data=dspPtr->mei_dsp_base+dspPtr->data;
  *out_addr = addr | PCDSP_DM;
  return(*in_data);
}

void Write_DSP(unsigned addr, int dm)
{
  unsigned short *out_addr;
  unsigned short *out_data;

  out_addr=dspPtr->mei_dsp_base+dspPtr->address;
  out_data=dspPtr->mei_dsp_base+dspPtr->data;
  *out_addr = addr | PCDSP_DM;
  *out_data = dm;
}
int trigger_int(int bit)
{
  set_bit(bit);
  sysIntDisable(MEI_IRQ);
  reset_bit(23);
  return 0;
}
#endif
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
void amp_reset(int axis)
{
  extern int cw_DIO316;

	DIO316_Write_Port (cw_DIO316,AMP_RESET,1<<axis);
        taskDelay (2);
	DIO316_Write_Port (cw_DIO316,AMP_RESET,0);
}
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
    printf ("AXIS %d: old stop rate=%f\r\n",axis,rate);
    set_stop_rate(axis,SDSS_STOP_RATE);
    get_stop_rate(axis,&rate);
    printf ("AXIS %d:  set stop rate=%f\r\n",axis,rate);

    get_e_stop_rate(axis,&rate);
    printf ("AXIS %d: old e_stop rate=%f\r\n",axis,rate);
    set_e_stop_rate(axis,SDSS_E_STOP_RATE);
    get_e_stop_rate(axis,&rate);
    printf ("AXIS %d:  set e_stop rate=%f\r\n",axis,rate);

    printf ("AXIS %d:  AMP level high, Controller Idle, Disable AMP\r\n",axis);
    set_amp_enable_level(axis,TRUE);
    controller_idle (axis);
    disable_amplifier (axis);
    get_error_limit(axis,&limit,&action);
    printf ("AXIS %d: error limit=%d, action=%d\r\n",axis,(long)limit,action);
    set_error_limit(axis,24000.,ABORT_EVENT);
    get_error_limit(axis,&limit,&action);
    printf ("AXIS %d: SET error limit=%d, action=%d\r\n",axis,(long)limit,action);
    init_coeffs(axis);
  }
  init_io(2,IO_INPUT);
/*  init_io(2,IO_OUTPUT);
  set_io(2,0xFF);*/
  arm_latch(TRUE);
  semMEI = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
  semSLC = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
/*
  semMEI = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
  semSLC = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
*/
  taskSpawn ("tmLatch",49,VX_FP_TASK,10000,(FUNCPTR)tm_latch,0,0,0,0,0,0,0,0,0,0);
  return 0;
}

/* extracted from home2.c 21 Feb 96 */
/* HOME2.C

:Two stage homing routine using the home input

This sample demonstrates a basic homing algorithm.  The home location is found
 based on the home input.
 
Here is the algorithm:
 1) Fast velocity move towards the home sensor.
 2) Stop at the home sensor (Stop Event generated by DSP).
 3) Position move off the home sensor (opposite direction).
 4) Slow velocity move towards the home sensor.
 5) Stop at the home sensor (Stop Event generated by DSP).
 6) Zero the position.

Here is the sensor logic:
 Home input = active high
 Index input = not used

Written for Version 2.4E  
*/

# define SLOW_VEL	200.0
# define FAST_VEL	20000.0
# define SLOW_ACCEL	10000.0
# define FAST_ACCEL	50000.0

int sdss_home(int axis)
{
	double distance;

/* initialize if not done */
        if (!sdss_was_init) sdss_init();
/* make sure no events are generated during home logic configuration */
	set_home(axis, NO_EVENT);
	sdss_error(clear_status(axis));
/* set configuration for a stop event at home position */	
	set_stop_rate(axis, SLOW_ACCEL);
	set_home_index_config(axis, HOME_ONLY);
	set_home_level(axis, TRUE);
        set_home(axis, STOP_EVENT);
/* move towards home */   
	printf("\nQuickly searching for home sensor\n");
	v_move(axis, FAST_VEL, SLOW_ACCEL);	/* velocity move towards home sensor */
	while (!motion_done(axis)) taskDelay(100);

/* found home, but overshot, so get back */	
        set_home(axis, STOP_EVENT);
	sdss_error(clear_status(axis));
/* move off by a dist equal to twice the decel distance */
	distance = FAST_VEL * FAST_VEL / SLOW_ACCEL;
	start_r_move(axis, -distance, FAST_VEL, FAST_ACCEL); 
	while (!motion_done(axis)) taskDelay(100);
/* got closer, now move back to home slowly */	
	printf("\nSlowly searching for home sensor.\n");
	set_stop_rate(axis, FAST_ACCEL);
        set_home(axis, STOP_EVENT);
	v_move(axis, SLOW_VEL, FAST_ACCEL);
	while (!motion_done(axis)) taskDelay(100);
/* found home again, but slowly...zero command and actual position */	
	set_position(axis, 0.0);			
	printf("\nAxis is home.\n");

	return 0;
}

int init_coeffs(int axis)
{
	short coeff[COEFFICIENTS];

	get_filter (axis,(P_INT)coeff);
	coeff[DF_P]=1; /* 1200 */
	coeff[DF_I]=0; /* 5 */
	coeff[DF_D]=0; /* 3000 */
	coeff[DF_ACCEL_FF]=0;
	coeff[DF_VEL_FF]=0;
	coeff[DF_SHIFT]=0;/* -8 */
	coeff[DF_DAC_LIMIT]=3276;
	coeff[DF_OFFSET]=0;
	set_filter (axis,(P_INT)coeff);
	set_integration (axis,IM_ALWAYS);
	return 0;
}
void save_firmware()
{
  upload_firmware_file ("vx_mei_firmware.bin");
}
void restore_firmware()
{
  download_firmware_file ("vx_mei_firmware.bin");
}

int sdss_error(int error_code)
{   
	char buffer[MAX_ERROR_LEN];

	switch (error_code)
	{
		case DSP_OK:
			/* No error, so we can ignore it. */
			return FALSE;
			break ;

		default:
			error_msg(error_code, buffer) ;
			printf("ERROR: %s (%d).\n", buffer, error_code);
			return TRUE;
			break;
	}
}

#define CLOCK_INT
#ifdef CLOCK_INT
float sdss_get_time()
{
  	  unsigned long micro_sec;

          micro_sec = (unsigned long)(1.0312733648*timer_read (1));
	  if (micro_sec>1000000) micro_sec=999999;
/*          micro_sec = timer_read (1);*/
          return (float)(SDSStime+((micro_sec%1000000)/1000000.));
}
float get_time()
{
  	  unsigned long micro_sec;

          micro_sec = (unsigned long)(1.0312733648*timer_read (1));
	  if (micro_sec>1000000) micro_sec=999999;
/*          micro_sec = timer_read (1);*/
	  printf ("\r\nSDSS time=%f",
		(float)(SDSStime+((micro_sec%1000000)/1000000.)));
          return (float)(SDSStime+((micro_sec%1000000)/1000000.));
}
#else
float sdss_get_time()
{
  	  struct timespec tp;
  	  unsigned long micro_sec;

  	  clock_gettime(CLOCK_REALTIME,&tp);
          micro_sec = timer_read (1);
          return ((float)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
float get_time()
{
  	  struct timespec tp;
  	  unsigned long micro_sec;

  	  clock_gettime(CLOCK_REALTIME,&tp);
          micro_sec = timer_read (1);
	  printf ("\r\nsec=%d, day_sec=%d, micro_sec=%d, time=%f",
		tp.tv_sec,tp.tv_sec%ONE_DAY,micro_sec,
		(float)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
          return ((float)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
#endif
float sdss_delta_time(float t2, float t1)
{
  if ((t2>=0.0)&&(t2<400.)&&(t1>86000.)&&(t1<86400.))
    return ((86400.-t1)+t2);
  if ((t1>=0.0)&&(t1<400.)&&(t2>86000.)&&(t2<86400.))
    return ((t2-86400.)-t1);
  return (t2-t1);
}
int test_dt(int t2,int t1)
{
  printf ("\r\ndt=%f",sdss_delta_time((float)t2,(float)t1));
  return (int)sdss_delta_time((float)t2,(float)t1);
}
int latch_done=FALSE;
void test_latch (int ticks)
{
  int i;
  double p;

/*
  init_io(2,IO_OUTPUT);
  set_io(2,0xFF);
  arm_latch(TRUE);
  taskDelay(30);
*/
  while (!latch_done)
  {
    latch();
    i=0x10000;
    while ((!latch_status())&&(i>0))i--;
      printf ("\r\nstart 0x10000, i=%d read latch_status %d times",
		i,0x10000-i);
/*    while (!latch_status());*/
    get_latched_position(0,&p);
    printf("\r\n0: latched position=%12.0lf",p);
    get_position(0,&p);
    printf(" position=%12.0lf\n\r",p);

    get_latched_position(1,&p);
    printf("\r\n1: latched position=%12.0lf",p);
    get_position(1,&p);
    printf(" position=%12.0lf\n\r",p);

    get_latched_position(2,&p);
    printf("\r\n2: latched position=%12.0lf",p);
    get_position(2,&p);
    printf(" position=%12.0lf\n\r",p);

    get_latched_position(3,&p);
    printf("\r\n3: latched position=%12.0lf",p);
    get_position(3,&p);
    printf(" position=%12.0lf\n\r",p);

    get_latched_position(4,&p);
    printf("\r\n4: latched position=%12.0lf",p);
    get_position(4,&p);
    printf(" position=%12.0lf\n\r",p);

    get_latched_position(5,&p);
    printf("\r\n5: latched position=%12.0lf",p);
    get_position(5,&p);
    printf(" position=%12.0lf\n\r",p);
    arm_latch(TRUE);

    taskDelay(ticks);
  }
}
void latch_it ()
{
  int i;
  double p;
  unsigned long elapsed_time;

    latch();
    timer_start(2);
    i=0x10000;
    while ((!latch_status())&&(i>0))i--;
    elapsed_time = timer_read(2);
    printf ("  %d usecs",elapsed_time);
    printf ("\r\nstart 0x10000, i=%d read latch_status %d times",
		i,0x10000-i);
    i=0x10;
    while ((!latch_status())&&((i--) > 0))
              printf ("%d:latch_status not TRUE\r\n",i);
/*    while (!latch_status());*/
    get_latched_position(0,&p);
    printf("\r\nlatched position=%12.0lf",p);
    get_position(0,&p);
    printf(" position=%12.0lf\n\r",p);
    arm_latch(TRUE);
}
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
unsigned long loadframecnt=0;
unsigned long loadframemissed=0;
void tm_load_frame()
{
  if (semLOADFRAME==NULL) semLOADFRAME = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
  for (;;)
  {
    if (semTake(semLOADFRAME,WAIT_FOREVER)!=ERROR)
    {
      if (semTake (semMEI,1)!=ERROR)
      {
	 loadframecnt++;
	 semGive (semMEI);
      }
      else
	 loadframemissed++;
    }
  }

}
void load_frames_trigger(int axisnum)
{
  semGive(semLOADFRAME);
}
WDOG_ID LFid;
void lfStart()
{
  taskSpawn ("tmLoadFrame",0,VX_FP_TASK,10000,(FUNCPTR)tm_load_frame,0,0,0,0,0,0,0,0,0,0);
/*  LFid=wdCreate();
  wdStart (LFid,3,(FUNCPTR)load_frames_trigger,3);*/
}
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
