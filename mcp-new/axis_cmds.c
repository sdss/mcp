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
#include "semLib.h"
#include "sigLib.h"
#include "tickLib.h"
#include "inetLib.h"
#include "taskLib.h"
#include "in.h"
#include "timers.h"
#include "time.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "dio316ld.h"
#include "did48ld.h"
#include "mv162IndPackInit.h"
#include "gendefs.h"
#include "logLib.h"
#include "ioLib.h"
#include "data_collection.h"
#include "io.h"
#include "math.h"
#include "tm.h"
#include "axis.h"

#define	DSP_IO_BASE		(0x300)			/* in A16/D16 space. */
/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/
/* Prototypes */
void axis_DIO316_shutdown(int type);
void DIO316_interrupt(int type);
void DIO316_initialize(unsigned char *addr, unsigned short vecnum);
void axis_DID48_shutdown(int type);
void DID48_interrupt(int type);
void DID48_initialize(unsigned char *addr, unsigned short vecnum);
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
int sdss_move_time (int axis, int vel, int accel, int time);
int sdss_move_pos (int axis, int vel, int accel, int pos);
int set_coeffs(int axis, int index, int val);
int init_coeffs(int axis);
int print_coeffs(int axis);
int sdss_remap(int axis);
int sdss_error(int error_code);
float sdss_get_time();
float get_time();

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

SEM_ID semMEI=NULL;
SEM_ID semSLC=NULL;
SEM_ID semLATCH=NULL;
int tm_DIO316,tm_DID48;
int axis_select=0;		/* 0=AZ,1=ALT,2=ROT */
int MEI_interrupt=FALSE;
int DIO316_Init=FALSE;
int DID48_Init=FALSE;
int sdss_was_init=FALSE;
struct FRAME_QUEUE axis_queue[]={
		{0,NULLPTR,NULLPTR},
		{0,NULLPTR,NULLPTR},
		{0,NULLPTR,NULLPTR}
};
double max_velocity[]={1.0,1.0,1.5,0,0,0};
double max_acceleration[]={.4,.4,.8,0,0,0};
int ms_azimuth_region=0;
int ms_altitude_region=0;
int ms_instrument_region=0;
int max_ms_azimuth;
int max_ms_altitude;
int max_ms_instrument;
struct MAG_SENSOR ms_azimuth[]={
	{0,0.0,0.0},
	{1,15.0,15.0},
	{2,30.0,30.0},
	{3,45.0,45.0},
	{4,60.0,60.0},
	{5,75.0,75.0},
	{6,90.0,90.0},
	{7,105.0,105.0},
	{8,120.0,120.0},
	{9,135.0,135.0},
	{10,150.0,150.0},
	{11,165.0,165.0},
	{12,180.0,180.0},
	{13,195.0,195.0},
	{14,210.0,210.0},
	{15,225.0,225.0},
	{16,240.0,240.0},
	{17,255.0,255.0},
	{18,270.0,270.0},
	{19,285.0,285.0},
	{20,300.0,300.0},
	{21,315.0,315.0},
	{22,330.0,330.0},
	{23,345.0,345.0},
	{24,360.0,0.0},

	{1+24,15.0,15.0},
	{2+24,30.0,30.0},
	{3+24,45.0,45.0},
	{4+24,60.0,60.0},
	{5+24,75.0,75.0},
	{6+24,90.0,90.0},
	{7+24,105.0,105.0},
	{8+24,120.0,120.0},
	{9+24,135.0,135.0},
	{10+24,150.0,150.0},
	{11+24,165.0,165.0},
	{12+24,180.0,180.0},
	{13+24,195.0,195.0},
	{14+24,210.0,210.0},
	{15+24,225.0,225.0},
	{16+24,240.0,240.0},
	{17+24,255.0,255.0},
	{18+24,270.0,270.0},
	{19+24,285.0,285.0},
	{20+24,300.0,300.0},
	{21+24,315.0,315.0},
	{22+24,330.0,330.0},
	{23+24,345.0,345.0},
	{24+24,360.0,0.0}
};
struct MAG_SENSOR ms_altitude[]={
	{0,0.0,0.0},
	{1,15.0,15.0},
	{2,30.0,30.0},
	{3,45.0,45.0},
	{4,60.0,60.0},
	{5,75.0,75.0},
	{6,90.0,90.0},
	{7,105.0,105.0},
	{8,120.0,120.0},
	{9,135.0,135.0},
	{10,150.0,150.0},
	{11,165.0,165.0},
	{12,180.0,180.0}
};
struct MAG_SENSOR ms_instrument[]={
	{0,0.0,0.0},
	{1,15.0,15.0},
	{2,30.0,30.0},
	{3,45.0,45.0},
	{4,60.0,60.0},
	{5,75.0,75.0},
	{6,90.0,90.0},
	{7,105.0,105.0},
	{8,120.0,120.0},
	{9,135.0,135.0},
	{10,150.0,150.0},
	{11,165.0,165.0},
	{12,180.0,180.0},
	{13,195.0,195.0},
	{14,210.0,210.0},
	{15,225.0,225.0},
	{16,240.0,240.0},
	{17,255.0,255.0},
	{18,270.0,270.0},
	{19,285.0,285.0},
	{20,300.0,300.0},
	{21,315.0,315.0},
	{22,330.0,330.0},
	{23,345.0,345.0},
	{24,360.0,0.0}
};
int CALC_verbose=FALSE;
int CALCFINAL_verbose=FALSE;
int FRAME_verbose=FALSE;
#define FRMHZ	20
#define FLTFRMHZ	20.
#define MAX_CALC 50
double tim[3][MAX_CALC],p[3][MAX_CALC],v[3][MAX_CALC],a[3][MAX_CALC],ji[3][MAX_CALC];
float time_off[3]={0.0,0.0,0.0};
int frame_break[3]={FALSE,FALSE,FALSE};

char *correct_cmd(char *cmd)
{
  printf (" CORRECT command fired\r\n");
  return 0;
}

double sec_per_tick[3]={AZ_TICK,ALT_TICK,ROT_TICK};
double ticks_per_degree[]={AZ_TICKS_DEG,
				ALT_TICKS_DEG,
				ROT_TICKS_DEG};
char *drift_cmd(char *cmd)
{
  static char *time_ans={"360.00000 0.500000 5040.00000               "};	/* */
  struct timespec tp;
  unsigned long micro_sec;
  long ap,vel;
  double position,velocity;
  double arcdeg, veldeg;
  static struct tm *t;
  extern struct TM_M68K *tmaxis[];

/*  printf (" DRIFT command fired\r\n");*/
  if (semTake (semMEI,60)!=ERROR)
  {
/*    ap=(*tmaxis[axis_select]).actual_position;*/
/*    vel=(*tmaxis[axis_select]).velocity;*/
    get_position(axis_select<<1,&position);
    get_velocity(axis_select<<1,&velocity);
    semGive (semMEI);
    ap=(long)position;
    vel=(long)velocity;
    veldeg=(sec_per_tick[axis_select]*vel)/3600.;
    arcdeg=(sec_per_tick[axis_select]*ap)/3600.;
  }
  sprintf (time_ans,"%lf %lf %f",arcdeg,veldeg,sdss_get_time());
  return time_ans;
}

char *gp1_cmd(char *cmd)
{
  printf ("NOT IMPLEMENTED:  GP1 command fired\r\n");
  return 0;
}

char *gp2_cmd(char *cmd)
{
  printf ("NOT IMPLEMENTED:  GP2 command fired\r\n");
  return 0;
}

char *id_ans={"0 None Specified MMM DD 19YY\r\nDSP Firmware=Vxxx.xx Rxx Sx, Option=xxxx Axes=x"};
char *id_cmd(char *cmd)
{
  static char *axis_name[]={"None Specified","Azimuth","Altitude","Rotator"};

/*  printf (" ID command fired\r\n");*/
  sprintf (id_ans,"%d %s %s\r\nDSP Firmware=V%f R%d S%d, Option=%d, Axes=%d",
		axis_select,axis_name[axis_select],__DATE__,
		dsp_version()/1600.,dsp_version()&0xF,(dsp_option()>>12)&0x7,
		dsp_option()&0xFFF,dsp_axes());
  return id_ans;
}

char *init_cmd(char *cmd)
{
  int e ;
  char buffer[MAX_ERROR_LEN] ;

/*  printf (" INIT command fired\r\n");*/
  tm_controller_idle (axis_select<<1);
  tm_controller_idle (axis_select<<1);
  tm_controller_idle (axis_select<<1);
  taskDelay (30);
  tm_controller_run (axis_select<<1);
  tm_controller_run (axis_select<<1);
  tm_controller_run (axis_select<<1);
  v_move(axis_select<<1,(double)0,(double)5000);
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
  return 0;
}

char *maxacc_cmd(char *cmd)
{
  printf (" MAXACC command fired\r\n");
  return 0;
}

char *maxvel_cmd(char *cmd)
{
  printf (" MAXVEL command fired\r\n");
  return 0;
}

char *mc_dump_cmd(char *cmd)
{
  printf (" MC.DUMP command fired\r\n");
  return 0;
}

char *mc_maxacc_cmd(char *cmd)
{
  printf (" MC.MAXACC command fired\r\n");
  return 0;
}

char *mc_maxpos_cmd(char *cmd)
{
  printf (" MC.MAXPOS command fired\r\n");
  return 0;
}

char *mc_maxvel_cmd(char *cmd)
{
  printf (" MC.MAXVEL command fired\r\n");
  return 0;
}

char *mc_minpos_cmd(char *cmd)
{
  printf (" MC.MINPOS command fired\r\n");
  return 0;
}

char *move_cmd(char *cmd)
{

  double position,velocity,pos;
  struct FRAME *frame,*nxtque,*lstque,*nframe;
  struct FRAME_QUEUE *queue;
  int cnt;

/*  printf (" MOVE command fired\r\n");*/
  queue = &axis_queue[axis_select];
  frame = (struct FRAME *)malloc (sizeof(struct FRAME));
  cnt=sscanf (cmd,"%12lf %12lf %12lf",&position,&velocity,&frame->end_time);
  switch (cnt)
  {
    case -1:
    case 0:
        tm_get_pos(axis_select<<1,&position);
        tm_get_vel(axis_select<<1,&velocity);
/*
        position = position/ticks_per_degree[axis_select];
	velocity /= ticks_per_degree[axis_select];
	frame->end_time = sdss_get_time()+max_acceleration[axis_select]*
	fabs(velocity)+1.0;
*/
/*	frame->end_time = sdss_get_time()+2.5;*/
/*	frame->end_time = ((queue->active)->nxt)->end_time+4.;*/
/*	printf("\r\n %lf %lf %lf",position,velocity,frame->end_time);*/
        tm_start_move (axis_select<<1,
		velocity,
		(ticks_per_degree[axis_select]*max_acceleration[axis_select])/2,
		position);
        printf ("\r\n Repositioning MOVE cmd to position=%lf",position);
        queue->active=NULL;
        frame_break[axis_select]=TRUE;
	free (frame);
        return 0;
/*        break;*/
    case 1:
        tm_get_pos(axis_select<<1,&pos);
	velocity = (double).10;
	frame->end_time = sdss_get_time()+
		(abs(pos/ticks_per_degree[axis_select]-position)/velocity);
        break;
    case 2:
        tm_get_pos(axis_select<<1,&pos);
	frame->end_time = sdss_get_time()+(abs(pos-position)/velocity);
        break;
    case 3:
        if (frame->end_time<sdss_get_time())
	{
	  free (frame);
	  return 0;
	}
	break;
  }
  frame->position=position;
  if (fabs(velocity)>max_velocity[axis_select]) 
      velocity=max_velocity[axis_select];
  frame->velocity=(double)velocity;
  frame->nxt = NULL;

/* queues are initialized with one dummy entry at sdss_init time */
  nxtque = queue->end;
  nxtque->nxt = frame;
  if (queue->active==NULL)/* end of queue, and becomes active frame */
    queue->active=frame;
  if (cnt==-1)/* this frame becomes active frame */
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
/* clean up queue for old entries */
  while ((queue->cnt>MAX_FRAME_CNT)&&(queue->top != queue->active))
  {
    nxtque = queue->top;
    queue->top = nxtque->nxt;
    free(nxtque);
    queue->cnt--;
  }
  return 0;
}
int calc_frames (int axis, struct FRAME *iframe, int start)
{
  double dx,dv,dt,vdot;
  double ai,j,t,lai,lj,lt,ldt;
  struct FRAME *fframe;
  struct FRAME *lframe;
  int i;
  
  while (iframe->nxt==NULL) taskDelay (10);
  fframe=iframe->nxt;
  dx=fframe->position-iframe->position;
  dv=fframe->velocity-iframe->velocity;
  dt=fframe->end_time-iframe->end_time;
  vdot=dx/dt;
  ai=(2/dt)*((3*vdot)-(2*iframe->velocity)-fframe->velocity);
  j=(6/(dt*dt))*(iframe->velocity+fframe->velocity-(2*vdot));
  if (CALC_verbose)
  {
    printf("\r\n dx=%12.8lf, dv=%12.8lf, dt=%12.8lf, vdot=%lf",dx,dv,dt,vdot);
    printf("\r\n ai=%12.8lf, j=%12.8lf, t=%f, start=%d, time_off=%f",ai,j,t,start,time_off[axis]);
  }
  t=(start+1)/FLTFRMHZ+time_off[axis];	/* for end condition */
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
  if ( ((int)(i+start)==(int)((dt-time_off[axis])*FRMHZ)) &&
	(t!=(dt-time_off[axis])) )
  {
    ldt=(((dt-time_off[axis])*FLTFRMHZ)-(int)((dt-time_off[axis])*FLTFRMHZ))/FLTFRMHZ;
/*    lt = dt-t;*/
/*    t = (1/FLTFRMHZ)-lt;*/
    t = (1/FLTFRMHZ)-ldt;
    lt = dt;
    time_off[axis]=t;
/*    tim[axis][i]=((dt*FRMHZ)-(int)(dt*FRMHZ))/FLTFRMHZ;*/
    tim[axis][i]=1/FLTFRMHZ;
    lframe=fframe;
    if (lframe->nxt==NULL) return i;
    fframe=lframe->nxt;
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
    if (i>550) printf ("\r\n calc_frames approaching problems %d",i+1);
    return (i+1);
  }
  else 
    return i;
}
void start_frame(int axis,double time)
{
  int e;
  FRAME frame;
  
  time_off[axis]=0.0;
  printf ("\r\nSTART axis=%d: time=%lf",axis<<1,time);
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
     set_gate(axis<<1);
     e=frame_m(&frame,"0l t un d",axis<<1,
	time,
	FTRG_TIME,NEW_FRAME);    
     reset_gate(axis<<1);
     semGive (semMEI);
  }
}
int get_frame_cnt(int axis, struct FRAME *iframe)
{
  struct FRAME *fframe;
  double dt;
  int cnt;

  fframe=iframe->nxt;
  dt=fframe->end_time-iframe->end_time;
  cnt=(int)((dt-time_off[axis]+(1/(FLTFRMHZ+1.)))*FLTFRMHZ);

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
void load_frames(int axis, int cnt, double sf)
{
  int e;
  int i;
  FRAME frame;
  
  if (FRAME_verbose)
    printf("\r\n Load %d Frames, sf=%lf",cnt,sf);
  for (i=0;i<cnt;i++)
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
      if (axis==2)
        if (coeffs_state_deg (axis<<1,v[axis][i]));

      e=frame_m(&frame,"0l xvajt un d",axis<<1,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,(double)ji[axis][i]*sf,
	tim[axis][i],
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,NEW_FRAME);
/*
      e=frame_m(&frame,"0l xvat un d",axis<<1,
	(double)p[axis][i]*sf,(double)v[axis][i]*sf,
	(double)a[axis][i]*sf,
	tim[axis][i],
	FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FTRG_TIME,NEW_FRAME);    
*/
      semGive (semMEI);
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
  int e;
  int i;
  FRAME frame;
   
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

void end_frame(int axis,int index,double sf)
{
  int e;
  FRAME frame;
  
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
      e=frame_m(&frame,"0l xvajt un d",axis<<1,
	(double)p[axis][index]*sf,(double)0.0,(double)0.0,
	(double)0.0,
	(double).00333333,
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
  
  if (semTake (semMEI,WAIT_FOREVER)!=ERROR)
  {
    cnt=frames_to_execute(axis<<1);
    semGive (semMEI);
  }
  return cnt;    
}
#define LOAD_MAX        50
void tm_TCC_test(int axis, struct FRAME *iframe, struct FRAME *fframe)
{
  int cnt;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;
  extern struct TM_M68K *tmaxis[];

  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
        ticks_per_degree[axis]);
  CALC_verbose=TRUE;
    frame=iframe;
      while ((frame!=fframe)&&(frame->nxt!=NULL))
      {
	frame_break[axis]=FALSE;
        frame_cnt=get_frame_cnt(axis,frame);
        printf ("\r\n frames_cnt=%d",frame_cnt);
        frame_idx=0;
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
  int cnt, lcnt;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;
  extern struct TM_M68K *tmaxis[];
  double position;
  long pos;
  
  tm_controller_run (axis<<1);
  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
	ticks_per_degree[axis]);
  FOREVER
  {
    while (axis_queue[axis].active==NULL)taskDelay (10);
    frame=axis_queue[axis].active;
/* reposition if neccessary */
    if (semTake (semMEI,60)!=ERROR)
    {
      get_position(axis<<1,&position);
      semGive (semMEI);
      pos=(long)position;
    }
    else
      pos=(*tmaxis[axis]).actual_position;
    if (abs((frame->position*ticks_per_degree[axis])-pos)>
		(.001*ticks_per_degree[axis]))
    {
      tm_start_move (axis<<1,
		1*(double)ticks_per_degree[axis],
		.5*(double)ticks_per_degree[axis],
		frame->position*(double)ticks_per_degree[axis]);
      printf ("\r\n Repositioning");
    }
    while (abs((frame->position*ticks_per_degree[axis])-pos)>
		(.0005*ticks_per_degree[axis]))
    {
      if (semTake (semMEI,60)!=ERROR)
      {
        get_position(axis<<1,&position);
        semGive (semMEI);
        pos=(long)position;
      }
      else
        pos=(*tmaxis[axis]).actual_position;
      taskDelay (1);
    }
/* check for time */
    while ((frame!=NULL)&&((frame->end_time-sdss_get_time())<.0))
    {
      frame = frame->nxt;
      axis_queue[axis].active=frame;
      printf ("\r\n Frame deleted due to time");
    }
    if (frame!=NULL)
    {
      while ((frame->end_time-sdss_get_time())>4.0)taskDelay (2*60);
      start_frame (axis,frame->end_time-sdss_get_time());
      while ((frame->nxt==NULL)&&((frame->end_time-sdss_get_time())>0.02))
        taskDelay (1);
      while (frame->nxt!=NULL)
      {
        while ((lcnt=tm_frames_to_execute(axis))>100)
        {
/*         printf ("\r\n frames left=%d",lcnt);*/
          taskDelay(1*30);
        }
        frame_cnt=get_frame_cnt(axis,frame);
/*      printf ("\r\n frames_cnt=%d",frame_cnt);*/
        frame_idx=0;
	frame_break[axis]=FALSE;
        for (i=0;i<((frame_cnt-1)/(LOAD_MAX-1))+1;i++)
        {
	  if (frame_break[axis]) break;
          cnt=calc_frames(axis,frame,frame_idx);
          frame_idx += cnt;
/*          printf ("\r\n cnt=%d, i=%d",cnt,i);*/
	  if (cnt>0)
            load_frames(axis,cnt,(double)ticks_per_degree[axis]);
          while ((lcnt=tm_frames_to_execute(axis))>100) taskDelay (1*30);
          if (lcnt<10) printf ("\r\n frames left=%d, i=%d",lcnt,i);
        }
	frame_break[axis]=FALSE;
        frame = frame->nxt;
        axis_queue[axis].active=frame;    
        while ((frame->nxt==NULL)&&((frame->end_time-sdss_get_time())>.02))
          taskDelay (1);
        while ((frame->nxt==NULL)&&((lcnt=tm_frames_to_execute(axis))>1));
      }
      printf ("\r\n Ran out: frames left=%d",lcnt);
      axis_queue[axis].active=NULL;    
      end_frame(axis,cnt-1,(double)ticks_per_degree[axis]);
    }
    else
      printf ("\r\nRestart no frames to process");
  }
}
void tm_test(int axis, int sf)
{
  int cnt, lcnt;
  struct FRAME *frame;
  int i;
  int frame_cnt, frame_idx;
  extern struct TM_M68K *tmaxis[];
  
  tm_controller_run (axis<<1);
  printf ("\r\n Axis=%d;  Ticks per degree=%lf",axis,
	(float)sf);
  FOREVER
  {
    while (axis_queue[axis].active==NULL)taskDelay (10);
    frame=axis_queue[axis].active;
/* reposition if neccessary */
    if (abs((frame->position*sf)-(*tmaxis[axis]).actual_position)>(1*sf))
    {
      tm_controller_run (axis<<1);
      tm_start_move (axis<<1,2*sf,2*sf,frame->position*sf);
      printf ("\r\n Repositioning");
    }
    while (abs((frame->position*sf)-(*tmaxis[axis]).actual_position)>(.5*sf))
      taskDelay (1);
/* check for time */
    while ((frame!=NULL)&&((frame->end_time-sdss_get_time())<.0))
    {
      frame = frame->nxt;
      axis_queue[axis].active=frame;
      printf ("\r\n Frames deleted due to time");
    }
    if (frame!=NULL)
    {
      while ((frame->end_time-sdss_get_time())>4.0)taskDelay (2*60);
      start_frame (axis,frame->end_time-sdss_get_time());
      while ((frame->nxt==NULL)&&((frame->end_time-sdss_get_time())>0.02))
        taskDelay (1);
      while (frame->nxt!=NULL)
      {
        while ((lcnt=tm_frames_to_execute(axis))>100)
        {
/*         printf ("\r\n frames left=%d",lcnt);*/
          taskDelay(1*30);
        }
        frame_cnt=get_frame_cnt(axis,frame);
/*      printf ("\r\n frames_cnt=%d",frame_cnt);*/
        frame_idx=0;
        for (i=0;i<((frame_cnt-1)/(LOAD_MAX-1))+1;i++)
        {
          cnt=calc_frames(axis,frame,frame_idx);
          frame_idx += cnt;
/*          printf ("\r\n cnt=%d, i=%d",cnt,i);*/
	  if (cnt>0)
            load_frames(axis,cnt,(double)sf);
          while ((lcnt=tm_frames_to_execute(axis))>100) taskDelay (1*30);
          printf ("\r\n frames left=%d, i=%d",lcnt,i);
        }
        frame = frame->nxt;
        axis_queue[axis].active=frame;    
        while ((frame->nxt==NULL)&&((frame->end_time-sdss_get_time())>.02))
          taskDelay (1);
        while ((frame->nxt==NULL)&&((lcnt=tm_frames_to_execute(axis))>1));
      }
      printf ("\r\n Ran out: frames left=%d",lcnt);
      axis_queue[axis].active=NULL;    
      end_frame(axis,cnt-1,(double)sf);
    }
    else
      printf ("\r\nRestart no frames to process");
  }
}
start_tm_TCC()
{
  taskSpawn("tmAz",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC,0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlt",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC,1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRot",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC,2,0,0,0,0,0,0,0,0,0);
}
start_tm_TCC_test()
{
  taskSpawn("tmAztest",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC_test,0,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmAlttest",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC_test,1,0,0,0,0,0,0,0,0,0);
  taskSpawn("tmRottest",62,VX_FP_TASK,10000,(FUNCPTR)tm_TCC_test,2,0,0,0,0,0,0,0,0,0);
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
  double position,velocity,frame_time;
  struct FRAME *nxtque;
  struct FRAME_QUEUE *queue;
  int cnt;

/*  printf (" +MOVE command fired\r\n");*/
  queue = &axis_queue[axis_select];
  cnt=sscanf (cmd,"%lf %lf %lf",&position,&velocity,&frame_time);
  switch (cnt)
  {
    case -1:
    case 0:
        break;		/* NULL offset - does nothing */
    case 1:
	if (queue->active!=NULL)
	{
	  nxtque = queue->active;
	  while (nxtque!=NULL)
	  {
	    nxtque->position+=position;
	    nxtque=nxtque->nxt;
	  }
	}
        break;
    case 2:
        if (queue->active!=NULL)
        {
          nxtque = queue->active;
          while (nxtque!=NULL)
          {
            nxtque->position+=position;
	    nxtque->velocity+=velocity;
            nxtque=nxtque->nxt;
          }
        }
        break;
    case 3:
        if (queue->active!=NULL)
        {
          nxtque = queue->active;
          while (nxtque!=NULL)
          {
            nxtque->position+=position;
            nxtque->velocity+=velocity;
            nxtque->end_time+=frame_time;
            nxtque=nxtque->nxt;
          }
        }
        break;
  }
  return 0;
}

char *mr_dump_cmd(char *cmd)
{
  printf (" MR.DUMP command fired\r\n");
  return 0;
}

char *ms_dump_cmd(char *cmd)
{
  printf (" MS.DUMP command fired\r\n");
  return 0;
}

char *ms_map_dump_cmd(char *cmd)
{
  printf (" MS.MAP.DUMP command fired\r\n");
  return 0;
}

char *ms_map_load_cmd(char *cmd)
{
  printf (" MS.MAP.LOAD?????? command fired\r\n");
  return 0;
}

char *ms_off_cmd(char *cmd)
{
  printf (" MS.OFF command fired\r\n");
  return 0;
}

char *ms_on_cmd(char *cmd)
{
  printf (" MS.ON command fired\r\n");
  return 0;
}

char *ms_pos_dump_cmd(char *cmd)
{
  printf (" MS.POS.DUMP command fired\r\n");
  return 0;
}

char *remap_cmd(char *cmd)
{
  printf (" REMAP command fired\r\n");
  return 0;
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
  return 0;
}

char *set_position_cmd(char *cmd)
{
  printf (" SET.POSITION command fired\r\n");
  return 0;
}

struct tm t;
unsigned long SDSStime=0;
char *set_time_cmd(char *cmd)
{
  float t1,t2,t3;
  int cnt;
  struct timespec tp;

/*  printf (" SET.TIME command fired\r\n");*/
  cnt=sscanf (cmd,"%d %d %d %f %f %f",&t.tm_mon,&t.tm_mday,&t.tm_year,&t1,&t2,&t3);
/*  printf ("\r\ncnt=%d",cnt);*/
  if (cnt>4)
  {
    t.tm_hour=(int)t1;
    t.tm_min=(int)t2;
    t.tm_sec=(int)t3;
    if ((t3-(int)t3)>.984)
    {
      taskDelay(2);
      t.tm_sec++;
    }
  }
  else
  {
    t.tm_hour = (int)(t1/3600);
    t.tm_min = (int)((t1-(t.tm_hour*3600))/60);
    t.tm_sec = (int)(t1-(t.tm_hour*3600)-(t.tm_min*60));
  }
  t.tm_year -= 1900;
  t.tm_mon -= 1;
/*  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);*/
  tp.tv_sec=mktime(&t);
  tp.tv_nsec=0;
  SDSStime=tp.tv_sec%ONE_DAY;
  timer_start (1);
/*  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);*/
/*  get_time();*/
  clock_settime(CLOCK_REALTIME,&tp);
/*  get_time();*/
/*  if (!DIO316_Init) 
    DIO316_initialize((unsigned char *)0xFFF58000,0xB0);
  if (!DID48_Init) 
    DID48_initialize((unsigned char *)0xFFF58000,0xB8);*/
  return 0;
}

char *stats_cmd(char *cmd)
{
  printf (" STATS command fired\r\n");
  return 0;
}

/* returns: "%position %velocity %time %status_word %index_position" */
static long status=0x40000000;
char *status_cmd(char *cmd)
{
  static char *status_ans=
  {"1073741824                                                                   "};	/* 0x40000000 */
  extern struct TM_M68K *tmaxis[];
  extern struct FIDUCIARY fiducial[3];

/*  printf (" STATUS command fired\r\n"); */
  if (semTake (semMEI,60)!=ERROR)
  {
    sprintf (status_ans,"%lf %lf %f %ld %lf",
	(*tmaxis[axis_select]).actual_position/ticks_per_degree[axis_select],
	(*tmaxis[axis_select]).velocity/ticks_per_degree[axis_select],
	sdss_get_time(),
	status,
	fiducial[axis_select].mark/ticks_per_degree[axis_select]);
    semGive (semMEI);
  }
  return status_ans;
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
  return 0;
}

char *time_cmd(char *cmd)
{
  static char *time_ans={"01 31 1996 86400.000"};	/* */
  static struct tm *t;
  struct timespec tp;
  unsigned long micro_sec;

/*  printf (" TIME? command fired\r\n");*/
  clock_gettime(CLOCK_REALTIME,&tp);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
  t = gmtime(&tp.tv_sec);
  printf ("t=%p, mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t,t->tm_mon,t->tm_mday,t->tm_year,t->tm_hour,t->tm_min,t->tm_sec);
  micro_sec = timer_read (1);

  sprintf (time_ans,"%d %d %d %f",t->tm_mon+1,t->tm_mday,t->tm_year+1900,
	(t->tm_hour*3600.)+(t->tm_min*60.)+
	t->tm_sec+((micro_sec%1000000)/1000000.));
  return time_ans;
}
int print_axis_queue(int axis)
{
  float position,velocity;
  struct FRAME *frame,*nxtque,*lstque;
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
unsigned long NIST_sec;
unsigned long int_count=0;
float latchpos4,latchpos5;
int lpos4,lpos5;
unsigned char int_bit=0;
void DIO316_interrupt(int type)
{
	  int i;
/*	  static unsigned char int_bit=0;
*/	  int direction, region;                        
	  double position;
 	  float pos,lpos;

/*	  latch();*/

	  int_count++;
          DIO316ReadISR (tm_DIO316,&int_bit);
          if (int_bit&NIST_INT)
          {
            SDSStime++;
            DIO316_Write_Port (tm_DIO316,3,0);
            NIST_sec=timer_read (1);
            timer_start (1);
/*            logMsg ("NIST_INTERRUPT,%d %d, int_bit=%2x\r\n",
		type,NIST_sec,(short)int_bit,0,0,0);*/
          }
	  else
	    semGive (semLATCH);
	  
/*
          if (int_bit&AZIMUTH_INT)
          {
            DIO316_Write_Port (tm_DIO316,3,0);
	    i=0x1000;
	    while ((!latch_status())&&(i>0))i--;
            logMsg ("%d:latch_status TRUE after %d tries\r\n",0x1000-i,0,0,0,0,0);
	    get_latched_position(0,&position);
	    lpos=(float)position;
	    get_position(0,&position);
	    pos=(float)position;
	    if (negative_direction(0))
            {
	      ms_azimuth_region--;
	      if (ms_azimuth_region<0)ms_azimuth_region=0;
	      region = ms_azimuth_region;
	      direction=ms_azimuth[region].direction=NEGATIVE_DIRECTION;
            }
	    else
            {
	      region = ms_azimuth_region;
	      direction=ms_azimuth[region].direction=POSITIVE_DIRECTION;
	      ms_azimuth_region++;
	      if (ms_azimuth_region>
                max_ms_azimuth) ms_azimuth_region=max_ms_azimuth;
            }
	    if (ms_azimuth[region].position[direction]==0.0)
	      ms_azimuth[region].position[direction]=position;
	    else
	      ms_azimuth[region].error[direction]=
		ms_azimuth[region].position[direction]-position;
            ms_azimuth[region].time[direction]=get_time();

            logMsg ("\r\nAZIMUTH_INTERRUPT,%d %d, int_bit=%hx, position=%f latched=%f\r\n",
		type,i,int_bit,pos,lpos,0);
          }
          if (int_bit&ALTITUDE_INT)
          {
            DIO316_Write_Port (tm_DIO316,3,0);
	    i=0x1000;
	    while ((!latch_status())&&(i>0))i--;
	    get_latched_position(2,&position);
	    lpos=(float)position;
	    get_position(2,&position);
	    pos=(float)position;
	    if (negative_direction(2))
            {
	      ms_altitude_region--;
	      if (ms_altitude_region<0)ms_altitude_region=0;
	      region = ms_altitude_region;
	      direction=ms_altitude[region].direction=NEGATIVE_DIRECTION;
            }
	    else
            {
	      region = ms_altitude_region;
	      direction=ms_altitude[region].direction=POSITIVE_DIRECTION;
	      ms_altitude_region++;
	      if (ms_altitude_region>
                max_ms_altitude) ms_altitude_region=max_ms_altitude;
            }
	    if (ms_altitude[region].position[direction]==0.0)
	      ms_altitude[region].position[direction]=position;
	    else
	      ms_altitude[region].error[direction]=
		ms_altitude[region].position[direction]-position;
            ms_altitude[region].time[direction]=get_time();

            logMsg ("ALTITUDE_INTERRUPT,%d %d, int_bit=%hx, position=%f, latched=%f\r\n",
		type,i,int_bit,pos,lpos,0);
          }
          if (int_bit&INSTRUMENT_INT)
          {
            DIO316_Write_Port (tm_DIO316,3,0);
	    i=0x10000;
	    while ((!latch_status())&&(i>0))i--;
	    if (latchidx<MAX_LATCHED)
	    {
	      get_latched_position(4,&latchpos[latchidx].pos1);
	      get_latched_position(5,&latchpos[latchidx].pos2);
              logMsg("\r\nlatched pos4=%f,pos5=%f",latchpos[latchidx].pos1,
		latchpos[latchidx].pos2,0,0,0,0);
	      latchidx++;
	    }
	    get_position(4,&position);
	    pos=(float)position;
	    if (negative_direction(4))
            {
	      ms_instrument_region--;
	      if (ms_instrument_region<0)ms_instrument_region=0;
	      region = ms_instrument_region;
	      direction=ms_instrument[region].direction=NEGATIVE_DIRECTION;
            }
	    else
            {
	      region = ms_instrument_region;
	      direction=ms_instrument[region].direction=POSITIVE_DIRECTION;
	      ms_instrument_region++;
	      if (ms_instrument_region>
                max_ms_instrument) ms_instrument_region=max_ms_instrument;
            }
	    if (ms_instrument[region].position[direction]==0.0)
	      ms_instrument[region].position[direction]=position;
	    else
	      ms_instrument[region].error[direction]=
		ms_instrument[region].position[direction]-position;
            ms_instrument[region].time[direction]=get_time();
            logMsg ("INSTRUMENT_INTERRUPT,%d %d, int_bit=%hx, position=%f latched=%f\r\n",
		type,i,int_bit,pos,lpos,0);
          }
          if (int_bit&(AZIMUTH_INT|ALTITUDE_INT|INSTRUMENT_INT))
            arm_latch(TRUE);

          DIO316ClearISR (tm_DIO316);
*/
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
long fiducial_position[3]={31016188     , 3766415+58807     , 640920+288012};
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
  }
  az_fiducial_position[9+24]=31016188;	/* 120:49:20:00 */
  for (i=0;i<sizeof(alt_fiducial)/sizeof(struct FIDUCIALS);i++)
  {
    alt_fiducial[i].markvalid=FALSE;
    alt_fiducial[i].last=0;
    alt_fiducial[i].err=0;
    alt_fiducial[i].poserr=0;
    alt_fiducial[i].mark=0;
  }
  alt_fiducial_position[0]=0x0;	/* 00:00:00:00 */
  alt_fiducial_position[1]=3766415+58807;	/* 090:00:00:00 */
  alt_fiducial_position[6]=0x0160E6C6;	/* 090:00:00:00 */
  for (i=0;i<sizeof(rot_fiducial)/sizeof(struct FIDUCIALS);i++)
  {
    rot_fiducial[i].markvalid=FALSE;
    rot_fiducial[i].last=0;
    rot_fiducial[i].err=0;
    rot_fiducial[i].poserr=0;
    rot_fiducial[i].mark=0;
  }
  rot_fiducial_position[40+43]=640920;   /* 001:13:35:373 */
}
void tm_latch()
{
  int i;
  double pos;
  extern int barcode_serial();
  int fididx;
/*  unsigned char int_bit;
*/
  init_fiducial();
  if (semLATCH==NULL) semLATCH = semBCreate(0,SEM_Q_FIFO);
  for (;;)
  {
    if (semTake(semLATCH,WAIT_FOREVER)!=ERROR)
    {
      if (semTake (semMEI,60)!=ERROR)
      {
/*        latch();*/
        i=0x1000;
	while ((!latch_status())&&(i>0))i--;
/*        printf ("\r\nlatch_status TRUE after %d tries; %x",0x1000-i,int_bit);*/
/*        DIO316ReadISR (tm_DIO316,&int_bit);*/
/*        printf ("; %x",int_bit);*/
	if (i!=0)
        {
        if (int_bit&AZIMUTH_INT)
        {
	  latchpos[latchidx].axis=AZIMUTH;
	  get_latched_position(0,&latchpos[latchidx].pos1);
	  get_latched_position(1,&latchpos[latchidx].pos2);
          if (LATCH_verbose)
            printf ("\r\nAXIS %d: latched pos0=%f,pos1=%f",latchpos[latchidx].axis,
	    (float)latchpos[latchidx].pos1,
	    (float)latchpos[latchidx].pos2);
          fididx = barcode_serial(3);	/* backwards from what you would think */
	  fididx = barcode_serial(3);
	  if (fididx!=-1)
	  {
	    if (latchpos[latchidx].pos1>24)
		fididx += 24;
            if ((fididx<48)&&(fididx>=0))
            {
              az_fiducial[fididx].last=az_fiducial[fididx].mark;
              az_fiducial[fididx].mark=latchpos[latchidx].pos1;
	      az_fiducial[fididx].err=az_fiducial[fididx].mark-
		az_fiducial[fididx].last;
	      az_fiducial[fididx].poserr=az_fiducial[fididx].mark-
		az_fiducial_position[fididx];
	      az_fiducial[fididx].markvalid=TRUE;
	    }
            if (fididx==fiducial[0].index)
            {
              fiducial[0].mark=az_fiducial[fididx].mark;
	      fiducial[0].markvalid=TRUE;
            }
	    fiducialidx[0]=fididx;
	  }
	}
        if (int_bit&ALTITUDE_INT)
        {
	  latchpos[latchidx].axis=ALTITUDE;
	  get_latched_position(2,&latchpos[latchidx].pos1);
	  get_latched_position(3,&latchpos[latchidx].pos2);
/*          printf ("\r\nAXIS %d: latched pos2=%f,pos3=%f",latchpos[latchidx].axis,
	    (float)latchpos[latchidx].pos1,
	    (float)latchpos[latchidx].pos2);*/
          fididx = barcode_serial(2);
	  fididx = barcode_serial(2);
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
	    }
            if (fididx==fiducial[1].index)
            {
              fiducial[1].mark=alt_fiducial[fididx].mark;
              fiducial[1].markvalid=TRUE;
            }
	    fiducialidx[1]=fididx;
	  }
	}
        if (int_bit&INSTRUMENT_INT)
        {
	  latchpos[latchidx].axis=INSTRUMENT;
/* switch to 5 for optical encoder */
	  get_latched_position(5,&latchpos[latchidx].pos1);
	  get_latched_position(4,&latchpos[latchidx].pos2);
          if (LATCH_verbose)
            printf ("\r\nAXIS %d: latched pos4=%f,pos5=%f",
	      latchpos[latchidx].axis,
	      (float)latchpos[latchidx].pos1,
	      (float)latchpos[latchidx].pos2);
	  if (rot_latch!=0)
          {
            if (abs((long)latchpos[latchidx].pos1-rot_latch)>500000)
	      fididx = abs(iround((latchpos[latchidx].pos1-rot_latch)/1600.) )-500;
	    else
	      fididx = 0;
            if (LATCH_verbose)
              printf ("\r\nAXIS %d: latched pos4=%d,rot_latch=%d,idx=%d,abspos=%d",
                latchpos[latchidx].axis,
                (long)latchpos[latchidx].pos1,
                rot_latch,fididx,
	        abs(iround((latchpos[latchidx].pos1-rot_latch)/1600.) ));
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
          if (int_bit&AZIMUTH_INT)
            latchpos[latchidx].axis=-(AZIMUTH+1);
          if (int_bit&ALTITUDE_INT)
            latchpos[latchidx].axis=-(ALTITUDE+1);
          if (int_bit&INSTRUMENT_INT)
	    latchpos[latchidx].axis=-(INSTRUMENT+1);
          printf ("\r\n BAD LATCH: latchidx=%d",latchidx);
        }
        arm_latch(TRUE);
	semGive (semMEI);
      }
      if (latchidx<MAX_LATCHED)
	latchidx++;
      else
        latchidx=0;
/*      taskDelay(60);  *//* slow the rate of interrupts */
      DIO316ClearISR (tm_DIO316);
    }
  }
}
test_azfiducials_idx (int axis, double pos, int fididx)
{
/*	    if (latchpos[latchidx].pos1>24)
		fididx += 24;*/
	    if ((pos<-24000000.)&&(fididx>5)) fididx -= 24;
	    else
	      if ((pos<24000000.)&&(fididx<12)) fididx += 24;
	    fididx += 18;
	printf ("\r\n pos=%f, fididx=%d",pos,fididx);
}

test_rotfiducials_idx (int axis, double pos, int fididx)
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
	break;	  
    case 1:
	if ((fididx<7)&&(fididx>=0))
	{
	  fiducial[axis].index=fididx;
	  fiducial[axis].markvalid=FALSE;
	  fiducial[axis].mark=fididx;
	  fiducial_position[axis]=pos;
	}
	break;	  
    case 2:
	if ((fididx<156)&&(fididx>=0))
	{
	  fiducial[axis].index=fididx;
	  fiducial[axis].markvalid=FALSE;
	  fiducial[axis].mark=fididx;
	  fiducial_position[axis]=pos;
	}
	break;	  
  }  
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
            az_fiducial_position[i]=az_fiducial[i].mark;
        }
        break;
    case 1:
        for (i=0;i<7;i++)
        {
          if (alt_fiducial[i].markvalid)
            alt_fiducial_position[i]=alt_fiducial[i].mark;
        }
        break;
    case 2:
        for (i=0;i<156;i++)
        {
          if (rot_fiducial[i].markvalid)
                rot_fiducial_position[i]=rot_fiducial[i].mark;
        }
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
	    printf ("\r\nALT FIDUCIAL %d:  pos=%d",i,
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
	    printf ("\r\nROT FIDUCIAL %d:  pos=%d",i,
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
unsigned long NIST_sec;
unsigned char int_bit;
unsigned long NIST_cnt=0;
void DID48_interrupt(int type)
{

          DID48_Read_Port (tm_DID48,5,&int_bit);
	  NIST_cnt++;
          if (int_bit&NIST_INT)
          {
            SDSStime++;
            NIST_sec=timer_read (1);
            timer_start (1);
/*            logMsg ("NIST_INTERRUPT,%d %d, int_bit=%x, cnt=%d\r\n",
		type,NIST_sec,(short)int_bit,NIST_cnt,0,0);*/
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
void DIO316_initialize(unsigned char *addr, unsigned short vecnum)
{
  STATUS stat;
  int i;
  struct IPACK *ip;

    Industry_Pack (addr,SYSTRAN_DIO316,ip);
    for (i=0;i<MAX_SLOTS;i++) 
      if (ip->adr[i]!=NULL)
      {
	printf ("\r\nFound at %d, %p",i,ip->adr[i]);
        tm_DIO316=DIO316Init(ip->adr[i], vecnum);
	break;
      }
    if (i>=MAX_SLOTS) 
    {
      printf ("\r\n****Missing DIO316 at %p****\r\n",addr);
      return;
    }
    DIO316_Init=TRUE;
/*    DIO316_Read_Reg(tm_DIO316,0xA,&vecnum);
    DIO316_Interrupt_Enable_Control (tm_DIO316,0,DIO316_INT_DIS);
    DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_DIS);
    DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_DIS);
    DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_DIS);
    if (vecnum==0) vecnum = DIO316_VECTOR;*/
    stat = intConnect (INUM_TO_IVEC(vecnum),
                                (VOIDFUNCPTR)DIO316_interrupt,
                                DIO316_TYPE);
    printf ("DIO316 vector = %d, interrupt address = %p, result = %8x\r\n",
                vecnum,DIO316_interrupt,stat);
    rebootHookAdd(axis_DIO316_shutdown);

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

    IP_Interrupt_Enable(ip,DIO316_IRQ);
    sysIntEnable(DIO316_IRQ);                                
}
void DID48_initialize(unsigned char *addr, unsigned short vecnum)
{
  STATUS stat;
  int i;
  struct IPACK *ip;

    Industry_Pack (addr,SYSTRAN_DID48,ip);
    for (i=0;i<MAX_SLOTS;i++) 
      if (ip->adr[i]!=NULL)
      {
	printf ("\r\nFound at %d, %p",i,ip->adr[i]);
        tm_DID48=DID48Init(ip->adr[i], vecnum);
	break;
      }
    if (i>=MAX_SLOTS) 
    {
      printf ("\r\n****Missing DID48 at %p****\r\n",addr);
      return;
    }
    DID48_Init=TRUE;
/*    DID48_Read_Reg(tm_DID48,0x6,&vecnum);
    if (vecnum==0) vecnum = DID48_VECTOR;*/
    stat = intConnect (INUM_TO_IVEC(vecnum),
                                (VOIDFUNCPTR)DID48_interrupt,
                                DID48_TYPE);
    printf ("DID48 vector = %d, interrupt address = %p, result = %8x\r\n",
                vecnum,DID48_interrupt,stat);
    rebootHookAdd(axis_DID48_shutdown);

    IP_Interrupt_Enable(ip,DID48_IRQ);
    sysIntEnable(DID48_IRQ);                                
    DID48_Write_Reg (tm_DID48,3,0x3); /* disable debounce for all byte lanes */
    DID48_Interrupt_Enable_Control (tm_DID48,5,DID48_INT_ENA);
}
void AZ()
{
    latch();
    DIO316_Write_Port (tm_DIO316,3,2);
}
void AL()
{
    latch();
    DIO316_Write_Port (tm_DIO316,3,4);
}
void IR()
{
    latch();
    DIO316_Write_Port (tm_DIO316,3,8);
}
void NBS()
{
    DIO316_Write_Port (tm_DIO316,3,1);
}
int amp_reset(int axis)
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
    set_error_limit(axis,16000.,ABORT_EVENT);
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
  taskSpawn ("tmLatch",0,VX_FP_TASK,10000,(FUNCPTR)tm_latch,0,0,0,0,0,0,0,0,0,0);
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
	int error_code;
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

int sdss_move_time (int axis, int vel, int accel, int time)
{
	print_coeffs(axis);
	v_move(axis,vel,accel);
	taskDelay (time);
	v_move(axis,0,accel);
}
int sdss_move_offset (int axis, int off)
{
	short coeff[COEFFICIENTS];

	print_coeffs(axis);
	get_filter (axis,(P_INT)coeff);
	if (off==0)
	{
	  get_boot_filter (axis,(P_INT)coeff);
	}
	else
	{
	  coeff[DF_P]=0;
	  coeff[DF_I]=0;
	  coeff[DF_D]=0;
	}
	coeff[DF_OFFSET]=off;
	set_filter (axis,(P_INT)coeff);

	print_coeffs(axis);
}
int sdss_move_pos (int axis, int vel, int accel, int pos)
{
	double position,last_pos,final_pos;

	print_coeffs(axis);
	get_position(axis,&position);
	last_pos=position;
	final_pos=position+pos;
	start_move(axis,final_pos,(double)vel,(double)accel);
	taskDelay(1);
	get_position(axis,&position);
	while (last_pos!=position) 
	{
	  last_pos=position;
	  taskDelay(1);
	  get_position(axis,&position);
	  printf("\r\npos=%f",(float)position);
	}
	printf("\r\n  Stop pos=%f",(float)position);
	set_stop (axis);
	while (!motion_done(axis));
	clear_status(axis);
        get_position(axis,&position);
        printf("\r\n  Final pos=%f",(float)position);
	if ((final_pos>position+10)||(final_pos<position-10))
	  printf ("\r\n ERROR: did not close in on position");
}
int print_coeffs(int axis)
{
	short coeff[COEFFICIENTS];

	get_filter (axis,(P_INT)coeff);
	printf ("\r\n AXIS %d: P=%d, I=%d, D=%d",axis,
		coeff[0],coeff[1],coeff[2]);
	printf ("\r\n          AFF=%d, VFF=%d, FFF=%d",
		coeff[3],coeff[4],coeff[9]);
	printf ("\r\n          ILIMIT=%d, OFFSET=%d, OLIMIT=%d",
		coeff[5],coeff[6],coeff[7]);
}
int set_coeffs(int axis, int index, int val)
{
	short coeff[COEFFICIENTS];

	get_filter (axis,(P_INT)coeff);
	coeff[index]=val;
	set_filter (axis,(P_INT)coeff);
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
	print_coeffs (axis);
}
int sdss_remap(int axis)
{
	int i;

/* initialize if not done */
        if (!sdss_was_init) sdss_init();
/* home the axis */
	sdss_home(axis);
/* initialize the sensors */
	switch (axis) {
	case AZIMUTH:
	  max_ms_azimuth = sizeof(ms_azimuth)/sizeof(struct MAG_SENSOR);
          for (i=0;i<max_ms_azimuth;i++)
          {
            ms_azimuth[i].encoder[0]=ms_azimuth[i].encoder[1]=0.0;
            ms_azimuth[i].error[0]=ms_azimuth[i].error[1]=0.0;
            ms_azimuth[i].status[0]=ms_azimuth[i].status[1]=0;
            ms_azimuth[i].time[0]=ms_azimuth[i].time[1]=0.0;
          }
	  ms_azimuth_region=0;
	  break;	  
	case ALTITUDE:
	  max_ms_altitude = sizeof(ms_altitude)/sizeof(struct MAG_SENSOR);
          for (i=0;i<max_ms_altitude;i++)
          {
            ms_altitude[i].encoder[0]=ms_altitude[i].encoder[1]=0.0;
            ms_altitude[i].error[0]=ms_altitude[i].error[1]=0.0;
            ms_altitude[i].status[0]=ms_altitude[i].status[1]=0;
            ms_altitude[i].time[0]=ms_altitude[i].time[1]=0.0;
          }
	  ms_altitude_region=0;
	  break;	  
	case INSTRUMENT:
	  max_ms_instrument = sizeof(ms_instrument)/sizeof(struct MAG_SENSOR);
          for (i=0;i<max_ms_instrument;i++)
          {
            ms_instrument[i].encoder[0]=ms_instrument[i].encoder[1]=0.0;
            ms_instrument[i].error[0]=ms_instrument[i].error[1]=0.0;
            ms_instrument[i].status[0]=ms_instrument[i].status[1]=0;
            ms_instrument[i].time[0]=ms_instrument[i].time[1]=0.0;
          }
	  ms_instrument_region=0;
	  break;	  
	}
/* start the movement and let the interrupt handler set the position */
	v_move(axis,MAP_VEL,MAP_ACCEL);
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
  	  struct timespec tp;
  	  unsigned long micro_sec;

          micro_sec = timer_read (1);
          return (float)(SDSStime+((micro_sec%1000000)/1000000.));
}
float get_time()
{
  	  struct timespec tp;
  	  unsigned long micro_sec;

          micro_sec = timer_read (1);
	  printf ("time=%f\r\n",
		(float)(SDSStime+((micro_sec%1000000)/10000000.)));
          return (float)(SDSStime+((micro_sec%1000000)/10000000.));
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
	  printf ("sec=%d, day_sec=%d, micro_sec=%d, time=%f\r\n",
		tp.tv_sec,tp.tv_sec%ONE_DAY,micro_sec,
		(float)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
          return ((float)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
#endif
int latch_done=FALSE;
test_latch (int ticks)
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
latch_it ()
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
int latchstart ()
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
latchprint (char *description)
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
latchexcel (int axis)
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
print_max ()
{
  int i;

  for (i=0;i<3;i++)
    printf ("\r\nAXIS %d: MAX ACC limit %lf deg/sec/sec current max %lf",
	  i,max_acceleration[i],max_acceleration[i+3]);
  for (i=0;i<3;i++)
    printf ("\r\nAXIS %d: MAX VEL limit %lf deg/sec current max %lf",
	  i,max_velocity[i],max_velocity[i+3]);
}
