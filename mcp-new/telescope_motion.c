#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	telescope_motion.c					*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:								*/
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
#include "stdio.h"
#include "semLib.h"
#include "sigLib.h"
#include "tickLib.h"
#include "taskLib.h"
#include "inetLib.h"
#include "in.h"
#include "timers.h"
#include "time.h"
#include "ms.h"
#include "abdh.h"
#include "idsp.h"
#include "pcdsp.h"
#include "dio316ld.h"
#include "did48ld.h"
#include "ad12f1ld.h"
#include "ip480.h"
#include "mv162IndPackInit.h"
#include "axis.h"
#include "frame.h"
#include "data_collection.h"
#include "gendefs.h"
#include "tm.h"

#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONCE if (YES)

/* Global Data */
int display_enable[6]={FALSE,FALSE,FALSE,FALSE,TRUE,TRUE};
int continuous_enable=FALSE;
int TM_verbose=FALSE;

short az1vlt,az1cur,az2vlt,az2cur;
short alt1vlt,alt1cur,alt2vlt,alt2cur;
short rot1vlt,rot1cur;
#define TM_ROT1VLT	6
#define TM_ROT1CUR	7
#define TM_AZ1VLT	0
#define TM_AZ1CUR	1
#define TM_AZ2VLT	4
#define TM_AZ2CUR	5
#define TM_ALT1VLT	2
#define TM_ALT1CUR	3
#define TM_ALT2VLT	6
#define TM_ALT2CUR	7
int tm_ADC128F1=-1;

void tm_move_time (int axis, int vel, int accel, int time)
{
	extern SEM_ID semMEI;
	extern void manTrg();

	manTrg();
	tm_print_coeffs(axis);
	semTake(semMEI,WAIT_FOREVER);
	v_move(axis,(double)vel,(double)accel);
	semGive (semMEI);
	taskDelay (time);
	semTake(semMEI,WAIT_FOREVER);
	v_move(axis,(double)0,(double)accel);
	semGive (semMEI);
}
void tm_move_offset (int axis, int off)
{
	short coeff[COEFFICIENTS];
	extern init_coeffs(int axis);
	extern SEM_ID semMEI;
	extern void manTrg();

	manTrg();
	tm_print_coeffs(axis);
	semTake(semMEI,WAIT_FOREVER);
	get_filter (axis,(P_INT)coeff);
	if (off==0)
	{
	  get_boot_filter (axis,(P_INT)coeff);
	  init_coeffs (axis);
	}
	else
	{
	  coeff[DF_P]=0;
	  coeff[DF_I]=0;
	  coeff[DF_D]=0;
	}
	coeff[DF_OFFSET]=off;
	set_filter (axis,(P_INT)coeff);

	semGive (semMEI);
	tm_print_coeffs(axis);
}
void tm_bf (int axis, int vel, int accel, int pos1,int pos2, int times)
{
  int i;
  int status;
  extern SEM_ID semMEI;

  for (i=0;i<times;i++)
  {
    printf ("\r\nPass %d",i);
      tm_controller_run (axis);
      tm_start_move (axis,vel,accel,pos1);
      status=FALSE;
      while (!status)
      {
	taskDelay(60);
	semTake(semMEI,WAIT_FOREVER);
	status=motion_done(axis);
	semGive (semMEI);
      }
      tm_start_move (axis,vel,accel,pos2);
      status=FALSE;
      while (!status)
      {
	taskDelay(60);
	semTake(semMEI,WAIT_FOREVER);
	status=motion_done(axis);
	semGive (semMEI);
      }
      tm_controller_idle(axis);    
    printf(" Done");
  }
}
void tm_start_move (int axis, int vel, int accel, int pos)
{
	extern SEM_ID semMEI;
	extern void manTrg();

/*	manTrg();*/
	semTake(semMEI,WAIT_FOREVER);
/*	print_coeffs(axis);*/
	start_move(axis,(double)pos,(double)vel,(double)accel);
	semGive (semMEI);
}
void tm_move_pos (int axis, int vel, int accel, int pos)
{
	double position,last_pos,final_pos;
	extern SEM_ID semMEI;
	extern void manTrg();

	manTrg();
	tm_print_coeffs(axis);
	semTake(semMEI,WAIT_FOREVER);
	get_position(axis,&position);
	last_pos=position;
	final_pos=position+(double)pos;
	start_move(axis,final_pos,(double)vel,(double)accel);
	semGive (semMEI);
	taskDelay(60);
	semTake(semMEI,WAIT_FOREVER);
	get_position(axis,&position);
	semGive (semMEI);
	while (last_pos!=position) 
	{
	  last_pos=position;
	  taskDelay(60);
	  semTake(semMEI,WAIT_FOREVER);
	  get_position(axis,&position);
	  semGive (semMEI);
	  printf("\r\npos=%f",(float)position);
	}
	printf("\r\n  Stop pos=%f",(float)position);
/*
	semTake(semMEI,WAIT_FOREVER);
	set_stop (axis);
	while (!motion_done(axis));
	clear_status(axis);
        get_position(axis,&position);
	semGive (semMEI);
*/
        printf("\r\n  Final pos=%f",(float)position);
	if ((final_pos>position+10)||(final_pos<position-10))
	  printf ("\r\n ERROR: did not close in on position");
}
void tm_print_coeffs(int axis)
{
	short coeff[COEFFICIENTS];
	extern SEM_ID semMEI;
	short mode;
	short rate;

	semTake(semMEI,WAIT_FOREVER);
	get_filter (axis,(P_INT)coeff);
	get_integration (axis,&mode);
	rate=dsp_sample_rate();
	semGive (semMEI);
	printf ("\r\n AXIS %d: P=%d, I=%d, D=%d",axis,
		coeff[0],coeff[1],coeff[2]);
	printf ("\r\n          AFF=%d, VFF=%d, FFF=%d",
		coeff[3],coeff[4],coeff[9]);
	printf ("\r\n          ILIMIT=%d, OFFSET=%d, OLIMIT=%d, SHIFT=%d",
		coeff[5],coeff[6],coeff[7],coeff[8]);
	printf ("\r\n integration mode is %d",mode);
	printf ("\r\n and sample is %d Hz",rate);
}
void tm_set_coeffs(int axis, int index, int val)
{
	short coeff[COEFFICIENTS];
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	get_filter (axis,(P_INT)coeff);
	coeff[index]=val;
	set_filter (axis,(P_INT)coeff);
	semGive (semMEI);
}
void tm_display_axis(int axis)
{
  display_enable[axis]=TRUE;
}
void tm_nodisplay_axis(int axis)
{
  display_enable[axis]=FALSE;
}
void tm_display_continuous()
{
  continuous_enable=TRUE;
}
void tm_display_once()
{
  continuous_enable=FALSE;
}
void tm_display (int delay)
{
	int i;
	short out;
	double pos;
	extern SEM_ID semMEI;
	int first=TRUE;

      while ((continuous_enable)||(first))
      {
	first=FALSE;
	taskDelay(delay);
	for (i=0;i<6;i++)
	{
	  if (display_enable[i])
	  {
	    if (semTake(semMEI,WAIT_FOREVER)!=ERROR)
	    {
	      get_dac_output(i,&out);
	      get_position(i,&pos);
	      semGive (semMEI);
	      printf ("\r\nAxis %d: out=%7.4f volts, pos=%10.0f ",
		i,(float)out/3276.8,(float)pos);
	    }
	  }
	}
      }
}
void tm_clear_pos (int axis)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_position (axis,0.0);
	semGive(semMEI);
}
void tm_get_pos (int axis,double *position)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
        get_position(axis,position);
	semGive(semMEI);
}
void tm_get_vel (int axis,double *velocity)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
        get_velocity(axis,velocity);
	semGive(semMEI);
}
void tm_set_sample_rate (unsigned short rate)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_sample_rate (rate);
	printf("\r\n Sample Rate=%d",(unsigned short)dsp_sample_rate());
	semGive(semMEI);
}
void tm_reset_integrator (unsigned short axis)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	reset_integrator (axis);
	semGive(semMEI);
}
void tm_set_pos (int axis,int pos)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_position (axis,(double)pos);
	semGive(semMEI);
}
void tm_set_analog_encoder(int axis, int channel)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_analog_channel(axis,channel,TRUE,TRUE);
	set_axis_analog (axis,TRUE);
	set_feedback(axis,FB_ANALOG);
	semGive(semMEI);
}
void tm_set_encoder(int axis)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_feedback(axis,FB_ENCODER);
	semGive(semMEI);
}
void tm_set_analog_channel(int axis, int channel)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_analog_channel(axis,channel,TRUE,TRUE);
	set_axis_analog (axis,TRUE);
	semGive(semMEI);
}
void tm_controller_run (int axis)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	controller_run (axis);
	controller_run (axis);
	controller_run (axis);
	semGive(semMEI);
}
void tm_controller_idle (int axis)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	controller_idle (axis);
	semGive(semMEI);
}
void tm_dual_loop (int axis, int dual)
{
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	set_dual_loop (axis,axis+1,dual);
	semGive(semMEI);
}

void tm_set_boot_filter (int axis)
{
	short coeff[COEFFICIENTS];
	extern SEM_ID semMEI;

	semTake(semMEI,WAIT_FOREVER);
	get_filter(axis,(P_INT)coeff);
	set_boot_filter(axis,(P_INT)coeff);
	semGive(semMEI);
}
void tmDisplay (int delay)
{
	taskSpawn("tmDisp",90,0,1000,(FUNCPTR)tm_display,delay,0,0,0,0,0,0,0,0,0);
}
char *help_TM[]={
        "TM_help;  axis 0,1=ALT, axis 2,3=AL, axis 4,5=ROT",
	"TM_Verbose, TM_Quiet",
	"tmDisplay(int delay)",
	"tm_move_time(int axis, int vel, int accel, int time)",
	"tm_start_move(int axis, int vel, int accel, int absolutepos)",
	"tm_move_pos(int axis, int vel, int accel, int relativepos)",
	"tm_set_coeffs(int axis, int index, int val)",
	" index=0(P),1(I),2(D),3(AFF),4(VFF),5(ILIM),6(OFF),7(DLIM)",
	" 8(SHIFT)(-5 is 1/32),9(FFF)",
	"tm_print_coeffs(int axis)",
	"tm_display_axis(int axis)",
	"tm_nodisplay_axis(int axis)",
	"tm_display(int delay)",
	"tm_clear_pos(int axis)",
	"tm_set_analog_encoder(int axis, int channel)",
	"tm_set_encoder(int axis)",
	"tm_controller_idle(int axis)",
	"tm_dual_loop(int axis, int dual)",
	"tm_set_fiducial(int axis); tm_get_fiducial_all()",
""
};                                                         
void TM_help()
{
  int i;

  for (i=0;i<sizeof(help_TM)/sizeof(char *);i++)
    printf ("%s\r\n",help_TM[i]);
}
void TM_Verbose()
{
	TM_verbose=TRUE;
}
void TM_Quiet()
{
	TM_verbose=FALSE;
}
int ADC128F1_initialize(unsigned char *addr, int occur)
{
  int i;
  struct IPACK ip;

  Industry_Pack (addr,SYSTRAN_ADC128F1,&ip);
  for (i=0;i<MAX_SLOTS;i++) 
    if (ip.adr[i]!=NULL)
    {
      if (occur==0)
      {	
        printf ("\r\nFound one at %d, %p",i,ip.adr[i]);
        tm_ADC128F1=ADC128F1Init(ip.adr[i]);
	printf ("\r\n tm_ADC128F1=%d",tm_ADC128F1);
        ADC128F1_CVT_Update_Control(tm_ADC128F1,ENABLE);
	break;
      }
      else occur--;;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing ADC128F1 at %p****\r\n",addr);
    return ERROR;
  }
}
void tm_data_collection()
{
  short adc;
  extern int cw_ADC128F1;

  if (cw_ADC128F1!=-1)
  {
    ADC128F1_Read_Reg(cw_ADC128F1,TM_ROT1VLT,&adc);
    if ((adc&0x800)==0x800) rot1vlt=adc|0xF000;
    else rot1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(cw_ADC128F1,TM_ROT1CUR,&adc);
    if ((adc&0x800)==0x800) rot1cur=adc|0xF000;
    else rot1cur = adc&0xFFF;
  }
  if (tm_ADC128F1!=-1)
  {
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ1VLT,&adc);
    if ((adc&0x800)==0x800) az1vlt=adc|0xF000;
    else az1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ1CUR,&adc);
    if ((adc&0x800)==0x800) az1cur=adc|0xF000;
    else az1cur = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ2VLT,&adc);
    if ((adc&0x800)==0x800) az2vlt=adc|0xF000;
    else az2vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ2CUR,&adc);
    if ((adc&0x800)==0x800) az2cur=adc|0xF000;
    else az2cur = adc&0xFFF;
      
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT1VLT,&adc);
    if ((adc&0x800)==0x800) alt1vlt=adc|0xF000;
    else alt1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT1CUR,&adc);
    if ((adc&0x800)==0x800) alt1cur=adc|0xF000;
    else alt1cur = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT2VLT,&adc);
    if ((adc&0x800)==0x800) alt2vlt=adc|0xF000;
    else alt2vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT2CUR,&adc);
    if ((adc&0x800)==0x800) alt2cur=adc|0xF000;
    else alt2cur = adc&0xFFF;
  }
}
void tm_read_all_adc(int cnt)
{
  int i,ii;
  extern int cw_ADC128F1;
  short adc;

  for (i=0;i<cnt;i++)
  {
    for (ii=6;ii<8;ii++)
    {
      ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\nCW chan %d:  0x%x  %4.2fv  ",ii,adc,(10*adc)/2048.);
    }
    for (ii=0;ii<8;ii++)
    {
      ADC128F1_Read_Reg(tm_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\nTM chan %d:   0x%x  %4.2fv  ",ii,adc,(10*adc)/2048.);
    }
    printf ("\r\n");
  }
}
void tm_jog_axis()
{
  char ch;

  while (ch=getchar()) printf("\r\n%x",ch);
}
int az_cnt;
int tm_az_brake(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10 tm_ctrl;   
   extern SEM_ID semSLC;
   extern struct SDSS_FRAME sdssdc;
   int cnt;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl,(char *)&tm_ctrl,2);
/*   printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   if (val==1) 
   {
     tm_ctrl.mcp_az_brk_en_cmd = 1;
     tm_ctrl.mcp_az_brk_dis_cmd = 0;
   }
   else
   {
     tm_ctrl.mcp_az_brk_en_cmd = 0;
     tm_ctrl.mcp_az_brk_dis_cmd = 1;
   }
   
/*   printf (" write ctrl = 0x%4x\r\n",tm_ctrl);*/
   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl,(char *)&tm_ctrl,2);
   if (val==1)
   {
     cnt=120;
     while ((sdssdc.status.i78.il0.az_brake_engaged==0)&&(cnt>0)) 
     {
       taskDelay(1);
       cnt--;
     }
/*     tm_ctrl.mcp_az_brk_en_cmd = 0;*/
   }
   else
   {
     cnt=60*6;
     while ((sdssdc.status.i78.il0.az_brake_disengaged==0)&&(cnt>0))
     {
       taskDelay(1);
       cnt--;
     }
     taskDelay(12*60);
     tm_ctrl.mcp_az_brk_dis_cmd = 0;
   }
   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
/*   printf ("\r\n cnt=%d",cnt);
   tm_brake_status();*/
	az_cnt=cnt;
}
void tm_az_brake_on()
{
    tm_az_brake (1);
}
void tm_az_brake_off()
{
    tm_az_brake (0);
}
void tm_sp_az_brake_on()
{
  if (taskIdFigure("tmAzBrk")!=NULL)
    taskSpawn("tmAzBrk",90,0,1000,(FUNCPTR)tm_az_brake_off,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_az_brake_off()
{
  if (taskIdFigure("tmAzBrk")!=NULL)
    taskSpawn("tmAzBrk",90,0,1000,(FUNCPTR)tm_az_brake_off,0,0,0,0,0,0,0,0,0,0);
}
int alt_cnt;
int tm_alt_brake(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10 tm_ctrl;   
   extern SEM_ID semSLC;
   extern struct SDSS_FRAME sdssdc;
   int cnt;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl,(char *)&tm_ctrl,2);
/*   printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   if (val==1) 
   {
     tm_ctrl.mcp_alt_brk_en_cmd = 1;
     tm_ctrl.mcp_alt_brk_dis_cmd = 0;
   }
   else
   {
     tm_ctrl.mcp_alt_brk_en_cmd = 0;
     tm_ctrl.mcp_alt_brk_dis_cmd = 1;
   }
   
/*   printf (" write ctrl = 0x%4x\r\n",tm_ctrl);*/
   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl,(char *)&tm_ctrl,2);
   cnt=60*4;
   if (val==1)
   {
     while ((sdssdc.status.i78.il0.alt_brake_engaged==0)&&(cnt>0))
     {
        taskDelay(1);
        cnt--;
     }
/*     tm_ctrl.mcp_alt_brk_en_cmd = 0;*/
   }
   else
   {
     while ((sdssdc.status.i78.il0.alt_brake_disengaged==0)&&(cnt>0)) 
     {
       taskDelay(1);
       cnt--;
     }
     taskDelay(60*4);
     tm_ctrl.mcp_alt_brk_dis_cmd = 0;
   }
   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
/*   printf ("\r\n cnt=%d",cnt);
   tm_brake_status();*/
   alt_cnt=cnt;
   return 0;
}
void tm_alt_brake_on()
{
    tm_alt_brake (1);
}
void tm_alt_brake_off()
{
    tm_alt_brake (0);
}
void tm_sp_alt_brake_on()
{
  if (taskIdFigure("tmAltBrk")!=NULL)
    taskSpawn("tmAltBrk",90,0,1000,(FUNCPTR)tm_alt_brake_off,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_alt_brake_off()
{
  if (taskIdFigure("tmAltBrk")!=NULL)
    taskSpawn("tmAltBrk",90,0,1000,(FUNCPTR)tm_alt_brake_off,0,0,0,0,0,0,0,0,0,0);
}
int tm_brake_status()
{
  int err;
  unsigned short ctrl;
  struct B10 tm_ctrl;   
  extern SEM_ID semSLC;
  extern struct SDSS_FRAME sdssdc;

  printf("\r\nAZ\tEngaged=%d\tDisengaged=%d, cnt=%d\n",
    sdssdc.status.i78.il0.az_brake_engaged,
    sdssdc.status.i78.il0.az_brake_disengaged,az_cnt);
  printf("\r\nALT\tEngaged=%d\tDisengaged=%d, cnt=%d\n",
    sdssdc.status.i78.il0.alt_brake_engaged,
    sdssdc.status.i78.il0.alt_brake_disengaged,alt_cnt);
  if (semTake (semSLC,60)!=ERROR)
  {
    err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
    semGive (semSLC);
    if (err)
    {
      printf ("R Err=%04x\r\n",err);
      return err;
    }
  }
  swab ((char *)&ctrl,(char *)&tm_ctrl,2);
  printf (" read ctrl = 0x%04x\r\n",ctrl);
  return 0;
}

int clamp_cnt;
int tm_clamp(short val) 
{
   int err;
   unsigned short ctrl[2];
   struct B10 tm_ctrl;   
   struct B10_1 tm_ctrl1;   
   extern SEM_ID semSLC;
   extern struct SDSS_FRAME sdssdc;
   int cnt;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);
/*   printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   if (val==1) 
   {
     tm_ctrl.mcp_clamp_en_cmd = 1;
     tm_ctrl1.mcp_clamp_dis_cmd = 0;
   }
   else
   {
     tm_ctrl.mcp_clamp_en_cmd = 0;
     tm_ctrl1.mcp_clamp_dis_cmd = 1;
   }
   
/*   printf (" write ctrl = 0x%4x\r\n",tm_ctrl);*/
   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   swab ((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);
   cnt=60*5;
   if (val==1) 
   {
     while ((sdssdc.status.i11o12.ol0.clamp_engaged_st==0)&&(cnt>0))
     {
        taskDelay(1);
        cnt--;
     }
     if (sdssdc.status.i11o12.ol0.clamp_engaged_st==0) /* did not work */
     {
       tm_ctrl.mcp_clamp_en_cmd = 0;
       printf ("\r\n Clamp did NOT engage...turning off ");
     }
   }
   else
   {
     while ((sdssdc.status.i11o12.ol0.clamp_disengaged_st==0)&&(cnt>0)) 
     {
       taskDelay(1);
       cnt--;
     }
     taskDelay(60*4);
     tm_ctrl1.mcp_clamp_dis_cmd = 0;
   }
   swab ((char *)&tm_ctrl,(char *)&ctrl[0],2);
   swab ((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
/*   printf ("\r\n cnt=%d",cnt);
   tm_clamp_status();*/
   clamp_cnt=cnt;
   return 0;
}
void tm_clamp_on()
{
    tm_clamp (1);
}
void tm_clamp_off()
{
    tm_clamp (0);
}
int tm_clamp_status()
{
  int err;
  unsigned short ctrl[0];
  struct B10 tm_ctrl;   
  extern SEM_ID semSLC;
  extern struct SDSS_FRAME sdssdc;

  printf("\r\nCLAMP\tEngaged=%d\tDisengaged=%d, cnt=%d\n",
    sdssdc.status.i11o12.ol0.clamp_engaged_st,
    sdssdc.status.i11o12.ol0.clamp_disengaged_st,alt_cnt);
  if (semTake (semSLC,60)!=ERROR)
  {
    err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
    semGive (semSLC);
    if (err)
    {
      printf ("R Err=%04x\r\n",err);
      return err;
    }
  }
  swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
  printf (" read ctrl = 0x%04x 0x%4x\r\n",ctrl[0],ctrl[1]);
  return 0;
}


int az_amp_ok()
{
  extern struct SDSS_FRAME sdssdc;

  if ((sdssdc.status.i56.il0.az_mtr_ccw) &&
	(sdssdc.status.i56.il0.az_mtr_cw))
	return TRUE;
  else
	return FALSE;
}
int alt_amp_ok()
{
  extern struct SDSS_FRAME sdssdc;

  if ((sdssdc.status.i78.il0.alt_mtr_dn) &&
	(sdssdc.status.i78.il0.alt_mtr_up))
	return TRUE;
  else
	return FALSE;
}
int rot_amp_ok()
{
  extern struct SDSS_FRAME sdssdc;

  if ((sdssdc.status.i78.il0.rot_mtr_rdy) &&
        (sdssdc.status.i78.il0.rot_mtr_ccw) &&
        (sdssdc.status.i78.il0.rot_mtr_cw))
	return TRUE;
  else
	return FALSE;
}
#define TM_WD		4		/* WD channel    15 */
void tm_amp_mgt()
{
  FOREVER
  {
    taskDelay (60);
    tm_amp_engage();
    if (!az_amp_ok())
    {
      tm_controller_idle(0);
      tm_controller_idle(0);
      tm_controller_idle(0);
    }
    if (!alt_amp_ok())
    {
      tm_controller_idle(2);
      tm_controller_idle(2);
      tm_controller_idle(2);
    }
    if (!rot_amp_ok())
    {
      tm_controller_idle(4);
      tm_controller_idle(4);
      tm_controller_idle(4);
    }
  }
}
void tm_print_amp_status()
{
  extern struct SDSS_FRAME sdssdc;

    if (!az_amp_ok())
      printf ("\r\nAz Amp Disengaged: az_mtr_ccw=%d,az_mtr_cw=%d",
	sdssdc.status.i56.il0.az_mtr_ccw,
	sdssdc.status.i56.il0.az_mtr_cw);
    else
      printf ("\r\nAZ Amp OK");
    if (!alt_amp_ok())
      printf ("\r\nAlt Amp Disengaged: alt_mtr_dn=%d,alt_mtr_up=%d",
	sdssdc.status.i78.il0.alt_mtr_dn,
	sdssdc.status.i78.il0.alt_mtr_up);
    else
      printf ("\r\nALT Amp OK");
    if (!rot_amp_ok())
      printf ("\r\nRot Amp Disengaged: rot_mtr_rdy=%d,rot_mtr_ccw=%d,rot_mtr_cw=%d",
	sdssdc.status.i78.il0.rot_mtr_rdy,
	sdssdc.status.i78.il0.rot_mtr_ccw,
	sdssdc.status.i78.il0.rot_mtr_cw);
    else
      printf ("\r\nROT Amp OK");
}
void tm_amp_disengage()
{
  extern struct conf_blk sbrd;

  StopCounter (&sbrd,TM_WD);
}
void tm_amp_engage()
{
  extern struct conf_blk sbrd;

  WriteCounterConstant (&sbrd,TM_WD);		/* 2 Sec */
  StartCounter (&sbrd,TM_WD);
}
void tm_setup_wd ()
{
  extern struct conf_blk sbrd;

  SetCounterSize (&sbrd,TM_WD,CtrSize32);
  SetCounterConstant (&sbrd,TM_WD,2000000);		/* 2 Sec */
  SetMode (&sbrd,TM_WD,Watchdog);
  SetDebounce (&sbrd,TM_WD,DebounceOff);
  SetInterruptEnable(&sbrd,TM_WD,IntEnable);
  SetClockSource (&sbrd,TM_WD,InC1Mhz);
  SetTriggerSource (&sbrd,TM_WD,InTrig);
  SetWatchdogLoad (&sbrd,TM_WD,WDIntLd);
  SetOutputPolarity (&sbrd,TM_WD,OutPolLow);
  ConfigureCounterTimer(&sbrd,TM_WD);
}
void tm_set_fiducial(int axis)
{
  extern long fiducial_position[3];
  int negative;
  long pos, deg, min, arcsec, marcsec;
  char buf[16];
   
  axis=axis>>1;
  printf("Set Fiducial Position    xxx:xx:xx:xxx  ");
  gets(buf);
				/*sscanf (buf,"%d",&pos);*/
  if (buf[0]=='-') 
  {
    sscanf (&buf[1],"%ld:%ld:%ld:%ld",
	&deg,&min,&arcsec,&marcsec);
    negative=TRUE;
  }
  else
  {
    sscanf (buf,"%ld:%ld:%ld:%ld",
    	&deg,&min,&arcsec,&marcsec);
    negative=FALSE;
  }
  if (axis==0)
    pos=(long)((abs(deg)*3600000.)+(min*60000.)+
	 (arcsec*1000.)+marcsec)/(AZ_TICK*1000);
  if (axis==1)
    pos=(long)((abs(deg)*3600000.)+(min*60000.)+
	 (arcsec*1000.)+marcsec)/(ALT_TICK*1000);
  if (axis==2)
     pos=(long)((abs(deg)*3600000.)+(min*60000.)+
         (arcsec*1000.)+marcsec)/(ROT_TICK*1000);
  if (negative) pos=-pos;
  fiducial_position[axis]=pos;
  tm_get_fiducial(axis<<1);
}
void tm_get_fiducial_all()
{
  int i;

  for(i = 0; i < 3; i++)
    tm_get_fiducial(i<<1);
  printf ("\r\n");
}
void tm_get_fiducial(int axis)
{
  extern struct FIDUCIARY fiducial[3];
  extern long fiducial_position[3];
  long marcs,arcs,arcm,arcd;
  double arcsec, farcsec;
  int i;

  i=axis>>1;
       	    if (i==0)
	    {
	      printf ("\r\nAxis AZ(0):");
	      arcsec=(AZ_TICK*abs(fiducial_position[i]));
	      farcsec=(AZ_TICK*abs(fiducial[i].mark));
	    }
       	    if (i==1)
	    {
	      printf ("\r\nAxis ALT(2):");
	      arcsec=(ALT_TICK*abs(fiducial_position[i]));
	      farcsec=(ALT_TICK*abs(fiducial[i].mark));
	    }
       	    if (i==2)
	    {
	      printf ("\r\nAxis ROT(4):");
	      arcsec=(ROT_TICK*abs(fiducial_position[i]));
	      farcsec=(ROT_TICK*abs(fiducial[i].mark));
	    }
	    arcd=(long)(arcsec)/3600;	     
	    arcm=((long)(arcsec)-(arcd*3600))/60;	     
	    arcs=((long)(arcsec)-(arcd*3600)-(arcm*60));	     
	    marcs = (arcsec-(long)arcsec)*1000;
	    printf ("\r\n Fiducial Position = ",i);
	    if (fiducial_position[i]<0)
	      printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	    else
	      printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	    arcd=(long)(farcsec)/3600;	     
	    arcm=((long)(farcsec)-(arcd*3600))/60;	     
	    arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
	    marcs = (farcsec-(long)farcsec)*1000;
	    printf (" Fiducial Position Mark = ");
	    if (fiducial[i].markvalid)
	    {
	      if (fiducial[i].mark<0)
	        printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	      else
	        printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	    }
	    else
	      printf("     NOT Valid");
}
void tm_set_fiducials(int axis)
{
  extern long fiducial_position[3];
  extern struct FIDUCIARY fiducial[3];
  extern long fiducial_position[3];
  extern struct TM_M68K *tmaxis[];
  double pos;

                        if (fiducial[axis/2].markvalid)
                        {
                          pos=fiducial_position[axis/2];
                          pos += ((*tmaxis[axis/2]).actual_position-
                                fiducial[axis/2].mark);
                          fiducial[axis/2].mark=fiducial_position[axis/2];
                          tm_set_pos(axis&0x6,pos);
/*                        if (axis==0)
                            tm_set_pos(axis+1,pos);*/
                          if (axis/2==2)
                            tm_set_pos(axis+1,pos);
                          if (axis/2==2)
                            tm_set_pos(axis-1,pos);
                        }
                        else
                          printf("ERR: fiducial for axis not crossed      ");
}
char *msg_axis_status[]=
	{"IN_SEQUENCE",
	 "IN_POSITION",
	 "IN_MOTION",
	 "DIRECTION positive",
	 "FRAMES_LEFT"};
int  tm_axis_status(int axis)
{
  int value;
  extern SEM_ID semMEI;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_status(axis);
  semGive(semMEI);
  return value;
}
void tm_print_axis_status(int axis)
{
  int i,value;
  extern SEM_ID semMEI;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_status(axis);

  semGive(semMEI);
  printf ("AXIS STATUS: %x",value);
  for (i=0;i<sizeof(msg_axis_status)/sizeof(char *);i++)
    if ((value>>(i+4))&1) printf ("     %s\r\n",msg_axis_status[i]);
}
char *msg_axis_state[]=
	{"NO_EVENT",
	 "NEW_FRAME",
	 "STOP_EVENT",
	 "E_STOP_EVENT",
	 "ABORT_EVENT",
	 "Running???",
	 "Undocumented Value"};
int tm_axis_state(int axis)
{
  int value;
  extern SEM_ID semMEI;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_state(axis);
  semGive(semMEI);
  return value;
}
void tm_print_axis_state(int axis)
{
  int i,value;
  extern SEM_ID semMEI;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_state(axis);
  semGive(semMEI);
  printf ("AXIS STATE: %x",value);
  switch (value)
  {
    case NO_EVENT:
      i=0;
      break;
    case NEW_FRAME:
      i=1;
      break;
    case STOP_EVENT:
      i=2;
      break;
    case E_STOP_EVENT:
      i=3;
      break;
    case ABORT_EVENT:
      i=4;
      break;
    case 1:
      i=5;
      break;
    default:
      i=6;
  }
  printf ("     %s\r\n",msg_axis_state[i]);
}
char *msg_axis_source[]=
	{"ID_NONE",
	 "ID_HOME_SWITCH",
	 "ID_POS_LIMIT",
	 "ID_NEG_LIMIT",
	 "ID_AMP_FAULT",
	 "unused",
	 "unused",
	 "ID_X_NEG_LIMIT",
	 "ID_X_POS_LIMIT",
	 "ID_ERROR_LIMIT",
	 "ID_PC_COMMAND",
	 "ID_OUT_OF_FRAMES",
	 "ID_TEMPO_PROBE_FAULT",
	 "ID_AXIS_COMMAND"};
void tm_print_axis_source(int axis)
{
  int value;
  extern SEM_ID semMEI;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_source(axis);
  semGive(semMEI);
  printf ("AXIS SOURCE: %x",value);
    printf ("     %s\r\n",msg_axis_source[value]);
}                                                              
