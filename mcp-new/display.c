#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      display.c
**
** ABSTRACT:
**	Menu and Inst
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
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
/******************************************************************************/
/*
Copyright (C) Fermi National Accelerator Laboratory
Filename: display.c
Revision: 1.1
Date and Time: 04/04/96 13:46:57
*/
/******************************************************************************
display.c
by Charlie Briegel 
Origination 4/22/98 

Routines to control the display screen for Menu
These functions can be used with a Vt220 terminal to position the cursor,
erase the screen etc.
******************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "display.h"
#include "stdio.h"
#include "tickLib.h"
#include "time.h"
#include "ioLib.h"
#include "semLib.h"
#include "string.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "taskLib.h"
#include "gendefs.h"
#include "ad12f1lb.h"
#include "axis.h"
#include "frame.h"
#include "tm.h"
#include "cw.h"
#include "io.h"
#include "data_collection.h"
/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
#define SoftwareVersion_	15
static int STOPed[6]={TRUE,TRUE,TRUE,TRUE,TRUE,TRUE};
static int last_clamp;
static int last_door1;
static int last_door2;
static int last_azbrake;
static int last_altbrake;
static int last_ffs;
static int last_ffl;
static int last_ffc;
static int refreshing=FALSE;
static char *limitstatus[]={"LU","L "," U","  "};
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
int question_mark=0;
int Axis=4;
long adjpos[6]={0,0,0,0,0,0};
long adjvel[6]={0,0,0,0,0,0};
long adjacc[6]={70000,10000,10000,10000,10000,10000};
long incvel[6]={1000,0,1000,0,1000,0};
long Axis_vel[6]={0,0,0,0,0,0};
long Axis_vel_neg[6]={-700000,0,-600000,0,-250000,0};
long Axis_vel_pos[6]={700000,0,600000,0,250000,0};
/* 8863 is pinned at .9 degrees; -9471 is zenith 90 degrees before */
/* 8857 is pinned at 0 degrees; -9504 is zenith 90 degrees 22-Aug-98 */
float altclino_sf=.0048683116163;/*.0049016925256;*/
/*.0048598736;*//*.0047368421 90 deg=19000*//*.0049011599*/
int altclino_off=8937;/*8857;*/
/*9048;*/         /*9500*/
char MenuInput[21];

/* prototypes */
void Menu();
static void PrintMenuBanner();
void PrintMenuMove();
void PrintMenuPos();

void Inst();
static void PrintInstBanner();
void PrintInstPos();
int check_stop_in();
/*=========================================================================
**=========================================================================
**
** ROUTINE: EraseDisplayAll
**	    EraseDisplayRest
**	    CursPos
**	    SaveCursPos
**	    RestoreCursPos
**	    GetCharNoEcho
**	    GetString
**
** DESCRIPTION:
**      Screen management routines for VT220.
**
** RETURN VALUES:
**      int	zero or ERROR
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
/* Terminal Escape sequences */
/*
#define ESC '\x1B'
#define CSI "\x1B["
*/
#define ESC '\033'
#define CSI "\033["
/******************************************************************************/
void EraseDisplayAll()
{
 printf("%s2J",CSI);
}
/******************************************************************************/
void EraseDisplayRest()
/* erases from present cursor to end of display */
{
 printf("%sJ",CSI);
}
/******************************************************************************/
void CursPos(int X,int Y)
{
  char buf[3];
  char buf1[3];

  if(X>80)X=80; if(Y>24)Y=24;
  if(X<0)X=0; if(Y<0)Y=0;
  sprintf(buf,"%d",Y);
  sprintf(buf1,"%d",X);
  printf("%s%s;%sH",CSI,buf,buf1);
}
/******************************************************************************/
void SaveCursPos()
{
  printf("%c7",ESC);
}
/******************************************************************************/
void RestoreCursPos()
{
  printf("%c8",ESC);
}
/******************************************************************************/
char GetCharNoEcho()
/* This function gets a character from the keyboard without the automatic
	echo that the regular scanf, getchar, etc routines have in them.
	It is sometimes desirable to turn off the getchar echo especially when
	one task is updating values on the screen at a particular cursor position
	and another task is getting user input at the same time.
	This routine could be easly expanded to GetStringNoEcho if need be.
*/	 
{
  int Options;
  char ch;

  Options=ioctl(0,FIOGETOPTIONS,0); /* save present keyboard options */
  ioctl(0,FIOOPTIONS,Options & ~OPT_ECHO & ~OPT_LINE);
  ch=getchar();  
  ioctl(0,FIOOPTIONS,Options); /* back to normal */
  return(ch);
}
/******************************************************************************/
int GetString(char *buf, int cnt)
/* This function gets a string from the keyboard without the automatic
	echo that the regular scanf, getchar, etc routines have in them.
	It is sometimes desirable to turn off the getchar echo especially when
	one task is updating values on the screen at a particular cursor position
	and another task is getting user input at the same time.
	This routine could be easly expanded to GetStringNoEcho if need be.
*/	 
{
  extern SEM_ID semMEIUPD;
  int i;

  semGive (semMEIUPD);
  *buf=getchar();
/*	printf ("0x%x",*buf);*/
  i=1;
  while ((*buf!=0xA)&&(i<cnt))
  {
    if (*buf==0x1b)
    {
      memset(&MenuInput[0],' ',20);
      semTake (semMEIUPD,60);
      return (FALSE);
    }
    if (*buf!=0x8)
    {
      i++; 
      buf++;
    }
    else
    {
      if (i>0)
      {
        i--; 
        buf--;
      }
      *buf=' ';
    }
    taskDelay (10);
    *buf=getchar();
  }
  *buf=NULL;  /* terminate the string overwrite the LF */
  semTake (semMEIUPD,60);
  if (i==1) return (FALSE);
  return(TRUE);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: TCC_check
**
** DESCRIPTION:
**      Check if TCC has active queue entries and print messages for each axis
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	axis_queue[]
**
**=========================================================================
*/
void TCC_check()
{
  extern struct FRAME_QUEUE axis_queue[];
  int i;

  for (i=0;i<3;i++)
    if (axis_queue[i].active!=NULL)
    {
      printf ("\r\n Axis %d: TCC is has active entries in queue",i);
      taskDelay (2*60);
    }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: PrintMenuBanner
**
** DESCRIPTION:
**      Print Menu Banner and initialize variables
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
static void PrintMenuBanner()
{
  CursPos(1,1);
  MenuInput[21]=NULL;
  last_clamp=last_door1=last_door2=last_azbrake=last_altbrake=-1;
  EraseDisplayRest();
  printf("     /////// ///////   ///////  ///////   Sloan Digital Sky Survey  Version: %d\n",SoftwareVersion_); 
  printf("    //       //   //  //       //           software by Charlie Briegel        \n");
  printf("   //////   //   //  ///////  ///////     Compiled: %s %s\n",__DATE__, __TIME__);
  printf("      //   //   //       //       //      Tag: %-20s            \n", "$Name$");
  printf("     //   //   //       //       //                                            \n");
  printf("//////  ///////    //////   //////                                             \n");  
  printf("Time Since Boot:       Days               Date:     \n");
  CursPos(1,8);
  printf ("    MSA DegMinSecMAS   ActualPos      CmdPos  VoltageOut  Fiducial;Pos\n\r");
  CursPos(1,9);    
  printf("AZ\n\r");
  CursPos(1,10);    
  printf("AL\n\r");
  CursPos(1,11);    
  printf("ROT\n\r");
  CursPos(1,12);    
  printf("Alt Clinomator=       degrees\n\r");
  CursPos(1,13);    
  printf("ALIGN Clamp            SP1 Slit                     SP2 Slit                 \n\r");
  CursPos(1,14);    
  printf("AZ Brake\n\r");
  CursPos(1,15);    
  printf("ALT Brake\n\r");
  CursPos(1,16);    
  printf("CW1\tCW2\tCW3\tCW4\n\r");
  CursPos(1,19);
  if (Axis==0) printf("Azimuth Controls  ");
  if (Axis==2) printf("Altitude Controls ");
  if (Axis/2==2) printf("Rotator Controls  ");
  printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",Axis_vel[Axis],incvel[Axis]);
  CursPos(1,22);
  printf("/////////////////////////////// Menu ////////////////////////////////\n\r");
  printf("R=Rotator Z=aZimuth L=aLtitude S=Stop H=Hold ?=help X=eXit......Command->\n\r");
  if (taskIdFigure("menuPos")==ERROR)
    taskSpawn("menuPos",99,VX_FP_TASK,8000,(FUNCPTR)PrintMenuPos,
	0,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: PrintMenuMove
**
** DESCRIPTION:
**      Print Menu Move display is in the lower screen and changes for
** either the azimuth, altitude, or rotator.  This screen is associated
** with motion to a specified position and velocity.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void PrintMenuMove()
{
  extern double sec_per_tick[];
  double arcsecond;
  long marcs,arcs,arcm,arcd;

  arcsecond=(sec_per_tick[Axis>>1]*abs(adjpos[Axis]));
  arcd=(long)(arcsecond)/3600;	     
  arcm=((long)(arcsecond)-(arcd*3600))/60;
  arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
  marcs = (arcsecond-(long)arcsecond)*1000;
  CursPos(18,20);
  printf("Move to  Position ");
  if (adjpos[Axis]<0)
    printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
	arcd,arcm,arcs,marcs,adjpos[Axis]);
  else
    printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
	arcd,arcm,arcs,marcs,adjpos[Axis]);
  CursPos(18,21);
  printf("   Velocity ");
  printf ("%10ld Cts/Sec",adjvel[Axis]);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: Menu
**
** DESCRIPTION:
**      This is the Menu which is used as a control and status display of
**	the telescope.  This is considered to be an engineering tool.
**	Refer to the help '?' for list of items to choose from.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void Menu()
/* This services the display terminal.  */
{
  int Running=TRUE;
  char buf[255];
  extern struct SDSS_FRAME sdssdc;
  extern struct TM_M68K *tmaxis[];
  extern SEM_ID semMEI;
  extern SEM_ID semMEIUPD;
  extern void tm_az_brake_on(),tm_az_brake_off();
  extern void tm_alt_brake_on(),tm_alt_brake_off();
  extern void tm_set_pos(int axis, int pos);
  extern void manTrg();
  extern int cw_abort();
  extern struct FIDUCIARY fiducial[3];
  extern long fiducial_position[3];
  extern double sec_per_tick[];
  extern int monitor_on[3];
  int cwpos;
  int cw;
  int inst;
  int Options;
  long pos, deg, min, arcsec, marcsec;
  double arcsecond;
  long marcs,arcs,arcm,arcd;
  int negative;
  time_t fidtim;
  extern FILE *fidfp;

  TCC_check();
  Options=ioctl(0,FIOGETOPTIONS,0); /* save present keyboard options */
  ioctl(0,FIOOPTIONS,Options & ~OPT_ECHO & ~OPT_LINE);
  if (semTake (semMEIUPD,60)!=ERROR)
  {
    refreshing=TRUE;
    PrintMenuBanner();
    PrintMenuMove();
    semGive (semMEIUPD);
  }
  else
  {
    printf ("\r\nCan't take semMEIUPD...DataCollection task probably at fault");
    return;
  }
  while(Running)
  {
    buf[0]=getchar();
 /*     gets(buf);  */
    if (semTake (semMEIUPD,60)!=ERROR)
    {
     switch(buf[0])
     {
       case 'Z': case 'z': CursPos(1,19);
         printf("Azimuth Controls  ");
         Axis=0;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
         Axis_vel[Axis],incvel[Axis]);
         PrintMenuMove();
         break;

       case 'L': case 'l': CursPos(1,19);
         printf("Altitude Controls ");
         Axis=2;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
         Axis_vel[Axis],incvel[Axis]);
         PrintMenuMove();
         break;

       case 'R': case 'r': CursPos(1,19);
         printf("Rotator Controls  ");
         Axis=4;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
         Axis_vel[Axis],incvel[Axis]);
         PrintMenuMove();
         break;

       case 'P': case 'p': CursPos(20,24);
         printf("Set Position    xxx:xx:xx:xxx           ");
         if (GetString(&MenuInput[0],20))
         {
           memcpy(&buf[0],&MenuInput[0],21);
           memset(&MenuInput[0],' ',20);
           min=0; arcsec=0; marcsec=0;
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
	   pos=(long)((abs(deg)*3600000.)+(min*60000.)+
	     (arcsec*1000.)+marcsec)/(sec_per_tick[Axis>>1]*1000);
           if (negative) pos = -pos;
           switch (Axis>>1)
           {
             case AZIMUTH:
               tm_set_pos(Axis,pos);
	       tm_set_pos(Axis+1,pos);
               fiducial[Axis/2].markvalid=FALSE;
               break;

             case ALTITUDE:
               tm_set_pos(Axis,pos);
	       tm_set_pos(Axis+1,pos);
               fiducial[Axis/2].markvalid=FALSE;
               break;

             case INSTRUMENT:
               tm_set_pos(Axis,pos);
	       tm_set_pos(Axis+1,pos);
               fiducial[Axis/2].markvalid=FALSE;
               break;

             default:
               break;
	   }
         }
         CursPos(20,24);
         printf("                                        ");
         break;

       case 'F': case 'f': CursPos(20,24);
     	 printf("Set Fiducial Position                    ");
	 if (fiducial[Axis/2].markvalid)
	 {
/*	   pos=fiducial_position[Axis/2];*/
/*         pos += ((*tmaxis[Axis/2]).actual_position-fiducial[Axis/2].mark);*/
/* use optical encoder for axis 4 */
           pos = (*tmaxis[Axis/2]).actual_position+
	     (fiducial_position[Axis/2]-fiducial[Axis/2].mark);
           switch (Axis>>1)
           {
             case INSTRUMENT:
	       tm_set_pos(Axis+1,pos);
               tm_set_pos(Axis,pos);
	       break;

             case ALTITUDE:
	       tm_set_pos(Axis&0x6,pos);
	       tm_set_pos(Axis+1,pos);
	       break;

             case AZIMUTH:
	       tm_set_pos(Axis&0x6,pos);
	       tm_set_pos(Axis+1,pos);
	       break;

             default:
               break;
           }
           if (fidfp!=NULL)
	   {
	     time (&fidtim);
             fprintf (fidfp,"\n#Menu %d\t%d\t%.25s:%lf\t%ld\t%ld",
	          Axis,fiducial[Axis/2].index,
	          ctime(&fidtim),sdss_get_time(),
	          (long)fiducial_position[Axis/2]-fiducial[Axis/2].mark,
                  (long)(*tmaxis[Axis/2]).actual_position);
           }
	   fiducial[Axis/2].mark=fiducial_position[Axis/2];
	 }
	 else
           printf("ERR: fiducial for axis not crossed      ");
	 break;

       case 'D': case 'd': CursPos(20,24);
	 printf("Dest.  Position xxx:xx:xx:xxx           ");
	 if (GetString(&MenuInput[0],20))
	 {
	   min=0;
	   arcsec=0;
	   marcsec=0;
	   memcpy(&buf[0],&MenuInput[0],21);
	   memset(&MenuInput[0],' ',20);
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
	   adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
	     (arcsec*1000.)+marcsec)/(sec_per_tick[Axis>>1]*1000);
	   if (negative) adjpos[Axis] = -adjpos[Axis];
	   CursPos(36,20);
	   if (adjpos[Axis]<0)
	     printf("-%03d:%02ld:%02ld:%03ld %10ld Cts",
		abs(deg),min,arcsec,marcsec,adjpos[Axis]);
     	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		deg,min,arcsec,marcsec,adjpos[Axis]);
	 }
	 CursPos(20,24);
	 printf("                                        ");
	 break;

       case 'O': case 'o': CursPos(20,24);
	 printf("Offset Position xxx:xx:xx:xxx           ");
	 if (GetString(&MenuInput[0],20))
	 {
	   memcpy(&buf[0],&MenuInput[0],21);
	   memset(&MenuInput[0],' ',20);
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
	   adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
	     (arcsec*1000.)+marcsec)/(sec_per_tick[Axis>>1]*1000);
	   if (negative) adjpos[Axis] = -adjpos[Axis];
           adjpos[Axis] += (*tmaxis[Axis/2]).actual_position;
	   arcsecond=(sec_per_tick[Axis>>1]*abs(adjpos[Axis]));
	   arcd=(long)(arcsecond)/3600;	     
	   arcm=((long)(arcsecond)-(arcd*3600))/60;
	   arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
	   marcs = (arcsecond-(long)arcsecond)*1000;
	   CursPos(36,20);
	   if (adjpos[Axis]<0)
	     printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[Axis]);
	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[Axis]);
	 }
	 CursPos(20,24);
	 printf("                                        ");
	 break;

       case 'A': case 'a': CursPos(20,24);
	 printf("AdjCnt Position dddddd                  ");
	 if (GetString(&MenuInput[0],20))
	 {
	   memcpy(&buf[0],&MenuInput[0],21);
	   memset(&MenuInput[0],' ',20);
	   sscanf (buf,"%ld",&adjpos[Axis]);
           adjpos[Axis] += (*tmaxis[Axis/2]).actual_position;
	   arcsecond=(sec_per_tick[Axis>>1]*abs(adjpos[Axis]));
	   arcd=(long)(arcsecond)/3600;	     
	   arcm=((long)(arcsecond)-(arcd*3600))/60;
	   arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
	   marcs = (arcsecond-(long)arcsecond)*1000;
	   CursPos(36,20);
	   if (adjpos[Axis]<0)
	     printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[Axis]);
	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[Axis]);
	 }
	 CursPos(20,24);
	 printf("                                        ");
	 break;

       case 'V': case 'v': CursPos(20,24);
	 printf("   Set Velocity                         ");
	 if (GetString(&MenuInput[0],20))
	 {
	   memcpy(&buf[0],&MenuInput[0],21);
	   memset(&MenuInput[0],' ',20);
	   sscanf (buf,"%ld",&adjvel[Axis]);
	   adjvel[Axis]=abs(adjvel[Axis]);
	   if (adjvel[Axis]>Axis_vel_pos[Axis]) 
	     adjvel[Axis]=Axis_vel_pos[Axis];
	   CursPos(30,21);
	   printf ("%10ld Cts/Sec",adjvel[Axis]);
	 }
	 CursPos(20,24);
	 printf("                                        ");
	 break;

         case 'I': case 'i': CursPos(20,24);
 	   printf("   Set Velocity Increment               ");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%ld",&incvel[Axis]);
	     incvel[Axis]=abs(incvel[Axis]);
	     if (incvel[Axis]>10000) incvel[Axis]=10000;
	     CursPos(49,19);
	     printf ("%ld Cts",incvel[Axis]);
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case 'T': case 't': CursPos(20,24);
	   printf("Manual Trigger                          ");
	   manTrg();
	   break;

         case 'B': case 'b': CursPos(20,24);
	   if (Axis==0)
	   {
	     printf("Azimuth Brake Turned On                 ");
	     tm_sp_az_brake_on();
	   }
	   if (Axis==2)
	   {
	     printf("Altitude Brake Turned On                ");
	     tm_sp_alt_brake_on();
	   }
           STOPed[Axis]=TRUE;
           tm_controller_idle (Axis);
           tm_reset_integrator(Axis);
	   break;

         case 'C': case 'c': CursPos(20,24);
	   if (Axis==0)
	   {
	     printf("Azimuth Brake Turned Off                ");
	     tm_sp_az_brake_off();
	   }
	   if (Axis==2)
	   {
	     printf("Altitude Brake Turned Off               ");
	     tm_sp_alt_brake_off();
	   }
           Axis_vel[Axis]=0;
           CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
           if (semTake (semMEI,60)!=ERROR)
	   {
             if (STOPed[Axis])
             {
               STOPed[Axis]=FALSE;
               sem_controller_run (Axis);
               if (Axis==4)
               while (coeffs_state_cts (Axis,0));
                 v_move(Axis,(double)0,(double)5000);
             }
             else
             {
               set_velocity(Axis,(double)(Axis_vel[Axis]));
               if (Axis==4)
               while (coeffs_state_cts (Axis,(long)Axis_vel[Axis]));
             }
             semGive (semMEI);
           }
           else
           {
	     CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	   }
 	   break;

         case '+': case '=':
	   CursPos(20,24);
	   printf("ALIGNment Clamp Turned On               ");
	   tm_clamp_on();
	   break;

         case '_': case '-':
           CursPos(20,24);
	   printf("ALIGNment Clamp Turned Off              ");
	   tm_clamp_off();
	   break;

         case '(':
	   CursPos(20,24);
	   printf("SP1 Slit Door Toggled          ");
	   if ((sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (!sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     tm_sp_slit_close(SPECTOGRAPH1);
	     break;
	   }
	   if ((!sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     tm_sp_slit_open(SPECTOGRAPH1);
	     break;
	   }
	   tm_sp_slit_open(SPECTOGRAPH1);
	   printf("ERR: Inconsistent State  ");
	   break;

         case ')':
	   CursPos(20,24);
	   printf("SP2 Slit Door Toggled           ");
	   if ((sdssdc.status.o1.ol9.slit_dr2_opn_perm)&&
	       (!sdssdc.status.i1.il9.slit_head_door2_cls))
	   {
	     tm_sp_slit_close(SPECTOGRAPH2);
	     break;
	   }
	   if ((!sdssdc.status.o1.ol9.slit_dr2_opn_perm)&&
	       (sdssdc.status.i1.il9.slit_head_door2_cls))
	   {
	     tm_sp_slit_open(SPECTOGRAPH2);
	     break;
	   }
	   tm_sp_slit_open(SPECTOGRAPH2);
	   printf("ERR: Inconsistent State  ");
	   break;

         case '{':
	   CursPos(20,24);
	   printf("SP1 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch1_opn_perm)
	     tm_cart_unlatch(SPECTOGRAPH1);
	   else
	     tm_cart_latch(SPECTOGRAPH1);
	   break;

         case '}':
	   CursPos(20,24);
	   printf("SP2 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch2_opn_perm)
	     tm_cart_unlatch(SPECTOGRAPH2);
	   else
	     tm_cart_latch(SPECTOGRAPH2);
	   break;

         case '~':
	   CursPos(20,24);
	   printf("Flat Field Screen Toggled ");
	   if (sdssdc.status.o1.ol14.ff_screen_open_pmt)
	   {
	     printf("Close ");
	     tm_sp_ffs_close();
	   }
	   else
	   {
	     printf("Open  ");
	     tm_sp_ffs_open();
	   }
	   break;

         case '|':
	   CursPos(20,24);
	   printf("Flat Field Lamps Toggled ");
	   if (sdssdc.status.o1.ol14.ff_lamps_on_pmt)
	   {
	     printf("Off   ");
	     tm_ffl_off();
	   }
	   else
	   {
	     printf("On    ");
	     tm_ffl_on();
	   }
	   break;

         case '"':
	   CursPos(20,24);
	   printf("Flat Field Neon Toggled ");
	   if (sdssdc.status.o1.ol14.ne_lamps_on_pmt)
	   {
	     printf("Off     ");
	     tm_neon_off();
	   }
	   else
	   {
	     printf("On     ");
	     tm_neon_on();
	   }
	   break;

         case ':':
	   CursPos(20,24);
	   printf("Flat Field HgCd Toggled ");
	   if (sdssdc.status.o1.ol14.hgcd_lamps_on_pmt)
	   {
	     printf("Off    ");
	     tm_hgcd_off();
	   }
	   else
	   {
	     printf("On     ");
	     tm_hgcd_on();
	   }
	   break;

         case 'X': case 'x':
	   Running=FALSE;
	   ioctl(0,FIOOPTIONS,Options); /* back to normal */
           taskDelete(taskIdFigure("menuPos"));
	   taskDelay (10);
	   EraseDisplayAll();
	   break;

         case 'W': case 'w':
	   CursPos(20,24);
	   printf("dd|c..c; 2=EMPTY;3=SCF;4=S;5=SC;6=SE;7=SEC;8=SI");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     if ((buf[0]>='0')&&(buf[0]<='9'))
	     {
	       sscanf (buf,"%d",&inst);
	       if ((inst<0)||(inst>16)) 
	       {
	         printf("ERR: Inst Out of Range (0-16)           ");
	         break;
	       }
             }
             else
             {
               inst=cw_get_inst (&buf[0]);
	       if (inst==ERROR)
               {
                 printf("ERR: Inst Name Incorrect               ");
                 break;
               }
             }
	     CursPos(20,24);
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
			  (int)inst,0,0,0,0,0,0,0,0,0);
	     else
	     {
	       printf("ERR: Altitude Brake NOT Engaged         ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '!': case '@': case '#': case '$':
	   CursPos(20,24);
	   if (buf[0]=='@') cw = 2;
	   else cw = buf[0]-0x20;
	   printf("CW %d vvv                              ",cw);
	   cw--;
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%d",&cwpos);
	     CursPos(20,24);
	     if ((cwpos<10)||(cwpos>800))
	     {
	       printf("ERR: Position out of Range (10-800)    ");
	       break;
	     }
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cwp",60,VX_FP_TASK,4000,(FUNCPTR)cw_positionv,
			  (int)cw,(int)cwpos,0,0,0,0,0,0,0,0);
	     else
             {
	       printf ("Alt Brake NOT Engaged                  ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '^': 
	   CursPos(20,24);
	   printf("All CW vvv                             ");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%d",&cwpos);
	     CursPos(20,24);
	     if ((cwpos<10)||(cwpos>800))
	     {
	       printf("ERR: Position out of Range (10-800)    ");
	       break;
	     }
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     cw_set_positionv(INST_DEFAULT,cwpos,cwpos,cwpos,cwpos);
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
			  (int)INST_DEFAULT,0,0,0,0,0,0,0,0,0);
	     else
	     {
	       printf("ERR: Altitude Brake NOT Engaged         ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '%':
	   CursPos(20,24);
           printf ("CW ABORT                               ");
           taskDelete(taskIdFigure("cw"));
           taskDelete(taskIdFigure("cwp"));
	   cw_abort();
	   break;

         case '&':
	   CursPos(20,24);
           printf ("Toggle the axis monitor_on to ");
	   if (monitor_on[Axis/2]) 
	   {
	     monitor_on[Axis/2]=FALSE;
             printf ("FALSE ");
	   }
           else 
	   {
	     monitor_on[Axis/2]=TRUE;
             printf ("TRUE ");
	   }
	   break;

         case '*':
	   CursPos(20,24);
           printf ("AMP RESET                              ");
           if (Axis==4)
	     amp_reset(Axis);
	   else
	   {
	     amp_reset(Axis);
	     amp_reset(Axis+1);
	   }
	   break;

         case 'H': case 'h':
           if (semTake (semMEI,60)!=ERROR)
	   {
	     if (STOPed[Axis])
	     {
	       Axis_vel[Axis]=0;
               STOPed[Axis]=FALSE;
               sem_controller_run (Axis);
               if (Axis==4)
                 while (coeffs_state_cts (Axis,0));
               v_move(Axis,(double)0,(double)5000);
  	     }
  	     else
	     {
               while (abs(Axis_vel[Axis])>5000)
	       {
	         if (Axis_vel[Axis]>0) Axis_vel[Axis]-=5000;
	         else Axis_vel[Axis]+=5000;
 	         set_velocity(Axis,(double)(Axis_vel[Axis]));
                 if (Axis==4)
                   while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
                 CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
	         taskDelay (15);
	       }
	       Axis_vel[Axis]=0;
  	       set_velocity(Axis,(double)(Axis_vel[Axis]));
               if (Axis==4)
                 while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	     }
	     semGive (semMEI);
             CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
           }
           else
	   {
	     CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	   }
	   break;

         case '?': 
	   if (question_mark==0)
	   {
	     CursPos(1,1);
printf("Extended Help1..*=AMP Reset; +|-=Align Close|Open; B|C=Brake Enable|Disable    \n");
	     CursPos(1,2);
printf(" D=Dest Position; O=Offset Position V=Set Velocity; A=AdjCnt Position;        \n");
	     CursPos(1,3);
printf(" M=Move to Position G=Gang Az+Alt Move; S(state)=*(running),Stop,Estop,Abort  \n");
	     CursPos(1,4);
printf(" K=pos; J=neg; S=Stop Motion; H=Hold Motion I=Set Velocity Increment;         \n");
	     CursPos(1,5);
printf(" P=SetPosition; F=SetFiducial; sp=RstScrn X=eXit; A(amp)=*(ok),Stop-in,?(TBD) \n");
	     CursPos(1,6);
printf(" W=Move CW; !|@|#|$=Move CW 1|2|3|4; %%=CW Halt ^=CW All                      \n");
	   }
	   if (question_mark==1)
	   {
	     CursPos(1,1);
printf("Extended Help2..(|)=slit1|slit2 toggle; {|}=slit latch1|slit latch2 toggle    \n");
	     CursPos(1,2);
printf(" |=flat field lamp toggle; \"=Ne lamp toggle; :=HgCd lamp toggle               \n");
	     CursPos(1,3);
printf(" ~=flat field screen toggle                                                   \n");
	     CursPos(1,4);
printf(" &=toggel on/off axis monitor                                                 \n");
	     CursPos(1,5);
printf("MSA is Monitor status,axis State,Amp status(Amp,Brake,EStop)                  \n");
	     CursPos(1,6);
printf("CW Options: 2=EMPTY;3=SCF;4=S;5=SC;6=SE;7=SEC;8=SI                            \n");
	   }
	   question_mark = (question_mark+1)%2;
	   break;
	
	 case 'S': case 's':	     
	   Axis_vel[Axis]=0;
	   CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
           STOPed[Axis]=TRUE;
           if (semTake (semMEI,60)!=ERROR)
	   {
             sem_controller_idle (Axis);
             reset_integrator(Axis);
	     semGive (semMEI); 
           }
           else
           {
	     CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	   }
           break;
	   
         case 'G': case 'g':  CursPos(20,24);
	   if ((adjvel[0]==0)||(adjvel[2]==0))
	   {
             printf("ERR: Velocity is Zero for AZ or Alt Axis");
	     break;
	   }
	   if (sdssdc.status.i9.il0.az_brake_en_stat)
	   {
             CursPos(20,24);
	     printf("ERR: AZ Brake is Engaged                ");
	     break;
	   }
	   if (sdssdc.status.i9.il0.alt_brake_en_stat)
	   {
	     CursPos(20,24);
	     printf("ERR: ALT Brake is Engaged               ");
	     break;
	   }
           if (semTake (semMEI,60)!=ERROR)
	   {
             if (STOPed[0])
             {
               STOPed[0]=FALSE;
               sem_controller_run (0);
             }
             if (STOPed[2])
             {
               STOPed[2]=FALSE;
               sem_controller_run (2);
             }
	     Axis_vel[0]=0;
	     Axis_vel[2]=0;
	     start_move(0,(double)(adjpos[0]),
		(double)adjvel[0],(double)adjacc[0]);
             start_move(2,(double)(adjpos[2]),
		(double)adjvel[2],(double)adjacc[2]);
	     semGive (semMEI); 
           }
           else
           {
	     CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	   }
	   break;

         case 'M': case 'm':  CursPos(20,24);
	   if (adjvel[Axis]==0)
	   {
                  printf("ERR: Velocity is Zero                   ");
	     break;
	   }
           if (STOPed[Axis])
           {
	     if ((Axis==0)&&(sdssdc.status.i9.il0.az_brake_en_stat))
	     {
	       CursPos(20,24);
	       printf("ERR: AZ Brake is Engaged                ");
	       break;
	     }
	     if ((Axis==2)&&(sdssdc.status.i9.il0.alt_brake_en_stat))
	     {
	       CursPos(20,24);
	       printf("ERR: ALT Brake is Engaged               ");
	       break;
	     }
	     if (Axis==4)
	     {
               while (coeffs_state_cts(Axis,(long)adjvel[Axis]));
	     }
             STOPed[Axis]=FALSE;
             if (semTake (semMEI,60)!=ERROR)
	     {
               sem_controller_run (Axis);
  	       Axis_vel[Axis]=0;
	       start_move(Axis,(double)(adjpos[Axis]),
		 (double)adjvel[Axis],(double)adjacc[Axis]);
	       semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	     }
           }
           else
           {
             if (semTake (semMEI,60)!=ERROR)
	     {
	       if (Axis==4)
	       {
                 while (coeffs_state_cts(Axis,(long)adjvel[Axis]));
	       }
               Axis_vel[Axis]=0;
	       start_move(Axis,(double)(adjpos[Axis]),
		 (double)adjvel[Axis],(double)adjacc[Axis]);
	       semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	     }
           }
	   break;

         case 'J': case 'j':
	   Axis_vel[Axis] -= incvel[Axis];
           if (Axis_vel[Axis]<Axis_vel_neg[Axis]) Axis_vel[Axis]=Axis_vel_neg[Axis];
	   CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
           if (STOPed[Axis])
           {
             if ((Axis==0)&&(sdssdc.status.i9.il0.az_brake_en_stat)&&
		(Axis_vel[Axis]!=0))
             {
               CursPos(20,24);
	       printf("ERR: AZ Brake is Engaged                ");
	       break;
             }
             if ((Axis==2)&&(sdssdc.status.i9.il0.alt_brake_en_stat)&&
		(Axis_vel[Axis]!=0))
             {
	       CursPos(20,24);
	       printf("ERR: ALT Brake is Engaged               ");
	       break;
	     }
             STOPed[Axis]=FALSE;
             if (semTake (semMEI,60)!=ERROR)
	     {
               sem_controller_run (Axis);
	       v_move(Axis,(double)0,(double)5000);
               taskDelay (10);
	       if (Axis==4)
	       {
                 while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity(Axis,(double)(Axis_vel[Axis]));
               semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	     }
           }
           else
           {
             if (semTake (semMEI,60)!=ERROR)
	     {
	       if (Axis==4)
	       {
                 while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis])); 
               semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	     }
           }
	   break;

         case 'K': case 'k':
	   Axis_vel[Axis] += incvel[Axis];
           if (Axis_vel[Axis]>Axis_vel_pos[Axis]) Axis_vel[Axis]=Axis_vel_pos[Axis];
	   CursPos(19,19);  printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
           if (STOPed[Axis])
           {
             if ((Axis==0)&&(sdssdc.status.i9.il0.az_brake_en_stat))
             {
               CursPos(20,24);
	       printf("ERR: AZ Brake is Engaged");
	       break;
             }
             if ((Axis==2)&&(sdssdc.status.i9.il0.alt_brake_en_stat))
             {
               CursPos(20,24);
	       printf("ERR: ALT Brake is Engaged");
	       break;
             }
             STOPed[Axis]=FALSE;
             if (semTake (semMEI,60)!=ERROR)
	     {
               sem_controller_run (Axis);
               v_move(Axis,(double)0,(double)5000);
               taskDelay(10);
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis]));
 	       semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	     }
           }
           else
           {
             if (semTake (semMEI,60)!=ERROR)
	     {
	       if (Axis==4)
	       {
                 while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis]));
 	       semGive (semMEI); 
             }
             else
             {
	       CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
             }
           }
	   break;
	   
         default:  
	   refreshing=TRUE;
 	   PrintMenuBanner(); PrintMenuMove();
/*
	   printf ("Illegal Char Input=%x, %x, %x\n",
		buf[0],buf[1],buf[2]);
*/
	   break;
      }
      semGive (semMEIUPD);
     }
     else
       printf ("\r\nIgnored input...Can't take semMEIUPD...DataCollection task probably at fault");
     taskDelay(5); /* 60/5 = 12Hz */
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: PrintMenuPos
**
** DESCRIPTION:
**      Updates the Menu display parameters as a stand-alone task.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void PrintMenuPos()
{
  extern struct SDSS_FRAME sdssdc;
  extern SEM_ID semMEIUPD;
  extern SEM_ID semMEI;
  extern int rawtick;
  extern struct TM_M68K *tmaxis[];
  extern struct FIDUCIARY fiducial[3];
  extern int fiducialidx[3];
  extern struct FIDUCIALS az_fiducial[];
  extern struct FIDUCIALS alt_fiducial[];
  extern struct FIDUCIALS rot_fiducial[];
  extern char *get_date();
  extern double sec_per_tick[];
  int state;
  int fidsign;
  int i;
  long ap,cp,ap1,ap2,vlt;
  double arcsec, farcsec;
  int lasttick;
  long marcs,arcs,arcm,arcd;
  short adc;
  int limidx;
  extern unsigned char cwLimit;
  extern int monitor_on[3];
	
  lasttick=0;
  FOREVER
  {  
    if (refreshing) 
    {
      taskDelay(60);
      refreshing=FALSE;
    }
    if (semTake (semMEIUPD,60)!=ERROR)
    {
      if (rawtick>(50+lasttick))
      {
        lasttick=rawtick;
        CursPos(17,7);
        printf("%5.2f\n",((double)rawtick)/(216000.0*24.0));
        CursPos(49,7);
        printf("%s",(char *)get_date());
      }
      for(i = 0; i < 3; i++)
      {
        CursPos(5,9+i);
        ap=(*tmaxis[i]).actual_position;
        cp=(*tmaxis[i]).position,
        vlt=(*tmaxis[i]).voltage;
	if (monitor_on[i]) printf ("*");
	else printf ("U");
        if (semTake (semMEI,NO_WAIT)!=ERROR)
        {
	  state=axis_state(i<<1);
	  semGive(semMEI);
	  switch (state)
          {
	    case NO_EVENT: case 1: case NEW_FRAME: printf ("*");
	     break;
	    case STOP_EVENT: printf ("S");
	     break;
	    case E_STOP_EVENT: printf ("E");
	     break;
	    case ABORT_EVENT: printf ("A");
	     break;
	    default: printf ("?");
	  }
	}
        arcsec=(sec_per_tick[i]*abs(ap));
	switch (i)
	{
          case AZIMUTH:
            farcsec=(sec_per_tick[i]*abs(az_fiducial[fiducialidx[i]].mark));
	    if (az_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
              else if (sdssdc.status.i9.il0.az_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (az_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case ALTITUDE:
            farcsec=(sec_per_tick[i]*abs(alt_fiducial[fiducialidx[i]].mark));
	    if (alt_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
              else if (sdssdc.status.i9.il0.alt_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (alt_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case INSTRUMENT:
            farcsec=(sec_per_tick[i]*abs(rot_fiducial[fiducialidx[i]].mark));
	    if (rot_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
	        else printf ("?");
	    }
            if (rot_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
	    break;

	  default:
	    arcsec=farcsec=0.;
            fidsign=0;
	    break;
        }
	if (fidsign!=0)
        {
          CursPos(8,9+i);
          arcd=(long)(arcsec)/3600;	     
          arcm=((long)(arcsec)-(arcd*3600))/60;	     
          arcs=((long)(arcsec)-(arcd*3600)-(arcm*60));	     
          marcs = (arcsec-(long)arcsec)*1000;
          if (ap<0)
            printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
          else
            printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
          printf(" %10ld  %10ld  %10ld",ap,cp,vlt);
          arcd=(long)(farcsec)/3600;	     
          arcm=((long)(farcsec)-(arcd*3600))/60;	     
          arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
          marcs = (farcsec-(long)farcsec)*1000;
          if (fiducialidx[i]!=-1)
          {
            if (fiducial[i].markvalid) printf("  V");
            else printf("   ");
            if (fidsign<0)
              printf("%3d;-%03ld:%02ld:%02ld:%03ld\n",
	        fiducialidx[i],arcd,arcm,arcs,marcs);
            else
              printf("%3d; %03ld:%02ld:%02ld:%03ld\n",
	        fiducialidx[i],arcd,arcm,arcs,marcs);
          }
          else
            printf("  No Crossing\n");
        }
      }
      printf("\t\t%4.2f",
        abs(sdssdc.status.i4.alt_position-altclino_off)*altclino_sf);
      ap=(*tmaxis[0]).actual_position2;
      ap1=(*tmaxis[1]).actual_position2;
      ap2=(*tmaxis[2]).actual_position2;
      printf("\t\tE1=%10ld\tE3=%10ld\tE5=%10ld\n",
        ap,
        ap1,
        ap2);
      semGive (semMEIUPD);
    }
    taskDelay(30);
    if (semTake (semMEIUPD,60)!=ERROR)
    {
      if (last_clamp!=(int)((sdssdc.status.i9.il0.clamp_en_stat)|
		      (sdssdc.status.i9.il0.clamp_dis_stat<<1)|
		      (sdssdc.status.o11.ol0.clamp_en<<2)|
		      (sdssdc.status.o11.ol0.clamp_dis<<3)) )
      {
      CursPos(14,13);
      printf("          ");
      CursPos(14,13);
      if (sdssdc.status.i9.il0.clamp_en_stat)
        printf("On  ");
      if (sdssdc.status.i9.il0.clamp_dis_stat)
        printf("Off ");
      if (sdssdc.status.o11.ol0.clamp_en)
        printf("OnCmd ");
      if (sdssdc.status.o11.ol0.clamp_dis)
        printf("OffCmd ");
      last_clamp=(sdssdc.status.i9.il0.clamp_en_stat)|
		      (sdssdc.status.i9.il0.clamp_dis_stat<<1)|
		      (sdssdc.status.o11.ol0.clamp_en<<2)|
		      (sdssdc.status.o11.ol0.clamp_dis<<3);
      }

      if (last_door1!=(int)((sdssdc.status.i1.il9.slit_head_door1_opn)|
		      (sdssdc.status.i1.il9.slit_head_door1_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch1_opn_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch1_opn<<2))*/ 
								)
      {
      CursPos(33,13);
      printf("                  ");
      CursPos(33,13);
      if (sdssdc.status.i1.il9.slit_head_door1_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door1_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch1_opn_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch1_opn)*/
        printf("LatchCmd   ");
      else
        printf("UnLatchCmd ");
      last_door1=(sdssdc.status.i1.il9.slit_head_door1_opn)|
		      (sdssdc.status.i1.il9.slit_head_door1_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch1_opn_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch1_opn<<2);*/ 
      }
      if (last_door2!=(int)((sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_opn_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch2_opn<<2))*/ 
		      						)
      {
      CursPos(62,13);
      printf("                  ");
      CursPos(62,13);
      if (sdssdc.status.i1.il9.slit_head_door2_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door2_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch2_opn_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch2_opn)*/
        printf("LatchCmd ");
      else
        printf("UnLatchCmd ");
      last_door2=(sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_opn_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch2_opn<<2);*/ 
      }
      if (last_azbrake!=(int)((sdssdc.status.i9.il0.az_brake_en_stat)|
		      (sdssdc.status.i9.il0.az_brake_dis_stat<<1)|
		      (sdssdc.status.o12.ol0.az_brake_en<<2)|
		      (sdssdc.status.o12.ol0.az_brake_dis<<3)) )
      {
      CursPos(14,14);
      printf("                 ");
      CursPos(14,14);
      if (sdssdc.status.i9.il0.az_brake_en_stat)
        printf("On  ");
      if (sdssdc.status.i9.il0.az_brake_dis_stat)
        printf("Off ");
      if (sdssdc.status.o12.ol0.az_brake_en)
        printf("OnCmd ");
      if (sdssdc.status.o12.ol0.az_brake_dis)
        printf("OffCmd ");
      last_azbrake=(sdssdc.status.i9.il0.az_brake_en_stat)|
		      (sdssdc.status.i9.il0.az_brake_dis_stat<<1)|
		      (sdssdc.status.o12.ol0.az_brake_en<<2)|
		      (sdssdc.status.o12.ol0.az_brake_dis<<3);
      }
      if (last_altbrake!=(int)((sdssdc.status.i9.il0.alt_brake_en_stat)|
		      (sdssdc.status.i9.il0.alt_brake_dis_stat<<1)|
		      (sdssdc.status.o12.ol0.alt_brake_en<<2)|
		      (sdssdc.status.o12.ol0.alt_brake_dis<<3)) )
      {
      CursPos(14,15);
      printf("                 ");
      CursPos(14,15);
      if (sdssdc.status.i9.il0.alt_brake_en_stat)
        printf("On  ");
      if (sdssdc.status.i9.il0.alt_brake_dis_stat)
        printf("Off ");
      if (sdssdc.status.o12.ol0.alt_brake_en)
        printf("OnCmd ");
      if (sdssdc.status.o12.ol0.alt_brake_dis)
        printf("OffCmd ");
      last_altbrake=(sdssdc.status.i9.il0.alt_brake_en_stat)|
		      (sdssdc.status.i9.il0.alt_brake_dis_stat<<1)|
		      (sdssdc.status.o12.ol0.alt_brake_en<<2)|
		      (sdssdc.status.o12.ol0.alt_brake_dis<<3);
      }
/*
      printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n",
        sdssdc.status.i9.il0.clamp_en_stat,
        sdssdc.status.i9.il0.clamp_dis_stat,
	sdssdc.status.o11.ol0.clamp_en,
	sdssdc.status.o10.ol0.clamp_dis);
      printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n",
        sdssdc.status.i9.il0.az_brake_en_stat,
	sdssdc.status.i9.il0.az_brake_dis_stat,
	sdssdc.status.o12.ol0.az_brake_en,
	sdssdc.status.o12.ol0.az_brake_dis);
      printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n", 
	sdssdc.status.i9.il0.alt_brake_en_stat,
        sdssdc.status.i9.il0.alt_brake_dis_stat,
        sdssdc.status.o12.ol0.alt_brake_en,
        sdssdc.status.o12.ol0.alt_brake_dis);
*/
      CursPos(1,17);
      for (i=0;i<4;i++)
      {
/*      printf("%d\t",sdssdc.weight[i].pos);*/
	adc=sdssdc.weight[i].pos;
        if ((adc&0x800)==0x800) adc |= 0xF000;
        else adc &= 0xFFF;
	limidx = (cwLimit>>(i*2))&0x3;
        printf ("%d %s\t",
	  (int)((1000*adc)/2048.),limitstatus[limidx]);
      }
      CursPos (60,24);
      printf ("%s",&MenuInput[0]);
      CursPos (74,23);
      semGive (semMEIUPD);
    }
    taskDelay (30);
  }
}
/****************************************************************************/
static void PrintInstBanner()
{
  CursPos(1,1);
  MenuInput[21]=NULL;
  last_clamp=last_door1=last_door2=last_azbrake=last_altbrake=-1;
  last_ffs=last_ffl=last_ffc=-1;
  EraseDisplayRest();
  printf("     /////// ///////   ///////  ///////   Sloan Digital Sky Survey  Version: %d\n",SoftwareVersion_); 
  printf("    //       //   //  //       //           software by Charlie Briegel        \n");
  printf("   //////   //   //  ///////  ///////     Compiled: %s %s\n",__DATE__, __TIME__);
  printf("      //   //   //       //       //                                           \n");
  printf("     //   //   //       //       //                                            \n");
  printf("//////  ///////    //////   //////                                             \n");  
  printf("Time Since Boot:       Days               Date:     \n");
  CursPos(1,8);
  printf ("    MSA DegMinSecMAS   ActualPos      CmdPos  VoltageOut  Fiducial;Pos\n\r");
  CursPos(1,9);    
  printf("AZ\n\r");
  CursPos(1,10);    
  printf("AL\n\r");
  CursPos(1,11);    
  printf("ROT\n\r");
  CursPos(1,12);    
  printf("Alt Clinomator=       degrees\n\r");
  CursPos(1,13);    
  printf("ALIGN Clamp            SP1 Slit                     SP2 Slit                 \n\r");
  CursPos(1,14);    
  printf("Leaf FF 1:OC 2:OC 3:OC 4:OC 5:OC 6:OC 7:OC 8:OC Cmd=Off");
  CursPos(1,15);
  printf("Lamp FF 1234 Cmd=Off; Ne 1234 Cmd=Off; HgCd 1234 Cmd=OFF");
  CursPos(1,16);
  printf("CW1\tCW2\tCW3\tCW4\t\tLiftPos\tLiftForc\n\r");
  CursPos(1,19);
  CursPos(1,22);
  printf("/////////////////////////////// Inst ////////////////////////////////\n\r");
  printf("                                             ?=help X=eXit......Command->\n\r");
  if (taskIdFigure("instPos")==ERROR)
    taskSpawn("instPos",99,VX_FP_TASK,4000,(FUNCPTR)PrintInstPos,
	0,0,0,0,0,0,0,0,0,0);
}
/****************************************************************************/
char *inst_display=NULL;
char *err_display=NULL;
char inst_msg[71]={"INST MSG: "};
int inst_answer=-1;

void Inst()
/* This services the display terminal.  */
{
  int Running=TRUE;
  char buf[255];
  extern struct SDSS_FRAME sdssdc;
  extern SEM_ID semMEIUPD;
  extern void manTrg();
  extern int cw_abort();
  extern int fsm();
  int cwpos;
  int cw;
  int inst;
  int Options;

  Options=ioctl(0,FIOGETOPTIONS,0); /* save present keyboard options */
  ioctl(0,FIOOPTIONS,Options & ~OPT_ECHO & ~OPT_LINE);
  if (semTake (semMEIUPD,60)!=ERROR)
  {
    PrintInstBanner();
    semGive (semMEIUPD);
  }
  else
  {
    printf ("\r\nCan't take semMEIUPD...DataCollection task probably at fault");
    return;
  }
  while(Running)
  {
    buf[0]=getchar();
 /*     gets(buf);  */
    if (semTake (semMEIUPD,60)!=ERROR)
    {
     switch(buf[0])
     {
         case 'Y': case 'y':
	   inst_answer=TRUE;
	   break;
	   
         case 'N': case 'n':
	   inst_answer=FALSE;
	   break;
	   
         case 'G': case 'g':
	   CursPos(20,24);
	   if (sdssdc.status.i9.il0.az_brake_en_stat)
	   {
             CursPos(20,24);
	     printf("ERR: AZ Brake is Engaged                ");
	     break;
	   }
	   if (sdssdc.status.i9.il0.alt_brake_en_stat)
	   {
	     CursPos(20,24);
	     printf("ERR: ALT Brake is Engaged               ");
	     break;
	   }
	   CursPos(20,24);
           printf ("AMP RESETs                             ");
           amp_reset(0);
	   amp_reset(1);
	   amp_reset(2);
	   amp_reset(3);
	   amp_reset(4);
             if (STOPed[0])
             {
               STOPed[0]=FALSE;
               tm_controller_run (0);
             }
             if (STOPed[2])
             {
               STOPed[2]=FALSE;
               tm_controller_run (2);
             }
             if (STOPed[4])
             {
               STOPed[4]=FALSE;
               tm_controller_run (4);
             }
	     Axis_vel[0]=0;
	     Axis_vel[2]=0;
	     Axis_vel[4]=0;
	   tm_move_instchange();
	   break;

         case 'T': case 't': CursPos(20,24);
	   printf("Manual Trigger                          ");
	   manTrg();
	   break;

         case '+': case '=':
	   CursPos(20,24);
	   printf("ALIGNment Clamp Turned On               ");
	   tm_clamp_on();
	   break;

         case '_': case '-':
           CursPos(20,24);
	   printf("ALIGNment Clamp Turned Off              ");
	   tm_clamp_off();
	   break;

         case '(':
	   CursPos(20,24);
	   printf("SP1 Slit Door Toggled          ");
	   if ((sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (!sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     tm_sp_slit_close(SPECTOGRAPH1);
	     break;
	   }
	   if ((!sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     tm_sp_slit_open(SPECTOGRAPH1);
	     break;
	   }
	   tm_sp_slit_open(SPECTOGRAPH1);
	   printf("ERR: Inconsistent State  ");
	   break;

         case ')':
	   CursPos(20,24);
	   printf("SP2 Slit Door Toggled           ");
	   if ((sdssdc.status.o1.ol9.slit_dr2_opn_perm)&&
	       (!sdssdc.status.i1.il9.slit_head_door2_cls))
	   {
	     tm_sp_slit_close(SPECTOGRAPH2);
	     break;
	   }
	   if ((!sdssdc.status.o1.ol9.slit_dr2_opn_perm)&&
	       (sdssdc.status.i1.il9.slit_head_door2_cls))
	   {
	     tm_sp_slit_open(SPECTOGRAPH2);
	     break;
	   }
	   tm_sp_slit_open(SPECTOGRAPH2);
	   printf("ERR: Inconsistent State  ");
	   break;

         case '{':
	   CursPos(20,24);
	   printf("SP1 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch1_opn_perm)
	     tm_cart_unlatch(SPECTOGRAPH1);
	   else
	     tm_cart_latch(SPECTOGRAPH1);
	   break;

         case '}':
	   CursPos(20,24);
	   printf("SP2 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch2_opn_perm)
	     tm_cart_unlatch(SPECTOGRAPH2);
	   else
	     tm_cart_latch(SPECTOGRAPH2);
	   break;

         case 'X': case 'x':
	   Running=FALSE;
	   ioctl(0,FIOOPTIONS,Options); /* back to normal */
           taskDelete(taskIdFigure("instPos"));
	   taskDelay (10);
	   EraseDisplayAll();
	   break;
	   
         case 'I': case 'i':
	   CursPos(20,24);
	   printf("Instrument FSM dd; 0=FIBER; 1=CorLens 2=TEST   ");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%d",&inst);
	     if ((inst<0)||(inst>16)) 
	     {
	       printf("ERR: Inst Out of Range (0-16)           ");
	       break;
	     }
	     taskSpawn ("fsm",60,VX_FP_TASK,4000,(FUNCPTR)fsm,
			  (int)inst,0,0,0,0,0,0,0,0,0);
	   }
	   break;

         case 'W': case 'w':
	   CursPos(20,24);
	   printf("dd|c..c; 2=EMPTY;3=SCF;4=S;5=SC;6=SE;7=SEC;8=SI");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     if ((buf[0]>='0')&&(buf[0]<='9'))
	     {
	       sscanf (buf,"%d",&inst);
	       if ((inst<0)||(inst>16)) 
	       {
	         printf("ERR: Inst Out of Range (0-16)           ");
	         break;
	       }
             }
             else
             {
               inst=cw_get_inst (&buf[0]);
	       if (inst==ERROR)
               {
                 printf("ERR: Inst Name Incorrect               ");
                 break;
               }
             }
	     CursPos(20,24);
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
			  (int)inst,0,0,0,0,0,0,0,0,0);
	     else
	     {
	       printf("ERR: Altitude Brake NOT Engaged         ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '!': case '@': case '#': case '$':
	   CursPos(20,24);
	   if (buf[0]=='@') cw = 2;
	   else cw = buf[0]-0x20;
	   printf("CW %d vvv                              ",cw);
	   cw--;
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%d",&cwpos);
	     CursPos(20,24);
	     if ((cwpos<10)||(cwpos>800))
	     {
	       printf("ERR: Position out of Range (10-800)    ");
	       break;
	     }
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cwp",60,VX_FP_TASK,4000,(FUNCPTR)cw_positionv,
			  (int)cw,(int)cwpos,0,0,0,0,0,0,0,0);
	     else
             {
	       printf ("Alt Brake NOT Engaged                  ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '^': 
	   CursPos(20,24);
	   printf("All CW vvv                             ");
	   if (GetString(&MenuInput[0],20))
	   {
	     memcpy(&buf[0],&MenuInput[0],21);
	     memset(&MenuInput[0],' ',20);
	     sscanf (buf,"%d",&cwpos);
	     CursPos(20,24);
	     if ((cwpos<10)||(cwpos>800))
	     {
	       printf("ERR: Position out of Range (10-800)    ");
	       break;
	     }
             if (taskIdFigure("cw")!=ERROR)
	     {
	       printf("ERR: CW task still active...be patient  ");
	       break;
	     }
             if (taskIdFigure("cwp")!=ERROR)
	     {
	       printf("ERR: CWP task still active..be patient  ");
	       break;
	     }
	     cw_set_positionv(INST_DEFAULT,cwpos,cwpos,cwpos,cwpos);
	     if (sdssdc.status.i9.il0.alt_brake_en_stat)
	       taskSpawn ("cw",60,VX_FP_TASK,4000,(FUNCPTR)balance_weight,
			  (int)INST_DEFAULT,0,0,0,0,0,0,0,0,0);
	     else
	     {
	       printf("ERR: Altitude Brake NOT Engaged         ");
	       break;
	     }
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case '%':
	   CursPos(20,24);
           printf ("CW ABORT                               ");
           taskDelete(taskIdFigure("cw"));
           taskDelete(taskIdFigure("cwp"));
	   cw_abort();
	   break;

         case '?': 
	   if (question_mark==0)
	   {
	     CursPos(1,1);
printf("Extended Help1..+|-=Align Close|Open;                                  \n");
             CursPos(1,2);
printf(" I=Instrument FSM dd; 0=FIBER; 1=Spectograph Corrector Lens 2=TEST            \n");
	     CursPos(1,3);
printf("                                                                              \n");
	     CursPos(1,5);
printf(" sp=RstScrn X=eXit                                                     \n");
	     CursPos(1,6);
printf(" W=Move CW; !|@|#|$=Move CW 1|2|3|4; %%=CW Halt                                \n");
	   }
	   if (question_mark==1)
	   {
	     CursPos(1,1);
printf("Extended Help2..(|)=slit1|slit2 toggle; {|}=cart latch1|cart latch2 toggle    \n");
	     CursPos(1,2);
printf(" |=flat field lamp toggle; \"=Ne lamp toggle; :=HgCd lamp toggle               \n");
	     CursPos(1,3);
printf(" ~=flat field screen toggle                                                   \n");
	     CursPos(1,4);
printf("                                                                              \n");
	     CursPos(1,5);
printf("MSA is Monitor status,axis State,Amp status(Amp,Brake,EStop)                  \n");
	     CursPos(1,6);
printf("CW Options: 2=EMPTY;3=SCF;4=S;5=SC;6=SE;7=SEC;8=SI                            \n");
	   }
	   question_mark = (question_mark+1)%2;
	   break;
	
         default:  
	   refreshing=TRUE;
 	   PrintInstBanner();
/*
	   printf ("Illegal Char Input=%x, %x, %x\n",
		buf[0],buf[1],buf[2]);
*/
	   break;
      }
/*        CursPos(74,23);*/
      semGive (semMEIUPD);
     }
     else
      printf ("\r\nIgnored input...Can't take semMEIUPD...DataCollection task probably at fault");
     taskDelay(5); /* 60/5 = 12Hz */
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: PrintInstPos
**
** DESCRIPTION:
**      Updates the Inst display parameters as a stand-alone task.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void PrintInstPos()
{
  extern struct SDSS_FRAME sdssdc;
  extern SEM_ID semMEIUPD;
  extern SEM_ID semMEI;
  extern int rawtick;
  extern struct TM_M68K *tmaxis[];
  extern struct FIDUCIARY fiducial[3];
  extern int fiducialidx[3];
  extern struct FIDUCIALS az_fiducial[];
  extern struct FIDUCIALS alt_fiducial[];
  extern struct FIDUCIALS rot_fiducial[];
  extern char *get_date();
  extern double sec_per_tick[];
  int fidsign;
  int state;
  int i;
  long ap,cp,ap2,vlt;
  double arcsec, farcsec;
  int lasttick;
  long marcs,arcs,arcm,arcd;
  short adc;
  int limidx;
  extern unsigned char cwLimit;
  extern int monitor_on[3];
  unsigned short ffs, ffl, ffc;
  char open[]={' ','O'};
  char close[]={' ','C'};
  char one[]={' ','1'};
  char two[]={' ','2'};
  char three[]={' ','3'};
  char four[]={' ','4'};
  char *oo[]={"Off"," On"};
  short pos, force;
  extern int il_ADC128F1;
	
  lasttick=0;
  FOREVER
  {  
    if (refreshing) 
    {
      taskDelay(60);
      refreshing=FALSE;
    }
    if (semTake (semMEIUPD,60)!=ERROR)
    {
      if (rawtick>(50+lasttick))
      {
        lasttick=rawtick;
        CursPos(17,7);
        printf("%5.2f\n",((double)rawtick)/(216000.0*24.0));
        CursPos(49,7);
        printf("%s",(char *)get_date());
      }
      for(i = 0; i < 3; i++)
      {
        CursPos(5,9+i);
        ap=(*tmaxis[i]).actual_position;
        cp=(*tmaxis[i]).position,
        vlt=(*tmaxis[i]).voltage;
	if (monitor_on[i]) printf ("*");
	else printf ("U");
        if (semTake (semMEI,NO_WAIT)!=ERROR)
        {
	  state=axis_state(i<<1);
	  semGive(semMEI);
	  switch (state)
          {
	    case NO_EVENT: case 1: case NEW_FRAME: printf ("*");
	     break;
	    case STOP_EVENT: printf ("S");
	     break;
	    case E_STOP_EVENT: printf ("E");
	     break;
	    case ABORT_EVENT: printf ("A");
	     break;
	    default: printf ("?");
	  }
	}
        arcsec=(sec_per_tick[i]*abs(ap));
	switch (i)
	{
          case AZIMUTH:
            farcsec=(sec_per_tick[i]*abs(az_fiducial[fiducialidx[i]].mark));
	    if (az_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
              else if (sdssdc.status.i9.il0.az_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (az_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case ALTITUDE:
            farcsec=(sec_per_tick[i]*abs(alt_fiducial[fiducialidx[i]].mark));
	    if (alt_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
              else if (sdssdc.status.i9.il0.alt_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (alt_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case INSTRUMENT:
            farcsec=(sec_per_tick[i]*abs(rot_fiducial[fiducialidx[i]].mark));
	    if (rot_amp_ok()) printf ("*");
	    else
	    {
	      if (check_stop_in()) printf ("S");
	        else printf ("?");
	    }
            if (rot_fiducial[fiducialidx[i]].mark<0)
              fidsign=-1;
            else
              fidsign=1;
	    break;

	  default:
	    arcsec=farcsec=0.;
            fidsign=0;
	    break;
        }
	if (fidsign!=0)
        {
          CursPos(8,9+i);
          arcd=(long)(arcsec)/3600;	     
          arcm=((long)(arcsec)-(arcd*3600))/60;	     
          arcs=((long)(arcsec)-(arcd*3600)-(arcm*60));	     
          marcs = (arcsec-(long)arcsec)*1000;
          if (ap<0)
            printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
          else
            printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
          printf(" %10ld  %10ld  %10ld",ap,cp,vlt);
          arcd=(long)(farcsec)/3600;	     
          arcm=((long)(farcsec)-(arcd*3600))/60;	     
          arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
          marcs = (farcsec-(long)farcsec)*1000;
          if (fiducialidx[i]!=-1)
          {
            if (fiducial[i].markvalid) printf("  V");
            else printf("   ");
            if (fidsign<0)
              printf("%3d;-%03ld:%02ld:%02ld:%03ld\n",
	        fiducialidx[i],arcd,arcm,arcs,marcs);
            else
              printf("%3d; %03ld:%02ld:%02ld:%03ld\n",
	        fiducialidx[i],arcd,arcm,arcs,marcs);
          }
          else
            printf("  No Crossing\n");
        }
      }
      printf("\t\t%4.2f",
        abs(sdssdc.status.i4.alt_position-altclino_off)*altclino_sf);
      ap=(*tmaxis[1]).actual_position2;
      ap2=(*tmaxis[2]).actual_position2;
      printf("\t\tR2=%10ld\tR3=%10ld\n",
        ap,
        ap2);
      semGive (semMEIUPD);
    }
    taskDelay(30);
    if (semTake (semMEIUPD,60)!=ERROR)
    {
      if (last_clamp!=(int)((sdssdc.status.i9.il0.clamp_en_stat)|
		      (sdssdc.status.i9.il0.clamp_dis_stat<<1)|
		      (sdssdc.status.o11.ol0.clamp_en<<2)|
		      (sdssdc.status.o11.ol0.clamp_dis<<3)) )
      {
      CursPos(14,13);
      printf("          ");
      CursPos(14,13);
      if (sdssdc.status.i9.il0.clamp_en_stat)
        printf("On  ");
      if (sdssdc.status.i9.il0.clamp_dis_stat)
        printf("Off ");
      if (sdssdc.status.o11.ol0.clamp_en)
        printf("OnCmd ");
      if (sdssdc.status.o11.ol0.clamp_dis)
        printf("OffCmd ");
      last_clamp=(sdssdc.status.i9.il0.clamp_en_stat)|
		      (sdssdc.status.i9.il0.clamp_dis_stat<<1)|
		      (sdssdc.status.o11.ol0.clamp_en<<2)|
		      (sdssdc.status.o11.ol0.clamp_dis<<3);
      }

      if (last_door1!=(int)((sdssdc.status.i1.il9.slit_head_door1_opn)|
		      (sdssdc.status.i1.il9.slit_head_door1_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch1_opn_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch1_opn<<2))*/ 
								)
      {
      CursPos(33,13);
      printf("                  ");
      CursPos(33,13);
      if (sdssdc.status.i1.il9.slit_head_door1_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door1_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch1_opn_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch1_opn)*/
        printf("LatchCmd   ");
      else
        printf("UnLatchCmd ");
      last_door1=(sdssdc.status.i1.il9.slit_head_door1_opn)|
		      (sdssdc.status.i1.il9.slit_head_door1_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch1_opn_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch1_opn<<2);*/ 
      }
      if (last_door2!=(int)((sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_opn_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch2_opn<<2))*/ 
		      						)
      {
      CursPos(62,13);
      printf("                  ");
      CursPos(62,13);
      if (sdssdc.status.i1.il9.slit_head_door2_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door2_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch2_opn_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch2_opn)*/
        printf("LatchCmd ");
      else
        printf("UnLatchCmd ");
      last_door2=(sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_opn_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch2_opn<<2);*/ 
      }

      ffs=(int)((sdssdc.status.i1.il13.leaf_1_open_stat)|
		(sdssdc.status.i1.il13.leaf_1_closed_stat<<1)|
		(sdssdc.status.i1.il13.leaf_2_open_stat<<2)|
		(sdssdc.status.i1.il13.leaf_2_closed_stat<<3)|
		(sdssdc.status.i1.il13.leaf_3_open_stat<<4)|
		(sdssdc.status.i1.il13.leaf_3_closed_stat<<5)|
		(sdssdc.status.i1.il13.leaf_4_open_stat<<6)|
		(sdssdc.status.i1.il13.leaf_4_closed_stat<<7)|
		(sdssdc.status.i1.il13.leaf_5_open_stat<<8)|
		(sdssdc.status.i1.il13.leaf_5_closed_stat<<9)|
		(sdssdc.status.i1.il13.leaf_6_open_stat<<10)|
		(sdssdc.status.i1.il13.leaf_6_closed_stat<<11)|
		(sdssdc.status.i1.il13.leaf_7_open_stat<<12)|
		(sdssdc.status.i1.il13.leaf_7_closed_stat<<13)|
		(sdssdc.status.i1.il13.leaf_8_open_stat<<14)|
		(sdssdc.status.i1.il13.leaf_8_closed_stat<<15) );

      if (last_ffs!=ffs)
      {
        CursPos(9,14);    
        printf ("1:%c%c 2:%c%c 3:%c%c 4:%c%c 5:%c%c 6:%c%c 7:%c%c 8:%c%c",
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
	close[sdssdc.status.i1.il13.leaf_8_closed_stat]);
        last_ffs=ffs;
      }
      ffl=(int)((sdssdc.status.i1.il13.ff_1_stat)|
	(sdssdc.status.i1.il13.ff_2_stat<<1)|
	(sdssdc.status.i1.il13.ff_3_stat<<2)|
	(sdssdc.status.i1.il13.ff_4_stat<<3)|
	(sdssdc.status.i1.il13.ne_1_stat<<4)|
	(sdssdc.status.i1.il13.ne_2_stat<<5)|
	(sdssdc.status.i1.il13.ne_3_stat<<6)|
	(sdssdc.status.i1.il13.ne_4_stat<<7)|
	(sdssdc.status.i1.il13.hgcd_1_stat<<8)|
	(sdssdc.status.i1.il13.hgcd_2_stat<<9)|
	(sdssdc.status.i1.il13.hgcd_3_stat<<10)|
	(sdssdc.status.i1.il13.hgcd_4_stat<<11) );
      if (last_ffl!=ffl)
      {
        CursPos(9,15);
        printf ("%c%c%c%c",
	one[sdssdc.status.i1.il13.ff_1_stat],
	two[sdssdc.status.i1.il13.ff_2_stat],
	three[sdssdc.status.i1.il13.ff_3_stat],
	four[sdssdc.status.i1.il13.ff_4_stat]);
        CursPos(26,15);
        printf ("%c%c%c%c",
	one[sdssdc.status.i1.il13.ne_1_stat],
	two[sdssdc.status.i1.il13.ne_2_stat],
	three[sdssdc.status.i1.il13.ne_3_stat],
	four[sdssdc.status.i1.il13.ne_4_stat]);
        CursPos(45,15);
        printf ("%c%c%c%c",
	one[sdssdc.status.i1.il13.hgcd_1_stat],
	two[sdssdc.status.i1.il13.hgcd_2_stat],
	three[sdssdc.status.i1.il13.hgcd_3_stat],
	four[sdssdc.status.i1.il13.hgcd_4_stat]);
        last_ffl=ffl;
      }
      ffc=(int)((sdssdc.status.o1.ol14.ff_screen_open_pmt)|
	(sdssdc.status.o1.ol14.ff_lamps_on_pmt<<1)|
	(sdssdc.status.o1.ol14.ne_lamps_on_pmt<<2)|
	(sdssdc.status.o1.ol14.hgcd_lamps_on_pmt<<3) );
      if (last_ffc!=ffc)
      {
        CursPos(53,14);
        printf("%s",oo[sdssdc.status.o1.ol14.ff_screen_open_pmt]);
        CursPos(18,15);
        printf("%s",oo[sdssdc.status.o1.ol14.ff_lamps_on_pmt]);
        CursPos(35,15);
        printf("%s",oo[sdssdc.status.o1.ol14.ne_lamps_on_pmt]);
        CursPos(54,15);
        printf("%s",oo[sdssdc.status.o1.ol14.hgcd_lamps_on_pmt]);
        last_ffc=ffc;
      }

      CursPos(1,17);
      for (i=0;i<4;i++)
      {
	adc=sdssdc.weight[i].pos;
        if ((adc&0x800)==0x800) adc |= 0xF000;
        else adc &= 0xFFF;
	limidx = (cwLimit>>(i*2))&0x3;
        printf ("%d %s\t",
	  (int)((1000*adc)/2048.),limitstatus[limidx]);
      }
      ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&pos);
      if ((pos&0x800)==0x800) pos |= 0xF000;
      else pos &= 0xFFF;
      ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&force);
      if ((force&0x800)==0x800) force |= 0xF000;
      else force &= 0xFFF;
      printf ("\t%d\t%d",pos,force);
      
      CursPos(1,19);
      memset(&inst_msg[10],' ',60);
      memcpy(&inst_msg[10],inst_display,min(strlen(inst_display),60));
      inst_msg[70]=NULL;
      if (inst_display!=NULL) printf ("%s",&inst_msg[0]);
      CursPos (60,24);
      printf ("%s",&MenuInput[0]);
      CursPos (74,23);
      semGive (semMEIUPD);
    }
    taskDelay (60);
  }
}
