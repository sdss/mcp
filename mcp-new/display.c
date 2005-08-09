#if 0
#include "copyright.h"
/**************************************************************************/
#include <stdio.h>
#include <ctype.h>
#include "display.h"
#include "tickLib.h"
#include "usrLib.h"
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
#include "instruments.h"
#include "mcpUtils.h"
#include "cw.h"
#include "mcpFiducials.h"
#include "cmd.h"

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
static int refreshing=FALSE;
static const char *limitstatus[]={"LU", "L.", ".U", ".."};

static int mei_axis=4;
static long adjpos[6]={0,0,0,0,0,0};
static long adjvel[6]={0,0,0,0,0,0};
static long adjacc[6]={70000,10000,10000,10000,10000,10000};
static long incvel[6]={1000,0,1000,0,1000,0};
static long Axis_vel[6]={0,0,0,0,0,0};
static long Axis_vel_neg[6]={-700000,0,-600000,0,-250000,0};
static long Axis_vel_pos[6]={700000,0,600000,0,250000,0};
static char MenuInput[21];

/* prototypes */
void Menu(void);			/* runnable at MCP prompt,
					   and thus not static */

static void PrintMenuBanner(void);
static void PrintMenuMove(void);
static void PrintMenuPos(void);

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
  *buf='\0';  /* terminate the string overwrite the LF */
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
void
TCC_check()
{
   int i;
   
   for (i=0;i<3;i++) {
      if (axis_queue[i].active!=NULL) {
	 printf ("\r\n Axis %d: TCC is has active entries in queue",i);
	 taskDelay (2*60);
      }
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
static void
PrintMenuBanner(void)
{
  CursPos(1,1);
  MenuInput[21]='\0';
  last_clamp=last_door1=last_door2=last_azbrake=last_altbrake=-1;
  EraseDisplayRest();
  printf("     /////// ///////   ///////  ///////   Sloan Digital Sky Survey  Version: %d\n",SoftwareVersion_); 
  printf("    //       //   //  //       //           software by Charlie and Robert     \n");
  printf("   //////   //   //  ///////  ///////     Compiled: %s %s\n",__DATE__, __TIME__);
  printf("      //   //   //       //       //      Tag: %-20s            \n", getCvsTagname());
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
  if (mei_axis==0) printf("Azimuth Controls  ");
  if (mei_axis==2) printf("Altitude Controls ");
  if (mei_axis/2==2) printf("Rotator Controls  ");
  printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
	  Axis_vel[mei_axis],incvel[mei_axis]);
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
static void
PrintMenuMove(void)
{
  double arcsecond;
  long marcs,arcs,arcm,arcd;

  arcsecond=(sec_per_tick[mei_axis>>1]*abs(adjpos[mei_axis]));
  arcd=(long)(arcsecond)/3600;	     
  arcm=((long)(arcsecond)-(arcd*3600))/60;
  arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
  marcs = (arcsecond-(long)arcsecond)*1000;
  CursPos(18,20);
  printf("Move to  Position ");
  if (adjpos[mei_axis]<0)
    printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
	arcd,arcm,arcs,marcs,adjpos[mei_axis]);
  else
    printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
	arcd,arcm,arcs,marcs,adjpos[mei_axis]);
  CursPos(18,21);
  printf("   Velocity ");
  printf ("%10ld Cts/Sec",adjvel[mei_axis]);
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
void
Menu(void)
{
  const char *ans;			/* answer string from a command */
  int Running = TRUE;
  char buf[255];
  int cw = -1;				/* desired counter weight */

  int cwpos;
  int inst;
  int Options;
  long pos, deg, min, arcsec, marcsec;
  double arcsecond;
  long marcs,arcs,arcm,arcd;
  int question_mark = 0;
  int negative;

  TCC_check();
  if(semTake (semMEIUPD,60) == ERROR) {
     printf("\r\nCan't take semMEIUPD..."
	    "DataCollection task probably at fault");
     return;
  }

  Options=ioctl(0,FIOGETOPTIONS,0); /* save present keyboard options */
  ioctl(0,FIOOPTIONS,Options & ~OPT_ECHO & ~OPT_LINE);

  refreshing=TRUE;
  PrintMenuBanner();
  PrintMenuMove();
  semGive (semMEIUPD);

   while(Running) {
      buf[0]=getchar();

    if(semTake (semMEIUPD,60) == ERROR) {
       printf ("\r\nIgnored input...Can't take semMEIUPD..."
	       "DataCollection task probably at fault");
       taskDelay(5); /* 60/5 = 12Hz */
       continue;
    }
    
    switch(buf[0]) {
       case 'Z': case 'z': CursPos(1,19);
         printf("Azimuth Controls  ");
         mei_axis=0;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
		 Axis_vel[mei_axis], incvel[mei_axis]);
         PrintMenuMove();
         break;

       case 'L': case 'l': CursPos(1,19);
         printf("Altitude Controls ");
         mei_axis=2;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
		 Axis_vel[mei_axis], incvel[mei_axis]);
         PrintMenuMove();
         break;

       case 'R': case 'r': CursPos(1,19);
         printf("Rotator Controls  ");
         mei_axis=4;
         printf ("<--J-- %6ld --K--> Increment=%ld Cts\n",
		 Axis_vel[mei_axis], incvel[mei_axis]);
         PrintMenuMove();
         break;

       case 'P': case 'p': CursPos(20,24);
         printf("Set Position    xxx:xx:xx:xxx           ");
         if(GetString(&MenuInput[0],20)) {
           memcpy(&buf[0],&MenuInput[0],21);
           memset(&MenuInput[0],' ',20);

           min=0; arcsec=0; marcsec=0;
           if (buf[0]=='-') {
             sscanf (&buf[1],"%ld:%ld:%ld:%ld",
			&deg,&min,&arcsec,&marcsec);
             negative=TRUE;
           } else {
             sscanf (buf,"%ld:%ld:%ld:%ld",
			&deg,&min,&arcsec,&marcsec);
             negative=FALSE;
           }
	   pos=(long)((abs(deg)*3600000.)+(min*60000.)+
		      (arcsec*1000.)+marcsec)/(sec_per_tick[mei_axis>>1]*1000);
	   
           if (negative) pos = -pos;

	   (void)mcp_set_pos(mei_axis/2, pos);
         }
         CursPos(20,24);
         printf("                                        ");
         break;

       case 'F': case 'f': CursPos(20,24);
     	 printf("Set Fiducial Position                    ");
	 if(mcp_set_fiducial(mei_axis/2) < 0) {
	    printf("ERR: fiducial for axis not crossed      ");
	 }
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
	   adjpos[mei_axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
	     (arcsec*1000.)+marcsec)/(sec_per_tick[mei_axis>>1]*1000);
	   if (negative) adjpos[mei_axis] = -adjpos[mei_axis];
	   CursPos(36,20);
	   if (adjpos[mei_axis]<0)
	     printf("-%03d:%02ld:%02ld:%03ld %10ld Cts",
		abs(deg),min,arcsec,marcsec,adjpos[mei_axis]);
     	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		deg,min,arcsec,marcsec,adjpos[mei_axis]);
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
	   adjpos[mei_axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
	     (arcsec*1000.)+marcsec)/(sec_per_tick[mei_axis>>1]*1000);
	   if (negative) adjpos[mei_axis] = -adjpos[mei_axis];
           adjpos[mei_axis] += (*tmaxis[mei_axis/2]).actual_position;
	   arcsecond=(sec_per_tick[mei_axis>>1]*abs(adjpos[mei_axis]));
	   arcd=(long)(arcsecond)/3600;	     
	   arcm=((long)(arcsecond)-(arcd*3600))/60;
	   arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
	   marcs = (arcsecond-(long)arcsecond)*1000;
	   CursPos(36,20);
	   if (adjpos[mei_axis]<0)
	     printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[mei_axis]);
	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[mei_axis]);
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
	   sscanf (buf,"%ld",&adjpos[mei_axis]);
           adjpos[mei_axis] += (*tmaxis[mei_axis/2]).actual_position;
	   arcsecond=(sec_per_tick[mei_axis>>1]*abs(adjpos[mei_axis]));
	   arcd=(long)(arcsecond)/3600;	     
	   arcm=((long)(arcsecond)-(arcd*3600))/60;
	   arcs=((long)(arcsecond)-(arcd*3600)-(arcm*60));
	   marcs = (arcsecond-(long)arcsecond)*1000;
	   CursPos(36,20);
	   if (adjpos[mei_axis]<0)
	     printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[mei_axis]);
	   else
	     printf(" %03ld:%02ld:%02ld:%03ld %10ld Cts",
		arcd,arcm,arcs,marcs,adjpos[mei_axis]);
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
	   sscanf (buf,"%ld",&adjvel[mei_axis]);
	   adjvel[mei_axis]=abs(adjvel[mei_axis]);
	   if (adjvel[mei_axis]>Axis_vel_pos[mei_axis]) 
	     adjvel[mei_axis]=Axis_vel_pos[mei_axis];
	   CursPos(30,21);
	   printf ("%10ld Cts/Sec",adjvel[mei_axis]);
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
	     sscanf (buf,"%ld",&incvel[mei_axis]);
	     incvel[mei_axis]=abs(incvel[mei_axis]);
	     if (incvel[mei_axis]>10000) incvel[mei_axis]=10000;
	     CursPos(49,19);
	     printf ("%ld Cts",incvel[mei_axis]);
	   }
	   CursPos(20,24);
	   printf("                                        ");
	   break;

         case 'B': case 'b': CursPos(20,24);
	   if(mei_axis==0) {
	     printf("Azimuth Brake Turned On                 ");
	   } else if(mei_axis==2) {
	     printf("Altitude Brake Turned On                ");
	   }
           mcp_set_brake(mei_axis/2);

           STOPed[mei_axis]=TRUE;
	   break;

         case 'C': case 'c': CursPos(20,24);
	   if(mei_axis == 0) {
	      printf("Azimuth Brake Turned Off                ");
	   } else if(mei_axis == 2) {
	      printf("Altitude Brake Turned Off               ");
	   }

           Axis_vel[mei_axis]=0;
           CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[mei_axis]);

	   if(mcp_unset_brake(mei_axis/2) < 0) {
	      CursPos(20,24);
	      printf("Err: Could not take semMEI semphore     ");
	      break;
	   }

	   STOPed[mei_axis] = FALSE;
      
 	   break;

         case '+': case '=':
	   CursPos(20,24);
	   printf("ALIGNment Clamp Turned On               ");
	   (void)cmd_handler(1, "CLAMP.ON", NULL);
	   break;

         case '_': case '-':
           CursPos(20,24);
	   printf("ALIGNment Clamp Turned Off              ");
	   (void)cmd_handler(1, "CLAMP.OFF", NULL);
	   break;

         case '(':
	   CursPos(20,24);
	   printf("SP1 Slit Door Toggled          ");
	   if ((sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (!sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     mcp_specdoor_close(SPECTROGRAPH1);
	     break;
	   }
	   if ((!sdssdc.status.i1.il9.slit_head_door1_opn)&&
	       (sdssdc.status.i1.il9.slit_head_door1_cls))
	   {
	     mcp_specdoor_open(SPECTROGRAPH1);
	     break;
	   }
	   mcp_specdoor_open(SPECTROGRAPH1);
	   printf("ERR: Inconsistent State  ");
	   break;

         case ')':
	   CursPos(20,24);
	   printf("SP2 Slit Door Toggled          ");
	   if(sdssdc.status.o1.ol9.slit_dr2_opn_perm &&
	      !sdssdc.status.i1.il9.slit_head_door2_cls) {
	      mcp_specdoor_close(SPECTROGRAPH2);
	      break;
	   }
	   if(!sdssdc.status.o1.ol9.slit_dr2_opn_perm &&
	      sdssdc.status.i1.il9.slit_head_door2_cls) {
	      mcp_specdoor_open(SPECTROGRAPH2);
	      break;
	   }

	   mcp_specdoor_open(SPECTROGRAPH2);
	   printf("ERR: Inconsistent State  ");
	   break;

         case '{':
	   CursPos(20,24);
	   printf("SP1 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch1_ext_perm)
	     mcp_slithead_latch_open(SPECTROGRAPH1);
	   else
	     mcp_slithead_latch_close(SPECTROGRAPH1);
	   break;

         case '}':
	   CursPos(20,24);
	   printf("SP2 Latch Toggled               ");
	   if (sdssdc.status.o1.ol9.slit_latch2_ext_perm)
	     mcp_slithead_latch_open(SPECTROGRAPH2);
	   else
	     mcp_slithead_latch_close(SPECTROGRAPH2);
	   break;

         case '~':
	   CursPos(20,24);
	   printf("Flat Field Screen Toggled ");
	   if(sdssdc.status.o1.ol14.ff_screen_open_pmt) {
	      printf("Close ");
	      (void)cmd_handler(1, "FFS.CLOSE", NULL);
	   } else {
	      printf("Open  ");
	      (void)cmd_handler(1, "FFS.OPEN", NULL);
	   }
	   break;

         case '|':
	   CursPos(20,24);
	   printf("Flat Field Lamps Toggled ");
	   if (sdssdc.status.o1.ol14.ff_lamps_on_pmt) {
	      printf("Off   ");
	      (void)cmd_handler(1, "FFL.OFF", NULL);
	   } else {
	      printf("On    ");
	      (void)cmd_handler(1, "FFL.ON", NULL);
	   }
	   break;

         case '"':
	   CursPos(20,24);
	   printf("Flat Field Neon Toggled ");
	   if(sdssdc.status.o1.ol14.ne_lamps_on_pmt) {
	      printf("Off     ");
	      (void)cmd_handler(1, "NE.OFF", NULL);
	   } else {
	      printf("On     ");	
	      (void)cmd_handler(1, "NE.ON", NULL);
	   }
	   break;

         case ':':
	   CursPos(20,24);
	   printf("Flat Field HgCd Toggled ");
	   if(sdssdc.status.o1.ol14.hgcd_lamps_on_pmt) {
	      printf("Off    ");
	      (void)cmd_handler(1, "HGCD.OFF", NULL);
	   } else {
	      printf("On     ");	
	      (void)cmd_handler(1, "HGCD.ON", NULL);
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
	   if (GetString(&MenuInput[0],20)) {
	      memcpy(&buf[0],&MenuInput[0],21);
	      memset(&MenuInput[0],' ',20);
	      
	      if(isdigit((int)buf[0])) {
		 (void)sscanf(buf, "%d", &inst); /* must succeed */
		 if(inst < 0 || inst >= NUMBER_INST) {
		    printf("ERR: Inst Out of Range (0-%d)           ",
			   NUMBER_INST - 1);
		    break;
		 }
	      } else {
		 inst = cw_get_inst(&buf[0]);
		 if(inst == ERROR) {
		    printf("ERR: Inst Name %s Incorrect               ", buf);
		    break;
		 }
	      }

	      mcp_set_cw(inst, ALL_CW, 0, &ans);
	      if(*ans != '\0') {
		 printf("%s", ans);
	      }
	   }
	   break;

         case '^':
         case '!': case '@': case '#': case '$':
	   switch (buf[0]) {
	    case '^': cw = ALL_CW; break;
	    case '!': cw = 0; break;
	    case '@': cw = 1; break;
	    case '#': cw = 2; break;
	    case '$': cw = 3; break;
	   }

	   CursPos(20,24);
           if(cw == ALL_CW) {
	      printf("All CW vvv                             ");
	   } else {
	      printf("CW %d vvv                              ",cw + 1);
	   }
      
	   if(GetString(&MenuInput[0],20)) {
	      memcpy(&buf[0],&MenuInput[0],21);
	      memset(&MenuInput[0],' ',20);

	      CursPos(20,24);

	      if(sscanf(buf,"%d",&cwpos) != 1) {
		 printf("ERR: unable to read CW position");
		 break;
	      }

	      mcp_set_cw(INST_DEFAULT, cw, cwpos, &ans);
	      if(*ans != '\0') {
		 printf("%s", ans);
	      }
	   }
	   break;

         case '%':
	   CursPos(20,24);
           printf ("CW ABORT                               ");
           mcp_cw_abort();
	   break;

         case '&':
	   CursPos(20,24);
	   mcp_set_monitor(mei_axis/2, -1);
	   printf("Toggle the axis monitor_on to %s",
		  (monitor_on[mei_axis/2] ? "TRUE " : "FALSE "));

	   break;

         case '*':
	   CursPos(20,24);
           printf ("AMP RESET                              ");
           mcp_amp_reset(mei_axis/2);
	   break;

         case 'H': case 'h':
	   if(mcp_hold(mei_axis/2) < 0) {
	      CursPos(20,24);
	      printf("Err: Could not take semMEI semphore     ");
	   }

	   Axis_vel[mei_axis] = 0;
	   STOPed[mei_axis] = FALSE;

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
	   Axis_vel[mei_axis]=0;
	   CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[mei_axis]);
           STOPed[mei_axis]=TRUE;
	   if(mcp_stop_axis(mei_axis/2) < 0) {
	      CursPos(20,24);
	      printf("Err: stopping axes");
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
               sem_controller_run(2*AZIMUTH);
             }
             if (STOPed[2])
             {
               STOPed[2]=FALSE;
               sem_controller_run(2*ALTITUDE);
             }
	     Axis_vel[0]=0;
	     Axis_vel[2]=0;
	     start_move_corr(0,(double)(adjpos[0]),
		(double)adjvel[0],(double)adjacc[0]);
             start_move_corr(2,(double)(adjpos[2]),
		(double)adjvel[2],(double)adjacc[2]);
	     semGive (semMEI); 
           }
           else
           {
	     CursPos(20,24); printf("Err: Could not take semMEI semphore     ");
	   }
	   break;

         case 'M': case 'm':  CursPos(20,24);
	   if (adjvel[mei_axis]==0) {
	      printf("ERR: Velocity is Zero                   ");
	      break;
	   }
	 
           if(STOPed[mei_axis]) {
	      if (mei_axis==0 && sdssdc.status.i9.il0.az_brake_en_stat) {
		 CursPos(20,24);
		 printf("ERR: AZ Brake is Engaged                ");
		 break;
	      }
	      if(mei_axis == 2 && sdssdc.status.i9.il0.alt_brake_en_stat) {
		 CursPos(20,24);
		 printf("ERR: ALT Brake is Engaged               ");
		 break;
	      }
	      STOPed[mei_axis]=FALSE;
	   }

	   if(mcp_move_va(mei_axis/2, adjpos[mei_axis],
			  adjvel[mei_axis], adjacc[mei_axis]) < 0) {
	      CursPos(20,24);
	      printf("Err: Could not take semMEI semphore     ");
	   } else {
	      Axis_vel[mei_axis] = 0;
	   }

	 break;

         case 'J': case 'j':
         case 'K': case 'k':
	   if(buf[0] == 'J' || buf[0] == 'j') {
	      Axis_vel[mei_axis] -= incvel[mei_axis];
	      if(Axis_vel[mei_axis] < Axis_vel_neg[mei_axis]) {
		 Axis_vel[mei_axis] = Axis_vel_neg[mei_axis];
	      }
	   } else {
	      Axis_vel[mei_axis] += incvel[mei_axis];
	      if(Axis_vel[mei_axis] > Axis_vel_pos[mei_axis]) {
		 Axis_vel[mei_axis] = Axis_vel_pos[mei_axis];
	      }
	   }
	 
	 CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[mei_axis]);

	 CursPos(20,24);
	 if(mei_axis == 0 && sdssdc.status.i9.il0.az_brake_en_stat &&
						     Axis_vel[mei_axis] != 0) {
	    printf("ERR: AZ Brake is Engaged                ");
	    break;
	 }
	 if(mei_axis == 2 && sdssdc.status.i9.il0.alt_brake_en_stat &&
						     Axis_vel[mei_axis] != 0) {
	    printf("ERR: ALT Brake is Engaged               ");
	    break;
	 }

	 if(STOPed[mei_axis]) {
	    if(mcp_unset_brake(mei_axis/2) < 0) {
	       printf("Err: Could not take semMEI semphore     ");
	    }

	    STOPed[mei_axis]=FALSE;
	 }

	 if(semTake (semMEI,60) == ERROR) {
	    printf("Err: Could not take semMEI semphore     ");
	 }
	 
#if SWITCH_PID_COEFFS
	 if(mei_axis == 4) {
	    while(coeffs_state_cts(mei_axis, Axis_vel[mei_axis]) == TRUE) {
	       ;
	    }
	 }
#endif
	 set_velocity (mei_axis, Axis_vel[mei_axis]);
	 semGive (semMEI); 

	   break;
	   
       default:  
	 refreshing=TRUE;
	 PrintMenuBanner(); PrintMenuMove();
	 break;
      }
      semGive (semMEIUPD);
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
void
PrintMenuPos()
{
  int state;
  int fidsign;
  int i;
  long ap,ccp,ap1,ap2,vlt;
  double arcsec, farcsec;
  int lasttick;
  long marcs,arcs,arcm,arcd;
  short adc;
  int limidx;
	
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
        ccp=(*tmaxis[i]).position,
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
            farcsec=(sec_per_tick[i]*abs(az_fiducial[fiducialidx[i]].mark[1]));
	    if (az_amp_ok(0)) printf ("*");
	    else
	    {
	      if (check_stop_in(0)) printf ("S");
              else if (sdssdc.status.i9.il0.az_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (az_fiducial[fiducialidx[i]].mark[1] < 0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case ALTITUDE:
            farcsec =
	      (sec_per_tick[i]*abs(alt_fiducial[fiducialidx[i]].mark[1]));
	    if (alt_amp_ok(0)) printf ("*");
	    else
	    {
	      if (check_stop_in(0)) printf ("S");
              else if (sdssdc.status.i9.il0.alt_brake_en_stat) printf ("B");
	        else printf ("?");
	    }
            if (alt_fiducial[fiducialidx[i]].mark[1] < 0)
              fidsign=-1;
            else
              fidsign=1;
            break;

          case INSTRUMENT:
            farcsec=(sec_per_tick[i]*
		     abs(rot_fiducial[fiducialidx[i]].mark[1] - ROT_FID_BIAS));
	    if(rot_amp_ok(0)) {
	       printf ("*");
	    } else {
	       if(check_stop_in(0)) printf ("S");
	       else printf ("?");
	    }
	    
            if(rot_fiducial[fiducialidx[i]].mark[1] < ROT_FID_BIAS) {
	       fidsign=-1;
            } else {
	       fidsign=1;
	    }
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
          printf(" %10ld  %10ld  %10ld",ap,ccp,vlt);
          arcd=(long)(farcsec)/3600;	     
          arcm=((long)(farcsec)-(arcd*3600))/60;	     
          arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
          marcs = (farcsec-(long)farcsec)*1000;
          if (fiducialidx[i]!=-1)
          {
	    printf("   ");
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
      printf("\t\t%4.2f", read_clinometer());
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
      		      (sdssdc.status.o1.ol9.slit_latch1_ext_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch1_ext<<2))*/ 
								)
      {
      CursPos(33,13);
      printf("                  ");
      CursPos(33,13);
      if (sdssdc.status.i1.il9.slit_head_door1_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door1_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch1_ext_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch1_ext)*/
        printf("LatchCmd   ");
      else
        printf("UnLatchCmd ");
      last_door1=(sdssdc.status.i1.il9.slit_head_door1_opn)|
		      (sdssdc.status.i1.il9.slit_head_door1_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch1_ext_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch1_ext<<2);*/ 
      }
      if (last_door2!=(int)((sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_ext_perm<<2))
/*		      (sdssdc.status.i1.il9.slit_head_latch2_ext<<2))*/ 
		      						)
      {
      CursPos(62,13);
      printf("                  ");
      CursPos(62,13);
      if (sdssdc.status.i1.il9.slit_head_door2_opn)
        printf("Open ");
      if (sdssdc.status.i1.il9.slit_head_door2_cls)
        printf("Closed ");
      if (sdssdc.status.o1.ol9.slit_latch2_ext_perm)
/*      if (sdssdc.status.i1.il9.slit_head_latch2_ext)*/
        printf("LatchCmd ");
      else
        printf("UnLatchCmd ");
      last_door2=(sdssdc.status.i1.il9.slit_head_door2_opn)|
		      (sdssdc.status.i1.il9.slit_head_door2_cls<<1)|
      		      (sdssdc.status.o1.ol9.slit_latch2_ext_perm<<2);
/*		      (sdssdc.status.i1.il9.slit_head_latch2_ext<<2);*/ 
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
#else
void dummy_display_c(void){}
#endif
