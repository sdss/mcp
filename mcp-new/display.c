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

Routines to control the display screen
These functions can be used with a Vt220 terminal to position the cursor,
erase the screen etc.
******************************************************************************/
void Menu();
static void PrintMenu();
static void PrintBanner();
void PrintMove();
void PrintPos();
#define SoftwareVersion_	7
/* INCLUDES */
#include "display.h"
#include <stdio.h>
#include "ioLib.h"
#include "semLib.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "taskLib.h"
#include "axis.h"
/******************************************************************************/
/* Terminal Escape sequences */
/*
#define ESC '\x1B'
#define CSI "\x1B["
*/
#define ESC '\033'
#define CSI "\033["
/******************************************************************************/
static char scm[]="@(#)display.c	1.1 04/04/96 13:46:57";
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
char MenuInput[21];
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

	*buf=getchar();
/*	printf ("0x%x",*buf);*/
	i=1;
	while ((*buf!=0xA)&&(i<cnt))
	{
	  if (*buf==0x1b)
	  {
	    memset(&MenuInput[0],' ',20);
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
        if (i==1) return (FALSE);
	return(TRUE);
}
#include "data_collection.h"
/*#define ROT_VEL_MODE	1*/
int Axis=4;
int Axis_vel[6]={0,0,0,0,0,0};
int adjpos[6]={0,0,0,0,0,0};
int adjvel[6]={0,0,0,0,0,0};
int adjacc[6]={10000,10000,10000,10000,10000,10000};
#ifdef ROT_VEL_MODE
int incvel[6]={1000,0,1000,0,100,0};
#else
int incvel[6]={1000,0,1000,0,1000,0};
#endif
/* 8863 is pinned at .9 degrees; -9471 is zenith 90 degrees before */
/* 8857 is pinned at 0 degrees; -9504 is zenith 90 degrees 22-Aug-98 */
float altclino_sf=.0049016925256;/*.0048598736;*//*.0047368421 90 deg=19000*//*.0049011599*/
int altclino_off=8857;/*9048;*/         /*9500*/
double tickperarcs[3]={AZ_TICK,ALT_TICK,ROT_TICK};
static int refreshing=FALSE;
/****************************************************************************/
static void PrintMenu()
{
   CursPos(1,22);
   EraseDisplayRest();
   printf("/////////////////////////////// SDSS ////////////////////////////////\n\r");
   printf("R=Rotator Z=aZimuth L=aLtitude S=Stop H=Hold ?=help X=eXit......Command->\n\r");
}  

/****************************************************************************/
static void PrintBanner()
{
float Days;
time_t CurrentTime;

   CursPos(1,1);
   MenuInput[21]=NULL;
   EraseDisplayRest();
   time(&CurrentTime);
   Days=(float)tickGet()/(216000.0*24.0);
   printf("     /////// ///////   ///////  ///////   Sloan Digital Sky Survey  Version: %d\n",SoftwareVersion_); 
   printf("    //       //   //  //       //           software by Charlie Briegel        \n");
   printf("   //////   //   //  ///////  ///////     Compiled: %s %s\n",__DATE__, __TIME__);
   printf("      //   //   //       //       //                                           \n");
   printf("     //   //   //       //       //                                            \n");
   printf("//////  ///////    //////   //////                                             \n");  
   printf("Time Since Boot: %5.2f Days               Date:     \n",
	Days/*,(char *)get_date()*/);
   CursPos(1,8);
   printf ("    DegMinSecMAS   ActualPosition  CmdPosition  VoltageOut    Fiducial;Pos\n\r");
   CursPos(1,9);    
   printf("AZ\n\r");
   CursPos(1,10);    
   printf("AL\n\r");
   CursPos(1,11);    
   printf("ROT\n\r");
   CursPos(1,12);    
   printf("Alt Clinomator=       degrees\n\r");
   CursPos(1,13);    
   printf("ALIGN Clamp\n\r");
   CursPos(1,14);    
   printf("AZ Brake\n\r");
   CursPos(1,15);    
   printf("ALT Brake\n\r");
   CursPos(1,16);    
   printf("CW1\t\tCW2\t\tCW3\t\tCW4\n\r");
   CursPos(1,19);
   if (Axis==0) printf("Azimuth Controls  ");
   if (Axis==2) printf("Altitude Controls ");
   if (Axis/2==2) printf("Rotator Controls  ");
   printf ("<--J-- %6ld --K--> Increment=%d Cts\n",Axis_vel[Axis],incvel[Axis]);
   CursPos(1,22);
   printf("/////////////////////////////// SDSS ////////////////////////////////\n\r");
   printf("R=Rotator Z=aZimuth L=aLtitude S=Stop H=Hold ?=help X=eXit......Command->\n\r");
   if (taskIdFigure("menuPos")==ERROR)
     taskSpawn("menuPos",99,VX_FP_TASK,4000,(FUNCPTR)PrintPos,
	0,0,0,0,0,0,0,0,0,0);
}
void PrintMove()
{
double arcsecond;
long marcs,arcs,arcm,arcd;

#ifdef ROT_VEL_MODE
	if (Axis==4) 
        {
	  CursPos(18,20);
	  printf("                                                     \n");
	  printf("                                                     ");
          return;
        }
#endif
       	if (Axis==0)
	  arcsecond=(AZ_TICK*abs(adjpos[Axis]));
       	if (Axis==2)
	  arcsecond=(ALT_TICK*abs(adjpos[Axis]));
       	if (Axis/2==2)
	  arcsecond=(ROT_TICK*abs(adjpos[Axis]));
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
static int STOPed[6]={TRUE,TRUE,TRUE,TRUE,TRUE,TRUE};
#ifdef ROT_VEL_MODE
int Axis_vel_neg[6]={-200000,0,-200000,0,-1600,0};
int Axis_vel_pos[6]={200000,0,200000,0,1600,0};
#else
int Axis_vel_neg[6]={-200000,0,-200000,0,-500000,0};
int Axis_vel_pos[6]={200000,0,200000,0,500000,0};
#endif
/****************************************************************************/
void Menu()
/* This services the display terminal.  */
{
int Running=TRUE;
char buf[255];
extern struct SDSS_FRAME sdssdc;
extern struct TM_M68K *tmaxis[];
extern SEM_ID semMEI;
extern SEM_ID semMEIDC;
extern void tm_az_brake_on(),tm_az_brake_off();
extern void tm_alt_brake_on(),tm_alt_brake_off();
extern void tm_set_pos(int axis, int pos);
extern void manTrg();
extern char *balance_weight(int inst);
extern int cw_pos(int cw, float pos);
extern int cw_abort();
int cw;
float fpos;
int inst;
extern int amp_reset(int axis);
extern struct FIDUCIARY fiducial[3];
extern long fiducial_position[3];
int Options;
long pos, deg, min, arcsec, marcsec;
double arcsecond;
long marcs,arcs,arcm,arcd;
int negative;

   Options=ioctl(0,FIOGETOPTIONS,0); /* save present keyboard options */
   ioctl(0,FIOOPTIONS,Options & ~OPT_ECHO & ~OPT_LINE);
   if (semTake (semMEIDC,60)!=ERROR)
   {
     PrintBanner();
     PrintMove();
     semGive (semMEIDC);
   }
   else
   {
     printf ("\r\nCan't take semMEIDC...DataCollection task probably at fault");
     return;
   }
   while(Running)
   {
      buf[0]=getchar();
 /*     gets(buf);  */
      switch(buf[0])
      {
         case 'Z': case 'z': CursPos(1,19);
	   printf("Azimuth Controls  ");
	   Axis=0;
	   printf ("<--J-- %6ld --K--> Increment=%d Cts\n",
	   Axis_vel[Axis],incvel[Axis]);
           if (semTake (semMEIDC,60)!=ERROR)
           {
	     PrintMove();
             semGive (semMEIDC);
	   }
	   break;
         case 'L': case 'l': CursPos(1,19);
	   printf("Altitude Controls ");
	   Axis=2;
	   printf ("<--J-- %6ld --K--> Increment=%d Cts\n",
	   Axis_vel[Axis],incvel[Axis]);
           if (semTake (semMEIDC,60)!=ERROR)
           {
	     PrintMove();
             semGive (semMEIDC);
	   }
	   break;
         case 'R': case 'r': CursPos(1,19);
	   printf("Rotator Controls  ");
	   Axis=4;
	   printf ("<--J-- %6ld --K--> Increment=%d Cts\n",
	   Axis_vel[Axis],incvel[Axis]);
           if (semTake (semMEIDC,60)!=ERROR)
           {
	     PrintMove();
             semGive (semMEIDC);
	   }
	   break;
         case 'P': case 'p': CursPos(20,24);
	   printf("Set Position    xxx:xx:xx:xxx           ");
		/*sscanf (buf,"%d",&pos);*/
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
	     if (Axis==0)
	       pos=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(AZ_TICK*1000);
	     if (Axis==2)
	       pos=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(ALT_TICK*1000);
	     if (Axis/2==2)
	     {
	       pos=(long)((abs(deg)*3600000.)+(min*60000.)+
                                    (arcsec*1000.)+marcsec)/(ROT_TICK*1000);
   	     }
	     if (negative) pos = -pos;
	     tm_set_pos(Axis,pos);
	     if (Axis==0) tm_set_pos(Axis+1,pos);
	     if (Axis==4) 
	     {
	       tm_set_pos(Axis+1,pos);
	       tm_set_pos(Axis-1,pos);
	     }
	     fiducial[Axis/2].markvalid=FALSE;
	   }
           CursPos(20,24);
           printf("                                        ");
	   break;
         case 'F': case 'f': CursPos(20,24);
		  	printf("Set Fiducial Position                    ");
			if (fiducial[Axis/2].markvalid)
			{
			  pos=fiducial_position[Axis/2];
/* use optical encoder for axis 4 */
    			  if (semTake (semMEIDC,60)!=ERROR)
     			  {
			    if (Axis/2==2) 
	  		    { 
		              pos += ((*tmaxis[Axis/2]).actual_position2-
				fiducial[Axis/2].mark);
			      tm_set_pos(Axis+1,pos);
  			      tm_set_pos(Axis-1,pos);
	                      pos = (pos*OPT_TICK)/ROT_TICK;
  			      tm_set_pos(Axis,pos);
		            }
			    else
			    {
		              pos += ((*tmaxis[Axis/2]).actual_position-
				fiducial[Axis/2].mark);
			      tm_set_pos(Axis&0x6,pos);
			    }
     			    semGive (semMEIDC);
			  }
			  else
                            printf("ERR: can't take MEIDC semaphore          ");
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
				if (Axis==0)
				  adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(AZ_TICK*1000);
				if (Axis==2)
				  adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(ALT_TICK*1000);
				if (Axis/2==2)
				 adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
                                  (arcsec*1000.)+marcsec)/(ROT_TICK*1000);
				if (negative) adjpos[Axis] = -adjpos[Axis];
				CursPos(36,20);
	    			if (adjpos[Axis]<0)
	      			  printf("-%03ld:%02ld:%02ld:%03ld %10ld Cts",
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
				if (Axis==0)
				  adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(AZ_TICK*1000);
				if (Axis==2)
				  adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
				    (arcsec*1000.)+marcsec)/(ALT_TICK*1000);
				if (Axis/2==2)
				 adjpos[Axis]=(long)((abs(deg)*3600000.)+(min*60000.)+
                                  (arcsec*1000.)+marcsec)/(ROT_TICK*1000);
				if (negative) adjpos[Axis] = -adjpos[Axis];
  				if (semTake (semMEIDC,60)!=ERROR)
  				{
			          adjpos[Axis] += (*tmaxis[Axis/2]).actual_position;
	  			  semGive (semMEIDC);
        			}
				else break;
       				if (Axis==0)
	  			  arcsecond=(AZ_TICK*abs(adjpos[Axis]));
       				if (Axis==2)
	   			  arcsecond=(ALT_TICK*abs(adjpos[Axis]));
       				if (Axis/2==2)
	  			  arcsecond=(ROT_TICK*abs(adjpos[Axis]));
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
				/*sscanf (buf,"%d",&pos);*/
				sscanf (buf,"%ld",
					&adjpos[Axis]);
  				if (semTake (semMEIDC,60)!=ERROR)
  				{
			          adjpos[Axis] += (*tmaxis[Axis/2]).actual_position;
	  			  semGive (semMEIDC);
        			}
				else break;
		            	if (Axis==0)
	      			  arcsecond=(AZ_TICK*abs(adjpos[Axis]));
		            	if (Axis==2)
	      			  arcsecond=(ALT_TICK*abs(adjpos[Axis]));
		            	if (Axis/2==2)
              			  arcsecond=(ROT_TICK*abs(adjpos[Axis]));
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
				/*sscanf (buf,"%d",&pos);*/
				sscanf (buf,"%ld",
					&adjvel[Axis]);
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
				/*sscanf (buf,"%d",&pos);*/
				sscanf (buf,"%ld",
					&incvel[Axis]);
				incvel[Axis]=abs(incvel[Axis]);
				if (incvel[Axis]>10000) incvel[Axis]=10000;
				CursPos(49,19);
				printf ("%d Cts",incvel[Axis]);
			}
			CursPos(20,24);
			printf("                                        ");
			break;

         case 'T': case 't': CursPos(20,24);
				printf("Manual Trigger                          ");
				manTrg();
				break;

         case 'B': case 'b': CursPos(20,24);
/*
	 			if (STOPed[Axis]==FALSE) 
				{
				  printf("ERR: Axis is Not Stopped                ");
				  break;
				}
*/
	 			if (Axis==0)
				{
				  printf("Azimuth Brake Turned On                 ");
				  tm_az_brake_on();
				}
	 			if (Axis==2)
				{
				  printf("Altitude Brake Turned On                ");
				  tm_alt_brake_on();
				}
                                STOPed[Axis]=TRUE;
                                tm_controller_idle (Axis);
                                tm_controller_idle (Axis);
                                tm_controller_idle (Axis);
				break;
         case 'C': case 'c': CursPos(20,24);

	 			if (Axis==0)
				{
		 		  printf("Azimuth Brake Turned Off                ");
				  tm_az_brake_off();
				}
	 			if (Axis==2)
				{
				  printf("Altitude Brake Turned Off               ");
				  tm_alt_brake_off();
				}
                  Axis_vel[Axis]=0;
                  CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
                  semTake (semMEI,WAIT_FOREVER);
                  if (STOPed[Axis])
                  {
                    STOPed[Axis]=FALSE;
                    controller_run (Axis);
                    controller_run (Axis);
                    controller_run (Axis);
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
		  break;

         case '+': case '=':           CursPos(20,24);
				printf("ALIGNment Clamp Turned On               ");
				tm_clamp_on();
				break;
         case '_': case '-':           CursPos(20,24);
				printf("ALIGNment Clamp Turned Off              ");
				tm_clamp_off();
				break;
         case 'X': case 'x': Running=FALSE;
				ioctl(0,FIOOPTIONS,Options); /* back to normal */
				semTake (semMEIDC,WAIT_FOREVER);
 			        taskDelete(taskIdFigure("menuPos"));
				semGive (semMEIDC);
				taskDelay (10);
				EraseDisplayAll();
	 			break;

         case 'W': case 'w': CursPos(20,24);
       		printf("CW Inst dd; 0=CAMERA;1=FIBER; 2=EMPTY   ");
		if (GetString(&MenuInput[0],20))
		{
		  memcpy(&buf[0],&MenuInput[0],21);
		  memset(&MenuInput[0],' ',20);
		/*sscanf (buf,"%d",&pos);*/
		  sscanf (buf,"%ld",
			&inst);
		  if ((inst<0)||(inst>16)) 
		  {
		    printf("ERR: Inst Out of Range (0-16)           ");
		    break;
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
		  if (sdssdc.status.i78.il0.alt_brake_engaged)
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

         case '!': case '@': case '#': case '$':  CursPos(20,24);
		if (buf[0]=='@') cw = 2;
		else cw = buf[0]-0x20;
  		printf("CW %d pp.pp                             ",cw);
		cw--;
		if (GetString(&MenuInput[0],20))
		{
		  memcpy(&buf[0],&MenuInput[0],21);
		  memset(&MenuInput[0],' ',20);
		  sscanf (buf,"%f",&fpos);
		  CursPos(20,24);
		  if ((fpos<.1)||(fpos>24.0))
		  {
  		    printf("ERR: Position out of Range (.1-24.0)    ");
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
		  if (sdssdc.status.i78.il0.alt_brake_engaged)
		    taskSpawn ("cwp",60,VX_FP_TASK,4000,(FUNCPTR)cw_pos,
			  (int)cw,(int)&fpos,0,0,0,0,0,0,0,0);
		  else
	          {
  		    printf ("Alt Brake NOT Engaged                  ");
		    break;
		  }
		}
		CursPos(20,24);
		printf("                                        ");
		break;

         case '%':   CursPos(20,24);
                printf ("CW ABORT                               ");
	        taskDelete(taskIdFigure("cw"));
	        taskDelete(taskIdFigure("cwp"));
		cw_abort();
		break;

         case '*':   CursPos(20,24);
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
#ifdef ROT_VEL_MODE
	   	if (Axis!=4)
		{
#endif
	 	  semTake (semMEI,WAIT_FOREVER);
		  if (STOPed[Axis])
		  {
		    Axis_vel[Axis]=0;
                    STOPed[Axis]=FALSE;
                    controller_run (Axis);
                    controller_run (Axis);
                    controller_run (Axis);
	            if (Axis==4)
                      while (coeffs_state_cts (Axis,0));
	            v_move(Axis,(double)0,(double)5000);
		  }
		  else
		  {
	            while (abs(Axis_vel[Axis])>10000)
		    {
		      if (Axis_vel[Axis]>0) Axis_vel[Axis]-=10000;
		      else Axis_vel[Axis]+=10000;
	 	      set_velocity(Axis,(double)(Axis_vel[Axis]));
	              if (Axis==4)
                        while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	              CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
		      taskDelay (30);
		    }
		    Axis_vel[Axis]=0;
	 	    set_velocity(Axis,(double)(Axis_vel[Axis]));
	            if (Axis==4)
                      while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
		  }
		  semGive (semMEI);
	          CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
#ifdef ROT_VEL_MODE
		}
#endif
		break;

          case '?': 
		if (semTake (semMEIDC,60)!=ERROR)
	        {
 		  CursPos(1,1);
printf("Extended Help...*=AMP Reset; +|-=Align Close|Open; B|C=Brake Enable|Disable   \n");
		  CursPos(1,2);
printf(" D=Dest Position; O=Offset Position V=Set Velocity; A=AdjCnt Position;        \n");
		  CursPos(1,3);
printf(" M=Move to Position G=Gang Az+Alt Move                                        \n");
		  CursPos(1,4);
printf(" K=pos; J=neg; S=Stop Motion; H=Hold Motion I=Set Velocity Increment;         \n");
		  CursPos(1,5);
printf(" P=SetPosition; F=SetFiducial; sp=RstScrn X=eXit                                     \n");
		  CursPos(1,6);
printf(" W=Move CW; !|@|#|$=Move CW 1|2|3|4; %%=CW Halt                                \n");
		  semGive (semMEIDC);
	        }
		break;

	 case 'S': case 's':	     
	   Axis_vel[Axis]=0;
	   semTake(semMEI,WAIT_FOREVER);
	   CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
#ifdef ROT_VEL_MODE
	   if (Axis==4)
           {
	     start_move(Axis+1,(double)Axis_vel[Axis],(double)1000,
		 		(double)1000);
             controller_idle (Axis+1);
             controller_idle (Axis+1);
             controller_idle (Axis+1);
           }
	   else
	   {
#endif
               STOPed[Axis]=TRUE;
               controller_idle (Axis);
               controller_idle (Axis);
               controller_idle (Axis);
/*               v_move(Axis,(double)Axis_vel[Axis],(double)5000);*/
#ifdef ROT_VEL_MODE
           }
#endif
	   semGive (semMEI); 
           break;
	   
         case 'G': case 'g':  CursPos(20,24);
	   if ((adjvel[0]==0)||(adjvel[2]==0))
	   {
             printf("ERR: Velocity is Zero for AZ or Alt Axis");
	     break;
	   }
	   if (sdssdc.status.i78.il0.az_brake_engaged)
	   {
             CursPos(20,24);
	     printf("ERR: AZ Brake is Engaged                ");
	     break;
	   }
	   if (sdssdc.status.i78.il0.alt_brake_engaged)
	   {
	     CursPos(20,24);
	     printf("ERR: ALT Brake is Engaged               ");
	     break;
	   }
	   semTake(semMEI,WAIT_FOREVER);
           if (STOPed[0])
           {
             STOPed[0]=FALSE;
             controller_run (0);
             controller_run (0);
             controller_run (0);
           }
           if (STOPed[2])
           {
             STOPed[2]=FALSE;
             controller_run (2);
             controller_run (2);
             controller_run (2);
           }
	   Axis_vel[0]=0;
	   Axis_vel[2]=0;
	   start_move(0,(double)(adjpos[0]),
		(double)adjvel[0],(double)adjacc[0]);
           start_move(2,(double)(adjpos[2]),
		(double)adjvel[2],(double)adjacc[2]);
	   semGive (semMEI); 
	   break;

         case 'M': case 'm':  CursPos(20,24);
	   if (adjvel[Axis]==0)
	   {
                  printf("ERR: Velocity is Zero                   ");
	     break;
	   }
#ifdef ROT_VEL_MODE
	   if (Axis!=4)
	   {
#endif
	     semTake(semMEI,WAIT_FOREVER);
             if (STOPed[Axis])
             {
	       if ((Axis==0)&&(sdssdc.status.i78.il0.az_brake_engaged))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
		  printf("ERR: AZ Brake is Engaged                ");
		  break;
	       }
	       if ((Axis==2)&&(sdssdc.status.i78.il0.alt_brake_engaged))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
		  printf("ERR: ALT Brake is Engaged               ");
		  break;
	       }
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)adjvel[Axis]));
	       }
               STOPed[Axis]=FALSE;
               controller_run (Axis);
               controller_run (Axis);
               controller_run (Axis);
  	       Axis_vel[Axis]=0;
	       start_move(Axis,(double)(adjpos[Axis]),
			(double)adjvel[Axis],(double)adjacc[Axis]);
             }
             else
             {
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)adjvel[Axis]));
	       }
               Axis_vel[Axis]=0;
	       start_move(Axis,(double)(adjpos[Axis]),
			(double)adjvel[Axis],(double)adjacc[Axis]);
             }
	     semGive (semMEI); 
#ifdef ROT_VEL_MODE
           }
	   else
	     printf ("ERR: Cannot M with Rotator             ");
#endif
	   break;

         case 'J': case 'j':
	   Axis_vel[Axis] -= incvel[Axis];
           if (Axis_vel[Axis]<Axis_vel_neg[Axis]) Axis_vel[Axis]=Axis_vel_neg[Axis];
	   semTake(semMEI,WAIT_FOREVER);
	   CursPos(19,19); printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
#ifdef ROT_VEL_MODE
	   if (Axis==4)
           {
             controller_run (Axis+1);
             controller_run (Axis+1);
             controller_run (Axis+1);
	     start_move(Axis+1,(double)Axis_vel[Axis],(double)1000,
			(double)1000);
             if (Axis_vel[Axis]==0) 
             {
               controller_idle(Axis+1);
               controller_idle(Axis+1);
               controller_idle(Axis+1);
	     }
           }
	   else
	   {
#endif
             if (STOPed[Axis])
             {
	       if ((Axis==0)&&(sdssdc.status.i78.il0.az_brake_engaged)&&
			(Axis_vel[Axis]!=0))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
/*		  printf("1234567890123456789012345678901234567890");*/
		  printf("ERR: AZ Brake is Engaged                ");
		  break;
	       }
	       if ((Axis==2)&&(sdssdc.status.i78.il0.alt_brake_engaged)&&
			(Axis_vel[Axis]!=0))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
		  printf("ERR: ALT Brake is Engaged               ");
		  break;
	       }
               STOPed[Axis]=FALSE;
               controller_run (Axis);
               controller_run (Axis);
               controller_run (Axis);
	       v_move(Axis,(double)0,(double)5000);
               taskDelay (10);
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity(Axis,(double)(Axis_vel[Axis]));
             }
             else
             {
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis])); 
             }
#ifdef ROT_VEL_MODE
           }
#endif
           semGive (semMEI); 
	   break;

         case 'K': case 'k':
	   Axis_vel[Axis] += incvel[Axis];
           if (Axis_vel[Axis]>Axis_vel_pos[Axis]) Axis_vel[Axis]=Axis_vel_pos[Axis];
	   semTake(semMEI,WAIT_FOREVER);
	   CursPos(19,19);  printf ("<--J-- %6ld --K-->\n",Axis_vel[Axis]);
#ifdef ROT_VEL_MODE
	   if (Axis==4)
           {
             controller_run (Axis+1);
             controller_run (Axis+1);
             controller_run (Axis+1);
	     start_move(Axis+1,(double)Axis_vel[Axis],(double)1000,
			(double)1000);
             if (Axis_vel[Axis]==0) 
             {
               controller_idle (Axis+1);
               controller_idle (Axis+1);
               controller_idle (Axis+1);
             }
           }
	   else
           {
#endif
             if (STOPed[Axis])
             {
	       if ((Axis==0)&&(sdssdc.status.i78.il0.az_brake_engaged))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
		  printf("ERR: AZ Brake is Engaged");
		  break;
	       }
	       if ((Axis==2)&&(sdssdc.status.i78.il0.alt_brake_engaged))
	       {
	          semGive (semMEI); 
	          CursPos(20,24);
		  printf("ERR: ALT Brake is Engaged");
		  break;
	       }
               STOPed[Axis]=FALSE;
               controller_run (Axis);
               controller_run (Axis);
               controller_run (Axis);
               v_move(Axis,(double)0,(double)5000);
               taskDelay(10);
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis]));
             }
             else
             {
	       if (Axis==4)
	       {
                  while (coeffs_state_cts(Axis,(long)Axis_vel[Axis]));
	       }
               set_velocity (Axis,(double)(Axis_vel[Axis]));
             }
#ifdef ROT_VEL_MODE
           }
#endif
	   semGive (semMEI); 
	   break;

	   
         default:  
           if (semTake (semMEIDC,60)!=ERROR)
           {
	     refreshing=TRUE;
 	     PrintBanner(); PrintMove();
             semGive (semMEIDC);
           }
           else break;
/*
	 			printf ("Illegal Char Input=%x, %x, %x\n",
				buf[0],buf[1],buf[2]);
*/
                   		break;
      }
      if (semTake (semMEIDC,60)!=ERROR)
      {
/*        CursPos(74,23);*/
	taskDelay(1);
        semGive (semMEIDC);
      }
      taskDelay(5); /* 60/5 = 12Hz */
   }
}
static char *limitstatus[]={"  "," U"," L","UL"};
void PrintPos()
{
	extern struct SDSS_FRAME sdssdc;
	extern SEM_ID semMEIDC;
	extern int rawtick;
	extern struct TM_M68K *tmaxis[];
	extern struct FIDUCIARY fiducial[3];
	extern long fiducial_position[3];
        extern int fiducialidx[3];
        extern struct FIDUCIALS az_fiducial[];
        extern struct FIDUCIALS alt_fiducial[];
        extern struct FIDUCIALS rot_fiducial[];
        int fidsign;
	int i;
	long ap,cp,ap2;
	double arcsec, farcsec;
	int lasttick;
	long marcs,arcs,arcm,arcd;
	short adc;
	int limidx;
	extern unsigned char cwLimit;
	
	FOREVER
	{  
  	if (semTake (semMEIDC,60)!=ERROR)
  	{
	  if (refreshing) 
	  {
            taskDelay(60);
	    refreshing=FALSE;
          }
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
	    CursPos(4,9+i);
	    ap=(*tmaxis[i]).actual_position;
	    cp=(*tmaxis[i]).position,
/*	    ap2=(*tmaxis[i]).actual_position2;*/
	    ap2=(*tmaxis[i]).voltage;
       	    if (i==0)
	    {
	      arcsec=(AZ_TICK*abs(ap));
	      farcsec=(AZ_TICK*abs(az_fiducial[fiducialidx[i]].mark));
              if (az_fiducial[fiducialidx[i]].mark<0)
                fidsign=-1;
              else
                fidsign=1;
	    }
       	    if (i==1)
	    {
	      arcsec=(ALT_TICK*abs(ap));
	      farcsec=(ALT_TICK*abs(alt_fiducial[fiducialidx[i]].mark));
              if (alt_fiducial[fiducialidx[i]].mark<0)
                fidsign=-1;
              else
                fidsign=1;
	    }
       	    if (i==2)
	    {
	      arcsec=(ROT_TICK*abs(ap));
	      farcsec=(ROT_TICK*abs(rot_fiducial[fiducialidx[i]].mark));
              if (rot_fiducial[fiducialidx[i]].mark<0)
	        fidsign=-1;
	      else
	        fidsign=1;
	    }
	    arcd=(long)(arcsec)/3600;	     
	    arcm=((long)(arcsec)-(arcd*3600))/60;	     
	    arcs=((long)(arcsec)-(arcd*3600)-(arcm*60));	     
	    marcs = (arcsec-(long)arcsec)*1000;
	    if (ap<0)
	      printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	    else
	      printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
	    printf("  %10ld\t%10ld\t%10ld",
	      ap,
	      cp,
	      ap2);
	    arcd=(long)(farcsec)/3600;	     
	    arcm=((long)(farcsec)-(arcd*3600))/60;	     
	    arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
	    marcs = (farcsec-(long)farcsec)*1000;
	    if (fiducialidx[i]!=-1)
	    {
	      if (fiducial[i].markvalid) printf("V");
	      else printf(" ");
	      if (fidsign<0)
	        printf("  %3d;-%03ld:%02ld:%02ld:%03ld\n",
		fiducialidx[i],arcd,arcm,arcs,marcs);
	      else
	        printf("  %3d; %03ld:%02ld:%02ld:%03ld\n",
		fiducialidx[i],arcd,arcm,arcs,marcs);
	    }
	    else
	      printf("  No Crossing\n");
	  }

	  printf("\t\t%4.2f",
	    abs(sdssdc.status.i4.alt_position-altclino_off)*altclino_sf);
	  ap=(*tmaxis[1]).actual_position2;
	  ap2=(*tmaxis[2]).actual_position2;
	  printf("\t\tR2=%10ld\tR3=%10ld\n",
	      ap,
	      ap2);
	  printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n",
	   sdssdc.status.i11o12.ol0.clamp_engaged_st,
	   sdssdc.status.i11o12.ol0.clamp_disengaged_st,
	   sdssdc.status.o910.ol0.clamp_engage_cmd,
	   sdssdc.status.o910.ol0.clamp_disen_cmd);
	  printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n",
	   sdssdc.status.i78.il0.az_brake_engaged,
	   sdssdc.status.i78.il0.az_brake_disengaged,
	   sdssdc.status.o910.ol0.az_brake_engage_cmd,
	   sdssdc.status.o910.ol0.az_brake_disen_cmd);
	  printf("\t\tOn=%d\tOff=%d\tOnCmd=%d\tOffCmd=%d\n", 
	   sdssdc.status.i78.il0.alt_brake_engaged,
           sdssdc.status.i78.il0.alt_brake_disengaged,
           sdssdc.status.o910.ol0.alt_brake_engage_cmd,
           sdssdc.status.o910.ol0.alt_brake_disen_cmd);
	  CursPos(1,17);
          for (i=0;i<4;i++)
	  {
/*	    printf("%d\t",sdssdc.weight[i].pos);*/
	    adc=sdssdc.weight[i].pos;
            if ((adc&0x800)==0x800) adc |= 0xF000;
            else adc &= 0xFFF;
	    limidx = (cwLimit>>(i*2))&0x3;
            printf ("%4.2f\" %4.2fv%s\t",
		(24*adc)/(2048*0.7802),(10*adc)/2048.,limitstatus[limidx]);
	  }
	}
	CursPos (60,24);
	printf ("%s",&MenuInput[0]);
	CursPos (74,23);
	semGive (semMEIDC);
	taskDelay (60);
	}
}
