#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      cmd.c
**
** ABSTRACT:
**	TCC Command handler both receives, distributes, and answers on
**	behalf of the action routines.
**	cmd_handler can be invoked from the serial driver, the TCP port,
**      or the shell so access is arbitrated by a semaphore
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
** cmd_ini		public	initialize semaphore
** cmd_handler		public	command handler
** dummy_cmd		local	provides default command noop function
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
/*   File:	cmd.c							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	cmd_handler : VxWorks					*/
/*   Modules:	cmd_handler : command parser				*/
/*		cmd_init    : initialization				*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
	Command handler to parse ASCII commands and arguments and call	
 the appropriate routine with the argument count and a pointer to a list
 of arguments.
	The following are the rules-of-the-road for the parser:
	1.  All commands must be echoed.
	2.  No unsolited output
	3.  Each line of output is terminated with a <CR>
	4.  Last line of output is terminated with an " OK"
	5.  Echoed command is in upper case
	6.  Data and Error output starts on a newline from the echoed command.
--*/
/************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"
#include "stdio.h"
#include "ctype.h"
#include "semLib.h"
#include "sigLib.h"
#include "tickLib.h"
#include "inetLib.h"
#include "in.h"
#include "string.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "axis.h"
#include "cmd.h"


/*****************************************************************************/
/*
 * Wrappers for MCP commands
 */
static char *
amp_reset_cmd(char *cmd)		/* NOTUSED */
{
   mcp_amp_reset(axis_select);

   return("");
}

static char *
halt_cmd(char *cmd)			/* NOTUSED */
{
   mcp_halt(axis_select);

   return("");
}

static char *
stop_cmd(char *cmd)			/* NOTUSED */
{
   mcp_stop_axis(axis_select);

   return("");
}

static char *
set_fiducial_cmd(char *cmd)			/* NOTUSED */
{
   mcp_set_fiducial(axis_select);

   return("");
}

static char *
set_pos_cmd(char *cmd)
{
   double pos;

   if(sscanf(cmd, "%lf", &pos) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_pos(axis_select, pos);

   return("");
}

static char *
set_pos_va_cmd(char *cmd)
{
   double pos, vel, acc;

   if(sscanf(cmd, "%lf %lf %lf", &pos, &vel, &acc) != 3) {
      return("ERR: malformed command argument");
   }
   
   mcp_move_va(axis_select, pos, vel, acc);

   return("");
}

static char *
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

struct COMMANDS axis_cmds[] = {
	{"\033",reboot_cmd},
	{"AMP.RESET",amp_reset_cmd},
	{"AXIS.STATUS",axis_status_cmd},
	{"CORRECT",correct_cmd},
	{"DRIFT",drift_cmd},
	{"HALT",halt_cmd},
	{"ID",id_cmd},
	{"INIT",init_cmd},
	{"MAXACC",maxacc_cmd},
	{"MAXVEL",maxvel_cmd},
	{"MC.DUMP",mc_dump_cmd},
	{"MC.MAXACC",mc_maxacc_cmd},
	{"MC.MAXPOS",mc_maxpos_cmd},
	{"MC.MAXVEL",mc_maxvel_cmd},
	{"MC.MINPOS",mc_minpos_cmd},
	{"MOVE",move_cmd},
	{"+MOVE",plus_move_cmd},
	{"SET.POS.VA",set_pos_va_cmd},
	{"MR.DUMP",mr_dump_cmd},
	{"MS.DUMP",ms_dump_cmd},
	{"MS.MAP.DUMP",ms_map_dump_cmd},
	{"MS.MAP.LOAD",ms_map_load_cmd},
	{"MS.OFF",ms_off_cmd},
	{"MS.ON",ms_on_cmd},
	{"MS.POS.DUMP",ms_pos_dump_cmd},
	{"REMAP",remap_cmd},
	{"IR",rot_cmd},
	{"SET.LIMITS",set_limits_cmd},
	{"SET.TIME",set_time_cmd},
	{"STATS",stats_cmd},
	{"STATUS.LONG",status_long_cmd},
	{"STATUS",status_cmd},
	{"STOP",stop_cmd},
	{"TEL1",tel1_cmd},
	{"TEL2",tel2_cmd},
	{"TICKLOST @ .",ticklost_cmd},
	{"TIME?",time_cmd},
	{"BRAKE.ON",brakeon_cmd},
	{"BRAKE.OFF",brakeoff_cmd},
	{"CLAMP.ON",clampon_cmd},
	{"CLAMP.OFF",clampoff_cmd},
	{"CWMOV",cwmov_cmd},
	{"CWINST",cwinst_cmd},
	{"CWABORT",cwabort_cmd},
	{"SET.FIDUCIAL",set_fiducial_cmd},
	{"SET.POSITION",set_pos_cmd},
	{"SET.VELOCITY",set_vel_cmd},
	{"CW.STATUS",cwstatus_cmd},
	{"SP1",sp1_cmd},
	{"SP2",sp2_cmd},
	{"SLIT.CLEAR",slitclear_cmd},
	{"SLIT.CLOSE",slitclose_cmd},
	{"SLIT.OPEN",slitopen_cmd},
	{"CART.LATCH",cartlatch_cmd},
	{"CART.UNLATCH",cartunlatch_cmd},
	{"SLIT.STATUS",slitstatus_cmd},
	{"FFS.CLOSE",ffsclose_cmd},
	{"FFS.OPEN",ffsopen_cmd},
	{"FFL.ON",fflon_cmd},
	{"FFL.OFF",ffloff_cmd},
	{"NE.ON",neon_cmd},
	{"NE.OFF",neoff_cmd},
	{"HGCD.ON",hgcdon_cmd},
	{"HGCD.OFF",hgcdoff_cmd},
	{"FF.STATUS",ffstatus_cmd},
	{"AB.STATUS",abstatus_cmd},
	{"VERSION",version_cmd},
	{NULL,dummy_cmd}		/* termination */
};

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
static SEM_ID semCMD=0;
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/

/* test message buffer  */
char sdss_version[100];
int CMD_verbose=FALSE;

/*=========================================================================
**=========================================================================
**
** ROUTINE: cmd_init
**
** DESCRIPTION:
**      Initializes the semaphore
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int
cmd_init()
{
  printf ("\r\n%s\r\n",sdss_version);
  if(semCMD == 0) {
     semCMD = semMCreate (SEM_Q_FIFO);
  }

  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cmd_handler
**
** DESCRIPTION:
**      Receives commands and emits an ASCII answer.
**
** RETURN VALUES:
**      char *	answer to be sent by driver making function invocation.
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *
cmd_handler(char *cmd)
{
  int i;
  struct COMMANDS *cmd_list;
  static char *cmd_error={"ERR: CMD ERROR"};
  char *ans;

  if (CMD_verbose) {
    printf ("cmd_handler:  SDSS Cmd>>%s<<\r\n",cmd);
 }

  for(i = 0;i < strlen(cmd); i++) {
     cmd[i]=toupper(cmd[i]);		/* upper case the string */
  }
  
  semTake(semCMD, WAIT_FOREVER);
  ans=NULL;				/* ans is returned from function */
  
  while (cmd != NULL && *cmd != '\0') {
    cmd_list = &axis_cmds[0];			/* top of search list */
    while(cmd_list->command != NULL &&		/* search until found or EOL */
	  strncmp(cmd,cmd_list->command,strlen(cmd_list->command)) != 0) {
       cmd_list++;
    }
    
    if(cmd_list->command == NULL) {	/* not a valid command */
      if(CMD_verbose) {
	 printf ("cmd_handler: CMD ERROR %s %p\r\n",cmd,cmd);
	 for (i=0;i<strlen(cmd);i++) printf ("0x%02x ",cmd[i]);
      }
      semGive (semCMD);
      return cmd_error;				/* return some error string */
    }
    
    cmd += strlen(cmd_list->command);	/* skip over command */
    ans=(*cmd_list->function)(cmd);	/* call function, passing
					   remaining string */
/*
 * Look for another command; this only works if the arguments passed
 * to the previous command were numbers
 */
    cmd = strpbrk(cmd,"ABCDEFGHIJKLMNOPQRSTUVWXYZ+");
  }
  semGive(semCMD);
  
  if(CMD_verbose) {
     printf("cmd_handler:       Ans>>%s \r\n",ans);
  }
  
  return ans;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: dummy_cmd
**
** DESCRIPTION:
**      Entry for routines which are either not defined or not reasonable
**	for 2.5M.  Does nothing.
**
** RETURN VALUES:
**      char *	answer to be sent by driver making function invocation.
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *
dummy_cmd(char *cmd)
{
  printf (" DUMMY command fired\r\n");
  return "ERR: Dummy command - no action routine";
}

