#include "copyright.h"

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
#include <vxWorks.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <semLib.h>
#include <sigLib.h>
#include <tickLib.h>
#include <inetLib.h>
#include <symLib.h>
#include <sysSymTbl.h>
#include "in.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "axis.h"
#include "cmd.h"
#include "dscTrace.h"
#include "mcpFiducials.h"

/*****************************************************************************/
/*
 * Which spectrograph are we commanding?
 */
static int spectograph_select = -1;	/* -1 == ERROR  */

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
hold_cmd(char *cmd)			/* NOTUSED */

{
   mcp_hold(axis_select);

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
set_monitor_cmd(char *cmd)			/* NOTUSED */
{
   int on_off;

   if(sscanf(cmd, "%d", &on_off) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_monitor(axis_select, on_off);

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
/*
 * Commands for the selected spectrograph
 */
char *
sp1_cmd(char *cmd)
{
  spectograph_select = SPECTOGRAPH1;
  
  return("");
}

char *
sp2_cmd(char *cmd)
{
  spectograph_select = SPECTOGRAPH2;

  return("");
}

/*
 * Neither close nor open the slit door...allow it to be moved by hand
 */
static char *
slitdoor_clear_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slit_clear(spectograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * open the door
 */
static char *
slitdoor_open_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slit_open(spectograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * close the door
 */
static char *
slitdoor_close_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slit_close(spectograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * latch the slithead
 */
static char *
slithead_latch_close_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_close(spectograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * unlatch the slithead
 */
static char *
slithead_latch_open_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_open(spectograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*****************************************************************************/
/*
 * Control the T-bars
 */
static char *
tbar_latch_cmd(char *cmd)
{
   int on_off;

   if(sscanf(cmd, "%d", &on_off) != 1) {
      return("ERR: malformed command argument");
   }

   mcp_set_tbars(on_off);
   
   return("");
}

/*****************************************************************************/

struct COMMANDS axis_cmds[] = {
	{"AMP.RESET", amp_reset_cmd, 0, 1, 1},
	{"AXIS.STATUS", axis_status_cmd, 0, 0, 0},
	{"SYSTEM.STATUS", system_status_cmd, 0, 0, 0},
	{"CORRECT", correct_cmd, 0, 1, 1},
	{"DRIFT", drift_cmd, 0, 1, 1},
	{"HALT", hold_cmd, 0, 1, 1},
	{"HOLD", hold_cmd, 0, 1, 1},
	{"ID", id_cmd, 0, 0, 1},
	{"INIT", init_cmd, 0, 1, 1},
	{"MAXACC", maxacc_cmd, 1},
	{"MAXVEL", maxvel_cmd, 1},
	{"MC.DUMP", mc_dump_cmd, 0, 0, 1},
	{"MC.MAXACC", mc_maxacc_cmd, 0, 0, 1},
	{"MC.MAXPOS", mc_maxpos_cmd, 0, 0, 1},
	{"MC.MAXVEL", mc_maxvel_cmd, 0, 0, 1},
	{"MC.MINPOS", mc_minpos_cmd, 0, 0, 1},
	{"MOVE", move_cmd, -1, 1, 1},
	{"+MOVE", plus_move_cmd, -1, 1, 1},
	{"SET.POS.VA", set_pos_va_cmd, 3, 1, 1},
	{"MS.MAP.DUMP", ms_map_dump_cmd, 0, 0, 1},
	{"MS.MAP.LOAD", ms_map_load_cmd, 0, 1, 1},
	{"MS.OFF", ms_off_cmd, 0, 1, 1},
	{"MS.ON", ms_on_cmd, 0, 1, 1},
	{"SET.MONITOR", set_monitor_cmd, 1},
	{"IR", rot_cmd, 0, 0, 0},
	{"SET.LIMITS", set_limits_cmd, 2, 1, 1},
	{"SET.TIME", set_time_cmd, -1, 1, 1},
	{"STATS", stats_cmd, 0, 0, 1},
	{"STATUS.LONG", status_long_cmd, 0, 0, 1},
	{"STATUS", status_cmd, 0, 0, 1},
	{"STOP", stop_cmd, 0, 1, 1},
	{"TEL1", tel1_cmd, 0, 0, 0},
	{"TEL2",tel2_cmd, 0, 0, 0},
	{"TICKLOST @ .", ticklost_cmd, 0, 0, 1},
	{"TIME?", time_cmd, 0, 0, 1},
	{"BRAKE.ON", brakeon_cmd, 0, 1, 1},
	{"BRAKE.OFF", brakeoff_cmd, 0, 1, 1},
	{"CLAMP.ON", clampon_cmd, 0, 1, 1},
	{"CLAMP.OFF", clampoff_cmd, 0, 1, 1},
	{"CWMOV", cwmov_cmd, 2, 1, 1},
	{"CWINST", cwinst_cmd, 1, 1, 1},
	{"CWABORT", cwabort_cmd, 0, 1, 1},
	{"SET.FIDUCIAL", set_fiducial_cmd, 0, 1, 1},
	{"SET.POSITION", set_pos_cmd, 1, 1, 1},
	{"SET.VELOCITY", set_vel_cmd, 1, 1, 1},
	{"CW.STATUS", cwstatus_cmd, 0, 0, 1},
	{"SP1", sp1_cmd, 0, 0, 0},
	{"SP2", sp2_cmd, 0, 0, 0},
	{"SLITDOOR.CLEAR", slitdoor_clear_cmd, 0, 1, 1},
	{"SLITDOOR.CLOSE", slitdoor_close_cmd, 0, 1, 1},
	{"SLITDOOR.OPEN", slitdoor_open_cmd, 0, 1, 1},
	{"SLITHEADLATCH.CLOSE", slithead_latch_close_cmd, 0, 1, 1},
	{"SLITHEADLATCH.OPEN", slithead_latch_open_cmd, 0, 1, 1},
	{"SLITHEADLATCH.RET", slithead_latch_close_cmd, 0, 1, 1},
	{"SLITHEADLATCH.EXT", slithead_latch_open_cmd, 0, 1, 1},
	{"SLIT.STATUS", slitstatus_cmd, 0, 0, 1},
	{"FFS.CLOSE", ffsclose_cmd, 0, 0, 1},
	{"FFS.OPEN", ffsopen_cmd, 0, 0, 1},
	{"FF.ON", fflon_cmd, 0, 0, 1},
	{"FF.OFF", ffloff_cmd, 0, 0, 1},
	{"FFL.ON", fflon_cmd, 0, 0, 1},
	{"FFL.OFF", ffloff_cmd, 0, 0, 1},
	{"NE.ON", neon_cmd, 0, 0, 1},
	{"NE.OFF", neoff_cmd, 0, 0, 1},
	{"HGCD.ON", hgcdon_cmd, 0, 0, 1},
	{"HGCD.OFF", hgcdoff_cmd, 0, 0, 1},
	{"FF.STATUS", ffstatus_cmd, 0, 0, 1},
	{"AB.STATUS", abstatus_cmd, 2, 0, 1},
	{"TBAR.LATCH", tbar_latch_cmd, 1, 1, 1},
	{"VERSION", version_cmd, 0, 0, 1},
	{NULL, dummy_cmd, 0, 1, 1}	/* termination */
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
static SEM_ID semCMD = NULL;
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

#define USE_VX_SYMBOL_TBL 1
#if USE_VX_SYMBOL_TBL
   SYMTAB_ID cmdSymTbl = NULL;
#endif

/*
 * Set up the command interpreter
 */
int
cmd_init(void)
{
   struct COMMANDS *cmd;
     
   if(semCMD == NULL) {
      semCMD = semMCreate (SEM_Q_FIFO);
   }
#if USE_VX_SYMBOL_TBL
   if(cmdSymTbl == NULL) {
      cmdSymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(cmdSymTbl != NULL);
   }

   for(cmd = &axis_cmds[0]; cmd->command != NULL; cmd++) {
      define_cmd(cmd->command, cmd->function,
		 cmd->narg, cmd->locked, cmd->murmur);
   }
#endif
   return 0;
}

/*
 * Define a command that can be executed via the serial or TCP interface;
 * both upper and lower case is allowed (but not mixed)
 */
void
define_cmd(char *name,			/* name of command */
	   char *(*addr)(char *),	/* function to call */
	   int narg,			/* number of arguments */
	   int locked,			/* does this cmd require semCmdPort? */
	   int murmur)			/* should cmd be echoed to murmur? */
{
   int i, j;
   int status;
   SYM_TYPE type;			/* type for this symbol */

   if(narg < 0) {
      narg = 0;
      type = CMD_TYPE_VARARG;
   } else {
      assert(narg == (narg & CMD_TYPE_NARG)); /*there are enough bits in mask*/
      type = narg;
   }
   if(locked) type |= CMD_TYPE_LOCKED;
   if(murmur) type |= CMD_TYPE_MURMUR;

   for(j = 0; j < 2; j++) {
      if(j == 0) {			/* upper case the string */
         for(i = 0;i < strlen(name); i++) {
	    if(islower(name[i])) {
	       name[i] = toupper(name[i]);
	    }
	 }
      } else {				/* lower case the string */
         for(i = 0;i < strlen(name); i++) {
	    if(isupper(name[i])) {
	       name[i] = tolower(name[i]);
	    }
	 }
      }

      status = symAdd(cmdSymTbl, name, (char *)addr, type, 0);
      
      if(status != OK) {
	 TRACE(0, "Failed to add %s (%d args) to symbol table", name, narg);
      }
   }
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
cmd_handler(int have_sem,		/* we have semCmdPort */
	    char *cmd)			/* command to execute */
{
   char *ans;
#if !USE_VX_SYMBOL_TBL
   int i;
   struct COMMANDS *cmd_list;
#endif
   static char *cmd_error = "ERR: CMD ERROR";
   static char *sem_error = "ERR: I don't have the semCmdPort semaphore";
   int varargs;				/* we don't know how many arguments
					   to expect */

   if (CMD_verbose) {
      printf ("cmd_handler:  SDSS Cmd>>%s<<\r\n",cmd);
   }
   
   semTake(semCMD, WAIT_FOREVER);
   ans=NULL;				/* ans is returned from function */
   
#if USE_VX_SYMBOL_TBL
  {
     char *(*addr)(char *);		/* function pointer */
     char *args;			/* arguments for this command */
     char *cmd_str = cmd;		/* pointer to cmd; or NULL */
     int lvl;				/* level for TRACE */
     int nskip;				/* number of tokens to skip */
     char *tok;				/* a token from cmd */
     SYM_TYPE type;			/* actually number of arguments */

     nskip = varargs = 0;
     while((tok = strtok(cmd_str, " \t")) != NULL) {
	cmd_str = NULL;

	if(*tok == '\0') {		/* more than one separator char */
	   continue;
	}

	if(nskip > 0) {			/* skipping arguments to last cmd */
	   nskip--;
	   continue;
	} else if(varargs) {		/* unknown number of arguments */
	   if(*tok != '+' && !isalpha(*tok)) {
	      continue;
	   }
	}
     
	if(symFindByName(cmdSymTbl, tok, (char **)&addr, &type) != OK) {
	   TRACE(1, "Unknown command %s", tok, 0);

	   semGive(semCMD);
	   return(cmd_error);
	} else {
	   lvl = 5;
	   if(!(type & CMD_TYPE_MURMUR)) {
	      lvl += 2;
	   }
	   if(client_pid <= 0) {
	      lvl += 2;
	   }

	   TRACE((lvl + 2), "PID %d: command %s", client_pid, tok);

	   if((type & CMD_TYPE_LOCKED) && !have_sem) {
	      semGive(semCMD);
	      return(sem_error);
	   }

	   nskip = type & CMD_TYPE_NARG;
	   if(type & CMD_TYPE_VARARG) {
	      varargs = 1;
	   }
	   if(nskip == 0) {
	      args = "";
	   } else {
	      args = strtok(cmd_str, "");
	   }
	   TRACE(lvl, "Command %s: %s\n", tok, args);
	}
    
	ans = (*addr)(args);		/* actually do the work */
     }
  }
#else

  for(i = 0;i < strlen(cmd); i++) {
     cmd[i]=toupper(cmd[i]);		/* upper case the string */
  }
  
  while (cmd != NULL && *cmd != '\0') {
    cmd_list = &axis_cmds[0];			/* top of search list */
    while(cmd_list->command != NULL &&		/* search until found or EOL */
	  strncmp(cmd,cmd_list->command,strlen(cmd_list->command)) != 0) {
       cmd_list++;
    }

    {
       int lvl = 5;
    
       if(strcmp(cmd_list->command, "AXIS.STATUS") == 0 ||
	  strcmp(cmd_list->command, "IR") == 0 ||
	  strcmp(cmd_list->command, "SP1") == 0 ||
	  strcmp(cmd_list->command, "SP2") == 0 ||
	  strcmp(cmd_list->command, "TEL1") == 0 ||
	  strcmp(cmd_list->command, "TEL2") == 0 ||
	  strcmp(cmd_list->command, "SYSTEM.STATUS") == 0) {
	  lvl += 2;
       }
       if(client_pid <= 0) {
	  lvl += 2;
       } 
       TRACE(lvl, "PID %d: command %s", client_pid, cmd_list->command);
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
#endif
  semGive(semCMD);
  
  if(CMD_verbose) {
     printf("cmd_handler:       Ans>>%s \r\n",ans);
  }
  
  return ans;
}

/*****************************************************************************/
/*
 * List all available commands
 */
#if USE_VX_SYMBOL_TBL

BOOL
print_a_cmd(char *name,			/* name of command */
	    int val,			/* value of entry */
	    SYM_TYPE type,		/* type of entry */
	    int ipattern,		/* pattern to match commands, or 0 */
	    UINT16 group)		/* NOTUSED */
{
   char funcname[50] = "";		/* name of function called */
   int funcval = 0;			/* value of funcname */
   SYM_TYPE functype = 0;		/* type of function called */
   char *pattern = (ipattern == 0) ? NULL : (char *)ipattern;
   
   if(!isupper(name[1])) {		/* lower case only; +move starts '+' */
      return(TRUE);
   }

   if(pattern != NULL && strstr(name, pattern) == NULL) {
      return(TRUE);			/* pattern doesn't match */
   }

   symFindByValue(sysSymTbl, val, funcname, &funcval, &functype);
   if(val != funcval) {
      strcpy(funcname, "(static)");
   }
   
   printf("%-20s %-20s  %-4d %-6d %-10d %d\n", name, funcname,
	  (type & CMD_TYPE_NARG),
	  (type & CMD_TYPE_VARARG ? 1 : 0),
	  (type & CMD_TYPE_LOCKED ? 1 : 0),
	  (type & CMD_TYPE_MURMUR ? 1 : 0));


   return(TRUE);
}

void
cmdList(char *pattern)
{
   printf("%-20s %-20s  %-4s %-6s %-10s %s\n\n",
	  "Name", " Function", "Narg", "Vararg", "Restricted", "Print");

   symEach(cmdSymTbl, (FUNCPTR)print_a_cmd, (int)pattern);
}
#endif

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
dummy_cmd(char *cmd)			/* NOTUSED */
{
  printf (" DUMMY command fired\r\n");
  return "ERR: Dummy command - no action routine";
}

