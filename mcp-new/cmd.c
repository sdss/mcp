#include "copyright.h"

/*
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

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static SEM_ID semCMD = NULL;

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

/* test message buffer  */
char sdss_version[100];

SYMTAB_ID cmdSymTbl = NULL;

/*
 * Set up the command interpreter
 */
int
cmdInit(void)
{
   if(semCMD == NULL) {
      semCMD = semMCreate (SEM_Q_FIFO);
   }

   if(cmdSymTbl == NULL) {
      cmdSymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(cmdSymTbl != NULL);
   }
/*
 * Define some misc commands that done belong in any init function
 */
   define_cmd("SET.TIME",     set_time_cmd, -1, 1, 1);
   define_cmd("TICKLOST @ .", ticklost_cmd,  0, 0, 1);
   define_cmd("TIME?",        time_cmd,      0, 0, 1);
   define_cmd("VERSION",      version_cmd,   0, 0, 1);

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
   char *(*addr)(char *);		/* function pointer */
   char *ans;
   char *args;				/* arguments for this command */
   char *cmd_str = cmd;			/* pointer to cmd; or NULL */
   static char *cmd_error = "ERR: CMD ERROR";
   int lvl;				/* level for TRACE */
   int nskip;				/* number of tokens to skip */
   static char *sem_error = "ERR: I don't have the semCmdPort semaphore";
   char *tok;				/* a token from cmd */
   SYM_TYPE type;			/* actually number of arguments */
   int varargs;				/* we don't know how many arguments
					   to expect */

   semTake(semCMD, WAIT_FOREVER);
   ans=NULL;				/* ans is returned from function */
   

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
   
  semGive(semCMD);
  
  return ans;
}

/*****************************************************************************/
/*
 * List all available commands
 */
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

