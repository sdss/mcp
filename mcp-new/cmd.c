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
#include <stdlib.h>
#include <vxWorks.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <semLib.h>
#include <sigLib.h>
#include <taskLib.h>
#include <taskVarLib.h>
#include <tickLib.h>
#include <inetLib.h>
#include <symLib.h>
#include <sysSymTbl.h>
#include "in.h"
#include "gendefs.h"
#include "frame.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "axis.h"
#include "cmd.h"
#include "dscTrace.h"
#include "mcpFiducials.h"
#include "mcpTimers.h"
#include "mcpUtils.h"

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static const char *rebootedMsg = NULL;	/* the message to send until iacked */
static SEM_ID semCMD = NULL;

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

SYMTAB_ID cmdSymTbl = NULL;		/* our symbol table */

/*****************************************************************************/
/*
 * A command to notify us that They know we rebooted
 */
static int iacked = 0;			/* set to 0 on reboot */

char *
iack_cmd(char *cmd)			/* NOTUSED */
{
   iacked = 1;
   return("");
}

/*****************************************************************************/
/*
 * Deal with logging commands
 *
 * vxWorks fflush doesn't seem to work, at least on NFS filesystems,
 * so we reopen the file every so many lines
 */
SEM_ID semLogfile = NULL;
static int log_command_bufsize = 5;	/* max. no. of lines before flushing */
static int log_all_commands = 0;	/* should I log everything? */

void
log_mcp_command(int type,		/* type of command */
		const char *cmd)	/* command, or NULL to flush file */
{
   static FILE *mcp_log_fd = NULL;	/* fd for logfile */
   static int nline = 0;		/* number of lines written */

   if(semTake(semLogfile, WAIT_FOREVER) != OK) {
      TRACE(0, "Cannot take semLogfile %d: %s", errno, strerror(errno));
      taskSuspend(0);
   }

   if(cmd == NULL) {			/* flush log file */
      if(mcp_log_fd != NULL) {
	 fclose(mcp_log_fd); mcp_log_fd = NULL;
	 nline = 0;
      }

      semGive(semLogfile);

      return;
   }

   if(mcp_log_fd == NULL) {
      /*
       * Open logfile
       */
      char filename[100];

      sprintf(filename, "mcpCmdLog-%d.dat", mjd());
      if((mcp_log_fd = fopen_logfile(filename, "a")) == NULL) {
	 TRACE(0, "Cannot open %s: %s", filename, strerror(errno));
	 semGive(semLogfile);
	 
	 return;
      }
   }

   if(cmd != NULL &&
      (log_all_commands || (type != -1 && (type & CMD_TYPE_MURMUR)))) {
      fprintf(mcp_log_fd, "%d:%d:%s\n", time(NULL), ublock->pid, cmd);
      nline++;
   }

   if(nline >= log_command_bufsize) {
      fclose(mcp_log_fd); mcp_log_fd = NULL;
      nline = 0;
   }

   semGive(semLogfile);
}

char *
log_flush_cmd(char *cmd)		/* NOTUSED */
{
   log_mcp_command(0, NULL);		/* flush the logfile to disk */
   
   return("");
}

char *
log_bufsize_cmd(char *cmd)
{
   sprintf(ublock->buff, "%d", log_command_bufsize);

   if(*cmd != '\0') {			/* not just a query */
      log_command_bufsize = atoi(cmd);
   }
   
   return(ublock->buff);
}

char *
log_all_cmd(char *cmd)
{
   sprintf(ublock->buff, "%d", log_all_commands);

   log_all_commands = atoi(cmd);
   
   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Allocate a UBLOCK for this task
 */
static UBLOCK ublock_default = {
   -1, "default", NOINST, NOINST
};
UBLOCK *ublock = &ublock_default;	/* this task's user block; made a task
					   variable after initialisation */

void
new_ublock(int pid,
	   const char *uname)
{
   if((ublock = malloc(sizeof(UBLOCK))) == NULL ||
					 taskVarAdd(0, (int *)&ublock) != OK) {
      TRACE(0, "Failed to allocate ublock private to cpsWorkTask: %s %s",
	    errno, strerror(errno));
      taskSuspend(0);
   }
   ublock->pid = pid;
   strncpy(ublock->uname, uname, UNAME_SIZE);
   ublock->axis_select = ublock->spectrograph_select = NOINST;
}

/*****************************************************************************/
/*
 * Set up the command interpreter
 */
int
cmdInit(const char *rebootStr)		/* the command to use until iacked */
{
   rebootedMsg = rebootStr;

   if(semCMD == NULL) {
      semCMD = semMCreate(SEM_Q_FIFO);
      assert(semCMD != NULL);
   }

   if(cmdSymTbl == NULL) {
      cmdSymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(cmdSymTbl != NULL);
   }
/*
 * Create semaphore that controls access to logfile
 */
   if(semLogfile == NULL) {
      semLogfile = semMCreate(SEM_Q_FIFO);
      assert(semLogfile != NULL);
   }
/*
 * log this reboot
 */
   log_mcp_command(CMD_TYPE_MURMUR, "rebooted");
   log_mcp_command(CMD_TYPE_MURMUR, mcpVersion(NULL, 0));
/*
 * Define some misc commands that don't belong in any init function
 */
   define_cmd("IACK",         iack_cmd,      0, 0, 1);
   define_cmd("LOG.BUFSIZE",  log_bufsize_cmd, 1, 0, 1);
   define_cmd("LOG.FLUSH",    log_flush_cmd, 0, 0, 0);
   define_cmd("LOG.ALL",      log_all_cmd,   1, 0, 0);
   define_cmd("VERSION",      version_cmd,   0, 0, 0);

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
**
**      Receives commands and emits an ASCII answer.
**
** RETURN VALUES:
**      char *	answer to be sent by driver making function invocation.
*/
char *
cmd_handler(int have_sem,		/* we have semCmdPort */
	    const char *cmd,		/* command to execute */
	    int *cmd_type)		/* return type; or NULL */
{
   char *(*addr)(char *);		/* function pointer */
   char *ans;
   char *args;				/* arguments for this command */
   char cmd_copy[256 + 1];		/* modifiable copy of cmd */
   char *cmd_str = cmd_copy;		/* pointer to command; or NULL */
   static int iack_counter = -1;	/* control too many rebootedMsg
					   messages; wait before the first */
   int lvl;				/* level for TRACE */
   int nskip;				/* number of tokens to skip */
   char *tok;				/* a token from cmd */
   SYM_TYPE type;			/* actually number of arguments */
   int varargs;				/* we don't know how many arguments
					   to expect */

   strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);

   if(!iacked) {
      if(iack_counter++%100 == 0) {
	 TRACE(0, "%s",
	       (rebootedMsg == NULL ? "System has rebooted" : rebootedMsg), 0);
      }
   }

   semTake(semCMD, WAIT_FOREVER);
   ans = NULL;				/* ans is returned from function */

   nskip = varargs = 0;
   while((tok = strtok(cmd_str, " \t")) != NULL) {
      cmd_str = NULL;

      while(isspace(*tok)) tok++;	/* skip white space */
      if(*tok == '\0') {
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
	 TRACE(1, "Unknown command %s 0x%x", tok, *(int *)tok);
	 
	 semGive(semCMD);

	 if(cmd_type != NULL) {
	    *cmd_type = -1;
	 }

	 return("ERR: CMD ERROR");
      } else {
	 if(cmd_type != NULL) {
	    *cmd_type = type;
	 }
/*
 * TRACE commands if we feel like it
 */
	 lvl = 5;
	 if(!(type & CMD_TYPE_MURMUR)) {
	    lvl += 2;
	 }
	 if(ublock->pid <= 0) {
	    lvl += 2;
	 }
	 
	 TRACE((lvl + 2), "PID %d: command %s", ublock->pid, tok);
	 
	 if((type & CMD_TYPE_LOCKED) && !have_sem) {
	    semGive(semCMD);
	    return("ERR: I don't have the semCmdPort semaphore");
	 }
	 
	 nskip = type & CMD_TYPE_NARG;
	 if(type & CMD_TYPE_VARARG) {
	    varargs = 1;
	 }
	 if(nskip == 0 && !varargs) {
	    TRACE(lvl, "Command %s:", tok, 0);
	    args = "";
	 } else {
	    args = strtok(cmd_str, "");
	    if(args == NULL) {
	       args = "";
	    }
	    cmd_str = args;
	    TRACE(lvl, "Command %s: %s", tok, cmd_str);
	 }
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
   char s_funcname[50] = "";		/* space for funcname */
   char *funcname = s_funcname;		/* name of function called */
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
   if(val == funcval) {			/* found it */
      if(*funcname == '_') {
	 funcname++;			/* skip leading _ */
      }
   } else {
      funcname = "(static)";
   }
   
   printf("%-20s %-25s  %-4d %-6d %-10d %d\n", name, funcname,
	  (type & CMD_TYPE_NARG),
	  (type & CMD_TYPE_VARARG ? 1 : 0),
	  (type & CMD_TYPE_LOCKED ? 1 : 0),
	  (type & CMD_TYPE_MURMUR ? 1 : 0));


   return(TRUE);
}

void
cmdShow(char *pattern)
{
   printf("%-20s %-25s %-4s %-6s %-10s %s\n\n",
	  "Name", "Function", "Narg", "Vararg", "Restricted", "Murmur");

   symEach(cmdSymTbl, (FUNCPTR)print_a_cmd, (int)pattern);
}
