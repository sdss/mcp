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
#include "mcpMsgQ.h"
#include "mcpTimers.h"
#include "mcpUtils.h"

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static const char *rebootedMsg = NULL;	/* the message to send until iacked */
SEM_ID semCMD = NULL;

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

SYMTAB_ID cmdSymTbl = NULL;		/* our symbol table */
SYMTAB_ID docSymTbl = NULL;		/* symbol table for documentation */

/*****************************************************************************/
/*
 * A command to notify us that They know we rebooted
 */
int iacked = 0;				/* set to 0 on reboot */

char *
iack_cmd(char *cmd)			/* NOTUSED */
{
   iacked = 1;
   return("");
}

/*****************************************************************************/
/*
 * Reset the MCP board and maybe the whole VME crate
 */
char *
sys_reset_cmd(char *args)
{
   int reset_crate = 0;
   
   if(sscanf(args, "%d", &reset_crate) != 1) {
      return("SYS.RESET: Please specify 0/1");
   }

   sysReset(reset_crate);

   return("");
}

/*****************************************************************************/
/*
 * Deal with logging commands
 *
 * vxWorks fflush doesn't seem to work, at least on NFS filesystems,
 * so we reopen the file every so many lines
 */
static int log_command_bufsize = 5;	/* max. no. of lines before flushing */
static int log_all_commands = 0;	/* should I log everything? */

/*****************************************************************************/

MSG_Q_ID msgCmdLog = NULL;		/* control command logging */
typedef struct {
   MCP_MSG msg;				/* message to send */
   char buff[UBLOCK_SIZE];		/* space for command being logged */
} MCP_CMD_MSG;

void
log_mcp_command(int type,		/* type of command */
		const char *cmd)	/* command, or NULL to flush file */
{
   MCP_CMD_MSG msg;			/* message to send */
     
   if(cmd == NULL) {			/* flush log file */
      msg.msg.type = cmdFlush_type;

      switch (msgQSend(msgCmdLog, (char *)&msg, sizeof(msg),
		       NO_WAIT, MSG_PRI_NORMAL)) {
       case OK:
	 break;
       case S_objLib_OBJ_UNAVAILABLE:
	 TRACE(0, "No room on msgQueue to flush logfile", 0, 0);
	 break;
       default:
	 TRACE(0, "Failed to flush mcpCmdLog", 0, 0);
	 break;
      }

      return;
   }

   if(log_all_commands < 0) {		/* log no commands */
      return;			
   }
   
   if(log_all_commands == 0 && (type == -1 || !(type & CMD_TYPE_MURMUR))) {
      return;				/* don't log this command */
   }
/*
 * We have work to do. 
 */
   msg.msg.type = cmdLog_type;
   sprintf(msg.msg.u.cmdLog.cmd, "%d:%d:%s\n", time(NULL), ublock->pid, cmd);
   
   switch (msgQSend(msgCmdLog, (char *)&msg, sizeof(msg),
		    NO_WAIT, MSG_PRI_NORMAL)) {
    case OK:
      break;
    case S_objLib_OBJ_UNAVAILABLE:
      TRACE(0, "log_mcp_command (PID %d) no room for %s", ublock->pid, cmd);
      break;
    default:
      TRACE(0, "Failed to send message to tCmdLog", 0, 0);
      break;
   }
}

/*****************************************************************************/
/*
 * Here's the spawned task that actually does the work of writing the log file
 */
void
tCmdLog(void)
{
   MCP_CMD_MSG msg;			/* message to pass around */
   static FILE *mcp_log_fd = NULL;	/* fd for logfile */
   static int nline = 0;		/* number of lines written */
   int ret;				/* return code */

   for(;;) {
      ret = msgQReceive(msgCmdLog, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg on msgCmdLog", 0, 0);

      switch (msg.msg.type) {
       case cmdFlush_type:
	 if(mcp_log_fd != NULL) {
	    fclose(mcp_log_fd); mcp_log_fd = NULL;
	    nline = 0;
	 }
	 
	 continue;
       case cmdLog_type:
/*
 * Open logfile if needs be
 */
	 if(mcp_log_fd == NULL) {
	    char filename[100];
	    
	    sprintf(filename, "mcpCmdLog-%d.dat", mjd());
	    if((mcp_log_fd = fopen_logfile(filename, "a")) == NULL) {
	       TRACE(0, "Cannot open %s: %s", filename, strerror(errno));
	       
	       continue;
	    }
	 }

	 if(fputs(msg.msg.u.cmdLog.cmd, mcp_log_fd) == EOF) {
	    TRACE(0, "Error logging command: %d (%s)",
		  errno, strerror(errno));
	    TRACE(0, "    %s", msg.msg.u.cmdLog.cmd, 0);
	 } else {
	    nline++;
	 }
/*
 * Flush to disk?
 */
	 if(nline >= log_command_bufsize) {
	    fclose(mcp_log_fd); mcp_log_fd = NULL;
	    nline = 0;
	 }

	 break;
       default:
	 TRACE(0, "Impossible message type: %d", msg.msg.type, 0);

	 continue;
      }
   }
}

/*****************************************************************************/

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
   log_mcp_command(0, NULL);		/* flush the logfile to disk */
   
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
   int ret;				/* return code */
   
   rebootedMsg = rebootStr;

   if(semCMD == NULL) {
      semCMD = semMCreate(SEM_Q_FIFO);
      assert(semCMD != NULL);
   }

   if(cmdSymTbl == NULL) {
      cmdSymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(cmdSymTbl != NULL);
   }
   if(docSymTbl == NULL) {
      docSymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(docSymTbl != NULL);
   }
/*
 * Create message queue of commands to write to disk.  The elements of the
 * queue must be declared to be of size (sizeof(MCP_MSG) + UBLOCK_SIZE) as
 * we didn't include the full buffer size in MCP_MSG.u.cmdLog as that would
 * have made _all_ MCP_MSGs much larger.
 */
   msgCmdLog = msgQCreate(40, sizeof(MCP_CMD_MSG), MSG_Q_FIFO);
   assert(msgCmdLog != NULL);
   
   ret = taskSpawn("tCmdLog",90,0,10000,(FUNCPTR)tCmdLog,
		   0,0,0,0,0,0,0,0,0,0);
   assert(ret != ERROR);   
/*
 * log this reboot
 */
   printf("About to log the reboot\n");
   log_mcp_command(CMD_TYPE_MURMUR, "rebooted");
   log_mcp_command(CMD_TYPE_MURMUR, mcpVersion(NULL, 0));
/*
 * Define some misc commands that don't belong in any init function
 */
   define_cmd("IACK",         iack_cmd,      0, 0, 0, 1,
	      "Acknowledge an MCP reboot");
   define_cmd("LOG.BUFSIZE",  log_bufsize_cmd, 1, 0, 0, 1, "");
   define_cmd("LOG.FLUSH",    log_flush_cmd, 0, 0, 0, 0, "");
   define_cmd("LOG.ALL",      log_all_cmd,   1, 0, 0, 0, "");
   define_cmd("VERSION",      version_cmd,   0, 0, 0, 0, "");
   define_cmd("SYS.RESET",    sys_reset_cmd, 1, 0, 0, 1,
	      "Reset the MCP. If the argument is true, reset the whole crate");

   return 0;
}

/*
 * Define a command that can be executed via the serial or TCP interface;
 * both upper and lower case is allowed (but not mixed)
 */
void
define_cmd(char *name,			/* name of command */
	   char *(*addr)(char *),	/* function to call */
	   int narg,			/* number of arguments (< 0: vararg) */
	   int need_sem,		/* does this cmd require semCmdPort? */
	   int may_take,		/* may this cmd take semCmdPort? */
	   int murmur,			/* should cmd be echoed to murmur? */
	   const char *doc)		/* documentation for command */
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
   if(need_sem) type |= CMD_TYPE_PRIV;
   if(may_take) type |= CMD_TYPE_MAY_TAKE;
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
      status = symAdd(docSymTbl, name, doc, 0, 0);
      
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
/*
 * If we are so requested, try to take the semCmdPort semaphore
 */
	 if(!have_sem && (type & CMD_TYPE_MAY_TAKE)) {
	    char name[100];			/* name of taker */
	    if(1 || taskIdSelf() == taskNameToId("TCC")) {
	       sprintf(name, "%s:%d", ublock->uname, ublock->pid);
	       
	       if(take_semCmdPort(60, name) != OK) {
		  semGive(semCMD);
		  return("ERR: failed to take semCmdPort semaphore");
	       }
	    
	       have_sem = (getSemTaskId(semCmdPort) == taskIdSelf()) ? 1 : 0;
	    }
	 }
/*
 * Do we need the semaphore?
 */
	 if((type & CMD_TYPE_PRIV) && !have_sem) {
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
 * Get an alphabetical list of commands
 */
static char **commands = NULL;		/* list of all commands */
static int ncommand = 0;		/* number of commands */

BOOL
get_command_names(char *name,		/* name of command */
		  int val,		/* NOTUSED */
		  SYM_TYPE type,	/* NOTUSED */
		  int ipattern,		/* NOTUSED */
		  UINT16 group)		/* NOTUSED */
{
   if(!isupper(name[1])) {		/* lower case only; +move starts '+' */
      return(TRUE);
   }

   commands[ncommand++] = name;

   return(TRUE);
}

static int
compar(const void *a, const void *b)
{
   return(strcmp(*(char **)a, *(char **)b));
}

/*
 * User-level command
 */
void
cmdShow(char *pattern,			/* pattern to match, or NULL */
	int verbose)			/* if > 0, include doc strings */
{
   int i;
   char *name;				/* name of command */
   int funcval;				/* value of symbol in sysSymTbl */
   char *funcname = NULL;		/* name of function called */
   SYM_TYPE functype = 0;		/* type of function in sysSymTbl
					   (not used) */
   SYM_TYPE type = 0;			/* type of function called */
   int val;				/* value of symbol in cmdSymTbl */
/*
 * Find and sort all commands
 */
   if(commands == NULL) {
      int size = cmdSymTbl->nsymbols/2;	/* there are upper and lower case names
					   but we only want lower */
      commands = malloc(size*sizeof(char *));
      assert(commands != NULL);

      ncommand = 0;
      symEach(cmdSymTbl, (FUNCPTR)get_command_names, (int)0);
      assert(size == ncommand);

      qsort(commands, ncommand, sizeof(char *), compar);
   }
/*
 * List desired commands
 */
   printf("%-20s %-25s %-4s %-6s %-8s %-5s %s\n\n",
	  "Name", "Function", "Narg", "Vararg", "Restrict", "Take?",
	  "Murmur");

   for(i = 0; i < ncommand; i++) {
      name = commands[i];

      if(pattern != NULL && strstr(name, pattern) == NULL) {
	 continue;			/* pattern doesn't match */
      }

      symFindByName(cmdSymTbl, name, (char **)&val, &type);
      symFindByValue(sysSymTbl, val, funcname, &funcval, &functype);

      if(val == funcval) {		/* found it */
	 if(*funcname == '_') {
	    funcname++;			/* skip leading _ */
	 }
      } else {
	 funcname = "(static)";
      }
   
      printf("%-20s %-25s  %-4d %-6d %-8d %-5d %d\n", name, funcname,
	     (type & CMD_TYPE_NARG),
	     (type & CMD_TYPE_VARARG ? 1 : 0),
	     (type & CMD_TYPE_PRIV ? 1 : 0),
	     (type & CMD_TYPE_MAY_TAKE ? 1 : 0),
	     (type & CMD_TYPE_MURMUR ? 1 : 0));

      if(verbose > 0) {
	 char *doc;
	 if(symFindByName(docSymTbl, name, (char **)&doc, &type) == OK &&
								*doc != '\0') {
	    printf("\t%s\n", doc);
	 }
      }
   }
}
