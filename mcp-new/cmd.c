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
#include "as2.h"

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
iack_cmd(int uid, unsigned long cid,
	 char *cmd)			/* NOTUSED */
{
   iacked = 1;

   sendStatusMsg_B(uid, cid, FINISHED_CODE, 1, "needIack", 0);
   
   return("");
}

/*****************************************************************************/
/*
 * Reset the MCP board and maybe the whole VME crate
 */
char *
sys_reset_cmd(int uid, unsigned long cid, char *args)
{
   int reset_crate = 0;
   
   if(sscanf(args, "%d", &reset_crate) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "sys_reset");
      return("SYS_RESET: Please specify 0/1");
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "sys_reset");

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
   int uid = 0, cid = 0;
   MCP_CMD_MSG msg;			/* message to send */
     
   if(cmd == NULL) {			/* flush log file */
      msg.msg.type = cmdFlush_type;

      switch (msgQSend(msgCmdLog, (char *)&msg, sizeof(msg),
		       NO_WAIT, MSG_PRI_NORMAL)) {
       case OK:
	 break;
       case S_objLib_OBJ_UNAVAILABLE:
	 NTRACE(0, uid, cid, "No room on msgQueue to flush logfile");
	 break;
       default:
	 NTRACE(0, uid, cid, "Failed to flush mcpCmdLog");
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
   sprintf(msg.msg.u.cmdLog.cmd, "%ld:%d:%s\n", time(NULL), ublock->pid, cmd);

   switch (msgQSend(msgCmdLog, (char *)&msg, sizeof(msg),
		    NO_WAIT, MSG_PRI_NORMAL)) {
    case OK:
      break;
    case S_objLib_OBJ_UNAVAILABLE:
      NTRACE_2(0, uid, cid, "log_mcp_command (PID %d) no room for %s", ublock->pid, cmd);
      break;
    default:
      NTRACE(0, uid, cid, "Failed to send message to tCmdLog");
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
   int uid = 0, cid = 0;   
   MCP_CMD_MSG msg;			/* message to pass around */
   static FILE *mcp_log_fd = NULL;	/* fd for logfile */
   static int nline = 0;		/* number of lines written */
   int ret;				/* return code */

   for(;;) {
      ret = msgQReceive(msgCmdLog, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      OTRACE(8, "read msg on msgCmdLog", 0, 0);

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
	    
	    sprintf(filename, "mcpCmdLog-%d.dat", get_mjd());
	    if((mcp_log_fd = fopen_logfile(filename, "a")) == NULL) {
	       NTRACE_2(0, uid, cid, "Cannot open %s: %s", filename, strerror(errno));
	       
	       continue;
	    }
	 }

	 if(fputs(msg.msg.u.cmdLog.cmd, mcp_log_fd) == EOF) {
	    NTRACE_2(0, uid, cid, "Error logging command: %d (%s)", errno, strerror(errno));
	    NTRACE_1(0, uid, cid, "    %s", msg.msg.u.cmdLog.cmd);
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
	 NTRACE_1(0, uid, cid, "Impossible message type on msgCmdLog: %d", msg.msg.type);

	 continue;
      }
   }
}

/*****************************************************************************/

char *
log_flush_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   log_mcp_command(0, NULL);		/* flush the logfile to disk */
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "log_flush");
   
   return("");
}

char *
log_bufsize_cmd(int uid, unsigned long cid, char *cmd)
{
   sprintf(ublock->buff, "%d", log_command_bufsize);

   if(*cmd != '\0') {			/* not just a query */
      log_command_bufsize = atoi(cmd);
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "log_bufsize");
   
   return(ublock->buff);
}

char *
log_all_cmd(int uid, unsigned long cid, char *cmd)
{
   sprintf(ublock->buff, "%d", log_all_commands);

   log_all_commands = atoi(cmd);
   log_mcp_command(0, NULL);		/* flush the logfile to disk */

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "log_all");
   
   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Allocate a UBLOCK for this task
 */
static UBLOCK ublock_default = {
   -1, "default", "", NOINST, NOINST, 0, 0
};
UBLOCK *ublock = &ublock_default;	/* this task's user block; made a task
					   variable after initialisation */

void
new_ublock(int pid,
	   int uid,
	   int protocol,
	   const char *uname)
{
   int cid = 0;
   if((ublock = malloc(sizeof(UBLOCK))) == NULL ||
					 taskVarAdd(0, (int *)&ublock) != OK) {
      NTRACE_2(0, uid, cid, "Failed to allocate ublock private to cpsWorkTask: %d %s",
	       errno, strerror(errno));
      taskSuspend(0);
   }
   ublock->pid = pid;
   strncpy(ublock->uname, uname, UNAME_SIZE);
   ublock->axis_select = ublock->spectrograph_select = NOINST;
   ublock->uid = uid;
   ublock->cid = 0;
   ublock->protocol = protocol;
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
   
   ret = taskSpawn("tCmdLog",90,VX_FP_TASK,10000,(FUNCPTR)tCmdLog,
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
   define_cmd("LOG_BUFSIZE",  log_bufsize_cmd, 1, 0, 0, 1,
	      "Set the number of commands logged before the output file is\n"
	      "flushed to disk.  Returns the previous value");
   define_cmd("LOG_FLUSH",    log_flush_cmd, 0, 0, 0, 0,
	      "Flush the mcpCmdLog file to disk");
   define_cmd("LOG_ALL",      log_all_cmd,   1, 0, 0, 0,
	      "Set the level of command logging\n"
	      "   -1:  No logging\n"
	      "    0:  Log `interesting' commands\n"
	      "    1:  Log all commands\n");
   define_cmd("VERSION",      version_cmd,   0, 0, 0, 0,
	      "Return the MCP version string");
   define_cmd("SYS_RESET",    sys_reset_cmd, 1, 1, 1, 1,
	      "Reset the MCP. If the argument is true, reset the whole crate");

   return 0;
}

/*
 * Define a command that can be executed via the serial or TCP interface;
 * both upper and lower case is allowed (but not mixed)
 */
void
define_cmd(char *name,			/* name of command */
	   char *(*addr)(int, unsigned long, char *), /* function to call */
	   int narg,			/* number of arguments (< 0: vararg; -1: args are numeric) */
	   int need_sem,		/* does this cmd require semCmdPort? */
	   int may_take,		/* may this cmd take semCmdPort? */
	   int murmur,			/* should cmd be echoed to murmur? */
	   const char *doc)		/* documentation for command */
{
   int uid = 0, cid = 0;
   int i, j;
   int status;
   SYM_TYPE type;			/* type for this symbol */

   if(narg < 0) {
      type = CMD_TYPE_VARARG;
      if (narg == -2) {
	 type |= 0x1;
      }
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
	    if(islower((int)name[i])) {
	       name[i] = toupper(name[i]);
	    }
	 }
      } else {				/* lower case the string */
         for(i = 0;i < strlen(name); i++) {
	    if(isupper((int)name[i])) {
	       name[i] = tolower(name[i]);
	    }
	 }
      }

      status = symAdd(cmdSymTbl, name, (char *)addr, type, 0);
      status = symAdd(docSymTbl, name, (char *)doc, 0, 0);
      
      if(status != OK) {
	 NTRACE_2(0, uid, cid, "Failed to add %s (%d args) to symbol table", name, narg);
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
	    int uid,			/* User (i.e. connection) ID */
	    unsigned long cid,		/* Command ID */
	    const char *cmd,		/* command to execute */
	    int *cmd_type)		/* return type; or NULL */
{
   char *(*addr)(int, unsigned long, char *); /* function pointer */
   char *ans;
   char *args;				/* arguments for this command */
   char cmd_copy[256 + 1];		/* modifiable copy of cmd */
   char *cmd_str = cmd_copy;		/* pointer to command; or NULL */
   int isFirst_cmd = 1;			/* is this the first command in the input command string, cmd? */
   static int iack_counter = -1;	/* control too many rebootedMsg
					   messages; wait before the first */
   int lvl;				/* level for OTRACE */
   int nskip;				/* number of tokens to skip */
   char *tok;				/* a token from cmd */
   SYM_TYPE type;			/* actually number of arguments */
   int varargs;				/* we don't know how many arguments
					   to expect */

   strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);

   if(!iacked) {
      if(iack_counter++%100 == 0) {
	 sendStatusMsg_B(0, 0, INFORMATION_CODE, 1, "needIack", 1);
      }
   }

   semTake(semCMD, WAIT_FOREVER);
   ans = NULL;				/* ans is returned from function */

   nskip = varargs = 0;
   while((tok = strtok(cmd_str, " \t")) != NULL) {
      cmd_str = NULL;

      while(isspace((int)*tok)) tok++;	/* skip white space */
      if(*tok == '\0') {
	 continue;
      }
      
      if(varargs) {			/* unknown number of arguments */
	 if (nskip > 0) {		/* skip all tokens */
	    continue;
	 } else {
	    if(*tok != '+' && !isalpha((int)*tok)) {
	       continue;
	    }
	 }
      } else if(nskip > 0) {			/* skipping arguments to last cmd */
	 nskip--;
	 continue;
      }
      /*
       * Replace any incoming periods (e.g. HGCD.ON) with underscore (HGCD_ON).
       */
      {
	 char *ptr = tok;
	 for (; *ptr != '\0' && !isspace((int)*ptr); ++ptr) {
	    if (*ptr == '.') {
	       *ptr = '_';
	    }
	 }
      }
      /*
       * Separate commands in cmd must have different cids
       */
      if (!isFirst_cmd) {
	 cid = ++ublock->cid;
      }
      isFirst_cmd = 0;
      
      if(symFindByName(cmdSymTbl, tok, (char **)&addr, &type) != OK) {
	 NTRACE_1(1, uid, cid, "Unknown command '%s'", tok);
#if 0
	 printf("Unknown command %s %d\n", tok, strlen(tok));
#endif
	 
	 semGive(semCMD);

	 if(cmd_type != NULL) {
	    *cmd_type = -1;
	 }

	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", cmd);
	 
	 return("ERR: CMD ERROR");
      } else {
	 if(cmd_type != NULL) {
	    *cmd_type = type;
	 }
/*
 * OTRACE commands if we feel like it
 */
	 lvl = 5;
	 if(!(type & CMD_TYPE_MURMUR)) {
	    lvl += 2;
	 }
	 if(ublock->pid <= 0) {
	    lvl += 2;
	 }
	 
	 OTRACE((lvl + 2), "PID %d: command %s", ublock->pid, tok);
/*
 * If we are so requested, try to take the semCmdPort semaphore
 */
	 if(!have_sem && (type & CMD_TYPE_MAY_TAKE)) {
	    if(take_semCmdPort(60, uid) != OK) {
	       semGive(semCMD);

	       sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "needSemaphore");
	       sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", cmd);
	       return("ERR: failed to take semCmdPort semaphore");
	    }
	    
	    have_sem = have_semaphore(uid);
	 }
/*
 * Do we need the semaphore?
 */
	 if((type & CMD_TYPE_PRIV) && !have_sem) {
	    semGive(semCMD);
	    sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "needSemaphore");
	    sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", cmd);
	    
	    return("ERR: I don't have the semCmdPort semaphore");
	 }
	 
	 nskip = type & CMD_TYPE_NARG;
	 if(type & CMD_TYPE_VARARG) {
	    varargs = 1;
	 }
	 if(nskip == 0 && !varargs) {
	    OTRACE(lvl, "Command %s:", tok, 0);
	    args = "";
	 } else {
	    args = strtok(cmd_str, "");
	    if(args == NULL) {
	       args = "";
	    }
	    cmd_str = args;
	    OTRACE(lvl, "Command %s: %s", tok, cmd_str);
	 }
      }
      
      ans = (*addr)(uid, cid, args);	/* actually do the work */
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
   if(!isupper((int)name[1])) {		/* lower case only; +move starts '+' */
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
 * User-level command to show all or some iop-available commands
 */
void
cmdShow(char *pattern,			/* pattern to match, or NULL */
	int verbose)			/* if > 0, include doc strings */
{
   int i;
   char *name;				/* name of command */
   int funcval;				/* value of symbol in sysSymTbl */
   char funcname_s[100], *funcname;	/* name of function called */
   SYM_TYPE functype = 0;		/* type of function in sysSymTbl
					   (not used) */
   SYM_TYPE type = 0;			/* type of function called */
   char upattern[100];			/* upper-case version of pattern */
   char (*addr)(char *);		/* value of symbol in cmdSymTbl */
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
 * If pattern is provided, convert it to uppercase
 */
   if(*pattern == '\0') { pattern = NULL; }
   if(pattern != NULL) {
      for(i = 0; i < sizeof(upattern) - 1 && pattern[i] != '\0'; i++) {
	 upattern[i] = toupper(pattern[i]);
      }
      upattern[i] = '\0';
      pattern = upattern;
   }
/*
 * List desired commands
 */
   printf("%-20s %-25s %-4s %-6s %-8s %-5s %s\n\n",
	  "Name", "Function", "Narg", "Vararg", "Restrict", "Take?",
	  "Murmur");

   for(i = 0; i < ncommand; i++) {
      name = commands[i];

      if(pattern != NULL && strstr(name, pattern) == NULL){
	 continue;			/* pattern doesn't match */
      }

      symFindByName(cmdSymTbl, name, (char **)&addr, &type);
      symFindByValue(sysSymTbl, (UINT)addr, funcname_s, &funcval, &functype);

      funcname = funcname_s;
      if((int)addr == funcval) {	/* found it */
	 if(*funcname == '_') {
	    funcname++;			/* skip leading _ */
	 }
      } else {
	 funcname = "(static)";
      }
   
      printf("%-20s %-25s  %-4d %-6d %-8d %-5d %d\n", name, funcname,
	     (type & CMD_TYPE_NARG),
	     (type & (CMD_TYPE_VARARG) ? 1 : 0),
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
