#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      cmd.c
**
** ABSTRACT:
**	TCC Command handler both receives, distributes, and answers on
**	behalf of the action routines.
**	cmd_handler can be invoked from the serial driver or the shell
**	so access is arbitrated by a semaphore
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
/*++ Notes:
	Utilizes cmd.h which specifies the commands.
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
#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONCE if (YES)

/* test message buffer  */
char *sdss_version = {"SDSS AXIS Version 1.0 - Charlie Briegel - 4 Jan 1996"};
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
int cmd_init()
{
  printf ("\r\n%s\r\n",sdss_version);
  if (semCMD==0)
    semCMD = semMCreate (SEM_Q_FIFO);

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
char *cmd_handler(char *cmd)
{
  int i;
  struct COMMANDS *cmd_list;
  static char *cmd_error={"ERR: CMD ERROR"};
  char *ans;

  if (CMD_verbose)
    printf ("cmd_handler:  SDSS Cmd>>%s<<\r\n",cmd);
/*
  if (CMD_verbose)
  {
    printf ("\r\n");
    for (i=0;i<(strlen(cmd)+1);i++) printf ("0x%02x ",cmd[i]); printf ("\r\n");
  }
*/
  for (i=0;i<strlen(cmd);i++) cmd[i]=toupper(cmd[i]);/* upper case the string */
  semTake (semCMD,WAIT_FOREVER);
  ans=NULL;					/* ans is returned from function */
  while ((cmd!=NULL)&&(*cmd!=NULL)&&(ans==NULL))
  {
/*    printf ("0x%x ",*cmd);*/
    cmd_list = &axis_cmds[0];			/* top of search list */
    while ((cmd_list->command!=NULL)&&		/* search until found or EOL */
      strncmp(cmd,cmd_list->command,strlen(cmd_list->command)))
    {
        cmd_list++;
/*	printf("\r\ncmd_list->command=%s, len=%d",
		cmd_list->command,strlen(cmd_list->command));*/
    }
    if (cmd_list->command==NULL)		/* not a valid command */
    {
      if (CMD_verbose)
      {
        printf ("cmd_handler: CMD ERROR %s %p\r\n",cmd,cmd);
        for (i=0;i<strlen(cmd);i++) printf ("0x%02x ",cmd[i]);
      }
      semGive (semCMD);
      return cmd_error;				/* return some error string */
    }
    cmd += strlen(cmd_list->command);		/* reposition around command */
    ans=(*cmd_list->function)(cmd);		/* call function, pass remaining string */
    if (*cmd!=NULL)
      cmd = strpbrk(cmd,"ABCDEFGHIJKLMNOPQRSTUVWXYZ+");/* reposition to next command */
  }
  semGive (semCMD);
  if (CMD_verbose)
    printf ("cmd_handler:       Ans>>%s \r\n",ans);
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
char *dummy_cmd(char *cmd)
{
  printf (" TERMINATION command fired\r\n");
  return "ERR: Dummy command - no action rountine";
}
