#include "copyright.h"
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
#include "semLib.h"
#include "sigLib.h"
#include "tickLib.h"
#include "inetLib.h"
#include "in.h"
#include "string.h"

#include "cmd.h"

/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/
#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONCE if (YES)

/* test message buffer  */

char *sdss_version = {"SDSS AXIS Version 1.0 - Charlie Briegel - 4 Jan 1996"};
SEM_ID semCMD=0;
int CMD_verbose=FALSE;

int cmd_init()
{
  printf ("\r\n%s\r\n",sdss_version);
  if (semCMD==0)
    semCMD = semMCreate (SEM_Q_FIFO);
}
char *cmd_handler(char *cmd)
{
  int i;
  struct COMMANDS *cmd_list;
  static char *cmd_error={"CMD ERROR"};
  char *ans;

  for (i=0;i<strlen(cmd);i++) cmd[i]=toupper(cmd[i]);/* upper case the string */
  if (CMD_verbose)
    printf ("cmd_handler:  SDSS Cmd>>%s<<\r\n",cmd);
/*  for (i=0;i<strlen(cmd);i++) printf ("0x%x ",cmd[i]);*/
  get_ctrl(semCMD);
  ans=NULL;					/* ans is returned from function */
  while ((*cmd!=NULL)&&(ans==NULL))
  {
    cmd_list = &axis_cmds[0];			/* top of search list */
    while ((cmd_list->command!=NULL)&&		/* search until found or EOL */
      strncmp(cmd,cmd_list->command,strlen(cmd_list->command)))
        cmd_list++;
    if (cmd_list->command==NULL)		/* not a valid command */
    {
      if (CMD_verbose)
        printf ("cmd_handler: CMD ERROR\r\n");
      rtn_ctrl(semCMD);
      return cmd_error;				/* return some error string */
    }
    cmd += strlen(cmd_list->command);		/* reposition around command */
    ans=(*cmd_list->function)(cmd);		/* call function, pass remaining string */
    cmd = strpbrk(cmd,"ABCDEFGHIJKLMNOPQRSTUVWXYZ+");/* reposition to next command */
  }
  rtn_ctrl(semCMD);
  if (CMD_verbose)
    printf ("cmd_handler:       Ans>>%s \r\n",ans);
  return ans;
}

char *dummy_cmd(char *cmd)
{
  printf (" TERMINATION command fired\r\n");
  return cmd;
}

void get_ctrl (SEM_ID sem)
{
  semTake (sem,WAIT_FOREVER);
  return;
}
void rtn_ctrl (SEM_ID sem)
{
  semGive (sem);
  return;
}
