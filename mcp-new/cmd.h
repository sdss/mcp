#include "copyright.h"

#ifndef __CMD_H__
/* function prototypes */
int cmd_init(void);
char *cmd_handler(int have_semPortCmd, char *cmd);

#if !USE_VX_SYMBOL_TBL
/*
 * commands will match incorrectly if the longest command is not first
 * with a subset of the string...i.e. CORRECTLY must come befor CORRECT
 * since CORRECTLY matches both CORRECTLY and CORRECT
 */
#endif
struct COMMANDS {
   char *command;			/* name of command */
   char *(*function)(char *);		/* function to call */
   int narg;				/* no. of arguments; -ve if variable */
   int locked;				/* does command need the semaphore? */
   int murmur;				/* should this command go to murmur? */
};

#define CMD_TYPE_NARG 0x7		/* mask in type for number of args */
#define CMD_TYPE_VARARG 0x10		/* variable number of args */
#define CMD_TYPE_LOCKED 0x20		/* cmd is locked; needs semCmdPort */
#define CMD_TYPE_MURMUR 0x40		/* send to murmur by default */

void define_cmd(char *name, char *(*addr)(char *),
		int narg, int locked, int murmur);

extern struct COMMANDS axis_cmds[];

extern int client_pid;			/* Process ID of cmdPort telnet proc */


#define __CMD_H__             /* do only once */

#endif	/* End __CMD_H__ */
