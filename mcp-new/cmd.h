#include "copyright.h"

#ifndef __CMD_H__
/* function prototypes */
int cmd_init();
char *cmd_handler(char *cmd);
void get_ctrl(SEM_ID sem), rtn_ctrl(SEM_ID sem);

/*
 * commands will match incorrectly if the longest command is not first
 * with a subset of the string...i.e. CORRECTLY must come befor CORRECT
 * since CORRECTLY matches both CORRECTLY and CORRECT
 */
struct COMMANDS {
	char *command;
	char *(*function)();
};

extern struct COMMANDS axis_cmds[];

extern int client_pid;			/* Process ID of cmdPort telnet proc */


#define __CMD_H__             /* do only once */

#endif	/* End __CMD_H__ */
