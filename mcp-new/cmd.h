#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	cmd.h							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	cmd_handler : VxWorks					*/
/*   Modules:	cmd_handler : command parser				*/
/*		cmd_init    : initialization				*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
	Utilizes cmd.h which specifies the commands.
--*/
/************************************************************************/

#ifndef __CMD_H__
/* function prototypes */
int cmd_init();
char *cmd_handler(char *cmd);
void get_ctrl(SEM_ID sem), rtn_ctrl(SEM_ID sem);

struct COMMANDS {
	char *command;
	char *(*function)();
};
/* commands will match incorrectly if the longest command is not first */
/* with a subset of the string...i.e. CORRECTLY must come befor CORRECT */
/* since CORRECTLY matches both CORRECTLY and CORRECT */
extern struct COMMANDS axis_cmds[];

#define __CMD_H__             /* do only once */

#endif	/* End __CMD_H__ */
