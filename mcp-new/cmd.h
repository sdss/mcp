#ifndef __CMD_H__
#define __CMD_H__			/* do only once */

#define CMD_TYPE_NARG 0x7		/* mask in type for number of args */
#define CMD_TYPE_VARARG 0x10		/* variable number of args */
#define CMD_TYPE_LOCKED 0x20		/* cmd is locked; needs semCmdPort */
#define CMD_TYPE_MURMUR 0x40		/* send to murmur by default */


int cmdInit(void);
char *cmd_handler(int have_semPortCmd, char *cmd);
void define_cmd(char *name, char *(*addr)(char *),
		int narg, int locked, int murmur);

extern int client_pid;			/* Process ID of cmdPort telnet proc */

#endif
