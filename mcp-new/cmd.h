#ifndef __CMD_H__
#define __CMD_H__			/* do only once */

/*
 * A type for holding information about a user who's connected to the MCP
 */
#define UNAME_SIZE 40			/* size for USER_BLOCK.uname */
#define UBLOCK_SIZE 200			/* size for USER_BLOCK.buff */

typedef struct {
   int pid;				/* User's PID on host machine */
   char uname[UNAME_SIZE + 1];		/* User's uname (and hostname)
					   on host machine */
   char buff[UBLOCK_SIZE + 1];		/* buffer to hold replies */
   int axis_select;			/* selected axis for this user */
   int spectrograph_select;		/* selected spectrograph */
} UBLOCK;

extern UBLOCK *ublock;			/* the user block for this user;
					   N.B.: this is a task variable */

void new_ublock(int pid, const char *uname);

/*****************************************************************************/

extern SEM_ID semCmdPort;		/* semaphore to control permission
					   to axis motions etc. */
extern char semCmdPortOwner[];		/* semCmdPort's owner */

#define CMD_TYPE_NARG 0x7		/* mask in type for number of args */
#define CMD_TYPE_VARARG 0x10		/* variable number of args */
#define CMD_TYPE_LOCKED 0x20		/* cmd is locked; needs semCmdPort */
#define CMD_TYPE_MURMUR 0x40		/* send to murmur by default */

int cmdInit(const char *msg);
char *cmd_handler(int have_semPortCmd, char *cmd, int *cmd_type);
void define_cmd(char *name, char *(*addr)(char *),
		int narg, int locked, int murmur);
void log_mcp_command(int type, const char *cmd);

int take_semCmdPort(int timeout, char *id);
int give_semCmdPort(int force);

#endif
