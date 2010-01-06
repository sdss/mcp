/*
 * Run a server listening to a port for commands from the unix host
 */
#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>
#include <sys/socket.h>
#include <in.h>
#include <fioLib.h>
#include <inetLib.h>
#include <ioLib.h>
#include <sockLib.h>
#include <taskLib.h>
#include <taskVarLib.h>
#include <telnetLib.h>
#include <usrLib.h>
#include "pcdsp.h"
#include "axis.h"
#include "cmd.h"
#include "dscTrace.h"
#include "mcpUtils.h"
#include "as2.h"

#define MAX_QUEUED_CONNECTIONS 3	/* max. number of queued connections
					   to allow */
#define SERVER_PRIORITY 100		/* priority for server task */
#define SERVER_STACKSIZE 20000		/* stack size for server task */
/*
 * Global so that they are visible from the vxWorks shell
 */
SEM_ID semCmdPort = NULL;		/* semaphore to control permission
					   to axis motions etc. */
SEM_ID semNewUID = NULL;		/* semaphore to control setting the user's UIDs */

char semCmdPortOwner[56] = "";		/* name of owner of semCmdPort */
int semUid = -1;                        /* The uid that has the semaphore; only unique within a task */


/*****************************************************************************/

int
have_semaphore(int uid)
{
   if (getSemTaskId(semCmdPort) != taskIdSelf()) {
      return 0;
   }

   return (uid == semUid) ? 1 : 0;
}

int
take_semCmdPort(int timeout,		/* timeout, in ticks */
		int uid)		/* UID of new owner */
{
   int ret;
   
   if(have_semaphore(uid)) {
      return(OK);			/* we've already got it */
   }
   
   ret = semTake(semCmdPort, timeout);
   
   if(ret != ERROR) {
      sprintf(ublock->buff, "%s:%d:%d", ublock->uname, ublock->pid, uid);
      assert(strlen(ublock->buff) < UBLOCK_SIZE);
      
      strncpy(semCmdPortOwner, ublock->buff, sizeof(semCmdPortOwner) - 1);
      semUid = uid;

      sendStatusMsg_S(0, 0, INFORMATION_CODE, 1, "semaphoreOwner", semCmdPortOwner);
   }

   return(ret);
}

int
give_semCmdPort(int force)		/* force the giving? */
{
   int ret;

   if(force) {
      if((ret = semMGiveForce(semCmdPort)) != ERROR) {
	 semCmdPortOwner[0] = '\0';
	 semUid = -1;
      }
   } else {
      if((ret = semGive(semCmdPort)) != ERROR) {
	 semCmdPortOwner[0] = '\0';
	 semUid = -1;
      }
   }
   
   return(ret);
}

/*****************************************************************************/
/*
 * Commands to handle the semaphore
 */
char *
sem_take_cmd(int uid, unsigned long cid,
	     char *str)
{
   (void)take_semCmdPort(60, uid);
   
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "haveSemaphore", have_semaphore(uid));

   if (have_semaphore(uid)) {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "sem_take");

      return "took semaphore";
   }

   sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "sem_take");
   
   sprintf(ublock->buff, "Unable to take semaphore owner %s: %s", semCmdPortOwner, strerror(errno));
   return ublock->buff;
}

char *
sem_steal_cmd(int uid, unsigned long cid,
	      char *str)		/* NOTUSED */
{
   char *reply = "";
   
   if (!have_semaphore(uid)) {
      (void)give_semCmdPort(1);
      
      (void)take_semCmdPort(60, uid);
   }
   
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "haveSemaphore", have_semaphore(uid));
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "semaphoreOwner", semCmdPortOwner);

   if(have_semaphore(uid)) {
      if (ublock->protocol == OLD_PROTOCOL) {
	 reply = "took semaphore";
      } else {
	 sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "sem_steal");
      }
   } else {
      if (ublock->protocol == OLD_PROTOCOL) {
	 sprintf(ublock->buff, "Unable to take semaphore owner %s: %s", semCmdPortOwner, strerror(errno));
	 reply = ublock->buff;
      } else {
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "sem_steal");
      }
   }

   return reply;
}

char *
sem_give_cmd(int uid, unsigned long cid, char *str)
{
   char *reply = "";
   int force = 0;
   
   (void)sscanf(str, "%d", &force);
   
   (void)give_semCmdPort(0);
   
   if(have_semaphore(uid) && force) {
      (void)give_semCmdPort(1);
   }
   
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "haveSemaphore", have_semaphore(uid));
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "semaphoreOwner", semCmdPortOwner);

   if(!have_semaphore(uid)) {
      if (ublock->protocol == OLD_PROTOCOL) {
	 reply = "gave semaphore";
      } else {
	 sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "sem_give");
      }
   } else {
      if (ublock->protocol == OLD_PROTOCOL) {
	 reply = "Unable to give semaphore";
      } else {
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "sem_give");
      }
   }

   return reply;
}

char *
sem_show_cmd(int uid, unsigned long cid, char *cmd)
{
   int full;
   if(sscanf(cmd, "SEM_SHOW %d", &full) == 1 && full) {
      if(*semCmdPortOwner != '\0') {
	 printf("Owner: %s\n", semCmdPortOwner);
      }
      semShow(semCmdPort, 1);
   }

   if (ublock->protocol == OLD_PROTOCOL) {
      sprintf(ublock->buff, "semCmdPort=%d, semCmdPortOwner=\"%s\"", have_semaphore(uid), semCmdPortOwner);
      
      return ublock->buff;
   }

   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "haveSemaphore", have_semaphore(uid));
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "semaphoreOwner", semCmdPortOwner);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "sem_show");

   return "";
}

/********************************************************************************/

char *
user_id_cmd(int uid, unsigned long cid, char *cmd)
{
   char buff[MSG_SIZE];			/* buffer to reply to messages */
   int pid;
   char *ptr;

   switch (sscanf(cmd, "%s %d", buff, &pid)) {
    case -1:
    case 0:
      sprintf(ublock->buff, "User %s:%d, TID 0x%x", ublock->uname, ublock->pid, taskIdSelf());

      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "userId", ublock->buff);
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "user_id");

      return ublock->buff;
    case 2:
      ublock->pid = pid;
      for(ptr = cmd; *ptr != '\0'; ptr++) {
	 if(isupper((int)*ptr)) { *ptr = tolower(*ptr); }
      }
      strncpy(ublock->uname, buff, UNAME_SIZE);
      
      sprintf(ublock->buff, "%s:%d", ublock->uname, ublock->pid);
      log_mcp_command(CMD_TYPE_MURMUR, ublock->buff);
      
      sprintf(ublock->buff, "User %s:%d, TID 0x%x", ublock->uname, ublock->pid, taskIdSelf());
      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "userId", ublock->buff);
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "user_id");

      return "Read userid/pid";
    default:
      if (ublock->protocol == OLD_PROTOCOL) {
	 return "Garbled USER_ID command";
      }
      
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badUserId");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "user_id");
      return "";
   }
   /* NOTREACHED */
}

/*****************************************************************************/
/*
 * The task that does the work of reading commands and executing them
 */
void retireUid(int uid);

void
cpsWorkTask(int fd,			/* as returned by accept() */
	    struct sockaddr_in *client, /* the client's socket address */
	    int uid,			/* the client's user ID */
	    int protocol)		/* the protocol the connection uses */
{
   char buff[MSG_SIZE];			/* buffer to reply to messages */
   char cmd_s[MSG_SIZE];		/* buffer to read messages */
   char *cmd;				/* pointer to cmd_s buffer */
   int n;				/* number of bytes read */
   int nerr = 0;			/* number of consecutive errors seen */
   const int port = ntohs(client->sin_port); /* the port they connected on */
   char *ptr;				/* utility pointer to char */
   char *reply = NULL;			/* reply to a command */

   new_ublock(-1, uid, protocol, "(telnet)"); /* task-specific UBLOCK */
   /*
    * This is all we know about the newly-connected user
    */
   if (ublock->protocol == OLD_PROTOCOL) {
      sprintf(buff, "connected\n");
      if(write(fd, buff, strlen(buff)) == -1) {
	 NTRACE_2(0, uid, 0, "telnet acking connection on port %d: %s", port, strerror(errno));
	 close(fd);
	 return;
      }
   } else {
      sendStatusMsg_FD(uid, taskIdSelf(), INFORMATION_CODE, 0, fd);
      sendStatusMsg_I(uid, 0, INFORMATION_CODE, 0, "yourUserNum", uid);
   }
   sprintf(ublock->buff, "User %s:%d, TID 0x%x", ublock->uname, ublock->pid, taskIdSelf());
   sendStatusMsg_S(uid, 0, INFORMATION_CODE, 1, "userId", ublock->buff);

   for(;;) {
      int cid = 0;			/* Command ID */
      char cmd_in[MSG_SIZE];            /* input command (cmd[]'s modified by cmd_handler) */
      int cmd_type;			/* type of command */

      errno = 0;
      cmd = cmd_s;
      if((n = fioRdString(fd, cmd, MSG_SIZE - 1)) == ERROR) {
	 if(errno != 0) {
	    printf("uid=%d cid=%d fd=%d telnet reading on port %d: %s", uid, cid, fd, port, strerror(errno));
	    NTRACE_2(0, uid, cid, "telnet reading on port %d: %s", port, strerror(errno));
	 }
	 if(nerr < 10 && !(errno == S_taskLib_NAME_NOT_FOUND || errno == S_objLib_OBJ_TIMEOUT)) {
	    break;
	 }

	 nerr++;
#if 0
	 taskDelay(5);
	 continue;
#endif
      } else if(n == 0) {
	 if(errno != 0) {
	    NTRACE_2(0, uid, cid, "telnet reading (2) on port %d: %s", port, strerror(errno));
	 }
      }
      nerr = 0;				/* number of error seen */
      
      cmd[n] = '\0'; n--;
      ptr = cmd + n - 1;		/* strip trailing white space */
      while(ptr >= cmd && isspace((int)*ptr)) {
	 *ptr-- = '\0';
      }

      NTRACE_1(5, uid, cid, "new telnet cmd: %s", cmd);
      /*
       * Modern clients send commands that look like "userId commandID commands [arg1 ...]";
       * If uid/cid aren't available we'll invent them
       */
      {
	 int _uid = 0;			/* User ID in command */
	 if (sscanf(cmd, "%d %d ", &_uid, &cid) == 2) { /* uid/cid are present */
#if 0					/* don't complain; different connections to the hub use different UIDs  */
	    if (_uid != ublock->uid) {
	       sendStatusMsg_I(uid, cid, WARNING_CODE, 0, "badUid", ublock->uid);
	    }
#endif
	    uid = ublock->uid = _uid;

	    while (*cmd != '\0' && !isdigit((int)*cmd)) {
	       sendStatusMsg_I(uid, cid, WARNING_CODE, 0, "badCharacter", *cmd);
	       cmd++;
	    }

	    while (isdigit((int)*cmd)) cmd++; /* skip uid */
	    while (isspace((int)*cmd)) cmd++;
	    while (isdigit((int)*cmd)) cmd++; /* skip cid */
	    while (isspace((int)*cmd)) cmd++;
	 } else {
	    uid = ublock->uid;
	    cid = ++ublock->cid;
	 }
      }
/*
 * Maybe execute command
 */
      strncpy(cmd_in, cmd, MSG_SIZE - 1); cmd_in[MSG_SIZE - 1] = '\0';
      
      reply = cmd_handler(have_semaphore(uid), uid, cid, cmd, &cmd_type);
      /*
       * write logfile of murmurable commands
       */
      log_mcp_command(cmd_type, cmd_in);

      if(reply == NULL) {
	 reply = "";
      }
      NTRACE_2(16, uid, cid, "PID %d: reply = %s", ublock->pid, reply);

      ptr = reply + strlen(reply) - 1;	/* strip trailing white space */
      while(ptr >= reply && isspace((int)*ptr)) {
	 *ptr-- = '\0';
      }
      
      if (ublock->protocol == OLD_PROTOCOL) {
	 sprintf(buff, "%s ok\n", reply);	/* OK even if buff == reply */
	 if(write(fd, buff, strlen(buff)) == -1) {
	    fprintf(stderr,"Sending reply %s to command %s: %s",
		    buff, cmd, strerror(errno));
	    errno = 0;			/* we've already handled error */

	    break;
	 }
      } else {
	 if (reply[0] == '\0') {
	    ;
	 } else {
	    if (cmd_type >= 0) {
#if 0					/* It's the command's job to send this if they deem it interesting */
	       sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "text", reply);
#endif
	    }
	 }
      }
   }

   if(n == ERROR && errno != 0) {
      fprintf(stderr,"Reading on port %d: %s\n", port, strerror(errno));
   }

   (void)give_semCmdPort(0);

   if (ublock->protocol == NEW_PROTOCOL) {
      sprintf(ublock->buff, "User %s:%d, TID -1", ublock->uname, ublock->pid);
      sendStatusMsg_S(uid, 0, INFORMATION_CODE, 1, "userId", ublock->buff);
      sendStatusMsg_FD(uid, 0, FINISHED_CODE, 0, fd);
   }
   retireUid(uid);
   close(fd);

   return;
}
/*
 * These port numbers are set in etc/mcp.login
 */
int oldServerPort = -1;			/* the port we've always connected to (e.g. mcpMenu) */
int newServerPort = -1;			/* the new port that the SDSS-III hub connects to  */

/*
 * Return a new UID, guaranteed to be unique
 */
static int uidsInUse[MAX_CONNECTIONS];	/* all the UIDs that we know about */
/*
 * Return a unique new ID.  We increment up to MAX_CONNECTIONS then start again near 0
 */
int newUid(void)
{
   int i;
   static int uid = -1;				/* our new UID */

   if (semTake(semNewUID, 10*60) == ERROR) {
      sendStatusMsgUrgent_S(0, 0, FATAL_CODE, 1, "text", "Unable to take semNewUID");
      return -1;
   }
   
   if (uid < 0) {
      /* User IDs for connections
	 0: spontaneous
	 1: TCC
	 2: internal recursive calls (== INTERNAL_UID)
      */
      for (i = 0; i <= INTERNAL_UID; ++i) {
	 uidsInUse[i] = 1;
      }
      for (; i < MAX_CONNECTIONS; ++i) {
	 uidsInUse[i] = 0;
      }

      uid = INTERNAL_UID;		/* last allocated uid */
   }
   /*
    * Look for an unused UID
    */
   if (++uid == MAX_CONNECTIONS) {
      uid = INTERNAL_UID;
   }

   for (; uid < MAX_CONNECTIONS; ++uid) {
      if (!uidsInUse[uid]) {
	 uidsInUse[uid] = 1;
	 printf("Returning UID == %d\n", uid);
	 semGive(semNewUID);

	 return uid;
      }
   }
   semGive(semNewUID);

   sendStatusMsgUrgent_S(0, 0, FATAL_CODE, 1, "text", "All UIDs are in use");

   return -1;
}

/*
 * Make uid available for reuse
 */
void
retireUid(int uid)
{
   assert(uid >= 0 && uid < MAX_CONNECTIONS);
   if (!uidsInUse[uid]) {
      sendStatusMsg_I(0, 0, INFORMATION_CODE, 1, "invalidRetireUid", uid);
      
   }
   printf("Retiring UID == %d\n", uid);
   uidsInUse[uid] = 0;
}

void
showUids(int max)			/* maximum number of UIDs to show */
{
   int i;
   int nactive = 0;

   if (max <= 0) {
      max = MAX_CONNECTIONS;
   }

   printf("UID InUse?\n");
   for (i = 0; i !=  MAX_CONNECTIONS; ++i) {
      if (i < max) {
	 printf("%-3d %d\n", i, uidsInUse[i]);
      }

      nactive += uidsInUse[i];
   }
   printf("%d active connections\n");
}

/*
 * Return the next cid for internal calls (i.e. uid == INTERNAL_UID)
 */
int nextInternalCid(void)
{
   static int cid = 0;
   return ++cid;
}


STATUS
cmdPortServer(int port)			/* port to bind to */
{
   int i;
   struct sockaddr_in client;		/* client's address */
   struct sockaddr_in server;		/* server's address */
   int sock;				/* server socket */
   int sockaddr_size = sizeof(client);	/* const, but passed by address */
   int newFd;				/* new f.d. from accept() */
/*
 * setup local address
 */
   memset(&server, sizeof(server), '\0');
   server.sin_family = PF_INET;
   server.sin_port = htons(port);
   server.sin_addr.s_addr = htonl(INADDR_ANY);
/*
 * create socket
 */
   if((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
      fprintf(stderr,"Opening server on port %d: %s",
						 port, strerror(errno));
      return(-1);
   }

   i = 1;
   if(setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&i, sizeof(i)) < 0)
     perror("reuse");
   if(setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, (char *)&i, sizeof(i)) < 0)
     perror("keepalive");
   {
      int sbufSize = 32768;
      if (setsockopt (sock, SOL_SOCKET, SO_SNDBUF, (char *)&sbufSize, sizeof (sbufSize)) < 0) {
	 perror("sndbuf");
      }
   }
/*
 * bind to local address
 */
   if(bind(sock, (struct sockaddr *)&server, sizeof(server)) == -1) {
      fprintf(stderr,"Binding server on port %d: %s",
						 port, strerror(errno));
      close(sock);
      return(-1);
   }
/*
 * Prepare to start accepting connections
 */
   if(listen(sock, MAX_QUEUED_CONNECTIONS) == -1) {
      fprintf(stderr,"Listening on port %d: %s", port, strerror(errno));
      close(sock);
      return(-1);
   }
/*
 * Create a semaphore to ensure that only one user can issue axis commands,
 * and another that can be used to request that the current holder relinquish
 * the semCmdPort semaphore
 */
   if(semCmdPort == NULL) {
      if((semCmdPort = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE)) == NULL){
	 fprintf(stderr,"Creating semCmdPort semaphore: %s", strerror(errno));
      }
      *semCmdPortOwner = '\0';
      semUid = -1;

      /*
       * Define semaphore commands
       */
      define_cmd("SEM_TAKE",  sem_take_cmd,   0, 0, 0, 0, "Steal the MCP semaphore if available");
      define_cmd("SEM_GIVE",  sem_give_cmd,  -1, 0, 0, 0, "Release the MCP semaphore");
      define_cmd("SEM_SHOW",  sem_show_cmd,   1, 0, 0, 0, "Tell me who has the MCP semaphore");
      define_cmd("SEM_STEAL", sem_steal_cmd,  0, 0, 0, 1, "Steal the MCP semaphore");

      define_cmd("USER_ID",  user_id_cmd,    -2, 0, 0, 0, "Tell the MCP who you are");
   }
/*
 * Create a semaphore to control access to getting a new UID
 */
   if(semNewUID == NULL) {
      if((semNewUID = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE)) == NULL){
	 fprintf(stderr,"Creating semNewUID semaphore: %s", strerror(errno));
      }
   }
/*
 * Loop, waiting for connection requests
 */
   for(;;) {
      int uid;
      if((newFd = accept(sock, (struct sockaddr *)&client, &sockaddr_size))
								       == -1) {
	 fprintf(stderr,"Accepting connection on port %d: %s",
						 port, strerror(errno));
	 close(sock);
	 return(-1);
      }

      uid = newUid();
      if (uid < 0) {			/* none available */
	 close(newFd);
	 close(sock);
	 continue;
      }

      if(taskSpawn("tTelnetCmd", SERVER_PRIORITY, VX_FP_TASK, SERVER_STACKSIZE,
		   (FUNCPTR)cpsWorkTask, newFd, (int)&client, uid,
		   (port == newServerPort ? NEW_PROTOCOL : OLD_PROTOCOL),
		   0, 0, 0, 0, 0, 0) == ERROR) {
	 fprintf(stderr,"Spawning task to service connection on port %d: %s",
						 port, strerror(errno));
	 close(newFd);
      }
   }

   return(0);				/* NOTREACHED */
}
