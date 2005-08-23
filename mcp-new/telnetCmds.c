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

#define MAX_QUEUED_CONNECTIONS 3	/* max. number of queued connections
					   to allow */
#define MSG_SIZE 800			/* maximum size of message to read */
#define SERVER_PRIORITY 100		/* priority for server task */
#define SERVER_STACKSIZE 10000		/* stack size for server task */
/*
 * Global so that they are visible from the vxWorks shell
 */
SEM_ID semCmdPort = NULL;		/* semaphore to control permission
					   to axis motions etc. */

char semCmdPortOwner[41] = "";		/* name of owner of semCmdPort */

int
take_semCmdPort(int timeout,		/* timeout, in ticks */
		char *id)		/* name of new owner of semaphore */
{
   int ret;

   if(getSemTaskId(semCmdPort) == taskIdSelf()) {
      return(OK);			/* we've already got it */
   }
   
   ret = semTake(semCmdPort, timeout);
   
   if(ret != ERROR) {
      strncpy(semCmdPortOwner, id, sizeof(semCmdPortOwner) - 1);
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
      }
   } else {
      if((ret = semGive(semCmdPort)) != ERROR) {
	 semCmdPortOwner[0] = '\0';
      }
   }
   
   return(ret);
}

/*****************************************************************************/
/*
 * Commands to handle the semaphore
 */
char *
sem_take_cmd(char *str)			/* NOTUSED */
{
   TRACE(5, "PID %d: command SEM.TAKE", ublock->pid, 0, 0, 0);
   
   sprintf(ublock->buff, "%s:%d", ublock->uname, ublock->pid);
   (void)take_semCmdPort(60, ublock->buff);
   
   if(getSemTaskId(semCmdPort) == taskIdSelf()) {
      return("took semaphore");
   }
      
   sprintf(ublock->buff, "Unable to take semaphore owner %s: %s",
	   semCmdPortOwner, strerror(errno));
   return(ublock->buff);
}

char *
sem_steal_cmd(char *str)		/* NOTUSED */
{
   TRACE(5, "PID %d: command SEM.STEAL", ublock->pid, 0, 0, 0);
   
   if(getSemTaskId(semCmdPort) != taskIdSelf()) {
      (void)give_semCmdPort(1);
      
      sprintf(ublock->buff, "%s:%d", ublock->uname, ublock->pid);
      (void)take_semCmdPort(60, ublock->buff);
   }
   
   if(getSemTaskId(semCmdPort) == taskIdSelf()) {
      return("took semaphore");
   }

   sprintf(ublock->buff, "Unable to take semCmdPort semaphore: %s",
	   strerror(errno));
   return(ublock->buff);
}


char *
sem_give_cmd(char *str)
{
   int force = 0;
   
   TRACE(5, "PID %d: command SEM.GIVE", ublock->pid, 0, 0, 0);
   
   (void)sscanf(str, "%d", &force);
   
   (void)give_semCmdPort(0);
   
   if(getSemTaskId(semCmdPort) != taskIdSelf()) {
      return("gave semaphore");
   }

   if(force) {
      (void)give_semCmdPort(1);
   }
   
   if(getSemTaskId(semCmdPort) != taskIdSelf()) {
      return("gave semaphore");
   }

   sprintf(ublock->buff, "Unable to give semaphore: %s", strerror(errno));
   return(ublock->buff);
}

/*****************************************************************************/
/*
 * The task that does the work of reading commands and executing them
 */
void
cpsWorkTask(int fd,			/* as returned by accept() */
	    struct sockaddr_in *client) /* the client's socket address */
{
   char buff[MSG_SIZE];			/* buffer to reply to messages */
   char cmd[MSG_SIZE];			/* buffer to read messages */
   int n;				/* number of bytes read */
   int nerr = 0;			/* number of consecutive errors seen */
   const int port = ntohs(client->sin_port); /* the port they connected on */
   char *ptr;				/* utility pointer to char */
   char *reply = NULL;			/* reply to a command */

   sprintf(buff, "connected\n");
   if(write(fd, buff, strlen(buff)) == -1) {
      fprintf(stderr,"Acknowledging connection on port %d: %s",
	      port, strerror(errno));
      close(fd);
      return;
   }

   new_ublock(-1, "(telnet)");		/* task-specific UBLOCK */

   for(;;) {
      errno = 0;
      if((n = fioRdString(fd, cmd, MSG_SIZE - 1)) == ERROR) {
	 if(errno != 0) {
	    fprintf(stderr,"Reading on port %d: %s\n", port, strerror(errno));
	 }
	 if(nerr < 10 && errno != S_taskLib_NAME_NOT_FOUND &&
	    errno != S_objLib_OBJ_TIMEOUT) {
	    taskDelay(5);
	    break;
	 }
	 nerr++;
      } else if(n == 0) {
	 if(errno != 0) {
	    fprintf(stderr,"Reading on port %d: %s", port, strerror(errno));
	 }
      }
      nerr = 0;				/* number of error seen */
      
      cmd[n] = '\0';
/*
 * Maybe execute command
 */
      if(strncmp(cmd, "USER.ID", 7) == 0) {
	 int pid;
	 switch (sscanf(cmd, "USER.ID %s %d", buff, &pid)) {
	  case -1:
	  case 0:
	    sprintf(buff, "User %s:%d, TID 0x%x", ublock->uname, ublock->pid,
		    taskIdSelf());
	    reply = buff;
	    break;
	  case 2:
	    ublock->pid = pid;
	    for(ptr = buff; *ptr != '\0'; ptr++) {
	       if(isupper((int)*ptr)) { *ptr = tolower(*ptr); }
	    }
	    strncpy(ublock->uname, buff, UNAME_SIZE);

	    sprintf(buff, "%s:%d", ublock->uname, ublock->pid);
	    log_mcp_command(CMD_TYPE_MURMUR, buff);
	    
	    reply = "Read userid/pid";
	    break;
	  default:
	    reply = "Garbled USER.ID command";
	    break;
	 }
      } else if(strncmp(cmd, "SEM.SHOW", 8) == 0) {
	 int full;
	 if(sscanf(cmd, "SEM.SHOW %d", &full) == 1 && full) {
	    if(*semCmdPortOwner != '\0') {
	       printf("Owner: %s\n", semCmdPortOwner);
	    }
	    semShow(semCmdPort, 1);
	 }

	 sprintf(buff, "semCmdPort=%d, semCmdPortOwner=\"%s\"",
		 (getSemTaskId(semCmdPort) == taskIdSelf() ? 1 : 0),
		 semCmdPortOwner);

	 reply = buff;
      } else if(strncmp(cmd, "TELNET.RESTART", 11) == 0) {
	 TRACE(5, "PID %d: command TELNET.RESTART", ublock->pid, 0, 0, 0);
	 if(taskDelete(taskIdFigure("tTelnetd")) != OK) {
	    TRACE(0, "Failed to kill tTelnetd task: %s (%d)",
                  strerror(errno), errno, 0, 0);
	    reply = "failed to kill tTelnetd";
	 } else {
	    telnetdInit(10, 0);		/* [re]start the telnet daemon */
	    reply = "restarted the tTelnetd";
	 }
      } else {
	 char cmd_in[MSG_SIZE];		/* input command
					   (cmd[]'s modified by cmd_handler) */
	 int cmd_type;			/* type of command */
	 strcpy(cmd_in, cmd);

	 reply =
	   cmd_handler((getSemTaskId(semCmdPort) == taskIdSelf() ? 1 : 0),
		       cmd, &cmd_type);
	 /*
	  * write logfile of murmurable commands
	  */
	 log_mcp_command(cmd_type, cmd_in);
      }

      if(reply == NULL) {
	 TRACE(0, "cmd_handler returns NULL for %s", cmd, 0, 0, 0);
	 reply = "";
      }
      TRACE(16, "PID %d: reply = %s", ublock->pid, reply, 0, 0)

      ptr = reply + strlen(reply) - 1;	/* strip trailing white space */
      while(ptr >= reply && isspace((int)*ptr)) {
	 *ptr-- = '\0';
      }
      
      sprintf(buff, "%s ok\n", reply);	/* OK even if buff == reply */
      if(write(fd, buff, strlen(buff)) == -1) {
	 fprintf(stderr,"Sending reply %s to command %s: %s",
		 buff, cmd, strerror(errno));
	 errno = 0;			/* we've already handled error */

	 break;
      }
   }

   if(n == ERROR && errno != 0) {
      fprintf(stderr,"Reading on port %d: %s\n", port, strerror(errno));
   }

   (void)give_semCmdPort(0);

   close(fd);

   return;
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
   }
/*
 * Define semaphore commands
 */
   define_cmd("SEM.TAKE",  sem_take_cmd,   0, 0, 0, 0, "");
   define_cmd("SEM.GIVE",  sem_give_cmd,  -1, 0, 0, 0, "");
   define_cmd("SEM.STEAL", sem_steal_cmd,  0, 0, 0, 1, "");
/*
 * Loop, waiting for connection requests
 */
   for(;;) {
      if((newFd = accept(sock, (struct sockaddr *)&client, &sockaddr_size))
								       == -1) {
	 fprintf(stderr,"Accepting connection on port %d: %s",
						 port, strerror(errno));
	 close(sock);
	 return(-1);
      }

      if(taskSpawn("tTelnetCmd", SERVER_PRIORITY, VX_FP_TASK, SERVER_STACKSIZE,
		   (FUNCPTR)cpsWorkTask, newFd, (int)&client,
					    0, 0, 0, 0, 0, 0, 0, 0) == ERROR) {
	 fprintf(stderr,"Spawning task to service connection on port %d: %s",
						 port, strerror(errno));
	 close(newFd);
      }
   }

   return(0);				/* NOTREACHED */
}
