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

#define STATIC static			/*  */

#define MAX_QUEUED_CONNECTIONS 3	/* max. number of queued connections
					   to allow */
#define MSG_SIZE 200			/* maximum size of message to read */
#define SERVER_PRIORITY 100		/* priority for server task */
#define SERVER_STACKSIZE 10000		/* stack size for server task */
/*
 * Global so that they are visible from the vxWorks shell
 */
SEM_ID semCmdPort = NULL;		/* semaphore to control permission
					   to axis motions etc. */
SEM_ID semPleaseGiveCmdPort = NULL;	/* semaphore to request its owner
					   to give up semCmdPort */

/*****************************************************************************/
/*
 * Return the task ID of the process holding a semaphore
 */
long
getSemTaskId(SEM_ID sem)
{
   if(sem == NULL) {
      return(0);
   } else {
      return((long)sem->state.owner);
   }
}

/*****************************************************************************/
/*
 * The task that does the work of reading commands and executing them
 */
int client_pid = -1;			/* Process ID of connected process */

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
   char uname[20] = "";			/* user name of connected process */

   sprintf(buff, "connected\n");
   if(write(fd, buff, strlen(buff)) == -1) {
      fprintf(stderr,"Acknowledging connection on port %d: %s",
	      port, strerror(errno));
      close(fd);
      return;
   }
   
   taskVarAdd(0, &client_pid);

   TRACE(16, "PID %d: Above fioRdString", client_pid, 0);
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
 * See if we can take semPleaseGiveCmdPort; if we cannot someone else
 * has taken it, so give the semCmdPort if we have it
 */
      if(getSemTaskId(semCmdPort) == taskIdSelf()) {
	 if(semTake(semPleaseGiveCmdPort, 0) == OK) { /* nothing to do */
	    semGive(semPleaseGiveCmdPort);
	 } else {			/* give up the semCmdPort semaphore */
	    if(semGive(semCmdPort) == ERROR) {
	       fprintf(stderr,"Unable to give semCmdPort semaphore: %s",
		       strerror(errno));
	    }
	 }
      }
/*
 * Take the sem that display.c used to take; XXX
 */
      if(semMEIUPD != NULL) {
	 if(semTake(semMEIUPD, 60) == ERROR) {
	    TRACE(0, "Cannot take semMEIUPD to process cmd %s (%d)", cmd,errno);
	    continue;
	 }
      }
/*
 * Maybe execute command
 */
      if(strncmp(cmd, "USER.ID", 7) == 0) {
	 if(strcmp(cmd, "USER.ID") == 0) {
	    sprintf(buff, "User %s PID %d", uname, client_pid);
	    reply = buff;
	 } else if(sscanf(cmd, "USER.ID %s %d", uname, &client_pid) == 2) {
	    for(ptr = uname; *ptr != '\0'; ptr++) {
	       if(isupper(*ptr)) { *ptr = tolower(*ptr); }
	    }
	    
	    reply = "Read userid/pid";
	 } else {
	    reply = "Garbled USER.ID command";
	 }
      } else if(strncmp(cmd, "SEM.TAKE", 8) == 0) {
	 TRACE(5, "PID %d: command SEM.TAKE", client_pid, 0);

	 (void)semTake(semCmdPort,60);

	 if(getSemTaskId(semCmdPort) == taskIdSelf()) {
	    reply = "took semaphore";
	 } else {
	    sprintf(buff, "Unable to take semaphore: %s",
		    strerror(errno));
	    reply = buff;
	 }
      } else if(strncmp(cmd, "SEM.STEAL", 8) == 0) {
	 TRACE(5, "PID %d: command SEM.STEAL", client_pid, 0);
	 reply = NULL;
	 
	 if(getSemTaskId(semCmdPort) != taskIdSelf()) {
	    if(semTake(semCmdPort,60) != OK) {
	       if(semTake(semPleaseGiveCmdPort, 60) == ERROR) {
		  sprintf(buff, "Unable to take semCmdPort semaphore: %s",
			  strerror(errno));
		  reply = buff;
	       } else {
		  if(semTake(semCmdPort,5*60) != OK) {
		     reply = "unable to steal semCmdPort semaphore";
		  }
		     
		  semGive(semPleaseGiveCmdPort);
	       }
	    }
	 }

	 if(getSemTaskId(semCmdPort) == taskIdSelf()) {
	    reply = "took semaphore";
	 } else {
	    if(reply == NULL) {
	       sprintf(buff, "Unable to take semCmdPort semaphore: %s",
		       strerror(errno));
	       reply = buff;
	    }
	 }
      } else if(strncmp(cmd, "SEM.GIVE", 8) == 0) {
	 int force = 0;

	 TRACE(5, "PID %d: command SEM.GIVE", client_pid, 0);

	 (void)sscanf(cmd, "SEM.GIVE %d", &force);

	 (void)semGive(semCmdPort);

	 if(getSemTaskId(semCmdPort) != taskIdSelf()) {
	    reply = "gave semaphore";
	 } else {
	    if(force) {
	       (void)semMGiveForce(semCmdPort);
	       if(semMEIUPD != NULL) {
		  (void)semMGiveForce(semMEIUPD);
	       }
	    }

	    if(getSemTaskId(semCmdPort) != taskIdSelf()) {
	       reply = "gave semaphore";
	    } else {
	       sprintf(buff, "Unable to give semaphore: %s", strerror(errno));
	    }
	    reply = buff;
	 }
      } else if(strncmp(cmd, "SEM.SHOW", 8) == 0) {
	 int full;
	 if(sscanf(cmd, "SEM.SHOW %d", &full) == 1 && full) {
	    semShow(semCmdPort, 1);
	 }

	 if(getSemTaskId(semCmdPort) == taskIdSelf()) {
	    reply = "semCmdPort=1";
	 } else {
 	    reply = "semCmdPort=0";
	 }
      } else if(strncmp(cmd, "TELNET.RESTART", 11) == 0) {
	 TRACE(5, "PID %d: command TELNET.RESTART", client_pid, 0);
	 if(taskDelete(taskIdFigure("tTelnetd")) != OK) {
	    TRACE(0, "Failed to kill tTelnetd task: %s (%d)",
		  strerror(errno), errno);
	    reply = "failed to kill tTelnetd";
	 } else {
	    telnetInit();		/* restart the telnet daemon */
	    reply = "restarted the tTelnetd";
	 }
      } else {
	 reply =
	   cmd_handler((getSemTaskId(semCmdPort) == taskIdSelf() ? 1 : 0),cmd);
      }

      if(reply == NULL) {
	 TRACE(0, "cmd_handler returns NULL for %s", cmd, 0);
	 reply = "";
      }
      TRACE(16, "PID %d: reply = %s", client_pid, reply)

      ptr = reply + strlen(reply) - 1;	/* strip trailing white space */
      while(ptr >= reply && isspace(*ptr)) {
	 *ptr-- = '\0';
      }
      
      sprintf(buff, "%s ok\n", reply);	/* OK even if buff == reply */
      if(write(fd, buff, strlen(buff)) == -1) {
	 fprintf(stderr,"Sending reply %s to command %s: %s",
		 buff, cmd, strerror(errno));
	 errno = 0;			/* we've already handled error */

	 if(semMEIUPD != NULL) {
	    semGive(semMEIUPD);
	 }

	 break;
      }

      if(semMEIUPD != NULL) {
	 semGive(semMEIUPD);
      }
   }

   if(n == ERROR && errno != 0) {
      fprintf(stderr,"Reading on port %d: %s\n", port, strerror(errno));
   }

   (void)semGive(semCmdPort);
   if(semMEIUPD != NULL) {
      (void)semGive(semMEIUPD);
   }

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
   }
   if(semPleaseGiveCmdPort == NULL) {
      if((semPleaseGiveCmdPort = semBCreate(SEM_Q_PRIORITY, SEM_FULL))
								     == NULL) {
	 fprintf(stderr,"Creating semPleaseGiveCmdPort semaphore: %s",
		 strerror(errno));
      }
   }
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
