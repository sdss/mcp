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
#include "pcdsp.h"
#include "axis.h"
#include "cmd.h"

#define STATIC				/*  */

#define MAX_QUEUED_CONNECTIONS 3	/* max. number of queued connections
					   to allow */
#define MSG_SIZE 200			/* maximum size of message to read */
#define SERVER_PRIORITY 100		/* priority for server task */
#define SERVER_STACKSIZE 10000		/* stack size for server task */

STATIC SEM_ID semCmdPort = NULL;	/* semaphore to control permission
					   to axis motions etc. */

/*****************************************************************************/
/*
 * Return 1 if the command should only be permitted to the holder of the
 * semCmdPort semaphore
 */
STATIC int
locked_command(const char *cmd)
{
   if(strcmp(cmd, "ID") == 0 ||
      strcmp(cmd, "MC.DUMP") == 0 ||
      strcmp(cmd, "MR.DUMP") == 0 ||
      strcmp(cmd, "MS.DUMP") == 0 ||
      strcmp(cmd, "MS.MAP.DUMP") == 0 ||
      strcmp(cmd, "MS.POS.DUMP") == 0 ||
      strcmp(cmd, "STATS") == 0 ||
      strcmp(cmd, "STATUS.LONG") == 0 ||
      strcmp(cmd, "STATUS") == 0 ||
      strcmp(cmd, "IR") == 0 ||
      strcmp(cmd, "TEL1") == 0 ||
      strcmp(cmd, "TEL2") == 0 ||
      strcmp(cmd, "TICKLOST @ .") == 0 ||
      strcmp(cmd, "CWSTATUS") == 0 ||
      strcmp(cmd, "SP1") == 0 ||
      strcmp(cmd, "SP2") == 0 ||
      strcmp(cmd, "SLIT.STATUS") == 0 ||
      strcmp(cmd, "FFS.CLOSE") == 0 ||
      strcmp(cmd, "FFS.OPEN") == 0 ||
      strcmp(cmd, "FFL.ON") == 0 ||
      strcmp(cmd, "FFL.OFF") == 0 ||
      strcmp(cmd, "NE.ON") == 0 ||
      strcmp(cmd, "NE.OFF") == 0 ||
      strcmp(cmd, "HGCD.ON") == 0 ||
      strcmp(cmd, "HGCD.OFF") == 0 ||
      strcmp(cmd, "FF.STATUS") == 0 ||
      strcmp(cmd, "AB.STATUS") == 0 ||
      strcmp(cmd, "VERSION") == 0) {
      return(0);
   }
   
   return(1);
}

/*****************************************************************************/
/*
 * The task that does the work of reading commands and executing them
 */
STATIC void
cpsWorkTask(int fd,			/* as returned by accept() */
	       struct sockaddr_in *client) /* the client's socket address */
{
   char buff[MSG_SIZE];			/* buffer to reply to messages */
   char cmd[MSG_SIZE];			/* buffer to read messages */
   int took_semCmdPort = 0;		/* did we take semCmdPort? */
   int n;				/* number of bytes read */
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

   while((n = fioRdString(fd, cmd, MSG_SIZE)) > 0) {
      cmd[n] = '\0';
/*
 * uppercase all commands
 */
      for(ptr = cmd; *ptr != '\0'; ptr++) {
	 if(islower(*ptr)) { *ptr = toupper(*ptr); }
      }
/*
 * Maybe execute it
 */
      if(strcmp(cmd, "SEM.TAKE") == 0) {
	 if(!took_semCmdPort && semTake(semCmdPort,60) == OK) {
	    took_semCmdPort = 1;
	 }

	 if(took_semCmdPort) {
	    reply = "took semaphore";
	 } else {
	    sprintf(buff, "Unable to take semaphore: %s",
		    strerror(errno));
	    reply = buff;
	 }
      } else if(strcmp(cmd, "SEM.GIVE") == 0) {
	 if(took_semCmdPort && semGive(semCmdPort) == OK) {
	    took_semCmdPort = 0;
	 }

	 if(!took_semCmdPort) {
	    reply = "gave semaphore";
	 } else {
	    sprintf(buff, "Unable to give semaphore: %s",
		    strerror(errno));
	    reply = buff;
	 }
      } else if(strcmp(cmd, "SEM.SHOW") == 0) {
	 semShow(semCmdPort, 1);
	 reply = "";
      } else if(!locked_command(cmd)) {
	 reply = cmd_handler(cmd);
      } else {
	 if(took_semCmdPort) {
	    reply = cmd_handler(cmd);
	 } else {
	    reply = "I don't have the semCmdPort semaphore";
	 }
      }

      assert(reply != NULL);

      ptr = reply + strlen(reply) - 1;	/* strip trailing white space */
      while(ptr >= reply && isspace(*ptr)) {
	 *ptr-- = '\0';
      }
      
      sprintf(buff, "%s ok\n", reply);	/* OK even if buff == reply */
      if(write(fd, buff, strlen(buff)) == -1) {
	 fprintf(stderr,"Sending reply %s to command %s: %s",
		 buff, cmd, strerror(errno));
	 close(fd);
	 if(took_semCmdPort) {
	    semGive(semCmdPort); took_semCmdPort = 0;
	 }
	 return;
      }
   }

   if(n == ERROR && errno != 0) {
      fprintf(stderr,"Reading on port %d: %s", port, strerror(errno));
   }

   if(took_semCmdPort) {
      semGive(semCmdPort); took_semCmdPort = 0;
   }

   close(fd);
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
 * Create a semaphore to ensure that only one user can issue axis commands
 */
   if(semCmdPort == NULL) {
      if((semCmdPort = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE)) == NULL){
	 fprintf(stderr,"Creating semaphore: %s", strerror(errno));
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

      if(taskSpawn("tTelnetCmd", SERVER_PRIORITY, 0, SERVER_STACKSIZE,
		   (FUNCPTR)cpsWorkTask, newFd, (int)&client,
					    0, 0, 0, 0, 0, 0, 0, 0) == ERROR) {
	 fprintf(stderr,"Spawning task to service connection on port %d: %s",
						 port, strerror(errno));
	 close(newFd);
      }
   }

   return(0);				/* NOTREACHED */
}
