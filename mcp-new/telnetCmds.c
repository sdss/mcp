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
   if(strcmp(cmd, "VERSION") == 0) {
      return(1);
   }
   
   return(0);
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
	 if(semTake(semCmdPort,60) == ERROR) {
	    sprintf(buff, "Unable to take semaphore: %s ok\n",
		    strerror(errno));
	    if(write(fd, buff, strlen(buff)) == -1) {
	       fprintf(stderr,"Sending reply %s to command %s: %s",
		       buff, cmd, strerror(errno));
	       close(fd);
	       (void)semGive(semCmdPort);
	       return;
	    }
	 }
	 reply = "";
      } else if(strcmp(cmd, "SEM.GIVE") == 0) {
	 semGive(semCmdPort);
	 reply = "";
      } else if(strcmp(cmd, "SEM.SHOW") == 0) {
	 semShow(semCmdPort, 1);
	 reply = "";
      } else if(!locked_command(cmd)) {
	 reply = cmd_handler(cmd);
      } else {
	 if(semTake(semCmdPort,60) == ERROR) {
	    sprintf(buff, "Unable to take semaphore for %s: %s ok\n",
		    reply, strerror(errno));
	    if(write(fd, buff, strlen(buff)) == -1) {
	       fprintf(stderr,"Sending reply %s to command %s: %s",
		       buff, cmd, strerror(errno));
	       close(fd);
	       (void)semGive(semCmdPort);
	       return;
	    }
	 } else {
	    reply = cmd_handler(cmd);
	 }
      }

      assert(reply != NULL);

      ptr = reply + strlen(reply) - 1;	/* strip trailing white space */
      while(ptr >= reply && isspace(*ptr)) {
	 *ptr-- = '\0';
      }
      
      sprintf(buff, "%s ok\n", reply);
      if(write(fd, buff, strlen(buff)) == -1) {
	 fprintf(stderr,"Sending reply %s to command %s: %s",
		 buff, cmd, strerror(errno));
	 close(fd);
	       (void)semGive(semCmdPort);
	 return;
      }
   }

   if(n == ERROR && errno != 0) {
      fprintf(stderr,"Reading on port %d: %s", port, strerror(errno));
   }

   (void)semGive(semCmdPort);
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
