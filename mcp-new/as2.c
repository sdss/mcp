/*****************************************************************************/
/*
 * Code to talk to the SDSS-III APO environment
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <taskLib.h>
#include <ioLib.h>
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "as2.h"
#include "cmd.h"

/*****************************************************************************/
/*
 * Handle reporting status to the Brave New World of the SDSS-III hub
 */
MSG_Q_ID msgStatus = NULL;		/* Queues of status keywords */

#define BUFF_SIZE 81			/* size of response buffers */

typedef struct {
   int uid;				/* user ID */
   int fd;				/* uid's file descriptor */
} USER_FD;				/* std::pair<int, int> */

void
tStatus(void)
{
   MCP_STATUS_MSG msg;			/* message to pass around */
   int broadcast;			/* should this message be broadcast to all listeners in fds? */
   char buff[BUFF_SIZE];		/* response buffer */
   USER_FD *fds;			/* active file descriptors */
   int i;
   int nwrite;				/* number of chars to write to fds[].fd */
   int nfd = 0;				/* number of active file descriptors */
   int ofd;				/* an output file descriptor */
   int size_fds = 0;			/* allocated size of fds */

   size_fds = 10;
   fds = malloc(size_fds*sizeof(USER_FD));
   assert (fds != NULL);
   
   for(;;) {
      int ret = msgQReceive(msgStatus, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      if (msg.code > 0) {
	 broadcast = 1;
      } else {
	 broadcast = 0;
	 msg.code = -msg.code;
      }

      if (msg.type == file_descriptor) {
	  ofd = msg.u.ival;
#if 0
	  printf("\nuid= %d fd=%d code: %c %d\n", msg.uid, ofd, msg.code, msg.code);
	  for (i = 0; i < nfd; ++i) {
	     printf("uid = %d fd = %d\n", fds[i].uid, fds[i].fd);
	  }
#endif
	  if (msg.code == FINISHED_CODE) {
	     for (i = 0; i < nfd; ++i) {
		if (fds[i].fd == ofd) {
		   for (; i < nfd; ++i) {
		      fds[i] = fds[i + 1];
		   }
		   nfd--;
		   break;
		}
	     }
	  } else {
	     if (nfd == size_fds) {	
		size_fds *= 2;
		fds = realloc(fds, size_fds*sizeof(USER_FD));
		assert(fds != NULL);
	     }
	     
	     fds[nfd].uid = msg.uid;
	     fds[nfd].fd = ofd;
	     ++nfd;
	  }
	  continue;
      }
      /*
       * OK, some reply with or without keywords
       */
      sprintf(buff, "%d %lu %c ", msg.uid, msg.cid, msg.code);
      nwrite = strlen(buff);
      assert (nwrite < BUFF_SIZE - sizeof(msg.key) - 11); /* so we always have room for the key if we want it, an =
							     and 10 bytes for a value --- good enough for bool/int */

      switch (msg.type) {
       case file_descriptor:
	 abort();			/* you can't get here */       
	case boolean:
	  sprintf(buff + nwrite, "%s=%s\n", msg.key, (msg.u.ival ? "true" : "false"));
	  break;
	case none:
	  sprintf(buff + nwrite, "\n");
	  break;
	case novalue:
	  sprintf(buff + nwrite, "%s\n", msg.key);
	  break;
	case integer:
	  sprintf(buff + nwrite, "%s=%d\n", msg.key, msg.u.ival);
	  break;
	case array:
	case string:
	  if (strlen(msg.key) + 3 /* ="" */ + strlen(msg.u.sval) < BUFF_SIZE - nwrite) {
	     if (msg.type == string) {
		sprintf(buff + nwrite, "%s=\"%s\"\n", msg.key, msg.u.sval);
	     } else {
		sprintf(buff + nwrite, "%s=%s\n", msg.key, msg.u.sval);
	     }
	  } else {
	     sprintf(buff + nwrite, "%s=", msg.key);
	     nwrite += strlen(buff + nwrite);

	     if (msg.type == string) {
		buff[nwrite++] = '"'; buff[nwrite] = '\0';
	     }

	     strncat(buff + nwrite, msg.u.sval, BUFF_SIZE - nwrite - 5);
	     nwrite += strlen(buff + nwrite);

	     if (msg.type == string) {
		buff[nwrite++] = '"';
	     }
	     buff[nwrite++] = '\n';
	     buff[nwrite] = '\0';
	  }
	  break;
      }

      nwrite = strlen(buff);
      if (nwrite >= BUFF_SIZE) {
	 printf("nwrite = %d, BUFF_SIZE = %d\n", nwrite, BUFF_SIZE);
      }
      assert (nwrite < BUFF_SIZE);

      for (i = 0; i != nfd; ++i) {
	 if (broadcast || fds[i].uid == msg.uid) {
	    if (write(fds[i].fd, buff, nwrite) != nwrite) {
	       TRACE(0, "Failed to write %d bytes to uid %d", nwrite, fds[i].uid);
	    }
	 }
      }
   }
}

/*
 * Send status without any key or value
 */
void
sendStatusMsg(int uid,			/* user ID */
	      unsigned long cid,	/* command ID */
	      MSG_CODE code,		/* code; e.g. : */
	      int broadcast		/* should this message be broadcast to all clients? */
   )
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = none;

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a (key, string) pair where the string should NOT be quoted (it's a , separated array)
 */
void
sendStatusMsg_A(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		const char *val)	/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = array;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   strncpy(msg.u.sval, val, sizeof(msg.u.sval) - 1); msg.u.sval[sizeof(msg.u.sval) - 1] = '\0';

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a (key, bool) pair
 */
void
sendStatusMsg_B(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		int val)		/* value, interpreted as a boolean (i.e. 0, not-0) */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = boolean;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.ival = val;

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a fileDescriptor
 */
void
sendStatusMsg_FD(int uid,		/* user ID */
		 unsigned long cid,	/* command ID */
		 MSG_CODE code,		/* code; e.g. : */
		 int broadcast,		/* should this message be broadcast to all clients? */
		 int fd)		/* the file descriptor in question */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = file_descriptor;
   msg.key[0] = '\0';
   msg.u.ival = fd;

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a (key, int) pair
 */
void
sendStatusMsg_I(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		int val)		/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = integer;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.ival = val;

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a key without a value
 */
void
sendStatusMsg_N(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key)	/* keyword */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = novalue;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*
 * Send a (key, string) pair
 */
void
sendStatusMsg_S(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		const char *val)	/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = string;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   strncpy(msg.u.sval, val, sizeof(msg.u.sval) - 1); msg.u.sval[sizeof(msg.u.sval) - 1] = '\0';

   ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/************************************************************************************************************/
/*
 * Respond to a ping
 */
char *
ping_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
    sendStatusMsg_B(uid, cid, FINISHED_CODE, 1, "ping", 1);

    return "";
}

/*
 * Define commands
 */
void
as2Init(void)
{
/*
 * Create the message queue to control sending status to the outside world
 */
   if(msgStatus == NULL) {
      int ret;				/* return code */
/*
 * Create message queue of status information.
 */
      msgStatus = msgQCreate(40, sizeof(MCP_STATUS_MSG), MSG_Q_FIFO);
      assert(msgStatus != NULL);
      ret = taskSpawn("tStatus",90,0,5000,(FUNCPTR)tStatus,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
/*
 * define spectro commands to the command interpreter
 */
   define_cmd("PING",           ping_cmd, 	       0, 0, 0, 0, "Ping the MCP");
}
