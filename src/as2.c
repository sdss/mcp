/*****************************************************************************/
/*
 * Code to talk to the SDSS-III APO environment
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <taskLib.h>
#include <ioLib.h>
#include <sysSymTbl.h>
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "as2.h"
#include "cmd.h"
#include "mcpUtils.h"

/*****************************************************************************/
/*
 * Handle reporting status to the Brave New World of the SDSS-III hub
 */
MSG_Q_ID msgStatus = NULL;		/* Queues of status keywords */

int keywordQueueDepth = 0;	        /* maximum number of messages waiting on the msgStatus queue */

#define BUFF_SIZE 81			/* size of response buffers */

typedef struct {
   int uid;				/* user ID */
   int fd;				/* uid's file descriptor */
   int tid;				/* uid's TaskID */
} USER_FD;

/* 
 * a queue which aborted cmd messages should get sent to so that they can be failed out.
 */
MSG_Q_ID msgReaper = NULL;

/********************************************************************************/

SYMTAB_ID keySymTbl = NULL;		/* our symbol table for keys that we've already sent */

typedef struct {
   char key[27];			/* The desired keyword */

   int alwaysSend;			/* should I always send this key? */

   MSG_TYPE type;			/* type of value */

   union {
      float fval;
      int ival;
      char sval[KEY_VALUE_LEN];
   } u;
} KEYWORD_DICT;

/*
 * Invalidate a KEYWORD_DICT's value
 */
static void
invalidateValue(KEYWORD_DICT *entry)
{
   switch (entry->type) {
    case file_descriptor:
      abort();			/* you can't get here */       
    case none:
    case novalue:
      break;
    case floating:
      entry->u.fval = INVALID_INT;
      break;
    case hex:
    case integer:
    case boolean:
      entry->u.ival = INVALID_INT;
      break;
    case array:
    case string:
      strcpy(entry->u.sval, "\007");
      break;
   }
}

/*
 * Declare a keyword.  Its value will be used to see whether we need to resend keys
 * to clients
 */
void declareKeyword(char const* key,    /* the key name; if NULL just create symbol table */
		    MSG_TYPE type,      /* the type (as used by MCP_STATUS_MSG) */
		    int alwaysSend,	/* should I always send this key, even if it hasn't changed */
		    char const* help	/* help string (NOTUSED) */
   )
{
   int status;
   KEYWORD_DICT *entry;

   if(keySymTbl == NULL) {
      keySymTbl = symTblCreate(256, FALSE, memSysPartId);
      assert(keySymTbl != NULL);

      if (key == NULL) {
	 return;
      }
   }
   
   entry = malloc(sizeof(KEYWORD_DICT));
   entry->type = type;

   if (type == array || type == boolean || type == floating || type == hex || type == integer || type == string) {
      entry->alwaysSend = alwaysSend;
   } else {
      entry->alwaysSend = 1;
   }

   strncpy(entry->key, key, sizeof(entry->key) - 1);
   entry->key[sizeof(entry->key) - 1] = '\0';

   invalidateValue(entry);

   status = symAdd(keySymTbl, entry->key, (char *)entry, 0, 0);
   if(status != OK) {
      printf("Failed to add %s to symbol table\n", entry->key);
   }
}

static BOOL
resetKey(char *name,		/* name of command */
	 int val,		/* the value saved as name */
	 SYM_TYPE type,		/* NOTUSED */
	 int userArg,		/* NOTUSED */
	 UINT16 group)		/* NOTUSED */
{
   KEYWORD_DICT *entry = (KEYWORD_DICT *)val;

   invalidateValue(entry);

   return(TRUE);
}

void
clearKeywordCache()
{
   if(keySymTbl == NULL) {
      declareKeyword(NULL, 0, 0, NULL);	/* allocate keySymTbl */
   }
   
   symEach(keySymTbl, (FUNCPTR)resetKey, (int)0);
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
/*
 * The task to send status to the hub
 */
static USER_FD *fds;			/* active file descriptors */
static int nfd = 0;			/* number of active file descriptors */
static int size_fds = 0;		/* allocated size of fds */

void showConnections()
{
   int i;

   printf("fd uid TID\n");
   for (i = 0; i < nfd; ++i) {
      printf("%2d %3d 0x%x\n", fds[i].fd, fds[i].uid, fds[i].tid);
   }
}

void
tStatus(void)
{
   int uid = 0, cid = 0;
   MCP_STATUS_MSG msg;			/* message to pass around */
   int broadcast;			/* should this message be broadcast to all listeners in fds? */
   char buff[BUFF_SIZE + 1];		/* response buffer */
   int i;
   int nwrite;				/* number of chars to write to fds[].fd */
   int ofd;				/* an output file descriptor */

   if (keySymTbl == NULL) {
      declareKeyword(NULL, 0, 0, NULL);	/* allocate keySymTbl */
   }

   size_fds = 10;
   fds = malloc(size_fds*sizeof(USER_FD));
   assert (fds != NULL);
   
   for(;;) {
      int ret = msgQReceive(msgStatus, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      if (ret == ERROR) {
	 NTRACE_1(0, uid, cid, "Failed to read msgStatus: %s", strerror(errno));
	 printf("Failed to read msgStatus: %d,  %s\n", ret, strerror(errno));
	 taskDelay(10);
	 continue;
      }

      {
	 int nmsgs = msgQNumMsgs(msgStatus);

	 if (nmsgs > keywordQueueDepth) {
	    keywordQueueDepth = nmsgs;

	    sprintf(buff, "%d %d %c %s=%d\n", uid, cid, DEBUG_CODE, "keywordQueueDepth", keywordQueueDepth);
	    nwrite = strlen(buff);
	    assert (nwrite < BUFF_SIZE);

	    for (i = 0; i != nfd; ++i) {
	       write(fds[i].fd, buff, nwrite);
	    }
	 }
      }

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
		   for (; i < nfd - 1; ++i) {
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
	     fds[nfd].tid = msg.cid;
	     ++nfd;
	  }
	  continue;
      }
      /*
       * OK, some reply with or without keywords
       */
      if (msg.uid == 0 && msg.cid == 0 && msg.code == FINISHED_CODE) {
	 msg.code = INFORMATION_CODE;
      }

      /*
       * See if we already know that keyword, and if so if it's changed
       *
       * Don't bother with keys "text" and "trace"
       */
      if (msg.code == INFORMATION_CODE) {
	 KEYWORD_DICT *entry = NULL;
	 if(symFindByName(keySymTbl, msg.key, (char **)&entry, NULL) == OK) {

	    if (!entry->alwaysSend) {
	       int is_same = 0;		/* keyword's value is the same as one that's already interned */

	       switch (entry->type) {
		case none:
		case novalue:
		case file_descriptor:
		  abort();			/* you can't get here */       
		case floating:
		  if (entry->u.fval == msg.u.fval) {
		     if (entry->u.fval != INVALID_INT) { /* we initialise the symbol table to INVALID_INT */
			is_same = 1;
		     }
		  } else {
		     entry->u.fval = msg.u.fval;
		  }
		  break;
		case boolean:
		case hex:
		case integer:
		  if (entry->u.ival == msg.u.ival) {
		     if (entry->u.ival != INVALID_INT) { /* we initialise the symbol table to INVALID_INT */
			is_same = 1;
		     }
		  } else {
		     entry->u.ival = msg.u.ival;
		  }
		  break;
		case array:
		case string:
		  if (strcmp(msg.u.sval, entry->u.sval) == 0) {
		     is_same = 1;
		  } else {
		     strncpy(entry->u.sval, msg.u.sval, KEY_VALUE_LEN - 1);
		     entry->u.sval[KEY_VALUE_LEN - 1] = '\0';
		  }
		  break;
	       }

	       if (is_same) {
		  continue;
	       }
	    }
	 }
      }

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
	case floating:
	  sprintf(buff + nwrite, "%s=%12f\n", msg.key, msg.u.fval);
	  break;
	case none:
	  sprintf(buff + nwrite, "\n");
	  break;
	case novalue:
	  sprintf(buff + nwrite, "%s\n", msg.key);
	  break;
	case hex:
	  sprintf(buff + nwrite, "%s=0x%08x\n", msg.key, msg.u.ival);
	  break;
	case integer:
	  sprintf(buff + nwrite, "%s=%d\n", msg.key, msg.u.ival);
	  break;
	case array:
	case string:
	  if (strlen(msg.u.sval) == 0) {
	     sprintf(buff + nwrite, "%s\n", msg.key);
	  } else if (strlen(msg.key) + 3 /* ="" */ + strlen(msg.u.sval) < BUFF_SIZE - nwrite) {
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
      if (nwrite > BUFF_SIZE) {
	 printf("nwrite = %d, BUFF_SIZE = %d\n", nwrite, BUFF_SIZE);
      }
      assert (nwrite <= BUFF_SIZE);

      for (i = 0; i != nfd; ++i) {
	 if (broadcast || fds[i].uid == msg.uid) {
	    errno = 0;
	    if (write(fds[i].fd, buff, nwrite) != nwrite) {
	       int j;

	       sprintf(buff, "writing %d bytes to %d: strerror = %s\n", nwrite, fds[i].uid, strerror(errno));
	       nwrite = strlen(buff);
	       assert (nwrite < BUFF_SIZE);

	       for (j = 0; j != nfd; ++j) {
		  if (j != i) {
		     write(fds[j].fd, buff, nwrite);
		  }
	       }
	    }
	 }
      }
   }
}

/*********************************************************************************/
/*
 * Print MCP_STATUS_MSG for debugging
 */
void printStatusMsg(MCP_STATUS_MSG const* msg) {
   switch (msg->type) {
    case none:
    case novalue:
    case file_descriptor:
      printf("%s\n", msg->key);
      break;		  
    case floating:
      printf("%s %g\n", msg->key, msg->u.fval);
      break;
    case hex:
      printf("%s 0x%x\n", msg->key, msg->u.ival);
      break;
    case integer:
    case boolean:
      printf("%s %d\n", msg->key, msg->u.ival);
      break;
    case array:
    case string:
      printf("%s %s\n", msg->key, msg->u.sval);
      break;
   }
}

/*
 * Actually send the message
 */
static void
doSendStatusMsg(MCP_STATUS_MSG const* msg, /* message to send */
		int wait,		/* how many ticks to wait */
		int priority		/* priortity of message */
   )
{
   int uid = 0, cid = 0;
   int ret = 0;
   
   if (msgStatus) {
#if 0
      if (msgQNumMsgs(msgStatus) > 100) {
	 printStatusMsg(msg);
      }
#endif

      if (msg->code == FINISHED_CODE && ublock->protocol == OLD_TCC_PROTOCOL) {
	 return;
      }

      ret = msgQSend(msgStatus, (char *)msg, sizeof(*msg), WAIT_FOREVER, MSG_PRI_NORMAL);
      if (ret != OK) {
	 printf("Failed to write message to msgStatus: %s", strerror(errno));

	 if (priority == MSG_PRI_NORMAL) {
	    (void)sendStatusMsgUrgent_S(uid, cid, INFORMATION_CODE, NO_WAIT, "statusSendFailed", strerror(errno));
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = none;

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = array;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   strncpy(msg.u.sval, val, sizeof(msg.u.sval) - 1); msg.u.sval[sizeof(msg.u.sval) - 1] = '\0';

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = boolean;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.ival = val;

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
}

/*
 * Send a (key, float) pair
 */
void
sendStatusMsg_F(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		float val)		/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = floating;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.fval = val;

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = file_descriptor;
   msg.key[0] = '\0';
   msg.u.ival = fd;

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
}

/*
 * Send a (key, int) pair
 *
 * N.b. sendStatusMsg_X also handles a (key, int) pair but formats the int in hex
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = integer;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.ival = val;

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = novalue;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
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
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = string;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   strncpy(msg.u.sval, val, sizeof(msg.u.sval) - 1); msg.u.sval[sizeof(msg.u.sval) - 1] = '\0';

   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
}

/*
 * Send a (key, int) pair, formatting the int in hex
 *
 * N.b. sendStatusMsg_I also handles a (key, int) pair but formats the int as a, well, int
 */
void
sendStatusMsg_X(int uid,		/* user ID */
		unsigned long cid,	/* command ID */
		MSG_CODE code,		/* code; e.g. : */
		int broadcast,		/* should this message be broadcast to all clients? */
		const char *key,	/* keyword */
		int val)		/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = hex;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.ival = val;
   
   doSendStatusMsg(&msg, WAIT_FOREVER, MSG_PRI_NORMAL);
}

/*
 * Send a (key, string) pair at high priority
 */
void
sendStatusMsgUrgent_S(int uid,		/* user ID */
		      unsigned long cid, /* command ID */
		      MSG_CODE code,	/* code; e.g. : */
		      int broadcast,    /* should this message be broadcast to all clients? */
		      const char *key,	/* keyword */
		      const char *val)	/* value */
{
   MCP_STATUS_MSG msg;			/* message to send */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = string;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   strncpy(msg.u.sval, val, sizeof(msg.u.sval) - 1); msg.u.sval[sizeof(msg.u.sval) - 1] = '\0';

   doSendStatusMsg(&msg, 10, MSG_PRI_URGENT);
}

/************************************************************************************************************/
/*
 * Respond to a ping
 */
char *
ping_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
    sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ping");

    return "";
}

/************************************************************************************************************/
/*
 * Report the lavaLamp's status
 */
int lava_lamp_on = 0;			/* is lava lamp on? */

char *
lava_on_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   lava_lamp_on = 1;

   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "lavaLamp", lava_lamp_on);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "lava_on");

   return "";
}

char *
lava_off_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   lava_lamp_on = 0;

   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "lavaLamp", lava_lamp_on);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "lava_off");

   return "";
}

/*****************************************************************************/
/*
 * Commands to allow an iop server to pass information through the hub to the alertsActor
 */
char *
im_camCheck_cmd(int uid, unsigned long cid, char *cmd) /* NOTUSED */
{
   char problems[KEY_VALUE_LEN];
   int len = sprintf(problems, "%ld,", time(NULL) + 34); /* approximate conversion from UTC to TAI */
   
   strncpy(problems + len, cmd, KEY_VALUE_LEN - len - 1);
   problems[KEY_VALUE_LEN - 1] = '\0';

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "imCamCheck", problems);

   return "";
}

/*********************************************************************************/
/*
 * The heartbeat task, used to demonstrate that we live and breath
 */
void
tHeartbeat(void)
{
   int uid = 0, cid = 0;

   for(;;) {
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "aliveAt", time(NULL));

      taskDelay(60*60);			/* 1 minute */
   }
}

/*********************************************************************************/
/*
 * The reaper task, used to fail any ns_e_aborted cmds.
 */
void
tReaper(void)
{
   MCP_MSG msg;
   
   for(;;) {
     int ret;
     int uid; 
     unsigned long cid;
     char valBuf[100];

     ret = msgQReceive(msgReaper, (char *)&msg, sizeof(msg), WAIT_FOREVER);
     assert(ret != ERROR);

     NTRACE(1, 0, 0, "read msg on msgReaper");
     get_uid_cid_from_tmr_msg(&msg, &uid, &cid);

     if (uid == 0 && cid == 0) {
       sprintf(valBuf, "NOT reaping msg_type=%d with ids=(%d,%ld)", msg.type, uid, cid);
       sendStatusMsg_S(uid, cid, WARNING_CODE, 1, "text", valBuf);
     } else {
       sprintf(valBuf, "reaped msg_type=%d with ids=(%d,%ld)", msg.type, uid, cid);
       sendStatusMsg_S(uid, cid, FATAL_CODE, 1, "text", valBuf);
     }
   }
}

/*
 * Initialise the task that handles status messages for the hub
 */
void
as2Init(void)
{
   if(msgStatus == NULL) {
      int ret;				/* return code */
      /*
       * Create message queue of status information.
       */
      msgStatus = msgQCreate(200, sizeof(MCP_STATUS_MSG), MSG_Q_FIFO);
      assert(msgStatus != NULL);
      /*
       * The task that sends the status
       */
      ret = taskSpawn("tStatus", (FALSE ? 91 : 68),0,10000,(FUNCPTR)tStatus,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
      /*
       * The heartbeat task
       */
      ret = taskSpawn("tHeartbeat",200,0,10000,(FUNCPTR)tHeartbeat,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
      /*
       * The cmd reaper task and queue
       */
      msgReaper = msgQCreate(200, sizeof(MCP_STATUS_MSG), MSG_Q_FIFO);
      assert(msgReaper != NULL);
      ret = taskSpawn("tReaper",200,0,10000,(FUNCPTR)tReaper,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
/*
 * define spectro commands to the command interpreter
 */
   define_cmd("PING",          ping_cmd, 	  0, 0, 0, 0, "Ping the MCP");
   define_cmd("LAVA_LAMP_ON",  lava_on_cmd,       0, 0, 0, 0, "Set the lava lamp status ON");
   define_cmd("LAVA_LAMP_OFF", lava_off_cmd,      0, 0, 0, 0, "Set the lava lamp status OFF");
   define_cmd("LAVA_ON",       lava_on_cmd,       0, 0, 0, 0, "Set the lava lamp status ON");
   define_cmd("LAVA_OFF",      lava_off_cmd,      0, 0, 0, 0, "Set the lava lamp status OFF");
   define_cmd("IM_CAMCHECK",   im_camCheck_cmd,   1, 0, 0, 0, "Pass on any problems detected in the imager");
}
