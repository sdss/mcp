/*****************************************************************************/
/*
 * Code to talk to the SDSS-III APO environment
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <taskLib.h>
#include <ioLib.h>
#include <sysSymTbl.h>
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

   if (type == array || type == boolean || type == floating || type == integer || type == string) {
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
resetKeywordDictionary()
{
   if(keySymTbl == NULL) {
      declareKeyword(NULL, 0, 0, NULL);	/* allocate keySymTbl */
   }
   
   symEach(keySymTbl, (FUNCPTR)resetKey, (int)0);
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

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

   if (keySymTbl == NULL) {
      declareKeyword(NULL, 0, 0, NULL);	/* allocate keySymTbl */
   }

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
		case integer:
		case boolean:
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
	       NTRACE_2(0, msg.uid, msg.cid, "Failed to write %d bytes to uid %d", nwrite, fds[i].uid);
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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
   int ret;				/* return code */
   
   msg.uid = uid;
   msg.cid = cid;
   msg.code = broadcast ? code : -code;

   msg.type = floating;
   strncpy(msg.key, key, sizeof(msg.key) - 1); msg.key[sizeof(msg.key) - 1] = '\0';
   msg.u.fval = val;

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      if (ret != OK) {
	 printf("Cannot write to msgStatus: %s", strerror(errno));
      }
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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

   if (msgStatus) {
      ret = msgQSend(msgStatus, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
   }
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
      msgStatus = msgQCreate(200, sizeof(MCP_STATUS_MSG), MSG_Q_FIFO);
      assert(msgStatus != NULL);
      ret = taskSpawn("tStatus",90,0,10000,(FUNCPTR)tStatus,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
/*
 * define spectro commands to the command interpreter
 */
   define_cmd("PING",           ping_cmd, 	       0, 0, 0, 0, "Ping the MCP");
}
