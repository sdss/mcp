#if !defined(AS2_H)
#define AS2_H
#include "mcpMsgQ.h"
/*
 * Support for the AS2 ICCs and hubs
 */
#define NTRACE(LEVEL, FORMAT, ARG1, ARG2) /* */

void sendStatusMsg(int uid, unsigned long cid, MSG_CODE code, int broadcast);
void sendStatusMsg_A(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, const char *msg);
void sendStatusMsg_B(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, int val);
void sendStatusMsg_FD(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                      int fd);
void sendStatusMsg_I(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, int val);
void sendStatusMsg_N(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key);
void sendStatusMsg_S(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, const char *msg);

/************************************************************************************************************/
/*
 * Routines to broadcast status information
 */
void broadcast_fiducial_status(int uid, unsigned long cid);
void broadcast_inst_status(int uid, unsigned long cid);
void broadcast_ffs_lamp_status(int uid, unsigned long cid, int petals, int lamps);

#endif
