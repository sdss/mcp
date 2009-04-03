#if !defined(AS2_H)
#define AS2_H
#include "mcpMsgQ.h"
/*
 * Support for the AS2 ICCs and hubs
 */
/*
 * OTRACE: old SDSS TRACE to murmur
 * NTRACE: new keyword logging of info that used to go to murmur
 */
#define OTRACE(LEVEL, FORMAT, VAL1, VAL2) \
    TRACE(LEVEL, FORMAT, VAL1, VAL2)

#define NTRACE_MAX 1                    /* Highest trace level to send out as keywords */

#define NTRACE(LEVEL, UID, CID, STRING) \
    TRACE(LEVEL, STRING, 0, 0); \
    if (LEVEL <= NTRACE_MAX) { \
        sendStatusMsg_S(UID, CID, INFORMATION_CODE, 1, "trace", STRING); \
    }

#define NTRACE_1(LEVEL, UID, CID, FORMAT, VAL1) \
    TRACE(LEVEL, FORMAT, VAL1, 0); \
    if (LEVEL <= NTRACE_MAX) { \
        char _ntrace_buff[KEY_VALUE_LEN];    \
        sprintf(_ntrace_buff, FORMAT, VAL1); \
        sendStatusMsg_S(UID, CID, INFORMATION_CODE, 1, "trace", _ntrace_buff); \
    }

#define NTRACE_2(LEVEL, UID, CID, FORMAT, VAL1, VAL2) \
    TRACE(LEVEL, FORMAT, VAL1, VAL2); \
    if (LEVEL <= NTRACE_MAX) { \
        char _ntrace_buff[KEY_VALUE_LEN];          \
        sprintf(_ntrace_buff, FORMAT, VAL1, VAL2); \
        sendStatusMsg_S(UID, CID, INFORMATION_CODE, 1, "trace", _ntrace_buff); \
    }

/*
 * Support for status keywords
 */
#define INVALID_INT 0xdeadbeef          /* An invalid value; in particular can't be cached and not resent */

void resetKeywordDictionary();
void declareKeyword(char const* key, MSG_TYPE type, int alwaysSend, char const*);

void sendStatusMsg(int uid, unsigned long cid, MSG_CODE code, int broadcast);
void sendStatusMsg_A(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, const char *msg);
void sendStatusMsg_B(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, int val);
void sendStatusMsg_F(int uid, unsigned long cid, MSG_CODE code, int broadcast,
                     const char *key, float val);
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
void broadcast_cw_status(int uid, unsigned long cid);

#endif
