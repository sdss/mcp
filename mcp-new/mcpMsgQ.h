#if !defined(MCP_MSG_Q_H)
#define MCP_MSG_Q_H 1

#include <msgQLib.h>

typedef struct {
   enum {
      latchCrossed_type,		/* we crossed a fiducial */
      latchReenable_type,		/* reenable fiducial latches */
      moveCW_type,			/* counterweights motion */
      moveCWAbort_type			/* abort counterweights */
   } type;
   
   union {
      struct {
	 unsigned int time;		/* when we saw the fiducial */
      } latchCrossed;

      struct {
	 int timeout;
	 unsigned char dio316int_bit;
      } latchReenable;

      struct {
	 int inst;			/* instrument to balance for */
	 int cw;			/* CW to move, or ALL_CW */
	 int cwpos;			/* desired position, or 0 to balance */
      } moveCW;

      struct {
	 int abort;			/* always true */
      } moveCWAbort;
   } u;
} MCP_MSG;

/*
 * moveCW task
 */
extern MSG_Q_ID msgMoveCW;
extern MSG_Q_ID msgMoveCWAbort;
extern SEM_ID semMoveCWBusy;
/*
 * tm_ClrInt/tLatch tasks
 */
extern MSG_Q_ID msgLatched;
extern MSG_Q_ID msgLatchReenable;

#endif
