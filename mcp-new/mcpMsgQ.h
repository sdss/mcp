#if !defined(MCP_MSG_Q_H)
#define MCP_MSG_Q_H 1

#include <msgQLib.h>

typedef struct {
   enum {
      DIO316ClearISR_type,
      moveCW_type,			/* counterweights motion */
      moveCWAbort_type			/* abort counterweights */
   } type;
   union {
      struct {
	 int timeout;
	 unsigned char dio316int_bit;
      } DIO316ClearISR;

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
 * tm_ClrInt task
 */
extern MSG_Q_ID msgDIO316ClearISR;

#endif
