#if !defined(MCP_MSG_Q_H)
#define MCP_MSG_Q_H 1

#include <msgQLib.h>

typedef struct {
   enum {
      DIO316ClearISR_type
   } type;
   union {
      struct {
	 int timeout;
	 unsigned char dio316int_bit;
      } DIO316ClearISR;
   } u;
} MCP_MSG;

#endif
