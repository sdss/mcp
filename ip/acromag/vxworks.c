#include <vxWorks.h>
#include <intLib.h>
#include <iv.h>
#include "ip480.h"
#include "acromag.h"

static VOIDFUNCPTR oldisr;

int attach_ihandler( trap, vector, zero, handler, hdata )
   int trap;
   BYTE vector;
   int zero;
   VOIDFUNCPTR handler;
   struct handler_data* hdata;
{
   oldisr = intVecGet(INUM_TO_IVEC(vector));
   intVecSet(INUM_TO_IVEC(vector),
	     intHandlerCreate(handler, (int)hdata));
   return(0);
}

int detach_ihandler( trap, vector, hdata )
   int trap;
   BYTE vector;
   struct handler_data* hdata;
{
   intVecSet(INUM_TO_IVEC(vector),oldisr);
   return(0);
}
