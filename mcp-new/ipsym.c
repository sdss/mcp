#include <stdio.h>
#include <timers.h>
#include <time.h>

#include "vxWorks.h"
#include "sysLib.h"
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "taskLib.h"
#include "sigLib.h"
#include "ioLib.h"
#include "rebootLib.h"
#include "timerint.h"
#include "mcpUtils.h"
#include "dscTrace.h"

int freqtick = 0;			/* counter for serverDCStart */

SEM_ID semDC=NULL;

/*****************************************************************************/
/*
 * Passed to TimerStart in mcp.login
 */
void
serverDCStart()
{
   int frequency = 1;			/* how many ticks to wait before
					   fireing semDC trigger */
   TRACE0(16, "serverDCStart", 0, 0, 0, 0);

   freqtick++;
   if(frequency == 0 || freqtick%frequency == 0) {
      if(semDC != NULL) {
	 semGive(semDC);		/* trigger serverData to do some work*/
      }
   }
}
   
#if 0
void
server_shutdown(int type)
{
    printf("SERVER Timer4Stop shutdown: \r\n");
#if 0
    Timer4Stop ();
#endif
    sysIntDisable (5);
    taskDelay(30);
}
#endif

/*****************************************************************************/
/*
 * Called from mcp.login
 */
void
serverData(int hz,			/* frequency at which to call */
	   void (*data_routine()))	/*      this routine */
{
#if 0
   rebootHookAdd((FUNCPTR)server_shutdown);
#endif
   semDC = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
#if 0
   Timer4Start (hz,5,serverDCStart);
#endif
   
   if(data_routine == NULL) {
      return;				/* nothing to do */
   }
   
   for(;;) {
      if(semTake(semDC, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "couldn't take semDC semaphore: %d %s",
	       errno, strerror(errno), 0, 0);
	 taskSuspend(0);
      }
      
      data_routine();
   }
}
