/****************************** Copyright Notice ******************************
 *                                                                            *
 * Copyright (c) 1992 Universities Research Association, Inc.                 *
 *               All Rights Reserved.                                         *
 *                                                                            *
 ******************************************************************************/

/* @+Public+@
 * FACILITY:	Drift Scan Camera
 *		TRACE
 *
 * ABSTRACT:	Routines / data to support messages.
 *
 * ENVIRONMENT:	ANSI C.
 *		dscTrace.c
 *
 * AUTHOR:	Ron Rechenmacher, Creation date: 25-Jun-1996
 *
 * @-Public-@
 ******************************************************************************/

#include	"vxWorks.h"
#include	"taskLib.h"	/* WIND_TCB */

#define		 dscTraceIMP
#include	"dscTrace.h"

/******************************************************************************
 * @+Public+@
 * ROUTINE: trc_tskSwHk:  Added by ron on 29-Mar-1995
 *
 * DESCRIPTION
 *	 task switch hook that traces
 *
 * RETURN VALUES:	None.
 *
 * SYSTEM CONCERNS:	Now called from startup script.
 *
 *****************************************************************************/

#define WATCH_MEMORY 0

#if WATCH_MEMORY
int *watch = (int *)0x4;		/* a location to be monitored
					   for damage */
int watch_val = 0;			/* "correct" value of *watch */
#endif

void
trc_tskSwHk(WIND_TCB	*pOldTcb,
	    WIND_TCB	*pNewTcb )
{							/* @-Public-@ */
   TRACEPROC("pSwHook");
   
#if WATCH_MEMORY
   TRACEP(31, "BEGIN: switching from cccc == 0x%08x (%p)",
	  *(int *)taskName((int)pOldTcb), taskName((int)pOldTcb));
   TRACEP(31, "       %p (name == %s)", pOldTcb, taskName((int)pOldTcb));

   TRACEP(31, "       switching to   cccc == 0x%08x (%p)",
	  *(int *)taskName((int)pNewTcb), taskName((int)pNewTcb));
   TRACEP(31, "END:   %p (watch = %d)", pNewTcb, *watch);

   if(*watch != watch_val || watch == NULL || *taskName((int)pOldTcb) == '/') {
      TRACE(31, "Suspending task %p", pOldTcb, 0);
      taskSuspend((int)pOldTcb);
      TRACE(31, "disabling trace", 0, 0);
      traceMode(traceModeGet() & ~0x1);
      traceMode(0);
   }
#else
   TRACEP(31, "switching from %s to %s",
	  taskName((int)pOldTcb), taskName((int)pNewTcb) );
#endif
}	/* trc_tskSwHk */

#if WATCH_MEMORY
/*
 * Set the address to watch
 */
void
trc_setWatch(int *addr)
{
   taskLock();

   watch = addr;
   watch_val = *addr;
   
   taskUnlock();
}
#endif

/*****************************************************************************/
