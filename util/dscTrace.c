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
#if 0
#  include	"dscTrace.h"
#else
#  include	"trace.h"
#endif

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
 ******************************************************************************/

void	trc_tskSwHk(  WIND_TCB	*pOldTcb
		   , WIND_TCB	*pNewTcb );
void	trc_tskSwHk(  WIND_TCB	*pOldTcb
		   , WIND_TCB	*pNewTcb )
{							/* @-Public-@ */
	TRACEPROC("pSwHook");

    TRACEP(  31, "switching from %s to %s"
	   , taskName((int)pOldTcb), taskName((int)pNewTcb) );
}	/* trc_tskSwHk */

/******************************************************************************/
