/*
 * This code started out as part of the SDSS DA system, 
 * AUTHOR:	Ron Rechenmacher, Creation date: 25-Jun-1996
 *****************************************************************************/

#include	"vxWorks.h"
#include	"taskLib.h"	/* WIND_TCB */
#include	"usrLib.h"	/* ti (task info) */
#include	"dbgLib.h"	/* l (as in list or disassemble) */
#include	"ioLib.h"	/* close, open, ioTaskStdSet */
#include	"vxLib.h"	/* vxMemProbe */
#include	"arch/mc68k/esfMc68k.h"	/* ESF0 struct */

#define		 dscTraceIMP
#include	"dscTrace.h"

/* Prototype that should have been included in trace package. */
void
traceShow( int delta, int lines );

/******************************************************************************
 *	 task switch hook that traces
 * Added by ron on 29-Mar-1995; hacked by RHL
 *****************************************************************************/
/*
 * If WATCH_MEMORY is defined you can watch a chosen address for mofification
 */
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
   TRACEP(31, "switching to 0x%x%x",
	  ((int *)taskName((int)pNewTcb))[0],
	  ((int *)taskName((int)pNewTcb))[1]);

   if(*watch != watch_val || watch == NULL || *taskName((int)pOldTcb) == '/') {
      TRACE(31, "Suspending task %p", pOldTcb, 0);
      taskSuspend((int)pOldTcb);
      TRACE(31, "disabling trace", 0, 0);
      traceMode(traceModeGet() & ~0x1);
   }
#else
   TRACEP(31, "switching to 0x%x%x",
	  ((int *)taskName((int)pNewTcb))[0],
	  ((int *)taskName((int)pNewTcb))[1]);
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


/******************************************************************************
 * @+Public+@
 * ROUTINE: trc_stdout{Change,Restore}:  Added by ron on 14-Mar-2000
 *
 * DESCRIPTION:		routines to change stdout destination to a file (on
 *                      unix host) 
 *
 ******************************************************************************/

/* vxWorks does not have dup, which would have been useful here */

static int
trc_stdoutChange( const char	*file )
{							/* @-Public-@ */
	int	fd;
	int	mode;

    mode = O_WRONLY | O_CREAT | O_TRUNC;
    fd = open( file, mode, 0666 );

    if (fd == -1) perror( "open" );
    else
    {             ioTaskStdSet( 0, 1, fd );
		  ioTaskStdSet( 0, 2, fd );
    }

    return (fd);
}   /* trc_stdoutChange */

static void
trc_stdoutRestore( int	fd )
{
    ioTaskStdSet( 0, 1, 1 );	/* 0=self, 1=STD_IN, 1=stdin */
    ioTaskStdSet( 0, 2, 2 );
    close( fd );
}   /* trc_stdoutRestore */


/******************************************************************************
 * @+Public+@
 * ROUTINE: trc_dumpInfo:  Added by ron on 14-Mar-2000
 *
 * DESCRIPTION:		attemps to dump information to file on unix system
 *
 ******************************************************************************/

WIND_TCB	trc_Tcb;	/* Note: in vxworks, tid is a pointer to TCB */


/*static*/ void
trc_dumpInfo( int tid )
{							/* @-Public-@ */
	int		fd;
	REG_SET		Reg;	/* $VX_DSC_DIR/h/arch/mc68k/regsMc68k.h */
#       if 0
	FPREG_SET	FpRegSet;/* $VX_DSC_DIR/h/arch/mc68k/fppMc68kLib.h */
#       endif
	int		ii;

    /* copy tcb as activity in this function can change its content */
    trc_Tcb = *(taskTcb(tid)); /* *(WIND_TCB *)tid; */

    fd = trc_stdoutChange("/p/mcpbase/trace/mcp.dump");
	
    printf( "pStackBase =0x%p\n", trc_Tcb.pStackBase );
    printf( "pStackLimit=0x%p\n", trc_Tcb.pStackLimit );
    printf( "pStackEnd  =0x%p\n", trc_Tcb.pStackEnd );
    
    printf( "ti 0x%x\n", tid );
    taskShow( (int)&trc_Tcb, 1 );
    taskRegsGet( tid, &Reg );
    printf(  "\nd0     = %8x   d1     = %8x   d2     = %8x   d3     = %8x\n"
	     "d4     = %8x   d5     = %8x   d6     = %8x   d7     = %8x\n"
	     "a0     = %8x   a1     = %8x   a2     = %8x   a3     = %8x\n"
	     "a4     = %8x   a5     = %8x   a6/fp  = %8x   a7/sp  = %8x\n"
	     "sr     = %8x   pc     = %8x\n"
	   , Reg.dataReg[0], Reg.dataReg[1], Reg.dataReg[2], Reg.dataReg[3]
	   , Reg.dataReg[4], Reg.dataReg[5], Reg.dataReg[6], Reg.dataReg[7]
	   , Reg.addrReg[0], Reg.addrReg[1], Reg.addrReg[2], Reg.addrReg[3]
	   , Reg.addrReg[4], Reg.addrReg[5], Reg.addrReg[6], Reg.addrReg[7]
	   , Reg.sr, (unsigned)(Reg.pc) );
#   if 0
    /*  This seems to corrupt the taskRegs ... I do not know how it could.
        I most likely do not need floating point regs anyway. */
    fppTaskRegsGet( tid, &FpRegSet );
    printf(  "fpcr   = %8x   fpsr   = %8x   fpiar  = %8x\n"
	   , FpRegSet.fpcr, FpRegSet.fpsr, FpRegSet.fpiar );
    for (ii=0; ii<8; ii++)
    {   printf( "f%d     = %8f   ", ii, *(double *)&(FpRegSet.fpx[ii]) );
	if ((ii&3) == 3) printf( "\n" );
    }
#   endif

    /*  Try to be carefull as the archiver, for example, is not a critical
        task and we do not want a bus error during disassembly.
	Test by spawning "sp(0x07000000)" */
    if (vxMemProbe((char *)(Reg.pc),VX_READ,4,(char *)&ii) == OK)
    {   printf( "\n\nl 0x%x,0x70\n", (unsigned)(Reg.pc)-0x30 );
	l( Reg.pc-0x30, 0x70 );
    }
    else
    {   printf( "\n\ndisassembly skipped because of access error\n" );
    }

    if (vxMemProbe((char *)(Reg.addrReg[7]),VX_READ,4,(char *)&ii) == OK)
    {   int	num;
#       if 0
	num = (int)(trc_Tcb.pStackBase) - Reg.addrReg[7];
	num /= 2;
	if (num > 0x100) num = 0x100;
#       else
        num = 0x100;
#       endif
	printf( "\n\nd 0x%x,0x%x,2\n", Reg.addrReg[7], num );
	d( (void *)(Reg.addrReg[7]), num, 2 );
    }
    else
    {   printf( "\n\nStack dump skipped because of access error\n" );
    }


    printf( "\n\ntraceShow\n" );
    traceShow( 0, 0 );

    trc_stdoutRestore( fd );
    return;	
}   /* trc_dumpInfo */


/******************************************************************************
 * @+Public+@
 * ROUTINE: trc_excHook:  Added by ron on 13-Mar-2000
 *
 * DESCRIPTION:		stop tracing upon exception
 *
 * RETURN VALUES:	None.
 *
 * SIDE EFFECTS:	Exceptions (bus errors) from the shell are somehow
 *                      handled differently (this function does not get
 *                      called).  This function gets called after the task
 *                      gets suspended.
 *
 ******************************************************************************/


void
trc_excHook(  int	tid	/* ID   of    offending    task */
	    , int	vecNum	/* exception   vector    number */
	    , ESF0	*pEsf )	/* pointer to exception  stack  frame */
{							/* @-Public-@ */
	int	traceMode_sav;

    TRACE( 30, "trc_excHook freezing trace buffer", 0,0 );
    traceMode_sav = traceModeGet();
    traceMode( traceMode_sav & ~0x1);

    if (traceMode_sav & 1)
    {   /* 1 to 0 transition ... dump attempt dump of info */

	/*  It appears that during the execution of this function, if a
	    task switch occurs, the information in the TASK_DESC structure
	    gets "updated" such that upon later (from the command line)
	    "ti" print outs, although the execption frame is the same, the
	    registers (namely the pc) in the register information does not
	    match the registers (pc) in the execption frame. */
	trc_dumpInfo( tid );
    }
    return;
}   /* trc_excHook */
