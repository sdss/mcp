#ifndef	DSCTRACE_H
#define	DSCTRACE_H

/****************************** Copyright Notice ******************************
 *                                                                            *
 * Copyright (c) 1992 Universities Research Association, Inc.                 *
 *               All Rights Reserved.                                         *
 *                                                                            *
 ******************************************************************************/

/* @+Public+@
 * PROJECT:	Drift Scan Camera
 *
 * ABSTRACT:	A C header file to include the vx_tools trace utility and
 *              customize it for use in the dsc/sdss project.
 *
 * ENVIRONMENT:	ANSI C header.
 *		dscTrace.h
 *
 * AUTHOR:	Ron R., Creation date: 28-Apr-1995
 * @-Public-@
 ******************************************************************************/

#ifdef	dscIMP
# undef	dscIMP
#endif
#ifdef	dscTraceIMP
# define	dscIMP
#else
# define	dscIMP	extern
#endif

/* to effectively turn the throttling off, set _LimPerTimPeriod ot 0x7fffffff
   and _TimPeriod to a relative small number of sec, i.e. 2000000.
   trace_usrUseLogMsg allows testing of the throttling w/o murmur */
dscIMP	int	trace_usrUseLogMsg
#ifdef	dscTraceIMP
				= 0		/* boolean */
#endif
				;
/*  NOTE: THESE LIMITS ARE FOR EACH TRACE; AS EACH TRACE HAS ITS OWN STATIC
    __cnt AND trc_usr_t1 VARIABLES. */
dscIMP	int	trace_limPerTimPeriod
#ifdef	dscTraceIMP
				= 6		/* number of msg allowed -
						   guarantee a set of msgs
						   for 6 CCDs. The 6th
						   messages will indicage
						   that throttling *could*
						   occur on the next
						   message. */
#endif
				;
dscIMP	int	trace_timPeriod
#ifdef	dscTraceIMP
				= 4500000	/* usec */
#endif
				;


/* The following macro defines stack variables -- a test using the following
   code segment indicates that these variables are created when the block of
   code is executed.  (This is not the case with the SGI ansi compiler.) So
   there shouldn't be any concern about the number of TRACE macros adding to
   the stack usage.  NOTE: There is still the concern about the sprintf output
   buffer space. (It would be nice if there was a function which would take an
   output buffer size.)  I will still use the trick of haveing sprintf
   possible start outputing over the fmt spec. NOTE: the strncpy is used, as
   stated below to stop a warning. 
void	test2( void )
{
	char	a[150] = "hello";

    printf( "a=0x%08x\n", (int)a );
    if (a[0])
    {        char b[100];
         printf( "b=0x%08x\n", (int)b );
    }
    if (a[0])
    {        char b[100];
         printf( "b=0x%08x\n", (int)b );
    }
}
output:
a=0x01faa366
b=0x01faa302
b=0x01faa302
*/
#define	TRACE_USR_FUNC(tlvl,msg_str,p1,p2) \
		{   char a[200]; unsigned mur_code;\
                    static unsigned int __cnt=0, trc_usr_t1;\
		    a[30]='\0'; /* incase msg_str has < 30 chars */ \
		    if      (tlvl>=3) mur_code=dsc_trcInfo;\
		    else if (tlvl==2) mur_code=dsc_trcWarn;\
		    else if (tlvl==1) mur_code=dsc_trcSucc;\
		    else              mur_code=dsc_trcErr;\
		    __cnt++;\
		    if (__cnt <= (trace_limPerTimPeriod-1))\
		    {   if  ((traceTS-trc_usr_t1) > trace_timPeriod)\
		        {   __cnt = 0;  trc_usr_t1 = traceTS;\
			}\
                        if (__cnt == (trace_limPerTimPeriod-1))\
                        {   if (!trace_usrUseLogMsg)\
			    {   strncpy( &a[100], "(throttle) " msg_str, 99 );\
                                a[199]='\0';\
			        sprintf( a, (const char *)&a[100], p1, p2 );\
			        mur_route_send(  mur_code, 2, MUR_STRING, a\
                                               , MUR_STRING, &a[30] );\
			    }\
			    else logMsg( "(throttle) utimer: %10u %s"msg_str"\n"\
                                        ,(int)*trace_.traceClk\
                                        ,(int)traceLvlStr[tlvl]\
                                        ,(int)p1, (int)p2, 0, 0 );\
                        }\
                        else\
                        {   if (!trace_usrUseLogMsg)\
			    {   strncpy( &a[100], msg_str, 99 ); a[199]='\0';\
			        sprintf( a, (const char *)&a[100], p1, p2 );\
			        mur_route_send( mur_code,2,MUR_STRING,a\
                                               ,MUR_STRING,&a[30]);\
			    }\
			    else logMsg( "utimer: %10u %s"msg_str"\n" \
			    		,(int)*trace_.traceClk\
                                        ,(int)traceLvlStr[tlvl]\
                                        ,(int)p1, (int)p2, 0, 0 );\
                         }\
		    }\
		    else if ((traceTS-trc_usr_t1) > trace_timPeriod)\
		    {   if (!trace_usrUseLogMsg)\
			{   strncpy( &a[100], "throttled %d -- "msg_str, 99 );\
                            a[199]='\0';\
			    sprintf( a, (const char *)&a[100]\
				    , __cnt-trace_limPerTimPeriod\
				    , p1, p2 );\
			    mur_route_send(  mur_code, 2, MUR_STRING, a\
                                           , MUR_STRING, &a[30] );\
			}\
			else logMsg( "throttled %d -- utimer: %10u %s"msg_str"\n"\
				    ,(int)(__cnt-trace_limPerTimPeriod)\
				    ,(int)*trace_.traceClk\
                                    ,(int)traceLvlStr[tlvl]\
                                    ,(int)p1, (int)p2, 0 );\
			__cnt = 0;\
			trc_usr_t1 = traceTS;\
		    }\
		}
		/* I can't use mur_route_send_text as that would not give me
		   the necessary color control.
		   b/c of this definition, there is now a dependance on
		   dsc_msg_c.h ( - ONE level of nested include within the
		   project is OK)
		   Note also: MUR_STRING is limited to 30 characters.  We
		   could use SOMETHING like:
		   mur_route_send(mur_code,2,MUR_STRING,b,MUR_APPEND,&b[30])
		   although the appended text is printed on the next line
		   or two MUR_STRINGs:
		   mur_route_send(mur_code,2,MUR_STRING,b,MUR_STRING,&b[30]) */

#include "dsc_msg_c.h"		/* dsc_trcErr,etc */
#define MUR_VXWORKS		/* to get prototypes */
#include "mur_user.h"		/* MUR_STRING enum */
#include "trace.h"		/* includes vxWorks.h and also logLib -
				   dscMake uses filterWarn to filter logMsg
				   arg 1 passing problems */
#include "stdio.h"		/* for sprintf */
#include "string.h"		/* for strncpy - which is used just to put
				   varible for format so the compiler can't
				   warn "too many arguments for format" - ref
				   above. */
#include "vxWorks.h"		/* base vxworks include */
#include "msgQLib.h"		/* MSG_Q_ID */

extern	int	trcDumping;
void
trc_dump(  int		enable_trace
	 , MSG_Q_ID	trcDumpMsgQ );


/******************************************************************************/
#endif /* DSCTRACE_H */

