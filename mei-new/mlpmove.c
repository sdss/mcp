/*
	MLPMOVE.C - medium level J-curve moves.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include <stdio.h>
#include <math.h>

#include "idsp.h"

#define	S_FRAME(t,j)	frame_m(&frame, "-0l tj u d", axis, (double)t, (double)j, FUPD_JERK | FTRG_TIME)
#define	A_FRAME(t,a)	frame_m(&frame, "-0l ta u d", axis,	(double)(t),(double)(a), FUPD_JERK | FUPD_ACCEL | FTRG_TIME)
#define	C_FRAME(t,a,j)	frame_m(&frame, "-0l taj u d", axis, (double)(t),(double)(a),(double)(j), FUPD_JERK | FUPD_ACCEL | FTRG_TIME)

/*	uom: counts, counts/sample, counts/(sample*sample), etc		*/

int16 FNTYPE pcdsp_p_trap(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	FRAME
		frame ;
	long
		m1, m2, m3, m4 ;
	double
		t1, t2, t3 ;

    double
        n1,n2,n3a,n3,n4,n5;

	DSP
		* oldDspPtr = dspPtr ;
	int16
		retval ;

	dspPtr = dsp ;

/*	Round off the jerk term so that it's an even number of samples.	*/
	m1 = (long)(accel / jerk + 0.5);
	if (! m1)
		m1 = 1 ;

/*	Round off accel.	*/
	m2 = (long)(vel / accel + 0.5);
	if (m2 < m1/2)
    {
        m2 = m1/2;
    }

	m3 = (long)(final / vel + 0.5);
    m4 = (long)(accel*accel*accel/(12.0*vel*jerk*jerk) + 0.5);

    t1 = m2 - m1/2;
    t2 = m1;
    t3 = m3 - m2 - m1 + m4;
    if(t3 < 0.0)
    {
        t3 = 0.0;
    }


/*    t = 2.0*t1 + 2.0*t2 + t3; */

    n1 = t1*(t1+1)*t2/2.0;
    n2 = t2*t1*t2 + t2*t2*(t2+1)/2.0 - t2*(t2+1)*(t2+2)/6.0;
    n3a = t2*t1 + t2*t2 - t2*(t2+1)/2.0;
    n3 = n3a*t3;
    n4 = n3a*t2 - t2*(t2+1)*(t2+2)/6.0;
    n5 = t2*(t1-1)*(t1-2)/2.0;

    jerk = final/(n1+n2+n3+n4+n5);
    accel = jerk * t2;

/*  All frames trigger on time, update jerk only */
/*  Don't download frames with T = 0 */

	retval = DSP_OK ;
    dsp_control(axis,FCTL_HOLD,TRUE);
	if (t1 && !retval)
    {
		retval = A_FRAME(t1, accel);
        dsp_control(axis,FCTL_HOLD,FALSE);
    }
	if (t2 && !retval)
    {
        if(t1)
        {
		    retval = S_FRAME(t2, -jerk);
        }
        else
        {
		    retval = C_FRAME(t2, accel, -jerk);
        }
        dsp_control(axis,FCTL_HOLD,FALSE);
    }

	if (t3 && !retval)
    {
		retval = S_FRAME(t3, 0);
        dsp_control(axis,FCTL_HOLD,FALSE);
    }
	if (t2 && !retval)
		retval = S_FRAME(t2, -jerk);
	if ( ((t1-1.0) > 0.0) && !retval)
		retval = A_FRAME(t1-1.0, -accel);

	dspPtr = oldDspPtr ;
	return retval ;
}



/*	This function turns all its parameters int16o counts, counts/sample,
	counts/(sample*sample), counts/(sample*sample*sample)
*/
static int16 LOCAL_FN ipcdsp_standardize(PDSP dsp, int16 axis, P_DOUBLE final, P_DOUBLE velocity, P_DOUBLE acceleration, P_DOUBLE jerk)
{
	LFIXED
		vel, accel, j, pp ;

	if (! (	ipcdsp_fixed_pos(dsp, axis, *final, &pp) ||
			ipcdsp_fixed_vel(dsp, axis, *velocity, &vel) ||
			ipcdsp_fixed_accel(dsp, axis, *acceleration, &accel) ||
			ipcdsp_fixed_jerk(dsp, axis, *jerk, &j)))
	{	*final = ipcdsp_double(&pp);
		*velocity = ipcdsp_double(&vel);
		*acceleration = ipcdsp_double(&accel);
		*jerk = ipcdsp_double(&j);
	}

	if ((sign_lfixed(vel) == 0) ||
		(sign_lfixed(accel) == 0) ||
		(sign_lfixed(j) == 0))
			dsp_error = DSP_ILLEGAL_PARAMETER ;

	return dsp_error ;
}


int16 FNTYPE ipcdsp_load_p_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	double
		x ;

	if (!ipcdsp_standardize(dsp, axis, &final, &vel, &accel, &jerk))
	{	x = final - dsp_last_command(dsp, axis) ;
		if (x)	/* Distance? */
		{	pcdsp_p_trap(dsp, axis, x,
				x < 0? -vel : vel,
				x < 0? -accel : accel,
				x < 0? -jerk : jerk);
			ipcdsp_last_frame(dsp, axis, final) ;
			dsp_set_last_command(dsp, axis, final) ;
		}
	}
	return dsp_error ;
}

int16 FNTYPE ipcdsp_p_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	if (! ipcdsp_load_p_move(dsp, axis, final, vel, accel, jerk))
		pcdsp_dwell_frame(dsp, axis) ;

	return dsp_error ;
}

int16 FNTYPE start_p_move(int16 axis, double final, double vel, double accel, double jerk)
{
	int16 r ;
    if (dsp_set_gate(dspPtr, axis))
    	return dsp_error ;

    r = ipcdsp_p_move(dspPtr, axis, final, vel, accel, jerk);
	dsp_reset_gate(dspPtr, axis);
	return (dsp_error = r);
}

int16 FNTYPE p_move(int16 axis, double final, double vel, double accel, double jerk)
{
	if (!start_p_move(axis, final, vel, accel, jerk))
		wait_for_done(axis);

	return dsp_error ;
}

