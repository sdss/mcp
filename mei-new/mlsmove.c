/*
	MLSMOVE.C - medium level S-curve moves.
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


#ifdef MEI_OS9
#	define SQRT(n)		sqrt(n)
#endif

#ifndef SQRT
#	define SQRT(n)		sqrtl(n)
#endif

#define	S_FRAME(t, j)	 frame_m(&frame, "-0l tj u d", axis, (double) (t),(double) (j), FUPD_JERK | FTRG_TIME)

/*	uom: counts, counts/sample, counts/(sample*sample), etc		*/
int16 FNTYPE pcdsp_s_trap(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	FRAME
		frame ;
	long
		m1, m2, m3 ;
	double
		t0, t1, t3 ;
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
	if (m2 < m1)
    {
        m2 = m1;
    }

	m3 = (long)(final / vel + 0.5);
    if(m3 < (m2 + m1))
    {
        m3 = m2 + m1;
    }
    
    t0 = m1;
    t1 = m2 - m1;
    t3 = m3 - m2 - m1;
    jerk = final/( ((double)m1) * ((double)m2) * ((double)m3) );
    
/*  All frames trigger on time, update jerk only */
/*  Don't download frames with T = 0 */

	retval = DSP_OK ;
    dsp_control(axis, FCTL_HOLD, TRUE);
	if (t0 && !retval)
		retval = S_FRAME(t0, jerk);
    dsp_control(axis, FCTL_HOLD, FALSE);
	if (t1 && !retval)
		retval = S_FRAME(t1, 0);
	if (t0 && !retval)
		retval = S_FRAME(t0, -jerk);
	if (t3 && !retval)
		retval = S_FRAME(t3, 0);
	if (t0 && !retval)
		retval = S_FRAME(t0, -jerk);
	if (t1 && !retval)
		retval = S_FRAME(t1, 0);
	if (t0 && !retval)
		retval = S_FRAME(t0, jerk);

	dspPtr = oldDspPtr ;
	return retval ;
}

/*	This function turns all its parameters int16 counts, counts/sample,
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

int16 FNTYPE ipcdsp_load_s_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	double
		x ;

	if (!ipcdsp_standardize(dsp, axis, &final, &vel, &accel, &jerk))
	{	x = final - dsp_last_command(dsp, axis) ;
		if (x)		/* Distance? */
		{	if (pcdsp_s_trap(dsp, axis, x,
					x < 0? -vel : vel,
					x < 0? -accel : accel,
					x < 0? -jerk : jerk))
				return dsp_error;
			ipcdsp_last_frame(dsp, axis, final);
  			dsp_set_last_command(dsp, axis, final);
		}
	}
	return dsp_error ;
}

int16 FNTYPE ipcdsp_s_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{
	if (! ipcdsp_load_s_move(dsp, axis, final, vel, accel, jerk))
		pcdsp_dwell_frame(dsp, axis) ;

	return dsp_error ;
}

int16 FNTYPE start_s_move(int16 axis, double final, double vel, double accel, double jerk)
{
	int16 r ;
    if (dsp_set_gate(dspPtr, axis))
    	return dsp_error ;
    r = ipcdsp_s_move(dspPtr, axis, final, vel, accel, jerk);
	dsp_reset_gate(dspPtr, axis);
	return (dsp_error = r) ;
}

int16 FNTYPE s_move(int16 axis, double final, double vel, double accel, double jerk)
{
	if (!start_s_move(axis, final, vel, accel, jerk))
		wait_for_done(axis);

	return dsp_error ;
}

/* Alternate S-curve profile algorithm.  This algorithm explicitly calculates the S-curve
	profile.  It guarantees a continuous profile with an accurate final position.
	Depending on the parameters, the velocity profile time is reduced to zero and then
	the	acceleration/deceleration profile time is reduced to zero.
*/
int16 FNTYPE pcdsp_sprof_trap(int16 axis, double delta, double vel, double accel, double jerk)
{
	FRAME frame;
	double ta, tv, tj, delta_j;
	int16 retval, dir_negative = 0;

	if (delta < 0)
	{	dir_negative = 1;
		delta = -delta;
	}

	/* Calculate the accel time.  If the accel time is negative, then the target
		accel must be adjusted so that ta = 0.  This will preserve the specified
		velocity.  Note that a negative accel time is not valid, because it is
		impossible to go backwards in time.
	*/ 
	if (accel*accel/jerk > vel)
	{ 	ta = 0.0;
		accel = SQRT(jerk*vel);
	}
	else
		ta = ((vel - accel*accel/jerk)/accel);

	/* Calculate the distance during the accel and jerk profile portion. */
	delta_j = (2.0*(accel*accel*accel)/(jerk*jerk))
				+ ((3.0*accel*accel*ta)/jerk)
				+ (accel*ta*ta);

	if (delta_j > delta)	/* Is the distance during jerk and accel too far? */
	{	tv = 0.0; 			/* Remove the constant velocity profile */      
		if (ta > 0.0)
		{	double   x;
			x = sqrt(1+(4*delta*jerk*jerk/(accel*accel*accel)));
			
			if(x > 3.0)
			{	tj = accel/jerk;
				ta = (0.5*tj)*(x - 3.0);
			}
			else
				ta = 0.0;
		}
		if (ta < 1.0)
		{	ta = 0.0;       
			tj = pow((delta/(2.0*jerk)), .3333333);
		}
	}                                         
	else                                      
	{	tv = (delta - delta_j)/vel;
		tj = accel/jerk;
	}

	if (tj < 1.0)
	{	dsp_error = DSP_ILLEGAL_PARAMETER;
		return dsp_error;
	}
	
	/* Reverse calculate to eliminate roundoff errors.  Since the DSP uses 
		whole time periods (no fractional time periods) to calculate the
		trajectory, the jerk, accel, or vel, may need to be adjusted to
		guarantee the accuracy of the final position.
	*/
	tj = floor(tj);
	accel = tj*jerk;

	if (ta != 0.0)		/* Is there an accel portion? */
	{	if (tv == 0.0)
		{	double   x;
			x = sqrt(1+(4*delta*jerk*jerk/(accel*accel*accel)));
			ta = (0.5*accel/jerk)*(x - 3.0);		/* Recalculate ta */
		}
		else
			ta = ((vel - accel*accel/jerk)/accel);	/* Recalculate ta */
		ta = floor(ta);
	}
	
	if (tv != 0.0) 		/* Is there a vel portion? */
	{	delta_j = (2.0*(accel*accel*accel)/(jerk*jerk))
					+ ((3.0*accel*accel*ta)/jerk)
					+ (accel*ta*ta);             
		vel = ta*accel + (accel*accel/jerk);	/* Recalculate vel */
		tv = (delta - delta_j)/vel;				/* Recalculate tv */
		tv = floor(tv);
	}

	jerk = delta / (tj * (tj + ta) * (2 * tj + ta + tv));

	if (dir_negative)
		jerk = -jerk;

	/*  Now, download the frames.  All frames trigger on time and update the
		jerk only.  Don't download frames with zero time. */

	retval = DSP_OK;
	dsp_control(axis, FCTL_HOLD, TRUE);
	if (tj && !retval)
		retval = S_FRAME(tj, jerk);
	dsp_control(axis, FCTL_HOLD, FALSE);
	if (ta && !retval)
		retval = S_FRAME(ta, 0);
	if (tj && !retval)
		retval = S_FRAME(tj, -jerk);
	if (tv && !retval)
		retval = S_FRAME(tv, 0);
	if (tj && !retval)
		retval = S_FRAME(tj, -jerk);
	if (ta && !retval)
		retval = S_FRAME(ta, 0);
	if (tj && !retval)
		retval = S_FRAME(tj, jerk);

   return retval;
}

int16 FNTYPE ipcdsp_load_sprof_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{	double x;
	if (!ipcdsp_standardize(dsp, axis, &final, &vel, &accel, &jerk))
	{	x = final - dsp_last_command(dsp, axis);
		if (x)	/* Distance? */
		{	if (pcdsp_sprof_trap(axis, x, vel, accel, jerk))
				return dsp_error;
			ipcdsp_last_frame(dsp, axis, final);
			dsp_set_last_command(dsp, axis, final);
		}
	}
	return dsp_error;
}

int16 FNTYPE ipcdsp_sprof_move(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{	if (!ipcdsp_load_sprof_move(dsp, axis, final, vel, accel, jerk))
		pcdsp_dwell_frame(dsp, axis);
	return dsp_error;
}

int16 FNTYPE start_sprof_move(int16 axis, double final, double vel, double accel, double jerk)
{
	int16 r ;
    if (dsp_set_gate(dspPtr, axis))
    	return dsp_error ;
    r = ipcdsp_sprof_move(dspPtr, axis, final, vel, accel, jerk);
	dsp_reset_gate(dspPtr, axis);
	return (dsp_error = r) ;
}

int16 FNTYPE sprof_move(int16 axis, double final, double vel, double accel, double jerk)
{
	if (!start_sprof_move(axis, final, vel, accel, jerk))
		wait_for_done(axis);

	return dsp_error ;
}
