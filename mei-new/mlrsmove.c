/*
	mlrsmove.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "idsp.h"
#include <math.h>


#define	DATA_STRUCT(dsp, axis, offset)		((dsp)->data_struct + (DS_SIZE * (axis)) + offset)
#define	S_FRAME(t, j)	 frame_m(&frame, "-0l tj u d", axis, (double) (t),(double) (j), FUPD_JERK | FTRG_TIME)

static int16 LOCAL_FN ipcdsp_cps(PDSP dsp, int16 axis, P_DOUBLE position, P_DOUBLE velocity, P_DOUBLE acceleration, P_DOUBLE jerk)
{
	LFIXED
		j, vel, accel, pp ;

	if (!(ipcdsp_fixed_pos(dsp, axis, *position, &pp) ||
		  ipcdsp_fixed_vel(dsp, axis, *velocity, &vel) ||
		  ipcdsp_fixed_accel(dsp, axis, *acceleration, &accel) ||
        ipcdsp_fixed_jerk(dsp, axis, *jerk, &j)))
	{	*position = ipcdsp_double(&pp);
		*velocity = ipcdsp_double(&vel);
		*acceleration = ipcdsp_double(&accel);
		*jerk = ipcdsp_double(&j);
	}
	return dsp_error ;
}

#define	F_POSITION				0x0D
#define	MF_DESTINATION			3

int16 FNTYPE start_rs_move(int16 axis, double distance, double vel, double accel, double jerk)
{
	FRAME frel, fupdate, frame;
	long m1, m2, m3;
	double
		t0, t1, t3, lc,
		x = distance, 
		v = vel, 
		a = accel,
		j = jerk;
	int16 dir_negative = 0, retval, fctl;
	LFIXED p;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	if(x < 0)
		dir_negative = 1;

	if (! ipcdsp_cps(dspPtr, axis, &x, &v, &a, &j))
	{	if (idsp_check_4params(&x, &v, &a, &j))		/* convert to positive values */
			return dsp_error;
	}

	if (! x)		/* No distance? */
		return (dsp_error = DSP_OK);

/*	Round off the jerk term so that it's an even number of samples.	*/
	m1 = (long)(a / j + 0.5);
	if (! m1)
		m1 = 1 ;

/*	Round off accel.	*/
	m2 = (long)(v / a + 0.5);
	if (m2 < m1)
	{
		m2 = m1;
	}

	m3 = (long)(x / v + 0.5);
	if(m3 < (m2 + m1))
	{
		m3 = m2 + m1;
	}
    
	t0 = m1;
	t1 = m2 - m1;
	t3 = m3 - m2 - m1;
	jerk = x/(((double)m1) * ((double)m2) * ((double)m3));
    
/*  All frames trigger on time, update jerk only */
/*  Don't download frames with T = 0 */

	if(dir_negative)
	{
		jerk = - jerk;
		x = -x;
	}
	ipcdsp_fixed(distance, &p);

#ifdef MEI_SWAPLONG
	mei_swaplong((unsigned32*)&(p.whole));
	mei_swaplong((unsigned32*)&(p.frac));
#endif /* MEI_SWAPLONG */

	fctl = dspPtr->frame_control[axis];
	dspPtr->frame_control[axis] |= FCTL_HOLD;
	dsp_set_gate(dspPtr, axis);
	retval = dsp_move_frame(&frel, axis, MF_ADD,
				0,
      			(P_DSP_DM)DATA_STRUCT(dspPtr, axis, DS_POSITION),
		      	MF_DATA_AREA,
      			sizeof(p) / sizeof(DSP_DM),
		      	(PDSP_DM) &p);
	dspPtr->frame_control[axis] = fctl;
	if (t0 && !retval)
		retval = S_FRAME(t0, jerk);
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
	frame_m(&fupdate, "0l und", axis,
			FUPD_JERK | FUPD_ACCEL | FUPD_VELOCITY | FUPD_POSITION, 0);

	{	P_DSP_DM reldest = fupdate.current + F_POSITION;
		dsp_write_dm((unsigned16)(frel.current + MF_DESTINATION), reldest);
	}

	get_last_command(axis, &lc);
	dsp_set_last_command(dspPtr, axis, x + lc);
	dsp_reset_gate(dspPtr, axis);

	return dsp_error ;
}

int16 FNTYPE rs_move(int16 axis, double distance, double velocity, double acceleration, double jerk)
{
	if (!start_rs_move(axis, distance, velocity, acceleration, jerk))
		wait_for_done(axis);
	return dsp_error ;
}

