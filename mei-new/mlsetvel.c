/*
	mlsetvel.c
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

static double LOCAL_FN current_velocity(PDSP pdsp, int16 axis)
{
	double
		current_vel ;
	int16 e = pcdsp_frames_left(pdsp, axis) || pdsp->laxis[axis].gate;

	if (e && (pdsp->laxis[axis].last == LA_VELOCITY))
		/* nothing */ ;
	else
	{
		if (! dsp_in_motion(pdsp, axis))	/* current velocity is zero? */
			copy_lfixed(pdsp->laxis[axis].vel, fixed_zero) ;
		else		/* current (command) velocity is non-zero. */
			pcdsp_get_velocity(pdsp, axis, &(pdsp->laxis[axis].vel));
	}
	ipcdsp_double_vel(pdsp, axis, &pdsp->laxis[axis].vel, &current_vel);

	return current_vel ;
}



/*	This function relies on 'current_velocity' being called before it,
	ensuring that LA_VELOCITY is there.
*/
static int16 LOCAL_FN velocity_change(PDSP pdsp, int16 axis, PFIXED newvel, PFIXED change)
{
	copy_lfixed(*change, *newvel) ;
	sub_lfixed(*change, pdsp->laxis[axis].vel) ;
	return 0;
}


int16 FNTYPE dsp_v_move(PDSP pdsp, int16 axis, double newvel, double accel)
{
	double
		current ;

	LFIXED
		fvel,
		faccel,
		change ;

	if ((axis < 0) || (axis >= pdsp->axes))
		return (dsp_error = DSP_INVALID_AXIS);

	if (accel < 0)
		accel = -accel ;

	current = current_velocity(pdsp, axis) ;
	if (current > newvel)
		accel = -accel ;

	ipcdsp_fixed_vel(pdsp, axis, newvel, &fvel);
	ipcdsp_fixed_accel(pdsp, axis, accel, &faccel) ;

	velocity_change(pdsp, axis, &fvel, &change) ;/* figure out if the change */
	sub_lfixed(change, faccel) ;				/* in velocity is less than */
	if (sign_lfixed(change) != sign_lfixed(faccel))	/* the acceleration.  If so, */
		copy_lfixed(faccel, fixed_zero) ;		/* then skip the accel frame. */

	if (dsp_set_gate(pdsp, axis))
		return dsp_error ;

	pcdsp_load_velocity(pdsp, axis, &fvel, &faccel) ;

	pcdsp_set_last(pdsp, axis, LA_VELOCITY, &fvel) ;
	return dsp_reset_gate(pdsp, axis) ;
}



int16 FNTYPE v_move(int16 axis, double newvel, double accel)
{	return dsp_v_move(dspPtr, axis, newvel, accel);
}
