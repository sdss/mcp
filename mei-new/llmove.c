/*
	llmove.c - an all integer trapezoidal profile move routine.
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


/*
	pcdsp_ll_move - this routine downloads trapezoidal profile move frames to the board.  If
	the velocity time is less than or equal to zero, then the velocity frame is ignored.
*/
int16 FNTYPE pcdsp_ll_move(PDSP pdsp, int16 axis, PFIXED vel, PFIXED veltime, PFIXED accel, PFIXED acceltime)
{
	FRAME f ;
	int16 e ;

	e = pcdsp_frame(pdsp, &f, "0l cat ud",
				axis, FCTL_DEFAULT | FCTL_HOLD,
				accel, acceltime, FUPD_ACCEL | FTRG_TIME) ;

	if (!e && (sign_lfixed(*veltime) > 0))
		e = pcdsp_frame(pdsp, &f, "0l vt ud",
				axis, vel, veltime, FUPD_ACCEL | FUPD_VELOCITY | FTRG_TIME) ;

	if (!e)
	{	neg_lfixed(*accel) ;
		e = pcdsp_frame(pdsp, &f, "0l at ud",
					axis, accel, acceltime, FUPD_ACCEL | FTRG_TIME) ;
	}

	if (!e)
		e = pcdsp_frame(pdsp, &f, "0l ud",
				axis, FUPD_VELOCITY | FUPD_ACCEL) ;

	return e;
}

double LOCAL_FN dsp_last_command(PDSP pdsp, int16 axis)
{	double r ;
	LFIXED lfixed ;
	PDSP p = dspPtr ;
	int16 e = pcdsp_frames_left(pdsp, axis) || pdsp->laxis[axis].gate, f = 0;
	dspPtr = pdsp ;
	f = e && (pdsp->laxis[axis].last == LA_COMMAND);
	if (!f)
		f = pdsp->flags[axis] & DF_AXIS_BUSY ;
	if (f)
		r = ipcdsp_double(&(pdsp->laxis[axis].pos));
	else
	{
		pcdsp_command(pdsp, axis, &lfixed);
		r = ipcdsp_double(&lfixed) ;
	}
	dspPtr = p ;
	return r;
}

int16 LOCAL_FN dsp_set_last_command(PDSP pdsp, int16 axis, double final)
{	LFIXED lfixed ;
	ipcdsp_fixed(final, &lfixed);
	return pcdsp_set_last(pdsp, axis, LA_COMMAND, &lfixed) ;
}

int16 LOCAL_FN idsp_check_3params(double * x, double * v, double * a)
{
	if ((! *v) || (! *a))	/* valid parameters? */
		return (dsp_error = DSP_ILLEGAL_PARAMETER);

    if (*x < 0)			*x = - *x;		/* convert to positive values */
    if (*v < 0)			*v = - *v;
	if (*a < 0)			*a = - *a;

	return (dsp_error = DSP_OK) ;
}

int16 LOCAL_FN idsp_check_4params(double * x, double * v, double * a, double * j)
{
	if ((! *v) || (! *a) || (! *j))		/* valid parameters? */
		return (dsp_error = DSP_ILLEGAL_PARAMETER);

    if (*x < 0)			*x = - *x;		/* convert to positive values */
    if (*v < 0)			*v = - *v;
	if (*a < 0)			*a = - *a;
	if (*j < 0)			*j = - *j;

	return (dsp_error = DSP_OK) ;
}
