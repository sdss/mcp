/*
	mlrmove.c
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

static int16 LOCAL_FN ipcdsp_cps(PDSP dsp, int16 axis, P_DOUBLE position, P_DOUBLE velocity, P_DOUBLE acceleration)
{
	LFIXED
		vel, accel, pp ;

	if (! (ipcdsp_fixed_pos(dsp, axis, *position, &pp) ||
		   ipcdsp_fixed_vel(dsp, axis, *velocity, &vel) ||
		   ipcdsp_fixed_accel(dsp, axis, *acceleration, &accel)))
	{	*position = ipcdsp_double(&pp);
		*velocity = ipcdsp_double(&vel);
		*acceleration = ipcdsp_double(&accel);
	}
	return dsp_error ;
}

#define	get_frame(f)	frame_clear(&(f)), dspPtr->FRAME_ALLOCATE(&(f), dspPtr, axis)
#define	F_POSITION		0x0D
#define	MF_DESTINATION	3
#define	MF_SOURCE_2		5
#define	MF_DATA			6

int16 FNTYPE start_r_move(int16 axis, double distance, double vel, double accel)
{
	FRAME frel, fupdate;
	double
		t1, t2,
		x = distance,
		v = vel,
		a = accel;
	int16
		fctl,
		u = FUPD_ACCEL | FTRG_TIME,
		dir_negative = 0;
	LFIXED p;
	long m1,m2;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	if(x < 0)
      dir_negative = 1;

	if (! ipcdsp_cps(dspPtr, axis, &x, &v, &a))
	{	if (idsp_check_3params(&x, &v, &a))		/* convert to positive values */
			return dsp_error;
	}

	if (! x)		/* No distance? */
		return (dsp_error = DSP_OK);

	m1 = (long)(v/a + 0.5);
	if(m1 == 0L)
	{
		m1 = 1L;
	}
	m2 = (long)(x/v + 0.5);
	if(m2 <= m1)
	{
		m1 = (long)(sqrt(x/a) + 0.5);
		if(m1 == 0L)
		{
			m1 = 1L;
		}
		m2 = m1;                                  
	}

	a = x/(((double)m1)*((double)m2));     /* important */
	t1 = m1;
	t2 = m2 - m1;

	if(dir_negative)
	{
		x = -x;
		v = -v;
		a = -a;
	}
	ipcdsp_fixed(x, &p);

#ifdef MEI_SWAPLONG
	mei_swaplong((unsigned32*) &(p.whole));
	mei_swaplong((unsigned32*) &(p.frac));
#endif /* MEI_SWAPLONG */

	fctl = dspPtr->frame_control[axis];
	dspPtr->frame_control[axis] |= FCTL_HOLD;			/* turn on hold bit */
	dsp_set_gate(dspPtr, axis);
	dsp_move_frame(&frel, axis, MF_ADD, 0,
			(P_DSP_DM)(DATA_STRUCT(dspPtr, axis, DS_POSITION)),
			MF_DATA_AREA,
			sizeof(p) / sizeof(DSP_DM),
			(PDSP_DM) &p);
	dspPtr->frame_control[axis] = fctl;					/* turn off hold bit */
	frame_m(NULL, "0l -at ud", axis, a, t1, u);
	if (t2 >= 1.0)
		frame_m(NULL, "0l -vt ud", axis, v, t2, u | FUPD_VELOCITY);
	frame_m(NULL, "0l -at ud", axis, -a, t1, u) ;
	frame_m(&fupdate, "0l und", axis, FUPD_ACCEL | FUPD_VELOCITY | FUPD_POSITION, 0) ;

	{	P_DSP_DM reldest = fupdate.current + F_POSITION ;
		dsp_write_dm((unsigned16)(frel.current + MF_DESTINATION), reldest);
	}
	dsp_reset_gate(dspPtr, axis);
	get_last_command(axis, &a) ;
	dsp_set_last_command(dspPtr, axis, x + a) ;

	return dsp_error ;
}

int16 FNTYPE r_move(int16 axis, double distance, double velocity, double acceleration)
{
	if (!start_r_move(axis, distance, velocity, acceleration))
		wait_for_done(axis);
	return dsp_error ;
}
