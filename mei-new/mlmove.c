/*
	mlmove.c - medium-level trapezoidal profile moves.
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

static long last_move_time[PCDSP_MAX_AXES];

int16 FNTYPE get_last_command(int16 axis, P_DOUBLE pos)
{
	if (pcdsp_sick(dspPtr, axis))		/* check axis */
		return dsp_error ;

	*pos=dsp_last_command(dspPtr, axis);/* return last command from	*/
	return dsp_error ;					/* lower level function		*/
}

int16 FNTYPE set_last_command(int16 axis, double position)
{	if (pcdsp_sick(dspPtr, axis))		/* check axis */
		return dsp_error;

	return dsp_set_last_command(dspPtr, axis, position);
}

int16 LOCAL_FN ipcdsp_move_frame(PDSP dsp, int16 axis, double a, unsigned long t, int16 hold)
{
	FRAME frame ;
	LFIXED lfixed ;

	frame_clear(&frame);
	if(dsp->FRAME_ALLOCATE(&frame, dsp, axis)==DSP_OK) 
    {									/* allocate free frame and */
        if(hold)						/* hold frame if specified */
            frame.f.control |= FCTL_HOLD;				

        frame.f.time = t;				/* load frame with move params */
		ipcdsp_fixed(a, &lfixed) ;
		copy_lfixed_to_fixd(frame.f.accel, lfixed);
		frame.f.trig_update =  FUPD_ACCEL | FTRG_TIME;
		frame.f.trig_action = dsp->frame_action[axis] ;

		dsp->FRAME_DOWNLOAD(&frame);	/* send frame to DSP */
	}
	return dsp_error ;
}

int16 LOCAL_FN ipcdsp_last_frame(PDSP dsp, int16 axis, double x)
{
	FRAME frame ;
	LFIXED lfixed ;

	frame_clear(&frame);

	if(dsp->FRAME_ALLOCATE(&frame, dsp, axis)==DSP_OK) 
	{									/* allocate free frame and */
		ipcdsp_fixed(x, &lfixed) ;		/* insert final position into frame	*/
		copy_lfixed(frame.f.position, lfixed) ;			
		frame.f.time = 1;
		frame.f.trig_update =	FUPD_POSITION | FUPD_VELOCITY | FUPD_JERK |
								FUPD_ACCEL | FTRG_TIME;
		frame.f.trig_action = dsp->frame_action[axis]  ;

		dsp->FRAME_DOWNLOAD(&frame);	/* send frame to DSP */
	}
	return dsp_error ;
}

long FNTYPE ipcdsp_move_time(PDSP dsp, int16 axis)
{
    dsp = dsp;
    return last_move_time[axis];
}

int16 FNTYPE pcdsp_trap(PDSP dsp, int16 axis, double x, double v, double a1, double a2)
{
	/*
	 *	Computes trapezoidal motion parameters 
	 *  and sends frame information to DSP
	 */
	double cx, dx, dx1,dx3,t1, t2, t3;
	long m1,m2,m3;
	int16 dir_negative = 0;

	if (pcdsp_sick(dsp, axis))			/* check axis */
		return dsp_error ;

	cx = dsp_last_command(dsp, axis);	/* compute distance delta */
	dx = x - cx ;
	if(dx >  2147483647.0)	dx = dx - 4294967296.0;
	if(dx < -2147483648.0)	dx = dx + 4294967296.0;
	if(dx < 0)				dir_negative = 1;

										/* convert to positive values */
	if (idsp_check_4params(&dx, &v, &a1, &a2))
		return dsp_error;

	if (! dx)							/* No distance? Don't bother trying */
	{
		dsp_error = DSP_NO_DISTANCE;	/* Let calling function know that */
		return(dsp_error);				/* no gate was set.	*/
	}												
	
	m1 = (long)(v/a1 + 0.5);			/* Calculate acceleration duration */
	if(m1 == 0L)
	{
		m1 = 1L;
	}
	m3 = (long)(v/a2 + 0.5);			/* Calculate deceleration duration */
	if(m3 == 0L)
	{
		m3 = 1L;
	}

	/* 
	 * Calculate actual accels 
	 */
    t1 = m1;
    t3 = m3;
    dx1 = a1*(t1*(t1+1.))/2.0;
    dx3 = (a1 * t1)*t3 - a2*(t3*(t3+1.))/2.0;
    m2 = (long)((dx - (dx1 + dx3))/v + 0.5);

	if(m2 < 0L)							/* is the move long enough?	*/
	{
		m1 = (long)(sqrt((2.0*dx)/(a1 * (1.0 + a1/a2))) + 0.5);
		if(m1 == 0L)
		{
			m1 = 1L;
		}
		m3 = (long)((a1/a2) * m1);
		if(m1 == 0L)
		{
			m1 = 1L;
		}
        m2 = 0L;
	}
	t1 = m1;
	t2 = m2;
	t3 = m3;

    /*	
	 * Kinematic equations:
     *   x = a1*t1*(t1+1)/2 + a1*t1*t2 + a1*t1*t3 - a2*t3*(t3-1)/2
     *   a1*t1 = a2*t3
     */

    a1 = dx/(t1*(t2 + (t3 + t1)/2.0));	/* using x equation	*/
    a2 = a1*t1/t3;						/* using v equation	*/

    last_move_time[axis] = m1 + m2 + m3;

	if(dir_negative)
    {
		a1 = -a1;
        a2 = -a2;
    }

	/*
	 * send frames to DSP
	 */
	if (dsp_set_gate(dspPtr, axis))		/* set gate for this axis */
		return dsp_error ;
										/* accelleration frame */
	if (ipcdsp_move_frame(dsp, axis,   a1 , m1, TRUE))
		return dsp_error ;

	if(m2)								/* constant velocity frame */
	{									/* if distance is long enough */
		if (ipcdsp_move_frame(dsp, axis, 0.0, m2, FALSE))
			return dsp_error ;
	}
										/* decelleration frame */
	if (ipcdsp_move_frame(dsp, axis, -a2, m3, FALSE))	
		return dsp_error ;

	if (ipcdsp_last_frame(dsp, axis, x))/* last frame */
		return dsp_error ;

	if (pcdsp_dwell_frame(dsp, axis))	/* add dwell frame to axis */
		return dsp_error ;

	return dsp_error;
}

static int16 LOCAL_FN ipcdsp_cps(PDSP dsp, int16 axis, P_DOUBLE position, P_DOUBLE velocity, P_DOUBLE accel1,  P_DOUBLE accel2)
{
	/*
	 *	Parameter conversion function
	 */
	LFIXED
		vel, ac1, ac2, pp ;

	if (! (ipcdsp_fixed_pos(dsp, axis, *position, &pp) ||
		   ipcdsp_fixed_vel(dsp, axis, *velocity, &vel) ||
		   ipcdsp_fixed_accel(dsp, axis, *accel1, &ac1) ||
		   ipcdsp_fixed_accel(dsp, axis, *accel2, &ac2)))
	{	*position = ipcdsp_double(&pp);
		*velocity = ipcdsp_double(&vel);
		*accel1 = ipcdsp_double(&ac1);
		*accel2 = ipcdsp_double(&ac2);
	}
	return dsp_error ;
}

int16 FNTYPE ipcdsp_load_move(PDSP dsp, int16 axis,
		double position, double velocity, double acceleration, double deceleration)
{
	if (pcdsp_sick(dsp, axis))
		return dsp_error ;

	/*
	 * Converts paramaters, calls trapizodal function, 
	 * and updates last command information.
	 */
	if (! ipcdsp_cps(dsp, axis, &position, &velocity, &acceleration, &deceleration))
	{	
		if (! pcdsp_trap(dsp, axis, position, velocity, acceleration, deceleration))
			dsp_set_last_command(dsp, axis, position);
	}
				
	return dsp_error ;	
}

int16 FNTYPE start_t_move(int16 axis, double posn, double vel, double accel, double decel)
{
	int16 r ;
    	
	r = ipcdsp_load_move(dspPtr, axis, posn, vel, accel, decel ) ;

	if (r != DSP_NO_DISTANCE)		
		dsp_reset_gate(dspPtr, axis);	/* Gate was set, reset it now. */
	
	return (dsp_error = r);
	
}

int16 FNTYPE t_move(int16 axis, double position, double velocity, double acceleration, double deceleration)
{
	if (! start_t_move(axis, position, velocity, acceleration , deceleration))
		wait_for_done(axis);			/* start trap. move and wait */

	return dsp_error ;
}

int16 FNTYPE start_move(int16 axis, double posn, double vel, double accel)
{  
	return start_t_move(axis, posn, vel, accel, accel);	/* accel == decel */
}

int16 FNTYPE move(int16 axis, double position, double velocity, double acceleration)
{
	if (! start_move(axis, position, velocity, acceleration))
		wait_for_done(axis);			/* start one axis and wait til done	*/

	return dsp_error ;
}

int16 FNTYPE start_move_all(int16 len, P_INT axes, P_DOUBLE pos, P_DOUBLE vel, P_DOUBLE accel )
{
/*
 *	sets position, velocity and acceleration for all axis' in "axis" array.
 */
	int16 i, retval = dsp_error, ax_array[PCDSP_MAX_AXES], ax_count=0;

	for (i = 0; i < len; i++)			/* check each axis */
		if (pcdsp_sick(dspPtr, axes[i]))
			return dsp_error ;
	/* 
	 * In the following loop, gates are set for each axis 
	 * only if pos != curr_position 
	 */
	for (i = 0; i < len; i++)
	{	
		if(!ipcdsp_load_move(dspPtr, axes[i], pos[i], vel[i], accel[i], accel[i]))
		{	ax_array[ax_count] = axes[i];	/* if axis was loaded, gate was	*/
			ax_count++;						/* set, add to list to be reset */
		}
		if(dsp_error == DSP_NO_DISTANCE)	/* DSP return was OK. dsp_error was */
			dsp_error = DSP_OK;				/* just used to flag an unset gate.	*/
											/* We reset dsp_error to reflect    */
											/* that things are OK.				*/
		if(dsp_error)
			retval = dsp_error;				/* keep track of any errors */
	}

	if(ax_count)							/* reset gates now */
		reset_gates(ax_count,ax_array);

	return (dsp_error = retval);
}

int16 FNTYPE wait_for_all(int16 len, P_INT axes)
{
	int16 i ;
	for (i = 0; i < len; i++)						/* wait until all axis */
		while (!axis_done(axes[i]) && !dsp_error)	/* in array are done.  */
			;

	return dsp_error ;
}

int16 FNTYPE move_all(int16 len, P_INT axes, P_DOUBLE pos, P_DOUBLE vel, P_DOUBLE accel)
{
	if (! start_move_all(len, axes, pos, vel, accel) )/* start all axis and wait */	
		wait_for_all(len, axes) ;					  /* until they are done.	 */

	return dsp_error ;
}

