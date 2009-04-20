/*
	mlstatus.c
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
#include "sercos.h"


int16 FNTYPE set_stop(int16 axis)
{	if (! pcdsp_sick(dspPtr, axis))
		pcdsp_set_event(dspPtr, axis, STOP_EVENT) ;
	return dsp_error ;
}

int16 FNTYPE set_e_stop(int16 axis)
{	if (! pcdsp_sick(dspPtr, axis))
		pcdsp_set_event(dspPtr, axis, E_STOP_EVENT) ;
	return dsp_error ;
}

int16 FNTYPE controller_idle(int16 axis)
{	if(!pcdsp_sick(dspPtr, axis))
	{	if(dspPtr->sercos)
			enable_sercos_amplifier(axis, FALSE);
		dsp_idle(dspPtr, axis);
	}
	return dsp_error ;
}

int16 FNTYPE pcdsp_clear_status(PDSP pdsp, int16 axis, int16 update)
{	int16 timeout = TIMEOUT ;
	DSP_DM dummy, fault, illegal_state;
	LFIXED lfixed ;

	if(!pcdsp_sick(pdsp, axis))
	{	if((pcdsp_status(pdsp, axis) & IN_SEQUENCE) ||
			(pcdsp_event(pdsp, axis)))
			return (dsp_error = DSP_CLEAR_STATUS) ;

		if(dsp_error)
			return dsp_error;

		pcdsp_set_last(pdsp, axis, LA_UNDEFINED, &fixed_zero) ;

		if(update)
		{	
			/* Clear feedback fault and illegal state registers. */
			fault = dsp_read_dm(FB_FAULT);
			illegal_state = dsp_read_dm(FB_ILLEGAL_STATE);
			fault &= (~(0x1 << axis));
			illegal_state &= (~(0x1 << axis));
			dsp_write_dm(FB_FAULT, fault);
			dsp_write_dm(FB_ILLEGAL_STATE, illegal_state);
				
			if (pcdsp_actual(pdsp, axis, &lfixed))
				return dsp_error ;
			if (pcdsp_set_command(pdsp, axis, &lfixed))
				return dsp_error ;
	 		pcdsp_transfer_block(pdsp, TRUE, FALSE, 0, 1, &dummy) ;
			reset_integrator(axis);
		}
		
		dsp_clear_gate(pdsp, axis);	/* clear the gate counter and flag */
		dsp_run(pdsp, axis);		/* set event register to PC_STATUS_OVERRIDE */
 		wait_cycles(2);				/* wait for DSP */

		while (timeout-- && ((pcdsp_status(pdsp, axis) & IN_SEQUENCE) ||
			pcdsp_event(pdsp, axis)))
			;

		if (! timeout)
			dsp_error = DSP_TIMEOUT_ERROR ;
	}
	return dsp_error ;
}

int16 FNTYPE controller_run(int16 axis)
{	return pcdsp_clear_status(dspPtr, axis, TRUE) ;
}

int16 FNTYPE clear_status(int16 axis)
{	if(!pcdsp_sick(dspPtr, axis))
	{	if(axis_state(axis) < ABORT_EVENT)
			return pcdsp_clear_status(dspPtr, axis, FALSE);
	}
	return dsp_error;
}

int16 FNTYPE pcdsp_in_sequence(PDSP pdsp, int16 axis)
{
	if (pcdsp_sick(pdsp, axis))
		return 0 ;

	if (pcdsp_event(pdsp, axis))
		return TRUE;

	if (pdsp->laxis[axis].gate)
		return TRUE ;

	return dsp_in_sequence(pdsp, axis) ;
}

int16 FNTYPE in_sequence(int16 axis)
{	return pcdsp_in_sequence(dspPtr, axis) ;
}

int16 FNTYPE in_position(int16 axis)
{
	if (pcdsp_sick(dspPtr, axis))
		return 0;

	return dsp_in_position(dspPtr, axis) ;
}

int16 FNTYPE in_motion(int16 axis)
{
	if (pcdsp_sick(dspPtr, axis))
		return 0;

	return dsp_in_motion(dspPtr, axis) ;
}

int16 FNTYPE negative_direction(int16 axis)
{
	if (pcdsp_sick(dspPtr, axis))
		return 0;

	return dsp_neg_direction(dspPtr, axis) ;
}

int16 FNTYPE pcdsp_frames_left(PDSP pdsp, int16 axis)
{
	DSP_DM inptr, outptr ;

	if (pcdsp_sick(pdsp, axis))
		return 0;

	inptr = pdsp->inptr + axis ;
	outptr = pdsp->outptr + axis ;
	return (idsp_read_dm(pdsp, inptr) != idsp_read_dm(pdsp, outptr)) ;
}

int16 FNTYPE frames_left(int16 axis)
{	return pcdsp_frames_left(dspPtr, axis) ;
}

int16 FNTYPE motion_done(int16 axis)
{
	if (pcdsp_event(dspPtr, axis))
		return FALSE ;

	if (pcdsp_status(dspPtr, axis) & DSP_IN_MOTION)
		return FALSE ;

	return TRUE ;
}

int16 FNTYPE axis_done(int16 axis)
{	int16 r ;

	if (pcdsp_sick(dspPtr, axis))
		return -1 ;

	if (pcdsp_event(dspPtr, axis))
		return FALSE ;


	r = pcdsp_status(dspPtr, axis) ;
	if (r & DSP_IN_MOTION)
		return FALSE ;

	if ((pcdsp_state(dspPtr, axis) & 0x000F) == ABORT_EVENT)
		return TRUE ;

	if (r & IN_POSITION)				return TRUE ;
	return FALSE ;
}

int16 FNTYPE axis_status(int16 axis)
{	if(pcdsp_sick(dspPtr, axis))
		return -1;
	return pcdsp_status(dspPtr, axis);
}

int16 FNTYPE axis_state(int16 axis)
{	if(!pcdsp_sick(dspPtr, axis))
		return pcdsp_state(dspPtr, axis) & 0x000F;
	return -1;
}

int16 FNTYPE axis_source(int16 axis)
{	if(!pcdsp_sick(dspPtr, axis))
		return (pcdsp_state(dspPtr, axis) >> 4) & 0x00FF;
	return -1;
}

int16 FNTYPE wait_for_done(int16 axis)
{
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	while (! motion_done(axis))
		;

	return dsp_error;
}

