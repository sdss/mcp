/*
	llsetvel.c - just downloads frames for a controlled velocity move.
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
#include <stddef.h>



int16 FNTYPE pcdsp_load_velocity(PDSP pdsp, int16 axis, PFIXED newvel, PFIXED newaccel)
{	FRAME	f ;
	int16		e = DSP_OK,
			trigger = TRIGGER_POSITIVE | (pdsp->frame_action[axis]);
	LFIXED triggervel ;

	copy_lfixed(triggervel, *newvel);
	if (sign_lfixed(*newaccel) < 0)
		trigger |= TRIGGER_NEGATIVE ;
	sub_lfixed(triggervel, *newaccel);

	if (sign_lfixed(*newaccel) != 0)
		e = pcdsp_frame(pdsp, &f, "0l av uncd", axis, newaccel, &triggervel,
				FUPD_ACCEL | FTRG_VELOCITY, trigger, 
            pdsp->frame_control[axis] | FCTL_HOLD) ;

	if (! e)
		e = pcdsp_frame(pdsp, &f, "0l av und", axis, NULL,
				newvel, FUPD_ACCEL | FUPD_VELOCITY, 0);

	return e;
}
