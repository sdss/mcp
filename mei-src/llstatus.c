/*
	llstatus.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include "idsp.h"

#	define	STATUS(dsp, axis, word)			((dsp)->pc_status + ((axis) << 1) + (word))

#	define	REALTIME(dsp, axis)				(P_DSP_DM)STATUS(dsp, axis, 0)
#	define	STATE(dsp, axis)				(P_DSP_DM)STATUS(dsp, axis, 1)


int16 FNTYPE pcdsp_status(PDSP pdsp, int16 axis)
{	return idsp_read_dm(pdsp, REALTIME(pdsp, axis)) ;
}

int16 FNTYPE pcdsp_state(PDSP pdsp, int16 axis)
{	return idsp_read_dm(pdsp, STATE(pdsp, axis)) ;
}

int16 FNTYPE pcdsp_event(PDSP pdsp, int16 axis)
{	return idsp_read_dm(pdsp, (P_DSP_DM)(pdsp->pc_event + axis));
}


int16 FNTYPE pcdsp_set_event(PDSP pdsp, int16 axis, DSP_DM event)
{	event |= (ID_PC_COMMAND << 4);
	idsp_write_dm(pdsp, (P_DSP_DM)(pdsp->pc_event + axis), event);
	return dsp_error ;
}


