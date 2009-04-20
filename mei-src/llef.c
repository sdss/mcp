/*
	llef.c
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


static P_DSP_DM LOCAL_FN pcdsp_ef_address(PDSP pdsp, int16 axis, int16 e_frame)
{	return idsp_read_dm(pdsp, DM_EVENT_FRAMES) + FRAME_SIZE
			* (axis + pdsp->axes * e_frame) ;
}


int16 FNTYPE pcdsp_read_ef(PDSP pdsp, PFRAME frame, int16 axis, int16 e_frame)
{
	int16 r ;
	if (pdsp)
	{	r = pcdsp_read_frame(pdsp,
					pcdsp_ef_address(pdsp, axis, e_frame),
					frame) ;
		if (! r)
			frame->axis = axis ;
		return r;
	}
	return (dsp_error = DSP_NOT_INITIALIZED) ;
}


int16 FNTYPE pcdsp_write_ef(PDSP pdsp, PFRAME frame)
{
	if (pdsp)
		return pcdsp_write_frame(pdsp, frame) ;

	return (dsp_error = DSP_NOT_INITIALIZED) ;
}
