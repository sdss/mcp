/*
	llpid.c -- low-level PID.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include  "idsp.h"



/*
	digital filter manipulation functions.
*/

int16 FNTYPE pcdsp_set_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * values)
{	P_DSP_DM filter = pdsp->e_data + ED(axis) + ED_C0 ;
	return pcdsp_transfer_block(pdsp, FALSE, FALSE, filter, COEFFICIENTS, values) ;
}



int16 FNTYPE pcdsp_get_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * dest)
{	P_DSP_DM filter = pdsp->e_data + ED(axis) + ED_C0 ;
	return pcdsp_transfer_block(pdsp, TRUE, FALSE, filter, COEFFICIENTS, dest) ;
}

int16 FNTYPE pcdsp_set_aux_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * values)
{	P_DSP_DM filter = pdsp->e_data + ED(axis) + ED_I0 ;
	return pcdsp_transfer_block(pdsp, FALSE, FALSE, filter, AUX_FILTER_COEFFS, values) ;
}



int16 FNTYPE pcdsp_get_aux_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * dest)
{	P_DSP_DM filter = pdsp->e_data + ED(axis) + ED_I0 ;
	return pcdsp_transfer_block(pdsp, TRUE, FALSE, filter, AUX_FILTER_COEFFS, dest) ;
}

