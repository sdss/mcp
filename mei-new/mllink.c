/*
	mllink.c
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

int16 FNTYPE mei_link(int16 master, int16 slave, double ratio, int16 source)
{
	P_DSP_DM    sourceAddr;
	LFIXED      lfixed;

	if (pcdsp_sick(dspPtr, master))
		return dsp_error;

	sourceAddr = dspPtr->data_struct + DS(master);

	switch (source)
	{
		case LINK_COMMAND:
			sourceAddr += DS_CV_1;
			break;

		case LINK_ACTUAL:
			sourceAddr += DS_CURRENT_VEL;
			break;

		default:
			return (dsp_error = DSP_ILLEGAL_PARAMETER);
	}
	ratio /= (dspPtr->conversion[master].countsPerDistance);
	ipcdsp_fixed_pos(dspPtr, slave, ratio, &lfixed);
	return pcdsp_set_link(dspPtr, slave, sourceAddr, &lfixed);
}

int16 FNTYPE endlink(int16 slave)
{  return pcdsp_endlink(dspPtr, slave);
}

