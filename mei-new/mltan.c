/*
	mltan.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include <math.h>
#	include "itan.h"

int16 FNTYPE init_tangent(int16 slave_axis, int16 x_axis, int16 y_axis, double max_v, unsigned16 counts_per_rev)
{	int16 i, axes, sf,
		ib_addr, vx_addr, vy_addr, vt_addr,
		asin_addr, norm_addr, asin_tbl, normalize;

	if(pcdsp_sick(dspPtr, slave_axis))
		return dsp_error;
	else
		axes = dspPtr->axes;
  
	asin_addr = 0x800					/* base of external program memory */
		+ (COMM_BLOCK_SIZE * axes)		/* commutation tables */
		+ (INTERP_BLOCK_SIZE * axes);	/* interpolation tables */

	norm_addr = asin_addr + ASIN_TABLE_SIZE;

	ib_addr = 0x800							/* base of external program memory */
		+ (COMM_BLOCK_SIZE * axes)			/* commutation tables */
		+ (INTERP_BLOCK_SIZE * slave_axis);	/* interpolation tables */

	for(i = 0; i < 2048; i++)
	{	double   x;
		x = (i+TAN_OFFSET_VALUE)/2048.0;
		normalize = (int16)(((1.0/sqrt(x))*0x800) + 0.5);
		x = i/2048.0;
		asin_tbl = (int16)(floor(asin(x) * 2048.0/3.14159265) + 0.5);
		dsp_write_pm((unsigned16)(norm_addr+i), normalize);
		dsp_write_pm((unsigned16)(asin_addr+i), asin_tbl);
	}
	sf = (int16)(-1.4427 * log(max_v/dsp_sample_rate()) - 2.0);

	vx_addr = dspPtr->data_struct + DS(x_axis) + DS_VELOCITY + 1;
	vy_addr = dspPtr->data_struct + DS(y_axis) + DS_VELOCITY + 1;
	vt_addr = dspPtr->data_struct + DS(slave_axis) + DS_VELOCITY + 1;
	dsp_write_pm((unsigned16)(ib_addr+TAN_SHIFT), sf);
	dsp_write_pm((unsigned16)(ib_addr+TAN_X), vx_addr);
	dsp_write_pm((unsigned16)(ib_addr+TAN_Y), vy_addr);
	dsp_write_pm((unsigned16)(ib_addr+TAN_OFFSET), TAN_OFFSET_VALUE);
	dsp_write_pm((unsigned16)(ib_addr+TAN_THETA), vt_addr);
	dsp_write_pm((unsigned16)(ib_addr+TAN_CPR), counts_per_rev);
	
	return dsp_error;
}

int16 FNTYPE init_angle(int16 axis, double init_angle)
{	int16 ib_addr, axes;
	unsigned16 angle;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	
	axes = dspPtr->axes;

	ib_addr = 0x800						/* base of external program memory */
		+ (COMM_BLOCK_SIZE * axes)		/* commutation tables */
		+ (INTERP_BLOCK_SIZE * axis);	/* interpolation tables */
	angle = (unsigned16)(init_angle * (65536.0/360.0));
	
	return dsp_write_pm((unsigned16)(ib_addr + TAN_INIT_ANGLE), angle);
}

int16 FNTYPE set_tangent(int16 axis, int16 on)
{	DSP_DM current;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(on)
		current |= CD_TANGENT;
	else
		current &= ~CD_TANGENT;
	return pcdsp_set_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
}

int16 FNTYPE set_boot_tangent(int16 axis, int16 on)
{	DSP_DM current;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(on)
		current |= CD_TANGENT;
	else
		current &= ~CD_TANGENT;
	return pcdsp_set_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
}
