/*
	mllsr.c -
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

# include <math.h>
# include "idsp.h"
# include "ilsr.h"

int16 FNTYPE init_laser_power(int16 laser_axis, int16 x_axis, double x_mult, 
	int16 y_axis, double y_mult, double max_v, int16 saturation_voltage)
{
	int16 i, sqrt_val, axes, vx_addr, vy_addr, cp_addr, sf;
	unsigned16 sqrt_addr, ib_addr;
	double x, v_per_samp, overall_gain, calc_gain;
	int16 msb = 0L, v, p, shift, xm, ym;
	int16 filter[10] = {0,0,0,0,0,0,0,0,0,0};

	if(pcdsp_sick(dspPtr, laser_axis))
		return dsp_error;
	axes = dsp_axes();  
	sqrt_addr = 0x800					/* base of external program memory */
		+ (COMM_BLOCK_SIZE * axes)		/* commutation tables */
		+ (INTERP_BLOCK_SIZE * axes)	/* interpolation tables */
		+ ASIN_TABLE_SIZE;

	ib_addr = 0x800							/* base of external program memory */
		+ (COMM_BLOCK_SIZE * axes)			/* commutation tables */
		+ (INTERP_BLOCK_SIZE * laser_axis);	/* interpolation tables */

	for(i = 0; i < 2048; i++)
	{	x = ((double)i)/2048.0;  /* table range from .5 to 1.56 */
		sqrt_val = (int16)((sqrt(x)*0x800) + 0.5);
		dsp_write_pm((unsigned16)(sqrt_addr+i), sqrt_val);
	}

	/* get MSB from max_v and use for sf calc */
	v_per_samp = max_v/dsp_sample_rate();
	v = (int16)((long)v_per_samp & 0x0000FFFF);
	for(i = 16; (msb == 0) && (i > 0); i--)
		msb = v & (1 << i);
	if(v_per_samp > (double)msb)
		msb = msb << 1;
	if(msb == 0)
		msb = 1;
	sf = (int16)(-1.4427 * log(msb) - 1.0);

	/* calculate overall gains and adjust filter parameters */
	overall_gain = ((double)saturation_voltage/2047.0)*(((double)msb*dsp_sample_rate())/max_v);
	p = (int16)overall_gain;
	calc_gain = (double)p;
	shift = 0;
	while(fabs(calc_gain - overall_gain) > .1)
	{	if(calc_gain < overall_gain)
			p = 2*(int16)floor(p) + 1;
		else
			p = 2*(int16)ceil(p) - 1;
		shift -= 1;
		if(shift < -4)
			break;
		calc_gain = ldexp((double)p, shift);
	}
	filter[DF_P] = p;
	filter[DF_SHIFT] = shift;
	filter[DF_DAC_LIMIT] = saturation_voltage;
	set_filter(laser_axis, filter);

	vx_addr = dspPtr->data_struct + DS(x_axis) + DS_VELOCITY + 1;
	vy_addr = dspPtr->data_struct + DS(y_axis) + DS_VELOCITY + 1;
	cp_addr = dspPtr->data_struct + DS(laser_axis) + DS_POSITION + 2;
	if(x_mult > 2.0)
		x_mult = 2.0;
	if(y_mult > 2.0)
		y_mult = 2.0;
	xm = (int16)(x_mult * 32768.0);
	ym = (int16)(y_mult * 32768.0);
	
	dsp_write_pm(ib_addr, sf);						/* shift value */
	dsp_write_pm((unsigned16)(ib_addr+1), vx_addr);	/* Vx address */
	dsp_write_pm((unsigned16)(ib_addr+2), vy_addr);	/* Vy address */
	dsp_write_pm((unsigned16)(ib_addr+3), xm);
	dsp_write_pm((unsigned16)(ib_addr+4), ym);
	dsp_write_pm((unsigned16)(ib_addr+9), cp_addr);	/* Command Power address */

	return dsp_error;
}

int16 FNTYPE power_control(int16 axis, int16 on)
{	DSP_DM     current;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(on)
		current |= CD_LASER;
	else
		current &= ~CD_LASER;
	pcdsp_set_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	return DSP_OK;
}



