/*
	mluom.c - units of measure conversion.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include <stddef.h>
#include <math.h>
#include "idsp.h"

int16 FNTYPE set_conversion(int16 axis, double cpd, double spp)
{	return pcdsp_set_conversion(dspPtr, axis, cpd, spp);
}

int16 FNTYPE set_aconversion(int16 axis, double cpd, double spp, double app)
{	return pcdsp_set_aconversion(dspPtr, axis, cpd, spp, app) ;
}

int16 FNTYPE get_conversion(int16 axis, P_DOUBLE cpd, P_DOUBLE spp)
{	double app ;
	return pcdsp_get_aconversion(dspPtr, axis, cpd, spp, &app);
}

int16 FNTYPE get_aconversion(int16 axis, P_DOUBLE cpd, P_DOUBLE spp, P_DOUBLE app)
{	return pcdsp_get_aconversion(dspPtr, axis, cpd, spp, app) ;
}

static PCONVERSION LOCAL_FN pcdsp_conversion(DSP * dsp, int16 axis)
{	if(!pcdsp_init_axis_check(dsp, axis))
		return & dsp->conversion[axis] ;
	return NULL ;
}

int16 FNTYPE pcdsp_set_conversion(PDSP dsp, int16 axis, double cpd, double spp)
{
	PCONVERSION	c = pcdsp_conversion(dsp, axis);

	if (c)
	{	c->countsPerDistance = cpd ;
		c->secondsPerPeriod = spp ;
	}

	return dsp_error ;
}

int16 FNTYPE pcdsp_set_aconversion(PDSP dsp, int16 axis, double cpd, double spp, double app)
{
	PCONVERSION	c = pcdsp_conversion(dsp, axis);

	if (c)
	{	c->countsPerDistance = cpd ;
		c->secondsPerPeriod = spp ;
		c->accelPeriod = app ;
	}

	return dsp_error ;
}

int16 FNTYPE pcdsp_get_aconversion(PDSP pdsp, int16 axis, P_DOUBLE cpd, P_DOUBLE spp, P_DOUBLE app)
{
	PCONVERSION c = pcdsp_conversion(pdsp, axis) ;

	if (c)
	{	*cpd = c->countsPerDistance ;
		*spp = c->secondsPerPeriod ;
		*app = c->accelPeriod ;
	}

	return dsp_error ;
}

int16 FNTYPE pcdsp_init_conversion(PDSP dsp, int16 axis)
{	/* set the default to counts/second */
	return pcdsp_set_aconversion(dsp, axis, 1.0, 1.0, 1.0);
}

/*
	returns a double where fdsp->whole is the whole portion,
	and (fdsp->frac / FRACTIONS_PER_COUNT) is the fractional part.
*/
double FNTYPE ipcdsp_double(PFIXED fdsp)
{
	double	r;
	double	s;

	r = (double) fdsp->whole;
	s = (double) fdsp->frac;

	r += (s / FRACTIONS_PER_COUNT);

	return r;
}

/* There is a bug in the floor function in OS9000 */
 void FNTYPE ipcdsp_fixed(double src, PFIXED dst)
 {	
#ifdef MEI_OS9000
	double w;
	dst->frac = (unsigned32)(modf(src, &w) * FRACTIONS_PER_COUNT);
	if((src < 0.0) && dst->frac)
		w--;
	dst->whole = (int32)w;
#else
 	double w, f;
	src += 0.5 / FRACTIONS_PER_COUNT;
  	w = floor(src);
	f = src - w;
 	f *= FRACTIONS_PER_COUNT;
	dst->frac = (unsigned32) f;
	dst->whole	= (int32) w;
#endif
}

int16 FNTYPE ipcdsp_fixed_flat(PDSP pdsp, int16 axis, double src, PFIXED dst)
{	pdsp = pdsp ;
	axis = axis ;
	ipcdsp_fixed(src, dst) ;
	return DSP_OK ;
}

int16 FNTYPE ipcdsp_double_flat(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{	pdsp = pdsp ;
	axis = axis ;
	*dst = ipcdsp_double(src) ;
	return DSP_OK ;
}


/*	----------------------------------------
	 convert various units int16o PFIXED's...
	----------------------------------------
*/
int16 FNTYPE ipcdsp_fixed_time(PDSP pdsp, int16 axis, double src, PFIXED dst)
{	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	if (! pconv)
		return dsp_error ;

	if (! pconv->secondsPerPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION) ;

	src *= pconv->secondsPerPeriod * ((double) pdsp->sample_rate) ;
	ipcdsp_fixed(src, dst);
	return dsp_error ;
}

int16 FNTYPE ipcdsp_fixed_pos(PDSP pdsp, int16 axis, double src, PFIXED dst)
{	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis) ;

	if (! pconv)
		return dsp_error;

	if (! pconv->countsPerDistance)
		return (dsp_error = DSP_ILLEGAL_CONVERSION) ;

	src *= pconv->countsPerDistance ;
	ipcdsp_fixed(src, dst);
	return dsp_error ;
}

int16 FNTYPE ipcdsp_fixed_vel(PDSP pdsp, int16 axis, double src, PFIXED dst)
{	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s ;

	if (! pconv)
		return dsp_error ;

	if (!pconv->countsPerDistance ||
        !pconv->secondsPerPeriod)
			return (dsp_error = DSP_ILLEGAL_CONVERSION) ;

	s = pdsp->sample_rate ;
	s *= pconv->secondsPerPeriod ;
	src *= pconv->countsPerDistance / s;

	ipcdsp_fixed(src, dst);
	return dsp_error ;
}

int16 FNTYPE ipcdsp_fixed_accel(PDSP pdsp, int16 axis, double src, PFIXED dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s, a;

	if (! pconv)
		return dsp_error ;

	if (!pconv->countsPerDistance ||
		!pconv->secondsPerPeriod ||
		!pconv->accelPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION);

    /* acceleration comes in (src) in units of user_units/(Period*accelPeriod)
        (usually accel period = 1 sec).  The output needs to be
        counts/(sample*sample).  U/(P*AP) = c/(s*s) * cf where
        cf = c/U /(s/P * s/AP) */

     /* samples/Period */
	s = ((double) pdsp->sample_rate) * pconv->secondsPerPeriod ;

    /* sample/accelPeriod */
	a  = ((double) pdsp->sample_rate) *pconv->accelPeriod ;

	src *= pconv->countsPerDistance / (s * a);

	ipcdsp_fixed(src, dst);
	return dsp_error ;
}

int16 FNTYPE ipcdsp_fixed_jerk(PDSP pdsp, int16 axis, double src, PFIXED dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s, a;

	if (! pconv)
		return dsp_error ;

	if (!pconv->countsPerDistance ||
		!pconv->secondsPerPeriod ||
		!pconv->accelPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION);

     /* samples/Period */
	s = ((double) pdsp->sample_rate) * pconv->secondsPerPeriod ;

    /* sample/accelPeriod */
	a  = ((double) pdsp->sample_rate) *pconv->accelPeriod ;

	src *= pconv->countsPerDistance / (s * a * a);

	ipcdsp_fixed(src, dst);
	return dsp_error ;
}



/*	----------------------------------------
	 convert various PFIXED's int16o doubles...
	----------------------------------------
*/
int16 FNTYPE ipcdsp_double_time(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{  PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	if (! pconv)
		return dsp_error ;

	if (! pconv->secondsPerPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION) ;

	*dst = ipcdsp_double(src);
	*dst /= (pconv->secondsPerPeriod * ((double) pdsp->sample_rate)) ;
	return dsp_error ;
}


int16 FNTYPE ipcdsp_double_pos(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	if (! pconv)
		return dsp_error ;

	if (! pconv->countsPerDistance)
		return (dsp_error = DSP_ILLEGAL_CONVERSION);

	*dst = ipcdsp_double(src);
	*dst /= pconv->countsPerDistance ;
	return (dsp_error = DSP_OK);
}


int16 FNTYPE ipcdsp_double_vel(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s = (double) pdsp->sample_rate ;

	if (!pconv)
		return dsp_error ;

	*dst = ipcdsp_double(src);

	if (! pconv->countsPerDistance)
		return (dsp_error = DSP_ILLEGAL_CONVERSION) ;

	s *= pconv->secondsPerPeriod ;
	*dst = (*dst * s) / pconv->countsPerDistance ;

	return (dsp_error = DSP_OK);
}


int16 FNTYPE ipcdsp_double_accel(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s = (double) pdsp->sample_rate,
		a ;


	if (! pconv)
		return dsp_error ;

	if (!pconv->countsPerDistance ||
		!pconv->secondsPerPeriod ||
		!pconv->accelPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION);

     /* samples/Period */
	s = ((double) pdsp->sample_rate) * pconv->secondsPerPeriod ;

    /* sample/accelPeriod */
	a  = ((double) pdsp->sample_rate) * pconv->accelPeriod ;

	*dst = ipcdsp_double(src) ;
	*dst = (*dst * s * a) / pconv->countsPerDistance ;
	return dsp_error ;
}



int16 FNTYPE ipcdsp_double_jerk(PDSP pdsp, int16 axis, PFIXED src, P_DOUBLE dst)
{
	PCONVERSION
		pconv = pcdsp_conversion(pdsp, axis);

	double
		s = (double) pdsp->sample_rate,
		a ;

	if (! pconv)
		return dsp_error ;

	if (! pconv->countsPerDistance ||
		! pconv->secondsPerPeriod)
		return (dsp_error = DSP_ILLEGAL_CONVERSION);

     /* samples/Period */
	s = ((double) pdsp->sample_rate) * pconv->secondsPerPeriod ;

    /* sample/accelPeriod */
	a  = ((double) pdsp->sample_rate) *pconv->accelPeriod ;

	*dst = ipcdsp_double(src) ;
	*dst = (*dst * s * a * a) / pconv->countsPerDistance ;
	return dsp_error ;
}
