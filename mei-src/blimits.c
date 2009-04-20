/*
	blimits.c
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


#	define	MASK(n)					(n & ACTION_MASK)
#	define	POS_ID(n, m)			(MASK(n) | (m << 4))
#	define	NEG_ID(n, m)			(MASK(n) | (m << 4) | TRIGGER_NEGATIVE)
#	define	ID(action, level, id)	level? POS_ID(action, id) : NEG_ID(action, id)
#	define	SET_ACTION(n,m)			(n = (((n) & ~ACTION_MASK) | (m)))


/*	-------------------
	  Software Limits
	-------------------
*/

int16 FNTYPE set_boot_positive_sw_limit(int16 axis, double position, int16 action)
{	LFIXED		lfixed ;
	DSP_DM		flag = POS_ID(action, ID_X_POS_LIMIT) ;
	ipcdsp_fixed_pos(dspPtr, axis, position, &lfixed) ;
	return pcdsp_set_boot_config_struct(dspPtr, axis, CL_X_POS, &lfixed, &flag) ;
}

int16 FNTYPE get_boot_positive_sw_limit(int16 axis, P_DOUBLE position, P_INT action)
{	LFIXED lfixed ;
	DSP_DM flag ;
	if (! pcdsp_get_boot_config_struct(dspPtr, axis, CL_X_POS, &lfixed, &flag))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, position) ;
		*action = MASK(flag) ;
	}
	return dsp_error ;
}

int16 FNTYPE set_boot_negative_sw_limit(int16 axis, double position, int16 action)
{	LFIXED		lfixed ;
	DSP_DM		flag = NEG_ID(action, ID_X_NEG_LIMIT) ;
	ipcdsp_fixed_pos(dspPtr, axis, position, &lfixed) ;
	return pcdsp_set_boot_config_struct(dspPtr, axis, CL_X_NEG, &lfixed, &flag) ;
}

int16 FNTYPE get_boot_negative_sw_limit(int16 axis, P_DOUBLE position, P_INT action)
{	LFIXED lfixed ;
	DSP_DM flag ;
	if (! pcdsp_get_boot_config_struct(dspPtr, axis, CL_X_NEG, &lfixed, &flag))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, position) ;
		*action = MASK(flag) ;
	}
	return dsp_error ;
}


int16 FNTYPE set_boot_in_position(int16 axis, double window)
{	LFIXED	lfixed ;

	if (!ipcdsp_fixed_pos(dspPtr, axis, window, &lfixed))
	{	pcdsp_set_boot_config_struct(dspPtr, axis, CL_IN_POSITION, &lfixed, NULL);
	}
	return dsp_error ;
}


int16 FNTYPE get_boot_in_position(int16 axis, P_DOUBLE window)
{	LFIXED	lfixed ;

	if (!pcdsp_get_boot_config_struct(dspPtr, axis, CL_IN_POSITION, &lfixed, NULL))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, window) ;
	}
	return dsp_error ;
}


int16 FNTYPE set_boot_error_limit(int16 axis, double window, int16 action)
{	LFIXED	lfixed ;
	DSP_DM	flag = POS_ID(action, ID_ERROR_LIMIT) ;

	if (!ipcdsp_fixed_pos(dspPtr, axis, window, &lfixed))
	{	pcdsp_set_boot_config_struct(dspPtr, axis, CL_ERROR, &lfixed, &flag);
	}
	return dsp_error ;
}


int16 FNTYPE get_boot_error_limit(int16 axis, P_DOUBLE window, P_INT action)
{	LFIXED	lfixed ;
	DSP_DM	flag ;

	if(!  ( pcdsp_get_boot_config_struct(dspPtr, axis, CL_ERROR, &lfixed, &flag) ||
			ipcdsp_double_pos(dspPtr, axis, &lfixed, window)))
		*action = MASK(flag) ;

	return dsp_error ;
}



/*	------------------------------
	  Hardware sensor responses.
  	------------------------------
  	Each function requires 2 samples.
*/

static int16 LOCAL_FN set_boot_action(PDSP pdsp, int16 axis, int16 limit, int16 action)
{	DSP_DM current ;
	int16 r = pcdsp_get_boot_config_struct(pdsp, axis, limit, NULL, &current);
	if (! r)
	{	SET_ACTION(current, action) ;
		r = pcdsp_set_boot_config_struct(pdsp, axis, limit, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_boot_action(PDSP pdsp, int16 axis, int16 limit, int16 * action)
{	DSP_DM flag ;
	int16 r = pcdsp_get_boot_config_struct(pdsp, axis, limit, NULL, &flag) ;
	*action = MASK(flag);
	return r ;
}

static int16 LOCAL_FN set_boot_level(PDSP pdsp, int16 axis, int16 limit, int16 level)
{	DSP_DM current;
	int16 r = pcdsp_get_boot_config_struct(pdsp, axis, limit, NULL, &current);
	if (! r)
	{	if (level)
			current &= ~TRIGGER_NEGATIVE;
		else
			current |= TRIGGER_NEGATIVE ;
		r = pcdsp_set_boot_config_struct(pdsp, axis, limit, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_boot_level(PDSP pdsp, int16 axis, int16 limit, int16 * level)
{	DSP_DM flag ;
	int16 r = pcdsp_get_boot_config_struct(pdsp, axis, limit, NULL, &flag) ;
	if (flag & TRIGGER_NEGATIVE)
		*level = FALSE ;
	else
		*level = TRUE ;
	return r;
}


int16 FNTYPE set_boot_positive_limit(int16 axis, int16 action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_action(dspPtr, axis, CL_POS_OVERTRAVEL, action) ;
}

int16 FNTYPE set_boot_positive_level(int16 axis, int16 level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_level(dspPtr, axis, CL_POS_OVERTRAVEL, level) ;
}

int16 FNTYPE set_boot_negative_limit(int16 axis, int16 action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_action(dspPtr, axis, CL_NEG_OVERTRAVEL, action) ;
}

int16 FNTYPE set_boot_negative_level(int16 axis, int16 level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_level(dspPtr, axis, CL_NEG_OVERTRAVEL, level) ;
}

int16 FNTYPE get_boot_positive_limit(int16 axis, P_INT action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_action(dspPtr, axis, CL_POS_OVERTRAVEL, action) ;
}

int16 FNTYPE get_boot_positive_level(int16 axis, P_INT level)		/* TRUE or FALSE */
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_level(dspPtr, axis, CL_POS_OVERTRAVEL, level) ;
}

int16 FNTYPE get_boot_negative_limit(int16 axis, P_INT action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_action(dspPtr, axis, CL_NEG_OVERTRAVEL, action) ;
}

int16 FNTYPE get_boot_negative_level(int16 axis, P_INT level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_level(dspPtr, axis, CL_NEG_OVERTRAVEL, level) ;
}


int16 FNTYPE set_boot_home(int16 axis, int16 action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_action(dspPtr, axis, CL_HOME, action) ;
}

int16 FNTYPE set_boot_home_level(int16 axis, int16 level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_level(dspPtr, axis, CL_HOME, level) ;
}


int16 FNTYPE get_boot_home(int16 axis, P_INT action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_action(dspPtr, axis, CL_HOME, action) ;
}

int16 FNTYPE get_boot_home_level(int16 axis, P_INT level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_level(dspPtr, axis, CL_HOME, level) ;
}


int16 FNTYPE set_boot_amp_fault (int16 axis, int16 action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_action(dspPtr, axis, CL_DEVICE_FAULT, action) ;
}

int16 FNTYPE set_boot_amp_fault_level(int16 axis, int16 level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_boot_level(dspPtr, axis, CL_DEVICE_FAULT, level) ;
}


int16 FNTYPE get_boot_amp_fault (int16 axis, P_INT action)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_action(dspPtr, axis, CL_DEVICE_FAULT, action) ;
}

int16 FNTYPE get_boot_amp_fault_level (int16 axis, P_INT level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_boot_level(dspPtr, axis, CL_DEVICE_FAULT, level) ;
}


int16 FNTYPE set_boot_amp_enable_level(int16 axis, int16 level)
{	int16 r;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);

	r = set_boot_level(dspPtr, axis, CL_AMP_ENABLE_MASK, level) ;
	if (! r)
		r = set_boot_amp_enable(axis, (int16)(!level)) ;		/* default to amplifier disable. */
	return r;
}

int16 FNTYPE get_boot_amp_enable_level(int16 axis, P_INT level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);

	return get_boot_level(dspPtr, axis, CL_AMP_ENABLE_MASK, level) ;
}

int16 FNTYPE set_boot_amp_enable(int16 axis, int16 level)
{	int16 bit;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);

	bit = (axis / 4)? 5 : 8 ;
	bit *= 8;
	bit += ((axis % 4) * 2) ;
	return change_boot_bit(bit, level) ;
}

int16 FNTYPE get_boot_amp_enable(int16 axis, P_INT level)
{	int16 bit;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);

	bit = (axis / 4)? 5 : 8 ;
	bit *= 8;
	bit += ((axis % 4) * 2) ;
	*level = boot_bit_on(bit) ;
	return dsp_error ;
}


