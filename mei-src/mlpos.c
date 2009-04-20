/*
	mlpos.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

# include "idsp.h"
# include "sercos.h"


int16 FNTYPE get_command(int16 axis, P_DOUBLE dst)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	pcdsp_command(dspPtr, axis, &lfixed);
	return ipcdsp_double_pos(dspPtr, axis, &lfixed, dst);
}

int16 FNTYPE set_command(int16 axis, double src)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	ipcdsp_fixed_pos(dspPtr, axis, src, &lfixed);
	return pcdsp_set_command(dspPtr, axis, &lfixed);
}

int16 FNTYPE get_position(int16 axis, P_DOUBLE dst)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	pcdsp_actual(dspPtr, axis, &lfixed);
	return ipcdsp_double_pos(dspPtr, axis, &lfixed, dst) ;
}

int16 FNTYPE set_position(int16 axis, double pos)
{	LFIXED lfixed ;
	int16 u = 0;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	if(dspPtr->sercos)
	{	int16 mode, channel;
		mode = dspPtr->sercdata[axis].mode;
		channel = dspPtr->sercdata[axis].channel;
		if(channel != -1)	/* if axis is mapped to a channel */
			if((mode != SERCOS_PM_TORQUE) || (mode != SERCOS_PM_VEL))
			{	unsigned16 buffer;
				int16 len = 1;
				read_idn(channel, (int16)148, &len, &buffer, FALSE);
				if(buffer == 0)	/* not executing drive controlled homing routine */
					return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);
			}
	}

	if (!dsp_read_dm(dsp_read_dm(dspPtr->inptr)) && (! in_sequence(axis)))
		u = 1;

	ipcdsp_fixed_pos(dspPtr, axis, pos, &lfixed) ;

	if(!pcdsp_set_position(dspPtr, axis, &lfixed) )
	{	if (u || dspPtr->laxis[axis].last == LA_COMMAND)
			pcdsp_set_last(dspPtr, axis, LA_COMMAND, &lfixed) ;
	}

    return dsp_error;
}

int16 FNTYPE get_velocity(int16 axis, P_DOUBLE vel)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_velocity(dspPtr, axis, &lfixed);
	return ipcdsp_double_vel(dspPtr, axis, &lfixed, vel) ;
}


int16 FNTYPE set_velocity(int16 axis, double vel)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	ipcdsp_fixed_vel(dspPtr, axis, vel, &lfixed) ;
	return pcdsp_set_vel(dspPtr, axis, &lfixed) ;
}



int16 FNTYPE get_accel(int16 axis, P_DOUBLE accel)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_acceleration(dspPtr, axis, &lfixed) ;
	return ipcdsp_double_accel(dspPtr, axis, &lfixed, accel) ;
}

int16 FNTYPE set_accel(int16 axis, double accel)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	ipcdsp_fixed_accel(dspPtr, axis, accel, &lfixed) ;
	return pcdsp_set_accel(dspPtr, axis, &lfixed);
}


int16 FNTYPE get_jerk(int16 axis, P_DOUBLE jerk)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	pcdsp_get_jerk(dspPtr, axis, &lfixed) ;
	return ipcdsp_double_jerk(dspPtr, axis, &lfixed, jerk) ;
}

int16 FNTYPE set_jerk(int16 axis, double jerk)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	ipcdsp_fixed_jerk(dspPtr, axis, jerk, &lfixed) ;
	return pcdsp_set_jerk(dspPtr, axis, &lfixed) ;
}


int16 FNTYPE get_error(int16 axis, P_DOUBLE error)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	pcdsp_get_error(dspPtr, axis, &lfixed) ;
	return ipcdsp_double_pos(dspPtr, axis, &lfixed, error) ;
}



int16 FNTYPE get_latched_position(int16 axis, P_DOUBLE dst)
{	LFIXED lfixed ;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
	pcdsp_latched(dspPtr, axis, &lfixed);
	return ipcdsp_double_pos(dspPtr, axis, &lfixed, dst) ;
}

int16 FNTYPE arm_latch(int16 enable)
{	return pcdsp_arm_latch(dspPtr, enable) ;
}

int16 FNTYPE latch_status(void)
{	if (! pcdsp_init_check(dspPtr))
		return pcdsp_latch_status(dspPtr) ;
	return 0;
}

int16 FNTYPE latch(void)
{	if (! reset_bit(22) )
		set_bit(22) ;
	return dsp_error ;
}


