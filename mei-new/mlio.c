/*
	mlio.c - input/output drivers.
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
#	include "sercos.h"

/*	-------------------------
	  Digital input/output.
	-------------------------	*/
int16 FNTYPE get_io(int16 port, P_INT value)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	*value = (pcdsp_get_io(dspPtr, port) & 0xFF);
	return dsp_error;
}

int16 FNTYPE set_io(int16 port, int16 value)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	return pcdsp_set_io(dspPtr, port, value) ;
}

int16 FNTYPE init_io(int16 port, int16 io_select)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	return pcdsp_init_io(dspPtr, port, io_select);
}


/*	------------------
	  Analog input.
	------------------	*/

int16 FNTYPE init_analog(int16 channel, int16 diff, int16 bipolar)
{	return pcdsp_config_analog(dspPtr, channel, diff, bipolar);
}

int16 FNTYPE get_analog(int16 channel, P_INT value)
{	int16 r = start_analog(channel) ;
	if (r || pcdsp_init_check(dspPtr))
		return r;
	return read_analog(value) ;
}

int16 FNTYPE start_analog(int16 channel)
{	int16 r = pcdsp_start_analog(dspPtr, channel) ;
	dspPtr->analog_channel = channel ;
	return r;
}

#	define	ANALOG_NEGATIVE		(2048)

int16 FNTYPE read_analog(P_INT value)
{	if (pcdsp_init_check(dspPtr))
		return dsp_error ;
	if (dspPtr->analog_channel == -1)
		dsp_error = DSP_ILLEGAL_ANALOG ;
	else
	{	*value = pcdsp_read_analog(dspPtr) ;
		if (dspPtr->analog_control[dspPtr->analog_channel] & ANALOG_BIPOLAR)
		{	if (*value & ANALOG_NEGATIVE)
				(*value) |= (-1 & ~(0xFFF)) ;
		}
	}
	return dsp_error ;
}

int16 FNTYPE read_axis_analog(int16 axis, P_INT value)
{
    int16 addr;
    if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;
    addr = dspPtr->global_data
        + dspPtr->axes * AXIS_ANALOG_OFFSET + GD_SIZE + axis;
    *value = dsp_read_dm(addr);
    return dsp_error;
}




/*	--------------------------
	  Consumer input/output.
	--------------------------	*/
int16 FNTYPE bit_on(int16 bitNo)
{
	int16
		port = bitNo / 8,
		mask = 1 << (bitNo % 8) ;

	return ((pcdsp_get_io(dspPtr, port) & mask) != 0);
}

int16 FNTYPE change_bit(int16 bitNo, int16 state)
{
	int16
		port = bitNo / 8,
		mask = 1 << (bitNo % 8),
		current = pcdsp_get_io(dspPtr, port);

	if (state)
		current |= mask ;
	else
		current &= ~mask ;

	pcdsp_set_io(dspPtr, port, current);
	return dsp_error ;
}

int16 FNTYPE set_bit(int16 bitNo)
{	return change_bit(bitNo, 1);
}

int16 FNTYPE reset_bit(int16 bitNo)
{	return change_bit(bitNo, 0);
}


/*	-------------------------------
	  axis digital sensor inputs.
	-------------------------------
*/
int16 FNTYPE home_switch(int16 axis)
{	return (pcdsp_switches(dspPtr, axis) & SW_HOME) != 0 ;
}

int16 FNTYPE pos_switch(int16 axis)
{	return (pcdsp_switches(dspPtr, axis) & SW_POS_LIMIT) != 0 ;
}

int16 FNTYPE neg_switch(int16 axis)
{	return (pcdsp_switches(dspPtr, axis) & SW_NEG_LIMIT) != 0 ;
}

int16 FNTYPE amp_fault_switch(int16 axis)
{	return (pcdsp_switches(dspPtr, axis) & SW_AMP_FAULT) != 0 ;
}



int16 FNTYPE set_amp_enable(int16 axis, int16 state)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
		
	if(dspPtr->sercos)
		return enable_sercos_amplifier(axis, state);
	else
		return pcdsp_enable(dspPtr, axis, state);
}

int16 LOCAL_FN set_amp_enable_out(int16 axis, int16 amp_on)
{
	DSP_DM   current, enable;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(dspPtr->sercos)
		return enable_sercos_amplifier(axis, amp_on);

	pcdsp_get_config_struct(dspPtr, axis, CL_AMP_ENABLE_MASK, NULL, &current);

	if(amp_on)
	{
		if(current & TRIGGER_NEGATIVE)
			enable = LOW;
		else
			enable = HIGH;
	}
	else
	{
		if(current & TRIGGER_NEGATIVE)
			enable = HIGH;
		else
			enable = LOW;
	}   
	return pcdsp_enable(dspPtr, axis, enable);
}

int16 FNTYPE enable_amplifier(int16 axis)
{  return set_amp_enable_out(axis, TRUE);
}

int16 FNTYPE disable_amplifier(int16 axis)
{  return set_amp_enable_out(axis, FALSE);
}

int16 FNTYPE get_amp_enable(int16 axis, P_INT state)
{
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if ((axis < 0) || (axis >= PCDSP_MAX_AXES))
		return (dsp_error = DSP_INVALID_AXIS) ;

	if(dspPtr->sercos)
		return sercos_enabled(axis, state);
	else
		return pcdsp_enabled(dspPtr, axis, state);
}

int16 FNTYPE set_home_index_config(int16 axis, int16 config)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	return pcdsp_set_home_index_config(dspPtr, axis, config);
}

int16 FNTYPE get_home_index_config(int16 axis, P_INT config)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	*config = pcdsp_home_index_config(dspPtr, axis);
	return dsp_error ;
}

int16 FNTYPE dsp_step_speed(int16 axis, P_INT spd)
{	return pcdsp_step_speed(dspPtr, axis, spd);
}

int16 FNTYPE dsp_set_step_speed(int16 axis, int16 spd)
{	return pcdsp_set_step_speed(dspPtr, axis, spd);
}


int16 FNTYPE dsp_closed_loop(int16 axis, P_INT closed)
{	*closed = pcdsp_closed_loop(dspPtr, axis);
	return dsp_error ;
}

int16 FNTYPE dsp_set_closed_loop(int16 axis, int16 closed)
{	return pcdsp_set_closed_loop(dspPtr, axis, closed);
}


int16 FNTYPE dsp_set_stepper(int16 axis, int16 stepper)
{	return pcdsp_set_stepper(dspPtr, axis, stepper);
}



int16 FNTYPE init_timer(int16 channel, int16 mode)
{	return pcdsp_init_timer(dspPtr, channel, mode);
}

int16 FNTYPE set_timer(int16 channel, unsigned16 t)
{	return pcdsp_set_timer(dspPtr, channel, t);
}

int16 FNTYPE get_timer(int16 channel, unsigned16 PTRTYPE * t)
{
	*t = pcdsp_get_timer(dspPtr, channel) ;
	return dsp_error ;
}

int16 FNTYPE io_mon(int16 port, int16 *status)
{
    return pcdsp_io_mon(port,status);
}

int16 FNTYPE clear_io_mon(void)
{
    return pcdsp_clear_io_mon();
}

int16 FNTYPE set_io_mon_mask(int16 port, int16 mask, int16 value)
{
    return pcdsp_set_io_mon_mask(port, mask, value);
}
int16 FNTYPE get_io_mon_mask(int16 port, int16 *mask, int16 *value)
{
    return pcdsp_get_io_mon_mask(port, mask, value);
}

int16 FNTYPE io_changed(void)
{
    return pcdsp_io_changed();
}

