/*
	backward.c

	This file contains routines that are included simply for backward
	compatibility.  Although I don't forsee ever losing any of the
	functions in this file, I do recommend working toward using
	the newer (and we hope better) replacements.
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

/* sources and events used to decode version 2.0+ status values */

int16 decode_source[32] =
{	0, HOME_ID,			 POS_LIMIT_ID,	 NEG_LIMIT_ID,	 AMP_FAULT_ID,	 A_LIMIT_ID,		
    V_LIMIT_ID,		 X_NEG_LIMIT_ID,	 X_POS_LIMIT_ID,	 ERROR_LIMIT_ID,	 PC_COMMAND_ID,
    OUT_OF_FRAMES_ID,
};
int16 decode_event[16] =
{
    0,1,2,3,4,5,6,7,
    STOP,
    9,
    E_STOP,
    11,12,13,
    ABORT,
    15
};

/* used to set new event v2.0+ action values */
int16 encode_event[16] =
{
    0,1,
    STOP_EVENT,
    3,
    E_STOP_EVENT,
    5,6,7,
    ABORT_EVENT,
    9,10,11,12,13,14,15
};


int16 FNTYPE dsp_status(int16 axis)
{   
	if (! pcdsp_sick(dspPtr, axis))
    {	int16 raw_stat,evt,src;
        raw_stat = pcdsp_state(dspPtr, axis) ;
        evt = decode_event[raw_stat & PC_STATUS_EXCEPTION];
        src = decode_source[(raw_stat >> 4) & 0x1F];
        return(src | evt);
    }
    return -1;
}


#	define	MASK(n)					decode_event[(n & ACTION_MASK)]

/*	-------------------
	  Software Limits
	-------------------
*/

int16 FNTYPE set_position_limit(int16 axis, 
		double lowest, int16 lowAction,
		double highest, int16 highAction)
{	if (!set_positive_sw_limit(axis, highest, highAction))
	{	set_negative_sw_limit(axis, lowest, lowAction);
	}	
	return dsp_error ;
}
								 

int16 FNTYPE get_position_limit(int16 axis, P_DOUBLE lowest,  P_INT lowAction, P_DOUBLE highest, P_INT highAction)
{	if (!get_negative_sw_limit(axis, lowest, lowAction))
	{	get_positive_sw_limit(axis, highest, highAction);
	}
	return dsp_error ;
}


int16 FNTYPE set_pos_limit(int16 axis, double low, double high, int16 action)
{	return set_position_limit(axis, low, action, high, action) ;
}


int16 FNTYPE get_pos_limit(int16 axis, P_DOUBLE low, P_DOUBLE high, P_INT action)
{	int16 lowaction, highaction, e ;
	e = get_position_limit(axis, low, &lowaction, high, &highaction) ;
	*action = lowaction > highaction? lowaction : highaction ;
	return e ;
}



/*	------------------------------
	  Hardware sensor responses.
  	------------------------------
*/

int16 FNTYPE set_limit_actions(int16 axis, int16 level, int16 negAction, int16 posAction)
{
	set_positive_level(axis, level) ;
	set_positive_limit(axis, posAction) ;
	set_negative_level(axis, level) ;
	set_negative_limit(axis, negAction) ;

	return dsp_error ;
}


int16 FNTYPE get_limit_actions(int16 axis, P_INT level, P_INT negAction, P_INT posAction)
{
	get_positive_level(axis, level) ;
	get_positive_limit(axis, posAction) ;
	get_negative_limit(axis, negAction) ;

	return dsp_error ;
}


int16 FNTYPE set_limit_action(int16 axis, int16 activeLevel, int16 action)
{	return set_limit_actions(axis, activeLevel, action, action) ;
}


int16 FNTYPE get_limit_action(int16 axis, P_INT activeLevel, P_INT action)
{	int16 lowAction, highAction ;
	get_limit_actions(axis, activeLevel, &lowAction, &highAction) ;
	*action = MASK(lowAction) > MASK(highAction)? MASK(lowAction) : MASK(highAction) ;
	return dsp_error ;
}


int16 FNTYPE set_home_action(int16 axis, int16 level, int16 action)
{	set_home_level(axis, level) ;
	set_home(axis, action) ;
	return dsp_error ;
}


int16 FNTYPE get_home_action(int16 axis, P_INT level, P_INT action)
{
	get_home(axis, action) ;
	get_home_level(axis, level) ;
	return dsp_error ;
}


int16 FNTYPE set_amp_fault_action (int16 axis, int16 level, int16 action)
{	set_amp_fault_level(axis, level) ;
	set_amp_fault(axis, action) ;
	return dsp_error ;
}


int16 FNTYPE get_amp_fault_action (int16 axis, P_INT level, P_INT action)
{
	get_amp_fault_level(axis, level) ;
	get_amp_fault(axis, action) ;
	return dsp_error ;
}


/*
	Home-oriented functions.
*/
int16 FNTYPE start_for_home(int16 axis, double vel, double accel)
{	return v_move(axis, vel, accel) ;
}


int16 FNTYPE goto_home(int16 axis, double vel, double accel)
{	if (! v_move(axis, vel, accel))
		wait_for_done(axis) ;
	return dsp_error ;
}

int16 FNTYPE set_index_required(int16 axis, int16 indexRequired)
{
   return(set_home_index_config(axis, (int16)(indexRequired?LOW_HOME_AND_INDEX:HOME_ONLY)));
}

int16 FNTYPE get_index_required(int16 axis, P_INT indexRequired)
{
   return(get_home_index_config(axis, indexRequired));
}

int16 FNTYPE set_boot_index_required(int16 axis, int16 indexRequired)
{
   return(set_boot_home_index_config(axis, (int16)(indexRequired?LOW_HOME_AND_INDEX:HOME_ONLY)));
}

int16 FNTYPE get_boot_index_required(int16 axis, P_INT indexRequired)
{
   return(get_boot_home_index_config(axis, indexRequired));
}

int16 FNTYPE set_vel_limit(int16 axis, double limit, int16 action)
{  axis = axis;
   limit = limit;
   action = action;

   return dsp_error;
}

int16 FNTYPE get_vel_limit(int16 axis, P_DOUBLE limit, P_INT action)
{  axis = axis;
   *limit = 0.0;
   *action = 0;

   return dsp_error;
}

int16 FNTYPE set_accel_limit(int16 axis, double limit, int16 action)
{  axis = axis;
   limit = limit;
   action = action;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE get_accel_limit(int16 axis, P_DOUBLE limit, P_INT action)
{  axis = axis;
   *limit = 0.0;
   *action = 0;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE set_boot_vel_limit(int16 axis, double limit, int16 action)
{  axis = axis;
   limit = limit;
   action = action;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE get_boot_vel_limit(int16 axis, P_DOUBLE limit, P_INT action)
{  axis = axis;
   *limit = 0.0;
   *action = 0;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE set_boot_accel_limit(int16 axis, double limit, int16 action)
{  axis = axis;
   limit = limit;
   action = action;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE get_boot_accel_limit(int16 axis, P_DOUBLE limit, P_INT action)
{  axis = axis;
   *limit = 0.0;
   *action = 0;

   return DSP_FUNCTION_NOT_AVAILABLE;
}

int16 FNTYPE interrupt_on_stop(int16 axis, int16 state)
{	if (!pcdsp_config_ef(dspPtr, axis, EF_POS_STOP, state))
	{	pcdsp_config_ef(dspPtr, axis, EF_NEG_STOP, state) ;
	}
	return dsp_error ;
}


int16 FNTYPE interrupt_on_e_stop(int16 axis, int16 state)
{	if (!pcdsp_config_ef(dspPtr, axis, EF_POS_E_STOP, state))
	{	pcdsp_config_ef(dspPtr, axis, EF_NEG_E_STOP, state);
	}
	return dsp_error ;
}

/* local function for setting interrupt bit in emergency frames. */
static int16 LOCAL_FN pcdsp_config_boot_ef(PDSP dsp, int16 axis, int16 ef, int16 set_int)
{
	FRAME frame ;

	if (pcdsp_read_boot_ef(dsp, &frame, axis, ef) )
		return dsp_error ;

	if (set_int)	frame.f.control |= FCTL_INTERRUPT ;
	else			frame.f.control &= ~FCTL_INTERRUPT ;

	pcdsp_write_boot_ef(&frame);
	return dsp_error ;
}

int16 FNTYPE boot_interrupt_on_stop(int16 axis, int16 state)
{	pcdsp_config_boot_ef(dspPtr, axis, BEF_POS_STOP, state) ;

	return dsp_error ;
}

int16 FNTYPE boot_interrupt_on_e_stop(int16 axis, int16 state)
{	pcdsp_config_boot_ef(dspPtr, axis, BEF_POS_E_STOP, state) ;

	return dsp_error ;
}
