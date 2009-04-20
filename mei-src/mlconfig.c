/*
	mlconfig.c
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

int16 FNTYPE set_positive_sw_limit(int16 axis, double position, int16 action)
{	LFIXED		lfixed ;
	DSP_DM		flag = POS_ID(action, ID_X_POS_LIMIT) ;
	ipcdsp_fixed_pos(dspPtr, axis, position, &lfixed) ;
	return pcdsp_set_config_struct(dspPtr, axis, CL_X_POS, &lfixed, &flag) ;
}

int16 FNTYPE get_positive_sw_limit(int16 axis, P_DOUBLE position, P_INT action)
{	LFIXED	lfixed ;
	DSP_DM	flag ;
	if (! pcdsp_get_config_struct(dspPtr, axis, CL_X_POS, &lfixed, &flag))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, position) ;
		*action = MASK(flag) ;
	}
	return dsp_error ;
}

int16 FNTYPE set_negative_sw_limit(int16 axis, double position, int16 action)
{	LFIXED		lfixed ;
	DSP_DM		flag = NEG_ID(action, ID_X_NEG_LIMIT) ;
	ipcdsp_fixed_pos(dspPtr, axis, position, &lfixed) ;
	return pcdsp_set_config_struct(dspPtr, axis, CL_X_NEG, &lfixed, &flag) ;
}

int16 FNTYPE get_negative_sw_limit(int16 axis, P_DOUBLE position, P_INT action)
{	LFIXED lfixed ;
	DSP_DM	flag ;
	if (! pcdsp_get_config_struct(dspPtr, axis, CL_X_NEG, &lfixed, &flag))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, position) ;
		*action = MASK(flag) ;
	}
	return dsp_error ;
}


int16 FNTYPE set_in_position(int16 axis, double window)
{	LFIXED	lfixed ;

	if(!ipcdsp_fixed_pos(dspPtr, axis, window, &lfixed))
	{	pcdsp_set_config_struct(dspPtr, axis, CL_IN_POSITION, &lfixed, NULL) ;
	}
	return dsp_error ;
}


int16 FNTYPE get_in_position(int16 axis, P_DOUBLE window)
{	LFIXED	lfixed ;

	if (!pcdsp_get_config_struct(dspPtr, axis, CL_IN_POSITION, &lfixed, NULL))
	{	ipcdsp_double_pos(dspPtr, axis, &lfixed, window) ;
	}
	return dsp_error ;
}


int16 FNTYPE set_error_limit(int16 axis, double window, int16 action)
{	LFIXED	lfixed;
	DSP_DM	flag = POS_ID(action, ID_ERROR_LIMIT);

	if (!ipcdsp_fixed_pos(dspPtr, axis, window, &lfixed))
	{	pcdsp_set_config_struct(dspPtr, axis, CL_ERROR, &lfixed, &flag);
	}
	return dsp_error ;
}


int16 FNTYPE get_error_limit(int16 axis, P_DOUBLE window, P_INT action)
{	LFIXED	lfixed ;
	DSP_DM	flag ;

	if(!  ( pcdsp_get_config_struct(dspPtr, axis, CL_ERROR, &lfixed, &flag) ||
			ipcdsp_double_pos(dspPtr, axis, &lfixed, window)))
		*action = MASK(flag) ;

	return dsp_error ;
}



/*	------------------------------
	  Hardware sensor responses.
  	------------------------------
*/

static int16 LOCAL_FN set_action(PDSP pdsp, int16 axis, int16 limit, int16 action)
{	DSP_DM current;
	int16 r = pcdsp_get_config_struct(pdsp, axis, limit, NULL, &current);
	if (! r)
	{	SET_ACTION(current, action) ;
		r = pcdsp_set_config_struct(pdsp, axis, limit, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_action(PDSP pdsp, int16 axis, int16 limit, int16 * action)
{	DSP_DM current ;
	int16 r = pcdsp_get_config_struct(pdsp, axis, limit, NULL, &current) ;
	*action = MASK(current);
	return r ;
}

static int16 LOCAL_FN set_level(PDSP pdsp, int16 axis, int16 limit, int16 level)
{	DSP_DM current;
	int16 r = pcdsp_get_config_struct(pdsp, axis, limit, NULL, &current);
	if (! r)
	{	if (level)
			current &= ~TRIGGER_NEGATIVE;
		else
			current |= TRIGGER_NEGATIVE ;
		r = pcdsp_set_config_struct(pdsp, axis, limit, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_level(PDSP pdsp, int16 axis, int16 limit, int16 * level)
{	DSP_DM current ;
	int16 r = pcdsp_get_config_struct(pdsp, axis, limit, NULL, &current) ;
	if (current & TRIGGER_NEGATIVE)
		*level = FALSE ;
	else
		*level = TRUE ;
	return r;
}


int16 FNTYPE set_positive_limit(int16 axis, int16 action)
{	return set_action(dspPtr, axis, CL_POS_OVERTRAVEL, action) ;
}

int16 FNTYPE set_positive_level(int16 axis, int16 level)		/* TRUE or FALSE */
{	return set_level(dspPtr, axis, CL_POS_OVERTRAVEL, level) ;
}


int16 FNTYPE set_negative_limit(int16 axis, int16 action)
{	return set_action(dspPtr, axis, CL_NEG_OVERTRAVEL, action) ;
}

int16 FNTYPE set_negative_level(int16 axis, int16 level)
{	return set_level(dspPtr, axis, CL_NEG_OVERTRAVEL, level) ;
}

int16 FNTYPE get_positive_limit(int16 axis, P_INT action)
{	return get_action(dspPtr, axis, CL_POS_OVERTRAVEL, action) ;
}

int16 FNTYPE get_positive_level(int16 axis, P_INT level)		/* TRUE or FALSE */
{	return get_level(dspPtr, axis, CL_POS_OVERTRAVEL, level) ;
}

int16 FNTYPE get_negative_limit(int16 axis, P_INT action)
{	return get_action(dspPtr, axis, CL_NEG_OVERTRAVEL, action) ;
}

int16 FNTYPE get_negative_level(int16 axis, P_INT level)
{	return get_level(dspPtr, axis, CL_NEG_OVERTRAVEL, level) ;
}


int16 FNTYPE set_home(int16 axis, int16 action)
{	return set_action(dspPtr, axis, CL_HOME, action) ;
}

int16 FNTYPE set_home_level(int16 axis, int16 level)
{	return set_level(dspPtr, axis, CL_HOME, level) ;
}


int16 FNTYPE get_home(int16 axis, P_INT action)
{	return get_action(dspPtr, axis, CL_HOME, action) ;
}

int16 FNTYPE get_home_level(int16 axis, P_INT level)
{	return get_level(dspPtr, axis, CL_HOME, level) ;
}


int16 FNTYPE set_amp_fault (int16 axis, int16 action)
{	return set_action(dspPtr, axis, CL_DEVICE_FAULT, action) ;
}

int16 FNTYPE set_amp_fault_level(int16 axis, int16 level)
{	return set_level(dspPtr, axis, CL_DEVICE_FAULT, level) ;
}


int16 FNTYPE get_amp_fault (int16 axis, P_INT action)
{	return get_action(dspPtr, axis, CL_DEVICE_FAULT, action) ;
}

int16 FNTYPE get_amp_fault_level (int16 axis, P_INT level)
{	return get_level(dspPtr, axis, CL_DEVICE_FAULT, level) ;
}

int16 FNTYPE set_amp_enable_level(int16 axis, int16 level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);
	else
		return set_level(dspPtr, axis, CL_AMP_ENABLE_MASK, level) ;
}

int16 FNTYPE get_amp_enable_level(int16 axis, P_INT level)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	if(dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);
	else
		return get_level(dspPtr, axis, CL_AMP_ENABLE_MASK, level) ;
}

static int16 LOCAL_FN set_config(PDSP pdsp, int16 axis, DSP_DM And, DSP_DM Or)
{	DSP_DM current;
	int16 r = pcdsp_get_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &current);
	if (! r)
	{	current &= And ;
		current |= Or ;
		r = pcdsp_set_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_config(PDSP pdsp, int16 axis, DSP_DM mask, DSP_DM * i)
{	DSP_DM dm ;
	int16 r = pcdsp_get_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &dm) ;
	if (! r)
		*i = (dm & mask) ;
	return r;
}

int16 FNTYPE set_dual_loop(int16 axis, int16 velocity_axis, int16 dual)
{	set_aux_encoder(axis, velocity_axis, CAM_ACTUAL);		/* actual velocity feedback */
	return set_config(dspPtr, axis, (DSP_DM)(~CD_DUAL), (DSP_DM)(dual? CD_DUAL : 0));
}

int16 FNTYPE get_dual_loop(int16 axis, P_INT velocity_axis, P_INT dual)
{	DSP_DM dm ;
	int16   r = get_aux_encoder(axis, velocity_axis, CAM_ACTUAL);
	if(!r)
		r = get_config(dspPtr, axis, CD_DUAL, &dm);
	if(!r)
		*dual = (dm != 0);
	return r;
}

int16 FNTYPE set_feedback(int16 axis, int16 t)
{	return set_config(dspPtr, axis, (DSP_DM)(~CD_FEEDBACK_MASK), t);
}

int16 FNTYPE get_feedback(int16 axis, P_INT t)
{	DSP_DM dm ;
	int16 r = get_config(dspPtr, axis, CD_FEEDBACK_MASK, &dm) ;
	if (! r)
		*t = (int16) dm ;
	return r;
}

int16 FNTYPE set_analog_channel(int16 axis, int16 chan, int16 differential, int16 bipolar)
{
    DSP_DM cfg;
    cfg = (chan & 7) | (differential ? ANALOG_DIFF : 0) | (bipolar ? ANALOG_BIPOLAR : 0);
    return(pcdsp_set_config_struct(dspPtr, axis, CL_ANALOG_CHANNEL, NULL, &cfg)) ;
}

int16 FNTYPE get_analog_channel(int16 axis, P_INT chan, P_INT differential, P_INT bipolar)
{
   DSP_DM cfg;
   int16    e;

   e = pcdsp_get_config_struct(dspPtr, axis, CL_ANALOG_CHANNEL, NULL, &cfg);
   if(e)
      return e;

   *chan = (cfg & 7);
   *differential = ((cfg & ANALOG_DIFF) ? 1 : 0);
   *bipolar = ((cfg & ANALOG_BIPOLAR) ? 1 : 0);

   return e;
}
int16 FNTYPE set_dac_channel(int16 axis, int16 chan)
{
    DSP_DM cfg;
    cfg = (chan & 7);
    return(pcdsp_set_config_struct(dspPtr, axis, CL_DAC_CHANNEL, NULL, &cfg)) ;
}

int16 FNTYPE get_dac_channel(int16 axis, P_INT chan)
{
   DSP_DM cfg;
   int16    e;

   e = pcdsp_get_config_struct(dspPtr, axis, CL_DAC_CHANNEL, NULL, &cfg);
   if(!e)
    *chan = cfg ;

   return e;
}

int16 FNTYPE set_cam(int16 master_axis, int16 cam_axis, int16 cam, int16 source)
{	set_aux_encoder(cam_axis, master_axis, source);
	return set_config(dspPtr, cam_axis, (DSP_DM)(~CD_CAM), (DSP_DM)(cam? CD_CAM : 0));
}

int16 FNTYPE set_aux_encoder(int16 axis, int16 encoder_channel, int16 source)
{	DSP_DM encoder_addr;
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error;
	if((encoder_channel < 0) || (encoder_channel > PCDSP_MAX_AXES))
		return (dsp_error = DSP_ILLEGAL_ENCODER_CHANNEL);

	encoder_addr = dspPtr->data_struct + DS(encoder_channel);
	switch (source)
	{
		case CAM_COMMAND:
			encoder_addr += DS_CV_1;
			break;

		case CAM_ACTUAL:
			encoder_addr += DS_CURRENT_VEL;
			break;

		default:
			return (dsp_error = DSP_ILLEGAL_PARAMETER);
	}
	return(pcdsp_set_config_struct(dspPtr, axis, CL_AUX_ENCODER, NULL, &encoder_addr)) ;
}

int16 FNTYPE get_aux_encoder(int16 axis, int16 * encoder_channel, int16 source)
{	DSP_DM  encoder_addr;
	int16 r = pcdsp_get_config_struct(dspPtr, axis, CL_AUX_ENCODER, NULL, &encoder_addr);

	switch (source)
	{
		case CAM_COMMAND:
			encoder_addr -= DS_CV_1;
			break;

		case CAM_ACTUAL:
			encoder_addr -= DS_CURRENT_VEL;
			break;

		default:
			return (dsp_error = DSP_ILLEGAL_PARAMETER);
	}
    if (!r)
	{	*encoder_channel = encoder_addr - dspPtr->data_struct;
		*encoder_channel /= DS_SIZE;
	}
	return r;
}

int16 FNTYPE set_integration(int16 axis, int16 im)
{	DSP_DM dm = 0;
	if (im == IM_ALWAYS)
		dm = CD_INTEGRATION ;
	return set_config(dspPtr, axis, (DSP_DM)(~CD_INTEGRATION), dm);
}

int16 FNTYPE get_integration(int16 axis, P_INT im)
{	DSP_DM dm ;
	int16 r = get_config(dspPtr, axis, CD_INTEGRATION, &dm);
	if (! r)
		*im = (dm != 0);
	return r;
}

int16 FNTYPE pcdsp_set_unipolar(DSP * dsp, int16 axis, int16 state)
{	return set_config(dsp, axis, (DSP_DM)(~CD_UNIPOLAR), (DSP_DM)(state ? CD_UNIPOLAR : 0));
}

int16 FNTYPE set_unipolar(int16 axis, int16 state)
{   return pcdsp_set_unipolar(dspPtr,  axis, state);
}

int16 FNTYPE is_unipolar(int16 axis)
{
	DSP_DM dm ;

	get_config(dspPtr, axis, CD_UNIPOLAR, &dm);
	return (dm != 0);
}

int16	FNTYPE pcdsp_stepper(PDSP dsp, int16 axis)
{
	DSP_DM dm ;

	get_config(dsp, axis, CD_STEPPER, &dm);
	return (dm != 0);
}

/*	enables or disables step pulse output.  remember that it makes
	no sense to disable step output and be in open-loop mode.  This
	also sets us for unipolar mode.
*/
int16 FNTYPE pcdsp_set_stepper(PDSP dsp, int16 axis, int16 stepper)
{
	int16 r  = set_config(dsp, axis, (DSP_DM)(~CD_STEPPER), (DSP_DM)(stepper ? CD_STEPPER : 0));
	if(!r)
		r = pcdsp_set_unipolar(dsp, axis, stepper);
	return r;
}

int16 FNTYPE set_axis_analog(int16 axis, int16 state)
{  return set_config(dspPtr, axis, (DSP_DM)(~CD_USER_ANALOG), (DSP_DM)(state ? CD_USER_ANALOG : 0)); 
}

int16 FNTYPE get_axis_analog(int16 axis, P_INT state)
{  DSP_DM      i;
   int16         r;
   r = get_config(dspPtr, axis, CD_USER_ANALOG, &i);
   if (!r)
	   *state = (int16)(i >> 9);
   return r;
}

int16 FNTYPE set_axis(int16 axis, int16 enable)
{	DSP_DM   current;
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	r = pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	
	if(enable)
		current &= ~CD_KILL_AXIS;
	else
		current |= CD_KILL_AXIS;
	if(!r)	
		pcdsp_set_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	return dsp_error;
}

int16 FNTYPE get_axis(int16 axis, P_INT enable)
{	DSP_DM	current;
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	r = pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(!r)
		*enable = ((current & CD_KILL_AXIS) ? FALSE:TRUE);
	return dsp_error;
}

int16 FNTYPE set_feedback_check(int16 axis, int16 state)
{ 	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
		
	return set_config(dspPtr, axis, (DSP_DM)(~CD_FEEDBACK_CHECK),
		(DSP_DM)(state ? CD_FEEDBACK_CHECK : 0));
}

int16 FNTYPE get_feedback_check(int16 axis, P_INT state)
{	DSP_DM dm;
	int16 r;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	
	r = get_config(dspPtr, axis, CD_FEEDBACK_CHECK, &dm);
	if (! r)
		*state = (int16) dm;
	return dsp_error;	
}
