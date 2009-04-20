/*
	mldspf.c - DSP frames.
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


int16 FNTYPE dsp_end_sequence(int16 axis)
{  return frame_m(NULL, "0l n d", axis,  0);
}

int16 FNTYPE dsp_dwell(int16 axis, double duration)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   if(duration < sample_time)
      duration = sample_time;
   return frame_m(NULL, "0l t u d", axis, duration, FTRG_TIME) ;
}

int16 FNTYPE dsp_jerk(int16 axis, double jerk, double duration)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   if(duration < sample_time)
      duration = sample_time;
   return frame_m(NULL, "0l jt ud", axis, jerk, duration, FUPD_JERK | FTRG_TIME) ;
}

int16 FNTYPE dsp_accel(int16 axis, double accel, double duration)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   if(duration < sample_time)
      duration = sample_time;
   return frame_m(NULL, "0l at ud", axis, accel, duration, FUPD_ACCEL | FTRG_TIME) ;
}

int16 FNTYPE dsp_velocity(int16 axis, double vel, double duration)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   if(duration < sample_time)
      duration = sample_time;
   return frame_m(NULL, "0l vt ud", axis, vel, duration, FUPD_VELOCITY | FTRG_TIME) ;
}

int16 FNTYPE dsp_position(int16 axis, double position, double duration)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   if(duration < sample_time)
      duration = sample_time;
   return frame_m(NULL, "0l xt ud", axis, position, duration, FUPD_POSITION | FTRG_TIME) ;
}

int16 FNTYPE dsp_marker(int16 axis, P_INT p)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   return frame_m(NULL, "0l *t ud", axis, p, sample_time, FTRG_TIME);
}

unsigned16 FNTYPE dsp_goto(int16 axis, int16 p)
{  DSP_DM   addr;
   double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
   frame_m(NULL, "0l *et ud", axis, &addr, p, sample_time, FTRG_TIME);
   return addr;
}

int16 FNTYPE dsp_control(int16 axis, int16 bit, int16 set)
{
	if (! pcdsp_init_axis_check(dspPtr, axis))
	{
		if (set)
			dspPtr->frame_control[axis] |= bit ;
		else
			dspPtr->frame_control[axis] &= ~(bit) ;
	}
	return dsp_error ;
}

int16 FNTYPE dsp_frame_action(int16 axis, int16 action)
{
	if (! pcdsp_init_axis_check(dspPtr, axis))
		dspPtr->frame_action[axis] = action ;

	return dsp_error ;
}

int16 FNTYPE _dsp_io_frame(int16 axis, int16 address, int16 ormask, int16 andmask)
{  double   sample_time;
   if((!pcdsp_sick(dspPtr, axis)) && (dspPtr->sample_rate!=0))
      sample_time = 1.0/(double)(dspPtr->sample_rate);
   else
      return dsp_error;
	return frame_m(NULL, "0l Ot u n d", axis, OUTPUT_JERK, address,
			ormask, andmask, sample_time, FUPD_OUTPUT | FTRG_TIME, 0) ;
}

int16 FNTYPE dsp_io_frame(int16 axis, int16 port, int16 ormask, int16 andmask)
{
	int16 address ;
	if (! dsp_port_address(port, &address))
		_dsp_io_frame(axis, address, ormask, andmask) ;
	return dsp_error ;	
}

int16 FNTYPE dsp_action(int16 axis, int16 trigger, int16 action)
{
	P_DSP_DM       address;
	DSP_DM         new_action;
	CONFIGTYPES    ct;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	address = pcdsp_address(dspPtr, axis, trigger, &ct) + 1;
	new_action = dsp_read_dm(address) & 0xFFF0 ;
	new_action |= (action & 0x000F) ;
	return _dsp_io_frame(axis, address, new_action, new_action) ;
}

int16 FNTYPE dsp_home_action(int16 axis, int16 action)
{
   return dsp_action(axis, CL_HOME, action);
}

int16 FNTYPE dsp_positive_limit_action(int16 axis, int16 action)
{
   return dsp_action(axis, CL_POS_OVERTRAVEL, action);
}

int16 FNTYPE dsp_negative_limit_action(int16 axis, int16 action)
{
   return dsp_action(axis, CL_NEG_OVERTRAVEL, action);
}

int16 FNTYPE dsp_error_action(int16 axis, int16 action)
{
   return dsp_action(axis, CL_ERROR, action);
}

int16 FNTYPE _dsp_io_trigger(int16 axis, int16 addr, int16 mask, int16 state)
/*	port is 0..2, bit is 0..7, state is TRUE (active high) or
	FALSE (active low).
*/
{	return frame_m(NULL,
			"0l O un d",
			axis, OUTPUT_JERK, addr, mask, 0,
			FTRG_INPUT_J,
			(dspPtr->frame_action[axis]) | (state? TRIGGER_POSITIVE : TRIGGER_NEGATIVE)
				) ;
}

int16 FNTYPE dsp_io_trigger(int16 axis, int16 bit, int16 state)
{	int16 addr ;
	int16 port = bit / 8;
	bit %= 8;
	if (! dsp_port_address(port, &addr))
		_dsp_io_trigger(axis, addr, (int16)(1 << bit), state) ;
	return dsp_error ;
}

int16 FNTYPE dsp_io_trigger_mask(int16 axis, int16 port, int16 mask, int16 state)
{	int16 addr ;
	if (! dsp_port_address(port, &addr))
		_dsp_io_trigger(axis, addr, mask, state) ;
	return dsp_error ;
}

int16 FNTYPE dsp_axis_command(int16 axis, int16 destaxis, int16 event)
{	event |= (ID_AXIS_COMMAND << 4);
	if (pcdsp_sick(dspPtr, axis) || pcdsp_sick(dspPtr, destaxis))
		return dsp_error ;
	return _dsp_io_frame(axis, (int16)(dspPtr->pc_event + destaxis), event, event) ;
}

#	define	MF_CONTROL			1
#	define	MF_LENGTH			2
#	define	MF_DESTINATION		3
#	define	MF_SOURCE_1			4
#	define	MF_SOURCE_2			5
#	define	MF_DATA				6
#	define	MF_MAX				14

static int16 dsp_move_control[] = { FCTL_MOVE, FCTL_ADD } ;

int16 FNTYPE dsp_move_frame(PFRAME frame, int16 axis, int16 opcode,
		P_DSP_DM destination,
		P_DSP_DM source1,
		P_DSP_DM source2,
		int16 words, PDSP_DM buffer)
{
	PDSP_DM dm = (PDSP_DM) &(frame->f) ;
	int16 i, r ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	if (   (words > MF_MAX)
		|| (opcode < 0)
		|| (opcode >= (sizeof(dsp_move_control) / sizeof(dsp_move_control[0]))))
			return (dsp_error = DSP_ILLEGAL_PARAMETER) ;

	frame_clear(frame) ;
	if (dspPtr->FRAME_ALLOCATE(frame, dspPtr, axis))
		return dsp_error ;

	dm[MF_CONTROL] |= dsp_move_control[opcode] ;
	dm[MF_LENGTH] = words;
	dm[MF_DESTINATION] =
			destination == MF_DATA_AREA? frame->current + MF_DATA : destination;
	dm[MF_SOURCE_1] =
			source1 == MF_DATA_AREA? frame->current + MF_DATA : source1 ;
	dm[MF_SOURCE_2] =
			source2 == MF_DATA_AREA? frame->current + MF_DATA : source2 ;

	for (i = 0; i < words; i++)
		dm[MF_DATA + i] = buffer[i] ;

	i = dspPtr->flags[axis] ;
	dspPtr->flags[axis] |= DF_FRAME_COMPRESS ;
	r = dspPtr->FRAME_DOWNLOAD(frame) ;
	dspPtr->flags[axis] = i ;
	return r;
}

int16 FNTYPE dsp_position_trigger(int16 axis, int16 triggeraxis, 
                   double triggerpos, int16 trigger_sense, int16 actual, int16 action)
{
   FRAME       f;
   DSP_DM      t_block[5], dest;
   long        p;

   t_block[0] = (DS(triggeraxis) + (actual ? DS_ACTUAL_POSITION : 
            (DS_POSITION + 2))) - (DS(axis) + DS_ACTUAL_POSITION + 1);
   t_block[1] = 2;
   t_block[2] = action | ((trigger_sense == NEGATIVE_SENSE) ? TRIGGER_NEGATIVE 
                                                        : TRIGGER_POSITIVE);
   p = (long) triggerpos;
   t_block[3] = (DSP_DM)(p & 0xFFFFL);
   t_block[4] = (DSP_DM)(p >> 16);
   dest = dspPtr->data_struct + DS(axis) + DS_TRIGGER;
   return dsp_move_frame(&f, axis, MF_MOVE, (P_DSP_DM) dest, MF_DATA_AREA, 0, 5, t_block);
}

int16 FNTYPE dsp_actual_position_trigger(int16 axis, double pos, int16 trigger_sense)
{
   return dsp_position_trigger(axis, axis, pos, trigger_sense, TRUE, NEW_FRAME);
}

int16 FNTYPE dsp_command_position_trigger(int16 axis, double pos, int16 trigger_sense)
{
   return dsp_position_trigger(axis, axis, pos, trigger_sense, FALSE, NEW_FRAME);
}

int16 FNTYPE dsp_set_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[COEFFICIENTS];
	int16 i;
	P_DSP_DM filter_address ;
	FRAME frame ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	filter_address = dspPtr->e_data + ED(axis) + ED_C0 ;
 	for (i = 0; i < COEFFICIENTS; i++)
		filter[idsp_filter_map[i] - ED_C0] = (DSP_DM) coeffs[i] ;

	return dsp_move_frame(&frame, axis, MF_MOVE, filter_address,
		MF_DATA_AREA, MF_DATA_AREA, i, filter) ;
}
