/*
	mldsp.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "idsp.h"
#include "sercos.h"

/*	The global dsp varaible. */
DSP DATATYPE	dsp ;
PDSP DATATYPE	dspPtr = NULL ;

int16 FNTYPE set_dspPtr(PDSP pdsp)
{	dspPtr = pdsp;
	return dsp_error;
}

int16 FNTYPE get_dspPtr(PDSP PTRTYPE * pdsp)
{  *pdsp = dspPtr;
   return dsp_error;
}

static int16 LOCAL_FN sample_rate(PDSP pdsp, DSP_DM clock, unsigned * rate)
{	*rate = (unsigned) (pdsp->timer_scale / (long) (clock + 1)) ;
	return DSP_OK ;
}

static DSP_DM LOCAL_FN timer_value(PDSP pdsp, unsigned samplesPerSecond)
{	return (unsigned) (pdsp->timer_scale / ((unsigned long) samplesPerSecond)) - 1;
}

int16 FNTYPE pcdsp_init(PDSP pdsp, unsigned16 iobase)
{
	int16 i, e, temp, axis, board_type;
	unsigned16 channel, nchannels, addr;
	
	e = pcdsp_init_board_comm(pdsp, iobase);

	if (! e)
		e = pcdsp_read_pointers(pdsp);

	if (! e)
	{	sample_rate(dspPtr, pdsp->sample_clock, &pdsp->sample_rate);

		for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
		{	pdsp->sercdata[axis].channel = SERCOS_MAX_CHANNELS;		/* default is uninitialized */
	  		pdsp->sercdata[axis].mode = SERCOS_MAX_CHANNELS;
	  		pdsp->sercdata[axis].drive_addr = SERCOS_MAX_CHANNELS;
	  		pdsp->sercdata[axis].drive_mfg = SERCOS_MAX_CHANNELS;
	  	}
		board_type = (dsp_read_dm(0x78) & 0xF);
		for(i = 0; i < 0xF; i++)
		{	dsp_write_dm(0x78, i);
			temp = dsp_read_dm(0x78) & 0xF;
			if(board_type != temp)
			{	board_type = -1;	/* older board, identification not supported */	
				break;
			}
		}
		if((board_type == SERCOS_PC) || (board_type == SERCOS_STD) || (board_type == SERCOS_104))
		{	pdsp->sercos = 1;
			if((dsp_read_pm((int16)(S410B_CNTRL + PHAS0_ADDR)) & 0x00FF) == 4)		/* phase 4, get info from board */
			{	addr = (unsigned16)(S410B_RAM + dsp_read_pm((unsigned16)(S410B_RAM+2)) - 5);
				nchannels = (unsigned16)dsp_read_pm(addr);
				for(channel = 0; channel < nchannels; channel++)
				{	addr = S410B_RAM + dsp_read_pm((unsigned16)(S410B_RAM+2+channel)) - 4;
					axis = dsp_read_pm(addr);
					pdsp->sercdata[axis].channel = channel;
					pdsp->sercdata[axis].mode = (unsigned16)dsp_read_pm((unsigned16)(addr + 1));
					pdsp->sercdata[axis].drive_addr = (unsigned16)dsp_read_pm((unsigned16)(addr + 2));
					pdsp->sercdata[axis].drive_mfg = (unsigned16)dsp_read_pm((unsigned16)(addr + 3));
				}
			}
		}
		else
			pdsp->sercos = 0;

		for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
		{	pcdsp_init_conversion(pdsp, axis);
			pcdsp_set_last(pdsp, axis, LA_UNDEFINED, NULL);
			pdsp->laxis[axis].gate = 0;
			pdsp->frame_control[axis] = FCTL_DEFAULT;
			pdsp->frame_action[axis] = NEW_FRAME;
			pdsp->flags[axis] = 0;
		}

		for (axis = 0; axis < PCDSP_ANALOG_CHANNELS; axis++)
			pdsp->analog_control[axis] = axis;

		pdsp->jog_x = -1;
		pdsp->jog_y = -1;
		pdsp->jog_z = -1;

		pdsp->FRAME_ALLOCATE = frame_allocate;
		pdsp->FRAME_DOWNLOAD = frame_download;
		pdsp->frame_buffer.length = 0;
		pdsp->frame_buffer.frames = 0;
		pdsp->frame_buffer.frames_left = 0;
		pdsp->frame_buffer.pbuffer = NULL;

		e = pcdsp_check(pdsp);
	}

	if(!dspPtr)
		dspPtr = pdsp;

	pdsp->analog_channel = -1;
	dsp_error = e;
	return e;
}

int16 FNTYPE dsp_init(int16 iobase)
{	if(!dspPtr)
		dspPtr = &dsp;
	return pcdsp_init(dspPtr, iobase) ;
}

int16 FNTYPE dsp_rset(void)
{	int16 e,i,
		timeout;
	DSP_DM
		dm = 0;

	e = pcdsp_reset(dspPtr);
	if (! e)
	{	idsp_write_dm(dspPtr, DM_SIGNATURE, dm) ;
		/* wait for boot-up to complete. */
        for(i = 0; i < 10; i++)
        {
            timeout = TIMEOUT;
		    while (!e && timeout && (idsp_read_dm(dspPtr, DM_SIGNATURE) != PCDSP_SIGNATURE))
			    timeout--;
        }
        timeout = TIMEOUT;

		while (!e && timeout && (idsp_read_dm(dspPtr, DM_SIGNATURE) != PCDSP_SIGNATURE))
			timeout--;

		if (! timeout)
			return (dsp_error = DSP_TIMEOUT_ERROR) ;

		if (! e)
#ifdef MEI_IOBASE
			e = pcdsp_init(dspPtr, dspPtr->iobase) ;
#else
			e = pcdsp_init(dspPtr, dspPtr->address) ;
#endif
	}
	return e ;
}

int16 FNTYPE dsp_reset(void)
{	if(dspPtr && (dspPtr->sercos))
		return (dsp_error = DSP_SERCOS_RESET);
	return dsp_rset();
}

int16 FNTYPE set_frame_output(int16 output_type, int16 length, char * buffer)
{  
	if(pcdsp_init_check(dspPtr))
		return dsp_error;

	if(length && (buffer == NULL))
		return (dsp_error = DSP_NO_EXTERNAL_BUFFER);

   switch(output_type)
   {  case FRAME_OUTPUT_BUFFER:
         dspPtr->FRAME_ALLOCATE = buffer_allocate;
         dspPtr->FRAME_DOWNLOAD = buffer_download;
         dspPtr->frame_buffer.length = length;
         dspPtr->frame_buffer.frames = 0;
         dspPtr->frame_buffer.frames_left = length/FRAME_STRUCT_SIZE;
         dspPtr->frame_buffer.pbuffer = (PFRAME) buffer;
         break;

      case FRAME_OUTPUT_BOARD:
      default:
         dspPtr->FRAME_ALLOCATE = frame_allocate;
         dspPtr->FRAME_DOWNLOAD = frame_download;
         dspPtr->frame_buffer.length = 0;
         dspPtr->frame_buffer.frames = 0;
         dspPtr->frame_buffer.frames_left = 0;
         dspPtr->frame_buffer.pbuffer = NULL;
         break;
   }
   return DSP_OK;
}

int16 FNTYPE get_frames_in_buffer(int16 * frames)
{  if(!pcdsp_init_check(dspPtr))
      *frames = dspPtr->frame_buffer.frames;
   return dsp_error;
}

int16 FNTYPE download_frames_from_buffer(int16 frames, char * buffer)
{  int16   i;
   PFRAME   frame = NULL;

   if(buffer == NULL)
      return (dsp_error = DSP_NO_EXTERNAL_BUFFER);

   frame = (PFRAME)buffer;

   if(pcdsp_init_check(frame->dsp))
      return dsp_error;

   if(frames > frame->dsp->frame_buffer.frames)
      return (dsp_error = DSP_ILLEGAL_PARAMETER);

   for(i = 0; i < frames; i++)
   {  if(frame_allocate(frame, frame->dsp, frame->axis))
         return dsp_error;
      if(frame_download(frame))
         return dsp_error;
      frame += 1;
   }
   return dsp_error;
}

unsigned16 FNTYPE dsp_sample_rate(void)
{
	DSP_DM clock ;
	unsigned rate = 0;

	if (! pcdsp_init_check(dspPtr))
	{	if (! pcdsp_transfer_block(dspPtr, TRUE, FALSE, PCDSP_TIMER, 1, &clock))
			sample_rate(dspPtr, clock, &rate) ;
	}

	return (unsigned16)rate;
}


int16 FNTYPE set_sample_rate(unsigned16 rate)
{
	if (! pcdsp_init_check(dspPtr))
	{	dspPtr->sample_clock = timer_value(dspPtr, rate) ;
		pcdsp_transfer_block(dspPtr, FALSE, FALSE, PCDSP_TIMER, 1,
				&dspPtr->sample_clock) ;
		dspPtr->sample_rate = rate ;
	}
	return dsp_error ;
}

int16 FNTYPE dsp_version(void)
{	if (!pcdsp_init_check(dspPtr))
      return dspPtr->version;
   return 0;
}

int16 FNTYPE dsp_option(void)
{	if(!pcdsp_init_check(dspPtr))
      return dspPtr->option;
   return 0;
}
    
int16 FNTYPE dsp_axes(void)
{	if (pcdsp_init_check(dspPtr))
		return dsp_error;
		
	return pcdsp_axes(dspPtr);		
}

int16 FNTYPE dsp_read_dm(unsigned16 addr)
{	return idsp_read_dm(dspPtr, addr) ;
}

int16 FNTYPE dsp_write_dm(unsigned16 addr, int16 dm)
{	DSP_DM m = dm ;
	return idsp_write_dm(dspPtr, addr, m) ;
}

int16 FNTYPE dsp_read_pm(unsigned16 addr)
{	return idsp_read_pm(dspPtr, addr) ;
}

int16 FNTYPE dsp_write_pm(unsigned16 addr, int16 pm)
{	DSP_DM m = pm ;
	return idsp_write_pm(dspPtr, addr, m) ;
}

int16 FNTYPE dsp_setup(unsigned16 iobase)
{
	if (! dspPtr)
		dspPtr = &dsp ;

	return pcdsp_set_address(dspPtr, iobase);
}

int16 FNTYPE dsp_stepper(int16 axis)
{
	if (! pcdsp_sick(dspPtr, axis))
		return pcdsp_stepper(dspPtr, axis) ;

	return 0;
}

int16 FNTYPE dsp_feed_rate(double override)
{
   int16 i;
   double feed;
   DSP_DM f[8] ;

	if (pcdsp_init_check(dspPtr))
		return dsp_error ;

   feed = override * 32768.0;
   if(feed > 65535.0) feed = 65535.0;
   if(feed < 0.0) feed = 0.0;
   for(i = 0; i < 8; i++)
   {
      f[i] = (DSP_DM) feed ;
   }
   
	pcdsp_transfer_block(dspPtr, FALSE, FALSE, (P_DSP_DM)(dspPtr->global_data + GD_FEED_RATE), 8, f);

   return dsp_error ;
}

int16 FNTYPE axis_feed_rate(int16 axis, double override)
{
    double feed;
    DSP_DM f ;

	if (pcdsp_init_check(dspPtr))
		return dsp_error ;

    feed = override * 32768.0;
    if(feed > 65535.0) feed = 65535.0;
    if(feed < 0.0) feed = 0.0;
    f = (DSP_DM) feed ;
   
	pcdsp_transfer_block(dspPtr, FALSE, FALSE, (P_DSP_DM)(dspPtr->global_data + GD_FEED_RATE + axis), 1, &f);

    return dsp_error ;
}

int16 FNTYPE dsp_interrupt_enable(int16 state)
{
	P_DSP_DM ip = PC_INTERRUPT_ENABLE ;
	DSP_DM curr ;
	
	if (! dspPtr)
		return dsp_error ;
	
	curr = idsp_read_dm(dspPtr, ip) ;

	if (state)
		curr |= PC_INTERRUPT_BIT ;
	else
		curr &= ~PC_INTERRUPT_BIT ;

	idsp_write_dm(dspPtr, ip, curr);
	return dsp_error ;
}


int16 FNTYPE pcdsp_set_last(PDSP pdsp, int16 axis, int16 last, PFIXED lastvalue)
{
	PLAXIS
		plaxis = &(pdsp->laxis[axis]) ;

	switch (last)
	{
		case LA_VELOCITY:
			copy_lfixed(plaxis->vel,  *lastvalue);
			break;
	
		case LA_COMMAND:
			copy_lfixed(plaxis->pos,  *lastvalue);
			break;

		case LA_UNDEFINED:
			break;

		default:
			return (dsp_error = DSP_ILLEGAL_PARAMETER) ;
	}
	plaxis->last = last ;
	return (dsp_error = DSP_OK) ;
}

/*	-1's in the filter_map cause the coefficient to be ignored. */
int16 idsp_filter_map[COEFFICIENTS] = 
	{	ED_C3, ED_C5, ED_C4, ED_C1,
		ED_C2, ED_C0, ED_C9, ED_C7,
		ED_C6, ED_C8 } ;

int16 FNTYPE get_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[COEFFICIENTS];
	int16 i,
		r ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	r = pcdsp_get_filter(dspPtr, axis, filter) ;
	for (i = 0; i < COEFFICIENTS; i++)
		coeffs[i] = (int16) (idsp_filter_map[i] >= 0? filter[idsp_filter_map[i] - ED_C0] : 0);
	return r;
}


int16 FNTYPE set_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[COEFFICIENTS];
	int16 i;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	for (i = 0; i < COEFFICIENTS; i++)
		if (idsp_filter_map[i] >= 0)
			filter[idsp_filter_map[i] - ED_C0] = (DSP_DM) coeffs[i] ;
	return pcdsp_set_filter(dspPtr, axis, filter) ;
}

/*	-1's in the filter_map cause the coefficient to be ignored. */
int16 idsp_aux_filter_map[AUX_FILTER_COEFFS] = 
	{	ED_I0, ED_I1, ED_I2, ED_I3,
		ED_I4, ED_I5, ED_I6, ED_I7,
		ED_I8, ED_I9 } ;

int16 FNTYPE get_aux_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[AUX_FILTER_COEFFS];
	int16 i,
		r ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	r = pcdsp_get_aux_filter(dspPtr, axis, filter) ;
	for (i = 0; i < AUX_FILTER_COEFFS; i++)
		coeffs[i] = (int16) (idsp_aux_filter_map[i] >= 0? filter[idsp_aux_filter_map[i] - ED_I0] : 0);
	return r;
}


int16 FNTYPE set_aux_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[AUX_FILTER_COEFFS];
	int16 i;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	for (i = 0; i < AUX_FILTER_COEFFS; i++)
		if (idsp_aux_filter_map[i] >= 0)
			filter[idsp_aux_filter_map[i] - ED_I0] = (DSP_DM) coeffs[i] ;
	return pcdsp_set_aux_filter(dspPtr, axis, filter) ;
}


int16 FNTYPE set_dac_output(int16 axis, int16 value)
{	P_DSP_DM filter ;
	DSP_DM dm_value = value ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	filter = (P_DSP_DM)(dspPtr->e_data + ED(axis) + idsp_filter_map[DF_OFFSET]) ;
	return pcdsp_transfer_block(dspPtr, FALSE, FALSE, filter, 1, &dm_value) ;
}


int16 FNTYPE get_dac_output(int16 axis, P_INT o)
{
	DSP_DM internal_offset, value ;
	P_DSP_DM filter ;

	if (pcdsp_get_config_struct(dspPtr, axis, CL_INTERNAL_OFFSET, NULL, &internal_offset))
		return dsp_error ;

	filter = (P_DSP_DM)(dspPtr->data_struct + DS(axis) + DS_D(8)) ;
	pcdsp_transfer_block(dspPtr, TRUE, FALSE, filter, 1, &value) ;
	*o =  value - internal_offset ;
	return dsp_error ;
}


int16 FNTYPE dsp_encoder(int16 axis)
{
	if (! pcdsp_sick(dspPtr, axis))
		return (int16) idsp_read_dm(dspPtr, (P_DSP_DM)(ENCODER_0 + axis)) ;

	return 0;
}

int16 LOCAL_FN gate_control(int16 length, P_INT axes, int16 reset) 
{  int16   i, andgate = 0x0, orgate = 0x0, mask = 0x0;
   for(i = 0; i < length; i++)
   {
      if(pcdsp_sick(dspPtr, axes[i]))
	      return dsp_error;

      if(dspPtr->laxis[axes[i]].gate && reset)
    	   dspPtr->laxis[axes[i]].gate--;

      if(!reset)
    	   dspPtr->laxis[axes[i]].gate++;

	   if(((!dspPtr->laxis[axes[i]].gate) && reset) || (dspPtr->laxis[axes[i]].gate && (!reset)))
		   mask |= (1 << axes[i]);
	}
   if(reset)
      andgate = mask;
   else
      orgate = mask;
   return pcdsp_set_gate(dspPtr, (int16)(~andgate), orgate);
}

int16 FNTYPE set_gates(int16 length, P_INT axes)
{  return gate_control(length, axes, 0);  
}

int16 FNTYPE reset_gates(int16 length, P_INT axes)
{  return gate_control(length, axes, 1);  
}

int16 FNTYPE set_gate(int16 axis)
{	return dsp_set_gate(dspPtr, axis);
}

int16 FNTYPE reset_gate(int16 axis)
{	return dsp_reset_gate(dspPtr, axis) ;
}

int16 FNTYPE jog_axis(int16 axis, int16 jog_channel, int16 c, int16 d, double m1, double m2, int16 enable)
{
	DSP_DM x1 = (int16) m1, x2 = (int16) m2;
	int16 *j = NULL, r = 0;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(jog_channel == JOG_X)
		j = &(dspPtr->jog_x);
	if(jog_channel == JOG_Y)
		j = &(dspPtr->jog_y);
	if(jog_channel == JOG_Z)
		j = &(dspPtr->jog_z);

	if(enable)
	{
		if(*j != -1)
			pcdsp_jog_disable(dspPtr, *j);

		*j = axis;

		r = pcdsp_jog_axis(dspPtr, axis, jog_channel, c, d, x1, x2);
		if (!r)
			pcdsp_jog_enable(dspPtr, axis);
	}
	else
	{
		pcdsp_jog_disable(dspPtr, axis);
		*j = -1;
	}
	if(dspPtr->jog_x == -1)
		pcdsp_jog_axis(dspPtr, -1, JOG_X, 0, 0, 0, 0);
	if(dspPtr->jog_y == -1)
		pcdsp_jog_axis(dspPtr, -1, JOG_Y, 0, 0, 0, 0);
	if(dspPtr->jog_z == -1)
		pcdsp_jog_axis(dspPtr, -1, JOG_Z, 0, 0, 0, 0);
	return r;
}

int16 FNTYPE reset_integrator(int16 axis)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	return pcdsp_set_integrator(dspPtr, axis, 0);
}

unsigned long FNTYPE get_frame_time(int16 axis)
{  DSP_DM      time[2];
   pcdsp_transfer_block(dspPtr, TRUE, FALSE, 
               (P_DSP_DM)(dspPtr->data_struct + DS(axis) + DS_TIME + 1), 
               2, (DSP_DM PTRTYPE *)time);
   return ((unsigned long)time[1] << 16) + (unsigned long)time[0];
}

unsigned16 FNTYPE get_dsp_time(void)
{	return (unsigned16)dsp_read_dm(0x11F);
}

unsigned16 FNTYPE get_last_transfer_time(void)
{	return (unsigned16)dsp_read_dm(0x11E);
}

int16 FNTYPE frames_to_execute(int16 axis)
{
    int16 fc_in,fc_out;
    int16 addr_in,addr_out;

    addr_in = dspPtr->global_data + dspPtr->axes * FC_OFFSET + GD_SIZE + axis;
    addr_out = addr_in + dspPtr->axes;
    fc_in = dsp_read_dm(addr_in);
    fc_out = dsp_read_dm(addr_out);
    return(fc_in - fc_out);
}

int16 FNTYPE inc_frame_count(int16 axis)
{
    int16 fc_in;
    int16 addr_in;

    addr_in = dspPtr->global_data + dspPtr->axes * FC_OFFSET + GD_SIZE + axis;
    fc_in = dsp_read_dm(addr_in);
    fc_in++;
    dsp_write_dm(addr_in,fc_in);
	return DSP_OK ;
}

int16 FNTYPE fifo_space(void)
{
    int16 i;
    int16 fcount = 0;

    for(i = 0; i < dspPtr->axes; i++)
    {
        fcount += frames_to_execute(i);
    }
    return(FIFO_SIZE - fcount);
}
