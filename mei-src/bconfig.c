/*
	bconfig.c - boot memory configuration stuff.
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


static P_DSP_DM LOCAL_FN boot_filter(PDSP pdsp, int16 axis)
{	return idsp_read_dm(pdsp, PB_CONFIG_STRUCT) + ED(axis) + ED_C0;
}


int16 FNTYPE pcdsp_get_boot_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * coeffs)
{	return pcdsp_read_bm(pdsp, boot_filter(pdsp, axis), COEFFICIENTS, coeffs) ;
}

int16 FNTYPE pcdsp_set_boot_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * coeffs)
{	return pcdsp_write_bm(pdsp, boot_filter(pdsp, axis), COEFFICIENTS, coeffs) ;
}


int16 FNTYPE get_boot_filter(int16 axis, P_INT coeffs)
{	DSP_DM filter[COEFFICIENTS];
	int16 i,
		r ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	r = pcdsp_get_boot_filter(dspPtr, axis, filter) ;
	for (i = 0; i < COEFFICIENTS; i++)
		coeffs[i] = (int16) (idsp_filter_map[i] >= 0? filter[idsp_filter_map[i] - ED_C0] : 0);
		
	return r;
}


int16 FNTYPE set_boot_filter(int16 axis, P_INT coeffs)
{
	DSP_DM filter[COEFFICIENTS];
	int16 i;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	pcdsp_get_boot_filter(dspPtr, axis, filter) ;
	for (i = 0; i < COEFFICIENTS; i++)
		if (idsp_filter_map[i] >= 0)
			filter[idsp_filter_map[i] - ED_C0] = (DSP_DM) coeffs[i] ;
	return pcdsp_set_boot_filter(dspPtr, axis, filter) ;
}

/* AUX_FILTER filter stuff */

static P_DSP_DM LOCAL_FN boot_aux_filter(PDSP pdsp, int16 axis)
{	return idsp_read_dm(pdsp, PB_CONFIG_STRUCT) + ED(axis) + ED_I0;
}


int16 FNTYPE pcdsp_get_boot_aux_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * coeffs)
{	return pcdsp_read_bm(pdsp, boot_aux_filter(pdsp, axis), AUX_FILTER_COEFFS, coeffs) ;
}

int16 FNTYPE pcdsp_set_boot_aux_filter(PDSP pdsp, int16 axis, DSP_DM PTRTYPE * coeffs)
{	return pcdsp_write_bm(pdsp, boot_aux_filter(pdsp, axis), AUX_FILTER_COEFFS, coeffs) ;
}


int16 FNTYPE get_boot_aux_filter(int16 axis, P_INT coeffs)
{	DSP_DM aux_filter[AUX_FILTER_COEFFS];
	int16 i,
		r ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	r = pcdsp_get_boot_aux_filter(dspPtr, axis, aux_filter) ;
	for (i = 0; i < AUX_FILTER_COEFFS; i++)
		coeffs[i] = (int16) (idsp_aux_filter_map[i] >= 0? aux_filter[idsp_aux_filter_map[i] - ED_I0] : 0);
		
	return r;
}


int16 FNTYPE set_boot_aux_filter(int16 axis, P_INT coeffs)
{
	DSP_DM aux_filter[AUX_FILTER_COEFFS];
	int16 i;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	pcdsp_get_boot_aux_filter(dspPtr, axis, aux_filter) ;
	for (i = 0; i < AUX_FILTER_COEFFS; i++)
		if (idsp_aux_filter_map[i] >= 0)
			aux_filter[idsp_aux_filter_map[i] - ED_I0] = (DSP_DM) coeffs[i] ;
	return pcdsp_set_boot_aux_filter(dspPtr, axis, aux_filter) ;
}

unsigned FNTYPE mei_checksum(void)
{
	unsigned16 csum = pcdsp_checksum(dspPtr);
	pcdsp_write_checksum(dspPtr, csum, 1);

	return (unsigned) csum ;
}

unsigned FNTYPE static_checksum(void)
{
	unsigned16 csum = pcdsp_checksum(dspPtr);
	pcdsp_write_checksum(dspPtr, csum, 0);

	return (unsigned) csum ;
}


static P_DSP_DM boot_port[] = {
	9, 10, 11,
	5, 6, 7,
	1, 2, 3,
	13, 14, 15,
	17, 18, 19
		} ;

#	define	BOOT_PORTS		(sizeof(boot_port) / sizeof(P_DSP_DM))

static P_DSP_DM boot_port_config[] = {
	8, 8, 8,
	4, 4, 4,
	0, 0, 0,
	12, 12, 12,
	16, 16, 16
    };

static DSP_DM boot_config_data[] =
		{ 0x10, 0x02, 0x09 };

/*	-----------------
	 User I/O stuff.
	-----------------
*/

static P_DSP_DM LOCAL_FN port_boot(PDSP pdsp, int16 port)
{	return idsp_read_dm(pdsp, PB_INIT_8255) + boot_port[port] ;
}
static P_DSP_DM LOCAL_FN config_boot(PDSP pdsp, int16 port)
{	return idsp_read_dm(pdsp, PB_INIT_8255) + boot_port_config[port] ;
} 

int16 FNTYPE pcdsp_get_boot_io(PDSP pdsp, int16 port)
{
	DSP_DM r;

	if ((port < 0) || (port >= BOOT_PORTS))
	{	dsp_error = DSP_ILLEGAL_IO;
		return -1 ;
	}

	pcdsp_read_bm(pdsp, port_boot(pdsp, port), 1, &r ) ;
	return (int16) r ;
}


int16 FNTYPE pcdsp_set_boot_io(PDSP pdsp, int16 port, int16 value)
{
	DSP_DM curr = value ;

	if ((port >= 0) && (port < BOOT_PORTS))
		pcdsp_write_bm(pdsp, port_boot(pdsp, port), 1, &curr) ;

	return dsp_error ;
}


int16 FNTYPE pcdsp_init_boot_io(PDSP pdsp, int16 port, int16 config)
{
	DSP_DM config_data;

	if ((port < 0) || (port >= BOOT_PORTS))
		return (dsp_error = DSP_ILLEGAL_IO);

	pcdsp_read_bm(pdsp, config_boot(pdsp, port), 1, &config_data) ;
    
    if(config) /* output? */
        config_data = (config_data & ~boot_config_data[port % 3]) | 0x80;
    else
        config_data = (config_data | boot_config_data[port % 3]) | 0x80;

	pcdsp_write_bm(pdsp, config_boot(pdsp, port), 1, &config_data);
	return dsp_error ;
}


int16 FNTYPE set_boot_io(int16 port, int16 value)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	return pcdsp_set_boot_io(dspPtr, port, value) ;
}

int16 FNTYPE get_boot_io(int16 port, P_INT value)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	*value = pcdsp_get_boot_io(dspPtr, port) ;
	return dsp_error ;
}

int16 FNTYPE init_boot_io(int16 port, int16 direction)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	return pcdsp_init_boot_io(dspPtr, port, direction) ;
}


int16 FNTYPE change_boot_bit(int16 bitNo, int16 state)
{	int16 port, bit, value;

	if(pcdsp_init_check(dspPtr))
		return dsp_error;

	port = bitNo / 8;
	bit = bitNo % 8;
	value = pcdsp_get_boot_io(dspPtr, port) & ~(1 << bit);

	if(state)
		value |= (1 << bit);

	return pcdsp_set_boot_io(dspPtr, port, value);
}


int16 FNTYPE boot_bit_on(int16 bitNo)
{	int16 port, bit, value;

	if(pcdsp_init_check(dspPtr))
		return dsp_error;

	port = bitNo / 8;
	bit = bitNo % 8;
	value = pcdsp_get_boot_io(dspPtr, port);

	return value & (1 << bit)? TRUE : FALSE;
}


int16 FNTYPE set_boot_home_index_config(int16 axis, int16 config)
{
	int16 shift, lowbit, port, value;
	
	if (! pcdsp_sick(dspPtr, axis))
	{ 
		shift = axis/4;
		if(shift)
			shift++;
		lowbit = 14 * 8 + shift;
  		port = lowbit / 8,
  		value = pcdsp_get_boot_io(dspPtr, port);

		value |= (3 << shift);
		config = config << shift;
		value &= ~config;

		pcdsp_set_boot_io(dspPtr, port, value) ;
	}
	return dsp_error ;
}

int16 FNTYPE get_boot_home_index_config(int16 axis, P_INT config)
{
	int16 shift, lowbit;

	if (! pcdsp_sick(dspPtr, axis))
	{ 
		shift = axis/4;
		if(shift)		
			shift++;
		lowbit = 14 * 8 + shift;
		*config = !boot_bit_on(lowbit);
		*config |= (!boot_bit_on((int16)(lowbit+1)) << 1);
	}
	return dsp_error ;
}

int16 FNTYPE dsp_set_boot_step_speed(int16 axis, int16 spd)
{	int16 bit = 9 * 8 + axis * 2 ;
	spd &= 3;
	change_boot_bit(bit, (int16)(spd & 1));
	change_boot_bit((int16)(bit + 1), (int16)(spd & 2));
	return dsp_error ;
}


int16 FNTYPE dsp_boot_step_speed(int16 axis, P_INT spd)
{	int16 bit = 9 * 8 + axis * 2 ;
	*spd = boot_bit_on(bit)? 1 : 0;
	*spd |= boot_bit_on((int16)(bit + 1))? 2 : 0 ;
	return dsp_error ;
}


int16 FNTYPE dsp_set_boot_closed_loop(int16 axis, int16 closed)
{	int16 bit = 11 * 8 + axis / 2 ;
	change_boot_bit(bit, closed) ;
	return dsp_error ;
}


int16 FNTYPE dsp_boot_closed_loop(int16 axis, P_INT closed)
{	int16 bit ;

	if (! pcdsp_sick(dspPtr, axis))
	{	bit = 11 * 8 + axis / 2;
		*closed = boot_bit_on(bit) ;
	}
	return dsp_error ;
}

/*
	boot e-frames.
*/

static P_DSP_DM LOCAL_FN pcdsp_boot_ef_address(PDSP pdsp, int16 axis, int16 e_frame)
{	return idsp_read_dm(pdsp, PB_EVENT_FRAMES) +
			FRAME_SIZE * (axis + pdsp->axes * e_frame) ;
}


int16 FNTYPE pcdsp_read_boot_ef(PDSP pdsp, PFRAME frame, int16 axis, int16 e_frame)
{
	if (frame)
	{
		frame->dsp = pdsp ;

		if (frame->dsp)
		{
			frame->axis = axis ;
			frame->current = e_frame ;
			return pcdsp_read_bm(pdsp,
				pcdsp_boot_ef_address(pdsp, axis, e_frame),
				FRAME_SIZE,
				(DSP_DM PTRTYPE *) &(frame->f)) ;
		}
	}
	return (dsp_error = DSP_NOT_INITIALIZED) ;
}


int16 FNTYPE pcdsp_write_boot_ef(PFRAME frame)
{
	if (frame && frame->dsp)
		return pcdsp_write_bm(frame->dsp,
			pcdsp_boot_ef_address(frame->dsp, frame->axis, frame->current),
			FRAME_SIZE,
			(DSP_DM PTRTYPE *) &(frame->f)) ;

	return (dsp_error = DSP_NOT_INITIALIZED) ;
}



static void LOCAL_FN set_boot_rate(PDSP pdsp, int16 axis, int16 pos, double rate)
{	FRAME pos_frame;
	LFIXED lpos;

	if (rate < 0)
		rate = -rate ;

	pcdsp_read_boot_ef(pdsp, &pos_frame, axis, pos) ;
	ipcdsp_fixed_accel(pdsp, axis, -rate, &lpos);
	copy_lfixed_to_fixd(pos_frame.f.accel, lpos) ;
	pcdsp_write_boot_ef(&pos_frame) ;
}



int16 FNTYPE set_boot_stop_rate(int16 axis, double rate)
{
	if (rate < 0)
		rate = -rate ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	set_boot_rate(dspPtr, axis, BEF_POS_STOP, rate);
	return DSP_OK ;
}


int16 FNTYPE set_boot_e_stop_rate(int16 axis, double rate)
{	if (rate < 0)
		rate = -rate ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	set_boot_rate(dspPtr, axis, BEF_POS_E_STOP, rate) ;
	return DSP_OK ;
}



static double LOCAL_FN get_boot_rate(PDSP pdsp, int16 axis, int16 pos)
{	FRAME pos_frame;
	double pos_rate;
	LFIXED lpos;

	pcdsp_read_boot_ef(pdsp, &pos_frame, axis, pos) ;
	copy_lfixed(lpos, pos_frame.f.accel) ;
	ipcdsp_double_accel(pdsp, axis, &lpos, &pos_rate) ;
	return pos_rate ;
}


int16 FNTYPE get_boot_stop_rate(int16 axis, double PTRTYPE * rate)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	*rate = get_boot_rate(dspPtr, axis, BEF_POS_STOP) ;
	return DSP_OK ;
}


int16 FNTYPE get_boot_e_stop_rate(int16 axis, double PTRTYPE * rate)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	*rate = get_boot_rate(dspPtr, axis, BEF_POS_E_STOP);
	return DSP_OK ;
}




static int16 LOCAL_FN set_boot_config(PDSP pdsp, int16 axis, DSP_DM And, DSP_DM Or)
{	DSP_DM current;
	int16 r;
	if (pcdsp_sick(pdsp, axis))
		return dsp_error;
	r = pcdsp_get_boot_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &current);
	if (! r)
	{	current &= And;
		current |= Or;
		r = pcdsp_set_boot_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &current) ;
	}
	return r;
}

static int16 LOCAL_FN get_boot_config(PDSP pdsp, int16 axis, DSP_DM mask, DSP_DM * i)
{	DSP_DM dm;
	int16 r;
    if (pcdsp_sick(pdsp, axis))
		return dsp_error;
	r = pcdsp_get_boot_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &dm) ;
	if (! r)
		*i = (dm & mask) ;
	return r;
}


int16 FNTYPE dsp_set_boot_stepper(int16 axis, int16 stepper)
{	return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_STEPPER), (DSP_DM)(stepper ? CD_STEPPER : 0));
}

int16 FNTYPE dsp_boot_stepper(int16 axis)
{
	DSP_DM dm ;

	get_boot_config(dspPtr, axis, CD_STEPPER, &dm);
	return (dm != 0);
}

int16 FNTYPE set_boot_unipolar(int16 axis, int16 state)
{	return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_UNIPOLAR), (DSP_DM)(state ? CD_UNIPOLAR : 0));
}

int16 FNTYPE is_boot_unipolar(int16 axis)
{
	DSP_DM dm ;

	get_boot_config(dspPtr, axis, CD_UNIPOLAR, &dm);
	return (dm != 0);
}

int16 FNTYPE set_boot_dual_loop(int16 axis, int16 velocity_axis, int16 dual)
{  set_boot_aux_encoder(axis, velocity_axis, CAM_ACTUAL);	/* actual velocity feedback */
   return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_DUAL), (DSP_DM)(dual? CD_DUAL : 0));
}

int16 FNTYPE get_boot_dual_loop(int16 axis,P_INT velocity_axis, P_INT dual)
{	DSP_DM dm ;
	int16   r = get_boot_aux_encoder(axis, velocity_axis, CAM_ACTUAL);	/* actual velocity feedback */
	if(!r)
	r = get_boot_config(dspPtr, axis, CD_DUAL, &dm);
	if (! r)
		*dual = (dm != 0);
	return r;
}

int16 FNTYPE set_boot_feedback(int16 axis, int16 t)
{	return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_FEEDBACK_MASK), t);
}

int16 FNTYPE get_boot_feedback(int16 axis, P_INT t)
{	DSP_DM dm ;
	int16 r = get_boot_config(dspPtr, axis, CD_FEEDBACK_MASK, &dm) ;
	if (! r)
		*t = (int16) dm ;
	return r;
}

int16 FNTYPE set_boot_analog_channel(int16 axis, int16 chan, int16 differential, int16 bipolar)
{
    DSP_DM cfg;
    cfg = (chan & 7) | (differential ? ANALOG_DIFF : 0) | (bipolar ? ANALOG_BIPOLAR : 0);
    return(pcdsp_set_boot_config_struct(dspPtr, axis, CL_ANALOG_CHANNEL, NULL, &cfg)) ;
}

int16 FNTYPE get_boot_analog_channel(int16 axis, P_INT chan, P_INT differential, P_INT bipolar)
{
   DSP_DM cfg;
   int16    e;

   e = pcdsp_get_boot_config_struct(dspPtr, axis, CL_ANALOG_CHANNEL, NULL, &cfg);
   if(e)
      return e;

   *chan = (cfg & 7);
   *differential = ((cfg & ANALOG_DIFF) ? 1 : 0);
   *bipolar = ((cfg & ANALOG_BIPOLAR) ? 1 : 0);

   return 0;
}

int16 FNTYPE set_boot_dac_channel(int16 axis, int16 chan)
{
    DSP_DM cfg;
    cfg = (chan & 7);
    return(pcdsp_set_boot_config_struct(dspPtr, axis, CL_DAC_CHANNEL, NULL, &cfg)) ;
}

int16 FNTYPE get_boot_dac_channel(int16 axis, P_INT chan)
{
   DSP_DM cfg;
   int16    e;

   e = pcdsp_get_boot_config_struct(dspPtr, axis, CL_DAC_CHANNEL, NULL, &cfg);
   if(!e)
    *chan = cfg ;

   return e;
}

int16 FNTYPE set_boot_aux_encoder(int16 axis, int16 encoder_channel, int16 source)
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
	return(pcdsp_set_boot_config_struct(dspPtr, axis, CL_AUX_ENCODER, NULL, &encoder_addr)) ;
}

int16 FNTYPE get_boot_aux_encoder(int16 axis, int16 * encoder_channel, int16 source)
{	DSP_DM  encoder_addr;
	int16 r = pcdsp_get_boot_config_struct(dspPtr, axis, CL_AUX_ENCODER, NULL, &encoder_addr);

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

int16 FNTYPE set_boot_cam(int16 master_axis, int16 cam_axis, int16 cam, int16 source)
{	set_boot_aux_encoder(cam_axis, master_axis, source);
	return set_boot_config(dspPtr, cam_axis, (DSP_DM)(~CD_CAM), (DSP_DM)(cam? CD_CAM : 0));
}

int16 FNTYPE set_boot_integration(int16 axis, int16 im)
{	DSP_DM dm = 0;
	if (im == IM_ALWAYS)
		dm = CD_INTEGRATION ;
	return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_INTEGRATION), dm);
}

int16 FNTYPE get_boot_integration(int16 axis, P_INT im)
{	DSP_DM dm ;
	int16 r = get_boot_config(dspPtr, axis, CD_INTEGRATION, &dm);
	if (! r)
		*im = (dm != 0);
	return r;
}

int16 FNTYPE set_boot_axis_analog(int16 axis, int16 state)
{  return set_boot_config(dspPtr, axis, (DSP_DM)(~CD_USER_ANALOG), (DSP_DM)(state ? CD_USER_ANALOG : 0)); 
}

int16 FNTYPE get_boot_axis_analog(int16 axis, P_INT state)
{  DSP_DM      i;
   int16         r;
   r = get_boot_config(dspPtr, axis, CD_USER_ANALOG, &i);
   if(!r)
	   *state = (int16)(i >> 9);
   return r;
}

int16 FNTYPE set_boot_axis(int16 axis, int16 enable)
{	DSP_DM   current;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	pcdsp_get_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(enable)
		current &= ~CD_KILL_AXIS;
	else
		current |= CD_KILL_AXIS;
	pcdsp_set_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	return dsp_error;
}

int16 FNTYPE get_boot_axis(int16 axis, P_INT enable)
{	DSP_DM current;	
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;

	r = pcdsp_get_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(!r)
		*enable = ((current & CD_KILL_AXIS) ? FALSE:TRUE);
	return dsp_error;
}
