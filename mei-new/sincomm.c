/*      
	Sincomm.c -
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "sincomm.h"

#define	DEGREES		(3.1415927/180.)


int16 FNTYPE set_phases(int16 axis, int16 ang, int16 lead, int16 table_size, int16 phase_seq)
{
	int16 phase_advance = table_size/PHASES;
	int16 ang_p_lead, ang_m_lead, ang_p_lead_p_pa, ang_p_lead_m_pa,
	 ang_m_lead_p_pa, ang_m_lead_m_pa;

	ang_p_lead = ang + lead;
	ang_m_lead = ang - lead;
	ang_p_lead_p_pa = ang + lead + phase_advance;
	ang_p_lead_m_pa = ang + lead - phase_advance;
	ang_m_lead_p_pa = ang - lead + phase_advance;
	ang_m_lead_m_pa = ang - lead - phase_advance;

	if((ang_p_lead) < 0)
		ang_p_lead = table_size + ang_p_lead;
	if((ang_p_lead) > table_size)
		ang_p_lead = ang_p_lead - table_size;

	if((ang_m_lead) < 0)
		ang_m_lead = table_size + ang_m_lead;
	if((ang_m_lead) > table_size)
		ang_m_lead = ang_m_lead - table_size;

	if((ang_p_lead_p_pa) < 0)
		ang_p_lead_p_pa = table_size + ang_p_lead_p_pa;
	if((ang_p_lead_p_pa) > table_size)
		ang_p_lead_p_pa = ang_p_lead_p_pa - table_size;

	if((ang_p_lead_m_pa) < 0)
		ang_p_lead_m_pa = table_size + ang_p_lead_m_pa;
	if((ang_p_lead_m_pa) > table_size)
		ang_p_lead_m_pa = ang_p_lead_m_pa - table_size;

	if((ang_m_lead_p_pa) < 0)
		ang_m_lead_p_pa = table_size + ang_m_lead_p_pa;
	if((ang_m_lead_p_pa) > table_size)
		ang_m_lead_p_pa = ang_m_lead_p_pa - table_size;

	if((ang_m_lead_m_pa) < 0)
		ang_m_lead_m_pa = table_size + ang_m_lead_m_pa;
	if((ang_m_lead_m_pa) > table_size)
		ang_m_lead_m_pa = ang_m_lead_m_pa - table_size;
 
	if(phase_seq == 1)			/* C = -(A + B) */
	{							/* phase a leads phase b */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_p_lead_m_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_m_lead_m_pa);
	}                                
	if(phase_seq == 2)			/* C = -(A + B) */
	{							/* phase b leads phase a */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_p_lead_p_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_m_lead_p_pa);
	}                          
	if(phase_seq == 3)			/* C = - A + B */
	{							/* phase a leads phase b */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_m_lead_m_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_p_lead_m_pa);
	}                          
	if(phase_seq == 4)			/* C = - A + B */
	{							/* phase b leads phase a */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_m_lead_p_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_p_lead_p_pa);
	}
	if(phase_seq == 5)			/* C =  A - B */
	{							/* phase a leads phase b */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_p_lead_m_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_m_lead_m_pa);
	}
	if(phase_seq == 6)			/* C =  A - B */
	{							/* phase b leads phase a */
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis)), ang_m_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis)), ang_p_lead);
		dsp_write_pm((unsigned16)(COMM_OFFSET_POS + CBS(axis) + 1), ang_p_lead_p_pa);
		dsp_write_pm((unsigned16)(COMM_OFFSET_NEG + CBS(axis) + 1), ang_m_lead_p_pa);
	}
	return 0;
}


int16 FNTYPE wait_and_clear(int16 axis, int16 duration)
{	
/*	P_DSP_DM act_addr, com_addr; */
	int16 t;
/*	DSP_DM buff[2]; */

/*	act_addr = dspPtr->data_struct + (DS_SIZE*axis) + DS_ACTUAL_POSITION; */
/*	com_addr = dspPtr->data_struct + (DS_SIZE*axis) + DS_POSITION + 2; */
	t = dsp_read_dm(0x11F);
	
	while(((int16)(dsp_read_dm(0x11F) - t)) < duration)
	{
/*		pcdsp_transfer_block(dspPtr, TRUE, FALSE, act_addr, 2, buff); */
/*		pcdsp_transfer_block(dspPtr, FALSE, FALSE, com_addr, 2, buff); */
		dsp_write_pm((unsigned16)(CBS(axis) + COMM_POINTER), 0); 
/*		dsp_write_dm((unsigned16)(0x14 + axis), 0); */
	}
	return 0;
}


int16 FNTYPE init_scheme_one(int16 axis, int16 phase_seq, int16 table_size, int16 volts)
{	int16
		i,
		lead = table_size/4,
		v = volts/100,
		phase_step = table_size/(3*25),
		phase_init = table_size/3,
		rate = dsp_sample_rate();

	set_dac_output(axis, 0);
	wait_and_clear(axis, 0);
	set_phases(axis, phase_init, 0, table_size, phase_seq);
	for(i = 1; i < 100; i++)
	{	set_dac_output(axis, (int16)(v * i));
		wait_and_clear(axis, 100);
	}
	wait_and_clear(axis, (int16)(10 * rate));

	for(i = 1; i < 26; i++)
	{	set_phases(axis, (int16)(phase_init - (phase_step*i)), 0, table_size, phase_seq);
		wait_and_clear(axis, 100);
	}
	set_phases(axis, 0, 0, table_size, phase_seq);
	wait_and_clear(axis, (int16)(10 * rate));

	/* This section is currently commented out until we can figure out what is
	 * going on here.  It appears to cause a bug (PH - 11/16/96)
	 */
	/* set_position(axis, 0.0); 
	while (!motion_done(axis))
		;
	*/
	set_phases(axis, 0, lead, table_size, phase_seq);

	return 0;
}


int16 FNTYPE get_actual_accel(int16 axis, int16 samples, double * accel)
{	
	int16 i;
	P_DSP_DM addr = dspPtr->data_struct + DS(axis);	/* internal data */
	DSP_DM initial[DS_TIME + 2], final[DS_TIME + 2];

	pcdsp_transfer_block(dspPtr, TRUE, FALSE, addr, DS_TIME + 2, initial);
	for(i = 1; i < samples; i++)
		pcdsp_transfer_block(dspPtr, TRUE, FALSE, addr, DS_TIME + 2, final);

	/* average acceleration over 'n' samples, units: cts/sample*sample */
	*accel = ((double)((int16)final[DS_CURRENT_VEL] - (int16)initial[DS_CURRENT_VEL])) /
		((double)(final[DS_TIME + 1] - initial[DS_TIME + 1]));
	return 0;
}


int16 FNTYPE init_scheme_two(int16 axis, int16 phase_seq, int16 table_size, int16 volts, int16 samples)
{	int16 
		i = 0,
		ang = 0,
		delta = table_size/2,
		lead = table_size/4,
		done = 0;
	double accel;

	set_phases(axis, ang, 0, table_size, phase_seq);
	set_dac_output(axis, volts);

	/* Find magnetic vector within one electrical cycle. */
	while(!done)
	{	
		set_phases(axis, ang, 0, table_size, phase_seq);
		get_actual_accel(axis, samples, &accel);

		if(((i > 0) && (accel == 0.0)) || (delta <= 20))
			done = TRUE;
		if(accel < 0)
			ang+=delta;
		if(accel > 0)
			ang-=delta;

		delta/=2;
		i++;
	}			
	
	wait_and_clear(axis, 1);  /* reset commutation pointer */
	set_position(axis, 0.0);
	set_dac_output(axis, 0);
	set_phases(axis, ang, lead, table_size, phase_seq); 
	return 0;
}


int16 FNTYPE read_hall_sensors(void)
{	int16 hall, a, b, c, region = 0;

	get_io(0, &hall);
	c = hall & 4;
	b = hall & 2;
	a = hall & 1;
	if(a)
	{
		if(b)
			region = 2;
		else
		{
			if(c)
				region = 6;
			else
				region = 1;
		}
	}
	else
		if(b)
		{
			if(c)
				region = 4;
			else
				region = 3;
		}
		else
			if(c)
				region = 5;

	return region;
}


/* get_ang returns the value of the angle at the beginning of the next region */
int16 FNTYPE get_ang(int16 phase_seq, int16 table_size, int16 region)
{	int16 ang = 0;

	if(phase_seq == 1)
		ang = ((10-region) * (table_size/6));
	if(phase_seq == 2)
		ang = ((region+2) * (table_size/6));
	if(phase_seq == 3)
	{
		if(region == 1)
			ang = 3 * (table_size/6);
		if(region == 2)
			ang = 2 * (table_size/6);
		if(region == 3)
			ang = 4 * (table_size/6);
		if(region == 4)
			ang = 6 * (table_size/6);
		if(region == 5)
			ang = 5 * (table_size/6);
		if(region == 6)
			ang = 1 * (table_size/6);
	}
	if(phase_seq == 4)
	{
		if(region == 1)
			ang = 3 * (table_size/6);
		if(region == 2)
			ang = 4 * (table_size/6);
		if(region == 3)
			ang = 2 * (table_size/6);
		if(region == 4)
			ang = 6 * (table_size/6);
		if(region == 5)
			ang = 1 * (table_size/6);
		if(region == 6)
			ang = 5 * (table_size/6);
	}
	if(phase_seq == 5)
	{
		if(region == 1)
			ang = 6 * (table_size/6);
		if(region == 2)
			ang = 2 * (table_size/6);
		if(region == 3)
			ang = 1 * (table_size/6);
		if(region == 4)
			ang = 3 * (table_size/6);
		if(region == 5)
			ang = 5 * (table_size/6);
		if(region == 6)
			ang = 4 * (table_size/6);
	}
	if(phase_seq == 6)
	{
		if(region == 1)
			ang = 6 * (table_size/6);
		if(region == 2)
			ang = 4 * (table_size/6);
		if(region == 3)
			ang = 5 * (table_size/6);
		if(region == 4)
			ang = 3 * (table_size/6);
		if(region == 5)
			ang = 1 * (table_size/6);
		if(region == 6)
			ang = 2 * (table_size/6);
	}
	ang = (int16) fmod((double)ang, (double)table_size);
	return ang;
}


int16 FNTYPE init_scheme_three(int16 axis, int16 phase_seq, int16 table_size, int16 volts)
{
	int16 region, ang, lead, i, v, hall_mid;

	init_io(0, IO_INPUT);

	lead = table_size/4;
	hall_mid = table_size/12;
	v = volts/100;

	region = read_hall_sensors();
	ang = get_ang(phase_seq, table_size, region);
	ang -= hall_mid;

	set_dac_output(axis, 0);
	wait_and_clear(axis, 0);
	set_phases(axis, ang, 0, table_size, phase_seq);
	for(i = 1; i < 100; i++)
	{	set_dac_output(axis, (int16)(v * i));
		wait_and_clear(axis, 50);
	}
	set_position(axis, 0.0);
	set_phases(axis, ang, lead, table_size, phase_seq);

	return 0;
}


int16 FNTYPE init_comm_table(int16 axis, long enc_res, long elec_cycles, int16 ncycles, P_INT cycle_table_size)
{
	unsigned16 addr;
	int16 cos_val, shift = 0;
	long i, table_size = enc_res / elec_cycles;

	while (table_size > COMM_SIN_SIZE)
	{	table_size /= 2;
		shift -= 1;
	}

	dsp_write_dm((unsigned16)(0x14 + axis), 0);
	dsp_write_pm((unsigned16)(COMM_TABLE_LEN + CBS(axis)), (int16)table_size);
	dsp_write_pm((unsigned16)(COMM_SHIFT + CBS(axis)), shift);
	
	for(i = 0; i < table_size; i++)
	{	cos_val = (int16)(32767.0 * cos((i*2*ncycles*PI)/table_size));
		addr = (unsigned16)(COMM_SIN_TABLE + CBS(axis) + i);
		dsp_write_pm(addr, cos_val);
	}
	*cycle_table_size = (int16)(table_size / ncycles);

	return dsp_error;
}


int16 FNTYPE init_comm(int16 axis, int16 phase_seq, int16 method, int16 cycle_table_size, double voltage)
{
	int16
		samples = 10,		/* number of DSP samples to determine average acceleration */	
		volts = (int16)(voltage * 3276.7);	/* convert volts to 16 bit DAC value */	

	if(method == 1)
		init_scheme_one(axis, phase_seq, cycle_table_size, volts);
	if(method == 2)
		init_scheme_two(axis, phase_seq, cycle_table_size, volts, samples);
	if(method == 3)
		init_scheme_three(axis, phase_seq, cycle_table_size, volts);

	return dsp_error;
}


int16 FNTYPE set_comm_axis(int16 axis, int16 enable)
{	DSP_DM   current;
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	r = pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	
	if(enable)
		current |= CD_SIN_COMM;
	else
		current &= ~CD_SIN_COMM;
	if(!r)	
		pcdsp_set_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	return dsp_error;
}


int16 FNTYPE get_comm_axis(int16 axis, P_INT enable)
{	DSP_DM	current;
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	r = pcdsp_get_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(!r)
		*enable = ((current & CD_SIN_COMM) ? TRUE:FALSE);
	return dsp_error;
}


int16 FNTYPE set_boot_comm_axis(int16 axis, int16 enable)
{	DSP_DM   current;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;
		
	pcdsp_get_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(enable)
		current |= CD_SIN_COMM;
	else
		current &= ~CD_SIN_COMM;
	pcdsp_set_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	return dsp_error;
}


int16 FNTYPE get_boot_comm_axis(int16 axis, P_INT enable)
{	DSP_DM current;	
	int16 r;
	if(pcdsp_init_check(dspPtr))
		return dsp_error;

	r = pcdsp_get_boot_config_struct(dspPtr, axis, CL_CONFIG_DATA, NULL, &current);
	if(!r)
		*enable = ((current & CD_SIN_COMM) ? TRUE:FALSE);
	return dsp_error;
}
