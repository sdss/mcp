/*
	MLSERC.C - Sercos specific routines
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "sercos.h"

int16 FNTYPE get_sercos_channel(int16 axis, unsigned16 * channel)
{
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	if(!dspPtr->sercos)
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);
	*channel = dspPtr->sercdata[axis].channel;
	if(*channel >= SERCOS_MAX_CHANNELS)		/* valid channel? */
		dsp_error = DSP_SERCOS_AXIS_ASSIGNMENT;
	return dsp_error;
}

int16 FNTYPE set_idn(int16 axis, unsigned16 idn, long val)
{	unsigned16 channel;
	int16 len = -1;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return write_idn(channel, idn, &len, (unsigned16*)(&val), FALSE);
}

int16 FNTYPE get_idn(int16 axis, unsigned16 idn, long *val)
{	unsigned16 channel;
	int16 len = -1;
	
	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	*val = 0;		/* set all bits for val to zero if IDN is only 2 bytes */
	return read_idn(channel, idn, &len, (unsigned16*)val, FALSE);
}

int16 FNTYPE get_idn_attributes(int16 axis, unsigned16 idn, IDN_ATTRIBUTES *attr)
{	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return read_idn_attributes(channel, idn, attr);
}

int16 FNTYPE set_idns(int16 axis, unsigned16 dr_addr, unsigned16 nidns, IDNS *idns)
{	unsigned16 i, channel;
	int16 nwords;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	channel = dspPtr->sercdata[axis].channel;
	if(channel >= SERCOS_MAX_CHANNELS)		/* valid channel? */
	{	set_drive(dr_addr);
		channel = 0;
	}
	for (i = 0; i < nidns; i++)
	{	nwords = -1;		/* let write_idn(...) determine the data size */
		if (!idns[i].error)	/* don't try to write idns that have errors */
		{	idns[i].error = write_idn(channel, idns[i].idn, &nwords, (unsigned16*)(&(idns[i].value)), FALSE);
			if (idns[i].error)
			{	if ((idns[i].error != DSP_SERCOS_IDN_NOT_AVAILABLE) &&
					(idns[i].error != DSP_SERCOS_ELEMENT_MISSING) &&
					(idns[i].error != DSP_SERCOS_WRITE_PROTECT) &&
					(idns[i].error != DSP_SERCOS_MIN) &&
					(idns[i].error != DSP_SERCOS_MAX) &&
					(idns[i].error != DSP_SERCOS_INVALID_DATA) &&
					(idns[i].error != DSP_SERCOS_VARIABLE_READ) &&
					(idns[i].error != DSP_SERCOS_SHORT_TRANS) &&
					(idns[i].error != DSP_SERCOS_LONG_TRANS) &&
					(idns[i].error != DSP_SERCOS_STATIC_VAL))
				return dsp_error;
			}	
		}
	}
	return (dsp_error = DSP_OK);
}

int16 FNTYPE get_idns(int16 axis, unsigned16 dr_addr, unsigned16 firstidn, unsigned16 nidns, IDNS *idns)
{	unsigned16 i, channel, idn;
	int16 nwords;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	channel = dspPtr->sercdata[axis].channel;
	if(channel >= SERCOS_MAX_CHANNELS)		/* valid channel? */
	{	set_drive(dr_addr);
		channel = 0;
	}
	for (i = 0; i < nidns; i++)
	{	idn = i + firstidn;
		idns[i].value = 0;	/* initialize value, since data could be 2 or 4 bytes */
		idns[i].idn = idn;	
		idns[i].error = read_idn(channel, idn, &nwords, (unsigned16*)(&(idns[i].value)), FALSE);
		if (idns[i].error)
		{	if ((idns[i].error != DSP_SERCOS_IDN_NOT_AVAILABLE) &&
				(idns[i].error != DSP_SERCOS_ELEMENT_MISSING) &&
				(idns[i].error != DSP_SERCOS_WRITE_PROTECT) &&
				(idns[i].error != DSP_SERCOS_MIN) &&
				(idns[i].error != DSP_SERCOS_MAX) &&
				(idns[i].error != DSP_SERCOS_INVALID_DATA) &&
				(idns[i].error != DSP_SERCOS_VARIABLE_READ) &&
				(idns[i].error != DSP_SERCOS_SHORT_TRANS) &&
				(idns[i].error != DSP_SERCOS_LONG_TRANS))
			return dsp_error;
		}
	}
	return (dsp_error = DSP_OK);
}

int16 FNTYPE start_exec_procedure(int16 axis, unsigned16 procedure)
{	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return start_exec_proc(channel, procedure);
}

int16 FNTYPE cancel_exec_procedure(int16 axis, unsigned16 procedure)
{	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return cancel_exec_proc(channel, procedure);
}

int16 FNTYPE exec_procedure_done(int16 axis, unsigned16 procedure, int16 *done)
{	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return exec_proc_done(channel, procedure, done);
}

int16 FNTYPE execute_procedure(int16 axis, unsigned16 procedure)
{	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;
	return execute_proc(channel, procedure);
}

unsigned16 LOCAL_FN get_mdt_data_addr(int16 axis)
{	unsigned16 channel, mdt_ptr, mdt_dc_ptr[PCDSP_MAX_AXES];
	int16 i;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;

	mdt_ptr = 0x800 + PHASE34_TTH;
	mdt_dc_ptr[0] = mdt_ptr + 4;
	for(i = 1; i < PCDSP_MAX_AXES; i++)
		mdt_dc_ptr[i] = mdt_dc_ptr[i - 1] + (dsp_read_pm(mdt_dc_ptr[i - 1]) & 0x1FF) + 1;
	return (mdt_dc_ptr[channel] + 1);
}

int16 FNTYPE enable_sercos_amplifier(int16 axis, int16 enable)
{	unsigned16 mdt_data;

	mdt_data = get_mdt_data_addr(axis);
	if(enable)      
		return serc_write(mdt_data, (int16)(0xE000));
	else
		return serc_write(mdt_data, (int16)(0x0000));
}

int16 FNTYPE sercos_enabled(int16 axis, P_INT state)
{	unsigned16 mdt_data;

	mdt_data = get_mdt_data_addr(axis);
	if(serc_read(mdt_data) & 0xe000)
		*state = 1;
	else
		*state = 0;
	return dsp_error;
}

int16 FNTYPE set_sercos_velocity_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	if(coeffs[0] != -1)
		set_idn(axis, 100, coeffs[0]);
	if(coeffs[1] != -1)
		set_idn(axis, 101, coeffs[1]);
	return dsp_error;
}

int16 FNTYPE set_sercos_position_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_idn(axis, 104, coeffs[0]);
}

int16 FNTYPE set_sercos_current_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return set_idn(axis, 106, coeffs[0]);
}

int16 FNTYPE get_sercos_velocity_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	get_idn(axis, 100, &coeffs[0]);
	get_idn(axis, 101, &coeffs[1]);
	return dsp_error;
}

int16 FNTYPE get_sercos_position_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_idn(axis, 104, &coeffs[0]);
}

int16 FNTYPE get_sercos_current_filter(int16 axis, long* coeffs)
{	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	return get_idn(axis, 106, &coeffs[0]);
}

int16 FNTYPE turn_on_sercos_led(void)
{	DSP_PM	current = serc_read(0xC02);
	return serc_write(0xC02, (int16)(current | 0x007F));
}

int16 FNTYPE turn_off_sercos_led(void)
{	DSP_PM	current = serc_read(0xC02);
	return serc_write(0xC02, (int16)(current & ~0x0040));
}

int16 FNTYPE read_cyclic_at_data(int16 axis, unsigned16 offset)
{	int16 addr;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	addr = dsp_read_dm((unsigned16)(C_DATA_START_IN+axis));
	return dsp_read_pm((unsigned16)(addr+offset));
}

int16 FNTYPE read_cyclic_mdt_data(int16 axis, unsigned16 offset)
{	int16 addr;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	addr = dsp_read_dm((unsigned16)(C_DATA_START_OUT+axis));
	return dsp_read_pm((unsigned16)(addr+offset));
}

int16 FNTYPE write_cyclic_mdt_data(int16 axis, unsigned16 offset, int16 data)
{	int16 addr;
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	addr = dsp_read_dm((unsigned16)(C_DATA_START_OUT+axis));
	return dsp_write_pm((unsigned16)(addr+offset), data);
}

int16 FNTYPE change_operation_mode(int16 axis, unsigned16 mode)
{	unsigned16 mdt_data;
	int16 old_val;
	
	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;

	if(mode > 3)
		return DSP_ILLEGAL_PARAMETER;

	mdt_data = get_mdt_data_addr(axis);
	old_val = serc_read(mdt_data);
	return serc_write(mdt_data, (int16)((old_val&~0x0300)|(mode<<8)));
}

int16 FNTYPE get_idn_size(int16 axis, unsigned16 idn, int16 *size)
{	int16 e_3;
	unsigned long elem_3;
	unsigned16 channel;

	if (get_sercos_channel(axis, &channel))
		return dsp_error;

	get_element_3(channel, idn, &elem_3);
	e_3 = (int16)((elem_3 >> 16) & 0x0007);
	if(e_3 > 2)
		*size = 0;
	else
		*size = e_3;
	return dsp_error;
}

int16 FNTYPE get_drive_status(int16 axis, int16 * status)
{	unsigned16 i, addr, channel;
	
	if (get_sercos_channel(axis, &channel))
		return dsp_error;

	addr = (unsigned16)serc_read((unsigned16)(S410B_RAM+1));
	for(i = 0; i < channel; i++)
	{	addr = (unsigned16)serc_read((unsigned16)(S410B_RAM+addr+3));
	}
	*status = serc_read((unsigned16)(S410B_RAM+addr+7));
	return dsp_error;
}

int16 FNTYPE reset_sercos_drive(int16 axis)
{	
	return(execute_procedure(axis, 99));
}

int16 FNTYPE get_idn_string(int16 axis, unsigned16 dr_addr, unsigned16 idn, char * msgstr)
{	unsigned16 channel;

	if(pcdsp_sick(dspPtr, axis))
		return dsp_error;
	channel = dspPtr->sercdata[axis].channel;
	if(channel >= SERCOS_MAX_CHANNELS)		/* valid channel? */
	{	set_drive(dr_addr);
		channel = 0;
	}
	return read_idn_string(channel, idn, msgstr);
}

int16 FNTYPE get_sercos_phase(int16 *phase)
{	if(pcdsp_init_check(dspPtr))
		return dsp_error;
	if(!(dspPtr->sercos))
		return (dsp_error = DSP_FUNCTION_NOT_APPLICABLE);

	/* read initialization phase from SERCON 410B */
	*phase = (dsp_read_pm((int16)(S410B_CNTRL + PHAS0_ADDR))) & 0x00FF;
	return dsp_error;
}
