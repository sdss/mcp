/*      
	LLSRSET.C - SERCOS RESET support Routines 
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include <stdlib.h>
#include <string.h>

#include "idsp.h"
#include "sercos.h"
#include "sercrset.h"

#define ENGLISH   1
#define GERMAN    0

extern DRIVE_CFG drive[PCDSP_MAX_AXES];

int16 assignedDrive[PCDSP_MAX_AXES];
int16 driveCount;
MsgStr driveMsgs[PCDSP_MAX_AXES][MSGS_PER_AXIS];
int16 logCount[PCDSP_MAX_AXES];
int16 phase2_idncount[PCDSP_MAX_AXES];
unsigned16 phase2_idnlist[PCDSP_MAX_AXES][MAX_IDNLIST_LEN];
int16 phase3_idncount[PCDSP_MAX_AXES];
unsigned16 phase3_idnlist[PCDSP_MAX_AXES][MAX_IDNLIST_LEN];

int16 FNTYPE find_drives(int16 ndrives)
{	int16 rec_stat, dr_num, error = 0;
	unsigned16 dr_addr, rec_addr;

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	
		dr_addr = drive[dr_num].address;
		set_drive(dr_addr);  /* setup transmit buffer to send telegram to drive */
		serc_write(0x812, 0); /* zero first word in receive buffer */
		wait_cycles(100);
		/* read first 8 bits of receive buffer control word (address of sending drive) */
		rec_addr = ((unsigned16)serc_read((unsigned16)0x80B)) & 0xFF;
		rec_stat = serc_read((unsigned16)0x812);	/* status of drive is returned in buffer */
		if((rec_addr == dr_addr) && (rec_stat & 1))
		{	assignedDrive[drive[dr_num].axis] = (int16)dr_addr;
			driveCount++;
		}
		else
		{	assignedDrive[drive[dr_num].axis] = -1;
			driveCount++;
			error++;
		}
	}
	serc_write((unsigned16)0x814, (int16)0x4000);		/* stop sending transmissions */
	if(error)
		return (dsp_error = DSP_SERCOS_DRIVE_INIT);
	else
		return (dsp_error = DSP_OK);
}

int16 FNTYPE loop_open(void)
{	int16 stat;
	unsigned16 addr = S410B_CNTRL + INT_RDIST_ADDR;

	if (!dspPtr->sercos)	/* is this a SERCOS controller? */
		return FALSE;

	serc_write(addr, (INT_FIBBR | INT_MSTMISS));	/* clear latched interrupts */
	stat = serc_read(addr);
	if (stat & INT_FIBBR)
	{	dsp_error = DSP_SERCOS_LOOP_OPEN;
		return TRUE;
	}
	if (stat & INT_MSTMISS)
	{	dsp_error = DSP_SERCOS_MST_MISSING;
		return TRUE;
	}
	return FALSE;
}

int16 FNTYPE loop_closed(void)
{	int16 stat, count = 0, closed = 0, done = 0;
	unsigned16 addr = S410B_CNTRL + INT_RDIST_ADDR;
	
	while(!done)
	{	serc_write(addr, (int16)0xFFFF);   /* clear all interrupts in 4H */
		wait_cycles(20);
		count++;
		stat = (serc_read(addr) & 0x382);
		if(stat == 0) /* loop closed */
		{	closed = 1;
			done = 1;
		}
		else /* loop not closed */
		{	if(count >= 50)
				done = 1;
		}
	}
	return closed;
}

int16 FNTYPE check_23(int16 ndrives)
{	int16 dr_num, len = -1, err, axis, proc_err_127 = 0;
	unsigned16 drive_addr,channel = 0, val = ENGLISH;

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	drive_addr = drive[dr_num].address;
		axis = drive[dr_num].axis;
		set_drive_sc(drive_addr, channel);

		/* if we have an INDRAMAT drive, tell it to speak English */
		if(drive[dr_num].drive_mfg == INDRAMAT) 
		{	val = ENGLISH;
			len = -1;
			write_idn(channel, (unsigned16)0x8005, &len, &val, FALSE);  
		}

		err = execute_proc(channel,99);
		get_asc(channel, axis, 95);
		if(err)
			return err;
		err = execute_proc(channel,127);
		get_asc(channel, axis, 95);
		if(err)
		{	proc_err_127 += 1;
			read_idn(channel, (int16)21, &phase2_idncount[axis], 
				phase2_idnlist[axis], TRUE);
		}
	}
	if(proc_err_127)
		return (dsp_error = DSP_SERCOS_127_FAILURE);
	return (dsp_error = DSP_OK);
}

int16 FNTYPE check_34(int16 ndrives)
{	int16 dr_num, err, axis, proc_err_128 = 0;

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	axis = drive[dr_num].axis;
		err = execute_proc(dr_num,99);
		get_asc(dr_num, axis, 95); /* channel does not correspond to axis number */
		if(err)
			return err;
		err = execute_proc(dr_num,128);
		get_asc(dr_num, axis, 95); /* channel does not correspond to axis number */
		if(err)
		{	proc_err_128 += 1;
			read_idn(dr_num, (int16)22, &phase3_idncount[axis], 
				phase3_idnlist[axis], TRUE);
		}
	}
	if(proc_err_128)
		return (dsp_error = DSP_SERCOS_128_FAILURE);
	return (dsp_error = DSP_OK);
}

int16 FNTYPE check_addrs(int16 ndrives, DriveInfo dinfo)
{	int16 i, j;
	unsigned16 addr;

	if((ndrives <= 0) || (ndrives > PCDSP_MAX_AXES))
		return (dsp_error = DSP_SERCOS_INVALID_DRIVE_NUMBER);
	for(i = 0; i < ndrives; i++)
	{	addr = dinfo[i].drive_addr;
	  	if(addr >= SERCOS_MAX_CHANNELS)
		 	return (dsp_error = DSP_SERCOS_INVALID_DRIVE_ADDR);
		for(j = i + 1; j < ndrives; j++)
		{	if(addr == dinfo[j].drive_addr)
				return (dsp_error = DSP_SERCOS_DUPLICATE_DRIVE_ADDR);
		}
	}
	return (dsp_error = DSP_OK);
}

int16 FNTYPE get_asc(unsigned16 channel, int16 axis, unsigned16 idn)
{	read_idn_string(channel, idn, (char*)driveMsgs[axis][logCount[axis]].msgstr);
	logCount[axis] += 1;
	return dsp_error;
}

int16 FNTYPE zero_log_count(void)
{	int16 i;

	for(i = 0; i < PCDSP_MAX_AXES; i++)
		logCount[i] = 0;
	return dsp_error;
}

int16 FNTYPE zero_phase2_idncount(void)
{	int16 i;

	for(i = 0; i < PCDSP_MAX_AXES; i++)
		phase2_idncount[i] = 0;
	return dsp_error;
}

int16 FNTYPE zero_phase3_idncount(void)
{	int16 i;

	for(i = 0; i < PCDSP_MAX_AXES; i++)
		phase3_idncount[i] = 0;
	return dsp_error;
}

