/*      
	SERCRSET.C - SERCOS RESET Routines 
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
#include <math.h>
#include <string.h>
#include "sercrset.h"

int16 SC[8] = {	SC_0_START,
				SC_1_START,
				SC_2_START,
				SC_3_START,
				SC_4_START,
				SC_5_START,
				SC_6_START,
				SC_7_START };

DRIVE_CFG drive[PCDSP_MAX_AXES];
DriveIdns phase2_idns = NULL;
unsigned16 nphase2_idns = 0;
DriveIdns phase3_idns = NULL;
unsigned16 nphase3_idns = 0;
CyclicData atdata = NULL;
unsigned16 n_atdata = 0;
CyclicData mdtdata = NULL;
unsigned16 n_mdtdata = 0;

unsigned16 dsp_serc_enable;
unsigned16 bit_rate;
unsigned16 sample_rate;

int16 FNTYPE calculate_phase12_S410B_regs(void)
{	int16 TCYCSTART, TSCYC0, TCNTST, MCLKST, JTSCYC1, JTSCYC2, 
			JTRDEL1, JTRDEL2;
	double t_bit, t_start, JT_scyc;

	t_bit = 1.0/(double)bit_rate;					/* uS */
	t_start = (5.5*t_bit)+(3.5/(double)F_MCLK);		/* uS */
	TCNTST = (int16)floor((double)(t_start+1));
	serc_write((unsigned16)(S410B_CNTRL+0x13), TCNTST);

	MCLKST = (int16)(((double)TCNTST-t_start)*(double)F_MCLK+1.0);
	serc_write((unsigned16)(S410B_CNTRL+0xE), (int16)((MCLKST<<8)|(F_MCLK-1)));

	TCYCSTART = 0;
	serc_write((int16)(S410B_CNTRL+0x14), TCYCSTART);

	sample_rate = dsp_sample_rate();
	if(sample_rate > PHASE12_MAX_SAMPLE_RATE)
	{	set_sample_rate(PHASE12_MAX_SAMPLE_RATE);
		TSCYC0 = PHASE12_MIN_CYCLE_TIME;
	}
	else
		TSCYC0 = (int16)(1000000.0/(double)sample_rate);
	serc_write((unsigned16)(S410B_CNTRL+0xF), TSCYC0);
	serc_write((unsigned16)(S410B_CNTRL+0x10), TSCYC0);

	JT_scyc = 5.0+(4.0*t_bit);

	JTSCYC1 = TCNTST - (int16)JT_scyc;
	JTSCYC2 = TCNTST + (int16)JT_scyc;
	serc_write((unsigned16)(S410B_CNTRL+0x15), JTSCYC1);
	serc_write((unsigned16)(S410B_CNTRL+0x16), JTSCYC2);

	JTRDEL1 = 0;
	JTRDEL2 = 500;		/* uS */
	serc_write((unsigned16)(S410B_CNTRL+0x18), JTRDEL1);
	serc_write((unsigned16)(S410B_CNTRL+0x19), JTRDEL2);

	return (dsp_error = DSP_OK);
}

int16 FNTYPE calculate_phase34_S410B_regs(void)
{	int16 TSCYC1, JTSCYC1, JTSCYC2, JTRDEL1, JTRDEL2, TCYCDEL, TCNTST;
	double t_bit, x, JT_scyc, t_cycdel, t_rdelnom;

	set_sample_rate(sample_rate);
	TSCYC1 = (int16)(1000000.0/(double)sample_rate);
	serc_write((unsigned16)(S410B_CNTRL+0x10), TSCYC1);

	t_bit = 1.0/(double)bit_rate;							/* uS */
	x = (double)TSCYC1*.005;
	if(x > 5.0)									/* min() function is not ANSI */
		x = 5.0;
	JT_scyc = x+(4.0*t_bit);

	TCNTST = serc_read((unsigned16)(S410B_CNTRL+0x13));
	JTSCYC1 = TCNTST - (int16)JT_scyc;
	JTSCYC2 = TCNTST + (int16)JT_scyc;
	serc_write((unsigned16)(S410B_CNTRL+0x15), JTSCYC1);
	serc_write((unsigned16)(S410B_CNTRL+0x16), JTSCYC2);

	TCYCDEL = serc_read((unsigned16)(S410B_CNTRL+0x11));
	t_cycdel = (double)(TCYCDEL - TCNTST);
	t_rdelnom = 29.5*t_bit + t_cycdel + 2.5/(double)F_MCLK;

	JTRDEL1 = (int16)(t_rdelnom - JT_scyc - t_bit/2.0 - 0.5/(double)F_MCLK - 1.0);
	JTRDEL2 = (int16)(t_rdelnom + JT_scyc + t_bit/2.0 + 0.5/(double)F_MCLK + 2.0);
	serc_write((unsigned16)(S410B_CNTRL+0x18), JTRDEL1);
	serc_write((unsigned16)(S410B_CNTRL+0x19), JTRDEL2);

	return (dsp_error = DSP_OK);
}

int16 write_S410B_control_regs(void)
{	int16 cw_1 = REGMODE | ((bit_rate == BIT_RATE2)?SWSBAUD:0) | POLTXD | ENTSBAUD;
	
	cw_1 |= (serc_read((unsigned16)(S410B_CNTRL+0x1))&RSTFL);
	serc_write((unsigned16)(S410B_CNTRL+0x1),cw_1);             /* control registers */
	serc_write((unsigned16)(S410B_CNTRL+0x2),0x383);
	serc_write((unsigned16)(S410B_CNTRL+0x4),(int16)0xFFFF);
	serc_write((unsigned16)(S410B_CNTRL+0x5),(int16)0xFFFF);
	serc_write((unsigned16)(S410B_CNTRL+0x6),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x7),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x8),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x9),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0xA),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0xD),(int16)0xFF0A);
	serc_write((unsigned16)(S410B_CNTRL+0x1A),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x1B),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x1C),0x0);
	serc_write((unsigned16)(S410B_CNTRL+0x1D),0x0);
	return dsp_error;
}

int16 FNTYPE setup_phase12_transmit_blocks(void)
{	/* start of transmission blocks */
	serc_write((unsigned16)S410B_RAM,PHASE12_RTH);	/* pointer to ATM for phases 1-2 */
	serc_write((unsigned16)(S410B_RAM+0x1),PHASE34_RTH);	/* pointer to ATM for phases 3-4 */
	serc_write((unsigned16)(S410B_RAM+0xA),0x0);	/* MST error counter: not used */

	/* Receive buffer */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH),0x1000);	/* control word */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+1),0x64);	/* time: 100 uS */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+2),0x2);	/* length */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+3),PHASE12_TTH);/* pointer to next telegram header */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+4),0x0);	/* error counter */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+5),(int16)0xC002);/* Data container */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+6),0x0);	/* position of data block in telegram */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+7),0x0);	/* Drive status word (returned) */
	serc_write((unsigned16)(S410B_RAM+PHASE12_RTH+8),0x0);	/* Service Container (receive) */

	/* Transmit buffer */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH),0x4000);	/* control word */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+1),0x258);	/* time: 600 uS */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+2),0x2);	/* length */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+3),PHASE12_EM);	/* pointer to end marker */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+4),(int16)0xC002);	/* Data container */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+5),0x1);			/* 2 word data buffer (transmit) */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH+6),0x0);			/* Service Container (transmit) */

	serc_write((unsigned16)(S410B_RAM+PHASE12_EM),(int16)0xC000);	/* end marker for phase 1-2 */
	serc_write((unsigned16)(S410B_RAM+PHASE12_EM+1),0x3B6);			/* time: 950 uS */
	return dsp_error;
}

int16 FNTYPE initialize_service_containers(void)
{	int16 i;

	serc_write((unsigned16)(S410B_RAM+SC_BASE), 0x0);	/* will contain the number of active service containers */
	/* Service containers (8 only) */
	for(i = 0; i < PCDSP_MAX_AXES; i++)
	{	serc_write((unsigned16)(S410B_RAM+0x2+i), SC[i]);	/* addresses of service containers */
		serc_write((unsigned16)(S410B_RAM+SC[i]-4), 0x0);	/* will contain the axis assigned to container */
		serc_write((unsigned16)(S410B_RAM+SC[i]-3), 0x0);	/* will contain drive mode */
		serc_write((unsigned16)(S410B_RAM+SC[i]-2), 0x0);	/* will contain drive address */
		serc_write((unsigned16)(S410B_RAM+SC[i]-1), 0x0);	/* will contain drive manufacturer */
		serc_write((unsigned16)(S410B_RAM+SC[i]), 0x86);		/* control word */
		serc_write((unsigned16)(S410B_RAM+SC[i]+SC_CNTRL_AT), 0x0);
		serc_write((unsigned16)(S410B_RAM+SC[i]+SC_CNTRL_WR), 0x0);		/* pointers to current and last positions in write buffer */
		serc_write((unsigned16)(S410B_RAM+SC[i]+SC_CNTRL_RD), SC_RD_INIT);	/* pointers to current and last positions in read buffer */
		serc_write((unsigned16)(S410B_RAM+SC[i]+SC_CNTRL_ERR), 0x0);
	}
	return dsp_error;
}

int16 FNTYPE set_user_drv_parms(unsigned16 channel, unsigned16 dr_addr, 
	unsigned16 n_idns, DriveIdns drive_idns)
{	unsigned16 i, *v;
	int16 e, err = 0, len = -1;
	
	for(i = 0; i < n_idns; i++)
	{	
		if(drive_idns[i].drive_addr == dr_addr)
		{	v = (unsigned16*)(&(drive_idns[i].value));	
			len = -1;
			e = write_idn(channel, drive_idns[i].idn, &len, v, FALSE);
			if(e)
				err += 1;
		}
	}
	return err;
}

int16 FNTYPE setup_phase1(void)
{	dsp_write_dm(dsp_serc_enable, 0x423);    
	if(loop_closed())
	{	serc_write((unsigned16)(S410B_CNTRL+0xA), PHASE1);
		serc_write((unsigned16)(S410B_CNTRL+0x3), 0x100);
		return dsp_error;
	}
	else
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
}

int16 FNTYPE setup_phase2(int16 ndrives, DriveInfo dinfo)
{	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
	if(get_cfg_type(ndrives, dinfo))
		return dsp_error;
	else
	{	if(!find_drives(ndrives))	/* no error in find_drives() */
			serc_write((int16)(S410B_CNTRL+0xA), PHASE2);
	}
	return dsp_error;
}

int16 FNTYPE setup_phase3(int16 ndrives)
{	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
 	if(do_cyclic_data(ndrives))
		return dsp_error;
	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
	if(cfg_drives(ndrives))
		return dsp_error;
	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
	if(check_23(ndrives) == DSP_OK)
	{	wait_cycles(50);
		calculate_phase34_S410B_regs();
		dsp_write_dm(dsp_serc_enable,0x427);    
		serc_write((unsigned16)(S410B_CNTRL+0xA), PHASE3);
		wait_cycles(50);
		serc_write((unsigned16)(S410B_CNTRL + 0x4), (int16)0xFFFF);	/* clear interrupts */
	}
	return dsp_error;
}

int16 FNTYPE goto_phase4(int16 ndrives)
{	int16 dr_num;
	unsigned16 dr_addr;

	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	dr_addr = drive[dr_num].address;
		set_user_drv_parms(dr_num, dr_addr, nphase3_idns, phase3_idns);
	}
	if(check_34(ndrives) == DSP_OK)
	{	wait_cycles(50);
		serc_write((unsigned16)(S410B_CNTRL+0xA), PHASE4);
	}
	return dsp_error;
}

int16 FNTYPE finish_phase4(int16 ndrives)
{	int16 err, dr_num;

	if(!loop_closed())
		return (dsp_error = DSP_SERCOS_LOOP_OPEN);
	for(dr_num = 0; dr_num < ndrives; dr_num++)
		get_asc((unsigned16)dr_num, drive[dr_num].axis, 95);
	err = enable_drives(ndrives);
	for(dr_num = 0; dr_num < ndrives; dr_num++)
		get_asc((unsigned16)dr_num, drive[dr_num].axis, 95);
	return err;
}

int16 FNTYPE get_drive_addresses(int16 baud, int16 *ndrives, unsigned16 *dr_addrs)
{	int16 rec_stat, phase, done = FALSE;
	unsigned16 dr_addr, rec_addr;
	
	if (pcdsp_init_check(dspPtr))
		return dsp_error;

	phase = serc_read((int16)(S410B_CNTRL + PHAS0_ADDR));
	if(((phase&0x7) < 2) || (phase == -1))	/* not yet in phase 2 */
	{	bit_rate = baud;
	   /* find a better way to calculate this */
		dsp_serc_enable = dspPtr->e_data - (1 + 65 + 65 + 1);

		serc_write((unsigned16)(S410B_CNTRL+0x1), 0x2);	/* reset sercos chip */
		write_S410B_control_regs();
		calculate_phase12_S410B_regs();
		setup_phase12_transmit_blocks();
		initialize_service_containers();

		if(setup_phase1())
			return dsp_error;
	}

	*ndrives = 0;
	dr_addr = 1;
	while((dr_addr < 255) && !done)
	{	set_drive(dr_addr);  /* setup transmit buffer to send telegram to drive */
		serc_write((unsigned16)0x812, 0); /* zero first word in receive buffer */
		wait_cycles(10);
		rec_addr = ((unsigned16)serc_read(0x80B)) & 0xFF; /* get first 8 bits of receive buffer
															control word (address of sending
															drive */
		rec_stat = serc_read(0x812);        /* status of drive is returned in buffer */
		if((rec_addr == dr_addr) && (rec_stat & 1))
		{	dr_addrs[*ndrives] = dr_addr;
			*ndrives +=1;
		}
		if(*ndrives > 7)
			done = TRUE;
		if(loop_open())
			done = TRUE;
		dr_addr++;
	}
	return dsp_error;
}

int16 FNTYPE configure_phase2_idns(unsigned16 nidns, DriveIdns didns)
{	if(nidns)
	{	phase2_idns = didns;
		nphase2_idns = nidns;
	}
	else
	{	phase2_idns = NULL;
		nphase2_idns = 0;
	}
	return dsp_error;
}

int16 FNTYPE configure_phase3_idns(unsigned16 nidns, DriveIdns didns)
{	if(nidns)
	{	phase3_idns = didns;
		nphase3_idns = nidns;
	}
	else
	{	phase3_idns = NULL;
		nphase3_idns = 0;
	}
	return dsp_error;
}

int16 FNTYPE configure_at_data(unsigned16 natdata, CyclicData at_data)
{	if(natdata)
	{	atdata = at_data;
		n_atdata = natdata;
	}
	else
	{	atdata = NULL;
		n_atdata = 0;
	}
	return dsp_error;
}

int16 FNTYPE configure_mdt_data(unsigned16 nmdtdata, CyclicData mdt_data)
{	if(nmdtdata)
	{	mdtdata = mdt_data;
		n_mdtdata = nmdtdata;
	}
	else
	{	mdtdata = NULL;
		n_mdtdata = 0;
	}
	return dsp_error;
}

int16 FNTYPE serc_reset(int16 baud, int16 ndrives, DriveInfo dinfo)
{	int16 axis, drive;
	unsigned16 rate;

	if(pcdsp_init_check(dspPtr))
		return dsp_error;

   /* make sure we are called with valid parameters */
	if((dinfo == NULL) || (ndrives < 1) || (ndrives > PCDSP_MAX_AXES))
		return (dsp_error = DSP_SERCOS_INVALID_PARAM);

	rate = dsp_sample_rate();
	if(dsp_rset())
		return dsp_error;
	for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	drive = 0;
		while((axis != dinfo[drive].drive_axis) && (drive < ndrives))
			drive++;
		if(drive == ndrives)
			set_axis(axis, FALSE);
	}
	set_sample_rate(rate);

	bit_rate = baud;

	zero_log_count();
	zero_phase2_idncount();
	zero_phase3_idncount();

	if(check_addrs(ndrives, dinfo)) /* check to see if drive addrs overlap */
		return dsp_error;

   /* find a better way to calculate this */
	dsp_serc_enable = dspPtr->e_data - (1 + 65 + 65 + 1);

	serc_write((unsigned16)(S410B_CNTRL+0x1), 0x2);	/* reset sercos chip */
	write_S410B_control_regs();
	calculate_phase12_S410B_regs();
	setup_phase12_transmit_blocks();
	initialize_service_containers();

	if(setup_phase1())
		return dsp_error;
	if(setup_phase2(ndrives, dinfo))
		return dsp_error;
	if(setup_phase3(ndrives))
		return dsp_error;
	if(goto_phase4(ndrives))
		return dsp_error;
	return finish_phase4(ndrives);
}

int16 FNTYPE cfg_drives(int16 ndrives)
{
	int16 dr_num, mdt_time, p_mdt, len,
		dur_mdt, dur_mst, dur_at, TSCYC1;
	unsigned16 t_mtsy, mdt_dc_ptr, buff[1], dr_addr, channel = 0, t3, t4;

	/* interrogate drive for timing data */
	p_mdt = 1;
	TSCYC1 = (int16)(1000000.0/(double)sample_rate);

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	dr_addr = drive[dr_num].address;
		set_drive_sc(dr_addr, channel);		/* setup continuous transmit buffer to send to dr_addr */
		drive[dr_num].t_scyc = TSCYC1;		/* interface cycle time in micro-secs */
		drive[dr_num].t_ncyc = drive[dr_num].t_scyc;
		read_idn(channel, 3, &len, buff, FALSE);	/* read idn 3 */
		drive[dr_num].t1_min = buff[0];
		read_idn(channel, 4, &len, buff, FALSE);	/* read idn 4 */
		drive[dr_num].t_atmt = buff[0];
		read_idn(channel, 5, &len, buff, FALSE);	/* read idn 5 */
		drive[dr_num].t5 = buff[0];
		read_idn(channel, 88, &len, buff, FALSE);	/* read idn 88 */
		drive[dr_num].t_mtsy = buff[0];
		read_idn(channel, 90, &len, buff, FALSE);	/* read idn 90 */
		drive[dr_num].t_mtsg = buff[0];
		drive[dr_num].p_mdt = p_mdt;
		p_mdt +=  4 + drive[dr_num].mdt_data_len * 2;
	}

	/* calculate at times */
	/* at length (bits) =
 	8               BOF
 	8               ADR
 	16              Status
 	16              Service Data
 	16 * AT_LEN     Cyclic Data
 	16              FSC
 	8               EOF
 	= 72 + 16 * AT_LEN */
	/* at transmit time is length/bit rate( e.g. len/2 for 2Mbit) */
	drive[0].t1 = drive[0].t1_min+ 20;
	drive[0].mdt_len = p_mdt - 1;
	dur_at = (2 * 8 + (7 + drive[0].at_data_len * 2) * 10)/bit_rate + 1;
	for(dr_num = 1; dr_num < ndrives; dr_num++)
	{	unsigned16 at_time;

		/* assumes t_atnt + Jt * 2  < 10 */
		/*	at_time = drive[dr_num-1].t1 + dur_at + 20; */
		/* removed fudge factor (+20 microsec) for drive calculations */
		at_time = drive[dr_num-1].t1 + dur_at;

		if(at_time < drive[dr_num].t1_min)
			at_time = drive[dr_num].t1_min;
		drive[dr_num].t1 = at_time;
		drive[dr_num].mdt_len = p_mdt - 1;

		dur_at = (2 * 8 + (7 + drive[dr_num].at_data_len * 2) * 10)/bit_rate + 1;
	}

	/* calculate MST and MDT duration */
	dur_mst = (2 * 8 + 4 * 10)/bit_rate + 1;
	dur_mdt = (2 * 8 + (7 + (p_mdt-1)) * 10)/bit_rate + 1;

	/* mdt_time = last at transmission start time + duration of last at +
		t_atmt for the drive receiving the last at */
	/* fudge factor (+30 microsec) for improper drive calculations */
	mdt_time = drive[ndrives-1].t1 + dur_at + drive[ndrives-1].t_atmt + 30;

	t_mtsy = drive[0].t_scyc - (mdt_time + dur_mdt + dur_mst);

	t4 = drive[0].t_scyc;
	t3 = 0;
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	if(t_mtsy < drive[dr_num].t_mtsy)
			return (dsp_error = DSP_SERCOS_INVALID_CYCLE_TIME);
		drive[dr_num].t2 = mdt_time;
		drive[dr_num].t3 = mdt_time +  dur_mdt + drive[dr_num].t_mtsg + 10;
		if(drive[dr_num].t3 > t3)
			t3 = drive[dr_num].t3;
		drive[dr_num].t4 = drive[dr_num].t_scyc - drive[dr_num].t5 - 10;
		if(drive[dr_num].t4 < t4)
			t4 = drive[dr_num].t4;
	}
	/* use smallest t4 and largest t3 for all drives */
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	drive[dr_num].t4 = t4;
		drive[dr_num].t3 = t3;
	}

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	
		dr_addr = drive[dr_num].address;
		set_drive_sc(dr_addr, channel);

		len = 1;
		write_idn(channel, 1, &len, &drive[dr_num].t_ncyc, FALSE);
		len = 1;
		if(write_idn(channel, 2, &len, &drive[dr_num].t_scyc, FALSE))
			return (dsp_error = DSP_SERCOS_INVALID_CYCLE_TIME);
		len = 1;
		write_idn(channel, 6, &len, &drive[dr_num].t1, FALSE);
		len = 1;
		write_idn(channel, 89, &len, &drive[dr_num].t2, FALSE);
		len = 1;
		write_idn(channel, 9, &len, &drive[dr_num].p_mdt, FALSE);
		len = 1;
		write_idn(channel, 10, &len, &drive[dr_num].mdt_len, FALSE);
		len = 1;
		write_idn(channel, 7, &len, &drive[dr_num].t4, FALSE);
		dsp_error = DSP_OK;						/* Ignore return code */
		len = 1;
		write_idn(channel, 8, &len, &drive[dr_num].t3, FALSE);
		dsp_error = DSP_OK;						/* Ignore return code */

		set_user_drv_parms(0, dr_addr, nphase2_idns, phase2_idns);
	}

	/* build AT and MDT stuctures */
	serc_write((unsigned16)(S410B_RAM+PHASE34_TTH), 0x50FF);
	serc_write((unsigned16)(S410B_RAM+PHASE34_TTH+1), mdt_time);
	serc_write((unsigned16)(S410B_RAM+PHASE34_TTH+2), (int16)((p_mdt-1)/2));
	serc_write((unsigned16)(S410B_RAM+PHASE34_TTH+3), PHASE34_EM);

	mdt_dc_ptr = S410B_RAM+PHASE34_TDH;

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	int16 dc_init;
		unsigned16 at_ptr;

		dr_addr = drive[dr_num].address;
		at_ptr = S410B_RAM+PHASE34_RTH+(dr_num*MAX_AT_LEN);
		serc_write(at_ptr, (int16)(0x1800 + dr_addr));		/* receive telegram control word */
		serc_write((unsigned16)(at_ptr+1), drive[dr_num].t1);	/* time */
		serc_write((unsigned16)(at_ptr+2), (int16)(drive[dr_num].at_data_len+2));		/* length */
		serc_write((unsigned16)(at_ptr+3), (int16)(PHASE34_RTH+(dr_num+1)*MAX_AT_LEN));	/* address of next telegram */
		serc_write((unsigned16)(at_ptr+4), 0);						/* zero error counter */
		dc_init = 0x4400+(dr_num*0x800)+drive[dr_num].at_data_len+2;
		if(dr_num == (ndrives-1))
		{	dc_init |= 0x8000;
			serc_write((unsigned16)(at_ptr+3), PHASE34_TTH);	/* circular buffer */
		}
		serc_write((unsigned16)(at_ptr+5), dc_init);
		serc_write((unsigned16)(at_ptr+6), 0);
		drive[dr_num].at_data = at_ptr+7;
		dc_init = 0x4400 + (dr_num*S410B_RAM)+drive[dr_num].mdt_data_len+2;
		if(dr_num == (ndrives-1))
		{	dc_init |= 0x8000;
		}
		serc_write(mdt_dc_ptr, dc_init);
		drive[dr_num].mdt_data = mdt_dc_ptr + 1;
		/* data container length = cfg_data + 3
			(container header + control + service) */
		mdt_dc_ptr += (drive[dr_num].mdt_data_len + 3);
	}
	serc_write((unsigned16)(S410B_RAM+PHASE34_EM),(int16)0xC000);		/* end marker for phase 3-4 */
	serc_write((unsigned16)(S410B_RAM+PHASE34_EM+1), (int16)(mdt_time+dur_mdt+10));
	return dsp_error;
}

int16 FNTYPE fill_out_sercdata_struct(int16 ndrives)
{	int16 axis_no, words, dr_num;
	unsigned16 addr, mode;

	for(axis_no = 0; axis_no < PCDSP_MAX_AXES; axis_no++)
	{	dspPtr->sercdata[axis_no].channel = SERCOS_MAX_CHANNELS;	/* uninitialized */
  		dspPtr->sercdata[axis_no].mode = SERCOS_MAX_CHANNELS;
  		dspPtr->sercdata[axis_no].drive_addr = SERCOS_MAX_CHANNELS;
  		dspPtr->sercdata[axis_no].drive_mfg = SERCOS_MAX_CHANNELS;
  	}
	addr = S410B_RAM + SC_BASE;
	serc_write(addr, ndrives);
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	addr = S410B_RAM + serc_read((int16)(0x802+dr_num)) - 4;
		axis_no = drive[dr_num].axis;
		if(serc_write(addr, axis_no))
			return dsp_error;
		dspPtr->sercdata[axis_no].channel = (unsigned16)dr_num;

		if(drive[dr_num].drive_mfg != LUTZE)
			read_idn(dr_num, 32, &words, &mode, FALSE);
		if(serc_write((unsigned16)(addr + 1), mode))
			return dsp_error;
		dspPtr->sercdata[axis_no].mode = mode;

		if(serc_write((unsigned16)(addr + 2), drive[dr_num].address))
			return dsp_error;
		dspPtr->sercdata[axis_no].drive_addr = drive[dr_num].address;

		if(serc_write((unsigned16)(addr + 3), drive[dr_num].drive_mfg))
			return dsp_error;
		dspPtr->sercdata[axis_no].drive_mfg = drive[dr_num].drive_mfg;
	}
	return dsp_error;
}

int16 FNTYPE setup_user_cyclic_in_data(int16 dr_num, int16 at_offset)
{	return dsp_write_dm((unsigned16)(C_DATA_START_IN+drive[dr_num].axis), (int16)(drive[dr_num].at_data+at_offset));
}

int16 FNTYPE setup_user_cyclic_out_data(int16 dr_num, int16 mdt_offset)
{	return dsp_write_dm((unsigned16)(C_DATA_START_OUT+drive[dr_num].axis), (int16)(drive[dr_num].mdt_data+mdt_offset));
}

int16 FNTYPE enable_drives(int16 ndrives)
{	int16 dr_num, axis, status_ptr, n_words_in = 0, n_words_out = 0;
	unsigned16  sc_in_ptr, sc_out_ptr;

	sc_in_ptr = dsp_serc_enable + 2;
	sc_out_ptr = dsp_serc_enable + 67;

	fill_out_sercdata_struct(ndrives);

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	int16 out_ptr,in_ptr;
		axis = drive[dr_num].axis;

		/* copy drive status word into external data memory, mapped by axis number */
		status_ptr = STATUSADDR + axis;
		dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data));
		sc_in_ptr++;
		dsp_write_dm(sc_in_ptr, status_ptr);
		sc_in_ptr++;
		n_words_in++;

		switch(drive[dr_num].drive_mode)
		{
			case TORQMODE:
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(8);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				setup_user_cyclic_in_data(dr_num, 4);
				setup_user_cyclic_out_data(dr_num, 3);
				break;

			case VELMODE:			   /* Velocity Mode */
			case EXT_VELMODE:		   /* Velocity Mode (External Encoder) */
			case VELOCITY_STD:
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(6);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr,out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+3));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;
				setup_user_cyclic_in_data(dr_num, 4);
				setup_user_cyclic_out_data(dr_num, 4);
				break;

			case POSMODE:
			case POSITION_STD:
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(4);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+3));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;

				setup_user_cyclic_in_data(dr_num, 4);
				setup_user_cyclic_out_data(dr_num, 4);
				break;

			case POS_VEL_STD:
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr,in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(4);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+3));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(6);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+4));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+5));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;

				setup_user_cyclic_in_data(dr_num, 6);
				setup_user_cyclic_out_data(dr_num, 6);
				break;

			case DUAL_LOOP_VELMODE:  /* Dual loop with External Encoder */
				/* in_ptr points to encoder temporary for the axis # */
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr,in_ptr);
				sc_in_ptr++;
				n_words_in++;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+4));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, (int16)(in_ptr+1));
				sc_in_ptr++;
				n_words_in++;

				/* velocity command value comming from axis' DS_D(6) register */
				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(6);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+3));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;

				setup_user_cyclic_in_data(dr_num, 6);
				setup_user_cyclic_out_data(dr_num, 4);
				break;

			case ANALOG_VELMODE: /* External Analog */
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				in_ptr = dspPtr->global_data + GD_SIZE + (4 * dspPtr->axes) + axis + 1;
				set_axis((int16)(axis+1), TRUE);
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+5)); /* store upper 16 bits */
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis + 1) + DS_D(6);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+3));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, (int16)(out_ptr+1));
				sc_out_ptr++;
				n_words_out++;
				setup_user_cyclic_in_data(dr_num, 6);
				setup_user_cyclic_out_data(dr_num, 4);
				break;

			case ANALOG_TORQUEMODE: /* External Analog */
				/* in_ptr points to encoder temporary for the axis # */
				in_ptr = dspPtr->global_data + GD_SIZE + (21 * dspPtr->axes) + axis;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+2));
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				/* in_ptr points to analog input data for the axis # + 1 */
				set_axis((int16)(axis+1), TRUE);
				in_ptr = dspPtr->global_data + GD_SIZE + (4 * dspPtr->axes) + axis + 1;
				dsp_write_dm(sc_in_ptr, (int16)(drive[dr_num].at_data+5)); /* store upper 16 bits */
				sc_in_ptr++;
				dsp_write_dm(sc_in_ptr, in_ptr);
				sc_in_ptr++;
				n_words_in++;

				out_ptr = dspPtr->data_struct + DS(axis) + DS_D(8);
				dsp_write_dm(sc_out_ptr, (int16)(drive[dr_num].mdt_data+2));
				sc_out_ptr++;
				dsp_write_dm(sc_out_ptr, out_ptr);
				sc_out_ptr++;
				n_words_out++;
				setup_user_cyclic_in_data(dr_num, 6);
				setup_user_cyclic_out_data(dr_num, 4);
				break;

			case USERMAP:
				setup_user_cyclic_in_data(dr_num, 2);
				setup_user_cyclic_out_data(dr_num, 2);
				break;

			default:
				break;
		}
	}
	dsp_write_dm((unsigned16)(dsp_serc_enable + 1), n_words_in);
	dsp_write_dm((unsigned16)(dsp_serc_enable + 66), n_words_out);

	/* get_position, set_positon, put controller in run state */
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	long lx;
		LFIXED lfixed ;
		int16 len = -1;
		unsigned16 cfg_addr, feedback_idn;

		if(drive[dr_num].drive_mode != USERMAP)
		{
			feedback_idn = 51;
			axis = drive[dr_num].axis;
			/* map drive status to the amp fault input */
		 	cfg_addr = dspPtr->e_data+ED_HOME_PORT_OFFSET+ED(axis);
		 	status_ptr = STATUSADDR + axis;
		 	dsp_write_dm(cfg_addr, status_ptr);
		 	cfg_addr = dspPtr->e_data+ED_AMP_FAULT_MASK+ED(axis);
		 	dsp_write_dm(cfg_addr, 0x2000);
		 	/* map home input to real-time status bit 1(bit 6 of status word) */
		 	cfg_addr = dspPtr->e_data+ED_HOME_MASK+ED(axis);
		 	dsp_write_dm(cfg_addr, 0x0040);
		 	/* map pos and neg limit inputs to real-time status bit 2(bit 7 of status word) */
		 	cfg_addr = dspPtr->e_data+ED_POS_LIMIT_MASK+ED(axis);
		 	dsp_write_dm(cfg_addr, 0x0080);
		 	cfg_addr = dspPtr->e_data+ED_NEG_LIMIT_MASK+ED(axis);
		 	dsp_write_dm(cfg_addr, 0x0080);
			if((drive[dr_num].drive_mode == EXT_VELMODE) || 
				(drive[dr_num].drive_mode == DUAL_LOOP_VELMODE))
				feedback_idn = (unsigned16)0x880A;
			if((drive[dr_num].drive_mode == ANALOG_VELMODE) || 
				(drive[dr_num].drive_mode == ANALOG_TORQUEMODE))
			{	feedback_idn = (unsigned16)0x8809;
				axis += 1;
			}
			if(read_idn(dr_num, feedback_idn, &len, (unsigned16*)(&lx), FALSE) == DSP_OK) /* get_position */
			{	ipcdsp_fixed_pos(dspPtr, axis, (double)lx, &lfixed);
				
				if(!pcdsp_set_position(dspPtr, axis, &lfixed))
				{	if (dspPtr->laxis[axis].last == LA_COMMAND)
						pcdsp_set_last(dspPtr, axis, LA_COMMAND, &lfixed);
				}
				wait_cycles(10);
				controller_idle(axis);
				wait_cycles(10);
				controller_run(axis);
			}
			else
				dsp_error = DSP_OK;	/* ignore error code from idn 51 */
		}
	}
	return dsp_error;
}

int16 FNTYPE get_cfg_type(int16 ndrives, DriveInfo dinfo)
{	int16 dr_num;

	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	if(dinfo->drive_mode == 0) 
		{	return (dsp_error = DSP_SERCOS_INVALID_DRIVE_TYPE); /* which drive??????? */
		}
		else 
		{ 
			drive[dr_num].axis = dinfo->drive_axis;
			drive[dr_num].address = dinfo->drive_addr;
			drive[dr_num].drive_mode = dinfo->drive_mode;
			drive[dr_num].drive_mfg = dinfo->drive_mfg;
		}
		++dinfo;
	}
	return dsp_error;
}

int16 FNTYPE write_cyclic_in_data(int16 dr_num)
{	int16 len;
	unsigned16 index, buff[32];
	unsigned long elem_3;

	drive[dr_num].at_data_len = 0;
	for(index = 0; index < drive[dr_num].cyclic_data_in_len; index++)
	{	get_element_3(0, drive[dr_num].in_data[index], &elem_3);
		if(((elem_3>>16)&7) == 1)
			drive[dr_num].at_data_len += 1;
		if(((elem_3>>16)&7) == 2)
			drive[dr_num].at_data_len += 2;
	}
	if(drive[dr_num].at_data_len > MAX_CYCLIC_IDN_WORDS)
		return DSP_SERCOS_USER_AT;
	for(index = 0; index < drive[dr_num].cyclic_data_in_len; index++)
		buff[index] = drive[dr_num].in_data[index];
	len = (int16)drive[dr_num].cyclic_data_in_len;
	if(write_idn(0, 16, &len, buff, TRUE))
		dsp_error = DSP_SERCOS_INVALID_IDN_AT;
	return dsp_error;
}

int16 FNTYPE write_cyclic_out_data(int16 dr_num)
{	int16 len;
	unsigned16 index, buff[32];
	unsigned long elem_3;

	drive[dr_num].mdt_data_len = 0;
	for(index = 0; index < drive[dr_num].cyclic_data_out_len; index++)
	{	get_element_3(0, drive[dr_num].out_data[index], &elem_3);
		if(((elem_3>>16)&7) == 1)
			drive[dr_num].mdt_data_len += 1;
		if(((elem_3>>16)&7) == 2)
			drive[dr_num].mdt_data_len += 2;
	}
	if(drive[dr_num].mdt_data_len > MAX_CYCLIC_IDN_WORDS)
		return DSP_SERCOS_USER_MDT;
	for(index = 0; index < drive[dr_num].cyclic_data_out_len; index++)
		buff[index] = drive[dr_num].out_data[index];
	len = (int16)drive[dr_num].cyclic_data_out_len;
	if(write_idn(0, 24, &len, buff, TRUE))
		dsp_error = DSP_SERCOS_INVALID_IDN_MDT;
	return dsp_error;
}

int16 FNTYPE do_cyclic_data(int16 ndrives)
{	int16 dr_num, j, len;
	unsigned16 buff[32], index, i, idn_ok, mode, dr_addr, channel = 0;
	
	enable_sc_transmission();		/* first use of service channel */
	for(dr_num = 0; dr_num < ndrives; dr_num++)
	{	dr_addr = drive[dr_num].address;
		set_drive_sc(dr_addr, channel);

		switch(drive[dr_num].drive_mode)
		{	case TORQMODE:
				mode = SERCOS_PM_TORQUE;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = 51;
				drive[dr_num].master_cyclic_data_in_len = 1;
				drive[dr_num].out_data[0] = 80;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case VELMODE:
				mode = SERCOS_PM_VEL;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = 51;
				drive[dr_num].master_cyclic_data_in_len = 1;
				drive[dr_num].out_data[0] = 36;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case POSMODE:
				mode = SERCOS_PM_POS;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = 51;
				drive[dr_num].master_cyclic_data_in_len = 1;
				drive[dr_num].out_data[0] = 47;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case VELOCITY_STD:
				mode = SERCOS_PM_VEL;
				drive[dr_num].telegram_type = 3;
				drive[dr_num].master_cyclic_data_in_len = 0;
				drive[dr_num].at_data_len = 2;
				drive[dr_num].master_cyclic_data_out_len = 0;
				drive[dr_num].mdt_data_len = 2;
				break;

			case POSITION_STD:
				mode = SERCOS_PM_POS;
				drive[dr_num].telegram_type = 4;
				drive[dr_num].master_cyclic_data_in_len = 0;
				drive[dr_num].at_data_len = 2;
				drive[dr_num].master_cyclic_data_out_len = 0;
				drive[dr_num].mdt_data_len = 2;
				break;

			case POS_VEL_STD:
				mode = SERCOS_PM_VEL;
				len = -1;
				write_idn(0, 33, &len, &mode, FALSE);	/* write secondary op-mode */
				mode = SERCOS_PM_POS;
				drive[dr_num].telegram_type = 5;
				drive[dr_num].master_cyclic_data_in_len = 0;
				drive[dr_num].at_data_len = 4;
				drive[dr_num].master_cyclic_data_out_len = 0;
				drive[dr_num].mdt_data_len = 4;
				break;

			case EXT_VELMODE:
				mode = SERCOS_PM_VEL;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = (unsigned16)0x880A;
				drive[dr_num].master_cyclic_data_in_len = 1;
				drive[dr_num].out_data[0] = 36;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case DUAL_LOOP_VELMODE:
				mode = SERCOS_PM_VEL;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = (unsigned16)0x880A;			/* external encoder feedback */
				drive[dr_num].in_data[1] = 51;					/* resolver position feedback */
				drive[dr_num].master_cyclic_data_in_len = 2;
				drive[dr_num].out_data[0] = 36;							/* velocity command IDN */
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case ANALOG_VELMODE:
				mode = SERCOS_PM_VEL;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = 51;				/* position feedback */
				drive[dr_num].in_data[1] = (unsigned16)0x8809;			/* analog feedback DRIVE SPECIFIC */
				drive[dr_num].master_cyclic_data_in_len = 2;
				drive[dr_num].out_data[0] = 36;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case ANALOG_TORQUEMODE:
				mode = SERCOS_PM_TORQUE;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].in_data[0] = 51;				/* position feedback */
				drive[dr_num].in_data[1] = (unsigned16)0x8809;			/* analog feedback DRIVE SPECIFIC */
				drive[dr_num].master_cyclic_data_in_len = 2;
				drive[dr_num].out_data[0] = 80;
				drive[dr_num].master_cyclic_data_out_len = 1;
				break;

			case USERMAP:
				mode = SERCOS_PM_IO;
				drive[dr_num].telegram_type = 7;
				drive[dr_num].master_cyclic_data_in_len = 0;
				drive[dr_num].master_cyclic_data_out_len = 0;
				break;

			default:
				drive[dr_num].telegram_type = 0;
				drive[dr_num].master_cyclic_data_in_len = 0;
				drive[dr_num].master_cyclic_data_out_len = 0;
				break;
		}
		drive[dr_num].cyclic_data_in_len = drive[dr_num].master_cyclic_data_in_len;
		drive[dr_num].cyclic_data_out_len = drive[dr_num].master_cyclic_data_out_len;
		len = 1;
		write_idn(channel, 15, &len, &drive[dr_num].telegram_type, FALSE);
		dsp_error = DSP_OK;						/* Ignore return code */
		len = 1;
		write_idn(channel, 32, &len, &mode, FALSE);	/* write primary op-mode */
		dsp_error = DSP_OK;						/* Ignore return code */

		if(drive[dr_num].telegram_type == 7)
		{	for(index = 0; index < n_atdata; index++)
			{	if(atdata[index].drive_addr == dr_addr)
				{	drive[dr_num].in_data[drive[dr_num].cyclic_data_in_len] 
						= atdata[index].idn;
					drive[dr_num].cyclic_data_in_len++;
				}
			}
			for(index = 0; index < n_mdtdata; index++)
			{	if(mdtdata[index].drive_addr == dr_addr)
				{	drive[dr_num].out_data[drive[dr_num].cyclic_data_out_len] 
						= mdtdata[index].idn;
					drive[dr_num].cyclic_data_out_len++;
				}
			}
			/* check for validity of idns.  NOTE: this is only done if the user
				specifies cyclic data. */
			len = 0;
			if(read_idn(channel, 187, &len, buff, TRUE))
				return dsp_error;
			idn_ok = 0;
			for(i = 0; i < drive[dr_num].cyclic_data_in_len; i++)
			{	for(j = 0; j < len; j++)
				{	if(drive[dr_num].in_data[i] == buff[j])
						idn_ok +=1;
				}
			}
			if(idn_ok != drive[dr_num].cyclic_data_in_len)
				return (dsp_error = DSP_SERCOS_INVALID_IDN_AT);
			len = 0;
			if(read_idn(channel, 188, &len, buff, TRUE))
				return dsp_error;
			idn_ok = 0;
			for(i = 0; i < drive[dr_num].cyclic_data_out_len; i++)
			{	for(j = 0; j < len; j++)
				{	if(drive[dr_num].out_data[i] == buff[j])
						idn_ok +=1;
				}
			}
			if(idn_ok != drive[dr_num].cyclic_data_out_len)
				return (dsp_error = DSP_SERCOS_INVALID_IDN_MDT);

			if(write_cyclic_in_data(dr_num))
				return dsp_error;
			if(write_cyclic_out_data(dr_num))
				return dsp_error;
		}
	}
	return dsp_error;
}

