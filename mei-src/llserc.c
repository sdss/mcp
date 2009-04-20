/*
	LLSERC.C - Low level Sercos routines
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
#include "sercos.h"
#include "sercrset.h"

int16 FNTYPE enable_sc_transmission(void)
{	serc_write((unsigned16)(S410B_RAM+PHASE12_RDH), (int16)(0xC402));		/* enable sc receive */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TDH), (int16)(0xC402));		/* enable sc transmit */
	return dsp_error;
}

int16 FNTYPE set_drive(unsigned16 drive_addr)
{	/* write control word to transmit buffer, lower 8 bits is the drive address */
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH), (int16)(0x5000 + drive_addr)); 
	wait_cycles(5);
	return dsp_error;
}

int16 FNTYPE set_drive_sc(unsigned16 drive_addr, unsigned16 channel)
{	unsigned16 sc_ptr = (unsigned16)sc_address(channel);
	int16 hs_mdt = 1;

	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH), (int16)(0x4000 + drive_addr));	/* stop sc transmission */
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_AT), 0x0);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_RD), SC_RD_INIT);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_ERR), 0);
	serc_write(sc_ptr, (int16)(END_MDT | LS_MDT | hs_mdt));
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH), (int16)(0x4800 + drive_addr));	/* send one sc transmission */
	wait_cycles(50);
	if((serc_read(0x80B)&0xFF) == (int16)drive_addr)
		hs_mdt = serc_read((unsigned16)(sc_ptr+SC_CNTRL_AT))&HS_AT;
	else
		hs_mdt = 0;

	serc_write((unsigned16)(sc_ptr+SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_RD), SC_RD_INIT);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_ERR), 0);
	serc_write(sc_ptr, (int16)(M_BUSY | END_MDT | LS_MDT | hs_mdt));
	serc_write((unsigned16)(S410B_RAM+PHASE12_TTH), (int16)(0x5000 + drive_addr));	/* continuous sc transmission */
	clear_sc(sc_ptr);
	return dsp_error;
}

int16 FNTYPE slave_err_check(unsigned16 sc_ptr)
{	int16 code, element, error;

	code = serc_read((unsigned16)(sc_ptr+SC_CNTRL_LEN+SC_WR_LEN));
	element = (code>>12)&0x7;
	error = code&0xF;
	switch(error)
	{	case 1:
			switch(element)
			{	case 0:
					dsp_error = DSP_SERCOS_NO_CHANNEL;
					break;

				case 1:
					dsp_error = DSP_SERCOS_IDN_NOT_AVAILABLE;
					break;

				default:
					dsp_error = DSP_SERCOS_ELEMENT_MISSING;
					break;
			}
			break;


		case 2:
			dsp_error = DSP_SERCOS_SHORT_TRANS;
			break;

		case 3:
			dsp_error = DSP_SERCOS_LONG_TRANS;
			break;
			
		case 4:
			dsp_error = DSP_SERCOS_STATIC_VAL;
			break;

		case 5:
			dsp_error = DSP_SERCOS_WRITE_PROTECT;
			break;

		case 6:
			dsp_error = DSP_SERCOS_MIN;
			break;

		case 7:
			dsp_error = DSP_SERCOS_MAX;
			break;

		case 8:
			dsp_error = DSP_SERCOS_INVALID_DATA;
			break;

		default:
			dsp_error = DSP_SERCOS_SLAVE_ERROR;
			break;
	}
	return dsp_error;
}

void rec_err_clear(unsigned16 sc_ptr)
{	int16 hs_mdt;

	hs_mdt = serc_read((unsigned16)(sc_ptr+SC_CNTRL_AT))&HS_AT;
	serc_write(sc_ptr, (int16)(M_BUSY | END_MDT | LS_MDT | hs_mdt));
	wait_cycles(5);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_RD), SC_RD_INIT);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_ERR), 0);
}

int16 FNTYPE wait_sc(unsigned16 sc_ptr)
{	unsigned16 wc = 0;
	int16 sc_err;

	while(((serc_read(sc_ptr) & (SL_ERR | M_BUSY)) == 0) && (wc < SC_TIMEOUT))
	{	
		if (loop_open())
			return dsp_error;
		sc_err = serc_read((unsigned16)(sc_ptr + SC_CNTRL_ERR));
		if(sc_err & INT_SC_ERR)
			return (dsp_error = DSP_SERCOS_PROTOCOL);
		if(sc_err & INT_HS_TIMEOUT)
			return (dsp_error = DSP_SERCOS_HS_TIMEOUT);

		wait_cycles(1);
		wc++;
	}
	if(wc >= SC_TIMEOUT)
		return (dsp_error = DSP_SERCOS_SC_TIMEOUT);

	return dsp_error;
}

int16 FNTYPE check_sc(unsigned16 sc_ptr)
{	if(serc_read(sc_ptr) & SL_ERR)   /* if slave error */
	{	slave_err_check(sc_ptr);
		clear_sc(sc_ptr);
		return dsp_error;
	}

	if(loop_open()) 
		return dsp_error;

	return (dsp_error = DSP_OK);
}

int16 FNTYPE clear_sc(unsigned16 sc_ptr)
{	serc_write((unsigned16)(sc_ptr+SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_RD), SC_RD_INIT);
	serc_write((unsigned16)(sc_ptr+SC_CNTRL_ERR), 0);
	serc_write(sc_ptr, (int16)(END_MDT | LS_MDT | sc_handshake(sc_ptr)));
	wait_sc(sc_ptr);
	return dsp_error;
}

int16 FNTYPE initialize_service_container(unsigned16 sc_ptr, unsigned16 idn)
{	int16 i, wc = 0;

	dsp_error = DSP_OK;
	while(((serc_read(sc_ptr)&M_BUSY) == 0) && (wc < SC_TIMEOUT))
	{	if (loop_open())
			return dsp_error;
		wait_cycles(1);
		wc++;
	}
	if(wc >= SC_TIMEOUT)
		return (dsp_error = DSP_SERCOS_SC_TIMEOUT);

	for(i = 1; i < (int16)(SC_WR_LEN+SC_WR_LEN); i++)		/* clear the service container buffers */
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_LEN + i), 0x0);
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_LEN), idn);	/* write idn to first word in SC buffer */
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), 0x0);    /* 1 word buffer */
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)(SC_RD_INIT));  /* 1 word read buffer */

	/* write control word to start service container trasmission */
	serc_write(sc_ptr, (int16)(sc_handshake(sc_ptr) | LS_MDT | END_MDT | ELEM_1));
	if(wait_sc(sc_ptr))
	{	clear_sc(sc_ptr);
		return dsp_error;
	}
	return (check_sc(sc_ptr));
}

int16 FNTYPE wait_cycles(int16 n_cycles)
{	int16 dt, t_end;

	t_end = dsp_read_dm(DM_CLOCK) + n_cycles;
	do
	{	dt = t_end - dsp_read_dm(DM_CLOCK);
	}
	while(dt > 0);
	return dsp_error;
}

/* used for either 1 or 2 word elements */
int16 FNTYPE get_element(unsigned16 channel, unsigned16 idn,
	unsigned16 element, int16 n_words, unsigned16 *elem)
{	unsigned16 sc_ptr = (unsigned16)sc_address(channel), cw;
	int16 i;

	if(initialize_service_container(sc_ptr, idn))
		return dsp_error;

	serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)(SC_RD_INIT + 0x100 * (n_words-1)));
	cw = sc_handshake(sc_ptr) | element;
	if(n_words == 1)
		cw |= END_MDT;
	else
		cw |= SETEND;
	serc_write(sc_ptr, cw);

	if(wait_sc(sc_ptr))
	{	clear_sc(sc_ptr);
		return dsp_error;
	}
	if(check_sc(sc_ptr))
		return dsp_error;

	for(i = 0; i < n_words; i++)
		elem[i] = (unsigned16)serc_read((unsigned16)(sc_ptr + SC_CNTRL_LEN + SC_WR_LEN + i));
	clear_sc(sc_ptr);
	return (dsp_error = DSP_OK);
}

/* used for variable length elements */
int16 FNTYPE get_element_variable(unsigned16 channel, unsigned16 idn,
	unsigned16 element, int16 *n_words, unsigned16 *elem)
{	unsigned16 sc_ptr = (unsigned16)sc_address(channel);
	int16 i, cw, n_bytes, words, wordsleft, wordswritten;

	if(initialize_service_container(sc_ptr, idn))
		return dsp_error;

	/* get length of string first */
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)(SC_RD_INIT+0x100));
	cw = sc_handshake(sc_ptr) | element;
	serc_write(sc_ptr, cw);

	if(wait_sc(sc_ptr))
	{	clear_sc(sc_ptr);
		return dsp_error;
	}
	if(check_sc(sc_ptr))
		return dsp_error;

	n_bytes = serc_read((unsigned16)(sc_ptr + SC_CNTRL_LEN + SC_WR_LEN));
	if(n_bytes%2)
		*n_words = n_bytes/2+1;
	else
		*n_words = n_bytes/2;

	wordsleft = *n_words;
	wordswritten = 0;
	while(wordsleft)
	{	if(wordsleft <= SC_RD_LEN)
		{	words = wordsleft;
			wordsleft = 0;
		}
		else
		{	words = SC_RD_LEN;
			wordsleft -= SC_RD_LEN;
		}
		/* get ready to read idn string */
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)(SC_RD_INIT + 0x100 * (words-1)));

		if(words == 1)
	     	cw = sc_handshake(sc_ptr) | element | END_MDT;
		else
	     	cw = sc_handshake(sc_ptr) | element | SETEND;
		if(wordsleft)
			cw &= ~SETEND;

	  	serc_write(sc_ptr, cw);

		if(wait_sc(sc_ptr))
		{	clear_sc(sc_ptr);
			return dsp_error;
		}
		if(check_sc(sc_ptr))
			return dsp_error;

   	/* read data from service container into data buffer for return */
  		for(i = 0; i < words; i++)
  			elem[wordswritten+i] = 
				serc_read((unsigned16)(sc_ptr + SC_CNTRL_LEN + SC_WR_LEN + i));
		wordswritten += words;
	}
	clear_sc(sc_ptr);
	return (dsp_error = DSP_OK);
}

int16 FNTYPE get_element_1(unsigned16 channel, unsigned16 idn, unsigned16 *elem_1)
{	return get_element(channel, idn, ELEM_1, 1, elem_1);
}

int16 FNTYPE get_element_2(unsigned16 channel, unsigned16 idn, unsigned16 *elem_2)
{	int16 i, len;
	for(i = 0; i < 30; i++)
		elem_2[i] = 0;
	return get_element_variable(channel, idn, ELEM_2, &len, elem_2);
}

int16 FNTYPE get_element_3(unsigned16 channel, unsigned16 idn, unsigned long *elem_3)
{	unsigned16 data[2];
	if(get_element(channel, idn, ELEM_3, 2, data))
		return dsp_error;

	*elem_3 = ((unsigned long)data[0]) | (((unsigned long)data[1])<<16);

	return dsp_error;
}

int16 FNTYPE get_element_4(unsigned16 channel, unsigned16 idn, unsigned16 *elem_4)
{	int16 i, len;
	for(i = 0; i < 6; i++)
		elem_4[i] = 0;
	return get_element_variable(channel, idn, ELEM_4, &len, elem_4);
}

int16 FNTYPE get_element_min_max(unsigned16 channel, unsigned16 idn, 
	unsigned16 element, long *elem)
{	int16 n_words = 0, e_3;
	unsigned16 data[2];
	unsigned long elem_3;

	get_element_3(channel, idn, &elem_3);
	e_3 = (int16)((elem_3 >> 16) & 0x0007);

	if(e_3 >= 0x4)		/* variable length string */
		*elem = -1;

	if(e_3 == 2)		/* 4 byte length */
		n_words = 2;
	else
		if(e_3 == 1)	/* 2 byte length */
			n_words = 1;

	if(get_element(channel, idn, element, n_words, data))
		return dsp_error;

	*elem = 0L;
	if(n_words == 2)
		*elem = ((unsigned long)data[0]) | (((unsigned long)data[1])<<16);
	else	/* words == 1 */
		*elem = ((unsigned long)data[0])&0xFFFFL;

	return dsp_error;
}

int16 FNTYPE get_element_5(unsigned16 channel, unsigned16 idn, long *elem_5)
{	return get_element_min_max(channel, idn, ELEM_5, elem_5);
}

int16 FNTYPE get_element_6(unsigned16 channel, unsigned16 idn, long *elem_6)
{	return get_element_min_max(channel, idn, ELEM_6, elem_6);
}

int16 FNTYPE read_idn_attributes(unsigned16 channel, unsigned16 idn, IDN_ATTRIBUTES *attr)
{	if(get_element_1(channel, idn, &(attr->elem_1)))
		return dsp_error;		
	if(get_element_2(channel, idn, attr->elem_2))
		memcpy(attr->elem_2, "No text available", 20);
	get_element_3(channel, idn, &(attr->elem_3));
	if(get_element_4(channel, idn, attr->elem_4))
		memcpy(attr->elem_2, "", 2);
	if(get_element_5(channel, idn, &(attr->elem_5)))
		attr->elem_5 = -1;
	if(get_element_6(channel, idn, &(attr->elem_6)))
		attr->elem_6 = -1;

	return (dsp_error = DSP_OK);
}

int16 FNTYPE read_idn(unsigned16 channel, unsigned16 idn, int16 *n_words, 
	unsigned16 *data, int16 variable_length)
{	int16 e_3;
	unsigned long elem_3;

	if(variable_length)
		return get_element_variable(channel, idn, ELEM_7, n_words, data);

	if(get_element_3(channel, idn, &elem_3))
		return dsp_error;
	e_3 = (int16)((elem_3 >> 16) & 0x0007);

	if(e_3 >= 0x4)		/* variable length string */
		return (dsp_error = DSP_SERCOS_VARIABLE_READ);

	if(e_3 == 2)		/* 4 byte length */
		*n_words = 2;
	else
		if(e_3 == 1)	/* 2 byte length */
			*n_words = 1;

	return get_element(channel, idn, ELEM_7, *n_words, data);
}

/*	If *n_words <= -1 then write_idn(...) determines the data length.
	If *n_words is > 0 then write_idn(...) uses the specified data length.
	If *n_words is 0 then write_idn(...) does nothing.
 	If variable_length is TRUE then write_idn(...) writes a variable length
	string.
*/
int16 FNTYPE write_idn(unsigned16 channel, unsigned16 idn, int16 *n_words, 
	unsigned16 *data, int16 variable_length)
{	unsigned16 sc_ptr = (unsigned16)sc_address(channel);
	int16 i, cw, e_3, words, wordsleft, wordswritten;
	unsigned long elem_3;

	if (!(* n_words))		/* no data length? */
		return dsp_error;
		
	/* If variable length string, write the length of the string first (2 words) */
	if(variable_length)
	{	if(initialize_service_container(sc_ptr, idn))
			return dsp_error;
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_LEN), (int16)(*n_words*2));
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_LEN + 1), (int16)(*n_words*2));
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), (int16)(0x0100));
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)SC_RD_INIT);
		cw = sc_handshake(sc_ptr) | ELEM_7 | LS_MDT;
		serc_write(sc_ptr, cw);

		if(wait_sc(sc_ptr))
		{	clear_sc(sc_ptr);
			return dsp_error;
		}
		if(check_sc(sc_ptr))
			return dsp_error;
	}
	else
	{	if(*n_words <= -1)
		{	get_element_3(channel, idn, &elem_3);
			e_3 = (int16)((elem_3 >> 16) & 0x0007);

			if(e_3 == 2)
				*n_words = 2;
			else
				if(e_3 == 1)
					*n_words = 1;
		}

		if(initialize_service_container(sc_ptr, idn))
			return dsp_error;
	}
	
	wordsleft = *n_words;
	wordswritten = 0;
	while(wordsleft)
	{	if(wordsleft <= SC_WR_LEN)
		{	words = wordsleft;
			wordsleft = 0;
		}
		else
		{	words = 0x20;
			wordsleft -= SC_WR_LEN;
		}
		for(i = 0; i < words; i++)
			serc_write((unsigned16)(sc_ptr + SC_CNTRL_LEN + i), data[wordswritten + i]);

		serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), (int16)(0x100 * (words-1)));
		serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)SC_RD_INIT);

		if(words == 1)
			cw = sc_handshake(sc_ptr) | ELEM_7 | END_MDT | LS_MDT;
		else
			cw = sc_handshake(sc_ptr) | ELEM_7 | SETEND | LS_MDT;
		if(wordsleft)
			cw &= ~SETEND;

		serc_write(sc_ptr, cw);

		if(wait_sc(sc_ptr))
		{	clear_sc(sc_ptr);
			return dsp_error;
		}
		if(check_sc(sc_ptr))
			return dsp_error;

		wordswritten += words;
	}
	clear_sc(sc_ptr);
	return (dsp_error = DSP_OK);
}

int16 FNTYPE check_proc(unsigned16 channel, unsigned16 idn, int16 *status)
{	unsigned16 sc_ptr = (unsigned16)sc_address(channel);

	if(initialize_service_container(sc_ptr, idn))
		return dsp_error;

	*status = serc_read((unsigned16)(sc_ptr + SC_CNTRL_LEN + SC_WR_LEN));
	clear_sc(sc_ptr);
	return dsp_error;
}	

int16 FNTYPE start_exec_proc(unsigned16 channel, unsigned16 proc)
{	int16 len = 1;
	unsigned16 buff = (PROCEDURE_SET | PROCEDURE_ENABLE);

	return(write_idn(channel, proc, &len, &buff, FALSE));
}

int16 FNTYPE cancel_exec_proc(unsigned16 channel, unsigned16 proc)
{	int16 len = 1;
	unsigned16 buff = 0;	/* (~PROCEDURE_SET & ~PROCEDURE_ENABLE) */

	return(write_idn(channel, proc, &len, &buff, FALSE));
}

int16 FNTYPE exec_proc_done(unsigned16 channel, unsigned16 proc, int16 *done)
{	int16 status;
	*done = 0;

	if (check_proc(channel, proc, &status))
		return dsp_error;
	if (!(status & PROCEDURE_IN_PROGRESS))
		*done = TRUE;
	if (status & PROCEDURE_ERROR)
	{	*done = TRUE;	/* assume error causes procedure to end */
		dsp_error = DSP_SERCOS_PROC_FAILURE;
	}
	return dsp_error;
}

int16 FNTYPE execute_proc(unsigned16 channel, unsigned16 proc)
{	int16 done = 0;

	if (start_exec_proc(channel, proc))
		return dsp_error;
	wait_cycles(50);	/* wait for drive to start procedure */

	while (!done)
	{	if (exec_proc_done(channel, proc, &done))
			return dsp_error;
	}
	return cancel_exec_proc(channel, proc);
}

int16 FNTYPE read_idn_string(unsigned16 channel, unsigned16 idn, char *str)
{	int16 i, len;

	for(i = 0; i < MAX_ERROR_LEN; i++)
		str[i] = 0;
	if(read_idn(channel, idn, &len, (unsigned16*)str, TRUE))
	{	memcpy(str, "Error reading IDN", 20);
		return dsp_error;
	}
	return (dsp_error = DSP_OK);
}

int16 FNTYPE get_sercos_data_length(unsigned16 channel, unsigned16 idn, unsigned16 element, int16 *nbytes)
{	unsigned16 sc_ptr;
	int16 cw;

	sc_ptr = (unsigned16)sc_address(channel);

	if (initialize_service_container(sc_ptr, idn))
		return dsp_error;
	
	/* get length of string first */
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_WR), 0x0);
	serc_write((unsigned16)(sc_ptr + SC_CNTRL_RD), (int16)(SC_RD_INIT + 0x100));
	cw = sc_handshake(sc_ptr) | element;
	serc_write(sc_ptr, cw);

	if (wait_sc(sc_ptr))
	{	clear_sc(sc_ptr);
		return dsp_error;
	}
	if(check_sc(sc_ptr))
		return dsp_error;
	
	*nbytes = serc_read((unsigned16)(sc_ptr + SC_CNTRL_LEN + SC_WR_LEN));
	clear_sc(sc_ptr);

	return dsp_error;
}