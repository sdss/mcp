/*
	imemory.c
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

#ifdef MEI_WINNT
#	include <stdarg.h>
#	include <winbase.h>
#	include <winioctl.h>
#  endif


#  ifdef __TURBOC__
#	include <dos.h>
#	endif

#	define	mei_min(a, b)			(((a) < (b))? (a) : (b))


#ifndef MEI_DD

unsigned DATATYPE _pcdsp_timeout = (unsigned)TIMEOUT ;
static unsigned _transfer_in_progress = 0;


unsigned int not_avail1=0;
unsigned int not_avail2=0;
int16 LOCAL_FN pcdsp_transfer_available(PDSP dsp)
{
	unsigned
		timeout = _pcdsp_timeout ;

	while (idsp_read_dm(dsp, (P_DSP_DM)(dsp->transfer + TRANSFER_CTRL)) && timeout)
		timeout -- ;

	if (! timeout)
		dsp_error = DSP_TIMEOUT_ERROR ;

	return dsp_error ;
}


int16 FNTYPE pcdsp_transfer_block(PDSP dsp, int16 read, int16 program, P_DSP_DM addr, unsigned length, PDSP_DM buffer)
{
	int16
		actual_length;

	DSP_DM
		control_word ;

	if (_transfer_in_progress)
		return (dsp_error = DSP_RESOURCE_IN_USE) ;

	_transfer_in_progress = 1;

	while (length)
	{
		/* first, wait for the previous transaction. */
		if (pcdsp_transfer_available(dsp))
		{
			not_avail1++;
			break;
		}

		actual_length = mei_min(TRANSFER_BUFF_SIZE, length);

		idsp_write_dm(dsp, (P_DSP_DM)(dsp->transfer + TRANSFER_ADDR), addr);
		
		if (!read)
			pcdsp_write_dm(dsp, (P_DSP_DM)(dsp->transfer + TRANSFER_BUFFER), actual_length, buffer);

		control_word = XFER_WORDS(actual_length) | XFER_GO ;
		control_word |= program? XFER_PROGRAM : XFER_DATA ;
		control_word |= read? XFER_READ : XFER_WRITE ;
	
		idsp_write_dm(dsp, (P_DSP_DM)(dsp->transfer + TRANSFER_CTRL), control_word);

		if (read)
		{	if (pcdsp_transfer_available(dsp))
			{
				not_avail2++;
				break;
			}

			pcdsp_read_dm(dsp, (P_DSP_DM)(dsp->transfer + TRANSFER_BUFFER), actual_length, buffer);
		}

		buffer += actual_length ;
		length -= actual_length ;
	}

	_transfer_in_progress = 0;
	return dsp_error ;
}

# endif


# ifndef MEI_DD

/*
	pcdsp_read_dm note:

	If the parameter 'len==1' and 'dest==NULL' then the value read
	will be given as the return value.  Otherwise an error code
	is returned.  Note that 'int16' must be at least 16 bits, and watch
	out for sign extensions from systems whose int16's are longer.
*/
int16 FNTYPE pcdsp_read_dm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM dest)
{
	unsigned i;
	DSP_DM r = DSP_OK ;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	for (i = 0; i < len; i++)
	{	CRITICAL ;
		DSP_OUT(dsp->address, (unsigned16)((addr + i) | PCDSP_DM));
		r = DSP_IN(dsp->data) ;
		ENDCRITICAL ;
		if (dest)
			dest[i] = r ;
	}

	if ((len != 1) || (dest))
		r = DSP_OK ;

	return r ;
}
int16 FNTYPE pcdsp_read_pm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM dest)
{
	unsigned i;
	DSP_DM r = DSP_OK ;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	for (i = 0; i < len; i++)
	{	CRITICAL ;
		DSP_OUT(dsp->address, (unsigned16)((addr + i) | PCDSP_PM));
		r = DSP_IN(dsp->data) ;
		ENDCRITICAL ;
		if (dest)
			dest[i] = r ;
	}

	if ((len != 1) || (dest))
		r = DSP_OK ;

	return r ;
}

#	endif

# ifndef MEI_DD

int16 FNTYPE pcdsp_write_dm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM src)
{
	unsigned i;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	for (i = 0; i < len; i++)
	{	CRITICAL ;
		DSP_OUT(dsp->address, (unsigned16)((addr + i) | PCDSP_DM));
		DSP_OUT(dsp->data, src[i]);
		ENDCRITICAL ;
	}

	return DSP_OK ;
}
int16 FNTYPE pcdsp_write_pm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM src)
{
	unsigned i;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	for (i = 0; i < len; i++)
	{	CRITICAL ;
		DSP_OUT(dsp->address, (unsigned16)((addr + i) | PCDSP_PM));
		DSP_OUT(dsp->data, src[i]);
		ENDCRITICAL ;
	}

	return DSP_OK ;
}

# endif

#ifdef MEI_DD

static int16 buffer[128];

#	endif

#ifdef MEI_DD

int16 FNTYPE pcdsp_transfer_block(PDSP dsp,
		int16 read, int16 program, P_DSP_DM addr, unsigned length, PDSP_DM data)
{	unsigned i;
	int16 r;
	buffer[0] = length;
	buffer[1] = addr ;
	buffer[2] = read ;
	if (! read)
	{	for (i = 0; i < length; i++)
			buffer[3 + i] = data[i] ;
	}
	r = dd_call(IOCTL_PCDSP_BLOCK, buffer) ;
	if (read)
	{	for (i = 0; i < length; i++)
			data[i] = buffer[i];
	}
	return r;
}

# endif

#ifdef MEI_DD

int16 FNTYPE pcdsp_read_dm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM dest)
{	unsigned i;
	int16 r;
	buffer[0] = len ;
	buffer[1] = addr | PCDSP_DM ;
	r = dd_call(IOCTL_PCDSP_IN, buffer);
	if (dest)
	{	for (i = 0; i < len; i++)
			dest[i] = buffer[i] ;
		return r;
	}
	return (int16) buffer[0] ;
}
int16 FNTYPE pcdsp_read_pm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM dest)
{	unsigned i;
	int16 r;
	buffer[0] = len ;
	buffer[1] = addr | PCDSP_PM ;
	r = dd_call(IOCTL_PCDSP_IN, buffer);
	if (dest)
	{	for (i = 0; i < len; i++)
			dest[i] = buffer[i] ;
		return r;
	}
	return (int16) buffer[0] ;
}

#	endif

#ifdef MEI_DD

int16 FNTYPE pcdsp_write_dm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM src)
{	unsigned i ;
	buffer[0] = len ;
	buffer[1] = addr | PCDSP_DM ;
	for (i = 0; i < len; i++)
		buffer[2 + i] = src[i] ;
	return dd_call(IOCTL_PCDSP_OUT, buffer) ;
}

int16 FNTYPE pcdsp_write_pm(PDSP dsp, P_DSP_DM addr, unsigned len, PDSP_DM src)
{	unsigned i ;
	buffer[0] = len ;
	buffer[1] = addr | PCDSP_PM ;
	for (i = 0; i < len; i++)
		buffer[2 + i] = src[i] ;
	return dd_call(IOCTL_PCDSP_OUT, buffer) ;
}

# endif


