/*
	llframe.c - frames in DSP memory.
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
#	include <stdarg.h>

/*
	Usage: pcdsp_frame(&frame, commands, ...);
		where commands are:
			c (unsigned)			control word,
			t (PFIXED)				time (samples),
			j (PFIXED)				jerk,
			a (PFIXED)				acceleration,
			v (PFIXED)				velocity,
			x (PFIXED)				position,
			u (unsigned)			trigger/update,
			n (unsigned)			trigger action,
			o (unsigned)			output port/value
			O (unsigned[4])			output offset/address/ormask/andmask
			0 (--)					frame_clear
			l (unsigned)			frame_allocate (axis)
			d (--)					frame_download
			- (--)					disable units of measure, through
									this command only.
			<space>					(ignored)
			* (unsigned PTRTYPE *)	the allocated address is assigned
			e (unsigned)			set the next pointer.
			P (PDSP)				set the dsp pointer.
		#ifdef DEBUG				(options only if the DEBUG symbol is defined)
			D (--)					display the current frame to stdout.
		# endif
*/


int16 FNTYPE pcdsp_frame_command(PDSP pdsp, char command, PFRAME f, void PTRTYPE * parm)
{
	int16 r = DSP_OK;
	P_INT pi = (P_INT) parm;
	PFIXED lfixed = (PFIXED) parm;

	if (! lfixed)
		lfixed = &fixed_zero ;

	switch (command)
	{	case 'c':	f->f.control = (DSP_DM) *pi;	break;
		case 'u':	f->f.trig_update = (DSP_DM) *pi; break;
		case 'n':	f->f.trig_action = (DSP_DM) *pi; break; 
		case 'o':	f->f.output = (DSP_DM) *pi; break ;
		case 'l':	r = (pdsp->FRAME_ALLOCATE)(f, pdsp, *pi);
					if (r == DSP_OK)
						f->f.trig_action = pdsp->frame_action[f->axis] ;
					break; 
		case '*':	*pi = f->current ; break;
		case 'e':	f->f.next = (P_DSP_DM) *pi ;	break ;
		case 't':	f->f.time = (unsigned32) lfixed->whole ; break;
		case 'j':	copy_lfixed_to_fixd(f->f.jerk, *lfixed); break;
		case 'a':	copy_lfixed_to_fixd(f->f.accel, *lfixed); break;
		case 'v':	copy_lfixed_to_fixd(f->f.velocity, *lfixed); break;
		case 'x':	copy_lfixed(f->f.position, *lfixed); break;
		case '0':	frame_clear(f) ; break;
		case 'd':	r = (pdsp->FRAME_DOWNLOAD)(f); break;
#ifndef  MEI_DD
		case 'b':	r = frame_download_and_detach(f); break;
#endif
		case ' ':	break;
		case 'O':
		{	unsigned16 * ufp = (unsigned16*) &(f->f);
			f->f.output = pi[0] ;
			ufp[pi[0]++] = pi[1] ;
			ufp[pi[0]++] = pi[2] ;
			ufp[pi[0]++] = pi[3] ;
		}	break ;
		default:	r = DSP_ILLEGAL_PARAMETER ;
	}

	return r ;
}



int16 C_FN pcdsp_frame(PDSP pdsp, FRAME PTRTYPE * f, char PTRTYPE * c, ...)
{
	va_list args;
	int16 r = DSP_OK, n;
	int i;
	P_INT pi;
	PFIXED lfixed = NULL;

	va_start(args, c);

	for (i = 0; (r == DSP_OK) && c && c[i]; i++)
	{	switch (c[i])
		{	case 'c':
			case 'u':
			case 'n':
			case 'o':
			case 'l':
			case 'e':
				n = (int16)va_arg(args, int);
				r = pcdsp_frame_command(pdsp, c[i], f, &n);
				break;

			case '*':
				pi = (P_INT)va_arg(args, int *);
				r = pcdsp_frame_command(pdsp, c[i], f, pi);
				break;

			case 't':
			case 'j':
			case 'a':
			case 'v':
			case 'x':
				lfixed = va_arg(args, PFIXED);
				r = pcdsp_frame_command(pdsp, c[i], f, lfixed);
				break;

			case 'P':
				pdsp = va_arg(args, PDSP);
				break;

#	ifdef	DEBUG
			case 'D':
				pcdsp_dump_frame(f);
				break;
#		endif

			default:
				r = pcdsp_frame_command(pdsp, c[i], f, NULL) ;
		}
	}

	return r ;
}


/*	read a frame from any address.	*/
int16 FNTYPE pcdsp_read_frame(PDSP pdsp, P_DSP_DM addr, PFRAME frame)
{
	DSP_DM buffer[FRAME_SIZE];
	pcdsp_read_dm(pdsp, addr, FRAME_SIZE, buffer);

	__frame_uncompress(frame, buffer) ;

	frame->dsp = pdsp;
	frame->current = addr ;

	return dsp_error ;
}


/*	write a frame without attaching it to an axis.	*/
int16 FNTYPE pcdsp_write_frame(PDSP pdsp, PFRAME frame)
{
	PDSP_DM dspdm = __frame_compress(frame) ;
	pcdsp_write_dm(pdsp, frame->current, FRAME_SIZE, dspdm) ;
	return dsp_error ;
}



/*	Compute which axis we're connected to.	*/
int16 FNTYPE pcdsp_get_frame(PDSP pdsp, P_DSP_DM addr, PFRAME frame)
{
	P_DSP_DM
		cPointer = 0;
	int16 a;

	if (pcdsp_read_frame(pdsp, addr, frame))
		return dsp_error ;

	/* next, calculate which axis we're connected to. */
	while (addr)
	{	cPointer = addr ;
		addr = idsp_read_dm(pdsp, addr);
	}

	/*	Now, whichever inPtr has cPointer determines which axis we belong to.
		The free list follows just after the inptr's for each axis so
		we actually check one more list then there are axes.
	*/
	frame->axis = -1;
	for (a = 0; (frame->axis < 0) && (a <= pdsp->axes); a++)
	{
		if (a == pdsp->axes)
			addr = pdsp->infree;
		else
			addr = pdsp->outptr + a ;

		if ((unsigned16)idsp_read_dm(pdsp, addr) == cPointer)
			frame->axis = a ;
	}

	if (frame->axis == -1)
		dsp_error = DSP_FRAME_NOT_CONNECTED;

	return dsp_error ;
}


/*
	This ifndef is because some brain-damaged linkers don't
	know how to replace library functions with your own new-and-improved
	routines.  In a particular application, we can't replace this dwell-frame
	routine with a new one; wherefore we must comment it out of this source
	instead.
*/
#	ifndef IM

int16 FNTYPE pcdsp_dwell_frame(PDSP pdsp, int16 axis)
{
	FRAME
		f;
		
	return pcdsp_frame(pdsp, &f, "0lund", axis,
			FUPD_ACCEL | FUPD_VELOCITY | FUPD_JERK, 0) ;
}

#	endif


