/*
	mlframe.c
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
#include <string.h>
#include <stdarg.h>


/*
	Usage: frame_m(&frame, "tva", (double) time, (double) vel, (double) accel);
		where commands are:
			c (unsigned)			control word,
			t (double)				time,
			j (double)				jerk,
			a (double)				acceleration,
			v (double)				velocity,
			x (double)				position,
			u (unsigned)			trigger/update,
			n (unsigned)			trigger action,
			o (unsigned)			output port/value,
			O (unsigned, unsigned, unsigned, unsigned)
									offset, address, ormask, andmask,
			0 (--)					frame_clear
			l (unsigned)			frame_allocate (axis)
			d (--)					frame_download
			- (--)					disable units of measure, through
									this command only.
			<space>					(ignored)
			* (unsigned*)			the allocated address is assigned
			e (unsigned)			set the next pointer.
			P (DSP*)				set the dspPtr.
*/
static int16		frame_in_use = 0;
static FRAME	default_frame ;

int16 C_FN EXPORT frame_m(PFRAME f, char PTRTYPE * c, ...)
{
	va_list args;
	int	i;
	int16 r = DSP_OK, PTRTYPE * pi, disableUOM = FALSE, glob = FALSE, j;
	LFIXED lfixed;
	double dd;
	PDSP oldPtr = dspPtr;
	DSP_FIXER fixer = NULL;

	if (!f)
	{	if (frame_in_use++)
		{	frame_in_use--;
			return (dsp_error = DSP_RESOURCE_IN_USE) ;
		}
		glob = TRUE ;
		f = &default_frame ;
	}

	va_start(args, c);

	for (i = 0; (r == DSP_OK) && c && c[i]; i++)
	{	switch (c[i])
		{	case 'c':
			case 'u':
			case 'n':
			case 'o':
			case 'l':
			case 'e':
				j = (int16)va_arg(args, int);
				r = pcdsp_frame_command(dspPtr, c[i], f, &j);
				break;

			case 'O':
			{	unsigned16 u[4] ;
				u[0] = va_arg(args, unsigned) ;		/* frame offset */
				u[1] = va_arg(args, unsigned) ;		/* address */
				u[2] = va_arg(args, unsigned) ;		/* ormask */
				u[3] = va_arg(args, unsigned) ;		/* andmask */
				r = pcdsp_frame_command(dspPtr, c[i], f, u) ;
			}	break;

			case '*':
				pi = (P_INT)va_arg(args, int PTRTYPE *) ;
				r = pcdsp_frame_command(dspPtr, c[i], f, pi) ;
				break;

			case 't':	fixer = ipcdsp_fixed_time ;
			case 'j':	if (c[i] == 'j')		fixer = ipcdsp_fixed_jerk ;
			case 'a':	if (c[i] == 'a')		fixer = ipcdsp_fixed_accel ;
			case 'v':	if (c[i] == 'v')		fixer = ipcdsp_fixed_vel ;
			case 'x':	if (c[i] == 'x')		fixer = ipcdsp_fixed_pos ;
				dd = va_arg(args, double) ;
				if (disableUOM)	fixer = ipcdsp_fixed_flat ;
				fixer(dspPtr, f->axis, dd, &lfixed) ;
				r = pcdsp_frame_command(dspPtr, c[i], f, &lfixed);
				break;

			case '-':
				disableUOM = TRUE ;
				break;

			case 'P':
				dspPtr = va_arg(args, PDSP);
				break;

#	ifdef	DEBUG
			case 'D':
				pcdsp_dump_frame(f);
				break;
#	endif

			default:
				r = pcdsp_frame_command(dspPtr, c[i], f, NULL) ;
		}
	}

	if (glob)
		frame_in_use-- ;

	dspPtr = oldPtr ;

	return r ;
}
