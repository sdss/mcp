/*
   bor45asm.c - Assembly code for borland 4.5 flat memory model.
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

unsigned16 FNTYPE DSP_IN(int16 pp)
{
	unsigned16   a;
	asm{
		movsx edx, pp
		in ax, dx
		mov a, ax
	}
	return a;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{
	asm{
		movsx edx, pp
		mov ax, vv
		out dx, ax
	}
}

unsigned char FNTYPE DSP_INB(int16 pp)
{
	unsigned char  a;
	asm{
		movsx edx, pp
		in al, dx
		mov a, al
	}
	return a;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{
	asm{
		movsx edx, pp
		mov al, vv
		out dx, al
	}
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{  return pcdsp_set_address(pdsp, iobase);
}

