/*
	lynxasm.c -	Assembly language LynxOS functions - compile with LynxOS cc
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

typedef char			int8 ;
typedef unsigned char	unsigned8 ;
typedef short			int16 ;
typedef unsigned short	unsigned16 ;
typedef long			int24 ;
typedef unsigned long	unsigned24 ;
typedef long			int32 ;
typedef unsigned long	unsigned32 ;



unsigned16 DSP_IN(pp)
int16	pp;
{
	asm
	{	xor	eax, eax
		mov	dx, pp[ebp]
		in	ax, dx
	}

	/* return value in EAX */
}

void DSP_OUT(pp, vv)
int16	pp;
unsigned16	vv;
{
	asm
	{ 	mov	dx, pp[ebp]
		mov	ax, vv[ebp]
		out	dx, ax
	}
}

unsigned char DSP_INB(pp)
int16	pp;
{
	asm
	{	xor	eax, eax
		mov	dx, pp[ebp]
		in    al, dx
	}
	/* return value in EAX */
}

void DSP_OUTB(pp, vv)
int16	pp;
unsigned char	vv;
{
	asm
	{	mov	dx, pp[ebp]
		mov	al, vv[ebp]
		out	dx, al
	}
}

