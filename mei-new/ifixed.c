/*
	ifixed.c
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


void __add_lfixed(PFIXED dst, PFIXED src)
{
	unsigned32 oldfrac = dst->frac ;
	dst->whole += src->whole ;
	dst->frac += src->frac ;
	if (dst->frac < oldfrac)
		dst->whole++ ;
}

void __sub_lfixed(PFIXED dst, PFIXED src)
{	LFIXED
		argh ;

	copy_lfixed(argh, *src);
	neg_lfixed(argh);
	__add_lfixed(dst, &argh);
}
