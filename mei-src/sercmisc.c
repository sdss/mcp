/*
   SERCMISC.C -  
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "sercrset.h"

int16 FNTYPE get_phase2_idnlists(int16 * size, unsigned16 * list)
{	int16 axis, i, a = 0;

	for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	 size[axis] = phase2_idncount[axis];
		 for(i = 0; i < size[axis]; i++)
		 {	list[a] = phase2_idnlist[axis][i];
		 	a++;
		 }
	}
	return DSP_OK;
}

int16 FNTYPE get_phase3_idnlists(int16 * size, unsigned16 * list)
{	int16 axis, i, a = 0;

	for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	 size[axis] = phase3_idncount[axis];
		 for(i = 0; i < size[axis]; i++)
		 {	list[a] = phase3_idnlist[axis][i];
		 	a++;
		 }
	}
	return DSP_OK;
}
