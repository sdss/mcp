/*
	mboard.c - Multiple Controller in one computer support code.
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
#	include "mboard.h"
#ifdef ALLOC
#  include <alloc.h>
#else
#  include <stdlib.h>
#endif

typedef struct
{	DSP		dsp ;
	int16	axes ;
} MBOARD ;


MBOARD	*	mboard = NULL ;
int16	mboards = 0,
		offendingBoard = 0,
		totalAxes = 0;


int16 FNTYPE m_board(int16 boardno)
{	dspPtr = NULL;

	if((boardno >= 0) && (boardno < mboards))
		dspPtr = &(mboard[boardno].dsp);

	return dspPtr == NULL;
}


int16 FNTYPE m_axis(int16 actual_axis)
{
	int16	board = 0;

	for(board = 0; (board < mboards) && (actual_axis >= mboard[board].axes);
		board++)
	{
		actual_axis -= mboard[board].axes;
	}

	if(board < mboards)				/* if this fails, the the later */
	{	dspPtr = &mboard[board].dsp;	/* routines will fail due to the */
		return actual_axis ;			/* invalid axis number. */
	}

	dsp_error = DSP_INVALID_AXIS ;
	return -1;
}

#ifdef MEI_WINNT
void __cdecl free_board_structs(void)
#else
void free_board_structs(void)
#endif
{	if(mboard != NULL)
	{	free(mboard);
		mboard = NULL;
	}
}


int16 FNTYPE m_setup(int16 len, int16 * baseAddresses)
{	int16	e = DSP_OK;

	free_board_structs();
	totalAxes = 0;
	mboards = len;
	mboard = (MBOARD*) calloc(len, sizeof(MBOARD));

#ifndef MEI_VRTXOS 
	if(mboard != NULL)
		atexit(free_board_structs);
#endif  /* MEI_VRTXOS */

	if(!mboard)
	{	mboards = 0;
		return MBOARD_OOM;
	}

	for(offendingBoard = 0;
		(e == DSP_OK) && (offendingBoard < mboards);
		offendingBoard++)
	{	dspPtr = &(mboard[offendingBoard].dsp);
		e = pcdsp_init(dspPtr, baseAddresses[offendingBoard]);
		if (!e)
		{	mboard[offendingBoard].axes = dsp_axes();
			totalAxes += dsp_axes();
		}
	}

	if(e)
		offendingBoard--;
	return e ;
}


