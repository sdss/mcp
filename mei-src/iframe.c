/*
	iframe.c -	Low-level PC-DSP frame manipulation routines.
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
#	include <string.h>
#	include <stdarg.h>

#ifdef MEI_WINNT
#	include <stdarg.h>
#	include <winbase.h>
#	include <winioctl.h>
#	endif

#  ifdef MEI_LYNX
#	define	MEI_COMPRESS
#	endif

#  ifdef MEI_OS2
#	define	MEI_COMPRESS
#	endif

#  ifdef MEI_OS2WARP
#	define	MEI_COMPRESS
#	endif

#  ifdef MEI_OS9
#	define	MEI_SWAPLONG
#	endif

#  ifdef MEI_VW
#     ifdef MEI_MC680X0_CPU
#	      define	MEI_SWAPLONG
#     endif
#	endif

#	ifdef MEI_VRTXOS
#		define	MEI_SWAPLONG
#	endif

/*
	Global Variables.
*/

int16 DATATYPE
	frame_wait = DSP_OK;
	
int16 DATATYPE
	_dsp_frames_allocated = 0,
	_dsp_frames_downloaded = 0;

int16 FNTYPE __default_frame_idle(PDSP dsp, int16 axis)
{	dsp = dsp;
	axis = axis;
	return frame_wait;
}

FRAME_IDLE DATATYPE
	frame_idle = __default_frame_idle;


void FNTYPE frame_clear(PFRAME dst)
{
	DSP_DM
		PTRTYPE * base = (DSP_DM PTRTYPE *) dst ;

	int16
		i ;

	for (i = 0; i < (sizeof(FRAME) / sizeof(DSP_DM)); i++)
		base[i] = 0;
}


#ifndef MEI_DD
/*	frame_allocate returns DSP_OK if frame allocation is Ok.	 */
int16 FNTYPE frame_allocate(PFRAME dst, PDSP dsp, int16 axis)
{	DSP_DM	next = 0;
	int16		r = DSP_OK;

	if(pcdsp_sick(dsp, axis))
		return dsp_error;

	/* set up some pointers... */
	dst->dsp = dsp;
	dst->axis = axis;

	dst->f.control |= dsp->frame_control[axis];

	/* if outfree points to 0x0 or the current frame points to 0x0, wait for
		next available frame */
	for(dst->current = idsp_read_dm(dsp, dsp->outfree);
		(!dst->current) || (!idsp_read_dm(dsp, dst->current));
		dst->current = idsp_read_dm(dsp, dsp->outfree))
	{	r = frame_idle(dsp, axis);
		if(r != DSP_OK)
			return (dsp_error = r);
	}

	/* get the address of the next pointer, and make it the head of the list */
	next = idsp_read_dm(dsp, dst->current);
	idsp_write_dm(dsp, dsp->outfree, next);

	/* finally, zero the next pointer in our current frame.	*/
	next = 0;
	idsp_write_dm(dsp, dst->current, next);

	_dsp_frames_allocated++;

	return dsp_error;		/* allocation successful. */
}

#else  /* MEI_DD is defined. */

int16 FNTYPE frame_allocate(PFRAME dst, PDSP dsp, int16 axis)
{
	int16 fa ;
	int16 r = DSP_OK ;

   if(pcdsp_sick(dsp, axis))
      return dsp_error;

	/* set up some pointers... */
	dst->dsp = dsp ;
	dst->axis = axis ;
	dst->f.control |= dsp->frame_control[axis] ;

	for (fa = 0; !fa; )
		dd_call(IOCTL_PCDSP_ALLOCATE, &fa);

	dst->current = fa;
	_dsp_frames_allocated++ ;

	return dsp_error ;		/* allocation successful. */
}
#endif


int16 FNTYPE buffer_allocate(PFRAME dst, PDSP dsp, int16 axis)
{
   if(pcdsp_init_axis_check(dsp, axis))
      return dsp_error;

   if(!dsp->frame_buffer.frames_left)
   {  dsp_error = DSP_NO_EXTERNAL_BUFFER;
      return dsp_error;
   }

	dst->dsp = dsp;
	dst->axis = axis;
	dst->f.control |= dsp->frame_control[axis];

	return dsp_error;		/* allocation successful. */
}

/*
	compress a frame.
*/
#	ifdef	MEI_COMPRESS
static DSP_DM _compress_buffer[FRAME_SIZE] ;
static unsigned16 * _compress_pointer ;

static int16 copy_words(void * f, int16 wrds)
{	int16 i ;
	unsigned16 * ii = (unsigned16 *) f ;
	for (i = 0; i < wrds; i++)
		_compress_pointer[i] = ii[i];
	_compress_pointer += wrds ;
	return wrds ;
}
#	define	CP(field, w)			copy_words(&(field), w)
#		endif

DSP_DM * FNTYPE __frame_compress(PFRAME pframe)
{
#	ifdef	MEI_COMPRESS
	if (pframe->dsp->flags[pframe->axis] & DF_FRAME_COMPRESS)
		return (DSP_DM *) &(pframe->f);
	_compress_pointer = (unsigned16*) _compress_buffer ;
	CP(pframe->f.next, 1);
	CP(pframe->f.control, 1);
	CP(pframe->f.time, 2);
	CP(pframe->f.jerk, 3) ;
	CP(pframe->f.accel, 3) ;
	CP(pframe->f.velocity, 3) ;
	CP(pframe->f.position, 4) ;
	CP(pframe->f.trig_update, 1) ;
	CP(pframe->f.trig_action, 1) ;
	CP(pframe->f.output, 1) ;
	return _compress_buffer ;
#		else
	return (DSP_DM *) &(pframe->f) ;
#		endif
}


#	ifdef	MEI_COMPRESS

static long ll(void)
{	long r = * (int32*) _compress_pointer ;
	_compress_pointer += sizeof(long) / sizeof(int16) ;
	return r;
}
static int16 ii(void)
{	int16 i = *(int16*) _compress_pointer ;
	_compress_pointer ++ ;
	return i;
}

#	endif


int16 FNTYPE __frame_uncompress(PFRAME pframe, PDSP_DM pdsp_dm)
{
#	ifdef	MEI_COMPRESS

	_compress_pointer = pdsp_dm ;
	pframe->f.next = ii() ;
	pframe->f.control = ii();
	pframe->f.time = ll();
	pframe->f.jerk.frac = ll() ;
	pframe->f.jerk.whole = ii() ;
	pframe->f.accel.frac = ll();
	pframe->f.accel.whole = ii() ;
	pframe->f.velocity.frac = ll() ;
	pframe->f.velocity.whole = ii() ;
	pframe->f.position.frac = ll();
	pframe->f.position.whole = ll() ;
	pframe->f.trig_update = ii() ;
	pframe->f.trig_action = ii() ;
	pframe->f.output = ii() ;

#		else
	memcpy(&(pframe->f), pdsp_dm, FRAME_SIZE * sizeof(int16)) ;
#		endif

	return 0 ;
}

/*
	KLUDGE.
*/
#ifdef MEI_SWAPLONG
int16 mei_swaplong(unsigned32 * p)
{	int16 * i = (int16*) p ;
	int16 temp ;
	temp = i[0] ;
	i[0] = i[1] ;
	i[1] = temp ;
	return 0;
}

int16 mei_swapfixed(LFIXED * p)
{	mei_swaplong((unsigned32*) &(p->whole));
	mei_swaplong((unsigned32*) &(p->frac));
	return 0;
}

int16 frame_swap(PFRAME p)
{	mei_swaplong(&(p->f.time));
	mei_swaplong(&(p->f.jerk.frac));
	mei_swaplong(&(p->f.accel.frac));
	mei_swaplong(&(p->f.velocity.frac));
	mei_swapfixed(&(p->f.position));
	return 0;
}
# endif

#ifndef MEI_DD
/*
	returns dsp_error != DSP_OK if unable to download the frame.
*/
int16 FNTYPE _frame_download(PFRAME pframe, int16 connect)
{
	P_DSP_DM
		inptr		= 0;

	DSP_DM
		* src = __frame_compress(pframe) ;

	/* first, make sure we have a DSP pointer. */
	if (! pframe->dsp)
		return (dsp_error = DSP_FRAME_UNALLOCATED);

	/* finally, make sure that we're using an allocated frame. */
	if (! pframe->current)
		return (dsp_error = DSP_FRAME_UNALLOCATED);

#ifdef MEI_SWAPLONG
	if (! (pframe->dsp->flags[pframe->axis] & DF_FRAME_COMPRESS))
	{	frame_swap(pframe);
	}
# endif

	inptr = pframe->dsp->inptr + pframe->axis ;

	/* Load the frame int16o external data memory. */
	pcdsp_write_dm(pframe->dsp, pframe->current, FRAME_SIZE, src) ;
	/* Set the last frame currently in the list to point to this one. */
   if(connect)
   	idsp_write_dm(pframe->dsp, idsp_read_dm(pframe->dsp, inptr),
	   						pframe->current) ;
	/* Set INPTR to point to this new frame. */
	idsp_write_dm(pframe->dsp, inptr, pframe->current) ;

	_dsp_frames_downloaded++;
	inc_frame_count(pframe->axis);

	return dsp_error ;
}

int16 FNTYPE frame_download(PFRAME pframe)
{  if(pcdsp_sick(pframe->dsp, pframe->axis))
      return dsp_error;
   return   _frame_download(pframe, TRUE);
}

int16 FNTYPE frame_download_and_detach(PFRAME pframe)
{  if(pcdsp_sick(pframe->dsp, pframe->axis))
      return dsp_error;
   return   _frame_download(pframe, FALSE);
}

#else		/* MEI_DD is defined. */

int16 FNTYPE frame_download(PFRAME pframe)
{
	DSP_DM * src = __frame_compress(pframe) ;
	int16 buffer[30], i, * p ;
	if (! pframe->dsp) return (dsp_error = DSP_FRAME_UNALLOCATED);
	if (! pframe->current) return (dsp_error = DSP_FRAME_UNALLOCATED);

	buffer[0] = pframe->current ;
	buffer[1] = pframe->axis ;
	p = (int16*) &(pframe->f);
	for (i = 0; i < 20; i++)
		buffer[2 + i] = p[i];

	dd_call(IOCTL_PCDSP_DOWNLOAD, buffer);

	_dsp_frames_downloaded++;
	inc_frame_count(pframe->axis);

	return dsp_error ;
}
#endif

int16 FNTYPE buffer_download(PFRAME pframe)
{
   if(dspPtr == NULL)
   {  dsp_error = DSP_NOT_INITIALIZED;
      return dsp_error;
   }

   *(dspPtr->frame_buffer.pbuffer + dspPtr->frame_buffer.frames) = *pframe;
   dspPtr->frame_buffer.frames += 1;
   dspPtr->frame_buffer.frames_left -= 1;
   return dsp_error;
}

int16 FNTYPE pcdsp_set_gate(PDSP pdsp, int16 andgate, int16 orgate)
{
	DSP_DM gate = idsp_read_dm(pdsp, (P_DSP_DM)(pdsp->global_data + GD_GATE));

	gate &= andgate ;
	gate |= orgate ;

	return idsp_write_dm(pdsp, (P_DSP_DM)(pdsp->global_data + GD_GATE), gate);
}


int16 FNTYPE dsp_set_gate(PDSP pdsp, int16 axis)
{
	if (pcdsp_sick(pdsp, axis))
		return dsp_error ;

	if (! pdsp->laxis[axis].gate)
		pcdsp_set_gate(pdsp, -1, (int16)(1 << axis));
	pdsp->laxis[axis].gate++ ;

	return DSP_OK ;
}

int16 FNTYPE dsp_reset_gate(PDSP pdsp, int16 axis)
{
	if (pcdsp_sick(pdsp, axis))
		return dsp_error ;

	if (pdsp->laxis[axis].gate)
		pdsp->laxis[axis].gate--;

	if (! pdsp->laxis[axis].gate)
		pcdsp_set_gate(pdsp, (int16)(~(1 << axis)), 0);

	pcdsp_set_event(pdsp, axis, NO_EVENT) ;

	return DSP_OK ;
}

int16 FNTYPE dsp_clear_gate(PDSP pdsp, int16 axis)
{	pdsp->laxis[axis].gate = 0;
	return dsp_reset_gate(pdsp, axis);
}


