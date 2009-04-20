/*
	llpos.c - low-level axis feedback
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#       include "idsp.h"
#       include <math.h>


/*      These are bit-fields for the 'type' parameter below. */
#       define  T_WFIXED                4
#       define  T_FIXED                 2
#       define  T_SFIXED                1
#       define  T_LFIXED                0


static int16 LOCAL_FN ipcdsp_get_lfixed(PDSP pdsp, P_DSP_DM pdm, PFIXED pfixed, int16 type)
{
	P_DSP_DM
		addr = pdm ;
	int16
		i ;

	if (type & T_SFIXED)
		addr -- ;
	if (type & T_WFIXED)
		addr -= 2 ;

	pcdsp_transfer_block(pdsp, TRUE, FALSE, addr, LFIXED_SIZE, (DSP_DM PTRTYPE *) pfixed) ;
	if (dsp_error)
		return dsp_error ;

# ifdef MEI_O_BENDIAN
	{       /* kludge. */
		DSP_DM PTRTYPE * p = (DSP_DM PTRTYPE *) pfixed ;
		DSP_DM temp ;
		temp = p[0] ;
		p[0] = p[1] ;
		p[1] = temp ;
		temp = p[2] ;
		p[2] = p[3] ;
		p[3] = temp ;
	}
#  endif

	/* trim the low 16 bits of frac. */
	if (type & T_SFIXED)
		pfixed->frac &= (unsigned long) 0xFFFF0000L ;

	if (type & T_WFIXED)
		pfixed->frac = 0L;

	/* cut the upper 16 bits of whole. */
	if (type & T_FIXED)
	{       i = (int16) pfixed->whole ;
		pfixed->whole = (int32) i ;
	}

	return dsp_error ;
}


static int16 LOCAL_FN ipcdsp_set_lfixed(PDSP pdsp, P_DSP_DM pdm, PFIXED pfixed, int16 type)
{       int16
		len = LFIXED_SIZE ;
	DSP_DM
		* dm = (P_DSP_DM *) pfixed ;

# ifdef MEI_O_BENDIAN
	/* Kludge. */
	DSP_DM temp ;
	temp = dm[0] ;
	dm[0] = dm[1] ;
	dm[1] = temp ;
	temp = dm[2] ;
	dm[2] = dm[3] ;
	dm[3] = temp ;
#  endif

	if (type & T_WFIXED)
	{       len-=2 ;
		dm+=2;          /* skip the first 2 words. */
	}
	if (type & T_SFIXED)
	{       len-- ;
		dm++;           /* skip the first word. */
	}

	if (type & T_FIXED)
		len--;          /* skip the last word. */

	return pcdsp_transfer_block(pdsp, FALSE, FALSE, pdm, len, dm);
}



#       define  DATA_STRUCT(dsp, axis, offset)          (P_DSP_DM)((dsp)->data_struct + (DS_SIZE * (axis)) + offset)


/*      gets the current commanded position int16o dst. */
int16 FNTYPE pcdsp_command(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_POSITION), dst, T_LFIXED) ;
}


/*      sets the current command position. */
int16 FNTYPE pcdsp_set_command(PDSP pdsp, int16 axis, PFIXED src)
{       return ipcdsp_set_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_POSITION), src, T_LFIXED) ;
}

/*      gets the current encoder position int16o dst. */
int16 FNTYPE pcdsp_actual(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_ACTUAL_POSITION), dst, T_WFIXED) ;
}

/*      sets the current actual position register. */
int16 FNTYPE pcdsp_set_actual(PDSP pdsp, int16 axis, PFIXED src)
{       return ipcdsp_set_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_ACTUAL_POSITION), src, T_WFIXED) ;
}

/*      sets both the actual and command positions. */
int16 FNTYPE pcdsp_set_position(PDSP pdsp, int16 axis, PFIXED pos)
{       FRAME    frame;
   pcdsp_frame(pdsp, &frame, "0l x u n c d",
			axis, pos, FUPD_POSITION | FUPD_ACTUAL, 0,
			(pdsp->frame_control[axis] & 0xFF00) | 0x0F);
   return pcdsp_set_event(pdsp, axis, NO_EVENT);
}

int16 FNTYPE pcdsp_get_velocity(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_VELOCITY), dst, T_FIXED) ;
}

int16 FNTYPE pcdsp_set_vel(PDSP pdsp, int16 axis, PFIXED src)
{       return ipcdsp_set_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_VELOCITY), src, T_FIXED) ;
}

int16 FNTYPE pcdsp_get_acceleration(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_ACCELERATION), dst, T_FIXED) ;
}

int16 FNTYPE pcdsp_set_accel(PDSP pdsp, int16 axis, PFIXED src)
{       return ipcdsp_set_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_ACCELERATION), src, T_FIXED) ;
}

int16 FNTYPE pcdsp_get_jerk(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_JERK), dst, T_FIXED) ;
}

int16 FNTYPE pcdsp_set_jerk(PDSP pdsp, int16 axis, PFIXED src)
{       return ipcdsp_set_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_JERK), src, T_FIXED) ;
}

int16 FNTYPE pcdsp_get_error(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_ERROR), dst, T_WFIXED | T_FIXED) ;
}

/*      gets the current latched position int16o dst. */
int16 FNTYPE pcdsp_latched(PDSP pdsp, int16 axis, PFIXED dst)
{       return ipcdsp_get_lfixed(pdsp, DATA_STRUCT(pdsp, axis, DS_LATCH), dst, T_WFIXED) ;
}

