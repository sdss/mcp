/*
	mlef.c
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

static void LOCAL_FN set_rate(PDSP pdsp, int16 axis, int16 neg, int16 pos, double rate)
{	FRAME pos_frame, neg_frame ;
	LFIXED lpos, lneg ;

	if (rate < 0)
		rate = -rate ;

	pcdsp_read_ef(pdsp, &pos_frame, axis, pos) ;
	pcdsp_read_ef(pdsp, &neg_frame, axis, neg) ;

	ipcdsp_fixed_accel(pdsp, axis, rate, &lneg);
	if (lneg.whole & 0xFFFF8000L)
	{	lneg.whole = 0x7FFF ;
		lneg.frac = 0xFFFFFFFFL ;
		lpos.whole = 0x8000;
		lpos.frac = 0;
	}
	else
		ipcdsp_fixed_accel(pdsp, axis, -rate, &lpos);

#ifdef MEI_SWAPLONG
	mei_swapfixed(&lpos);
	mei_swapfixed(&lneg);
#endif

	copy_lfixed_to_fixd(pos_frame.f.accel, lpos) ;
	copy_lfixed_to_fixd(neg_frame.f.accel, lneg) ;

	pcdsp_write_ef(pdsp, &pos_frame) ;
	pcdsp_write_ef(pdsp, &neg_frame) ;
}



int16 FNTYPE set_stop_rate(int16 axis, double rate)
{
	if (rate < 0)
		rate = -rate ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	set_rate(dspPtr, axis, EF_NEG_STOP, EF_POS_STOP, rate);
	return DSP_OK ;
}


int16 FNTYPE set_e_stop_rate(int16 axis, double rate)
{	if (rate < 0)
		rate = -rate ;

	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	set_rate(dspPtr, axis, EF_NEG_E_STOP, EF_POS_E_STOP, rate) ;
	return DSP_OK ;
}



static double LOCAL_FN get_rate(PDSP pdsp, int16 axis, int16 neg, int16 pos)
{	FRAME pos_frame, neg_frame ;
	double pos_rate, neg_rate ;
	LFIXED lpos, lneg ;

	pcdsp_read_ef(pdsp, &pos_frame, axis, pos) ;
	pcdsp_read_ef(pdsp, &neg_frame, axis, neg) ;

	copy_lfixed(lpos, pos_frame.f.accel) ;
	copy_lfixed(lneg, neg_frame.f.accel) ;

#ifdef MEI_SWAPLONG
	mei_swapfixed(&lpos);
	mei_swapfixed(&lneg);
#endif

	ipcdsp_double_accel(pdsp, axis, &lpos, &pos_rate) ;
	ipcdsp_double_accel(pdsp, axis, &lneg, &neg_rate) ;
	neg_rate = -neg_rate ;

	if (pos_rate > neg_rate)
		return pos_rate ;
	else
		return neg_rate ;
}


int16 FNTYPE get_stop_rate(int16 axis, double PTRTYPE * rate)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	*rate = get_rate(dspPtr, axis, EF_NEG_STOP, EF_POS_STOP) ;
	return DSP_OK ;
}


int16 FNTYPE get_e_stop_rate(int16 axis, double PTRTYPE * rate)
{
	if (pcdsp_sick(dspPtr, axis))
		return dsp_error ;

	*rate = get_rate(dspPtr, axis, EF_NEG_E_STOP, EF_POS_E_STOP);
	return DSP_OK ;
}


int16 FNTYPE pcdsp_config_ef(PDSP dsp, int16 axis, int16 ef, int16 set_int)
{
	FRAME frame ;

	if (pcdsp_read_ef(dsp, &frame, axis, ef) )
		return dsp_error ;

	if (set_int)	frame.f.control |= FCTL_INTERRUPT ;
	else			frame.f.control &= ~FCTL_INTERRUPT ;

	pcdsp_write_ef(dsp, &frame);
	return dsp_error ;
}

int16 FNTYPE interrupt_on_event(int16 axis, int16 state)
{  return pcdsp_config_ef(dspPtr, axis, EF_STOPPED, state);
}




