/*
	llconfig.c
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

typedef struct
{	P_DSP_DM	offset ;
	CONFIGTYPES	type ;
} CONFIGDATA ;

static CONFIGDATA configdata [CONFIGURABLE_LIMITS] = {
	{ ED_CONFIG_DATA, Mask },
	{ ED_INTERNAL_OFFSET, Mask },
	{ ED_HOME_PORT_OFFSET, Mask },
	{ ED_HOME_MASK, MaskAction },
	{ ED_AMP_FAULT_MASK, MaskAction },
	{ ED_POS_LIMIT_MASK, MaskAction },
	{ ED_NEG_LIMIT_MASK, MaskAction },
	{ ED_X_NEG, WholeAction },
	{ ED_X_POS, WholeAction },
	{ ED_ERROR_LIMIT, ValueAction },
	{ ED_IN_POS_LIMIT, ValueAction },
	{ ED_STATUS_PORT, Mask },
	{ ED_AMP_ENABLE_MASK, Mask },
	{ ED_IN_POS_MASK, Mask },
	{ ED_ANALOG_CHANNEL, Mask },
	{ ED_DAC_CHANNEL, Mask },
	{ ED_AUX_ENCODER, Mask },
			} ;


P_DSP_DM FNTYPE pcdsp_address(PDSP pdsp, int16 axis, int16 parameter, CONFIGTYPES * type)
{	if((parameter < 0) || (parameter >= CONFIGURABLE_LIMITS))
	{	dsp_error = DSP_ILLEGAL_PARAMETER;
		return 0;
	}

	*type = configdata[parameter].type;
	return (pdsp->e_data + configdata[parameter].offset + ED(axis));
}


static P_DSP_DM LOCAL_FN figure_boot_address(PDSP pdsp, int16 axis, int16 parameter, CONFIGTYPES * type)
{	if ((parameter < 0) || (parameter >= CONFIGURABLE_LIMITS))
	{	dsp_error = DSP_ILLEGAL_PARAMETER ;
		return 0;
	}

	*type = configdata[parameter].type ;
	return (idsp_read_dm(pdsp, PB_CONFIG_STRUCT) + configdata[parameter].offset + ED(axis)) ;
}


#	define	WHOLE_WORDS			(sizeof(pfixed->whole) / sizeof(DSP_DM))


int16 FNTYPE pcdsp_set_config_struct(PDSP pdsp, int16 axis, int16 parameter, PFIXED pfixed, PDSP_DM paction)
{
	CONFIGTYPES	type ;
	P_DSP_DM	address = pcdsp_address(pdsp, axis, parameter, &type) ;
	PDSP_DM		destination = (PDSP_DM) (pfixed) ;

#ifdef MEI_SWAPLONG
	if (pfixed)
	{
		mei_swapfixed (pfixed);
	}
#endif /* MEI_SWAPLONG */

	if (address)
	{
		switch (type)
		{
			case MaskAction:		/* pfixed really points to a DSP_DM Mask. */
				if (pfixed)
					pcdsp_write_dm(pdsp, address, 1, destination) ;
				if (paction)
					pcdsp_write_dm(pdsp, ++address, 1, paction) ;
				break;

			case ValueAction:
				if (pfixed)
					pcdsp_write_dm(pdsp, address, 1, &(destination[2])) ;
				if (paction)
				{	address += 1;
					pcdsp_write_dm(pdsp, address, 1, paction) ;
				}
				break;

			case Mask:
				pcdsp_write_dm(pdsp, address, 1, paction) ;
				break;

			case WholeAction:
				if (pfixed)
					pcdsp_write_dm(pdsp, address, WHOLE_WORDS, &(destination[2])) ;
				if (paction)
				{	address += WHOLE_WORDS ;
					pcdsp_write_dm(pdsp, address, 1, paction) ;
				}
				break;
		}
	}

	return dsp_error ;
}


int16 FNTYPE pcdsp_get_config_struct(PDSP pdsp, int16 axis, int16 parameter, PFIXED pfixed, PDSP_DM paction)
{
	CONFIGTYPES	type ;
	P_DSP_DM	address = pcdsp_address(pdsp, axis, parameter, &type) ;

	if (address)
	{
		switch (type)
		{
			case MaskAction:		/* pfixed really points to a DSP_DM Mask. */
				if (pfixed)
					pcdsp_read_dm(pdsp, address, 1, (PDSP_DM) pfixed);
				if (paction)
					pcdsp_read_dm(pdsp, ++address, 1, paction);
				break;

			case WholeAction:
				if (pfixed)
				{	pcdsp_read_dm(pdsp, address, sizeof(pfixed->whole) / sizeof(DSP_DM), (PDSP_DM) &(pfixed->whole));
					pfixed->frac = 0;
				}
				if (paction)
				{	address += WHOLE_WORDS ;
					pcdsp_read_dm(pdsp, address, 1, paction) ;
				}
				break;

			case Mask:
				pcdsp_read_dm(pdsp, address, 1, paction) ;
				break;

			case ValueAction:
				if (pfixed)
				{	PDSP_DM destination = (PDSP_DM) pfixed ;
					pfixed->whole = 0;
					pfixed->frac = 0;
					pcdsp_read_dm(pdsp, address, 1, &(destination[2]));
				}
				if (paction)
				{	address += 1;
					pcdsp_read_dm(pdsp, address, 1, paction) ;
				}
				break;

		}
	}

#ifdef MEI_SWAPLONG
	if (pfixed)
	{
		mei_swapfixed(pfixed);
	}
#endif /* MEI_SWAPLONG */

	return dsp_error ;
}



int16 FNTYPE pcdsp_set_boot_config_struct(PDSP pdsp, int16 axis, int16 parameter, PFIXED pfixed, PDSP_DM paction)
{
	CONFIGTYPES	type ;
	P_DSP_DM	address = figure_boot_address(pdsp, axis, parameter, &type) ;
	PDSP_DM		destination = (PDSP_DM) pfixed ;

#ifdef MEI_SWAPLONG
	if (pfixed)
	{
		mei_swapfixed (pfixed);
	}
#endif /* MEI_SWAPLONG */

	if (address)
	{
		switch (type)
		{
			case MaskAction:		/* pfixed really points to a DSP_DM Mask. */
				if (pfixed)
					pcdsp_write_bm(pdsp, address, 1, destination) ;
				if (paction)
					pcdsp_write_bm(pdsp, ++address, 1, paction) ;
				break;

			case ValueAction:
				if (pfixed)
					pcdsp_write_bm(pdsp, address, 1 , (PDSP_DM) &(destination[2])) ;
				if (paction)
				{	address += 1;
					pcdsp_write_bm(pdsp, address, 1, paction) ;
				}
				break;

			case Mask:
				pcdsp_write_bm(pdsp, address, 1, paction) ;
				break;

			case WholeAction:
				if (pfixed)
					pcdsp_write_bm(pdsp, address, WHOLE_WORDS, &(destination[2])) ;
				if (paction)
				{	address += WHOLE_WORDS ;
					pcdsp_write_bm(pdsp, address, 1, paction) ;
				}
				break;
		}
	}

	return dsp_error ;
}


int16 FNTYPE pcdsp_get_boot_config_struct(PDSP pdsp, int16 axis, int16 parameter, PFIXED pfixed, PDSP_DM paction)
{
	CONFIGTYPES	type ;
	P_DSP_DM	address = figure_boot_address(pdsp, axis, parameter, &type) ;
	PDSP_DM		destination = (PDSP_DM) pfixed ;

	if (pfixed)
	{	pfixed->whole = 0;
		pfixed->frac = 0;
	}

	if (address)
	{
		switch (type)
		{
			case MaskAction:		/* pfixed really points to a DSP_DM Mask. */
				if (pfixed)
					pcdsp_read_bm(pdsp, address, 1, destination);
				if (paction)
					pcdsp_read_bm(pdsp, ++address, 1, paction);
				break;

			case ValueAction:
				if (pfixed)
                {
					pfixed->whole = 0;
					pfixed->frac = 0;
					pcdsp_read_bm(pdsp, address, 1, (PDSP_DM) &(destination[2]));
                }
				if (pfixed)
				{	address += 1;
					pcdsp_read_bm(pdsp, address, 1, paction) ;
				}
				break;

			case Mask:
				pcdsp_read_bm(pdsp, address, 1, paction) ;
				break;

			case WholeAction:
				if (pfixed)
					pcdsp_read_bm(pdsp, address, WHOLE_WORDS, (PDSP_DM) &(destination[2]));
				if (pfixed)
				{	address += WHOLE_WORDS ;
					pcdsp_read_bm(pdsp, address, 1, paction) ;
				}
				break;

		}
	}

#ifdef MEI_SWAPLONG
	if (pfixed)
	{
		mei_swapfixed(pfixed);
	}
#endif /* MEI_SWAPLONG */

	return dsp_error ;
}
