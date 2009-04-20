/*
	extras.c - Special functions.
		get_tuner_data - for the Visual Basic Tuner.
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
#	include "extras.h"

#	define	DATA_STRUCT(dsp, axis, offset)		(P_DSP_DM)((dsp)->data_struct + (DS_SIZE * (axis)) + offset)

#define DATA_LEN (DS_D(8) - DS_ACTUAL_POSITION + 4)
#define X_OFFSET (DS_POSITION - DS_ACTUAL_POSITION)
#define V_OFFSET (DS_VELOCITY - DS_ACTUAL_POSITION)
#define A_OFFSET (DS_ACCELERATION - DS_ACTUAL_POSITION)
#	define	VOLTAGE (DS_D(8) - DS_ACTUAL_POSITION)


int16 FNTYPE get_tuner_data(int16 axis, int16 data_length, long PTRTYPE * apos,  long PTRTYPE * cpos, P_INT time, P_INT state,
	P_INT voltage)
{
    DSP_DM data_array[DATA_LEN];

    long *l;
    int16 i;
    for(i = 0; i < data_length; i++)
    {
        int16 a,v,s;

	    pcdsp_transfer_block(dspPtr, TRUE, FALSE, DATA_STRUCT(dspPtr, axis, DS_ACTUAL_POSITION) , DATA_LEN, data_array) ;

	    if (dsp_error)
        {
		    return dsp_error ;
        }

        time[i] = dsp_read_dm(0x11E);

        l = (long *) &data_array[0];
        apos[i] = *l;

        l = (long *) &data_array[X_OFFSET + 2];
        cpos[i] = *l;

        voltage[i] = data_array[VOLTAGE] ;

        if (state)
		{	s = 0;
			a = data_array[A_OFFSET] | data_array[A_OFFSET + 1] | data_array[A_OFFSET + 2];
			if(a)
			{
				if (data_array[A_OFFSET + 2] & 0x8000)
				{
					s = 3;
				}
				else
				{
					s = 1;
				}
			}
			else
			{
				v = data_array[V_OFFSET] | data_array[V_OFFSET + 1] | data_array[V_OFFSET + 2];
				if(v)
				{
					s = 2;
				}
			}
			state[i] = s;
		}
    }
    return 0;
}

int16 FNTYPE get_coord_data(int16 axis, int16 data_length, long PTRTYPE * x_p,  long PTRTYPE * y_p, P_INT time)
{
    DSP_DM data_array[DS_SIZE+2];

    long *l;
    int16 i;
    for(i = 0; i < data_length; i++)
    {
	    pcdsp_transfer_block(dspPtr, TRUE, FALSE, DATA_STRUCT(dspPtr, axis, DS_POSITION+1) , DS_SIZE+2, data_array) ;

	    if (dsp_error)
        {
		    return dsp_error ;
        }

        time[i] = dsp_read_dm(0x11E);

        l = (long *) &data_array[0];
        x_p[i] = *l;

        l = (long *) &data_array[DS_SIZE];
        y_p[i] = *l;

    }
    return 0;
}

