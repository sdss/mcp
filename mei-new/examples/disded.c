/* DISDED.C

:Disable the DSP from reading/writing to the dedicated output bits.

Each axis has 2 dedicated output bits (in-position and amplifier enable).  The
 dedicated output bits for axes 0-3 are located on I/O port 8 and the dedicated
 output bits for axes 4-7 are located on I/O port 5.
 
Normally, the DSP reads each dedicated output port (once per axis), masks the
 8 bits, and writes to each dedicated output port.  When disabling the dedicated
 output bits, be sure to disable the axes in groups of four (0-3 and/or 4-7).

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/


	
# include <stdio.h>
# include <stdlib.h>
# include "pcdsp.h"
# include "idsp.h"


void error(int16 error_code)
{   
	char buffer[MAX_ERROR_LEN];

	switch (error_code)
	{
		case DSP_OK:
			/* No error, so we can ignore it. */
			break ;

		default:
			error_msg(error_code, buffer) ;
			fprintf(stderr, "ERROR: %s (%d).\n", buffer, error_code) ;
			exit(1);
			break;
	}
}


int16	main()
{
	int16 error_code, axis, axes, ded_addr;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	axes = dsp_axes();

	for (axis = 0; axis < axes; axis++)
	{
		ded_addr = dspPtr->e_data + ED_STATUS_PORT + DS(axis);
		dsp_write_dm(ded_addr, 0x0);
	}
	
    return 0;
}
