/* GETDATA.C

:Reads a data buffer with the device driver under Windows NT.

This sample demonstrates how to use the internal library function
 get_data_from_driver(...) under Windows NT.  In this sample, the
 configuration word, internal offset, home port offset, home mask,
 and home action for the first three axes is read.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/

#include <windows.h>
#include <winioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "idsp.h"

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define AXES		3


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

int main()
{
	int16 error_code, *data, addr, data_offset, datum_len, pieces_of_data, axis;

	data = (int16*)calloc(AXES, 2*5);	/* allocate a buffer to hold the data */
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	addr = dspPtr->e_data | PCDSP_DM;	/* beginning of config structs */
	data_offset = ED_SIZE;				/* size of config struct */
	datum_len = 5;						/* reading the first 5 words */
	pieces_of_data = AXES;				/* read info for first 3 axes */
	get_data_from_driver(addr, data_offset, datum_len, pieces_of_data, data);

	for(axis = 0; axis < AXES; axis++)
	{	printf("Axis: %d\n", axis);
		printf("Config word: %d\n", data[axis*5]);
		printf("Internal Offset: %d\n", data[axis*5+1]);
		printf("Home Port Offset: %d\n", data[axis*5+2]);
		printf("Home Mask: %d\n", data[axis*5+3]);
		printf("Home Action: %d\n\n", data[axis*5+4]);
	}
	return 0;
}
