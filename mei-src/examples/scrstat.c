/* SCRSTAT.C

:SERCOS communication ring status.

This sample demonstrates how to determine the SERCOS communication ring
 status.  The SERCON 410B has two status flags which monitor the data and
 the ring closure.

The RDIST bit is low when data is valid and high when the data is distorted.

The FIBBR bit is low when the loop is closed and high when the loop is open.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5  
*/

	
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"

#define RING_STATUS_ADDR	0xC02	/* SERCON 410B Register */

#define RDIST				0x1000	/* distorted data bit */
#define FIBBR				0x2000	/* loop open bit */


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

int	main()
{
	int16 error_code, value;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	while (!kbhit())
	{	value = dsp_read_pm(RING_STATUS_ADDR);
		printf("\rDistort:%d Loop Open:%d", 
			(value & RDIST) ? 1:0, (value & FIBBR) ? 1:0);
	}	
	getch();

    return 0;
}
