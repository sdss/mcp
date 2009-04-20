/* SERCPL2.C

:Position Latching with a SERCOS drive.

This sample demonstrates how to configure a SERCOS drive to latch the
 position based on two probe inputs.  Probe 1 and 2 are configured to trigger
 on the rising edge.  The probe status is mapped into the Drive Status
 word.  The real time status word is also copied to external memory by the
 DSP every sample.

Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before configuring the drive for position latching.
  
For more information on Probe based Position Latching, please consult the
 drive manufacturer's documentation.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5  
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "sercos.h"

#define AXIS			0
#define RT_STATUS_1		0x40
#define RT_STATUS_2		0x80


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
	int16 error_code, drive_status;
	long val;
	double act;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(set_idn(AXIS, 405, 1));	/* Probe 1 Enable */
	error(set_idn(AXIS, 406, 1));	/* Probe 2 Enable */
	set_idn(AXIS, 169, 0x5);	/* Probe Control, trigger on rising edges */
	
	set_idn(AXIS, 305, 409);	/* Allocate Probe 1 status to Real Time Status */
	set_idn(AXIS, 307, 411);	/* Allocate Probe 2 status to Real Time Status */

	set_idn(AXIS, 170, 0x3);	/* Probe set/enable procedure */

	printf("\nSERCOS drive ready for position latching.\n");

	while (!kbhit())
	{ 	error(get_drive_status(AXIS, &drive_status));	/* Drive Status */
		error(get_position(AXIS, &act));
		drive_status &= (RT_STATUS_1 | RT_STATUS_2);
		printf("\rAct:%10.0lf Status:%x", act, drive_status);  
	}
	getch();	

	error(get_idn(AXIS, 130, &val));
	printf("\nLatch P1:%ld", val);
	error(get_idn(AXIS, 132, &val));
	printf("\nLatch P2:%ld", val);
	
	return 0;
}
