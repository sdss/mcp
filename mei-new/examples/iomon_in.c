/* IOMON_IN.C

:Initialization for I/O generated interrupts under Windows NT.

This sample demonstrates how to initialize a DSP-Series controller
 to monitor I/O lines and generate interrupts under Windows NT.

This program is put to "sleep" until an interrupt is sent from the 
 DSP-Series controller to the host CPU.

When using interrupts, be sure to set the appropriate IRQ switch on the
 DSP-Series controller.  Also, make sure the device driver "DSPIO" is 
 configured for the same IRQ.

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
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	init_io(0, IO_INPUT);	/* initialize I/O port for inputs */
	init_io(1, IO_INPUT);

	init_io(2, IO_OUTPUT);	/* initialize I/O port for outputs */
	reset_bit(23);			/* enable interrupt generation */

	/* initialize I/O Monitoring */
	set_io_mon_mask(0, 1, 0);
	clear_io_mon();

	/* Wait for I/O Monitioring to send an Interrupt */
	/* for MEI_TOGGLE and MEI_IO_MON axis is ignored, use 0.*/ 
	mei_sleep_until_interrupt(dspPtr->dsp_file, 0, MEI_IO_MON);

	printf("io monitor interrupt!!!!!!!\n");
	printf("Hit any key to exit.\n");

	getch();

	return 0;	
}
