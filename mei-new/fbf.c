/* FBF.C

:Feedback fault checking.

This sample demonstrates how to configure the DSP-Series controller to detect
 feedback fault conditions.  Currently, broken wires and illegal states
 can be detected with differential incremental encoders. 

Feedback fault checking requires version 2.4F4 firmware (or higher) and the
 following hardware revisions (or higher):

	Controller	Rev

	PCX/DSP		3
	STD/DSP		8
	104/DSP		6
	LC/DSP		8
	V6U/DSP		3
 
Two registers determine the status of the encoder for each axis:

	Address		Status
	
	0x70		Broken Encoder Wire (each bit represents an axis)
	0x71		Illegal Encoder State (each bit represents an axis)

Every sample, the DSP will check these registers if the FB_CHECK (0x400)
 bit is set in the config word for a given axis.  If a bit is set in either 
 register (0x70 or 0x71), the DSP will generate an Abort event corresponding
 to the axis at fault.
 
The type of encoder fault can be determined by reading the Broken wire and
 illegal state registers.  These registers can be cleared by writting a 0 to 
 them.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5  
Written for Firmware Version 2.4F4  
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "idsp.h"

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
	int16 error_code, source, axis, axes;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	/* Enable feedback fault checking. */
	for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
		set_feedback_check(axis, TRUE);
		
	printf("\nFeedback fault checking enabled (any key to quit).\n");

	while (!kbhit())
		printf("Fault: 0x%4X Illegal: 0x%4X\r", dsp_read_dm(FB_FAULT),
			dsp_read_dm(FB_ILLEGAL_STATE));
	getch();
	
	printf("\n\nAxis Source");
	for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{ 	source = axis_source(axis);
		printf("\n %1d    0x%2X  ", axis, source);

		if (source == ID_FEEDBACK_FAULT)
			printf("  Fault!");
		if (source == ID_FEEDBACK_ILLEGAL_STATE)
			printf("  Illegal State!");	
	}	
	
	for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	set_feedback_check(axis, FALSE);
		controller_run(axis);
	}	
	
    return 0;
}
