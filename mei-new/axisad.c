/* AXISAD.C

:Configure the DSP to read A/D (one channel per axis)

This code configures the DSP to read one analog input channel per axis.
 The analog input channels are configured for unipolar voltage, single
 ended inputs.  The A/D value is displayed to the screen.

Remember, the CPU and the DSP cannot directly read from the A/D converter
 at the same time.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"


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

void display(int16 axes)
{
	int16 axis, value;
	
	while (!kbhit())
	{ 	for (axis = 0; axis < axes; axis++)
		{ 	read_axis_analog(axis, &value);
			printf("%1d:%4d ", axis, value);
		}
		printf("\r");
	}
	getch();
	
}

int16	main()
{
	int16 error_code, axis, axes;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(dsp_reset());		/* reset the hardware */

	axes = dsp_axes();
		
	for (axis = 0; axis < axes; axis++)
	{ 	set_analog_channel(axis, axis, FALSE, FALSE);
		set_axis_analog(axis, TRUE);	/* enable the DSP to read the A/D */
	}		

	display(axes);

    return 0;
}
