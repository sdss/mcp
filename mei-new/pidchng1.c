/* PIDCHNG.C

:Change the PID filter parameters when the error limit is exceeded.

This sample demonstrates how to use a frame to update the PID filter
 parameters when an axis' error limit is exceeded.  The error limit is
 configured to generate a New Frame Event.  Then the frame buffer is loaded
 with a very long dwell frame and a dsp_set_filter(...) frame.  When the error
 limit is exceeded, the DSP will throw out the dwell frame and execute the
 dsp_set_filter(...) frame.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/



# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"

# define AXIS			0


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

void display(int16 axis)
{
	int16 coeffs[COEFFICIENTS];
	double error;
	
	while (!kbhit())
	{ 	get_error(axis, &error);
		get_filter(axis, coeffs);
		printf("Error: %6.0lf P: %6d\r", error, coeffs[0]);
	}	
	getch();
}

int16	main()
{
	int16
		error_code,
		pid_coeffs[COEFFICIENTS],
		soft_coeffs[COEFFICIENTS] = {80, 7, 300, 0, 0, 32767, 29, 32767, -9};
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	get_filter(AXIS, pid_coeffs);
	set_error_limit(AXIS, 300.0, NEW_FRAME);
	
	dsp_dwell(AXIS, 1000000.0);			/* DSP will wait for error limit */
    dsp_set_filter(AXIS, soft_coeffs);	/* DSP will change filter params */

    display(AXIS);

    return 0;
}
