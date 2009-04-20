/* HOME5.C

:Homing routine using a mechanical hard stop.

This sample demonstrates a basic homing algorithm.  The home location is found
 based on a mechanical hard stop.  The DSP senses the hard stop when the
 difference between the command and actual positions exceeds the error limit. 
 
Here is the algorithm:
 1) Velocity move towards the hard stop.
 2) Stop at the mechanical hard stop (when the error limit is exceeded).
 3) Zero the command and actual position.

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
# define VEL			1000.0
# define ACCEL			1000.0
# define FAST_ACCEL		100000.0

# define BACK_OFF_DIST	(-3000.0)
# define HOME_ERR_LIMIT	20.0
# define HOME_DAC_LIMIT	3276		/* limits control voltage to +/- 1 volt */

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
	double cmd, act;
	
	while (!motion_done(axis))
	{	get_command(axis, &cmd);
		get_position(axis, &act);
		printf("\rCmd: %10.0lf Act: %10.0lf", cmd, act);	
	}
	printf("\n");
}

int16 main()
{
	int16 error_code, coeffs[COEFFICIENTS], dac_limit, err_event;
	double err_limit;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	get_filter(AXIS, coeffs);
	get_error_limit(AXIS, &err_limit, &err_event);
	
	dac_limit = coeffs[DF_DAC_LIMIT];	/* reduce the dac limit */
	coeffs[DF_DAC_LIMIT] = HOME_DAC_LIMIT;
	set_filter(AXIS, coeffs);
	
	printf("\nSearching for mechanical limit.\n");
	set_error_limit(AXIS, HOME_ERR_LIMIT, NO_EVENT);
	v_move(AXIS, VEL, ACCEL);			/* velocity move towards hard stop */
	dsp_error_action(AXIS, NEW_FRAME);
	dsp_dwell(AXIS, 100000.0);			/* wait for New Frame event */
	dsp_error_action(AXIS, NO_EVENT);
	v_move(AXIS, 0.0, FAST_ACCEL);		/* decel to a stop */
	set_position(AXIS, 0.0);			/* zero command and actual position */
	dsp_end_sequence(AXIS);
	display(AXIS);
	
	printf("\nAxis is home. Hit any key to back off home.\n");
	getch();
	start_r_move(AXIS, BACK_OFF_DIST, VEL, ACCEL);
	display(AXIS);
	
	coeffs[DF_DAC_LIMIT] = dac_limit;	/* reset the dac limit */
	set_filter(AXIS, coeffs);
	set_error_limit(AXIS, err_limit, err_event);

	return 0;
}
