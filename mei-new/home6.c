/* HOME6.C

:Simple homing routine using the encoders' index pulse.

This sample demonstrates a basic homing algorithm.  The home location is found
 based on the encoders' index pulse.
 
Here is the algorithm:
 1) Velocity move towards the index pulse.
 2) Stop at the index pulse (Stop Event generated by DSP).
 3) Zero the position.

Here is the sensor logic:
 Index input = active high
 Home input = not used

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/

/*  Revision Control System Information
	$Source$ 
	$Revision$
	$Date$

	$Log$
	Revision 1.1  1999/08/31 16:43:10  briegel
	source for MEI library

*/

# include <stdio.h>
# include <stdlib.h>
# include <conio.h>   
# include "pcdsp.h"

# define AXIS       0
# define ACCEL      50000.0 
# define HOME_VEL   1000.0         /* Velocity must be less than the DSP's
									  sample rate to guarantee the index
									  pulse is not missed.              */
								 
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
	int16 home_logic, state;
	double cmd;
	
	while (!motion_done(axis))
	{   
		get_command(axis, &cmd);
		printf("\rCmd: %10.0lf Home Logic: %d", cmd, home_switch(AXIS));    
	
		if (kbhit())
		{   getch();
			exit(1);
		}   
	}
	printf("\n");
}

int16 main()
{
	int16 error_code;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	error(dsp_reset());     /* hardware reset */
	
	set_home_index_config(AXIS, INDEX_ONLY);
	set_home_level(AXIS, TRUE);         /* home input logic active high */
	set_home(AXIS, STOP_EVENT);
	set_stop_rate(AXIS, ACCEL);
	
	/* Clear any events generated during home logic configuration */
	error(clear_status(AXIS));
   
	printf("\nSearching for index pulse. (Press any key to quit)\n");
	v_move(AXIS, HOME_VEL, ACCEL);      /* velocity move towards index pulse */
	display(AXIS);
	
	set_home(AXIS, NO_EVENT);
	error(clear_status(AXIS));
	set_position(AXIS, 0.0);            /* zero command and actual position */
	printf("\nAxis is home.\n");

	return 0;
}