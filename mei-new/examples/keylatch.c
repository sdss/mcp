/* KEYLATCH.C

:Latch the actual positions of all axes based on keyboard input.

Be sure user I/O bit #22 is left unconnected when using this sample program.
 The falling edge of bit #22 triggers the DSP's interrupt.  The DSP's
 interrupt routine handles the latching of the actual positions of all axes
 within 4 microseconds.

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

int16	main()
{
	int16 error_code, axis, axes, done = 0;
	double p;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	axes = dsp_axes();
	printf("\nPress any key to latch %d axes, esc=quit\n", axes);	

	init_io(2, IO_OUTPUT);
	set_bit(22);			/* initialize bit #22 high */
	arm_latch(TRUE);

	while (!done)
	{			
		while (!latch_status())
		{
			if (kbhit())
			{
				if (getch() == 0x1B)
					exit(1);
				else
					latch();	
			}		
		}		
		for (axis = 0; axis < axes; axis++)
		{ 	get_latched_position(axis, &p);
			printf("Axis:%d Position:%12.0lf\n", axis, p);
		}
		printf("\n");
		arm_latch(TRUE);		/* reset the latch. */
	}	

    return 0;
}
