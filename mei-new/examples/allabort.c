/* ALLABORT.C

:Generate ABORT_EVENTs based on the amp_enable bits.

This sample creates a sequence of frames on a phantom axis.  The sequence of
 frames generate ABORT_EVENTs on axes (0-2) when any of the amp_enable's on
 axes 0-2 are low.  This is very useful during coordinated motion.
 
The phantom axis is created by downloading 4axis.abs to a 3 axis board.  

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include "pcdsp.h"

# define	PHANTOM		3
# define	AXES		4


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

void disable_hardware_limits(int16 axis)
{
	set_positive_limit(axis, NO_EVENT);
	set_negative_limit(axis, NO_EVENT);
	set_home(axis, NO_EVENT);
	set_amp_fault(axis, NO_EVENT);
}

void disable_software_limits(int16 axis)
{
	int16 action;
	double position;
	
	get_positive_sw_limit(axis, &position, &action);
	set_positive_sw_limit(axis, position, NO_EVENT);
	get_negative_sw_limit(axis, &position, &action);
	set_negative_sw_limit(axis, position, NO_EVENT);
	get_error_limit(axis, &position, &action);
	set_error_limit(axis, position, NO_EVENT);
}

int16	main()
{
	int16 error_code, axis;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	disable_hardware_limits(PHANTOM);	/* prevent unintended events */
	disable_software_limits(PHANTOM);

	for (axis = 0; axis < AXES; axis++)
	{
		set_amp_enable_level(axis, TRUE);
		while (controller_run(axis))
			;
		enable_amplifier(axis);
	}

/* 	The amp enable output bits for axes 0, 1 and 2 (bits 0, 2 and 4) are on 
	user I/O port 8.  The value for the amp enables mask = 0x1 + 0x4 + 0x10.
	When the state parameter is FALSE, the next frame is triggered if any
	masked bit is low (0 volts).
*/ 
	dsp_io_trigger_mask(PHANTOM, 8, 0x15, FALSE);
	
	dsp_axis_command(PHANTOM, 0, ABORT_EVENT);
	dsp_axis_command(PHANTOM, 1, ABORT_EVENT);
	dsp_axis_command(PHANTOM, 2, ABORT_EVENT);

    return 0;
}
