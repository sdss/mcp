/* ANA2ENC3.C

:Switching between analog and encoder feedback.

This sample code configures the first axis, ENCODER_AXIS for encoder feedback
 and the second axis, ANALOG_AXIS for analog feedback.  Based on keyboard input
 the +/- 10 volt analog output channels are switched between the ENCODER_AXIS
 and the ANALOG_AXIS.
 
The encoder input is used for position control and the analog feedback is
 typically used for force control.  By switching the analog output channels
 we are really switching feedback devices, each with their own PID loop.  
 The motor is controlled by a single analog control voltage output, the
 DAC_CHANNEL.  The ANALOG_AXIS' analog output channel is not used.
 
Note:  The default analog output configuration is axis = dac channel.

Since the analog output for the ANALOG_AXIS is not used, it does not require
 the digital to analog circuitry.  For example, a single axis controller can
 control one motor with one channel of encoder feedback and one channel of
 analog feedback.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5
*/

/*	Revision Control System Information
	$Source$ 
	$Revision$
	$Date$
	$Log$
	Revision 1.1  1999/08/31 16:42:53  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"

# define AXES			2

# define ENCODER_AXIS	0	/* encoder feedback axis */	
# define ANALOG_AXIS	1	/* analog feedback axis */

# define DAC_CHANNEL	0 	/* DAC channel to control motor */


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

void display_dacs(int16 axes)
{
	int16 axis, value;

	for (axis = 0; axis < axes; axis++)
	{
		get_dac_output(axis, &value);
		printf("Axis%d Dac%d  ", axis, value);
	}	
	printf("\n");
}

void display_positions(int16 axes)
{
	int16 axis;
	double cmd, act;

	printf("\r");
	for (axis = 0; axis < axes; axis++)
	{ 	
		get_command(axis, &cmd);
		get_position(axis, &act);
		printf("Axis%d Cmd%6.0lf Act%6.0lf  ", axis, cmd, act);
	}	
}	

void set_control_axis(int16 control_axis, int16 other_axis)  
{
	int16 dac_a, dac_b;

	/* Read the configured dac channel for each axis. */ 
	get_dac_channel(control_axis, &dac_a);
	get_dac_channel(other_axis, &dac_b);

	if (dac_a != DAC_CHANNEL)		/* switch the dac channels? */
	{
	/* Set the command position equal to the actual position to prevent the 
		motor from "snapping" to position.
	*/
		controller_run(control_axis);
		controller_run(other_axis);

		display_dacs(AXES);
		
	/* Switch the dac channel configuration.  Be sure to always change the
	 	control axis first.  By switching the dac channels we are really
	 	switching feedback devices, each with their own PID loop.  
	*/	
		set_dac_channel(control_axis, dac_b);
		set_dac_channel(other_axis, dac_a);
	}	
}

int16	main()
{
	int16 error_code, done, key;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	set_feedback(ANALOG_AXIS, FB_ANALOG);
	set_analog_channel(ANALOG_AXIS, 0, FALSE, FALSE);

	printf("\na=analog feedback, e=encoder feedback, <esc> to quit.\n");
	for (done = 0; !done; )
	{
		display_positions(AXES);
		
		if (kbhit())	/* key pressed? */
		{	key = getch();
			switch (key)
			{
                case 'e':
                	printf("\n\nEncoder feedback control. \n");
                  	set_control_axis(ENCODER_AXIS, ANALOG_AXIS); 
                    break;

                case 'a':
                	printf("\n\nAnalog feedback control.  \n");
                  	set_control_axis(ANALOG_AXIS, ENCODER_AXIS); 
                    break;

				case 0x1B:	/* <ESC> */
                    done = TRUE;
					break;
			}		
		}
	}

    return 0;
}
