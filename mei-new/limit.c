/* LIMIT.C

:Simple limit switch recovery 

This sample demonstrates a basic limit switch recovery algorithm.  Each axis
 has a positive and negative hardware limit input.  The active level and
 exception event response for each limit input is software configurable.
 
Each limit input is logically combined with the command velocity.  The negative
 limit is disabled when the axis moves in the positive direction and the
 positive limit is disabled when the axis moves in the negative direction.   
 
The DSP generates the exception event (None, Stop, E-Stop, or Abort) when the
 limit input reaches the active state and the command velocity is non-zero in
 the direction of the corresponding limit sensor. 
 
Here is the algorithm:
 1) Velocity move towards the positive limit input.
 2) Stop at the positive limit sensor (Stop Event generated by the DSP).
 3) Clear the Event.
 4) Move off the positive limit sensor.
 5) Velocity move towards the negative limit input.
 6) E-Stop at the negative limit sensor (E-Stop Event generated by the DSP).
 7) Clear the Event.
 8) Move off the negative limit sensor.

Here is the sensor logic:
 + limit input = active low
 - limit input = active low

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
	Revision 1.1  1999/08/31 16:43:20  briegel
	source for MEI library

*/

# include <stdio.h>
# include <stdlib.h>
# include <conio.h>   
# include "pcdsp.h"

# define AXIS			0
# define VEL			1000.0
# define ACCEL			5000.0
# define RECOVER_DIST	4000.0

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
	int16 pos_lim, neg_lim, state;
	double cmd;
	
	while (!motion_done(axis))
	{	get_command(axis, &cmd);
		pos_lim = pos_switch(AXIS);
		neg_lim = neg_switch(AXIS);
		printf("Cmd:%8.0lf +Lim:%d -Lim:%d\r", cmd, pos_lim, neg_lim);	
	
		if (kbhit())
		{	getch();
			v_move(axis, 0.0, ACCEL);
			exit(1);
		}	
	}
	printf("\n");
}

int16 check_for_event(int16 axis)
{
	int16 state;

	state = axis_state(axis);
		
	/* Check if an Exception Event occured */	
	switch (state)
	{
		case STOP_EVENT:
		{	printf("\nException Event = Stop");
			return state;
		}	
			
		case E_STOP_EVENT:
		{	printf("\nException Event = Emergency Stop");
			return state;
		}	
	}
	return 0;
}

int16 recover(int16 axis, int16 state)
{
	int16 recover_dir;

	switch (axis_source(axis))
	{
		case ID_POS_LIMIT:
		{	printf("\nSource = Positive Limit");
			recover_dir = -1;
			break;
		}	
		
		case ID_NEG_LIMIT:
		{	printf("\nSource = Negative Limit");
			recover_dir = 1;
			break;
		}	

		default:
			printf("\nNo Limit was triggered\n");
	}
	
	/* Make sure the event is complete before clearing the status */
	while (!motion_done(axis))
		;
		
	if ((state == E_STOP_EVENT) || (state == STOP_EVENT))	
	{	error(clear_status(axis));
		
		printf("\nMoving off the limit\n");
		start_r_move(axis, (RECOVER_DIST * recover_dir), VEL, ACCEL);
	
		display(axis);	
	}	

	/* Was the recovery successful? */
	if (check_for_event(axis))
	{	printf("\nUnable to Recover - Both limits tripped?\n");
		return 1;
	}	

	return 0;
}

int16 main()
{
	int16 error_code, state;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */
	
	/* Configure the limits */
	set_positive_limit(AXIS, STOP_EVENT);
	set_negative_limit(AXIS, E_STOP_EVENT);
	set_positive_level(AXIS, FALSE);
	set_negative_level(AXIS, FALSE);
	set_stop_rate(AXIS, ACCEL);
	set_e_stop_rate(AXIS, ACCEL);
   
	printf("\nSearching for the positive limit sensor. (any key to quit)\n\n");
	v_move(AXIS, VEL, ACCEL);		/* velocity move towards positive limit */
	display(AXIS);
	
	state = check_for_event(AXIS);	/* Did we hit the positive limit? */
	if (state)			
	 	recover(AXIS, state);
	 	
	printf("\nSearching for the negative limit sensor. (any key to quit)\n\n");
	v_move(AXIS, -VEL, ACCEL);		/* velocity move towards positive limit */
	display(AXIS);
	
	state = check_for_event(AXIS);	/* Did we hit the negative limit? */
	if (state)			
	 	recover(AXIS, state);

	return 0;
}
