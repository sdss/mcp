/* EVENTREC.C

:Simple Exception Event recovery 

This sample demonstrates how to recover from an Exception Event.  The Exception
 Events can be generated by the DSP (dedicated digital I/O or software limits)
 or directly from software.  The Events are Stop, Emergency Stop, and Abort.

The steps to exception event handling are:
 1) Determine what type of exception event occured.
 2) Determine the cause of the exception event.
 3) Recover from the event based on the cause. 

In this simple sample, the event recovery does not depend upon the cause of 
 the event.  In a "real" application the recovery routine may be different
 depending on the source of the event.
 
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
# define ACCEL			5000.0

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
		printf("Cmd:%8.0lf\r", cmd);	
	
		if (kbhit())
		{	getch();
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
		
		case ABORT_EVENT:
		{	printf("\nException Event = Abort");
			return state;
		}	
	}
	return 0;
}

int16 recover(int16 axis, int16 state)
{
	int16 recover_dir;

	/*  For this sample, the source of the event is reported to the screen and
		 then the event is cleared:
		
		 clear_status(...) - for STOP_EVENT or E_STOP_EVENT
		 controller_run(...), enable_amplifier(...) - for ABORT_EVENT

		In a "real" application the recovery routine may be different depending
		 on the source of the event.
	*/	
	
	switch (axis_source(axis))
	{
		case ID_HOME_SWITCH:
		{	printf("\nSource = Home Input");
			break;
		}	
		
		case ID_POS_LIMIT:
		{	printf("\nSource = Positive Limit Input");
			break;
		}	
		
		case ID_NEG_LIMIT:
		{	printf("\nSource = Negative Limit Input");
			break;
		}	
		
		case ID_AMP_FAULT:
		{	printf("\nSource = Amp Fault Input");
			break;
		}	
		
		case ID_X_NEG_LIMIT:
		{	printf("\nSource = Negative Software Limit Exceeded");
			break;
		}	
		
		case ID_X_POS_LIMIT:
		{	printf("\nSource = Positive Software Limit Exceeded");
			break;
		}	

		case ID_ERROR_LIMIT:
		{	printf("\nSource = Position Error Limit Exceeded");
			break;
		}
	}
	
	/* Make sure the event is complete before clearing the status. */
	while (!motion_done(axis))
		;
		
	/* Note: Simply clearing the exception event (no matter what caused the 
		event) may not be appropriate for all applications.
	*/	
	if (state == ABORT_EVENT)
	{ 	controller_run(axis);
		enable_amplifier(axis);	
	}	
	
	if ((state == E_STOP_EVENT) || (state == STOP_EVENT))	
		error(clear_status(axis));

	return 0;
}

int16 main()
{
	int16 error_code, state;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */
	
	/* Configure the software limit */
	set_positive_sw_limit(AXIS, 10000.0, STOP_EVENT);
   
	printf("\nMoving to the positive software limit (any key to quit)\n\n");
	v_move(AXIS, VEL, ACCEL);
	display(AXIS);
	
	state = check_for_event(AXIS);
	if (state)			
	 	recover(AXIS, state);
	 	
	return 0;
}
