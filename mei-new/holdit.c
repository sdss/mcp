/* HOLDIT.C

:Pause and resume motion on path.

This sample demonstrates how to use the feed speed override to pause and
 continue motion on path.  To pause the motion the feed rate is reduced to
 zero.  To resume motion the feed rate is increased to 100%.  To abort the
 motion, Stop Events are generated on the axes.
		
This sample is programmed as a state machine.

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
	Revision 1.1  1999/08/31 16:43:07  briegel
	source for MEI library

*/


# include <stdio.h>
# include <conio.h>
# include <dos.h>
# include <stdlib.h>
# include "pcdsp.h"

int16 state;						/* one state variable for all axes */
double rate;					/* feed speed override rate */

/* State Values */
# define READY			0
# define HALT			1		/* pause a move */
# define GO				2		/* continue a move */
# define MOVE_FWD		3
# define MOVE_BACK		4
# define NEVER_MIND		5		/* abort the motion */

# define AXES			2


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

void display(int16 n_axes, int16 * map)
{
    int16 i;
    double cmd;

    for (i = 0; i < n_axes; i++)
    { 	get_command(map[i], &cmd);
        printf("%d: %10.0lf ", map[i], cmd);
    }
    printf("\r");
}

void CheckState(int16 n_axes, int16 * map)
{
	int16 i;
	
	switch (state)
	{
		case HALT:
			rate -= .01;
			if(rate <= 0)
			{	rate = 0;
				state = READY;
			}
			error(dsp_feed_rate(rate));		/* update feed rate */
			break;

		case GO:
			rate += .01;
			if(rate >= 1)
			{	rate = 1;
				state = READY;
			}
			error(dsp_feed_rate(rate));     /* update feed rate */
			break;

		case MOVE_FWD:
			move_2(100000.0, 200000.0);
			state = READY;
			break;
			
		case MOVE_BACK:
			move_2(0.0, 0.0);
			state = READY;
			break;

		case NEVER_MIND:					/* discontinue the move */
			if(rate == 0.0)
			{	for (i = 0; i < n_axes; i++)
				{ 	set_jerk(map[i], 0.0);	/* zero the trajectory generator */
					set_accel(map[i], 0.0);
					set_velocity(map[i], 0.0);
					stop_motion();					/* generate Stop Events */
					rate = 1.0;
					error(dsp_feed_rate(rate));		/* execute the Stop Events */
				}	
			}
			else
				stop_motion();

			while(!all_done())
				;
			for (i = 0; i < n_axes; i++)	
				error(clear_status(map[i]));
				
			printf("\n");
			state = READY;
			break;
	}
}

int16	main()
{
	int16 error_code, done, key, n_axes, axes[] = {0, 1};
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	n_axes = (sizeof(axes) / sizeof(int16));
	
	map_axes(n_axes, axes);
	set_move_speed(8000.0);
	set_move_accel(80000.0);

	printf("\nf=fwd, b=back, h=halt, g=go, n=endmove, esc=quit\n");

	for (done = 0; !done; )
	{
		display(n_axes, axes);
		CheckState(n_axes, axes);

		if (kbhit())     /* key pressed? */
		{	key = getch() ;

			switch (key)
			{
			case 'h':       /* Pause the motion */
				state = HALT;
				break;

			case 'g':       /* Resume the motion */
				state = GO;
				break;

			case 'f':       /* Move the axes forward */
				state = MOVE_FWD;
				break;

			case 'b':       /* Move the axes backward */
				state = MOVE_BACK;
				break;

			case 'n':       /* End the motion */
				state = NEVER_MIND;
				break;

			case 0x1B:      /* <ESC> */
				done = TRUE;
				break;
			}
		}
	}
	return 0;
}
