/* STOPLAT.C

:Move, latch positions, generate stop events, and back off a relative distance.

This sample creates a sequence of frames on a phantom axis.  The frames cause
 the DSP to latch positions and generate Stop Events based on an User I/O bit.
 The steps are:

 1) Configure a phantom axis (requires n+1 axis firmware on an n axis card).
 2) Download frames to trigger Stop Events when User I/O bit #22 goes low.
 3) Configure and arm position latching.
 4) Command motion on 3 axes. 
 5) Wait for position latch and Stop Event.
 6) Clear the Stop Events.
 7) Command a relative move in the opposite direction. 
 
Be sure user I/O bit #22 is normally driven high, and is pulled low to
 activate the latch.  The falling edge of bit #22 triggers the DSP's
 interrupt.  The DSP's interrupt routine handles the latching of the actual
 positions of all axes within 4 microseconds.
 
The phantom axis is created by downloading 4axis.abs to a 3 axis board.  

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
	Revision 1.1  1999/08/31 16:43:59  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"
# include "idsp.h"

# define AXES			4
# define PHANTOM		3

# define LATCH_BIT		22
# define VELOCITY		1000.0
# define ACCEL			10000.0

# define STOP_RATE		10000.0		/* Stop Event Deceleration */


double 
	dist[] = {25000.0, 25000.0, 25000.0},
	back_off_dist[] = {7000.0, 8000.0, 5000.0};

int16 back_dir[PCDSP_MAX_AXES] = {0, 0, 0};


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

void stop_when_latched(int16 phantom, int16 n_axes, int16 * map)
{
	int16 i;

	/* Download frames to generate a Stop Event when the LATCH_BIT goes low. */
	dsp_io_trigger(phantom, LATCH_BIT, FALSE); 
	for (i = 0; i < n_axes; i++)
		dsp_axis_command(phantom, map[i], STOP_EVENT);
}

void display(int16 n_axes, int16 * map)
{
    int16 i;
    double cmd;

    printf("\r");
    
    for (i = 0; i < n_axes; i++)
    { 	get_command(map[i], &cmd);
        printf("%d: %10.0lf ", i, cmd);
    }
}

int16 end_of_motion(int16 n_axes, int16 * map)
{
    int16 i;
    
    for(i = 0; i < n_axes; i++)
    {	if(!motion_done(map[i]))
            return 0;
    }
    return 1;
}


void recover(int16 n_axes, int16 * map)
{
	int16 i, temp_state;
	
	while (!end_of_motion(n_axes, map))
		display(n_axes, map);

	for (i = 0; i < n_axes; i++)
	{ 	error(clear_status(map[i]));
		start_r_move(map[i], (back_dir[i] * back_off_dist[i]), VELOCITY, ACCEL);
		back_dir[i] = 0;	/* reset the direction */
	}
}

void initialize(int16 phantom, int16 n_axes, int16 * map)
{
	int16 i, error_code;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	disable_hardware_limits(phantom);	/* prevent unintended events */
	disable_software_limits(phantom);
	
	init_io(2, IO_INPUT);
	for (i = 0; i < n_axes; i++)
		set_stop_rate(map[i], STOP_RATE);
	
	stop_when_latched(PHANTOM, n_axes, map);    
	arm_latch(TRUE);
}


int16 main()
{
	int16 i, n_axes, axes[] = {0, 1, 2};
	double position;

	n_axes = (sizeof(axes)/sizeof(int16));
	
	initialize(PHANTOM, n_axes, axes);
	printf("\nToggle bit #22 to latch positions.\n\n"); 
	
	set_gates(n_axes, axes);
	for (i = 0; i < n_axes; i++)
	{ 	start_r_move(axes[i], dist[i], VELOCITY, ACCEL); 
		/* set the back off direction */
		if (dist[i] > 0)
			back_dir[i] = -1;
		if (dist[i] < 0)
			back_dir[i] = 1;	
	}		
	reset_gates(n_axes, axes);
	
	while (!latch_status())
		display(n_axes, axes);
		
	for (i = 0; i < n_axes; i++)
	{	get_latched_position(axes[i], &position);
		printf("\nAxis: %d Latched Pos: %12.0lf", axes[i], position);
	}
	printf("\n");
		
	recover(n_axes, axes);
	while (!end_of_motion(n_axes, axes))
		display(n_axes, axes); 
	
	return 0;
}
