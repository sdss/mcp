/* JOGLAT2.C

:Jog, latch positions, generate stop events, and back off a relative distance.

This code demonstrates how to configure the DSP for analog jogging, position
 latching, and Stop Event generation based on an User I/O bit.  The steps are:

 1) Initialize the analog inputs and configure three axes for jogging.
 2) Download frames to trigger Stop Events when User I/O bit #22 goes low.
 3) Configure and arm position latching.
 4) Wait for position latch and Stop Event.
 5) Determine the last commanded direction.
 6) Clear the Stop Events.
 7) Command a relative move in the opposite direction. 
 
The jogging velocity is calculated as follows:

	Each sample, the DSP runs through the following calculations to 
	determine the velocity that the motor will run at.

		if A/D value > (center + deadband)
			then J = A/D value - center - deadband
		if A/D value < (center - deadband)
			then J = A/D value - center + deadband
		if (center + deadband) > A/D value > (center - deadband)
			then J = 0

		motor velocity (counts/sec) = (linear term * 1/65536 * J
									   + cubic term * 3.632E-12 * (J * J * J))
									   * sample_rate

Be sure user I/O bit #22 is normally driven high, and is pulled low to
 activate the latch.  The falling edge of bit #22 triggers the DSP's
 interrupt.  The DSP's interrupt routine handles the latching of the actual
 positions of all axes within 4 microseconds.

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"
# include "idsp.h"


# define LATCH_BIT		22
# define VELOCITY		1000.0
# define ACCEL			10000.0

# define STOP_RATE		10000.0		/* Stop Event Deceleration */

# define EXP_EVENT		0x000D

int16
	jog_axes[] = {0, 1, 2},
	jog_channel[] = {0, 1, 2},
	jog_analog_channel[] = {0, 1, 2},
	jog_center[] = {2048, 2048, 2048},
	jog_deadband[] = {50, 50, 50},
	jog_linear[] = {5, 20, 50},
	jog_cubic[] = {10, 50, 100};

double back_off_dist[] = {7000.0, 8000.0, 5000.0};


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

void set_jogging(int16 n_axes, int16 * map, int16 enable)
{
	int16 i;
	
	for (i = 0; i < n_axes; i++)
		jog_axis(map[i], jog_channel[i], jog_center[i], jog_deadband[i],
			 jog_linear[i], jog_cubic[i], enable);
}

void stop_when_latched(int16 n_axes, int16 * map)
{
	int16 i;

	/* Download frames to generate a Stop Event when the LATCH_BIT goes low. */
	
	for (i = 0; i < n_axes; i++)
	{
		dsp_io_trigger(map[i], LATCH_BIT, FALSE); 
		dsp_axis_command(map[i], map[i], STOP_EVENT);
	}
}

void display(int16 n_axes, int16 * map)
{
    int16 i;
    double cmd;

    printf("\r");
    
    for (i = 0; i < n_axes; i++)
    {
      	get_command(map[i], &cmd);
        printf("%d: %10.0lf ", i, cmd);
    }
}

int16 end_of_motion(int16 n_axes, int16 * map)
{
    int16 i;
    
    for(i = 0; i < n_axes; i++)
    { 	
    	if(!motion_done(map[i]))
            return 0;
    }
    return 1;
}

void find_direction(int16 n_axes, int16 * map, int16 * direction)
{
	int16 i, temp_state, ts_addr;

	for (i = 0; i < n_axes; i++)
	{
		direction[i] = 0;
		
		ts_addr = dspPtr->pc_status + (dspPtr->axes * 2) + map[i];
		temp_state = dsp_read_dm(ts_addr);

		if ((temp_state & EXP_EVENT) == EXP_EVENT)
		{	
			if (temp_state & TRIGGER_NEGATIVE)
				direction[i] = -1;
			else
				direction[i] = 1;	
		}
	}
}

void recover(int16 n_axes, int16 * map)
{
	int16 i, dir[PCDSP_MAX_AXES], temp_state;
	
	set_jogging(n_axes, map, FALSE);	/* Disable jogging */
	
	while (!end_of_motion(n_axes, map))
		display(n_axes, jog_axes);

	find_direction(n_axes, map, dir);	
	for (i = 0; i < n_axes; i++)
	{
		error(clear_status(map[i]));
		start_r_move(map[i], (dir[i] * back_off_dist[i]), VELOCITY, ACCEL);
	}
}

void initialize(int16 n_axes, int16 * map)
{
	int16 i, error_code;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	init_io(2, IO_INPUT);
	for (i = 0; i < n_axes; i++)
		set_stop_rate(map[i], STOP_RATE);
	
	/* Initialize the jog axes to use specific analog channels and configure
	 	these channels for unipolar and single ended operation.
		The default analog channel configuration is the analog channel = axis,
		unipolar, and single ended.
	*/
	for (i = 0; i < n_axes; i++)
		set_analog_channel(map[i], jog_analog_channel[i], FALSE, FALSE);
		
	set_jogging(n_axes, jog_axes, TRUE);	/* enable jogging */
	stop_when_latched(n_axes, jog_axes);    
	arm_latch(TRUE);
}

int16 main()
{
	int16 i, n_axes;
	double position;
	
	n_axes = (sizeof(jog_axes)/sizeof(int16));	
	
	initialize(n_axes, jog_axes);
	printf("\nToggle bit #22 to latch positions.\n\n"); 
	
	while (!latch_status())
		display(n_axes, jog_axes);
		
	for (i = 0; i < n_axes; i++)
	{ 	
		get_latched_position(jog_axes[i], &position);
		printf("\nAxis: %d Latched Pos: %12.0lf", jog_axes[i], position);
	}
	printf("\n");
		
	recover(n_axes, jog_axes);
	while (!end_of_motion(n_axes, jog_axes))
		display(n_axes, jog_axes); 
	
	return 0;
}
