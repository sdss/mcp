/* PVT4.C

:Generate a multi-axis coordinated motion profile using frames.

This sample shows how to update jerk, acceleration, velocity, and position at
 specified times using frames.

Each frame is a 20 word structure that contains information about the motion
 trajectory.  Every sample, the DSP's calculates an axis' command position
 based on the jerk, acceleration, and velocity values.  The frames are used
 to load the values for the jerk, acceleration, velocity, and/or command 
 position.
 
The frames are stored in an on-board buffer.  The buffer can hold a maximum of
 600 frames.  The DSP executes each frame and then releases it back to the
 "free list".
 
Here are the steps to generate a multi-axis coordinated motion profile.  The
 downloading of the frames is based on a variable called frame_index:

frame_index == FIRST_FRAME
 1) Set the gate flag for each axis to prevent the frames from executing.
 2) Download the first frame for each axis.  Be sure to set the control word
 	to enable the hold bit and check frames.  Also, set the action for check
 	frames.  The check frames will cause the DSP to generate an E-Stop if a
 	valid next frame does not exist.
 3) Reset the gate flag(s) to start the frame execution.	

frame_index == MID_FRAME
 1) Download the next frame in the list.  Be sure to set the control word to
 	enable check frames and set the action for check frames.

frame_index == LAST_FRAME
 1) Download the last frame.  Be sure to set the action word to 0 (to clear the
 	in_sequence flag).
 
Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/



# include <stdio.h>
# include <conio.h>
# include <dos.h>
# include <stdlib.h>
# include <math.h>
# include "pcdsp.h"
# include "idsp.h"


# define BUFFER_SIZE	300		/* number of frames */
# define MAX_POINTS		4

# define ACCEL			1000.0	/* counts per sec * sec */
# define TIME			10.0	/* time between frames (seconds) */

# define FIRST_FRAME	0
# define MID_FRAME		1
# define LAST_FRAME		2

double x[MAX_POINTS], v[MAX_POINTS], a[MAX_POINTS], j[MAX_POINTS];

int16 
	frame_index = LAST_FRAME,
	map[] = {0, 1},
	axes = (sizeof(map) / sizeof(int16));


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

int16 end_of_motion(int16 axes, int16 * map)
{
    int16 i;
    
    for(i = 0; i < axes; i++)
    { 	
    	if(!motion_done(map[i]))
            return 0;
    }
    return 1;
}

void display(int16 axes, int16 * map)
{
    int16 i;
    double cmd;

    for(i = 0; i < axes; i++)
    {
      	get_command(map[i], &cmd);
        printf("%d:%8.0lf ", map[i], cmd);
    }
    printf("\r");
}

int16 load_frames(double * cmd, double * vel, double * acc, double * jerk,
	double time)
{
	int16 i;
	FRAME frame;

	if (fifo_space() < BUFFER_SIZE)		/* Is there enough space? */
		return 1;

	if (frame_index == FIRST_FRAME)
		frame_index = MID_FRAME;
	if (frame_index == LAST_FRAME)
		frame_index = FIRST_FRAME;
		
	for (i = 0; i < axes; i++)
	{	if ((vel[i] == 0.0) && (acc[i] == 0.0) && (jerk[i] == 0.0))
			frame_index = LAST_FRAME;
	}		
	
	switch (frame_index)
	{
		case FIRST_FRAME:
		{
			for (i = 0; i < axes; i++)
			{	set_gate(map[i]);
				frame_m(&frame, "0 l xvajt cund", map[i], cmd[i], vel[i],
					acc[i], jerk[i], time, FCTL_DEFAULT | FCTL_HOLD | 0x16,
					FUPD_POSITION | FUPD_VELOCITY | FUPD_ACCEL | FUPD_JERK |
					FTRG_TIME, CHECK_FRAMES);
			}		
			reset_gates(axes, map);		/* ready, set, go! */
			break;
		}			 

		case MID_FRAME:
		{
			for (i = 0; i < axes; i++)
				frame_m(&frame, "0 l xvajt cund", map[i], cmd[i], vel[i],
					acc[i], jerk[i], time, FCTL_DEFAULT | 0x16, FUPD_POSITION |
					FUPD_VELOCITY | FUPD_ACCEL | FUPD_JERK | FTRG_TIME,
					CHECK_FRAMES);
			break;
		}

		case LAST_FRAME:
		{	
			for (i = 0; i < axes; i++)
			{	frame_m(&frame, "0 l xvajt cund", map[i], cmd[i], vel[i],
					acc[i], jerk[i], (1.0 / dsp_sample_rate()), FCTL_DEFAULT |
					0x16, FUPD_POSITION | FUPD_VELOCITY | FUPD_ACCEL |
					FUPD_JERK | FTRG_TIME, 0);
				/* set the last command position variable */	
				dsp_set_last_command(dspPtr, map[i], cmd[i]);
			}		
			break;
		}
	}		
		
	return 0;
}

void load_points(int16 n_points, double * x, double * v, double * a, double * j,
	double time)
{
	int16 i, p = 0, axis;
	double
		 cmd_[PCDSP_MAX_AXES],
		 vel_[PCDSP_MAX_AXES],
		 acc_[PCDSP_MAX_AXES],
		 jerk_[PCDSP_MAX_AXES];
		 
	while (p < n_points)
	{
		for (i = 0; i < axes; i++)
		{	cmd_[i] = x[p];
			vel_[i] = v[p];
			acc_[i] = a[p];
			jerk_[i] = j[p];
		}
		
		if (load_frames(cmd_, vel_, acc_, jerk_, time))
			display(axes, map);
		else
			p++;
	}
}

void calc_points(double * x, double * v, double * a, double * j, double time)
{
	int16 i;
	
	/* Calculate points to generate a simple trapezoidal profile motion. */
	j[0] = 0.0;
	a[0] = ACCEL;
	v[0] = 0.0;
	x[0] = 0.0;
	
	j[1] = 0.0;
	a[1] = 0.0;
	v[1] = a[0] * time;
	x[1] = .5 * a[0] * time * time;
	
	j[2] = 0.0;
	a[2] = (-ACCEL);
	v[2] = v[1];
	x[2] = x[1] + (v[1] * time);
	
	j[3] = 0.0;
	a[3] = 0.0;
	v[3] = 0.0;
	x[3] = x[2] - (.5 * a[2] * time * time);
	
	for (i = 0; i < MAX_POINTS; i++)
		printf("\n%d J:%8.0lf A:%8.0lf V:%8.0lf X:%8.0lf", i, j[i], a[i], v[i], x[i]);
}

int16	main()
{
	int16 error_code; 
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
    error(dsp_reset());		/* hardware reset */

	printf("\nCalculating points...");
	calc_points(x, v, a, j, TIME);
	printf("\ndone.\n");

	printf("\nHit any key to move...\n");	
	getch();
	load_points(4, x, v, a, j, TIME);
	
	while(!end_of_motion(axes, map))
		display(axes, map);

    return 0;
}       
