/* MASYNC.C

:Multiple axis synchronized motion.

This code demonstrates how to generate sequences of multiple axis S-Curve
 profile motion.  The execution of the profiles are synchronized so that all
 of the axes start in the same sample.  

The steps are:
 1) Set the DSP's gate flag for each axis.
 2) Download the profiles.  Be sure to turn on the Hold flag in the first
	frame of each sequence.  Also, load a dwell frame and an end sequence
	frame on axes that are not commanded to move.
 3) Reset the DSP's gate flag for each axis.  The frames will execute.
 4) Wait for the dwell frame and end sequence frames to execute (about two
 	samples).
 5) Download the next frame sequence for each axis.
 6) Wait for the previous motion to complete.
 7) Reset the DSP's gate flag for each axis.  The next frame sequences will
	execute.   	
 
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
	Revision 1.1  1999/08/31 16:43:29  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include "pcdsp.h"
# include "idsp.h"


double
	pos[] = {50000.0, 60000.0, 85000.0, 90000},
	vel[] = {4000.0, 4000.0, 4000.0, 4000.0},
	accel[] = {16000.0, 16000.0, 16000.0, 16000.0},
	jerk[] = {100000.0, 100000.0, 100000.0, 100000.0}; 

	
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
    { 	
    	get_command(map[i], &cmd);
        printf("%d: %10.0lf ", map[i], cmd);
    }
    printf("\r");
}    

int16 seq_done(int16 n_axes, int16 mask, int16 * map)
{
	int16 i, axis;
	
	/* Check if the frame sequence on the non-motion axes is completed.
		This check is required before calling set_gates(...) to guarantee
		the synchronization of the axes.
	*/
	
	for (i = 0; i < n_axes; i++)
	{	
		axis = map[i];
		if ((~mask & (1 << axis)) && (!motion_done(axis)))
			return 0;
	}	
	return 1;
}

int16 moving(int16 n_axes, int16 * map)
{
	int16 i;
	
	for (i = 0; i < n_axes; i++)
	{	
		if (in_motion(map[i]))
			return 1;
	}		
	return 0;
}

int16 multi_s_move(int16 n_axes, int16 mask, int16 * map)
{
	int16 axis, i;

	set_gates(n_axes, map);		/* prevent frames from executing */
	
	for (i = 0; i < n_axes; i++)
	{
		axis = map[i];
		if (mask & (1 << axis))
			start_s_move(axis, pos[i], vel[i], accel[i], jerk[i]);
		else
		{	/* Turn on the Hold bit in the first frame */
			dsp_control(axis, FCTL_HOLD, TRUE);
			dsp_dwell(axis, 0.0);			/* one sample delay */
			dsp_control(axis, FCTL_HOLD, FALSE);
			dsp_end_sequence(axis);
		}
	}
	
	/* Wait for the previous motion to complete before starting this one */
	while (moving(n_axes, map))
		display(n_axes, map);
			
	reset_gates(n_axes, map);	/* ready, set, go! */
	
	return 0;
}

int16	main()
{
	int16 error_code, n_axes, axis_mask, axes[] = {0, 1, 2, 3};
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */
	
	n_axes = (sizeof(axes) / sizeof(int16));

	axis_mask = 3;			/* axes 0 and 1 */
	multi_s_move(n_axes, axis_mask, axes);
	while (!seq_done(n_axes, axis_mask, axes))
		;

	axis_mask = 1;			/* axis 0 */
	pos[0] = 110000.0;
	multi_s_move(n_axes, axis_mask, axes);
	while (!seq_done(n_axes, axis_mask, axes))
		;

	axis_mask = 3;			/* axes 0 and 1 */
	pos[0] = 0.0;
	pos[1] = 0.0;
	multi_s_move(n_axes, axis_mask, axes);
	while (!seq_done(n_axes, axis_mask, axes))
		;
	while (moving(n_axes, axes))
		display(n_axes, axes);
		
    return 0;
}
