/* SCANADT.C

:Trapezoidal profile motion and capture analog values at specific times.

This code configures the DSP to generate a trapezoidal profile motion and
 capture analog values at specific time intervals.  The minimum time interval
 is one sample (800 microseconds for the default sample rate).
 
The actual position (analog value) capturing is handled automatically by the
 DSP.  After each frame is executed, the DSP copies the actual position into
 the frame and releases it to the free list.  Later, the captured positions
 are retrieved from the frames.  There are 600 frames in the DSP's external
 memory. 
 
The steps are:
 1) Configure an axis for analog feedback.
 2) Download a trapezoidal profile motion.
 3) Download a sequence of dwell frames and read their addresses.  The addresses
 	will be needed later to read the captured positions.
 4) Read the captured positions (analog values) from the frames.
 
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
	Revision 1.1  1999/08/31 16:43:46  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"
# include "idsp.h"

# define MOVE_AXIS		0
# define ANALOG_AXIS	2
# define ANALOG_CHAN	0

# define DISTANCE		30000.0
# define VELOCITY		10000.0
# define ACCEL			100000.0

# define CAPTURES		30
# define TIME_INTERVAL	.1		/* units = seconds, minimum is one DSP sample */

# define RELEASE_POS_2	0x2
# define RELEASE_POS_3	0x3

int16 scan_addr[CAPTURES];
long scan_pos[CAPTURES];


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

void load_scan_frames(int16 axis, int16 captures, double interval)
{
	int16 i;
	
	dsp_control(axis, FCTL_HOLD, TRUE);
	dsp_dwell(axis, interval);
	scan_addr[0] = dsp_read_dm(dspPtr->inptr + axis);		/* frame address */
	dsp_control(axis, FCTL_HOLD, FALSE);
	
	for (i = 1; i < captures; i++)
	{ 	dsp_dwell(axis, interval);
		scan_addr[i] = dsp_read_dm(dspPtr->inptr + axis);	/* frame address */
	}	
	dsp_end_sequence(axis);
}

void read_positions(int16 captures)
{
	int16 i;
	unsigned16 lower;
	long upper;

	for (i = 0; i < captures; i++)
	{ 	
		lower = dsp_read_dm(scan_addr[i] + RELEASE_POS_2);
		upper = dsp_read_dm(scan_addr[i] + RELEASE_POS_3);
		scan_pos[i] = (lower | (upper << 16));
		printf("\nAddr: 0x%x Scan: %ld", scan_addr[i], scan_pos[i]);
	}	
}

int16	main()
{
	int16 error_code, axes[] = {MOVE_AXIS, ANALOG_AXIS};
	double position;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	/* Configure an axis for analog feedback. */
	set_feedback(ANALOG_AXIS, FB_ANALOG);
	set_analog_channel(ANALOG_AXIS, ANALOG_CHAN, FALSE, FALSE);

	set_gates(2, axes);		/* Prevent frame execution */
	load_scan_frames(ANALOG_AXIS, CAPTURES, TIME_INTERVAL);
	start_r_move(MOVE_AXIS, DISTANCE, VELOCITY, ACCEL);
	reset_gates(2, axes);	/* ready, set, go! */
	printf("\nScanning positions\n");	

	while (!motion_done(MOVE_AXIS) || !motion_done(ANALOG_AXIS))
	{ 	get_command(MOVE_AXIS, &position);
		printf("\rPosition: %8.0lf ", position);
	}	
	read_positions(CAPTURES);

    return 0;
}
