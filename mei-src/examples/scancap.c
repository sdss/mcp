/* SCANCAP.C

:Velocity profile scan to capture positions.

This code configures the DSP to generate a velocity profile and capture
 positions at specific time intervals.  The minimum time interval is one sample
 (800 microseconds for the default sample rate).  The position capturing is
 handled automatically by the DSP.  After each frame is executed, the DSP
 copies the actual position into the frame and releases it to the free list.
 Later, the captured positions are retrieved from the frames.  There are 600
 frames in the DSP's external memory. 
 
The steps are:
 1) Download a velocity profile motion.
 2) Download a sequence of dwell frames and read their addresses.  These will be
  	needed later to read the captured positions.
 3) Read the captured positions from the frames.
 
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

# define AXIS			0
# define VELOCITY		20000.0
# define ACCEL			100000.0

# define CAPTURES		100
# define TIME_INTERVAL	.01		/* units = seconds, minimum is one DSP sample */

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

void load_profile(int16 axis, int16 captures, double interval)
{
	int16 i;
	
	/* Turn on the axis gate flag to prevent frame execution */
	set_gate(axis);	
	v_move(axis, VELOCITY, ACCEL); 
	
	for (i = 0; i < captures; i++)
	{ 	
		dsp_dwell(axis, interval);
		scan_addr[i] = dsp_read_dm(dspPtr->inptr + axis);	/* frame address */
	}	
	v_move(axis, 0.0, ACCEL);
	
	reset_gate(axis);	/* ready, set, go! */
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
	int16 error_code;
	double position;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	load_profile(AXIS, CAPTURES, TIME_INTERVAL);
	printf("\nScanning positions\n");	

	while (!motion_done(AXIS))
	{
		get_position(AXIS, &position);
		printf("\rPosition: %8.0lf ", position);
	}	
	read_positions(CAPTURES);

    return 0;
}
