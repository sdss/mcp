/* FLOOP4.C

:Demonstrates how to create/destroy a frame looping sequence.

This program downloads a looping frame profile to the controller.  The DSP
 continually executes this profile until an exception event occurs or 
 stop_frame_loop(...) is called.  Then the frame loop is cleared from the DSP's
 memory with release_frame_loop(...).

stop_frame_loop(...) stops the motion and prevents the frames from executing:
 
 1) Generate an E-Stop (either immediately or with the last frame).
 2) Wait for the E-Stop Event to complete.
 
release_frame_loop(...) disables the frames, breaks the frame loop, and releases
 the frames to the free list:
 
 1) Re-write the control, trigger/update, and action fields.  This is to
    guarantee that the DSP can step through the frames without executing them.
 2) Turn on the Hold bit in the second frame.  Set the gate.  This will force
    the DSP to halt at the first frame.
 3) Clear the status. 
 4) Wait for the DSP to reach the first frame.
 5) Turn on the release bit in each frame.
 6) Break the loop by setting the last frame's 'next' pointer to zero.
 7) Reset the gate flag.  The DSP will step through the frames one by one and
    release each one to the free list.
 8) Wait for all of the frames to be released to the free list.

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
	Revision 1.1  1999/08/31 16:43:04  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "pcdsp.h"
# include "idsp.h"

# define DISTANCE		2000.0
# define VELOCITY		10000.0
# define ACCELERATION	100000.0
# define DWELL_TIME		1.0

/* Frame register offsets - registers are 16 bits */
# define CONTROL		0x1
# define TRIG_UPDATE	0x11
# define ACTION			0x12


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

int16 display(int16 n_axes, int16 * map)
{
	int16 i;
	double cmd;

	printf("\nDsp frame buffer space available: %d\n", fifo_space());
	while (!kbhit())
	{
		for (i = 0; i < n_axes; i++)
		{ 	get_command(map[i], &cmd);
			printf("Ax:%d Cmd:%8.0lf  ", map[i], cmd);
		}
		printf("\r");
	}
	getch();
	
	return 0;
}

int16 download_profile(int16 n_axes, int16 * map, int16 * last_frame)
{
	int16 i, marker;

	set_gates(n_axes, map);
	for (i = 0; i < n_axes; i++)
	{
		dsp_control(map[i], FCTL_HOLD, TRUE);
		dsp_control(map[i], FCTL_RELEASE, FALSE);
		
		dsp_marker(map[i], &marker);
		dsp_control(map[i], FCTL_HOLD, FALSE);
		dsp_io_trigger(map[i], 0, FALSE);		/* wait for bit 0 low */
		dsp_dwell(map[i], 0);
		start_r_move(map[i], DISTANCE, VELOCITY, ACCELERATION);
		dsp_dwell(map[i], DWELL_TIME);
		last_frame[i] = dsp_goto(map[i], marker);
		
		dsp_control(map[i], FCTL_RELEASE, TRUE);
	}

	return 0;
}

int16 stop_frame_loop(int16 n_axes, int16 * map, int16 * last_frame, int16 immediate)
{
	int16 i;

	for(i = 0; i < n_axes; i++)
	{ 	if (immediate)
			set_e_stop(map[i]);
		else	
			dsp_write_dm(last_frame[i] + ACTION, E_STOP_EVENT);
	}	
	
	/* Wait for the E-Stop Event to complete */
	for (i = 0; i < n_axes; i++)
		while((axis_state(map[i]) != E_STOP_EVENT) || 
			(axis_status(map[i]) & IN_MOTION) ||
			(axis_status(map[i]) & IN_SEQUENCE));

	return dsp_error;
}

int16 disable_frame_loop(int16 n_axes, int16 * map, int16 * last_frame)
{
	int16 i, control, first_frame[PCDSP_MAX_AXES], fp, fp2;

	for (i = 0; i < n_axes; i++)
	{ 	fp = first_frame[i] = dsp_read_dm(last_frame[i]);
		fp2 = dsp_read_dm(fp);				/* address of second frame */
		
		/* Disable the looping frames */
		do
		{ 	dsp_write_dm(fp + CONTROL, (FCTL_DEFAULT & ~FCTL_RELEASE));
			dsp_write_dm(fp + TRIG_UPDATE, 0);
			dsp_write_dm(fp + ACTION, 0);
			fp = dsp_read_dm(fp);		/* address of next frame */
		}
		while(fp != first_frame[i]);
		
		/* Turn on the hold bit in the second frame */
		dsp_write_dm(fp2 + CONTROL, (FCTL_DEFAULT & ~FCTL_RELEASE | FCTL_HOLD));
		set_gate(map[i]);
		error(clear_status(map[i]));	
	}

	/* wait for the DSP to reach the first frame */
	for(i = 0; i < n_axes; i++)
		while (first_frame[i] != (dsp_read_dm(dspPtr->outptr + map[i])))
			;
	
	for(i = 0; i < n_axes; i++)
	{ 	fp = first_frame[i];
		
		/* Turn on the release bit in the looping frames */
		do
		{ 	control = dsp_read_dm(fp + CONTROL);	
			dsp_write_dm(fp + CONTROL, (control | FCTL_RELEASE));	
			fp = dsp_read_dm(fp);			/* address of next frame */
		}
		while(fp != first_frame[i]);
		
		dsp_write_dm(last_frame[i], 0x0);	/* break the loop */
		reset_gate(map[i]);					/* release the frames */
	}

	for(i = 0; i < n_axes; i++)
		while(axis_status(map[i]) & FRAMES_LEFT)
			;
	
   	return dsp_error;
}

int16	main()
{
	int16 i, error_code, axes, axis_map[] = {0, 1}, last_frame[PCDSP_MAX_AXES];
	
	axes = (sizeof(axis_map)/sizeof(int16));
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	init_io(0, IO_INPUT);
	
	download_profile(axes, axis_map, last_frame);
	
	printf("\n\nPress any key to start the frame loop sequence.");
	display(axes, axis_map);
	reset_gates(axes, axis_map);

	printf("\n\nPress any key to stop the frame loop sequence.");
	display(axes, axis_map);
	stop_frame_loop(axes, axis_map, last_frame, TRUE);

	printf("\n\nPress any key to release the frames.");
	display(axes, axis_map);
	disable_frame_loop(axes, axis_map, last_frame);
	printf("\n\nDsp frame buffer space available: %d\n", fifo_space());
	
    return 0;
}
