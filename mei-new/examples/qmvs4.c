/* QMVS4.C

:Download multiple frame sequences and selectively execute them.

This sample shows how to download a fixed set of trapezoidal profile motions
 and switch between them by adjusting the next pointer in the dsp_goto(...) 
 frames.
 
Here are the steps to execute several frame sequences:

 1) Download several frame sequences.  Each motion frame sequence starts with a
  	"dsp_marker(...)" frame and ends with a "dsp_goto(...)" frame.  Be sure to
  	turn the hold bit on in the first frame of each sequence.
 2) Set the "next" pointer in the "dsp_goto(...)" frame to select the next
 	sequence.	
 3) Toggle the "gate" to execute the motion.

To destroy the frame sequences, first call connect_sequence(...).  This will 
 connect all of the frame sequences together into one long sequence.  Then call
 stop_frame_loop(...).  This function stops the motion and prevents the frames
 from executing.  Motion can be stopped immediately or with the last frame.
 
Then call disable_frame_loop(...).  This function disables the frames, breaks
 the frame loop, and releases the frames to the free list:
 
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


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "pcdsp.h"
# include "idsp.h"

# define SAMPLE_RATE 	1250	/* samples per second */

# define AXIS			0
# define MOVES			3		/* number of moves */
# define MAX_FRAMES		10

double distance[MOVES] = {4096.0, 8192.0, -32768.0}; 
double velocity[MOVES] = {10000.0, 10000.0, 10000.0}; 
double accel[MOVES] = {500000.0, 500000.0, 500000.0}; 

int16 start[MOVES];
int16 end[MOVES];

/* Frame register offsets - registers are 16 bits */
# define CONTROL		0x1
# define TRIG_UPDATE    0x11
# define ACTION         0x12


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
	double cmd;

	get_command(axis, &cmd);
	printf("\rCmd:%8.0lf", cmd);
}

void go(int16 axis)
{
	int16 sample_time;
	
	reset_gate(axis);			/* ready, set, go! */
	
	dsp_write_dm(0x100, 0x0);	/* write to DSP's signature word */
	while (!dsp_read_dm(0x100))	/* wait for DSP to write signature */
		;
	dsp_write_dm(0x100, 0x0);
	while (!dsp_read_dm(0x100))
		;

	set_gate(axis);				/* prevent sequence from executing twice */
}

int16 set_first_move(int16 axis, int16 next, int16 * start)
{
	int16 addr;
		
	/* Set the 'next' pointer in the place holder frame. */
	addr = dsp_read_dm(dspPtr->outptr + axis);
	dsp_write_dm(addr, start[next]);

	return next;
}

int16 set_next_move(int16 current, int16 next, int16 * start, int16 * end)
{
	/* Set the 'next' pointer in the dsp_goto(...) frame. */
	dsp_write_dm(end[current], start[next]);
	
	return next;
}

void connect_sequence(int16 * start, int16 * end)
{
	int16 i, next;

	/* Connect all of the frame sequnces together. */
	for (i = 0; i < MOVES; i++)
	{
		next = (i + 1) % MOVES;
		dsp_write_dm(end[i], start[next]);
	}	
}

int16 stop_frame_loop(int16 axis, int16 last_frame, int16 immediate)
{
	if (immediate)
		set_e_stop(axis);
	else	
		dsp_write_dm(last_frame + ACTION, E_STOP_EVENT);
	
	/* Wait for the E-Stop Event to complete */
	while((axis_state(axis) != E_STOP_EVENT) || in_motion(axis))
		;

	return dsp_error;
}

int16 disable_frame_loop(int16 axis, int16 last_frame)
{
	int16 i, control, first_frame, fp, fp2;

	fp = first_frame = dsp_read_dm(last_frame);
	fp2 = dsp_read_dm(fp);				/* address of second frame */
		
	/* Disable the looping frames */
	do
	{ 	dsp_write_dm(fp + CONTROL, (FCTL_DEFAULT & ~FCTL_RELEASE));
		dsp_write_dm(fp + TRIG_UPDATE, 0);
		dsp_write_dm(fp + ACTION, 0);
		fp = dsp_read_dm(fp);		/* address of next frame */
	}
	while(fp != first_frame);
		
	/* Turn on the hold bit in the second frame */
	dsp_write_dm(fp2 + CONTROL, (FCTL_DEFAULT & ~FCTL_RELEASE | FCTL_HOLD));
	set_gate(axis);
	while(clear_status(axis))
		;	

	/* wait for the DSP to reach the first frame */
	while (first_frame != (dsp_read_dm(dspPtr->outptr + axis)))
		;
	
	fp = first_frame;
		
	/* Turn on the release bit in the looping frames */
	do
	{ 	control = dsp_read_dm(fp + CONTROL);	
		dsp_write_dm(fp + CONTROL, (control | FCTL_RELEASE));	
		fp = dsp_read_dm(fp);			/* address of next frame */
	}
	while(fp != first_frame);
		
	dsp_write_dm(last_frame, 0x0);		/* break the loop */
	reset_gate(axis);					/* release the frames */

	while(axis_status(axis) & FRAMES_LEFT)
		;
	
   	return dsp_error;
}

int16 initialize_frames(int16 axis, int16 * start, int16 * end)
{
	int16 i, j, control, last_frame, frame_addr[MAX_FRAMES];

	set_gate(axis);		/* prevent frames from executing */
	
	for (i = 0; i < MOVES; i++)
	{	
		dsp_control(axis, FCTL_RELEASE, FALSE);	/* turn off the release bit */
		dsp_control(axis, FCTL_HOLD, TRUE);		/* turn on the hold bit */
		frame_m(NULL, "0l * n d", axis, start + i, 0);	/* place holder */
		dsp_control(axis, FCTL_HOLD, FALSE);	/* turn off the hold bit */
		
		start_r_move(axis, distance[i], velocity[i], accel[i]);
		
		dsp_goto(axis, 0x0);	/* We don't know where to go yet! */

		/* Turn off the hold bit in the frames, find the last frame. */ 
		j = 1;
		frame_addr[j] = dsp_read_dm(start[i]);
		while ((dsp_read_dm(frame_addr[j])) && (j < MAX_FRAMES))
		{
			frame_addr[j+1] = dsp_read_dm(frame_addr[j]);
			control = dsp_read_dm(frame_addr[j] + CONTROL); 
			dsp_write_dm(frame_addr[j] + CONTROL, control & ~FCTL_HOLD);
			j++;
		}
		last_frame = end[i] = frame_addr[j];
	}	
	dsp_control(axis, FCTL_RELEASE, TRUE);	/* turn the release bit back on */

	return last_frame;
}	

void initialize(void)
{
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */
	
	set_sample_rate(SAMPLE_RATE);
}

int16	main()
{
	int16 done, key, current, next, last_frame, first_move = 1;
	
	initialize();
	last_frame = initialize_frames(AXIS, start, end);
	current = 0;		/* first move type */

	printf("\n\n(0-%d)=move type, g=go, d=disable frames, esc=quit\n", MOVES-1);

	for (done = 0; !done; )
	{
		display(AXIS);

		if (kbhit())     /* key pressed? */
		{	key = getch() ;

			switch (key)
			{
				case 'g':
			 		go(AXIS);	/* Be sure the next move is defined! */	
			 		first_move = FALSE;
					break;

				case 'd':
					connect_sequence(start, end);	
					stop_frame_loop(AXIS, last_frame, TRUE);
					disable_frame_loop(AXIS, last_frame);
					printf("\nFrame loop disabled.\n");
					break;

				case 0x1B:      /* <ESC> */
					done = TRUE;
					break;

				default:
					next = (key - 48);
					if ((next >= 0) && (next < MOVES))
					{
						printf("\nCurrent: %d Next: %d Distance: %8.0lf \n",
							current, next, distance[key-48]);

						if (first_move)			
							current = set_first_move(AXIS, next, start);
						else	
							current = set_next_move(current, next, start, end);
					}	
					break;
			}
		}
	}
    return 0;
}
