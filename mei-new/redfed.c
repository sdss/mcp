/* REDFED.C

:Redundant feedback checking.

This code demonstrates how to configure the DSP to check for invalid feedback
 based on two feedback devices.  The first feedback device is connected to
 the "Real" axis.  The second feedback device is connected to the "Redundant"
 axis.  Motion is commanded on the "Real" axis.  Both feedback devices are
 connected to the same mechanical system. 

Here are the steps to configure feedback checking:

 1) Set the "Real" and "Redundant" actual positions equal.
 2) Positionally link the "Real" and "Redundant" axes.
 3) Configure the error limits and event responses for the axes.
 4) Configure the "Redundant" axis to send an exception event to the "Real" axis
  	when an exception event occurs.	
   
If the difference between "Real" axis' command position and the actual position
 exceed the error limit, an exception event will be generated on the "Real"
 axis.  If the difference between the actual positions of the axes exceed the
 error limit an exception event will be generated on the "Redundant" axis.
 If an exception event occurs on the "Redundant" axis, an event will be sent
 to the "Real" axis.  The DSP handles the error limit checking and exception
 event generation.
 
The Stop, E-Stop, and Abort Events are executed by the DSP based on an axis'
 limit switches, home switch, software limits, error limits, etc.  Internally,
 the DSP executes some special frames to complete the exception event.

When a Stop Event or E-Stop Event is generated, the DSP executes a Stop or
 E-Stop frame to decelerate the axis.  Then the DSP executes the Stopped frame
 to set the command velocity to zero.  An Abort Event puts the axis in idle
 mode (no PID update) and executes the Stopped frame.  See the "C Programming"
 manual for more information about Exception Events.

By modifying an axis' Stopped exception frame, an exception event can be sent to
 another axis by the DSP after a Stop, E-Stop, or Abort Event executes.

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
	Revision 1.1  1999/08/31 16:43:44  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"
# include "idsp.h"

# define REAL_AXIS			0
# define REDUNDANT_AXIS		1

# define RATIO				(-1.0)
# define ERROR_LIMIT		20.0
# define ACT_DIFF_LIMIT		30.0


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

void display(int16 axis1, int16 axis2)
{
	int16 state1, state2;
	double act1, act2;

	while (!kbhit())
	{
		get_position(axis1, &act1); 
		get_position(axis2, &act2); 
		state1 = axis_state(axis1);
		state2 = axis_state(axis2);
		printf("Pos: %8.0lf State: %d Pos: %8.0lf State: %d\r", act1, state1,
			act2, state2);
	}
	getch();
}

int16 config_stopped_frame(PFRAME f, int16 address, int16 ormask, int16 andmask)
{
	int16 update = f->f.trig_update & 0xFF;	/* get trigger/update field */
	int16 * i = (int16*) &(f->f.position);	/* pointer to position field */

	if (!(update & FUPD_POSITION))
	{
		f->f.output = OUTPUT_POSITION ;		/* set output to use position field */
		i[0] = address;
		i[1] = ormask;
		i[2] = andmask;
		f->f.trig_update |= FUPD_OUTPUT;	/* update register from position field */
		pcdsp_write_ef(dspPtr, f);			/* download the exception frame */
		return DSP_OK ;
	}
	return DSP_RESOURCE_IN_USE;
}

int16 set_stopped_frame(int16 axis, int16 destaxis, int16 event)
{
	FRAME f;

	event |= (ID_AXIS_COMMAND << 4);
	
	if (pcdsp_sick(dspPtr, axis) || pcdsp_sick(dspPtr, destaxis))
		return dsp_error;
		
	if (pcdsp_read_ef(dspPtr, &f, axis, EF_STOPPED))
		return dsp_error;
		
	if (config_stopped_frame(&f, dspPtr->pc_event + destaxis, event, event))
		return dsp_error;
	else
		return pcdsp_write_ef(dspPtr, &f);
} 

int16	main()
{
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());

	set_position(REAL_AXIS, 0.0);		/* synchronize position registers */
	set_position(REDUNDANT_AXIS, 0.0);
	
	set_positive_sw_limit(REDUNDANT_AXIS, 100000.0, NO_EVENT);
	set_negative_sw_limit(REDUNDANT_AXIS, 100000.0, NO_EVENT);
	set_error_limit(REAL_AXIS, ERROR_LIMIT, ABORT_EVENT);
	set_error_limit(REDUNDANT_AXIS, ACT_DIFF_LIMIT, ABORT_EVENT);

	set_stopped_frame(REDUNDANT_AXIS, REAL_AXIS, ABORT_EVENT);
	
	mei_link(REAL_AXIS, REDUNDANT_AXIS, RATIO, LINK_ACTUAL);
	display(REAL_AXIS, REDUNDANT_AXIS);
	
	endlink(REAL_AXIS);

    return 0;
}
