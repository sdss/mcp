/* ALLEVENT.C

:Configure the DSP to generate exception events on all axes.

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
 
This code configures the Stopped frame for each axis to generate an Abort Event
 on the next sequential axis.  Thus, if an Abort Event is generated on any axis,
 then Abort Events will be generated on all axes.  Also, if a Stop or E-Stop 
 Event is generated on an axis, then Abort Events will be generated on all axes
 (after the original axis decelerates to a stop).
 
Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
*/

/*	Revision Control System Information
	$Source$ 
	$Revision$
	$Date$

	$Log$
	Revision 1.1  1999/08/31 16:42:52  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"
# include "idsp.h"


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
	int16 error_code, axis, axes, dest_axis;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	axes = dsp_axes();

	for (axis = 0; axis < axes; axis++)
	{
		dest_axis = (axis + 1) % axes;
		set_stopped_frame(axis, dest_axis, ABORT_EVENT);
		printf("\n%d %d", axis, dest_axis);
	}	
	
    return 0;
}
