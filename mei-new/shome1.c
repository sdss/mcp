/* SHOME1.C

:SERCOS sample homing routine.

This sample demonstrates a basic homing algorithm.  The home procedure
 (IDN 148) is activated in the servo drive.  The drive accelerates
 (IDN 42) the axis to constant velocity (IDN 41) with internal position
 control.  When the home switch is activated, the drive decelerates the
 axis to a stop and sets a bit in the "Position Feedback Status" (IDN 403).

Then the command position (IDN 47) is read from the drive and the
 controller's command/actual position registers are set with
 set_position(...).  Finally, the drive procedure is ended with
 cancel_exec_procedure(...).

Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before starting a drive controlled home procedure.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5
*/

	
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "sercos.h"

#define AXIS			0
#define HOME_OFFSET		0L

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


int	main()
{	int16 error_code;
	long pos, done = 0;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	set_error_limit(AXIS, 0.0, NO_EVENT);	/* disable controller error limit */
	enable_amplifier(AXIS);					/* enable drive */

  	error(set_idn(AXIS, 52, HOME_OFFSET));	/* home reference distance */
	error(start_exec_procedure(AXIS, 148));	/* start home procedure */
	
	printf("\nMoving towards home switch (hit any key to quit)");
	while((!done) && (!kbhit()))
		error(get_idn(AXIS, 403, &done));	/* position reference status */

	if (done)
		printf("\nHome found");	

	error(get_idn(AXIS, 47, &pos));			/* command position */
	set_position(AXIS, (double)pos);
	error(cancel_exec_procedure(AXIS, 148));	/* end the home procedure */

	set_error_limit(AXIS, 32767.0, ABORT_EVENT);

	return 0;
}
