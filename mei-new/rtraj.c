/* RTRAJ.C

:Read real-time trajectory information from the DSP.

This code shows how to use get_tuner_data(...) to sample the command positions,
 actual positions, error, time, state, and dac output.  Any motion can be 
 commanded while the real-time response is sampled.

This feature is very useful in examining the system response to determine
 optimum tuning parameters.

Note: get_tuner_data(...) is an "extra" function and is not documented in the
 "C Programming" manual.

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
	Revision 1.1  1999/08/31 16:43:45  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <dos.h>
# include <conio.h>
# include "pcdsp.h"
# include "extras.h"


# define AXIS				0
# define BUFFER_SECONDS		5


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

int16	main()
{
	int16 i, error_code;
	size_t buffer_length;

	long
		* apos,
		* cpos;
	int16
		* time,
		* state,
		* voltage;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
	buffer_length = BUFFER_SECONDS * dsp_sample_rate();

	apos = calloc(buffer_length, sizeof(apos[0]));
	cpos = calloc(buffer_length, sizeof(cpos[0]));
	time = calloc(buffer_length, sizeof(time[0]));
	state = calloc(buffer_length, sizeof(state[0]));
	voltage = calloc(buffer_length, sizeof(voltage[0]));

	if (!(apos && cpos && time && state && voltage))
	{	fprintf(stderr, "Not enough memory.\n");
		return 1;
	}

	fprintf(stderr, "Press any key to begin.\n");
	getch();

	fprintf(stderr, "Sampling...");

	/*  ****************************************************  **
		put whatever type of motion you want to sample here.
	**  ****************************************************  */

	get_tuner_data(AXIS, buffer_length, apos, cpos, time, state, voltage);

	fprintf(stderr, "done.\n");

	printf("apos\tcpos\ttime\tstate\tvoltage\n");
	for (i = 0; i < buffer_length; i++)
	{	printf("%ld\t%ld\t%u\t%d\t%d\n",
			apos[i], cpos[i], time[i], state[i], voltage[i]);
	}

	return 0;
}
