/* ACTVEL1.C

:Read, calculate and print actual velocity over a 100 millisecond period.

A simple move is commanded and the actual velocity (counts/sec) is printed 
 to the screen.  The velocity is calculated using get_position() twice 
 and dividing by the time interval using dsp_read_dm().

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/

/*  Revision Control System Information
	$Source$ 
	$Revision$
	$Date$

	$Log$
	Revision 1.1  1999/08/31 16:42:49  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "pcdsp.h"


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

double actual_velocity(int16 axis)
{
	double position1, position2, vel;
	int16 transfer_time1, transfer_time2;

	get_position(axis, &position1);
	transfer_time1 = dsp_read_dm(0x011E); /* read data transfer sample time */
	delay(100);
	get_position(axis, &position2);
	transfer_time2 = dsp_read_dm(0x011E);
	
	vel = (((position2-position1)/(transfer_time2-transfer_time1))
				* dsp_sample_rate());
	
	return vel;
}


int16 main()
{
	int16 error_code;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	error(dsp_reset());
	
	printf("Press a key to exit program \n");
	start_move(0, 100000.0, 10000.0, 100000.0);
	
	while(!kbhit())
	{
		printf(" %lf \r",actual_velocity(0));
	}
	getch();
	return 0;
}
