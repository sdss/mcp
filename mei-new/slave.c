/* SLAVE.C

:Simple demonstration of commanded motion with linked axes.

This code links two axes together by actual position.  Then the master axis
 is put in Idle mode so its motor shaft can be turned by hand.  The DSP will
 command the slave axis to follow the master.  Then the slave is commanded to
 move.  Then the master is put into Run mode and commanded to move.

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
	Revision 1.1  1999/08/31 16:43:56  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"

# define	MASTER	0
# define	SLAVE	1
# define	RATIO	1.0

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

int16 display(void)
{
	double actual, command;
	
	while (!kbhit())				/* wait for a key press */
	{ 	get_command(SLAVE, &command);
		get_position(SLAVE, &actual);
		printf("Slave Cmd: %8.0lf Act: %8.0lf\r", command, actual);
	}
	getch();
	
	return 0;
}

int16	main()
{
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	mei_link(MASTER, SLAVE, RATIO, LINK_ACTUAL);
	controller_idle(MASTER);				/* disable PID control */
	printf("\nAxes linked, Master is in Idle mode.\n");
	display();
		
	start_move(SLAVE, 4000.0, 1000.0, 4000.0);	/* move the SLAVE axis */
	printf("\nCommanding slave axis to move\n");
	display();
	
	controller_run(MASTER);					/* enable PID control */

	start_move(MASTER, 4000.0, 1000.0, 4000.0);	/* move linked axes */
	start_move(MASTER, -4000.0, 1000.0, 4000.0);
	printf("\nCommanding master axis to move\n");
	display();	
	
	
	endlink(SLAVE);							/* unlink axes */

    return 0;
}
