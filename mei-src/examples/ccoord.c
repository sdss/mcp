/* CCOORD.C

:Some simple circular coordinated moves

This code initializes the mapped axes, the vector velocity, and the vector
 acceleration.  Then a single circular coordinated move is performed followed
 by a few interpolated circular coordinated motion.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"


# define	MAX_AXES	2

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
	double x, y;
    
    while (!kbhit())
    {   get_command(0, &x);
        get_command(1, &y);
        printf("X: %12.4lf Y: %12.4lf\r", x, y);
    }
    getch();

    return 0;
}

int16	main()
{
	int16 error_code, axes[MAX_AXES] = {0, 1};
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

    error(dsp_reset());		/* reset the hardware */

	error(map_axes(MAX_AXES, axes));	/* initialize the coordinated axes */
	set_move_speed(200.0);				/* set the vector velocity */
	set_move_accel(8000.0);				/* set the vector acceleration */
	set_arc_division(10.0);				/* arc segment every 10 degrees */

	arc_2(0.0, 100.0, 90.0);			/* perform a single coordinated move */
    display();
	
	/*  Perform an interpolated motion through several points */	
	start_point_list();
    arc_2(100.0, 0.0, 45.0);
    arc_2(200.0, 100.0, -90.0);
    
    end_point_list();
    
    start_motion();
    
    display();

    return 0;
}
