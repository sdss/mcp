/* LCOORD.C

:Some simple linear coordinated moves

This code initializes the mapped axes, the vector velocity, and the vector
 acceleration.  Then a single linear coordinated move is performed followed
 by an interpolated linear coordinated motion.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"


# define	MAX_AXES	3

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
	double x, y, z;
    
    while (!kbhit())
    {   get_command(0, &x);
        get_command(1, &y);
        get_command(2, &z);
        printf("X: %12.4lf Y: %12.4lf Z: %12.4lf\r", x, y, z);
    }
    getch();

    return 0;
}

int16	main()
{
	int16 error_code, axes[MAX_AXES] = {0, 1, 2};
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

    error(dsp_reset());		/* reset the hardware */

	error(map_axes(MAX_AXES, axes));	/* initialize the coordinated axes */
	set_move_speed(200.0);				/* set the vector velocity */
	set_move_accel(8000.0);				/* set the vector acceleration */

	move_3(100.0, 100.0, 100.0);		/* perform a single coordinated move */
    display();
	
	/*  Perform an interpolated motion through several points */	
	start_point_list();
    move_3(100.0, 100.0, 100.0);
    move_3(200.0, 1000.0, 1000.0);
    move_3(50.0, 50.0, 50.0);
    end_point_list();
    
    start_motion();
    
    display();

    return 0;
}
