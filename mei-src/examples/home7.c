/* HOME7.C

:Homing routine to find midpoint between positive and negative limits.

Here is the algorithm:
 1) Move to negative limit, store position.
 2) Move to positive limit, store position.
 3) Calculate total width of system.
 4) Set position referenced from zero. (middle position)

Here is the sensor logic:
 Home input = not used
 Index input = not used
 +Limit input = active high
 -Limit input = active high

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <math.h>
# include "pcdsp.h"

# define    AXIS        0
# define    POS_VEL     10000.0
# define    NEG_VEL     (-10000.0)
# define    ACCEL       50000.0    


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
	int16 home_logic, state;
	double cmd;
	
	while (!motion_done(axis))
	{   
		get_command(axis, &cmd);
		printf("\rCmd: %10.0lf Positive Limit: %d Negative Limit: %d", cmd, pos_switch(AXIS), neg_switch(AXIS));    
		
		
		if (kbhit())
		{   getch();
			exit(1);
		}   
	}
	printf("\n");
}


int16 initialize_limits(int16 axis)
{
	set_positive_limit(axis, STOP_EVENT);    
	set_positive_level(axis, TRUE);
	
	set_negative_limit(axis, STOP_EVENT);   
	set_negative_level(axis, TRUE);
	
	return 0;
}


int16 find_neg_limit(int16 axis, double * min_pos)
{
	v_move(axis, NEG_VEL, ACCEL);            /* move to negative limit */
	printf("Moving to negative limit...\n");
	display(axis);
	get_position(axis, min_pos);                 
	error(clear_status(axis));  
	
	return 0;
}


int16 find_pos_limit(int16 axis, double * max_pos)
{
	v_move(axis, POS_VEL, ACCEL);            /* move to positive limit */
	printf("Moving to positive limit...\n");
	display(axis);
	get_position(axis, max_pos);                 
	error(clear_status(axis));  
	
	return 0;
}


int16 main()
{
	int16 error_code;
	double min_pos, max_pos, width;   

	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */

	initialize_limits(AXIS);                 /* configure limit switches */

	find_neg_limit(AXIS, &min_pos);
	find_pos_limit(AXIS, &max_pos);
	width = (fabs(min_pos) + fabs(max_pos));
	printf("Width of system: %lf \n",width); 
	
	/* reference position from middle (zero) of system */

	set_position(AXIS, (width / 2.0));       /* position at positive limit */
	printf("Current Position: %lf\n", (width / 2));

	return 0;
}
