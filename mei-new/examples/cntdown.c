/* CNTDOWN.C

:Demonstrates the 8254 counter/timer in count down mode.

This code toggles user I/O bit #0 when 'd' is pressed.  The hardware user
 I/O bit #0 should be connected to Clock 0 of Channel 0.  Gate 0 should
 be connected with a pull-up resistor (1K) to +5 volts.

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
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

int16	main()
{
	int16 error_code, key, done;
    unsigned16 value;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
    init_io(0, IO_OUTPUT);
    set_io(0, 0);
    init_timer(0, 0);
    set_timer(0, 2000);		/* load the counter/timer with a start value */

    printf("\nd=decrement timer, esc=exit\n");

	for (done = 0; !done; )
	{
        get_timer(0, &value);
        printf("Value: %7u\r", value);

		if (kbhit())		/* key pressed? */
		{	key = getch();

			switch (key)
			{
				case 'd':   /* decrement timer */
					set_bit(0);
                    reset_bit(0);
                    break;

				case 0x1B:	/* <ESC> */
					done = TRUE;
					break;
			}
		}
	}

    return 0;
}
