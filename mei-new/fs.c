/* FS.C

:Simple feed speed control

This code commands the AXIS to move at a constant velocity.  The feed speed
 is adjusted for all of the axes based on keyboard input.
 
Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"

# define	AXIS			0

# define	READY		    0
# define	INCREASE		2
# define	DECREASE		3

# define	RATE_CHANGE		.1

int16	state;
double feed_rate;

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


void CheckState(void)
{
	switch (state)
	{
        case INCREASE:
        	if (feed_rate < 2.0)
        	{	feed_rate += RATE_CHANGE;
				dsp_feed_rate(feed_rate);
			}	
            state = READY;
            break;

        case DECREASE:
        	if (feed_rate >= 0.0)
        	{	feed_rate -= RATE_CHANGE;
				dsp_feed_rate(feed_rate);
			}	
            state = READY;
            break;

        case READY:
            break;
    }
}


int16	main()
{
	int16 error_code, done, key;
    double cmd, act, err;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
    feed_rate = 1.0;
    v_move(AXIS, 1000.0, 10000.0);		/* accelerate to constant velocity */
    
    printf("\n +:increase, -:decrease, esc:quit\n");
    
	for (done = 0; !done; )
	{
		get_position(AXIS, &act);
		get_command(AXIS, &cmd);
		get_error(AXIS, &err);
		printf("CMD %6.0lf ACT %6.0lf ERR %6.0lf FEED %4.2lf\r", cmd, act, err, feed_rate);
		
		CheckState();

		if (kbhit())	/* key pressed? */
		{	key = getch();

			switch (key)
			{
                case '+':
                    state = INCREASE;
                    break;

                case '-':
                    state = DECREASE;
                    break;

				case 0x1B:	/* <ESC> */
					v_move(AXIS, 0.0, 10000.0);
                    done = TRUE;
					break;
			}
		}
	}
    return 0;
}
