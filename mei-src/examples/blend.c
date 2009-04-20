/* BLEND.C

:Sample to adjust an axis' profile with a phantom axis.

Axis 0 is the command axis, which is linked to Axis 3's command position.
  While axis 0 performs a long move, axis 3 is advanced based on keyboard
  input.  All that is required from axis 3 is its command position.  The
  simulated axis is configured by downloading 4axis.abs (4 axis firmware)
  to a 3 axis card.

Note: Disable all hardware and software limits for the phantom axis.

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "pcdsp.h"

# define	READY		0
# define	GO			1
# define	MOVE_FWD	2
# define	MOVE_BACK	3

# define	PHANTOM		3	/* simulated axis */
# define	SLAVE		0

int16	state;	/* one state variable for all axes. */

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

int16 CheckState(void)
{
    double act, offset;
    
    get_position(SLAVE, &act);
    get_command(PHANTOM, &offset);
    printf("Axis: %8.0lf Offset: %8.0lf\r", act, offset);
    
	switch (state)
	{
		case GO:
			start_move(SLAVE, 20000.0, 1000.0, 10000.0);
			start_move(SLAVE, 0.0, 1000.0, 10000.0);
			state = READY;
			break;
			
        case MOVE_FWD:
            start_r_move(PHANTOM, 500.0, 1000.0, 10000.0);
            state = READY;
            break;

        case MOVE_BACK:
            start_r_move(PHANTOM, -500.0, 1000.0, 10000.0);
            state = READY;
            break;

       	case READY:
       		break;

    }
    return(0);
}

int16 main()
{
	int16 error_code, axis, done, key;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
    mei_link(PHANTOM, SLAVE, 1.0, LINK_COMMAND);
    printf("\ng=go, f=fwd, b=back, esc=quit\n");
    
	for (done = 0; !done; )
	{
		CheckState();

		if (kbhit())	/* key pressed? */
		{	key = getch() ;

			switch (key)
			{
                case 'g':	/* Go axis 0 */
					state = GO;
					break;
                
                case 'f':	/* adjust the SLAVE forward */
					state = MOVE_FWD;
					break;

                case 'b':	/* adjust the SLAVE backward */
					state = MOVE_BACK;
					break;

				case 0x1B:	/* <ESC> */
					done = TRUE;
					break;
			}
		}
	}
	return 0;
}
