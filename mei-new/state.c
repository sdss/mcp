/* STATE.C

:Displays axis state, status, and source.

This program is written in the form of a state machine.
   
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
	Revision 1.1  1999/08/31 16:43:58  briegel
	source for MEI library

*/

# include <stdio.h>
# include <stdlib.h>
# include <conio.h>	  
# include <string.h>
# include "pcdsp.h"

# define  READY             0
# define  MOVE              1
# define  GOTO_0            2
# define  AXIS_COMMAND      3

int16   state;

char * states[] = {"running",
                   "running",
                   "new frame",
                   "","","","","",
                   "stop event",
                   "",
                   "e stop event",
                   "","","",
                   "abort event"};

char * sources[] = {"none",
                    "home switch",
                    "pos limit switch",
                    "neg limit switch",
                    "amp fault",
                    "not available",
                    "not available",
                    "neg position limit",
                    "pos position limit",
                    "error limit",
                    "pc command",
                    "out of frames",
                    "temposonics probe fault",
                    "axis command"};

char * stati[] = {" ",
                  "in sequence ",
                  "in position ",
                  "in motion ",
                  "neg direction ",
                  "frames left "};
                

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

void display_axis_state(int16 axis)
{
   int16      state, source, status, i;
   char     state_string[25], source_string[25], status_string[65];

   state = axis_state(axis);
   source = axis_source(axis);
   status = axis_status(axis);

   strcpy(state_string, states[state]);
   strcpy(source_string, sources[source]);

   strcpy(status_string, "");
   for(i = 0; i < 5; i++)
      if(status & (1 << (4 + i)))
         strcat(status_string, stati[i + 1]);
   // Fill the rest of the status buffer with " ".
   for(i = 0; i < (66 - strlen(status_string)); i++)
      strcat(status_string, stati[0]);

   printf("%s\t%s\t%s\r", state_string, source_string, status_string);
}

void CheckState(int16 axis)
{
   switch(state)
   {
      case READY:
         break;

      case MOVE:
         start_move(axis, 4000.0, 1000.0, 10000.0);
         state = READY;
         break;

      case  GOTO_0:
         start_move(axis, 0.0, 1000.0, 10000.0);
         state = READY;
         break;

      case AXIS_COMMAND:
         dsp_axis_command(1, axis, STOP_EVENT);
         state = READY;
         break;
   }
}

int16 main()
{
   int16   done = FALSE, axis = 0;
   int16 error_code;
	
   error_code = do_dsp();	/* initialize communication with the controller */
   error(error_code);		/* any problems initializing? */

   state = READY;

   printf("<ESC> = exit, m = move, 0 = goto 0, c = send stop event from another axis\n");
   while(!done)
   {
      CheckState(axis);
      display_axis_state(axis);
	
      if (kbhit())
      {  
         switch(getch())
         {
            case 0x1B:
               done = TRUE;
               break;

            case 'm':
               state = MOVE;
               break;

            case '0':
               state = GOTO_0;
               break;

            case 'c':
               state = AXIS_COMMAND;
               break;

            case READY:
               break;
         }
      }
   }
   return 0;
}
