/* EVENTINT.C

:Generates interrupts to the CPU based on exception events.

Sample program to demonstrate simple interrupt handling.  This code generates
 a STOP_EVENT on the DSP which generates an interrupt to the CPU.

The interrupt routine and other functions used by this sample are in the
 file INTHND.C.
		
The interrupt handler updates the varaible io_interrupt and axis_interrupt.
 After the interrupt is handled, then this sample examines io_interrupt
 and axis_interrupt[] to determine the cause.

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

extern int16 dsp_irq_frame(int16 axis);
extern void install_dsp_handler(int16 into);
extern int16 axis_interrupts[PCDSP_MAX_AXES];
extern int16 io_interrupts;
extern int16 interrupt_number = 5;	/* IRQ 5 */


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

int16 main()
{   
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
	error(dsp_reset());

	init_io(2, IO_OUTPUT);		/* PC interrupt bit is located on port 2 */
	set_io(2, 0x0);
	
	install_dsp_handler(interrupt_number);		/* use IRQ 5 */
	
	interrupt_on_event(0, TRUE);

	printf("Int:%d Gen:%d\n", axis_interrupts[0], io_interrupts);
	getch();
	
	set_stop(0);
	
	while (!kbhit())
		printf("Int:%d Gen:%d\r", axis_interrupts[0], io_interrupts);
	getch();	
	
	return 0;
}

