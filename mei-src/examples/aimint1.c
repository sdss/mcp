/* AIMINT1.C

:Axis motion and DSP I/O monitoring with interrupt handling.

Sample program to demonstrate simple interrupt handling.  Motion is commanded
 with start_move(...) and the DSP is configured to monitor the User I/O.
 When the dsp_irq_frame(...) is executed by the DSP or any of the specified
 I/O bits change the DSP generates an interrupt to the PC.

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

int16 display(int16 axis, int16 port, int16 expected_value)
{
	int16 done = 0, key, value, status;
	double cmd;
	
	while (!done)
	{ 	get_io(port, &value);
		get_command(axis, &cmd);
		printf("Axis:%d Cmd:%8.0lf Port:%d Val:%d\r", axis, cmd, port, value);
		
		if (io_interrupts)					/* updated by interrupt routine */
		{	set_io(port, expected_value);	/* set bit(s) to expected state */
			clear_io_mon();					/* reset the DSP's I/O monitoring */
			io_interrupts = 0;				/* reset interrupt counter */ 
			io_mon(port, &status);
			printf("\nInterrupt--Port: %d Status: 0x%x\n\n", port, status);
		}	

		if (axis_interrupts[axis])			/* updated by interrupt routine */
		{ 	printf("\nInterrupt--Cmd: %8.0lf\n\n", cmd);	
			axis_interrupts[axis] = 0;		/* reset interrupt counter */
		}
			
		if (kbhit())
		{	if (getch() == 0x1B)
				done = TRUE;
			else
				set_bit(0);	
		}
	}	
	return 0;
}

int16 main()
{   
	int16 error_code, port, mask, expected_value, axis = 0;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
	error(dsp_reset());

	init_io(0, IO_OUTPUT);
	init_io(2, IO_OUTPUT);			/* PC interrupt bit is located on port 2 */ 
	
	install_dsp_handler(interrupt_number);	/* use IRQ 5 */
	reset_bit(23);					/* enable interrupts */

	set_io(0, 0x0);
	mask = 0x1;						/* monitor bit 0, on port 0 */		
	port = 0;
	expected_value = 0x0;
	set_io_mon_mask(port, mask, expected_value);
	
	printf("\nPress any key to set bit high, <esc> to quit");
	printf("\nMonitoring = port:%d mask:0x%x\n\n", port, mask);
	start_move(axis, 1000.0, 500.0, 1000.0);
	dsp_irq_frame(axis);
	
	display(axis, port, expected_value);
	set_bit(23);					/* disable interrupts */
	
	return 0;
}
