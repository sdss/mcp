/*  INTSIMPL.C

:Sample code for Single-Board Interrupt Support under DOS.

This code demonstrates how to handle interrupts generated from
  a single DSP-Series controller.

When programming with interrupt routines, make sure to define 
  CRITICAL and ENDCRITICAL.  These are located in idsp.h.  CRITICAL
  is used to disable interrupts and ENDCRITICAL re-enables interrupts.  
  This is very important!  Interrupts during communication can cause
  strange problems.

If you are using a Borland C compiler (3.1 or greater), then CRITICAL
  and ENDCRITICAL are defined for you.

When compiling code with an interrupt routine, be sure to turn stack
  checking off.
		
This program uses a general method for installing and removing interrupt
  handlers.  The handler itself is written specifically for this program
  and can easily be modified.

If you modify dsp_interrupt(...), watch out for floating point
  operations, most compilers do not support them.  Also, MEI's standard
  function library is not supported inside interrupt routines.

Written for Borland and Microsoft compilers.	

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
	Revision 1.1  1999/08/31 16:43:14  briegel
	source for MEI library

*/

#	include <stdio.h>
#	include <stdlib.h>
#	include <dos.h>
#	include <conio.h>
#	include "idsp.h"

#	define  R_8259_EOI	0x20
#	define  R_8259_IMR	0x21
#	define  EOI			0x20

typedef void (interrupt * INTERRUPT_HANDLER)();
void install_dsp_handler(int16 intno);
INTERRUPT_HANDLER old_handler = NULL;
static int16 imr, interrupt_number;

int16 INTERRUPT_FLAG = FALSE;

void error(int16 error_code)
{   
	char buffer[MAX_ERROR_LEN];
	
    if(error_code)
	{  	error_msg(error_code, buffer); 
		printf("ERROR: %s\n", buffer);
        exit(1);
    }
}

void _far interrupt dsp_interrupt()
{
	int16 i;
	
	_disable();
   INTERRUPT_FLAG = TRUE;

   outp(R_8259_EOI, EOI);
	_enable() ;
}

void remove_handler(void)
{
	_disable() ;
	_dos_setvect(interrupt_number + 8, old_handler) ;
	outp(R_8259_IMR, imr) ;
	outp(R_8259_EOI, EOI) ;
	_enable() ;
}

void install_dsp_handler(int16 intno)
{
	int16 new_imr;

	interrupt_number = intno ;
	old_handler = _dos_getvect(interrupt_number + 8) ;
	imr = inp(R_8259_IMR) ;

	atexit(remove_handler) ;

	new_imr = imr & ~(1 << interrupt_number) ;
	_disable() ;
	_dos_setvect(interrupt_number + 8, dsp_interrupt) ;
	outp(R_8259_IMR, new_imr) ;
	outp(R_8259_EOI, EOI) ;
	_enable() ;
}

int16 main(void)
{
   int16 error_code;
	
   error_code = do_dsp();	/* initialize communication with the controller */
   error(error_code);		/* any problems initializing? */

   // Initialize port 2 for output.  All bits including bit 23 will be low.
   init_io(2, IO_OUTPUT);

   // Enable the DSP card to send interrupts to the PC.
   dsp_interrupt_enable(TRUE);

   // Install the interrupt handler.
   install_dsp_handler(5);

   // The INTERRUPT_FLAG should be 0 before the interrupt is sent.
   while(!kbhit())
      if(INTERRUPT_FLAG)
         printf("Interrupt received    \r");
      else
         printf("Interrupt not received\r");
   getch();

   // Send an interrupt by setting bit 23 HIGH.
   set_bit(23);

   // The INTERRUPT_FLAG should be 1 after the interrupt is sent.
   while(!kbhit())
      if(INTERRUPT_FLAG)
         printf("Interrupt received    \r");
      else
         printf("Interrupt not received\r");
   getch();

   return 0;
}
