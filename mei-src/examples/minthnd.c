/* MINTHND.C

:Sample code for Multi-Board Interrupt Support under DOS.

This code demonstrates how to handle interrupts generated from multiple
 DSP-Series controllers.  Sharing interrupts requires an external pull-up
 resistor on the IRQ line.  The latest revisions of the surface mount
 controllers have this resistor.  The PC/DSP does not have this resistor. 

When programming with interrupt routines, make sure to define CRITICAL
 and ENDCRITICAL.  These are located in idsp.h.  CRITICAL is used to disable
 interrupts and ENDCRITICAL re-enables interrupts.  This is very important!
 Interrupts during communication can cause strange problems.

If you are using a Borland C compiler (3.1 or greater), then CRITICAL and
 ENDCRITICAL are defined for you.

When compiling code with an interrupt routine, turn stack checking off.
		
If you modify dsp_interrupt(...), watch out for floating point operations,
 most compilers do not support them.  Also, MEI's standard function library
 is not supported inside interrupt routines.

Written for Borland and Microsoft compilers.	

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/



# include <stdio.h>
# include <stdlib.h>
# include <dos.h>
# include <conio.h>
# include "idsp.h"
# include "mboard.h"

# define  R_8259_EOI						0x20
# define  R_8259_IMR						0x21
# define  EOI								0x20
# define  EF_NONE							(-1)
# define  DATA_STRUCT(dsp, axis, offset)	((dsp)->data_struct + (DS_SIZE * (axis)) + offset)

typedef void (interrupt * INTERRUPT_HANDLER)();

int16 set_board(int16 boardno);
static int16 Read_DSP(unsigned addr);
static void Write_DSP(unsigned addr, int16 dm);
void install_dsp_handler(int16 intno);
typedef void (interrupt * INTERRUPT_HANDLER)();
INTERRUPT_HANDLER old_handler = NULL;
static int16 imr, interrupt_number;

#	define	BOARDS		3

int16 board = 0;
extern int16 Boards;

typedef int16 I_LIST [PCDSP_MAX_AXES];

I_LIST
	axis_interrupt_list[BOARDS];
	
int16 * axis_interrupts;
int16 io_interrupts[BOARDS];

static int16 inc_acks(int16 axis)
{
    int16 addr1,addr2;
    int16 v1, v2 ;
    
    addr1 = dspPtr->global_data + axis + GD_SIZE;		/* irq_count */
    addr2 = addr1 + dspPtr->axes;						/* ack_count */
    v1 = Read_DSP(addr1);
    v2 = Read_DSP(addr2);
    
    if (v1 != v2)				/* Is the DSP generating interrupts? */
		Write_DSP(addr2,v1);	/* set ack_count = irq_count */
    return (v1 - v2);
}
    
void _far interrupt dsp_interrupt()
{
	int16 i, l, m;
	
	_disable();

	m = board;
	
	for (l = 0; l < Boards; l++)
	{
		set_board(l);

		/* Is the interrupt caused by the DSP I/O monitoring? */
		if (Read_DSP((Read_DSP(DM_IO_BLOCK)) + IO_CHANGE))
		{
			io_interrupts[l]++;
			/* Clear the I/O monitoring flag. */
			Write_DSP((Read_DSP(DM_IO_BLOCK) + IO_CHANGE), 0);
		}	
		
		for (i = 0; i < PCDSP_MAX_AXES; i++)
		{
			if (inc_acks(i))
			{
				axis_interrupts[i]++;
			}
		}
	}

	set_board(m);
    outp(R_8259_EOI, EOI);
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

int16 dsp_irq_frame(int16 axis)
{
	FRAME frame ;
	frame_clear(&frame)  ;
	if (frame_allocate(&frame, dspPtr, axis))
		return dsp_error ;
	frame.f.control |= FCTL_INTERRUPT ;
	return frame_download(&frame) ;
}

int16 set_board(int16 boardno)
{
	board = boardno;
	axis_interrupts = axis_interrupt_list[board];
	return m_board(boardno);
}

static int16 Read_DSP(unsigned addr)
{	outpw(dspPtr->address,addr | 0x8000);
	return(inpw(dspPtr->data));
}

static void Write_DSP(unsigned addr, int16 dm)
{	
	outpw(dspPtr->address,addr | 0x8000);
	outpw(dspPtr->data,dm);
}

