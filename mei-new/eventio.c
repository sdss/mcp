/* EVENTIO.C

:Toggles an output bit when a Stop, E-Stop, or Abort Event occurs.

This program demonstrates how to configure the DSP to toggle an I/O bit when
 a Stop, E-Stop, or Abort Event occurs.  This is useful when extenal devices
 such as lasers or welding torches must be disabled during exception events.

The Stop, E-Stop, and Abort Events are executed by the DSP based on an axis'
 limit switches, home switch, software limits, error limits, etc.  Internally,
 the DSP executes some special frames to complete the exception event.

When a Stop Event or E-Stop Event is generated, the DSP executes a Stop or
 E-Stop frame to decelerate the axis.  Then the DSP executes the Stopped frame
 to set the command velocity to zero.  An Abort Event puts the axis in idle
 mode (no PID update) and executes the Stopped frame.  See the "C Programming"
 manual for more information about Exception Events.

The frame offset addresses of the Stop, E-Stop, and Stopped frames are defined
 in IDSP.H:
#	define	EF_POS_STOP		0
#	define	EF_NEG_STOP		1
#	define	EF_POS_E_STOP	2
#	define	EF_NEG_E_STOP	3
#	define	EF_STOPPED		4

By modifying the Stop or E-Stop exception frames, a user I/O bit (or bits) can
 be toggled by the DSP during the decelration portion of a Stop or E-Stop.
 
By modifying the Stopped exception frame, a user I/O bit (or bits) can be
 toggled by the DSP after a Stop, E-Stop, or Abort Event executes.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.

Written for Version 2.5
*/

/* Revision Control System Information
$Source$
$Revision$
$Date$
$Log$
Revision 1.1  1999/08/31 16:43:01  briegel
source for MEI library

*/

# include <stdio.h>
# include <stdlib.h>
# include <conio.h>

# include "pcdsp.h"
# include "idsp.h"

# define AXIS			0
# define PORT			0

# define BIT_NUMBER		0		/* Bits are numbered 0 to 47*/

# define VELOCITY	4000.0
# define ACCEL		40000.0

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

int16 display(int16 axis, int16 io_bit)
{
	int16 a_state, bit;
	double cmd, act;

	printf("\n");
	do
	{ 	get_position(axis, &act);
		get_command(axis, &cmd);
		a_state = axis_state(axis);
		bit = bit_on(io_bit);
		printf("Cmd %6.0lf Act %6.0lf State %d Bit %d\r", cmd, act, a_state, bit);
		
		if (kbhit())
		{ 	getch();
			return 0;
		}
	}	
	while (!motion_done(axis));
	
	return 0;
}

void recover(int16 axis, int16 io_bit)
{
	display(axis, io_bit);
	error(controller_run(axis));
	reset_bit(io_bit);
}

int16 config_ef_io(PFRAME f, int16 port, int16 ormask, int16 andmask)
{
	int16 address,
		update = f->f.trig_update & 0xFF;	/* get trigger/update field */
	int16 * i = (int16*) &(f->f.position);	/* pointer to position field */

	if (!(update & FUPD_POSITION))
	{
		dsp_port_address(port, &address);	/* read the port address */
		f->f.output = OUTPUT_POSITION ;		/* set output to use position field */
		i[0] = address;
		i[1] = ormask;
		i[2] = andmask;
		f->f.trig_update |= FUPD_OUTPUT;	/* update I/O from position field */
		pcdsp_write_ef(dspPtr, f);			/* download the exception frame */
		return DSP_OK ;
	}
	return DSP_RESOURCE_IN_USE;
}

int16 set_ef_io(int16 axis, int16 ef, int16 port, int16 ormask, int16 andmask)
{
	FRAME f;
	
	if (pcdsp_read_ef(dspPtr, &f, axis, ef))
		return dsp_error;
		
	if (config_ef_io(&f, port, ormask, andmask))
		return dsp_error;
	else
		return pcdsp_write_ef(dspPtr, &f);
} 

int16 main()
{
	int16 error_code;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	init_io(PORT, IO_OUTPUT);
	set_io(PORT, 0x00);		/* set all 8 bits low */
	
	/* Configure the Stop Event frames to set an I/O bit high */
	error(set_ef_io(AXIS, EF_POS_STOP, PORT, 0x1, 0xFF));
	error(set_ef_io(AXIS, EF_NEG_STOP, PORT, 0x1, 0xFF));
	v_move(AXIS, VELOCITY, ACCEL);

	printf("\n\nPress a key to generate a Stop Event (with a bit toggle)");
	display(AXIS, BIT_NUMBER);
	set_stop(AXIS);
	recover(AXIS, BIT_NUMBER);

	/* Configure the E-Stop Event frames to set an I/O bit high */
	set_ef_io(AXIS, EF_POS_E_STOP, PORT, 0x1, 0xFF);
	set_ef_io(AXIS, EF_NEG_E_STOP, PORT, 0x1, 0xFF);
	v_move(AXIS, VELOCITY, ACCEL);

	printf("\n\nPress a key to generate an E-Stop Event (with a bit toggle)");
	display(AXIS, BIT_NUMBER);
	set_e_stop(AXIS);
	recover(AXIS, BIT_NUMBER);
	
	/* Configure the Stopped frame to set an I/O bit high.  The Stopped frame
		executes after a Stop, E-Stop, or Abort Event */
	set_ef_io(AXIS, EF_STOPPED, PORT, 0x1, 0xFF);
	v_move(AXIS, VELOCITY, ACCEL);

	printf("\n\nPress a key to generate an Abort Event (with a bit toggle)");
	display(AXIS, BIT_NUMBER);
	controller_idle(AXIS);	/* generate an Abort Event */
	recover(AXIS, BIT_NUMBER);

	/* Return frames to default configuration */
	error(dsp_reset());		/* hardware reset */
	
	return 0;
}
