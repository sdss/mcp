/* LSINT.C

:Link synchronization based on I/O sensors in an interrupt routine.

This sample link's two axes together and commands a velocity move on the master
 axis.  Then the DSP is configured to interrupt the host CPU every sample.  The
 "delta" distance and current link "ratio" are displayed.  The axes are 
 synchronized based on keyboard input.
 
 
The interrupt routine is as follows:
   
 1) Read the sensor inputs (for simulation purposes, read the index inputs).
 2) Read the 16 bit encoder inputs and update the 32 bit software position.
 3) Determine the positions at the sensors.
 4) If "sync_flag" is TRUE, synchronize the axes. 
 
 
Here is the synchronization algorithm:

 1) Determine the slave positions at the sensors.
 2) Check if the sync_flag is enabled.
 3) Check if the sync timer has expired.
 4) Calculate the delta distance between sensor positions.
 5) If the delta has changed and is larger than the deadband, then calculate
 	a new link ratio:
			ratio = initial_ratio + (delta * P_RATIO);
 6) Convert the new link ratio to 16 bit whole, 16 bit fractional.
 7) Download a frame to set the new link ratio.
 
The link ratio is calculated based on the slave's "delta" distance between
 the master sensor and the slave sensor.  Then the delta is multiplied by the
 proportional term, P_RATIO.

The link ratio is represented as a 32 bit signed value.  The upper 16 bits
 represent the whole portion and the lower 16 bits represent the fractional
 portion.

There are several parameters to control the response of the synchronization
 algorithm that depend on the mechanical system:

	RATIO - Initial link ratio (floating point)
	MIN_RATIO - Minimum link ratio boundry (16 bit whole, 16 bit fractional)
	MAX_RATIO - Maximum link ratio boundry (16 bit whole, 16 bit fractional)
	P_RATIO - Proportional term for synchronization response,
			 (16 bit whole, 16 bit fractional)
	SYNC_DELAY - Number of samples to delay before next link ratio update.
	MAX_DIST - Distance between sensors on slave axis.
	DEADBAND - Maximum tolerance between master and slave synchronization.
 
 
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


# define R_8259_EOI						0x20
# define R_8259_IMR						0x21
# define EOI							0x20

# define MASTER			0
# define SLAVE			1
# define VELOCITY		700.0
# define ACCEL			4000.0

# define RATIO			1.0			/* used to set the initial link */
# define MIN_RATIO		0.0
# define MAX_RATIO		2.0
# define P_RATIO		8			/* proportional term for synchronization,
										16 bit whole, 16 bit fractional */
# define SYNC_DELAY		20			/* number of samples to delay next update */	

# define MAX_DIST		8192		/* Slave distance between sensors */
# define HALF_MAX_DIST	(MAX_DIST / 2)
# define DEADBAND		100			/* Maximum tolerance */

# define MAX_IO_PORTS	9
# define SENSOR_PORT	6 			/* I/O port with home/index inputs */ 
# define SENSOR_ADDR	0x30		/* Address of home input port */	
# define M_MASK			0x04		/* Master sensor mask */
# define S_MASK			0x40		/* Slave sensor mask */


int16	
	encoder[PCDSP_MAX_AXES],			/* encoder counts */
	act_velocity[PCDSP_MAX_AXES],		/* counts per sample */
	io_port[MAX_IO_PORTS],
	sync_flag = 0,
	whole_ratio,
	dir;

unsigned16 
	frac_ratio,
	sync_cnt = SYNC_DELAY;
	
int32
	act_position[PCDSP_MAX_AXES],
	s_pos_0,
	s_pos_1,
	initial_ratio,
	ratio,	
	min_ratio,
	max_ratio,
	delta;	

typedef void (interrupt * INTERRUPT_HANDLER)();
INTERRUPT_HANDLER old_handler = NULL;
static int16 imr, interrupt_number = 5;	/* IRQ 5 */


static int16 Read_DSP(unsigned addr)
{	
	outpw(dspPtr->address,addr | 0x8000);
	return(inpw(dspPtr->data));
}

static void Write_DSP(unsigned addr, int16 dm)
{	
	outpw(dspPtr->address,addr | 0x8000);
	outpw(dspPtr->data,dm);
}

static int16 set_dsp_irq(int16 axis, int16 enable)
{
    int16 addr1,addr2;
    int16 v1, v2 ;
    
    addr1 = dspPtr->global_data + axis + GD_SIZE;		/* irq_count */
    addr2 = addr1 + dspPtr->axes;						/* ack_count */
    v1 = Read_DSP(addr1);
    v2 = Read_DSP(addr2);
    
    if (enable)
    	Write_DSP(addr2, v1 + 1);	/* DSP generate interrupts every sample */
    else
		Write_DSP(addr2, v1);		/* set ack_count = irq_count */
		
    return (v1 - v2);
}
    
void link_frame(int16 slave_axis, int16 whole, unsigned16 frac)
{
	FRAME frame;
	DSP_DM addr = dspPtr->data_struct + DS(slave_axis) + DS_RATIO, buffer[2];

	buffer[0] = frac;
	buffer[1] = whole;

	dsp_move_frame(&frame, slave_axis, MF_MOVE, addr, MF_DATA_AREA,
		MF_DATA_AREA, 2, buffer);
}
    
void _far interrupt dsp_interrupt()
{
	int16 axis, new_enc;
	int32 new_delta;
	
	disable();

	io_port[SENSOR_PORT] = Read_DSP(SENSOR_ADDR);	/* read the index inputs */

	for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{ 	new_enc = Read_DSP(ENCODER_0 + axis);
		act_velocity[axis] = new_enc - encoder[axis];
		act_position[axis] += act_velocity[axis];
		encoder[axis] = new_enc;
	}	

	if (io_port[SENSOR_PORT] & M_MASK)		/* Master index pulse */
		s_pos_0 = act_position[SLAVE];
	if (io_port[SENSOR_PORT] & S_MASK)		/* Slave index pulse */
		s_pos_1 = act_position[SLAVE];
		
	if (sync_flag)
	{
		if (!sync_cnt)
		{	
			new_delta = s_pos_1 - s_pos_0;
			new_delta %= MAX_DIST;
			
			if (new_delta != delta)
			{
				delta = new_delta;
			
				if (delta > 0)
					dir = 1;
				else	
					dir = -1;

				if ((delta * dir) > DEADBAND)
				{ 	
					if ((dir * delta) < HALF_MAX_DIST)
						ratio = initial_ratio + (delta * P_RATIO);
					else
						ratio = initial_ratio - (delta * P_RATIO);
				}	
				else
					ratio = initial_ratio;

				if (ratio > max_ratio)
					ratio = max_ratio;
				if (ratio < min_ratio)
					ratio = min_ratio;
			
				whole_ratio = (int16)(ratio >> 16);	
				frac_ratio = (unsigned16)(ratio & 0xFFFF);
				link_frame(SLAVE, whole_ratio, frac_ratio);
				sync_cnt = SYNC_DELAY;		/* reset sync count down */
			}	
		}	
		else
			sync_cnt--;
	}
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

void initialize(void)
{
	int16	axis, error_code;
	double x, src, w, f;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	/* This is really only needed for code that doesn't do a dsp_reset. */
	for (axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{ 	get_position(axis, &x);
		act_position[axis] = (long)x;
		encoder[axis] = dsp_encoder(axis);
	}
	
	set_home_index_config(MASTER, INDEX_ONLY);	/* use index for simulation */
	set_home_index_config(SLAVE, INDEX_ONLY);
	
	mei_link(MASTER, SLAVE, RATIO, LINK_ACTUAL);
	v_move(MASTER, VELOCITY, ACCEL);			/* simulate master */

	/* Convert link ratio from floating point to fixed point format,
		16 bit signed whole, 16 bit unsigned fractional */
	initial_ratio = ratio = (int32) (RATIO * SFRACTIONS_PER_COUNT);
	min_ratio = (int32) (MIN_RATIO * SFRACTIONS_PER_COUNT);
	max_ratio = (int32) (MAX_RATIO * SFRACTIONS_PER_COUNT);
	printf("\nInitial Ratio: %ld\n", initial_ratio);
	
	install_dsp_handler(interrupt_number);
	init_io(2, IO_OUTPUT);			/* PC interrupt bit is located on port 2 */ 
	reset_bit(23);					/* enable interrupts */
	set_dsp_irq(0, TRUE); 			/* interrupt host CPU every sample */
}

int16	main()
{
	int16 key, done = 0;

	initialize();
	printf("\ns=turn synchronize on/off, esc=quit\n");
	
	while (!done)
	{ 	
		printf("Delta:%8ld Sync:%1d Whole:0x%4x Frac:0x%4x\r",
			 delta, sync_flag, whole_ratio, frac_ratio);

		if (kbhit())	/* key pressed? */
		{	key = getch();

			switch (key)
			{
				case 's':		/* Turn on/off synchronization */
					if (sync_flag)
						sync_flag = 0;
					else
						sync_flag = 1;	
					break;

				case 0x1B:	/* <ESC> */
					v_move(MASTER, 0.0, ACCEL);		/* Stop the master */
					done = TRUE;
					break;
			}
		}
	}
	while (!motion_done(MASTER))
		;
	endlink(SLAVE);
	set_bit(23);					/* disable interrupts */
	set_dsp_irq(0, FALSE); 			/* turn off interrupts */
	
	return 0;
}
