/*	PVU5.C

:Looping frame sequence to update positions and velocities every 'n' samples.

This sample demonstrates how to update an axis' command velocity and command 
 position very quickly.  The motion profile is generated one point ahead of the
 current motion.  The points are equally spaced in time.
   
First, a frame is downloaded to each axis in the DSP's frame buffer.  Then each
 frame's next pointer is adjusted so that it points to itself.  Each frame is
 then executed by the DSP every 'n' sample(s) based on the trigger/update
 register.  The last axis' frame is configured to generate an interrupt to the
 host.
 
The command velocity (16 bit whole portion) and command position (32 bit whole
 portion) is updated by writing directly to the frame with dsp_write_dm(...).
 
The command velocity is based on three 16 bit registers (whole, upper fractional,
 and lower fractional).  The units of the whole part are in terms of encoder
 counts per sample.  The default sample rate is 1250 samples per second. 
 
The command position is based on four 16 bit registers (upper whole, lower
 whole, upper fractional, and lower fractional).  The units of the whole part
 are in terms of encoder counts.

At the end of the point updates, E-Stop's are generated and all of the axes 
 decelerate to a stop.  Then the looping frames are cleared from the DSP's
 axis lists. 
 
When programming with interrupt routines, make sure to define CRITICAL
 and ENDCRITICAL.  These are located in idsp.h.  CRITICAL is used to disable 
 interrupts and ENDCRITICAL re-enables interrupts.  This is very important!
 Interrupts during communication can cause strange problems.

If you are using a Borland C compiler (3.1 or greater), then CRITICAL and
 ENDCRITICAL are defined for you.

When compiling code with an interrupt routine, to turn stack checking off.
		
If you modify dsp_interrupt(...), watch out for floating point operations,
 many compilers do not support them.  Also, MEI's standard function library
 is not supported inside interrupt routines.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
 
Written for Borland and Microsoft compilers.	
  
Written for Version 2.5  
*/



# include <stdio.h>
# include <conio.h>
# include <dos.h>
# include <stdlib.h>
# include <math.h>
# include "pcdsp.h"
# include "idsp.h"


# define POINTS 		1000

# define SAMPLE_RATE 	1250	/* samples per second */
# define NEXT_UPDATE  	10		/* number of samples for next update */

/* Frame register offsets - registers are 16 bits */
# define CONTROL		0x1
# define TIME_0			0x2
# define TIME_1			0x3
# define JERK_0			0x4
# define JERK_1			0x5
# define JERK_2			0x6
# define ACCEL_0		0x7
# define ACCEL_1		0x8
# define ACCEL_2		0x9
# define VEL_0			0xA
# define VEL_1			0xB
# define VEL_2			0xC
# define POSITION_0		0xD
# define POSITION_1		0xE
# define POSITION_2		0xF
# define POSITION_3		0x10
# define TRIG_UPDATE	0x11
# define ACTION			0x12
# define OUTPUT			0x13

# define R_8259_EOI		0x20
# define R_8259_IMR		0x21
# define EOI			0x20
# define EF_NONE		(-1)

# define IRQ_LINE		5 	

long whole_pos[PCDSP_MAX_AXES] = {0, 0, 0, 0, 0, 0, 0, 0};
int16	whole_vel[PCDSP_MAX_AXES] = {1, -1, 1, -1, 1, 1, 1, 1};	/* cts per sample */
	
int16 frame_addr[PCDSP_MAX_AXES];
int16 whole_vel_addr[PCDSP_MAX_AXES];
int16 whole_pos_addr[PCDSP_MAX_AXES];

typedef void (interrupt * INTERRUPT_HANDLER)();

void install_dsp_handler(int16 intno);
typedef void (interrupt * INTERRUPT_HANDLER)();
INTERRUPT_HANDLER old_handler = NULL;
static int16 imr, interrupt_number;

int16 axis_interrupts[PCDSP_MAX_AXES];
int16 update_flag = FALSE;


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

static int16 Read_DSP(unsigned addr)
{	outpw(dspPtr->address,addr | 0x8000);
	return(inpw(dspPtr->data));
}

static void Write_DSP(unsigned addr, int16 dm)
{	
	outpw(dspPtr->address,addr | 0x8000);
	outpw(dspPtr->data,dm);
}

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
	int16 i;
	
	_disable();

	/* Did an axis generate the interrupt? */
	for (i = 0; i < PCDSP_MAX_AXES; i++)
	{
		if (inc_acks(i))
		{
			axis_interrupts[i]++;
			update_flag = TRUE;
		}
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

int16 end_of_motion(int16 n_axes, int16 * axes)
{
    int16 i;
    
    for(i = 0; i < n_axes; i++)
    { 	if(in_motion(axes[i]))
            return 0;
    }
    return 1;
}

void display(int16 n_axes, int16 * axes)
{
    int16 i;
    double cmd;

    for(i = 0; i < n_axes; i++)
    {
      	get_command(axes[i], &cmd);
        printf("%d:%8.0lf ", axes[i], cmd);
    }
    printf("\r");
}

void init_loop_frames(int16 n_axes, int16 * axes, DSP_DM update)
{
	int16 i, axis, addr, value;
	
	for (i = 0; i < n_axes; i++)
	{	 
		axis = axes[i];
		dsp_control(axis, FCTL_RELEASE, FALSE);
		dsp_control(axis, FCTL_HOLD, TRUE);
		set_gate(axis);
	
		dsp_marker(axis, &addr);		/* download a frame to the DSP */
		dsp_write_dm(addr, addr);		/* have the frame point to itself */
		
		dsp_write_dm(addr + TIME_0, NEXT_UPDATE);	/* time trigger */
		dsp_write_dm(addr + TRIG_UPDATE, update); 

		frame_addr[i] = addr;
		whole_vel_addr[i] = addr + VEL_2; 
		whole_pos_addr[i] = addr + POSITION_2;

		dsp_control(axis, FCTL_RELEASE, TRUE);
		dsp_control(axis, FCTL_HOLD, FALSE);
	}	

	/* Configure the last axis to generate interrupts to the host. */
	addr = frame_addr[n_axes - 1] + CONTROL;
	value = dsp_read_dm(addr);
	dsp_write_dm(addr, value | FCTL_INTERRUPT); 
}

void update_frames(int16 n_axes, int16 * axes, int16 * vel, long * pos)
{
	int16 i;
	int16 upper_pos, lower_pos;
	
	for (i = 0; i < n_axes; i++)
	{
		lower_pos = 0xFFFF & pos[i];
		upper_pos = pos[i] >> 16; 
		dsp_write_dm(whole_vel_addr[i], vel[i]);
		dsp_write_dm(whole_pos_addr[i], lower_pos);
		dsp_write_dm(whole_pos_addr[i] + 1, upper_pos);
	}
	reset_gates(n_axes, axes);			/* ready, set, go! */
}

void download_point(int16 n_axes, int16 * axes)
{
    int16 i;

	/* Wait for the previous point to execute before calculating the next
		point.  Since the last frame generates an interrupt, we only need to
		check the interrupt flag. 
	*/
		
	while (!update_flag)
		;	
	update_flag = FALSE;	
	set_gates(n_axes, axes);		/* ready, set, wait! */
		
	for(i = 0; i < n_axes; i++)
	{	
		/* Calculate the next point.  In this sample we simply calculate the 
			next point based on constant velocity.
		*/
		whole_pos[i] += (whole_vel[i] * NEXT_UPDATE);
	}
	update_frames(n_axes, axes, whole_vel, whole_pos);
}

void initialize(int16 n_axes, int16 * axes)
{
	int16 error_code, axis;
	DSP_DM update;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
    error(dsp_reset());
    
    set_sample_rate(SAMPLE_RATE);
    
    /* Be sure to connect bit #23 to ground (enables host interrupts) */ 
    init_io(2, IO_INPUT);
    
    init_loop_frames(n_axes, axes, FUPD_POSITION | FUPD_VELOCITY | FTRG_TIME);
    install_dsp_handler(IRQ_LINE);
}

void release_frame(int16 n_axes, int16 * axes)
{
	int16 i, axis;

	for (i = 0; i < n_axes; i++)
	{
		axis = axes[i];
		set_gate(axis);
		dsp_write_dm(frame_addr[i], 0);
		dsp_write_dm(frame_addr[i] + 1, FCTL_RELEASE | FCTL_HOLD);
		dsp_write_dm(frame_addr[i] + ACTION, 0);
		dsp_write_dm(frame_addr[i] + TRIG_UPDATE, 0);
		reset_gate(axis);		
	}
	
	for (i = 0; i < n_axes; i++)
	{
		axis = axes[i];
		while(axis_status(axis) & FRAMES_LEFT)
			;
		while(clear_status(axis))
			;
	}
}

int16	main()
{
	int16 i, axis, n_axes, axes[] = {0, 1, 2, 3, 4, 5};
	long point;
	DSP_DM update;
		
	n_axes = (sizeof(axes)/sizeof(int16));	
	initialize(n_axes, axes);
	reset_gates(n_axes, axes);		/* ready, set, go! */

	for (point = 1; point < POINTS; point++)
	{	download_point(n_axes, axes);
		display(n_axes, axes);
	}
	
	for (i = 0; i < n_axes; i++)
		set_e_stop(axes[i]);
	
	while (!end_of_motion(n_axes, axes))
	{	printf("\n");
		display(n_axes, axes);
	}
	release_frame(n_axes, axes);
		
    return 0;
}       
