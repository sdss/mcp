/* BLENDPT.C

:Sample to adjust an axis' profile with a position trigger on a phantom axis.

During the execution of a motion profile on the real axis, the profile and
 final position can be adjusted by using a phantom axis.  The real axis motion
 profile is blended by positionally linking a real axis to a phantom axis.
 Then motion is commanded on both axes.  The DSP handles the linking (blending)
 without host CPU intervention. 

Smooth profile blending is achieved by coordinating the motion profile of the 
 real axis with the phantom axis.  This is controlled by determining the amount
 of blending and calculating a trigger position for the execution of the 
 phantom axis' motion profile.
 
The trigger position is calculated based on the distance offset:

 1)  Small increase in distance.
 2)  Large increase in distance.
 3)  Small decrease in distance.
 4)  Medium decrease in distance.
 5)  Large decrease in distance. 
 
This method of profile blending works best for small adjustments to the real
 real axis' profile.  Since the same acceleration and velocity parameters
 are used for both the real and phantom axes, the blended velocity will not
 exceed the original real axis command velocity.  

A phantom axis can be created by downloading (n+1) axis firmware to an (n) axis
 board.  All that is required from the phantom axis is its command position.
 Be sure to disable all hardware and software limits for the phantom axis.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include <math.h>
# include "idsp.h"


# define FINAL		(30000.0)
# define VEL		10000.0
# define ACCEL		10000.0

# define OFFSET		(8000.0)

# define PHANTOM	2	/* simulated axis */
# define REAL		0

struct
{ 	double start;
	double final;
	double vel;
	double accel;
} params[PCDSP_MAX_AXES];


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

void disable_hardware_limits(int16 axis)
{
	set_positive_limit(axis, NO_EVENT);
	set_negative_limit(axis, NO_EVENT);
	set_home(axis, NO_EVENT);
	set_amp_fault(axis, NO_EVENT);
}

void disable_software_limits(int16 axis)
{
	int16 action;
	double position;
	
	get_positive_sw_limit(axis, &position, &action);
	set_positive_sw_limit(axis, position, NO_EVENT);
	get_negative_sw_limit(axis, &position, &action);
	set_negative_sw_limit(axis, position, NO_EVENT);
	get_error_limit(axis, &position, &action);
	set_error_limit(axis, position, NO_EVENT);
}

void display(int16 axis, int16 phantom, int16 n_values)
{
	int16 i;
	double cmda, cmdb, vela, velb;

	for (i = 0; i < n_values; i++)
	{
		get_command(axis, &cmda);
		get_command(phantom, &cmdb);
		get_velocity(axis, &vela);
		get_velocity(phantom, &velb);
		printf("\nC:%8.0lf V:%8.0lf C:%8.0lf Vel:%8.0lf", cmda, vela, cmdb, velb);
	}
}	

int16 adjust_profile(int16 axis, int16 phantom, double offset)
{
	int16 mdir, odir, sense;
	double dist, decel_dist, decel_time, trigger;

	if (params[axis].final > params[axis].start)	/* travel direction */
	{	mdir = 1;
		sense = POSITIVE_SENSE;
	}	
	else
	{ 	mdir = -1;	
		sense = NEGATIVE_SENSE;
	}	
		
	if (offset > 0.0)	/* find the offset direction */
		odir = 1;
	else
		odir = -1;	
		
	decel_time = params[axis].vel / params[axis].accel; 
	decel_dist = params[axis].accel * decel_time * decel_time * .5;
	dist = mdir * (params[axis].final - params[axis].start);

	if (mdir == odir)	/* Increase the move distance */
	{		
		if ((odir * offset) >= (2.0 * decel_dist))
			trigger = params[axis].final - (mdir * decel_dist);
		else
			trigger = params[axis].final - (offset / 2.0);
	}	
	if (mdir != odir)	/* Decrease the move distance */
	{
		if ((odir * offset) >= (dist - decel_dist))
			trigger = params[axis].final - (mdir * dist) + (mdir * decel_dist);
			
		if ((odir * offset) < (dist - decel_dist))
			trigger = params[axis].final - (mdir * decel_dist) + offset;
			
		if ((odir * offset) < (2.0 * decel_dist))
		{
			trigger = params[axis].final - (mdir * params[axis].vel *
				(sqrt(fabs(offset) / params[axis].accel))) -
				(mdir * decel_dist);
		}		
	}	

	printf("\nDist:%8.0lf Offset:%8.0lf\n", dist, offset);
	printf("Decel Dist: %lf Trig:%lf\n", decel_dist, trigger);
	dsp_position_trigger(phantom, axis, trigger, sense, FALSE, NEW_FRAME);
	
	start_r_move(phantom, offset, params[axis].vel, params[axis].accel);
	
	return 0;
}

int16 main()
{
	int16 error_code, axis, done, key;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());
	
	disable_hardware_limits(PHANTOM);
	disable_software_limits(PHANTOM);
	controller_run(PHANTOM);
	
    mei_link(PHANTOM, REAL, 1.0, LINK_COMMAND);

	params[REAL].start = 0.0;
    params[REAL].final = FINAL; 
    params[REAL].vel = VEL; 
    params[REAL].accel = ACCEL; 
    
    start_move(REAL, params[REAL].final, params[REAL].vel, params[REAL].accel);
	display(REAL, PHANTOM, 50);
    adjust_profile(REAL, PHANTOM, OFFSET);
	while (!motion_done(REAL) || !motion_done(PHANTOM))
		display(REAL, PHANTOM, 5);
    
	return 0;
}
