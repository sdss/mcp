/* ACFG.C

:Simple axis configuration.

This sample demonstrates how to configure the axes for open-loop stepper or
 closed-loop servo (default) operation.

The following switches can be set from the command line:

 n - where "n" is the number of axes to configure
 smt - surface mount constructed board
 boot - save changes to boot memory
 olstep - open loop stepper
 clserv - closed loop servo
 fast - highest step pulse output range
 medium - middle step pulse output range
 slow - lowest step pulse output range 

Note:  The configuration for open/closed loop operation must occur in axis
 pairs (0 and 1, 2 and 3, etc.).

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
	Revision 1.1  1999/08/31 16:42:48  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "pcdsp.h"

struct
{	int16 axes;			/* number of axes to configure */
	int16 surface_mount;	/* surface mount or through hole controller */
	int16 boot;			/* update DSP's boot memory or dynamic memory */
} board;

struct
{	int16 step_motor;		/* servo or stepper */
	int16 closed_loop;	/* closed loop or open loop (in pairs of axes) */
	int16 step_speed;		/* fast, medium, slow, or disabled */
	int16 unipolar;		/* unipolar or bi-polar voltage output */
	int16 feedback;		/* encoder, analog, parallel, ... */
	int16 i_mode;			/* integration always or only when standing */
	int16 * coeffs;		/* PID filter parameters */
} axis;

int16
	clserv_coeffs[] = {1024, 32, 4096, 0, 0, 32767, 0, 32767, -5, 0},
	olstep_coeffs[] = {200, 20, 0, 20, 2760, 32767, 0, 32767, -3, 0},
	olstep_smt_coeffs[] = {320, 32, 0, 32, 3750, 32767, 0, 32767, -1, 0};


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

void config_axes(int16 n_axes, int16 * axes)
{
	int16 i;
	
	for (i = 0; i < n_axes; i++)
	{
		dsp_set_stepper(axes[i], axis.step_motor);
		dsp_set_closed_loop(axes[i], axis.closed_loop);
		
		if (axis.step_motor && (!axis.closed_loop))		/* open-loop stepper */
		{	
			axis.unipolar = TRUE;	
			if (board.surface_mount)
			{ 	axis.coeffs = olstep_smt_coeffs;
				axis.coeffs[DF_SHIFT] = ((axis.step_speed * 2) - 7);
			}
			else
			{ 	axis.coeffs = olstep_coeffs;
				axis.coeffs[DF_SHIFT] = ((axis.step_speed * 2) - 9);
			}
		}	
		else			/* closed-loop servo */
		{
			axis.step_speed = DISABLE_STEP_OUTPUT;
			axis.unipolar = FALSE;
			axis.coeffs = clserv_coeffs;
		}			
		
		dsp_set_step_speed(axes[i], axis.step_speed);
		set_unipolar(axes[i], axis.unipolar);
		set_feedback(axes[i], FB_ENCODER);
		set_integration(axes[i], axis.i_mode);
		set_filter(axes[i], axis.coeffs);
	}
}

void config_boot_axes(int16 n_axes, int16 * axes)
{
	int16 i;
	
	for (i = 0; i < n_axes; i++)
	{
		dsp_set_boot_stepper(axes[i], axis.step_motor);
		dsp_set_boot_closed_loop(axes[i], axis.closed_loop);
		dsp_set_boot_step_speed(axes[i], axis.step_speed);
		set_boot_unipolar(axes[i], axis.unipolar);
		set_boot_feedback(axes[i], FB_ENCODER);
		set_boot_integration(axes[i], axis.i_mode);
		set_boot_filter(axes[i], axis.coeffs);
		mei_checksum();
	}
}

void initialize(void)
{	
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	board.axes = dsp_axes();
	
	/* Default axis configuration */
	board.surface_mount = FALSE;
	board.boot = FALSE;
	axis.step_motor = FALSE;
	axis.closed_loop = TRUE;
	axis.step_speed = SLOW_STEP_OUTPUT;
	axis.unipolar = FALSE;
	axis.feedback = FB_ENCODER;
	axis.i_mode = IM_STANDING;	
	axis.coeffs = clserv_coeffs;
}

void arguments(int16 argc, char * argv[])
{
	int16 i;
	char * s;
	
    for (i = 1; i < argc; i++)
   	{	s = argv[i]; 
     	if (atoi(s))
     	{	board.axes = atoi(s);
     		continue;
		}		
		if (!strcmpi(s, "smt"))
		{	board.surface_mount = TRUE;
			continue;
		}		
		if (!strcmpi(s, "boot"))
		{	board.boot = TRUE;
			continue;
		}
		if (!strcmpi(s, "olstep"))
		{	axis.step_motor = TRUE;		
			axis.closed_loop = FALSE;
			continue;
		}
		if (!strcmpi(s, "clserv"))
		{	axis.step_motor = FALSE;		
			axis.closed_loop = TRUE;
			continue;
		}	
		if (!strcmpi(s, "fast"))
		{	axis.step_speed = FAST_STEP_OUTPUT;
			continue;
		}
		if (!strcmpi(s, "medium"))
		{	axis.step_speed = MEDIUM_STEP_OUTPUT;
			continue;
		}
		if (!strcmpi(s, "slow"))
		{	axis.step_speed = SLOW_STEP_OUTPUT;
			continue;
		}
		if (!strcmpi(s, "/?"))
		{	printf("\nCommand line switches: [smt] [boot] ");
			printf("[olstep (fast|medium|slow)] [clserv]\n");
			exit(0);
		}
	}	
}

int16 main(int16 argc, char * argv[])
{
	int16 axes[] = {0, 1, 2, 3, 4, 5, 6, 7};
	
    initialize();
	arguments(argc, argv);
	
	config_axes(board.axes, axes);	
	if (board.boot)
		config_boot_axes(board.axes, axes);

	printf("\n%d axes are configured.", board.axes); 
	
    return 0;
}
