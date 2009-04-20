/* JOGLAT1.C

:Jog, latch positions, generate stop events, and clear status on three axes.

This code initializes three analog input channels for unipolar (0 to 5 volt)
 operation.  Then the DSP is configured for velocity based jogging.
 When positions are latched (by toggling User I/O bit #22), a sequence of 
 frames on a phantom axis are executed. These frames generate STOP_EVENTs 
 on axes (0-2). The status of each axis is then cleared to allow any  
 further motion to be programmed.

The jogging velocity is calculated as follows:

	Each sample, the DSP runs through the following calculations to 
	determine the velocity that the motor will run at.

		if A/D value > (center + deadband)
			then J = A/D value - center - deadband
		if A/D value < (center - deadband)
			then J = A/D value - center + deadband
		if (center + deadband) > A/D value > (center - deadband)
			then J = 0

		motor velocity (counts/sec) = (linear term * 1/65536 * J
									   + cubic term * 3.632E-12 * (J * J * J))
									   * sample_rate

Be sure user I/O bit #22 is normally driven high, and is pulled low to
 activate the latch.  The falling edge of bit #22 triggers the DSP's
 interrupt.  The DSP's interrupt routine handles the latching of the actual
 positions of all axes within 4 microseconds.

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include "pcdsp.h"


# define    X                 0
# define    Y                 1
# define    Z                 2

# define    XJOG              0
# define    YJOG              1
# define    ZJOG              2

# define    XANALOGCHANNEL    0
# define    YANALOGCHANNEL    1
# define    ZANALOGCHANNEL    2

# define    XCENTER           2048
# define    YCENTER           2048
# define    ZCENTER           2048

# define    XDEADBAND         5
# define    YDEADBAND         20
# define    ZDEADBAND         50

# define    XLINEAR           10
# define    YLINEAR           100
# define    ZLINEAR           1000

# define    XCUBIC            10
# define    YCUBIC            50
# define    ZCUBIC            100


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

void begin_jogging(void)
{
	/* Initialize X, Y, and Z channels to use specific analog channels and 
	configure these channels to be unipolar and single ended.
	NOTE: the default analog channel for an axis is the same as the axis
	number and is configured for unipolar and single ended.
	*/
   
	set_analog_channel(X, XANALOGCHANNEL, FALSE, FALSE);
	set_analog_channel(Y, YANALOGCHANNEL, FALSE, FALSE);
	set_analog_channel(Z, ZANALOGCHANNEL, FALSE, FALSE);

	jog_axis(X, XJOG, XCENTER, XDEADBAND, XLINEAR, XCUBIC, TRUE);
	jog_axis(Y, YJOG, YCENTER, YDEADBAND, YLINEAR, YCUBIC, TRUE);
	jog_axis(Z, ZJOG, ZCENTER, ZDEADBAND, ZLINEAR, ZCUBIC, TRUE);
}

void stop_with_latch(void)
{
	dsp_io_trigger(0,22,FALSE);  /* Execute next frame when bit 22 goes low */
	dsp_io_trigger(1,22,FALSE);
	dsp_io_trigger(2,22,FALSE);

	dsp_axis_command(0,0,STOP_EVENT);  
	dsp_axis_command(1,1,STOP_EVENT);
	dsp_axis_command(2,2,STOP_EVENT);
}

void recover_from_latch(void)
{
	/* Disable jogging on three axes */
	
	jog_axis(X, XJOG, XCENTER, XDEADBAND, XLINEAR, XCUBIC, FALSE);
	jog_axis(Y, YJOG, YCENTER, YDEADBAND, YLINEAR, YCUBIC, FALSE);
	jog_axis(Z, ZJOG, ZCENTER, ZDEADBAND, ZLINEAR, ZCUBIC, FALSE);
	
	while(!motion_done(0) || !motion_done(1) || !motion_done(2))
		;
	error(clear_status(X));
	error(clear_status(Y));
	error(clear_status(Z));
}

int16 main()
{
	int16 error_code, axis, axes;
	double p;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	
	begin_jogging();    
	axes = dsp_axes();
	printf("\nToggle bit #22 to latch %d axes.\n\n", axes); 
	
	init_io(2, IO_INPUT);
	stop_with_latch();    
	arm_latch(TRUE);
	
	while(!latch_status())
		;
	for (axis = 0; axis < axes; axis++)
	{
		get_latched_position(axis, &p);
		printf("Axis:%d Position:%12.0lf\n", axis, p);
	}
	
	printf("\n");
	printf("Press any key to disable jogging and clear status on all axes...\n");
	getch();
	
	recover_from_latch();    

	return 0;
}
