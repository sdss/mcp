/* SETTLE3.C

:Determine when the actual motion has settled.

When motion is commanded to an axis, the DSP generates a real-time command
 position trajectory.  Every sample, the DSP calculates an axis' output (analog
 voltage or pulse rate) based on a PID servo control algorithm.  The input to 
 the PID control algorithm is the position error.  The position error equals the
 difference between the command position and the actual position (feedback). 
 
The commanded motion is complete when motion_done(...) returns non-zero.
 Determining the completion of the actual motion is more difficult.  Since the
 DSP is always executing a PID algorithm to control the closed loop system, 
 the motion of the motor is based on the system's response. 
 
This sample shows how to determine the completion of the motion based on the
 feedback device.  The motor is considered "settled" when the following
 criteria are met:
 
 1) The position error is less than a specified value for 'n' consecutive reads.
 2) The rate of change of position error is zero for 'n' consecutive reads.   
 
The rate of change of position error is calculated by the DSP as the difference
 between the position error in the current sample, and the position error in the
 previous sample.  The default sample rate is 1250 samples per second.
 
Both the rate of change of position error and the position error are read in the
 same sample period using the transfer block.  The transfer block is a special
 DSP feature that copies a block of internal memory (up to 64 words) to external
 memory.  Then the host CPU can read the information from external memory.

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
	Revision 1.1  1999/08/31 16:43:51  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <conio.h>
# include <stdlib.h>
# include <dos.h>
# include "pcdsp.h"
# include "idsp.h"

# define AXIS			0
# define DISTANCE		20000.0
# define VELOCITY		100000.0
# define ACCELERATION	5000000.0

# define ERROR_WINDOW	5	/* position error in counts */
# define SETTLE_COUNTS	4	/* number of consecutive reads to define settled */

# define BUFFER_SIZE	1000

int16
	p_err[BUFFER_SIZE],	/* position error from DSP */
	vel[BUFFER_SIZE];	/* actual velocity from DSP */
	

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

int16 get_settle(int16 axis, int16 * error, int16 * actvel)
{
	P_DSP_DM addr = dspPtr->data_struct + DS_D(0) + DS(axis); 
	DSP_DM buffer[2];

	pcdsp_transfer_block(dspPtr, TRUE, FALSE, addr, 2, buffer);
	
	* error = buffer[0];
	* actvel = buffer[1];

	return dsp_error;
}
	
int16	main()
{
	int16
		i, error_code,
		settle_counter = 0,
		data_point = 0;
		int16 start, end;
	double settle_time;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
	start_r_move(AXIS, DISTANCE, VELOCITY, ACCELERATION);
	while (!motion_done(AXIS))	/* wait for the commanded motion to complete */
		;	

	start = get_dsp_time();	/* read the dsp sample clock */
	
	while (settle_counter < SETTLE_COUNTS)
	{
		get_settle(AXIS, p_err + data_point, vel + data_point);
		
		if (!vel[data_point] && (p_err[data_point] < ERROR_WINDOW) &&
				(p_err[data_point] > (-ERROR_WINDOW)))
			settle_counter++;
		else
			settle_counter = 0;

		if (data_point < BUFFER_SIZE)
			data_point++;
	}
	end = get_dsp_time();	/* read the dsp sample clock */
	
	/* Display the response of the system. */
	for (i = 0; i < data_point; i++)
		printf("\n%4d Vel: %4d Err: %4d", i, vel[i], p_err[i]);
		
	settle_time = (end - start) * 1000.0 / dsp_sample_rate();
	printf("\nSettling time (msec): %6.4lf", settle_time);

    return 0;
}
