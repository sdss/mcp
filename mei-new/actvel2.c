/* ACTVEL2.C


:Read, calculate and print actual velocity over a 3 millisecond period.

A simple move is commanded and the actual velocity (counts/sec) is printed 
 to the screen.  The velocity is calculated using pcdsp_transfer_block(...), 
 which reads the current velocity register in the DSP's data memory.  The 
 actual velocity is read three times and averaged.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "idsp.h"


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

double actual_velocity(int16 axis)
{
	P_DSP_DM current_v_addr = dspPtr->data_struct + DS_CURRENT_VEL + DS(axis); /* velocity in cts/sample */ 
	
	DSP_DM VelA[1], VelB[1], VelC[1];
	double vel;
	
	pcdsp_transfer_block(dspPtr, TRUE, FALSE, current_v_addr, 1, VelA);
	pcdsp_transfer_block(dspPtr, TRUE, FALSE, current_v_addr, 1, VelB);  
	pcdsp_transfer_block(dspPtr, TRUE, FALSE, current_v_addr, 1, VelC);  

	vel = (( VelA[0] + VelB[0] + VelC[0])/3) * dsp_sample_rate();
	
	return vel;
}

int16 main()
{
	int16 error_code;
	
	error_code = do_dsp();  /* initialize communication with the controller */
	error(error_code);      /* any problems initializing? */
	error(dsp_reset());
	
	printf("Press any key to exit program \n");
	start_move(0, 100000.0, 10000.0, 100000.0);

	while(!kbhit())
	{
	   printf(" %lf\r", actual_velocity(0));
	}
	getch();
	
	return 0;
}
