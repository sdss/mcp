/* LINKSYNC.C

:Electronic gearing with synchronization based on I/O sensors.

This sample code link's two axes together and commands a velocity move on the
 master axis.  During the motion, the actual position of the slave axis is
 displayed based on two input sensors.  For this sample, the index inputs are
 used as the input sensors.

Based on keyboard input, a synchronization routine adjusts the offset distance
 between the input sensors.

Here is the synchronization algorithm:

 1) Read the slave's position when the first sensor goes high.
 2) Read the slave's position when the second sensor goes high.
 3) Calculate the offset distance.
 4) Command a motion to compensate for the offset distance.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <math.h>
# include "pcdsp.h"

# define MASTER			0
# define SLAVE			1
# define VELOCITY		500.0
# define ACCEL			4000.0

# define RATIO			1.0
# define MAX_DIST		8192	/* Slave distance between sensors */

# define SENSOR_PORT	6 		/* User I/O port */ 
# define M_MASK			0x04	/* Master sensor mask */
# define S_MASK			0x40	/* Slave sensor mask */


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

void synchronize(int16 axis, int16 port, int16 m_mask, int16 s_mask)
{
	int16 dir, value = 0;
	double delta, offset, s_pos_0, s_pos_1;

	while (!(value & m_mask))			/* wait for master sensor */
		get_io(port, &value);
	get_position(axis, &s_pos_0);	
		
	while (!(value & s_mask))			/* wait for slave sensor */
		get_io(port, &value);
	get_position(axis, &s_pos_1);		
	
	delta = s_pos_1 - s_pos_0;
	
	if (delta > 0.0)
		dir = 1;
	else	
		dir = -1;
		
	printf("\nDir: %d Delta: %6.0lf", dir, delta);
	/* Make sure the delta is a fraction of one full sensor cycle. */  
	delta = fmod((dir * delta), MAX_DIST);

	/* Calculate the shortest distance to synchronize the master and slave. */
	if (delta > (MAX_DIST / 2.0))
		offset = (dir * delta) - (dir * MAX_DIST);
	else
		offset = (dir * delta);	
		
	/* Command a relative move to compensate for the offset distance. */
	start_r_move(axis, offset, VELOCITY, ACCEL);
	printf("\nOffset: %6.0lf\n\n", offset);
}

void display(int16 axis, int16 sensor_port, int16 m_mask, int16 s_mask)
{
	int16 value;

	get_io(sensor_port, &value);
	
	if ((value & m_mask) || (value & s_mask))
		printf("I/O: 0x0%x Enc: %d\n", value, dsp_encoder(axis)); 
}


int16	main()
{
	int16 error_code, key, done = 0;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	set_home_index_config(MASTER, INDEX_ONLY);	/* use index for simulation */
	set_home_index_config(SLAVE, INDEX_ONLY);
	
	mei_link(MASTER, SLAVE, RATIO, LINK_ACTUAL);
	v_move(MASTER, VELOCITY, ACCEL);		/* simulate master */

	printf("\ns=synchronize, esc=quit\n");
	
	while (!done)
	{ 	display(SLAVE, SENSOR_PORT, M_MASK, S_MASK);

		if (kbhit())	/* key pressed? */
		{	key = getch();

			switch (key)
			{
				case 's':	/* Synchronize */
					printf("\nSynchronizing...\n");
					synchronize(SLAVE, SENSOR_PORT, M_MASK, S_MASK);
					break;
                
				case 0x1B:	/* <ESC> */
					v_move(MASTER, 0.0, ACCEL);		/* Stop the master */
					while (!motion_done(MASTER))
						;
					endlink(SLAVE);
					done = TRUE;
					break;
			}
		}
	}
	return 0;
}

