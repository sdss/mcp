/* COIL.C

:Sample coil winding application for 2 axes.

Axis 0 is the master and axis 1 is the cam.  As the master rotates the cam axis
 moves back and forth to place the wire evenly (similar to a fishing reel) on
 the master drum.

For this sample an acceleration and deceleration of the cammed axis is
 calculated to require exactly one sample.

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
	Revision 1.1  1999/08/31 16:42:58  briegel
	source for MEI library

*/


# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include "pcdsp.h"

# define MASTER              0
# define CAM                 1

# define CAM_DIST            1000.0

# define COUNTS_PER_REV      4000.0
# define REVS_PER_LAYER      5.0
# define LAYERS              2.0
# define MASTER_VEL          12500.0
# define MASTER_ACCEL        100000.0


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

int16 display(void)                                                                                                                              
{
	int16 m_flag = 0, c_flag = 0;
	double master, cam, acam;

    while (!kbhit())
    {   get_command(MASTER, &master);
        get_command(CAM, &cam);
        get_position(CAM, &acam);
        if((master > 40000.0) & (m_flag == 0))
        {
            printf("\nCam when master is at 40000: %12.2lf\n", cam); 
            m_flag = 1;
        }
        if((master > 1000) & (cam == 0) & (c_flag ==0))
        {
            printf("\nCam at zero master at: %12.2lf\n", master);
            c_flag = 1;
        }
        printf("Master: %12.2lf Cam: %12.2lf Cam Actual: %12.2lf\r", master, cam, acam);
    }
    getch();

    return 0;
}

int16 main()
{
    int16 error_code, i, dir;
    double  cam_time, cam_vel, cam_accel, sample_rate, time_ratio, master_dist;
    double  one_master_sample_CAM_time;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
    error(dsp_reset());		/* hardware reset */
    
    sample_rate = (double)dsp_sample_rate();


/******************************************************************************
Calculate the distance for the motion of master axis.  Since the change in
	position of the master axis is being substituted for the time of the cam
	axis we are really calculating the time for the cam to wind one layer.
*******************************************************************************/

   cam_time = COUNTS_PER_REV * REVS_PER_LAYER / sample_rate;  
    
/******************************************************************************
Calculate the time factor ratio between the cam and the master axis.  
          
	one count of "real" time = (1 / sample_rate) * (1 sample) = 1 sec
	one count of "cam" time = (1 / sample_rate) * (1 cam_sample)
	1 cam_sample = 1 master encoder count
	one count of "cam" time = (1 / sample_rate) * (1 master count)

Taking the ratio between "cam" and "real" times:

	time_ratio = (1 master count) / (1 sample)
	time_ratio = ((1 master count) / (1 sample)) * ((1 sec) / (1 sec))
	time_ratio = (1 master count / sec) / (1 sample / sec)
	time_ratio = (master velocity) / (sample_rate)
*******************************************************************************/

	time_ratio = MASTER_VEL / sample_rate;

/******************************************************************************
Convert the time intervel for one sample on the master axis to the cam time:

	one_master_sample_CAM_time =
		one_master_sample_REAL_time * MASTER_VEL / sample_rate 
	one_master_sample_CAM_time = one_master_sample_REAL_time * time_ratio
	one_master_sample_REAL_time = 1 / sample_rate
	one_master_sample_CAM_time = (1 / sample_rate) * time_ratio
*******************************************************************************/

	one_master_sample_CAM_time = time_ratio / sample_rate;

/*******************************************************************************
Now the velocity can be calculated by vel = dist / time.  For the cam velocity:

	cam_vel = CAM_DIST / cam_slew_time

	In order for the cam axis and the master axis to reach their respective 
	endpoints at the same time, the amount of time passed in 4 samples on the 
	master axis must be subtracted from the cam_time.  This is because the 
	previous calculation for the cam_time is the amount of time allotted for a
	move on the cam axis.  Two of the frames subtracted are for the acceleration 
	and deceleration of the cam axis.  The other two frames are dwell frames 
	between the moves. 

	cam_slew_time = cam_time - (4 * one_master_sample_CAM_time)
	cam_vel = CAM_DIST / (cam_time - (4 * one_master_sample_CAM_time))
*******************************************************************************/

	cam_vel = CAM_DIST / (cam_time - (4.0 * one_master_sample_CAM_time));

/******************************************************************************
The acceleration can be calculated by accel = vel / time.  In the case of the 
	cam axis:

	cam_accel = cam_vel / cam_accel_time
 
	The cam_accel_time is the time passed on the cam axis during on sample of
	the master axis, which is calculated above as one_master_sample_CAM_time.

	cam_accel_time = one_master_sample_CAM_time
	cam_accel = cam_vel / one_master_sample_CAM_time
******************************************************************************/

	cam_accel = cam_vel / one_master_sample_CAM_time;
    
	printf("READY? (press any key to start)\n");
	getch();                        /* hit a key to start */

	set_cam(MASTER, CAM, TRUE, CAM_ACTUAL);		/* enable the cam axis */

	dir = 1;  
	for (i = 0; i < LAYERS; i++)
	{   start_r_move(CAM, dir * CAM_DIST, cam_vel, cam_accel);
		dir = -dir;
	}
    
	master_dist = cam_time * LAYERS * sample_rate;
	start_r_move(MASTER, 2*master_dist, MASTER_VEL, MASTER_ACCEL/100);
	display();

	return 0;
}


/*******************************************************************************
For camming applications, time on the cam axis is no longer "real" time.  Time 
   is instead based on the change in position of the master axis.  Each 
   position count of the master axis is translated into a sample count on the
   slave axis.  For example: If the master is moving at a velocity of 10 counts
   per second, then in one second of "real" time, the time passed on the cammed 
   axis is equal to the amount of time it takes for 10 samples to execute.  
   This "cam" time can be calculated, but it is much easier to think in terms
   of the time_ratio calculated in the above sample program.  Using this 
   time_ratio, the following chart outlines the basic transformation equations
   from "real" time to "cam" time:

      "REAL" Time                "CAM" Time
--------------------------------------------------------------------------------
      MASTER_DISTANCE      |  MASTER_DISTANCE
      MASTER_VELOCITY      |  MASTER_VELOCITY
      MASTER_ACCELERATION  |  MASTER_ACCELERATION
      MASTER_JERK          |  MASTER_JERK
--------------------------------------------------------------------------------
      CAM_DISTANCE         |  CAM_DISTANCE
      CAM_VELOCITY         |  CAM_VELOCITY / time_ratio
      CAM_ACCELERATION     |  CAM_ACCELERATION / (time_ratio * time_ratio)
      CAM_JERK             |  CAM_JERK / (time_ratio * time_ratio * time_ratio)
--------------------------------------------------------------------------------
*******************************************************************************/
