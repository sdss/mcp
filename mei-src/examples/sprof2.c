/* SPROF1.C

:Alternate algorithm for S-Curve profile motion.

In the standard function library, start_s_move(...) and s_move(...) are used to
 generate S-Curve profile motions.  These functions have some guidelines that
 are documented in the C Programming manual.  The algorithm to generate the
 S-Curve profiles is optimized for calculation time and to guarantee a smooth
 profile.

This sample demonstrates an alternate method of generating a S-Curve motion
 profiles.  Many of the functions in this sample are from the lower level
 routines in the standard function library.
 
Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/


	
# include <stdio.h>
# include <stdlib.h>
# include <math.h>
# include <conio.h>
# include <string.h>
# include <dir.h>
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

# define S_FRAME(t, j)	 frame_m(&frame, "-0l tj u d", axis, (double) (t),(double) (j), FUPD_JERK | FTRG_TIME)

int16 pcdsp_s_curve(int16 axis, double delta, double vel, double accel, double jerk)
{
	FRAME    frame;
	double   ta, tv, tj, delta_j;
	int16      retval, sw = 0;

	printf("\nInitial values (time in samples)\n");
	printf("delta: %lf\n", delta);
	printf("velocity: %lf\n", vel);
	printf("accel: %lf\n", accel);
	printf("jerk: %lf\n", jerk);
	
	if(delta < 0)
	{	sw = 1;
		delta = -delta;
	}

	/* Calculate the accel time.  If the accel time is negative, then the target
		accel must be adjusted so that ta = 0.  This will preserve the specified
		velocity.  Note that a negative accel time is not valid, because it is
		impossible to go backwards in time.
	*/ 
	if(accel*accel/jerk > vel)
	{ 	ta = 0.0;
		accel = sqrtl(jerk*vel);
	}
	else
		ta = ((vel - accel*accel/jerk)/accel);

	/* Calculate the distance during the accel and jerk profile portion. */
	delta_j = (2.0*(accel*accel*accel)/(jerk*jerk))
				+ ((3.0*accel*accel*ta)/jerk)
				+ (accel*ta*ta);

	if(delta_j > delta)		/* Is the distance during jerk and accel too far? */
	{	tv = 0.0; 			/* Remove the constant velocity profile */      
		if(ta > 0.0)
		{	double   x;
			x = sqrt(1+(4*delta*jerk*jerk/(accel*accel*accel)));
			
			if(x > 3.0)
			{	tj = accel/jerk;
				ta = (0.5*tj)*(x - 3.0);
			}
			else
				ta = 0.0;
		}
		if(ta < 1.0)
		{	ta = 0.0;       
			tj = pow((delta/(2.0*jerk)), .3333333);
		}
	}                                         
	else                                      
	{	tv = (delta - delta_j)/vel;
		tj = accel/jerk;
	}

	if(tj < 1.0)
	{	dsp_error = DSP_ILLEGAL_PARAMETER;
		return dsp_error;
	}
		
	/* Reverse calculate to eliminate roundoff errors.  Since the DSP uses 
		whole time periods (no fractional time periods) to calculate the
		trajectory, the jerk, accel, or vel, may need to be adjusted to
		guarantee the accuracy of the final position.
	*/
	tj = floor(tj);
	accel = tj*jerk;

	if(ta != 0.0)		/* Is there an accel portion? */
	{	if(tv == 0.0)
		{	double   x;
			x = sqrt(1+(4*delta*jerk*jerk/(accel*accel*accel)));
			ta = (0.5*accel/jerk)*(x - 3.0);		/* Recalculate ta */
		}
		else
			ta = ((vel - accel*accel/jerk)/accel);	/* Recalculate ta */
		ta = floor(ta);
	}
	
	if(tv != 0.0) 		/* Is there a vel portion? */
	{	delta_j = (2.0*(accel*accel*accel)/(jerk*jerk))
					+ ((3.0*accel*accel*ta)/jerk)
					+ (accel*ta*ta);             
		vel = ta*accel + (accel*accel/jerk);	/* Recalculate vel */
		tv = (delta - delta_j)/vel;				/* Recalculate tv */
		tv = floor(tv);
	}

	jerk = delta / (tj * (tj + ta) * (2 * tj + ta + tv));

	if(sw)
		jerk = -jerk;

	printf("\nAfter calculations (time in samples)\n");
	printf("delta: %lf\n", delta);
	printf("velocity: %lf\n", vel);
	printf("accel: %lf\n", accel);
	printf("jerk: %lf\n", jerk);
	printf("tv: %lf\n", tv);
	printf("ta: %lf\n", ta);
	printf("tj: %lf\n\n", tj);
	
	/*  Now, download the frames.  All frames trigger on time and update the
		jerk only.  Don't download frames with zero time. */

	retval = DSP_OK ;
	dsp_control(axis,FCTL_HOLD,TRUE);
	if(tj && !retval)
		retval = S_FRAME(tj, jerk);
	dsp_control(axis,FCTL_HOLD,FALSE);
	if(ta && !retval)
		retval = S_FRAME(ta, 0);
	if(tj && !retval)
		retval = S_FRAME(tj, -jerk);
	if(tv && !retval)
		retval = S_FRAME(tv, 0);
	if(tj && !retval)
		retval = S_FRAME(tj, -jerk);
	if(ta && !retval)
		retval = S_FRAME(ta, 0);
	if(tj && !retval)
		retval = S_FRAME(tj, jerk);

   return retval;
}

/*	This function converts the units for the arguments into counts,
	counts/sample, counts/(sample*sample), counts/(sample*sample*sample)
*/
static int16 ipcdsp_standardize(PDSP dsp, int16 axis, P_DOUBLE final, P_DOUBLE velocity, P_DOUBLE acceleration, P_DOUBLE jerk)
{
   LFIXED   vel, accel, j, pp;

   if(!(ipcdsp_fixed_pos(dsp, axis, *final, &pp) ||
         ipcdsp_fixed_vel(dsp, axis, *velocity, &vel) ||
			ipcdsp_fixed_accel(dsp, axis, *acceleration, &accel) ||
			ipcdsp_fixed_jerk(dsp, axis, *jerk, &j)))
	{	*final = ipcdsp_double(&pp);
		*velocity = ipcdsp_double(&vel);
		*acceleration = ipcdsp_double(&accel);
		*jerk = ipcdsp_double(&j);
	}

	if((sign_lfixed(vel) == 0) ||
      (sign_lfixed(accel) == 0) ||
		(sign_lfixed(j) == 0))
         dsp_error = DSP_ILLEGAL_PARAMETER;

	return dsp_error;
}

static int16 ipcdsp_last_frame(PDSP dsp, int16 axis, double x)
{
	FRAME frame ;
	LFIXED lfixed ;

	frame_clear(&frame);

	if (frame_allocate(&frame, dsp, axis) == DSP_OK)
	{
		ipcdsp_fixed(x, &lfixed) ;
		copy_lfixed(frame.f.position, lfixed) ;
		frame.f.time = 1;
		frame.f.trig_update =	FUPD_POSITION | FUPD_VELOCITY |
								FUPD_ACCEL | FTRG_TIME;
		frame.f.trig_action = NEW_FRAME ;

		frame_download(&frame);
	}
	return dsp_error ;
}

int16 ipcdsp_load_s_curve(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{  double   x;
   if(!ipcdsp_standardize(dsp, axis, &final, &vel, &accel, &jerk))
   {	x = final - dsp_last_command(dsp, axis);
    	if (pcdsp_s_curve(axis, x, vel, accel, jerk))
    		return dsp_error;
		ipcdsp_last_frame(dsp, axis, final);
		dsp_set_last_command(dsp, axis, final);
   }
   return dsp_error;
}

int16 ipcdsp_s_curve(PDSP dsp, int16 axis, double final, double vel, double accel, double jerk)
{  if(!ipcdsp_load_s_curve(dsp, axis, final, vel, accel, jerk))
      pcdsp_dwell_frame(dsp, axis);
   return dsp_error;
}

int16 start_s_curve(int16 axis, double final, double vel, double accel, double jerk)
{  int16   r;
   if(dsp_set_gate(dspPtr, axis))
      return dsp_error;
   r = ipcdsp_s_curve(dspPtr, axis, final, vel, accel, jerk);
   dsp_reset_gate(dspPtr, axis);
   return (dsp_error = r);
}

int16 usage(char * program)
{
	char buffer[80] ;
	fnsplit(program, NULL, NULL, buffer, NULL) ;
	strlwr(buffer) ;
	printf("Usage: %s axis, position, vel, accel, jerk\n", buffer) ;
	return 0;
}

int16 main(int16 argc, char * argv[])
{
	int16	axis, error_code;
	double pos, vel, accel, jerk, p, v, a, j;
	char *endp;

	if(argc < 5)
	{	usage(argv[0]);
		return -1;
	}	
		
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	axis = (int16)strtol(argv[1], &endp, 10);
	pos = (double)strtol(argv[2], &endp, 10);
	vel = (double)strtol(argv[3], &endp, 10);
	accel = (double)strtol(argv[4], &endp, 10);
	jerk = (double)strtol(argv[5], &endp, 10);
	
	set_position(axis, 0.0);		/* start at position 0.0 */
	
	printf("\nInput values (time in seconds)\n");
	printf("delta: %lf\n", pos);
	printf("velocity: %lf\n", vel);
	printf("accel: %lf\n", accel);
	printf("jerk: %lf\n", jerk);

	start_s_curve(axis, pos, vel, accel, jerk);
	
	while(!kbhit())
	{	get_command(axis, &p);
		get_velocity(axis, &v);
		get_accel(axis, &a);
		get_jerk(axis, &j);
		printf("\rX: %12.0lf V:%12.0lf A:%12.0lf J:%12.0lf", p, v, a, j);
	}
	getch();
   
	return 0;
}
