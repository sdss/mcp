/*
	sydsp.c 
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include <math.h>
#include "idsp.h"
#include "systems.h"

#define DEGREES		(3.14159265/180.0)


int16 FNTYPE sy_start_list(PSYSTEM psystem)
{
    int16 x;

    if (sy_sick(psystem))
		return dsp_error ;

	if (psystem->in_sequence)	/* is the previous sequence complete? */
		if (sy_end(psystem))
			return dsp_error;

    for(x = 0; x < psystem->axes; x++)
    {	psystem->a_o[x] = 0.0;
        psystem->v_o[x] = 0.0;
		get_last_command(psystem->axis[x], &(psystem->xx_o[x])) ;
		dsp_set_last_command(psystem->pdsp[x], psystem->axis[x], 
					psystem->xx_o[x]) ;	/* */
		psystem->port[x] = psystem->l_port[x] = 0;
		psystem->andmask[x] = psystem->l_andmask[x] = -1 ;
		psystem->ormask[x] = psystem->l_ormask[x] = 0 ;
		psystem->pdsp[x]->flags[psystem->axis[x]] |= DF_AXIS_BUSY ;
    }
	psystem->t = 0L ;
	psystem->t2 = 0L ;
	sy_set_gate(psystem) ;
	psystem->hold = TRUE ;
	psystem->in_sequence = TRUE ;
	psystem->points = 0;
    return(0);
}


int16 FNTYPE psystem_move_frame(PSYSTEM psystem, int16 i, double a, unsigned long t, int16 a_frame)
{
	FRAME frame ;
	LFIXED lfixed ;
	PDSP pdsp = dspPtr ;

	dspPtr = psystem->pdsp[i] ;
	frame_clear(&frame);
	if (dspPtr->FRAME_ALLOCATE(&frame, dspPtr, psystem->axis[i]) == DSP_OK)
    {
    /*    frame.f.control = FCTL_RELEASE | 0x16; */
    	frame.f.control = dspPtr->frame_control[psystem->axis[i]] & (~0xFF) ;
    	frame.f.control |= 0x16 ;
        if (psystem->hold)
        	frame.f.control |= FCTL_HOLD ;
        frame.f.time = t;
		ipcdsp_fixed(a, &lfixed) ;
		copy_lfixed_to_fixd(frame.f.accel, lfixed);
		ipcdsp_fixed(psystem->v_o[i], &lfixed) ;
		copy_lfixed_to_fixd(frame.f.velocity, lfixed);
		ipcdsp_fixed(psystem->xx_o[i], &lfixed) ;
		copy_lfixed(frame.f.position, lfixed);
		frame.f.trig_update =  FUPD_ACCEL | FUPD_POSITION | FUPD_VELOCITY | FTRG_TIME;
        if(psystem->l_port[i] && a_frame)
        {	DSP_DM * datum = (DSP_DM *) &(frame.f) ;
			frame.f.trig_update |=  FUPD_OUTPUT ;
            frame.f.output = OUTPUT_JERK;
			datum[frame.f.output] = psystem->l_port[i] ;
			datum[frame.f.output + 1] = psystem->l_ormask[i] ;
			datum[frame.f.output + 2] = psystem->l_andmask[i] ;
			psystem->l_port[i] = 0;
			psystem->l_andmask[i] = -1 ;
			psystem->l_ormask[i] = 0;
        }
		frame.f.trig_action = CHECK_FRAMES;
		dspPtr->FRAME_DOWNLOAD(&frame);
	}
    psystem->xx_o[i] += (((double) t) * psystem->v_o[i]) + ((double)(t) * (double)(t+1)) * a/2.0;
    psystem->v_o[i] += (((double) t) * a);

	dspPtr = pdsp ;
	return dsp_error ;
}
int16 FNTYPE psystem_vel_frame(PSYSTEM psystem, int16 i, double v, unsigned long t)
{
	FRAME frame ;
	LFIXED lfixed ;
	PDSP pdsp = dspPtr ;

	dspPtr = psystem->pdsp[i] ;
	frame_clear(&frame);
	if (dspPtr->FRAME_ALLOCATE(&frame, dspPtr, psystem->axis[i]) == DSP_OK)
    {
    /*    frame.f.control = FCTL_RELEASE | 0x16; */
    	frame.f.control = dspPtr->frame_control[psystem->axis[i]] & (~0xFF) ;
    	frame.f.control |= 0x16 ;
        if (psystem->hold)
        	frame.f.control |= FCTL_HOLD ;
        frame.f.time = t;
		ipcdsp_fixed(v, &lfixed) ;
		copy_lfixed_to_fixd(frame.f.velocity, lfixed);
		ipcdsp_fixed(psystem->xx_o[i], &lfixed) ;
		copy_lfixed(frame.f.position, lfixed);
		frame.f.trig_update =  FUPD_POSITION | FUPD_VELOCITY | FTRG_TIME;
        if(psystem->port[i])
        {	DSP_DM * datum = (DSP_DM *) &(frame.f) ;
			frame.f.trig_update |=  FUPD_OUTPUT ;
            frame.f.output = OUTPUT_JERK;
			datum[frame.f.output] = psystem->port[i] ;
			datum[frame.f.output + 1] = psystem->ormask[i] ;
			datum[frame.f.output + 2] = psystem->andmask[i] ;
			psystem->port[i] = 0;
			psystem->andmask[i] = -1 ;
			psystem->ormask[i] = 0;
        }
		frame.f.trig_action = CHECK_FRAMES;
		dspPtr->FRAME_DOWNLOAD(&frame);
	}
    psystem->xx_o[i] += (((double) t) * v);
    psystem->v_o[i] = v;

	dspPtr = pdsp ;
	return dsp_error ;
}
int16 FNTYPE psystem_dwell_frame(PSYSTEM psystem, int16 i, unsigned long t)
{
	FRAME frame ;
	PDSP pdsp = dspPtr ;

	dspPtr = psystem->pdsp[i] ;
	frame_clear(&frame);
	if (dspPtr->FRAME_ALLOCATE(&frame, dspPtr, psystem->axis[i]) == DSP_OK)
    {
    	frame.f.control = dspPtr->frame_control[psystem->axis[i]] & (~0xFF) ;
    	frame.f.control |= 0x16 ;
        if (psystem->hold)
        	frame.f.control |= FCTL_HOLD ;
        frame.f.time = t;
		frame.f.trig_update =  FUPD_ACCEL | FUPD_VELOCITY | FTRG_TIME;
		frame.f.trig_action = CHECK_FRAMES;
		dspPtr->FRAME_DOWNLOAD(&frame);
	}

	dspPtr = pdsp ;
	return dsp_error ;
}


int16 FNTYPE psystem_last_frame(PSYSTEM psystem, int16 i)
{
	FRAME frame ;
	LFIXED lfixed ;
	PDSP pdsp = dspPtr ;
	double p ;

	dspPtr = psystem->pdsp[i] ;
	frame_clear(&frame);
	if (dspPtr->FRAME_ALLOCATE(&frame, dspPtr, psystem->axis[i]) == DSP_OK)
    {
    	get_last_command(psystem->axis[i], &p) ;		/* */
		ipcdsp_fixed(p, &lfixed) ;
		copy_lfixed(frame.f.position, lfixed);
/*        frame.f.control = FCTL_RELEASE | 0x16; */
    	frame.f.control = dspPtr->frame_control[psystem->axis[i]] & (~0xFF) ;
    	frame.f.control |= 0x16 ;
		frame.f.trig_update =  FUPD_ACCEL | FUPD_POSITION | FUPD_VELOCITY;
		dspPtr->FRAME_DOWNLOAD(&frame);
	}
	dspPtr = pdsp ;
	return dsp_error ;
}


int16 FNTYPE sy_add_point(PSYSTEM psystem, VECT point)
{
    double l, vel, acc, aa, command, r1 = 0.0, r2 = 0.0;
    long m1, m2, t1, t2, ta = 0;
    int16 i, n_frames;

    if (sy_sick(psystem))
    	return dsp_error ;

    if(((!psystem->mode) && (psystem->accel == 0.0)) || (psystem->speed == 0.0))
      return (dsp_error = DSP_ILLEGAL_PARAMETER) ;

    if((psystem->a_scale == 0.0) || (psystem->v_scale == 0.0))
      return (dsp_error = DSP_ILLEGAL_CONVERSION);

    /* Fifo checking stuff. */
    n_frames = 2*psystem->axes;
    if((n_frames+3*psystem->axes) > fifo_space())	/* leave room for end_point_list */
        return dsp_error = DSP_NO_ROOM;

	psystem->points++ ;

    l = 0.0;
    for(i = 0; i < psystem->axes; i++)
    {   double w;
        double x =
        		point[i] *
        		dspPtr->conversion[psystem->axis[i]].countsPerDistance *
        		psystem->ratio[i] ;
        get_last_command(psystem->axis[i], &command) ;
        psystem->workspace[i] = x - command ;
		dsp_set_last_command(dspPtr, psystem->axis[i], x) ;	/* */
        w = psystem->workspace[i] / (dspPtr->conversion[psystem->axis[i]].countsPerDistance * psystem->ratio[i]);
        l += w * w;
    }
   
    l = sqrt(l);

	if (!l)
		return (dsp_error = DSP_NO_DISTANCE);

	vel = psystem->speed * psystem->v_scale ;
	acc = psystem->accel * psystem->a_scale ;

    if(psystem->mode == 0)
    {
        m1 = (long)(vel/acc + 0.5);
	    if(m1 == 0L)		m1 = 2L;
	    if (m1 & 1L)		m1++ ;

        m2 = (long)(l/vel + 0.5);
        if(m2 < m1)
        {	m1 = (long)(sqrt(l/acc) + 0.5);
		    if(m1 == 0L)	m1 = 2L;
		    if (m1 & 1L)	m1++ ;

            m2 = m1;
        }
        t1 = m1;
        t2 = m2 - m1;

        aa = 1.0/((double)m1 * (double)m2);

	    if (t1 == psystem->t || !psystem->t)
	    {	ta = t1;
		    r1 = 1.0 ;
		    r2 = 1.0;
	    }
	    if (psystem->t > t1)
	    {	ta = t1;
		    r1 = 1.0;
		    r2 = (double) psystem->t / (double) ta ;
		    psystem->t2 += (psystem->t - ta) / 2L ;
	    }
	    if ((t1 > psystem->t) && psystem->t)
	    {	ta = psystem->t ;
		    r1 = (double) t1 / (double) ta ;
		    r2 = 1.0 ;
		    t2 += (t1 - ta) / 2L ;
	    }

        for(i = 0; i < psystem->axes; i++)
        {
            double a, a_new, a_old;

            a = psystem->workspace[i] * aa;
            a_new = a * r1 ;
            a_old = psystem->a_o[i] * r2 ;
            psystem->a_o[i] = a ;

		    if (psystem->t2)
		    {	if (psystem->move_frame(psystem, i, 0.0, psystem->t2, FALSE))
				    return dsp_error ;
		    }
		    if (psystem->move_frame(psystem, i, a_new - a_old, ta, TRUE))
			    return dsp_error ;

		    psystem->l_port[i] = psystem->port[i];
		    psystem->l_andmask[i] = psystem->andmask[i] ;
		    psystem->l_ormask[i] = psystem->ormask[i] ;
		    psystem->port[i] = 0;
		    psystem->andmask[i] = -1 ;
		    psystem->ormask[i] = 0;
        }

    }
    else
    {
        t1 = (long)(l/vel + 0.5);
	    if(t1 == 0L)		t1 = 1L;
        t2 = (long)((psystem->corner_sharpness * (double)psystem->filter_length) + 0.5);

        aa = 1.0/((double)t1);

        for(i = 0; i < psystem->axes; i++)
        {
            double v;

            v = psystem->workspace[i] * aa;
		    if (t1)
		    {
                if(psystem_vel_frame(psystem,i,v,t1))
				    return dsp_error ;
		    }
		    if (t2)
		    {
                if(psystem_dwell_frame(psystem,i,t2))
				    return dsp_error ;
		    }
		    psystem->l_port[i] = psystem->port[i];
		    psystem->l_andmask[i] = psystem->andmask[i] ;
		    psystem->l_ormask[i] = psystem->ormask[i] ;
		    psystem->port[i] = 0;
		    psystem->andmask[i] = -1 ;
		    psystem->ormask[i] = 0;
        }
    }

    if( (psystem->start_points > 0) &&
    	(psystem->points > psystem->start_points))
    {
    	if (system_gated(psystem))
    		sy_start(psystem) ;
    }

    psystem->t = t1;
    psystem->t2 = t2 ;
    psystem->hold = FALSE ;

    return(0);
}


int16 FNTYPE sy_add_arc(PSYSTEM psystem, VECT center, double angle, double division)
{
   double      x,
               y,
               dx,
               dy,
               cx,
               cy,
               command;
   double      theta,
               d_theta,
               save_accel,
               radius,
               r2;
   int16         n_steps,
               i,
               n_frames;
   VECT        arc_point;
   double      sine,
               cosine,
					pos_tmp;

   if(sy_sick(psystem))
      return dsp_error ;

   save_accel = psystem->accel ;

   theta = angle * DEGREES;
   if(theta == 0.0)
      return(0);

   d_theta = division * DEGREES;
   if(d_theta == 0.0)
      return (dsp_error = DSP_ILLEGAL_PARAMETER) ;

/* Read the current position from the board. */
   for(i = 0; i < psystem->axes; i++)
	{	get_last_command(psystem->axis[i], &pos_tmp);
		arc_point[i] = pos_tmp/psystem->ratio[i];
	}

/* generate an arc from current point to end point */
   cx = center[0];     /* center of circle */
   cy = center[1];     /* center of circle */
  
  	command = arc_point[0] ;
   command /= (dspPtr->conversion[psystem->axis[0]].countsPerDistance * psystem->ratio[0]);
	cx = center[0] ;
	dx = command - cx ;
	command = arc_point[1];
   command /= (dspPtr->conversion[psystem->axis[1]].countsPerDistance * psystem->ratio[1]);
	cy = center[1] ;
	dy = command - cy;

   if(theta < 0.0)
   {  if(d_theta > -theta)
         d_theta = -theta;
   }
   else
   {  if(d_theta > theta)
         d_theta = theta;
   }
   n_steps = (int16)(theta / d_theta);
   if(n_steps < 0)
      n_steps = -n_steps;

   /* Fifo checking stuff. */
   n_frames = 2*psystem->axes * n_steps;
   if((n_frames+3*psystem->axes) > fifo_space())	/* leave room for end_point_list */
      return dsp_error = DSP_NO_ROOM;

   sine = sin (theta / n_steps);
   cosine = cos (theta / n_steps);

   if(psystem->optimize_arcs)
   {  r2 = dx * dx + dy * dy ;
      if(r2 < 1e-12)
         return DSP_OK ;
      radius = sqrt(r2) ;
      psystem->accel = psystem->speed * psystem->speed / (radius * d_theta) ;
   }
   if(psystem->accel < 0)
      psystem->accel = -psystem->accel ;

   for(i = 0; i < n_steps; i++)
   {
      int16 err;

      x = dx * cosine + dy * sine;
      y = dy * cosine - dx * sine;

      arc_point[0] = cx + x;
      arc_point[1] = cy + y;

      err = sy_add_point(psystem, arc_point);
      if(err)
         return(err);

      dx = x;
      dy = y;
   }

   psystem->accel = save_accel ;
      return(0);
}


int16 FNTYPE sy_end(PSYSTEM psystem)
{
    int16 i;

    if (sy_sick(psystem))
    	return dsp_error ;

    psystem->in_sequence = FALSE ;

    if(psystem->mode == 0)
    {
        for(i = 0; i < psystem->axes; i++)
        {	if (psystem->t2)
    		    if (psystem->move_frame(psystem, i, 0.0, psystem->t2, FALSE))
    			    return dsp_error ;

    	    if(psystem->move_frame(psystem, i, -psystem->a_o[i], psystem->t, TRUE))
                return dsp_error;

		    psystem->pdsp[i]->flags[psystem->axis[i]] &= ~(DF_AXIS_BUSY) ;
            psystem->last_frame(psystem, i);
		    psystem->port[i] = psystem->l_port[i] = 0;
		    psystem->andmask[i] = psystem->l_andmask[i] = -1 ;
		    psystem->ormask[i] = psystem->l_ormask[i] = 0 ;
        
        }
    }
    else
    {
        for(i = 0; i < psystem->axes; i++)
        {
		    psystem->pdsp[i]->flags[psystem->axis[i]] &= ~(DF_AXIS_BUSY) ;
            psystem->last_frame(psystem, i);
		    psystem->port[i] = psystem->l_port[i] = 0;
		    psystem->andmask[i] = psystem->l_andmask[i] = -1 ;
		    psystem->ormask[i] = psystem->l_ormask[i] = 0 ;
        
        }
    }
    return(dsp_error);
}

int16 FNTYPE sy_start(PSYSTEM psystem)
{	return sy_clear_gate(psystem) ;
}


int16 FNTYPE sy_stop(PSYSTEM psystem)
{	return sy_set_stop(psystem);
}


int16 FNTYPE sy_set_speed(PSYSTEM psystem, double speed)
{	psystem->speed = speed ;
	return dsp_error ;
}


int16 FNTYPE sy_set_accel(PSYSTEM psystem, double accel)
{	psystem->accel = accel ;
    return dsp_error ;
}


int16 FNTYPE sy_arc_division(PSYSTEM psystem, double degrees)
{
    psystem->arc_division = degrees;
    return(0);
}


int16 FNTYPE sy_done(PSYSTEM psystem)
{	return system_done(psystem) ;
}

int16 FNTYPE sy_sharpness(PSYSTEM psystem, double sharpness)
{	psystem->corner_sharpness = sharpness ;
	return dsp_error ;
}

int16 FNTYPE sy_set_mode(PSYSTEM psystem, int16 mode)
{	psystem->mode = mode;
	return dsp_error ;
}
int16 FNTYPE sy_set_filter(PSYSTEM psystem, int16 length)
{
    int16 x,ax,addr;

    if (sy_sick(psystem))
    	return dsp_error ;

    for(x = 0; x < psystem->axes; x++)
    {   ax = psystem->axis[x];
        addr = dspPtr->global_data + dspPtr->axes + 13 + ax;
        dsp_write_dm(addr, length);
    }
    psystem->filter_length = 1 << length;
	return dsp_error ;
}

