/*
	hldsp.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include "idsp.h"
#	include "systems.h"

#	include <stddef.h>

#ifdef MEI_MMAP

SYSTEM * hldsp_system = NULL ;

#	else

static int16 mei_axis[PCDSP_MAX_AXES] ;
static PDSP mei_pdsp[PCDSP_MAX_AXES] ;
static double mei_xx_o[PCDSP_MAX_AXES], mei_v_o[PCDSP_MAX_AXES], mei_a_o[PCDSP_MAX_AXES],
	mei_workspace[PCDSP_MAX_AXES * 2], mei_ratio[PCDSP_MAX_AXES] ;
static int16 mei_port[PCDSP_MAX_AXES], mei_andmask[PCDSP_MAX_AXES], mei_ormask[PCDSP_MAX_AXES],
	mei_l_port[PCDSP_MAX_AXES], mei_l_andmask[PCDSP_MAX_AXES], mei_l_ormask[PCDSP_MAX_AXES];

static SYSTEM mei_system ;

SYSTEM * hldsp_system = &mei_system ;

#	endif


int16 FNTYPE start_point_list(void)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_start_list(hldsp_system) ;
}

int16 FNTYPE add_point(VECT point)
{	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_add_point(hldsp_system, point) ;
}

int16 FNTYPE add_arc(VECT center, double angle, double division)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_add_arc(hldsp_system, center, angle, division);
}

int16 FNTYPE end_point_list(void)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_end(hldsp_system) ;
}

int16 FNTYPE start_motion(void)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_start(hldsp_system) ;
}

int16 FNTYPE stop_motion(void)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP);

	return sy_stop(hldsp_system) ;
}

int16 FNTYPE set_points(int16 points)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_set_points(hldsp_system, points) ;
}

int16 FNTYPE set_move_speed(double speed)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_set_speed(hldsp_system, speed) ;
}

int16 FNTYPE set_move_accel(double accel)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_set_accel(hldsp_system, accel) ;
}

int16 FNTYPE set_corner_sharpness(double sharpness)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_sharpness(hldsp_system, sharpness) ;
}

int16 FNTYPE all_done(void)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_done(hldsp_system) ;
}

int16 FNTYPE set_arc_division(double degrees)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_arc_division(hldsp_system, degrees) ;
}

int16 FNTYPE set_path_mode(int16 mode)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_set_mode(hldsp_system, mode) ;
}

int16 FNTYPE set_path_filter(int16 length)
{
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP) ;

	return sy_set_filter(hldsp_system, length) ;
}

/*	----------------------------------------------------------------------	*/


static VECT move_vect ;

int16 LOCAL_FN add_vect(void)
{	
   int16      e;
	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP);

    if (sy_in_seq(hldsp_system))
        return add_point(move_vect);
    else
    {	if (!start_point_list())
		{  e = add_point(move_vect);
         if (!e)
			{	end_point_list();
				start_motion();
			}
         else
            return e;
		}
    }
    return dsp_error;
}

int16 FNTYPE move_2(double x, double y)
{	move_vect[0] = x ;
	move_vect[1] = y ;
	return add_vect() ;
}

int16 FNTYPE move_3(double x, double y, double z)
{	move_vect[0] = x;
    move_vect[1] = y;
    move_vect[2] = z;
    return add_vect() ;
}

int16 FNTYPE move_4(double x, double y, double z, double w)
{	move_vect[0] = x;
    move_vect[1] = y;
    move_vect[2] = z;
    move_vect[3] = w;
    return add_vect() ;
}


int16 FNTYPE move_n(P_DOUBLE d)
{	int16  i;

	if (sy_sick(hldsp_system))
		return dsp_error ;

	for (i = 0; i < hldsp_system->axes; i++)
		move_vect[i] = d[i] ;

    return add_vect() ;
}


int16 FNTYPE arc_2(double x_center, double y_center,  double angle)
{
   int16      e;
	move_vect[0] = x_center;
	move_vect[1] = y_center;

	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP);

   if(sy_in_seq(hldsp_system))
      return add_arc(move_vect, angle, hldsp_system->arc_division);
   else
   {  if (!start_point_list())
      {  e = add_arc(move_vect, angle, hldsp_system->arc_division);  
         if (!e)
         {  end_point_list();
            start_motion();
         }	
         else
            return e;
		}
    }
    return dsp_error;
}


int16 FNTYPE map_axes(int16 n_axes, int16 *map_array)
{
#ifdef MEI_MMAP
	if (hldsp_system)
		hldsp_system = rm_system(hldsp_system) ;
	hldsp_system = imk_system(n_axes, map_array) ;
#	else
	hldsp_system = &mei_system ;
	mei_system.axis = mei_axis ;
	mei_system.pdsp = mei_pdsp ;
	mei_system.xx_o = mei_xx_o ;
	mei_system.v_o = mei_v_o ;
	mei_system.a_o = mei_a_o ;
	mei_system.workspace = mei_workspace ;
	mei_system.ratio = mei_ratio ;
	mei_system.port = mei_port ;
	mei_system.andmask = mei_andmask ;
	mei_system.ormask = mei_ormask ;
	mei_system.l_port = mei_l_port ;
	mei_system.l_andmask = mei_l_andmask ;
	mei_system.l_ormask = mei_l_ormask ;

	sy_init(hldsp_system, n_axes, map_array) ;
#	endif
	return dsp_error ;
}


int16 FNTYPE get_last_point(VECT x)
{	int16 i ;
	if (! sy_sick(hldsp_system))
	{	for (i = 0; i < hldsp_system->axes; i++)
		{	get_last_command(hldsp_system->axis[i], &(x[i]));
            x[i] /= dspPtr->conversion[hldsp_system->axis[i]].countsPerDistance ;
		}
	}
	return dsp_error ;
}


int16 FNTYPE set_move_output(unsigned16 value)
{	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP);

	if (! sy_port(hldsp_system, 0, (int16)(value & 0xFF), (int16)(value & 0xFF)))
		sy_port(hldsp_system, 1, (int16)(value >> 8), (int16)(value >> 8));
	return dsp_error ;
}

int16 FNTYPE change_move_bit(int16 bitNo, int16 state)
{	if (! hldsp_system)
		return (dsp_error = DSP_NO_MAP);

	return sy_change_bit(hldsp_system, bitNo, state) ;
}

int16 FNTYPE set_move_bit(int16 bitNo)
{	return sy_change_bit(hldsp_system, bitNo, TRUE) ;
}

int16 FNTYPE reset_move_bit(int16 bitNo)
{	return sy_change_bit(hldsp_system, bitNo, FALSE);
}

int16 FNTYPE arc_optimization(int16 opt)
{	return sy_optimize_arcs(hldsp_system, opt) ;
}

int16 FNTYPE set_move_ratio(P_DOUBLE ratio)
{	return sy_set_ratio(hldsp_system, ratio) ;
}

