/*
	system.c - support for 'systems' of axes.
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
#	include <stdarg.h>

# ifdef MEI_OS9000
#	include <stdlib.h>
#	define	MEI_M
#	endif

# ifdef MEI_OS9
#	include <stdlib.h>
#	define	MEI_M
#	endif

# ifdef MEI_VW
#	include <stdlib.h>
#	define	MEI_M
#	endif

# ifdef MEI_VRTXOS
#	include <stdlib.h>
#	define	MEI_M
#	endif

# ifdef MEI_LYNX
#	include <stdlib.h>
#	define MEI_M
#	endif

#ifndef MEI_M
#  ifdef ALLOC
#     include <alloc.h>
#  else
#     include <stdlib.h>
#  endif
#endif


int16 sy_sick(PSYSTEM psystem)
{
	if (! psystem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	if (psystem->sig != PCDSP_SIGNATURE)
		return (dsp_error = DSP_NOT_INITIALIZED);

	psystem->offending_axis = -1;
	return DSP_OK ;
}


int16 FNTYPE sy_init(PSYSTEM system, int16 axes, int16 * ax)
{
	int16 i ;
	double spp, app, cpd ;

	pcdsp_init_check(dspPtr);

	if (system)
	{
		system->sig = 0;		/* initialize sig as invalid */

		for (i = 0; (i < axes) && !dsp_error; i++)
		{	system->pdsp[i] = dspPtr ;
			system->axis[i] = ax[i];
			system->ratio[i] = 1.0 ;
			pcdsp_sick(system->pdsp[i], system->axis[i]) ;
			system->port[i] = system->l_port[i] = 0 ;
			system->andmask[i] = system->l_andmask[i] = -1 ;
			system->ormask[i] = system->l_ormask[i] = 0 ;
		}
		if (dsp_error)
			return dsp_error ;

		system->axes = axes ;
        system->mode = 0;
        system->speed = 0.0;
        system->accel = 0.0;
		system->arc_division = 5.0;
        system->corner_sharpness = 0.0;
        system->filter_length = 0;
		system->move_frame = psystem_move_frame ;
		system->last_frame = psystem_last_frame ;
		system->optimize_arcs = TRUE ;
		system->points = 0;
		system->start_points = 0;
		if (!pcdsp_get_aconversion(system->pdsp[0], ax[0], &cpd, &spp, &app))
			sy_set_period(system, spp, app) ;
		else
			sy_set_period(system, 1, 1);

		system->in_sequence = 0 ;

		if (! dsp_error)
			system->sig = PCDSP_SIGNATURE;
	}

	return dsp_error ;
}



PSYSTEM FNTYPE imk_system(int16 axes, int16 * nn)
{
	PSYSTEM r = (PSYSTEM) malloc(sizeof(SYSTEM)) ;
	int16 double_bytes ;

	if (r)
	{	double_bytes = sizeof(double) * axes ;
		r->pdsp = (PDSP*) malloc(sizeof(PDSP) * axes) ;
		r->axis = (P_INT) malloc(sizeof(int16) * axes) ;
		r->xx_o = (P_DOUBLE) malloc(double_bytes) ;
		r->v_o  = (P_DOUBLE) malloc(double_bytes) ;
		r->a_o  = (P_DOUBLE) malloc(double_bytes) ;
		r->workspace = (P_DOUBLE) malloc(double_bytes * 2) ;
		r->ratio = (P_DOUBLE) malloc(double_bytes) ;
		r->port = (P_INT) malloc(sizeof(int16) * axes) ;
		r->andmask = (P_INT) malloc(sizeof(int16) * axes) ;
		r->ormask = (P_INT) malloc(sizeof(int16) * axes) ;
		r->l_port = (P_INT) malloc(sizeof(int16) * axes) ;
		r->l_andmask = (P_INT) malloc(sizeof(int16) * axes) ;
		r->l_ormask = (P_INT) malloc(sizeof(int16) * axes) ;

		sy_init(r, axes, nn);
	}

	return r;
}


PSYSTEM FNTYPE rm_system(PSYSTEM system)
{
	free(system->pdsp) ;
	free(system->axis);
	free(system->xx_o);
	free(system->v_o);
	free(system->a_o);
	free(system->workspace) ;
	free(system->port);
	free(system->andmask) ;
	free(system->ormask) ;
	free(system->l_port);
	free(system->l_andmask) ;
	free(system->l_ormask) ;
	free(system);
	return NULL;
}


PSYSTEM C_FN mk_system(int16 axes, ...)
{	va_list args ;
	int16 ax[MAX_COORDINATED_AXES], i;

	va_start(args, axes) ;

	for (i = 0; i < axes; i++)
		ax[i] = va_arg(args, int16) ;

	va_end(args) ;
	return imk_system(axes, ax);
}


/*
	internal function.
*/
int16 isy_iterate(PSYSTEM psystem, SYSTEM_ITERATOR system_iterator)
{
	int16 s, r = sy_sick(psystem), x ;
	PDSP pdsp = dspPtr ;

	for (x = 0; !r && (x < psystem->axes); x++)
	{	dspPtr = psystem->pdsp[x] ;
		s = system_iterator(psystem, x);
		if (s > r)
		{	r = s;
			psystem->error = r;
			psystem->offending_axis = x;
		}
	}

	dspPtr = pdsp ;
	return r;
}



int16 FNTYPE isy_clear_status(PSYSTEM psystem, int16 i)
{	return clear_status(psystem->axis[i]);
}

int16 FNTYPE sy_clear_status(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_clear_status) ;
}


int16 FNTYPE isy_set_gate(PSYSTEM psystem, int16 i)
{	return dsp_set_gate(dspPtr, psystem->axis[i]) ;
}
int16 FNTYPE sy_set_gate(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_set_gate) ;
}


int16 FNTYPE isy_idle(PSYSTEM psystem, int16 i)
{	return dsp_idle(dspPtr, psystem->axis[i]);
}
int16 FNTYPE sy_idle(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_idle) ;
}



int16 FNTYPE isy_clear_gate(PSYSTEM psystem, int16 i)
{	return dsp_clear_gate(dspPtr, psystem->axis[i]) ;
}
int16 FNTYPE sy_clear_gate(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_clear_gate) ;
}



int16 FNTYPE isy_set_stop(PSYSTEM psystem, int16 i)
{	return set_stop(psystem->axis[i]) ;
}
int16 FNTYPE sy_set_stop(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_set_stop) ;
}


int16 FNTYPE isy_enable(PSYSTEM psystem, int16 i)
{	int16 enable_level ;
	get_amp_enable_level(psystem->axis[i], &enable_level) ;
	return set_amp_enable(psystem->axis[i], enable_level) ;
}
int16 FNTYPE sy_enable(PSYSTEM psystem)
{	return isy_iterate(psystem, isy_enable) ;
}



int16 FNTYPE asy_state(PSYSTEM psystem, int16 ax)
{
	int16 st = 0 ;
	PDSP pdsp = dspPtr ;

	if (	sy_sick(psystem) ||
			(ax < 0) ||
			(ax > psystem->axes))
		return -1 ;

	dspPtr = psystem->pdsp[ax] ;
	st = axis_state(psystem->axis[ax]) ;

	dspPtr = pdsp ;
	return st;
}

int16 FNTYPE sy_state(PSYSTEM psystem)
{
	int16 x, st = 0, s;

	if (sy_sick(psystem))
		return -1;

	psystem->offending_axis = -1 ;
	for (x = 0; x < psystem->axes; x++)
	{	s = asy_state(psystem, x);
		if (s == -1)							/* this is always the worst error. */
		{	psystem->offending_axis = x ;
			return s ;
		}
		if (s > st)
		{	st = s;
			if (st > 2)
				psystem->offending_axis = x;
		}
	}

	return st;
}


int16 FNTYPE system_done(PSYSTEM psystem)
{
	int16 i ;
	for (i = 0; i < psystem->axes; i++)
		if (! motion_done(psystem->axis[i]))
			return FALSE ;

	return TRUE ;
}


int16 FNTYPE system_gated(PSYSTEM psystem)
{
	int16 i ;

	if (sy_sick(psystem))
		return 0;

	for (i = 0; i < psystem->axes; i++)
	{
		if (! psystem->pdsp[i]->laxis[psystem->axis[i]].gate)
			return FALSE ;
	}
	return TRUE ;
}


int16 FNTYPE sy_get_position(PSYSTEM psystem, P_DOUBLE postn)
{
	int16 i ;
	PDSP pdsp = dspPtr ;
	for (i = 0; i < psystem->axes; i++)
	{	dspPtr = psystem->pdsp[i] ;
		get_position(psystem->axis[i], &(postn[i])) ;
	}

	dspPtr = pdsp ;
	return dsp_error ;
}


int16 FNTYPE sy_move(PSYSTEM psystem, ...)
{
	int16 i, r, e = FALSE ;
	va_list args ;

	va_start(args, psystem) ;

	if (sy_sick(psystem))
	{	va_end(args) ;
		return dsp_error ;
	}

	for (i = 0; i < psystem->axes; i++)
		psystem->workspace[psystem->axes + i] = va_arg(args, double) ;

	va_end(args) ;

	if (! psystem->in_sequence)
	{	e = TRUE ;
		if (sy_start_list(psystem))
			return dsp_error ;
	}
	r = sy_add_point(psystem, &(psystem->workspace[psystem->axes])) ;
	if (e)
	{	sy_end(psystem);
		sy_start(psystem);
	}

	return r;
}


int16 FNTYPE sy_arc(PSYSTEM psystem, double cx, double cy, double angle)
{
	double center[2] ;
	int16 r, e = FALSE ;
	center[0] = cx ;
	center[1] = cy ;
	if (! psystem->in_sequence)
	{	e = TRUE ;
		sy_start_list(psystem);
	}
	r = sy_add_arc(psystem, center, angle, psystem->arc_division);
	if (e)
	{	sy_end(psystem);
		sy_start(psystem);
	}
	return r;
}


int16 FNTYPE sy_port(PSYSTEM psystem, int16 port, int16 andmask, int16 ormask)
{	int16 i, a ;
	if (! sy_sick(psystem))
	{	dsp_port_address(port, &a) ;
		for (i = 0; psystem->port[i] &&
					(psystem->port[i] != a) &&
					(i < psystem->axes);
						i++)
			;

		if (i == psystem->axes)
			dsp_error = DSP_ILLEGAL_IO ;
		else
		{	psystem->port[i] = a ;
			psystem->andmask[i] &= andmask ;
			psystem->ormask[i] |= ormask ;
		}
	}
	return dsp_error ;
}

int16 FNTYPE sy_change_port(PSYSTEM psystem, int16 ioport, int16 value)
{	return sy_port(psystem, ioport, value, value) ;
}


int16 FNTYPE sy_change_bit(PSYSTEM psystem, int16 bit, int16 state)
{
	int16 port = bit / 8,
		mask = 1 << (bit % 8),
		a = state? -1 : ~mask,
		o = state? mask : 0 ;

	return sy_port(psystem, port, a, o);
}


#	ifdef	DEBUG

#	include <stdio.h>

void sy_dump(PSYSTEM psystem)
{
	int16 i ;

	printf("System @ 0x%p\n", psystem) ;
	if (! psystem)
		return ;

	printf("\n%d axes:\n", psystem->axes) ;
	for (i = 0; i < psystem->axes; i++)
		printf("%d: actually %d\n", i, psystem->axis[i]) ;

	printf("\n\n") ;
}


#		endif

int16 FNTYPE sy_in_seq(PSYSTEM psystem)
{	if (! sy_sick(psystem))
		return psystem->in_sequence? TRUE : FALSE ;
	return FALSE ;
}

int16 FNTYPE sy_optimize_arcs(PSYSTEM psystem, int16 opt)
{
	int16 r = sy_sick(psystem) ;

	if (! r)
		psystem->optimize_arcs = opt ;

	return r ;
}

int16 FNTYPE sy_set_period(PSYSTEM psystem, double spp, double app)
{
	double s ;

	if (psystem && psystem->pdsp[0])
	{	s = psystem->pdsp[0]->sample_rate ;
		if (s)
		{	psystem->v_scale = 1.0/(s * spp) ;
			psystem->a_scale = 1.0/(s * s * spp * app);
		}
		else
			dsp_error = DSP_NOT_FOUND ;
	}
	return dsp_error ;
}

int16 FNTYPE sy_set_points(PSYSTEM psystem, int16 points)
{
	if (! sy_sick(psystem))
		psystem->start_points = points ;

	return dsp_error ;
}


int16 FNTYPE sy_set_ratio(PSYSTEM psystem, P_DOUBLE ratio)
{
	int16 i ;

	if (! sy_sick(psystem))
		for (i = 0; (i < psystem->axes); i++)
			psystem->ratio[i] = ratio[i] ;

	return dsp_error ;
}



