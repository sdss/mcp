/*
	systems.h
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#ifndef __SYSTEM_H
#	define	__SYSTEM_H

#	include "pcdsp.h"

#ifndef	__IDSP_H
typedef struct __DSP DSP ;
typedef DSP * PDSP ;
#	endif

#	define	MAX_COORDINATED_AXES				8

typedef struct _SYSTEM PTRTYPE * PSYSTEM ;

typedef int16 (FNTYPE * MOVE_FRAME) (PSYSTEM, int16, double, unsigned long, int16) ;
typedef int16 (FNTYPE * LAST_FRAME) (PSYSTEM, int16) ;

typedef struct _SYSTEM
{	int16		sig ;
    int16     mode;
    int16     filter_length;
	int16		axes ;
	P_INT	axis ;
	PDSP *	pdsp ;
	int16		error, offending_axis ;
	P_DOUBLE	xx_o, v_o, a_o, workspace, ratio ;
	double	speed, accel, arc_division,corner_sharpness ;
	double	a_scale, v_scale ;
	long	t, t2 ;	
	int16		hold, in_sequence ;
	MOVE_FRAME	move_frame ;
	LAST_FRAME	last_frame ;
	int16		optimize_arcs ;
	P_INT	port, andmask, ormask, l_port, l_andmask, l_ormask ;
	int16		points, start_points;
} SYSTEM ;

typedef int16 (FNTYPE * SYSTEM_ITERATOR)(PSYSTEM, int16) ;

int16 sy_sick(PSYSTEM psystem) ;

PSYSTEM C_FN mk_system(int16 axes, ...) ;
PSYSTEM FNTYPE imk_system(int16 axes, int16 * nn) ;
PSYSTEM FNTYPE rm_system(PSYSTEM system) ;
int16 FNTYPE sy_init(PSYSTEM system, int16 axes, int16 * ax);
int16 FNTYPE sy_clear_status(PSYSTEM psystem) ;
int16 FNTYPE sy_set_gate(PSYSTEM psystem) ;
int16 FNTYPE sy_clear_gate(PSYSTEM psystem) ;
int16 FNTYPE sy_set_stop(PSYSTEM psystem) ;
int16 FNTYPE system_done(PSYSTEM psystem) ;
int16 FNTYPE system_gated(PSYSTEM psystem);
int16 FNTYPE sy_get_position(PSYSTEM psystem, P_DOUBLE postn) ;
int16 FNTYPE sy_enable(PSYSTEM psystem) ;
int16 FNTYPE sy_state(PSYSTEM psystem) ;
int16 FNTYPE asy_state(PSYSTEM psystem, int16 ax) ;
int16 FNTYPE sy_in_seq(PSYSTEM psystem);
int16 FNTYPE sy_idle(PSYSTEM psystem) ;
int16 FNTYPE sy_optimize_arcs(PSYSTEM psystem, int16 opt) ;
int16 FNTYPE sy_set_period(PSYSTEM psystem, double spp, double app) ;
int16 FNTYPE sy_set_points(PSYSTEM psystem, int16 points) ;



#	ifdef DEBUG
void sy_dump(PSYSTEM psystem) ;
#		endif


int16 FNTYPE sy_start_list(PSYSTEM psystem) ;
int16 FNTYPE sy_add_point(PSYSTEM psystem, VECT point) ;
int16 FNTYPE sy_add_arc(PSYSTEM psystem, VECT center, double angle, double division);
int16 FNTYPE sy_end(PSYSTEM psystem) ;
int16 FNTYPE sy_start(PSYSTEM psystem) ;
int16 FNTYPE sy_stop(PSYSTEM psystem) ;
int16 FNTYPE sy_set_speed(PSYSTEM psystem, double speed) ;
int16 FNTYPE sy_set_accel(PSYSTEM psystem, double accel) ;
int16 FNTYPE sy_done(PSYSTEM psystem) ;
int16 FNTYPE sy_arc_division(PSYSTEM psystem, double degrees) ;
int16 FNTYPE psystem_move_frame(PSYSTEM psystem, int16 i, double a, unsigned long t, int16 a_frame) ;
int16 FNTYPE psystem_last_frame(PSYSTEM psystem, int16 i) ;

int16 FNTYPE sy_change_bit(PSYSTEM psystem, int16 bit, int16 state) ;
int16 FNTYPE sy_change_port(PSYSTEM psystem, int16 ioport, int16 value) ;
int16 FNTYPE sy_port(PSYSTEM psystem, int16 port, int16 andmask, int16 ormask) ;

int16 FNTYPE sy_move(PSYSTEM psystem, ...) ;
int16 FNTYPE sy_arc(PSYSTEM psystem, double cx, double cy, double deg) ;

int16 FNTYPE sy_set_mode(PSYSTEM psystem, int16 mode);
int16 FNTYPE sy_sharpness(PSYSTEM psystem, double sharpness);
int16 FNTYPE sy_set_filter(PSYSTEM psystem, int16 length);
int16 FNTYPE sy_set_ratio(PSYSTEM psystem, P_DOUBLE ratio);

#	endif
