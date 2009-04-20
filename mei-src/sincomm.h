/*
	SINCOMM.H
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#ifndef __SINCOMM_H
#define __SIMCOMM_H

#include <math.h>
#include "idsp.h"

#define	COMM_BASE				(unsigned16) 0x0800
#define	COMM_OFFSET_POS			(COMM_BASE + 0)
#define	COMM_OFFSET_NEG			(COMM_BASE + 2)           
#define	COMM_TABLE_LEN			(COMM_BASE + 4)           
#define	COMM_POINTER			(COMM_BASE + 5)           
#define	COMM_SHIFT				(COMM_BASE + 6)           
#define	COMM_OLD_THETA			(COMM_BASE + 7)           
#define	COMM_DIAG				(COMM_BASE + 8)           
#define	COMM_SIN_TABLE			(COMM_BASE + 16)
#define	COMM_SIN_TABLE_SIZE		(unsigned16) 16
#define	COMM_SIN_SIZE			(unsigned16) 1024
#define	COMM_BLOCK_SIZE			(COMM_SIN_SIZE + COMM_SIN_TABLE_SIZE)
#define	CBS(a)					(COMM_BLOCK_SIZE * (a))

#ifndef  MEI_OS9
	#define	PI					3.1415926
#endif  /* MEI_OS9 */

#define	PHASES					(unsigned16) 3

#ifdef __cplusplus
	extern "C" {
#endif

int16 FNTYPE set_phases(int16 axis, int16 ang, int16 lead, int16 table_size, int16 phase_seq);
int16 FNTYPE wait_and_clear(int16 axis, int16 duration);
int16 FNTYPE init_scheme_one(int16 axis, int16 phase_seq, int16 table_size, int16 volts);
int16 FNTYPE get_actual_accel(int16 axis, int16 samples, double * accel);
int16 FNTYPE init_scheme_two(int16 axis, int16 phase_seq, int16 table_size, int16 volts, int16 samples);
int16 FNTYPE read_hall_sensors(void);
int16 FNTYPE get_ang(int16 phase_seq, int16 table_size, int16 region);
int16 FNTYPE init_scheme_three(int16 axis, int16 phase_seq, int16 table_size, int16 volts);
int16 FNTYPE init_comm_table(int16 axis, long enc_res, long elec_cycles, int16 ncycles, P_INT cycle_table_size);
int16 FNTYPE init_comm(int16 axis, int16 phase_seq, int16 method, int16 cycle_table_size, double voltage);
int16 FNTYPE set_comm_axis(int16 axis, int16 enable);
int16 FNTYPE get_comm_axis(int16 axis, P_INT enable);
int16 FNTYPE set_boot_comm_axis(int16 axis, int16 enable);
int16 FNTYPE get_boot_comm_axis(int16 axis, P_INT enable);

#ifdef __cplusplus
	} ;
#endif

#endif  /* __SINCOMM_H */
