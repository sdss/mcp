/*
	mldebug.c - contains routines for debugging your programs.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include <stdio.h>
#include "idsp.h"

/*
	Portability:  Note that this file is not a
	required portion of the library.  It is included
	for debugging of more complex programs...  and
	contains lots of code for viewing the PC-DSP's 
	current state.  If you are compiling under a different
	system, leaving this file out will have no adverse
	effects (except, of course, loss of it's functionality).
*/


static char PTRTYPE * frame_control_bit[16] = {
	"[unused 0]", "[unused 1]", "[unused 2]", "[unused 3]",
	"[unused 4]", "[unused 5]", "[unused 6]", "[unused 7]",
	"[unused 8]", "[unused 9]", "[unused 10]", "[unused 11]",
	"[unused 12]", "[unused 13]", "Interrupt PC (x4000)", "Release (x8000)" } ;

static char PTRTYPE * trig_update_bit[16] = {
	"Upd Jerk (x01)", "Upd Accel (x02)", "Upd Vel (x04)", "Upd Pos (x08)",
	"Upd Output (0x10)", "[unused (x20)]", "[unused (x40)]", "[unused (x80)]",
	"[unused (x100)]", "[unused (x200)]", "[unused (x400)]", "[unused (0x800)]",
	"[unused (x1000)]", "[unused (x2000)]", "[unused (x4000)]", "[unused (x8000)]" } ;


static int16 LOCAL_FN describe_bits(unsigned16 ctrl, char PTRTYPE * labels[])
{
	int16 bit, argh = 1 ;
	
	if (ctrl)
	{
		printf("               (") ;
		for (bit = 0; bit < 16; bit++)
		{	if (ctrl & argh)
			{	ctrl &= ~argh ;
				printf("%s%s", labels[bit], ctrl? ", " : ")\n");
			}
			argh <<= 1;
		}
	}
	return 0;
}

#define SAMPLE_RATE (1250.0)

int16 FNTYPE pcdsp_dump_frame(PFRAME pframe)
{
	LFIXED fixed ;
	double argh ;

	printf("\nDump of the frame at 0x%p\n", pframe);
	printf(  " Allocated to DSP at 0x%p", pframe->dsp);
	if (pframe->dsp)  printf(" (I/O base 0x%4.4X)", pframe->dsp->address) ;
	printf("\n           Axis: %d\n", pframe->axis);
	printf("   Allocated at: 0x%4.4X\n", pframe->current);
	printf("\n");
	printf("     Next frame: 0x%4.4X\n", pframe->f.next);
	printf("        Control: 0x%4.4X\n", pframe->f.control) ;
	describe_bits(pframe->f.control, frame_control_bit) ;

	argh = ((double) pframe->f.time) / SAMPLE_RATE ;
	printf("           Time: %f seconds (%lu samples).\n", argh, pframe->f.time);

	copy_lfixed(fixed, pframe->f.jerk) ;
	argh = ipcdsp_double(&fixed) ;
	argh *= (SAMPLE_RATE * SAMPLE_RATE * SAMPLE_RATE) ;
	printf("           Jerk: %f cts/(sec^3) (0x%4.4X.%lX)\n", argh, pframe->f.jerk.whole, pframe->f.jerk.frac);

	copy_lfixed(fixed, pframe->f.accel) ;
	argh = ipcdsp_double(&fixed) ;
	argh *= (SAMPLE_RATE * SAMPLE_RATE) ;
	printf("          Accel: %f cts/(sec^2) (0x%4.4X.%lX)\n", argh, pframe->f.accel.whole, pframe->f.accel.frac) ;

	copy_lfixed(fixed, pframe->f.velocity);
	argh = ipcdsp_double(&fixed) ;
	argh *= SAMPLE_RATE ;
	printf("       Velocity: %f cts/sec (0x%4.4X.%lX)\n", argh, pframe->f.velocity.whole, pframe->f.velocity.frac );

	argh = ipcdsp_double(&(pframe->f.position)) ;
	printf("       Position: %f cts (0x%8.8lX.%lX)\n", argh, pframe->f.position.whole, pframe->f.position.frac) ;

	printf("    Trig/Update: 0x%4.4X\n", pframe->f.trig_update) ;
	describe_bits(pframe->f.trig_update, trig_update_bit) ;
	printf("    Trig Action: 0x%4.4X\n", pframe->f.trig_action);
	printf("         Output: 0x%4.4X\n", pframe->f.output);

	printf("\n\n");

	return 0;
}
