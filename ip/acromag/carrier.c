#include <stdio.h>
#include "ip480.h"
#include "acromag.h"

#define C_MASK  (unsigned)0xFFFFFC00  /* Forms carrier base address from IP address */


/*
{+D}
    FILENAME:		carrier.c

    MODULE NAME:	carrier.c

    VERSION:		A

    CREATION DATE:      07/25/96

    CODED BY:		FJM

    ABSTRACT:		This file contains the common carrier board data
			structures.

    CALLING SEQUENCE:

    MODULE TYPE:

    I/O RESOURCES:

    SYSTEM RESOURCES:

    MODULES CALLED:

    REVISIONS:

      DATE	BY	PURPOSE
    --------   -----	---------------------------------------------------

{-D}
*/

/*
    MODULES FUNCTION DETAILS

    This file contains the common data structures
*/

/*
    Board Status Register bit positions
*/

#define GLOBAL_EN	8	/* global interrupt enable bit position */

/*
    Defined below is the memory map template for the AVME96x0 Board.
    This structure provides access to the various registers on the board.
*/

struct map96x0			/* Memory map of the board */
{
    BYTE ip_a_io[128];		/* IPA 128 BYTEs I/O space */
    struct ip_a_id		/* IPA Module ID PROM */
    {
	char unused1;		/* undefined */
	char prom_a;		/* IPA id information */
    } id_map_a[32];

    BYTE unused2;		/* undefined */
    BYTE sts_reg;		/* Status Register */

    BYTE unused3;		/* undefined */
    BYTE lev_reg;		/* Interrupt Level Register */

    BYTE unused4;		/* undefined */
    BYTE err_reg;		/* Error Register */

    BYTE unused5;		/* undefined */
    BYTE mem_en_reg;		/* Memory Enable Register */

    BYTE unused6[9];		/* undefined */
    BYTE ipambasr;		/* IPA memory base addr & size register */

    BYTE unused7;		/* undefined */
    BYTE ipbmbasr;		/* IPB memory base addr & size register */

    BYTE unused8;		/* undefined */
    BYTE ipcmbasr;		/* IPC memory base addr & size register */

    BYTE unused9;		/* undefined */
    BYTE ipdmbasr;		/* IPD memory base addr & size register */

    BYTE unused10[9];		/* undefined */
    BYTE en_reg;		/* Interrupt Enable Register */

    BYTE unused11;		/* undefined */
    BYTE pnd_reg;		/* Interrupt Pending Register */

    BYTE unused12;		/* undefined */
    BYTE clr_reg;		/* Interrupt Clear Register */

    BYTE unused13[26];		/* undefined */

    BYTE ip_b_io[128];		/* IPB 128 BYTEs I/O space */
    struct ip_b_id		/* IPB Module ID PROM */
    {
	char unused14;		/* undefined */
	char prom_b;		/* IPB id information */
    } id_map_b[32];

    BYTE unused15[64];		/* undefined */

    BYTE ip_c_io[128];		/* IPC 128 BYTEs I/O space */
    struct ip_c_id		/* IPC Module ID PROM */
    {
	char unused16;		/* undefined */
	char prom_c;		/* IPC id information */
    } id_map_c[32];

    BYTE unused17[64];		/* undefined */

    BYTE ip_d_io[128];		/* IPD 128 BYTEs I/O space */
    struct ip_d_id		/* IPD Module ID PROM */
    {
	char unused18;		/* undefined */
	char prom_d;		/* IPD id information */
    } id_map_d[32];

    BYTE unused19[64];		/* undefined */
} ;

void carrier(struct map96x0 *map_ptr);


void carrier(map_ptr)

struct map96x0 *map_ptr;    /* pointer to board memory map */

{

/*
    ENTRY POINT OF ROUTINE:
*/

    map_ptr->sts_reg |= GLOBAL_EN;
}



/*
{+D}
   SYSTEM:	    Acromag Input Board

   MODULE NAME:     isrX.c - interrupt handler

   VERSION:	    A

   CREATION DATE:  07/25/96

   CODED BY:	    FM

   ABSTRACT:	    These routines perform interrupt exception handling
                    for interrupts on the IP480 Input board.

   CALLING
	SEQUENCE:   This subroutine runs as a result of an interrupt occuring.
		    When called, OS-9 provides in 680XX register a2 a pointer
		    to the interrupt handler data area of the associated
		    parent process.

   MODULE TYPE:    n/a

   I/O RESOURCES:  user specific

   SYSTEM
	RESOURCES:

   MODULES
	CALLED:     user specific

   REVISIONS:

 DATE	   BY	    PURPOSE
-------- ----  ------------------------------------------------

{-D}
*/

/*
   MODULES FUNCTIONAL DETAILS:
*/


void isrX(cblk)

struct conf_blk *cblk;		/* pointer to config block */

{

/*
	External data areas
*/

/*
	Local data areas
*/
int i,j;
UWORD i_stat;
/*
    DECLARE MODULES CALLED:
*/
#if !defined(ACROMAG_H)
   UWORD inpw();
   UWORD outpw();
#endif

/*
    DECLARE LOCAL DATA AREAS
*/

/*
	Entry point of routine:
*/



  i_stat = inpw((UWORD *)(cblk->brd_ptr + InterruptPending));
  if(cblk->num_chan == 2)                  /* check if it's a 2 or 6 channel board */
   {
    i_stat &= 0x0300;                      /* and off the unused upper bits */
    j = 2;
   }
  else
   {
    i_stat &= 0x3F00;	      		/* and off the unused bits and save the useful ones */
    j = 6;
   }


  if( i_stat )								/* any interrupt(s) pending? */
  {
           cblk->event_status |= (BYTE)(i_stat >> 8 );         /* update event status */

	/* service the hardware */
	/* check each bit for an interrupt pending state */

    for( i = 0; i < j; i++ )					/* check each counter timer */
    {
	  if( i_stat & (1 << (i + 8)) )				/* build interrupt clear word */
		  i_stat &= (~(1 << (i + 8)));			/* clear interrupting bit */
	  else
		  i_stat |= (1 << (i + 8));				/* set bit to ignore */
	}
   outpw((UWORD *)(cblk->brd_ptr + InterruptPending), i_stat);       /* write interrupt pending */
  }

}



/*
{+D}
	SYSTEM: 	IP480 Software

	MODULE NAME:	outpw

	VERSION:	V1.0

	CREATION DATE:  01-22-97

	DESIGNED BY:	RH

	CODED BY:	RH

	ABSTRACT:	Low level interface to write word routine.

	CALLING
	  SEQUENCE:	BYTE = m_outpw(addr,b);
			Where
				addr (pointer*) to unsigned I/O port address
                                b    (UWORD) data to write
	MODULE TYPE:

	I/O
	  RESOURCES:	None

	SYSTEM
	  RESOURCES:	None

	MODULES
	  CALLED:

	REVISIONS:

  DATE	    BY	   PURPOSE
---------  ----   -------------------------------------------------------

{-D}
*/

/*
   MODULES FUNCTIONAL DETAILS:
*/


UWORD outpw(addr,b)

UWORD *addr;
UWORD b;

{

/*
	DECLARE MODULES CALLED:
*/

/*
	Entry point of routine
*/

  *((UWORD *)addr) = (UWORD)b;
  return(0);
}


/*
{+D}
	SYSTEM: 	IP480 Software

	MODULE NAME:	inpw

	VERSION:	V1.0

	CREATION DATE:  01-22-97

	DESIGNED BY:	RH

	CODED BY:	RH

	ABSTRACT:	Low level interface to read word routine.

	CALLING
	  SEQUENCE:	return = input(addr);
			Where
				return (BYTE) character read
				addr (pointer*) to unsigned I/O port address
	MODULE TYPE:

	I/O
	  RESOURCES:	None

	SYSTEM
	  RESOURCES:	None

	MODULES
	  CALLED:

	REVISIONS:

  DATE	    BY	   PURPOSE
---------  ----   -------------------------------------------------------

{-D}
*/

/*
   MODULES FUNCTIONAL DETAILS:
*/


UWORD inpw(addr)

UWORD *addr;

{

/*
	DECLARE MODULES CALLED:
*/

/*
	Entry point of routine
*/

  return((UWORD) *((UWORD *)addr));
}
