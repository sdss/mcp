#include <stdio.h>
#include "ip480.h"

/*
{+D}
    SYSTEM:         Library Software - ip480 Board

    FILENAME:       rmid480.c

    MODULE NAME:    rmid480 - read I.D. of ip480 board

    VERSION:        A

    CREATION DATE:  07/08/96

    DESIGNED BY:    F.J.M

    CODED BY:       F.J.M.
    
    ABSTRACT:       This module is used to read the I.D. of the ip480 board.

    CALLING
        SEQUENCE:   rmid480(ptr);
                    where:
                        ptr (pointer to structure)
                            Pointer to the configuration block structure.

    MODULE TYPE:    void

    I/O RESOURCES:  

    SYSTEM
        RESOURCES:  

    MODULES
        CALLED:     

    REVISIONS:      

  DATE    BY        PURPOSE
-------  ----   ------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:

    This module is used to perform the read status function
    for the ip480 board.  A pointer to the Configuration Block will
    be passed to this routine.  The routine will use a pointer
    within the Configuration Block together with offsets
    to reference the registers on the Board and will transfer the 
    status information from the Board to the Configuration Block.
*/



void rmid480(c_blk)
struct conf_blk *c_blk;
{

/*
    declare local storage
*/

    BYTE *id_ptr;               /* pointer to board ID memory map */
    int i,j;                    /* loop index */

/*
    ENTRY POINT OF ROUTINE:
    Initialize local storage.  Initialize pointer to board memory map.
*/

    id_ptr = (BYTE *)c_blk->brd_ptr + 0x80;

/*
    read board information
*/

    for(i = 0, j = 1; i < 32; i++, j += 2)
        c_blk->id_prom[i] = id_ptr[j];
}
