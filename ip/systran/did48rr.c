/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48rr                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Read_Reg                                      */
/*                                                                          */
/*  Description: This routine is used to read the DID48 registers.         */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            ID of 16 bit register to be read                              */
/*            Ptr to read buffer                                            */
/*  Outputs:  Register value is placed in read buffer                       */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-22-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./did48dr.h"

int DID48_Read_Reg(int path, int reg_id, u_int16 *buf) {
	
	/* local variables */
	int reg_offset;
	
	/* convert reg number to offset */
	reg_offset = reg_id * 2;
	
	/* do the read. if bus error occurred, return with error */
	if(DID48ReadReg16(path, reg_offset, buf) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}
	
	return(0);

}
