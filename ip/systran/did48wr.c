/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48wr                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Write_Reg                                     */
/*                                                                          */
/*  Description: This routine is used to write the DID48 registers.        */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            id of 16 bit register to write                                */
/*            Value to write                                                */
/*  Outputs:  Register is updated with write value                          */
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

int DID48_Write_Reg(int path, int reg_id, u_int16 reg_val) {
	
 	/* local variables */
	int reg_offset;
	
	/* calculate offset of register */
	reg_offset = reg_id * 2;
	
	/* if bus error occurred, return with error */
	if(DID48WriteReg16(path, reg_offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}
	
	return(0);

}
