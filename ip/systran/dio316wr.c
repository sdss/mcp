/****************************************************************************/
/*                                                                          */
/*   DDDDD      IIIIIIIII      OOOO       333333       1         66666      */
/*   D    D         I         O    O     3      3     11        6     6     */
/*   D     D        I        O      O           3      1       6            */
/*   D      D       I        O      O           3      1       6            */
/*   D      D       I        O      O         33       1       6            */
/*   D      D       I        O      O           3      1       6  6666      */
/*   D      D       I        O      O           3      1       6 6    6     */
/*   D     D        I        O      O           3      1       6       6    */
/*   D    D         I         O    O     3      3      1        6     6     */
/*   DDDDD      IIIIIIIII      OOOO       333333     11111       66666      */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                     This software was developed by:                      */
/*                                                                          */
/*                              SYSTRAN Corp.                               */
/*                            4126 Linden Ave.                              */
/*                         Dayton, Ohio 45432-3066                          */
/*                             (513) 252-5601                               */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  Module Name: dio316wr                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Write_Reg                                     */
/*                                                                          */
/*  Description: This routine is used to write the DIO316 registers.        */
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
#include "./dio316dr.h"

int DIO316_Write_Reg(int path, int reg_id, u_int16 reg_val) {
	
 	/* local variables */
	int reg_offset;
	
	/* calculate offset of register */
	reg_offset = reg_id * 2;
	
	/* if bus error occurred, return with error */
	if(DIO316WriteReg16(path, reg_offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}
	
	return(0);

}
