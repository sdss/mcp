/****************************************************************************/
/*                                                                          */
/*   DDDD         A        CCC      1      222       8888      V         V  */
/*   D   D       A A      C   C    11     2   2     8    8     V         V  */
/*   D    D     A   A    C     C    1    2     2   8      8     V       V   */
/*   D     D   A     A   C          1          2    8    8      V       V   */
/*   D     D   A     A   C          1         2      8888        V     V    */
/*   D     D   AAAAAAA   C          1        2      8    8       V     V    */
/*   D     D   A     A   C          1       2      8      8       V   V     */
/*   D    D    A     A   C     C    1      2       8      8        V V      */
/*   D   D     A     A    C   C     1     2         8    8         V V      */
/*   DDDD      A     A     CCC     111   2222222     8888           V       */
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
/*  Module Name: da128vwr                                                   */
/*                                                                          */
/*  Procedure Name(s): DAC128V_Write_Reg                                    */
/*                                                                          */
/*  Description: This routine is used to write the DAC128V registers.       */
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
/*  10-18-94  Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./da128vdr.h"

int DAC128V_Write_Reg(int path, int reg_id, u_int16 reg_val) {
	
 	/* local variables */
	int reg_offset;
	
	/* calculate offset of register */
	reg_offset = reg_id * 2;
	
	/* if bus error occurred, return with error */
	if(DAC128VWriteReg16(path, reg_offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	return(0);

}
