/****************************************************************************/
/*                                                                          */
/*       A     DDDD       CCC      1      222      8888    FFFFFFF   1      */
/*      A A    D   D     C   C    11     2   2    8    8   F        11      */
/*     A   A   D    D   C     C    1    2     2  8      8  F         1      */
/*    A     A  D     D  C          1          2   8    8   F         1      */
/*    A     A  D     D  C          1         2     8888    FFFFF     1      */
/*    AAAAAAA  D     D  C          1        2     8    8   F         1      */
/*    A     A  D     D  C          1       2     8      8  F         1      */
/*    A     A  D    D   C     C    1      2      8      8  F         1      */
/*    A     A  D   D     C   C     1     2        8    8   F         1      */
/*    A     A  DDDD       CCC     111   2222222    8888    F        111     */
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
/*  Module Name: ad12f1wr                                                   */
/*                                                                          */
/*  Procedure Name(s): ADC128F1_Write_Reg                                   */
/*                                                                          */
/*  Description: This routine is used to write the ADC128F1 registers.      */
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
/*  11-3-94    Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./ad12f1dr.h"

int ADC128F1_Write_Reg(int path, int reg_id, u_int16 reg_val) {
	
 	/* local variables */
	int reg_offset;
	
	/* calculate offset of register */
	reg_offset = reg_id * 2;
	
	/* if bus error occurred, return with error */
	if(ADC128F1WriteReg16(path, reg_offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	return(0);

}
