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
/*  Module Name: dio316ie                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Interrupt_Enable_Control                      */
/*                                                                          */
/*  Description: This routine controls the interrupt enable/disable         */
/*  for the DIO316 interrupt capable input bits.                            */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            bit number to enable/disable                                  */
/*            Interrupt enable command                                      */
/*  Outputs:  Control regs are set to achieve desired interrupt             */
/*            enable/disable                                                */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-27-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <errno.h>
#include "gendefs.h"
#include "dio316rg.h"

/* Definitons */

/* Global Variables */

/* Function Prototypes */
#include "dio316lb.h"


int DIO316_Interrupt_Enable_Control(int path, int bit_num,
	enum DIO316_Interrupt_Ena_Dis_Type int_ena_cmd) {
	
	/* Local Variables */
	u_int16 mask, reg_val;

	switch(int_ena_cmd) {
		
		case DIO316_INT_DIS:
			mask = DIO316_CSR5_IEN_DIS;
			break;

		case DIO316_INT_ENA:
			mask = DIO316_CSR5_IEN_ENA;
			break;

		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}
	
	/* if a valid mask was formulated, modify the interrupt enable register */
	/* range test the bit value */
	if((bit_num < 0) || (bit_num > 3)) {
		errno = E_ILLARG;
		return(ERROR);
	}
	/* modify the interrupt enable register */
	if(DIO316_Read_Reg(path, DIO316_CSR_INT_ENA, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR5_IEN_ENA << bit_num);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask << bit_num;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_INT_ENA, reg_val) == ERROR) {
		return(ERROR);
	}

	return(0);

}
