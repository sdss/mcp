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
/*  Module Name: dio316ic                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Interrupt_Configuration                       */
/*                                                                          */
/*  Description: This routine controls the interrupt input configuration    */
/*  for the DIO316 interrupt capable input bits.                            */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            bit number to configure                                       */
/*            Interrupt configuration command                               */
/*  Outputs:  Control regs are set to achieve desired interrupt             */
/*            configuration                                                 */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-24-94   Mark Rowe       Original Release                              */
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


int DIO316_Interrupt_Configuration(int path, int bit_num,
	enum DIO316_Interrupt_Config_Cmd_Type int_config_cmd) {
	
	/* Local Variables */
	u_int16 mask_el, mask_hl, mask_oc, reg_val;
	
	switch(int_config_cmd) {
		
		case DIO316_INT_LOW_LVL:
			mask_el = DIO316_CSR7_IEL_LVL;
			mask_hl = DIO316_CSR8_IHL_LOW;
			mask_oc = DIO316_CSR9_IOC_DIS;
			break;

		case DIO316_INT_HIGH_LVL:
			mask_el = DIO316_CSR7_IEL_LVL;
			mask_hl = DIO316_CSR8_IHL_HIGH;
			mask_oc = DIO316_CSR9_IOC_DIS;
			break;

		case DIO316_INT_FALL_EDGE:
			mask_el = DIO316_CSR7_IEL_EDG;
			mask_hl = DIO316_CSR8_IHL_LOW;
			mask_oc = DIO316_CSR9_IOC_DIS;
			break;

		case DIO316_INT_RISE_EDGE:
			mask_el = DIO316_CSR7_IEL_EDG;
			mask_hl = DIO316_CSR8_IHL_HIGH;
			mask_oc = DIO316_CSR9_IOC_DIS;
			break;

		case DIO316_INT_ON_CHANGE:
			mask_el = DIO316_CSR7_IEL_LVL;
			mask_hl = DIO316_CSR8_IHL_LOW;
			mask_oc = DIO316_CSR9_IOC_ENA;
			break;

		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}
	
	/* if a valid mask was formulated, modify the interrupt config registers */
	/* range test the bit value */
	if((bit_num < 0) || (bit_num > 3)) {
		errno = E_ILLARG;
		return(ERROR);
	}
	/* we must modify three registers to set the int config */
		
	/* first, modify the EDGE-LEVEL register */
	if(DIO316_Read_Reg(path, DIO316_CSR_INT_EL, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR7_IEL_EDG << bit_num);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask_el << bit_num;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_INT_EL, reg_val) == ERROR) {
		return(ERROR);
	}

	/* next, modify the HIGH-LOW register */
	if(DIO316_Read_Reg(path, DIO316_CSR_INT_HL, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR8_IHL_HIGH << bit_num);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask_hl << bit_num;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_INT_HL, reg_val) == ERROR) {
		return(ERROR);
	}

	/* finally, modify the ON CHANGE register */
	if(DIO316_Read_Reg(path, DIO316_CSR_INT_OC, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR9_IOC_ENA << bit_num);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask_oc << bit_num;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_INT_OC, reg_val) == ERROR) {
		return(ERROR);
	}

	return(0);

}
