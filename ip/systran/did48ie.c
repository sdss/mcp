/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48ie                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Interrupt_Enable_Control                      */
/*                                                                          */
/*  Description: This routine controls the interrupt enable/disable         */
/*  for the DID48 interrupt capable input bits.                            */
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
#include "did48rg.h"

/* Definitons */

/* Global Variables */

/* Function Prototypes */
#include "did48lb.h"


int DID48_Interrupt_Enable_Control(int path, int port,
	enum DID48_Interrupt_Ena_Dis_Type int_ena_cmd) {
	
	/* Local Variables */
	u_int8 mask;
	unsigned short reg_val;

	switch(int_ena_cmd) {
		
		case DID48_INT_DIS:
			mask = DID48_CSR4_IEN_DIS;
			break;

		case DID48_INT_ENA:
			mask = DID48_CSR4_IEN_ENA;
			break;

		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}
	
	/* if a valid mask was formulated, modify the interrupt enable register */
	/* range test the bit value */
	if((port < 0) || (port > 5)) {
		errno = E_ILLARG;
		return(ERROR);
	}
	/* modify the interrupt enable register */
	if(DID48_Read_Reg(path, DID48_CSR_INT_ENA, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DID48_CSR4_IEN_ENA << port);
	/* OR in the bits corresponding to the selection above */
	reg_val |= (mask << port);
	/* write the register */
	if(DID48_Write_Reg(path, DID48_CSR_INT_ENA, reg_val) == ERROR) {
		return(ERROR);
	}

	return(0);

}
