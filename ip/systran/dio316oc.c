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
/*  Module Name: dio316oc                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_OE_Control                                    */
/*                                                                          */
/*  Description: This routine controls the output enable mode for DIO316    */
/*  digital output ports.                                                   */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            ID of port to control                                         */
/*            Output enable control command                                 */
/*  Outputs:  Control reg is set to achieve desired OE control              */
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


int DIO316_OE_Control(int path, int port, enum DIO316_OE_Cmd_Type OE_cmd) {
	
	/* Local Variables */
	u_int16 mask, reg_val;
	
	switch(OE_cmd) {
		
		case DIO316_OE_ENA:
			mask = DIO316_CSR3_OE_ENA;
			break;

		case DIO316_OE_DIS:
			mask = DIO316_CSR3_OE_DIS;
			break;

		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}
	
	/* if a valid mask was formulated, modify the control register */

	/* range test the port value */
	if((port < 0) || (port > 5)) {
		errno = E_ILLARG;
		return(ERROR);
	}
	/* first, AND out OE control bit */
	if(DIO316_Read_Reg(path, DIO316_CSR_OE_CTL, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR3_OE_ENA << port);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask << port;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_OE_CTL, reg_val) == ERROR) {
		return(ERROR);
	}

	return(0);

}
