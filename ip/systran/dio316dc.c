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
/*  Module Name: dio316dc                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Port_Direction_Control                        */
/*                                                                          */
/*  Description: This routine controls the port direction for the           */
/*  DIO316 bi-directional ports.                                            */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            ID of port to control                                         */
/*            Port direction control command                                */
/*  Outputs:  Direction control reg is set to achieve desired port          */
/*            direction                                                     */
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


int DIO316_Port_Direction_Control(int path, int port,
	enum DIO316_Direction_Cmd_Type direction_cmd) {
	
	/* Local Variables */
	u_int16 mask, reg_val;
	
	switch(direction_cmd) {
		
		case DIO316_DIR_IN:
			mask = DIO316_CSR4_DIR_IN;
			break;

		case DIO316_DIR_OUT:
			mask = DIO316_CSR4_DIR_OUT;
			break;

		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}
	
	/* if a valid mask was formulated, modify the control register */
	/* range test the port value */
	if((port < 0) || (port > 1)) {
		errno = E_ILLARG;
		return(ERROR);
	}
	/* first, AND out DIR control bit */
	if(DIO316_Read_Reg(path, DIO316_CSR_DIR_CTL, &reg_val) == ERROR) {
		return(ERROR);
	}
	reg_val &= ~(DIO316_CSR4_DIR_OUT << port);
	/* OR in the bits corresponding to the selection above */
	reg_val |= mask << port;
	/* write the register */
	if(DIO316_Write_Reg(path, DIO316_CSR_DIR_CTL, reg_val) == ERROR) {
		return(ERROR);
	}

	return(0);

}
