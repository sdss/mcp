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
/*  Module Name: dio316wp                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Write_Port                                    */
/*                                                                          */
/*  Description: This routine is used to write the DIO316 digital ports.    */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            id of 8 bit port to write                                     */
/*            Value to write                                                */
/*  Outputs:  port is updated with write value                              */
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

int DIO316_Write_Port(int path, int port_id, u_int8 reg_val) {
	
	/* local variables */
	int offset;

	switch(port_id) {
		case 0:
			offset = 1;
			break;
		case 1:
			offset = 0;
			break;
		case 2:
			offset = 3;
			break;
		case 3:
			offset = 2;
			break;
		case 4:
			offset = 5;
			break;
		case 5:
			offset = 4;
			break;
		default:
			errno = E_ILLARG;
			return(ERROR);
	}

	/* if bus error occurred, return with error */
	if(DIO316WriteReg8(path, offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}
	
	return(0);

}
