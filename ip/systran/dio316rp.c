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
/*  Module Name: dio316rp                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Read_Port                                     */
/*                                                                          */
/*  Description: This routine is used to read the DIO316 ports.             */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            id of 8 bit port to be read                                   */
/*            Ptr to read buffer                                            */
/*  Outputs:  port value is placed in read buffer                           */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-22-94    Mark Rowe       Original Release                             */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./dio316dr.h"

int DIO316_Read_Port(int path, int port_id, u_int8 *buf) {
	
	/* local variables */
	int offset;
	u_int16 buf16;

	switch(port_id) {
		case 0:
		case 1:
			offset = 0;
			break;
		case 2:
		case 3:
			offset = 2;
			break;
		case 4:
		case 5:
			offset = 4;
			break;
		default:
			errno = E_ILLARG;
			return(ERROR);
	}

	/* do the read, if bus error occurred return with error */
	if(DIO316ReadReg16(path, offset, &buf16)) {
		errno = E_BUSERR;
		return(ERROR);
	}

	/* return the appropriate byte in buf */
	switch(port_id) {
		case 0:
		case 2:
		case 4:
			*buf = buf16 & 0xff;
			break;
		case 1:
		case 3:
		case 5:
			*buf = (buf16 >> 8) & 0xff;
			break;
		default:
			errno = E_ILLARG;
			return(ERROR);
	}
	
	return(0);

}
