/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48wp                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Write_Port                                    */
/*                                                                          */
/*  Description: This routine is used to write the DID48 digital ports.    */
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
#include "./did48dr.h"

int DID48_Write_Port(int path, int port_id, u_int8 reg_val) {
	
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
	if(DID48WriteReg8(path, offset, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}
	
	return(0);

}
