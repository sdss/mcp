/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48ic                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Interrupt_Configuration                       */
/*                                                                          */
/*  Description: This routine controls the interrupt input configuration    */
/*  for the DID48 interrupt capable input bits.                            */
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
#include "did48rg.h"

/* Definitons */

/* Global Variables */

/* Function Prototypes */
#include "did48lb.h"


int DID48_Configuration(int path, int port, enum DID48_Config_Cmd_Type cmd) {
		
	if(DID48_Write_Reg(path, DID48_CSR_CTL, cmd<<port) == ERROR) {
		return(ERROR);
	}

	return(0);

}
