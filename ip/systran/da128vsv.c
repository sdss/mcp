/****************************************************************************/
/*                                                                          */
/*   DDDD         A        CCC      1      222       8888      V         V  */
/*   D   D       A A      C   C    11     2   2     8    8     V         V  */
/*   D    D     A   A    C     C    1    2     2   8      8     V       V   */
/*   D     D   A     A   C          1          2    8    8      V       V   */
/*   D     D   A     A   C          1         2      8888        V     V    */
/*   D     D   AAAAAAA   C          1        2      8    8       V     V    */
/*   D     D   A     A   C          1       2      8      8       V   V     */
/*   D    D    A     A   C     C    1      2       8      8        V V      */
/*   D   D     A     A    C   C     1     2         8    8         V V      */
/*   DDDD      A     A     CCC     111   2222222     8888           V       */
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
/*  Module Name: da128vsv                                                   */
/*                                                                          */
/*  Procedure Name(s): DAC128V_Set_Volts                                    */
/*                                                                          */
/*  Description: This routine is used to set the specified DAC128V          */
/*  channel to the specified voltage.                                       */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            Channel                                                       */
/*            Volts value                                                   */
/*  Outputs:  Specified register is set to value corresponding to voltage   */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  10-18-94  Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./da128vlb.h"

int DAC128V_Set_Volts(int path, int channel, float volts) {

	/* local variables */
	u_int16 reg_val;

	/* range limit the specified volts */
	if(volts > 5.0) {
		volts = 5.0;
	}
	else if(volts < -5.0) {
		volts = -5.0;
	}

	/* convert volts to hex register value */
	reg_val = (u_int16)((volts/0.0024415) + 2048);
        printf ("\r\n Set Volts = %x, %f, %f",reg_val,volts,volts/0.0024415);

	/* do the write. if bus error occurred, return with error */
	if(DAC128V_Write_Reg(path, channel - 1, reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	return(0);

}
