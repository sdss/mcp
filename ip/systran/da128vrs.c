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
/*  Module Name: da128vrs                                                   */
/*                                                                          */
/*  Procedure Name(s): DAC128V_Read_Setting                                 */
/*                                                                          */
/*  Description: This routine is used to read the voltage setting of the    */
/*  specified DAC128V channel.                                              */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            Channel                                                       */
/*            Ptr to volts setting read buffer                              */
/*  Outputs:  Voltage corresponding to register value is placed in          */
/*            read buffer                                                   */
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
#include "./da128vdr.h"

int DAC128V_Read_Setting(int path, int channel, float *volts_buf) {

	/* local variables */
	u_int16 reg_val;

	/* read the specified register. if bus error occurred, return with error */
	if(DAC128V_Read_Reg(path, channel, &reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	/* convert hex register value to volts */
	*volts_buf = (float)((reg_val - 2048) * 0.0024415);

	return(0);

}
