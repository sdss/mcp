/****************************************************************************/
/*                                                                          */
/*       A     DDDD       CCC      1      222      8888    FFFFFFF   1      */
/*      A A    D   D     C   C    11     2   2    8    8   F        11      */
/*     A   A   D    D   C     C    1    2     2  8      8  F         1      */
/*    A     A  D     D  C          1          2   8    8   F         1      */
/*    A     A  D     D  C          1         2     8888    FFFFF     1      */
/*    AAAAAAA  D     D  C          1        2     8    8   F         1      */
/*    A     A  D     D  C          1       2     8      8  F         1      */
/*    A     A  D    D   C     C    1      2      8      8  F         1      */
/*    A     A  D   D     C   C     1     2        8    8   F         1      */
/*    A     A  DDDD       CCC     111   2222222    8888    F        111     */
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
/*  Module Name: ad12f1rv                                                   */
/*                                                                          */
/*  Procedure Name(s): ADC128F1_Read_Volts                                  */
/*                                                                          */
/*  Description: This routine is used to read the voltage setting of the    */
/*  specified ADC128F1 channel.                                             */
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
/*  11-3-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* Global Variables */

/* Function Prototypes */
#include "./ad12f1dr.h"

int ADC128F1_Read_Volts(int path, int channel, float *volts_buf) {

	/* local variables */
	u_int16 reg_val;
	short sreg_val;

	/* adjust and range test the channel number */
	channel -= 1;
	if((channel < 0) || (channel > 7)) {
		errno = E_ILLARG;
		return(ERROR);
	}

	/* read the specified register. if bus error occurred, return with error */
	if(ADC128F1_Read_Reg(path, channel, &reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	/* convert hex register value to volts */
	reg_val &= 0x0fff;
	/* if value >= 800 then sign extend it */
	if(reg_val >= 0x0800) {
		sreg_val = reg_val | 0xf000;
	}
	else {
		sreg_val = reg_val;
	}
	*volts_buf = (float)(sreg_val) * 0.0048828;

	return(0);

}
