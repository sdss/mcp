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
/*  Module Name: ad12f1uc                                                   */
/*                                                                          */
/*  Procedure Name(s): ADC128F1_CVT_Update_Control                          */
/*                                                                          */
/*  Description: This routine is used to enable or disable the update of    */
/*  ADC128F1 AD current value table.                                        */
/*                                                                          */
/*  Inputs:   Path to device driver                                         */
/*            Enable/Disable command                                        */
/*  Outputs:  The CVT update is Enabled/Disabled                            */
/*            Return status - 0 = OK, -1 = ERROR                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  1-5-94    Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"
#include "ad12f1ld.h"
#include "ad12f1rg.h"

/* Global Variables */

/* Function Prototypes */
#include "./ad12f1dr.h"

int ADC128F1_CVT_Update_Control(int path, enum Enable_Disable_Type ena_dis) {

	/* local variables */
	u_int16 reg_val, mask;

	/* read the control register. if bus error occurred, return with error */
	if(ADC128F1_Read_Reg(path, ADC128F1_CTL, &reg_val) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	/* either OR in or AND out the CVT update control bit */
	switch(ena_dis) {
	
		case DISABLE:
			mask = reg_val & (!ADC128F1_CSR8_N_SUPDIS);
			break;

		case ENABLE:
			mask = reg_val | ADC128F1_CSR8_N_SUPDIS;
			break;
			
		default:
			errno = E_ILLARG;
			return(ERROR);
			break;
			
	}

	/* write the control register with the new value */
	if(ADC128F1_Write_Reg(path, ADC128F1_CTL, mask) == ERROR) {
		errno = E_BUSERR;
		return(ERROR);
	}

	return(0);

}
