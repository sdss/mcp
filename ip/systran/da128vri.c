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
/*  Module Name: da128vri                                                   */
/*                                                                          */
/*  Procedure Name(s): DAC128V_Read_ID_PROM                                 */
/*                                                                          */
/*  Description: These are the routines used to read the DAC128V ID PROM.   */
/*                                                                          */
/*  Inputs:   Path to device                                                */
/*            Ptr to read buffer                                            */
/*  Outputs:  ID PROM values are placed in read buffer                      */
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
#include "types.h"
#include "errno.h"
#include "gendefs.h"

/* definitions */
#define DAC128V_ID_SIZE 12			/* 12 bytes */

/* Global Variables */

/* Function Prototypes */
#include "./da128vdr.h"                                                                    

static struct DAC128V *paths[MAX_DAC128V]={(struct DAC128V *)-1,NULL};

int DAC128VInit(struct DAC128V *addr) {
        int i,ii;

/* first pass, initialize at RT all the paths */
        if ((int)paths[0]==-1)
          for (i=0;i<MAX_DAC128V;i++) paths[i]=NULL;
/* find the first available index of addresses */
        for (i=0;i<MAX_DAC128V;i++)
        {
          if (paths[i]==NULL) 
	  {
	    paths[i]=addr;
            break;
	  }
        }
        if (i>=MAX_DAC128V)
        {
          printf ("/r/n*** No Path Available (max=%d) ***/r/n",i);
          return -1;
        }
        printf ("Path %d: ID = ",i);
        for (ii=0;ii<12;ii++) printf (" %02x",addr->ID[ii]);
        printf ("\r\n");
        return i;
}
int DAC128VWriteReg16(int path,int off,u_int16 val)
{
        paths[path]->REG.rg16[off>>1]=val;
        return 0;
};
int DAC128VReadReg16(int path,int off,u_int16 *val)
{
        *val = paths[path]->REG.rg16[off>>1];
        return 0;
};
int DAC128VReadID(int path,int off,u_int16 *val)
{
        *val = paths[path]->ID[off];
        return 0;
};
 
int DAC128V_Read_ID_PROM(int path, u_int8 *buf) {
	
	/* local variables */
	int i, reg_offset;
	u_int16 buf_16[DAC128V_ID_SIZE];

	i = 0;
	/* do the read */
	for( i = 0 ; i < DAC128V_ID_SIZE ; i++) {
		/* the ID info is every other byte on odd boundaries */
		reg_offset = i * 2;
		/* if bus error occurred, return with error */
		if(DAC128VReadID(path, reg_offset, &buf_16[i])) {
			errno = E_BUSERR;
			return(ERROR);
		}
		*buf++ = (u_int8)(buf_16[i] & 0x00FF);
	}

	return(0);
}
