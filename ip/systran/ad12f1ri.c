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
/*  Module Name: ad12f1ri                                                   */
/*                                                                          */
/*  Procedure Name(s): ADC128F1_Read_ID_PROM                                */
/*                                                                          */
/*  Description: These are the routines used to read the ADC128F1 ID PROM.  */
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
/*  11-3-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include <types.h>
#include <errno.h>
#include "gendefs.h"

/* definitions */
#define ADC128F1_ID_SIZE 12			/* 12 bytes */

/* Global Variables */

/* Function Prototypes */
#include "./ad12f1dr.h"

static struct ADC128F1 *paths[MAX_ADC128F1]={(struct ADC128F1 *)-1,NULL};

int ADC128F1Init(struct ADC128F1 *addr) {
        int i,ii;

/* first pass, initialize at RT all the paths */
        if ((int)paths[0]==-1)
          for (i=0;i<MAX_ADC128F1;i++) paths[i]=NULL;
/* find the first available index of addresses */
        for (i=0;i<MAX_ADC128F1;i++)
        {
          if (paths[i]==NULL) 
	  {
 	    paths[i]=addr;
            break;
	  }
        }
        if (i>=MAX_ADC128F1)
        {
/*          printf ("/r/n No Path Available (max=%d) /r/n",i);*/
          return -1;
        }
/*        printf ("Path %d: ID = ",i);
        for (ii=0;ii<12;ii++) printf (" %x",addr->ID[ii]);
        printf ("\r\n");*/
        return i;
}
int ADC128F1WriteReg16(int path,int off,u_int16 val)
{
        paths[path]->REG.rg16[off>>1]=val;
        return 0;
};
int ADC128F1ReadReg16(int path,int off,u_int16 *val)
{
        *val = paths[path]->REG.rg16[off>>1];
        return 0;
};
int ADC128F1ReadID(int path,int off,u_int16 *val)
{
        *val = paths[path]->ID[off];
        return 0;
};

int ADC128F1_Read_ID_PROM(int path, u_int8 *buf) {
	
	/* local variables */
	int i, reg_offset;
	u_int16 buf_16[ADC128F1_ID_SIZE];

	i = 0;
	/* do the read */
	for( i = 0 ; i < ADC128F1_ID_SIZE ; i++) {
		/* the ID info is every other byte on odd boundaries */
		reg_offset = i * 2;
		/* if bus error occurred, return with error */
		if(ADC128F1ReadID(path, reg_offset, &buf_16[i])) {
			errno = E_BUSERR;
			return(ERROR);
		}
		*buf++ = (u_int8)(buf_16[i] & 0x00FF);
	}

	return(0);
}
