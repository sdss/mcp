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
/*  Module Name: dio316ri                                                   */
/*                                                                          */
/*  Procedure Name(s): DIO316_Read_ID_PROM                                  */
/*                                                                          */
/*  Description: These are the routines used to read the DIO316 ID PROM.    */
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
/*  6-22-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include "types.h"
#include "errno.h"
#include "gendefs.h"

/* definitions */
#define DIO316_ID_SIZE 12			/* 12 bytes */

/* Global Variables */

/* Function Prototypes */                                        
#include "./dio316dr.h"

static struct DIO316 *paths[MAX_DIO316]={(struct DIO316 *)-1,NULL};

int DIO316Init(struct DIO316 *addr, int vecno) {
	int i,ii;

/* first pass, initialize at RT all the paths */
	if ((int)paths[0]==-1) 
	  for (i=0;i<MAX_DIO316;i++) paths[i]=NULL;
/* find the first available index of addresses */
	for (i=0;i<MAX_DIO316;i++)
        {
          if (paths[i]==NULL) 
          {
	    paths[i]=addr;
	    break;
	  }
        }
	if (i>=MAX_DIO316) 
	{
	  printf ("/r/n*** No Path Available (max=%d) ***/r/n",i);
	  return -1;
	}
	addr->REG.rg16[10]=vecno;
	printf ("Path %d: ID = ",i);
	for (ii=0;ii<12;ii++) printf (" %02x",addr->ID[ii]);
	printf ("\r\n");
	return i;
}
int DIO316_Read_ID_PROM(int path, u_int8 *buf) {
	
	/* local variables */
	int i, reg_offset;
	u_int16 buf_16[DIO316_ID_SIZE];

	i = 0;
	/* do the read */
	for( i = 0 ; i < DIO316_ID_SIZE ; i++) {
		/* the ID info is every other byte on odd boundaries */
		reg_offset = i * 2;
		/* if bus error occurred, return with error */
		if(DIO316ReadID(path, reg_offset, &buf_16[i])) {
			errno = E_BUSERR;
			return(ERROR);
		}
		*buf++ = (u_int8)(buf_16[i] & 0x00FF);
	}

	return(0);
}
int DIO316WriteReg8(int path,int off,u_int8 val) 
{
	paths[path]->REG.rg8[off]=val;
	return 0;
};
int DIO316WriteReg16(int path,int off,u_int16 val) 
{
	paths[path]->REG.rg16[off>>1]=val;
	return 0;
};
int DIO316ReadReg16(int path,int off,u_int16 *val) 
{
	*val = paths[path]->REG.rg16[off>>1];
	return 0;
};
int DIO316ReadID(int path,int off,u_int16 *val) 
{
	*val = paths[path]->ID[off];
	return 0;
};
unsigned char DIO316_Int_Status=0;
int DIO316ClearISR(int path) 
{
	DIO316_Int_Status=0;
	return 0;
};
int DIO316ReadISR(int path,u_int8 *val) 
{
	DIO316_Int_Status=paths[path]->REG.rg8[13];
	*val=DIO316_Int_Status;
	paths[path]->REG.rg8[13]=DIO316_Int_Status;
	return 0;
};
