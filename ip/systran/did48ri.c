/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48ri                                                   */
/*                                                                          */
/*  Procedure Name(s): DID48_Read_ID_PROM                                  */
/*                                                                          */
/*  Description: These are the routines used to read the DID48 ID PROM.    */
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
/*  2-4-97    Charlie Briegel Original Release                              */
/*                                                                          */
/****************************************************************************/

/* Include Files */
#include "types.h"
#include "errno.h"
#include "gendefs.h"

/* definitions */
#define DID48_ID_SIZE 12			/* 12 bytes */

/* Global Variables */

/* Function Prototypes */                                        
#include "./did48dr.h"

static struct DID48 *paths[MAX_DID48]={(struct DID48 *)-1,NULL};

int DID48Init(struct DID48 *addr, int vecno) {
	int i,ii;

/* first pass, initialize at RT all the paths */
	if ((int)paths[0]==-1) 
	  for (i=0;i<MAX_DID48;i++) paths[i]=NULL;
/* find the first available index of addresses */
	for (i=0;i<MAX_DID48;i++)
        {
          if (paths[i]==NULL) 
          {
	    paths[i]=addr;
	    break;
	  }
        }
	if (i>=MAX_DID48) 
	{
/*	  printf ("/r/n No Path Available (max=%d) /r/n",i);*/
	  return -1;
	}
	addr->REG.rg16[6]=vecno;
/*	printf ("Path %d: ID = ",i);
	for (ii=0;ii<12;ii++) printf (" %02x",addr->ID[ii]);
	printf ("\r\n");*/
	return i;
}
int DID48_Read_ID_PROM(int path, u_int8 *buf) {
	
	/* local variables */
	int i, reg_offset;
	u_int16 buf_16[DID48_ID_SIZE];

	i = 0;
	/* do the read */
	for( i = 0 ; i < DID48_ID_SIZE ; i++) {
		/* the ID info is every other byte on odd boundaries */
		reg_offset = i * 2;
		/* if bus error occurred, return with error */
		if(DID48ReadID(path, reg_offset, &buf_16[i])) {
			errno = E_BUSERR;
			return(ERROR);
		}
		*buf++ = (u_int8)(buf_16[i] & 0x00FF);
	}

	return(0);
}
int DID48WriteReg8(int path,int off,u_int8 val) 
{
	paths[path]->REG.rg8[off]=val;
	return 0;
};
int DID48WriteReg16(int path,int off,u_int16 val) 
{
	paths[path]->REG.rg16[off>>1]=val;
	return 0;
};
int DID48ReadReg16(int path,int off,u_int16 *val) 
{
	*val = paths[path]->REG.rg16[off>>1];
	return 0;
};
int DID48ReadID(int path,int off,u_int16 *val) 
{
	*val = paths[path]->ID[off];
	return 0;
};
unsigned char DID48_Int_Status=0;
int DID48ClearISR(int path,u_int8 val) 
{
	paths[path]->REG.rg8[9]=val;
	return 0;
};
int DID48ReadISR(int path,u_int8 *val) 
{
	*val=paths[path]->REG.rg8[9];
	return 0;
};
