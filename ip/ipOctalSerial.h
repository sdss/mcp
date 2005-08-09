/* ipOctalSerial.h - Header file to GreenSpring IP octal serial driver 
*
* Version: "@(#)ipOctalSerial.h	1.1    27 May 1994 TSL"
*
* Copyright (c) 1994 The Svedberg Laboratory.
*/

/* #pragma ident "@(#)ipOctalSerial.h	1.1    27 May 1994 TSL" */


/*
modification history
--------------------
940420,LT   written.
*/

#ifndef __INCipOctalSerialh
#define __INCipOctalSerialh

#ifdef __cplusplus
extern "C" {
#endif

#include "tyLib.h"
#include "scc2698.h"

/*
* Bit values for parity settings 
*/
#define PARITY_NO           PAR_MODE_NO
#define PARITY_ODD          (PAR_MODE_YES | PARITY_TYPE)
#define PARITY_EVEN         PAR_MODE_YES
#define PARITY_SPACE        PAR_MODE_FORCE
#define PARITY_MARK         (PAR_MODE_FORCE | PARITY_TYPE)
#define PARITY_BIT_MSK      0x1C

#define DATABITS_BIT_MSK	0x3

/*
* Command register codes
*/
#define TIMEOUT_MODE_EN		0xa0
#define TIMEOUT_MODE_DIS	0xc0

/*
* Device descriptor
*/
typedef struct	{	/* OCT_SER_DEV */
	TY_DEV tyDev;
	BOOL   created;				/* True if device has already been created */
	int    index;				/* Channel index */

								/* CHANNEL REGISTER POINTERS */
	volatile UINT8  *mode12;	/* Mode register 1 and 2 */
	volatile UINT8 *status;		/* Status register */
	volatile UINT8 *clkSel;		/* Clock select register */
	volatile UINT8 *cmd;		/* Command register */
	volatile UINT8 *data;		/* Receiver/Transmitter holding register */

								/* BLOCK REGISTER POINTERS */	
	volatile UINT8 *auxCntrl;	/* Auxiliary control register */
	volatile UINT8 *intStat;	/* Interrupt status register */
	volatile UINT8 *intMsk; 	/* Interrupt mask register */
	volatile UINT8 *cntMsb;		/* Counter register MSB */
	volatile UINT8 *cntLsb;		/* Counter register LSB */
	volatile UINT8 *outPortCfg;	/* Output port configuration register */
	volatile UINT8 *cntStart; 	/* Start counter */
	volatile UINT8 *cntStop; 	/* Stop counter */

	UINT8  *pIntMskVal;			/* Pointer to saved interrupt mask */
	UINT8  txIntBit;			/* Transmitter interrupt mask bit  */
	UINT8  rxIntBit;			/* Receiver interrupt mask bit */
    UINT8  mode1Val;            /* Saved mode register 1 */
    UINT8  mode2Val;            /* Saved mode register 2 */
	UINT8  clkSelVal;			/* Saved clock select register */
} OCT_SER_DEV;


typedef struct	{	/* BAUD */
	int rate;		/* a baud rate */
	UINT8 csrVal;	/* Corresponding value to write to clock select register */
} BAUD;

typedef struct	{	/* PARITY */
	char parity;	/* a parity represented by a character */
	UINT8 parVal;	/* Corresponding bit value in mode register 1 */
}PARITY;


/* Function prototypes */

OCT_SER_DEV *octSerModuleInit(UINT8 *pIpMemBase, UINT8 *pIpIoBase,
							  int intVecNum);
STATUS octSerDrv(void);
STATUS octSerDevCreate(OCT_SER_DEV *pOctSerDv, char *name,
					   int rdBufSize, int wrtBufSize);


#ifdef __cplusplus
}
#endif

#endif /* __INCipOctalSerialh */
