/* ipOctalSerial.c - GreenSpring IP-Octal serial driver
*
* Version: "%W%    %G% TSL"
*/                                                               

#pragma ident "%W%    %G% TSL" 


/*
modification history
--------------------
940420,1.1,LT	written.
*/

/*
DESCRIPTION
This is a driver for the GreenSpring IP Octal serial module.


USER-CALLABLE ROUTINES
Most of the routines in this driver are accessible only through the I/O
system. Three routines, however, must be called directly:

octSerModuleInit()  - To Create device descriptors and initialize the module
					  hardware.
octSerDrv() 		- To install the driver in the I/O system.
octSerDevCreate() 	- To create devices.


IOCTL FUNCTIONS
This driver supports the same ioctl() codes as a normal vxWorks tty driver.
In addition to that it also supports the following function codes defined
in ioLibx.h

FIOPARITY	- Set parity. The parity is specified with a character argument.
				'N' - No parity
				'E' - Even parity
				'O' - Odd parity
				'S' - Space parity
				'M' - Mark parity

FIODATABITS - Set number of data bits. Integer  5 <= databits <= 8.

FIOFMODE	- Set FIFO mode on/off. arg != 0 => on, arg == 0 => off.
			  FIFO mode is only available on channels with even channel
			  numbers as explained below.
			  This setting controls if receiver interrupts are
			  generated for every character received, or only when the
              receiver FIFO is full. To use the FIFO-full interrupt mode
			  a timeout is needed to drain any half filled FIFO's when
			  there is a halt in the input data stream. The SCC2698 have
              a special hardware timer for this purpose unfortunately
			  it has only 4 timers and 8 serial channels. The FIFO mode
			  is there for only implemented on channels with even channel
			  numbers.


AUTHOR
Leif Thuresson,  Control System Group,
				 The Svedberg National Accelerator Laboratory,
                 Uppsala, Sweden
                 leif@tsl.uu.se

*/

#include "vxWorks.h"
#include "iv.h"
#include "ioLibx.h"
#include "iosLib.h"
#include "tyLib.h"
#include "intLib.h"
#include "errnoLib.h"
#include "ctype.h"
#include "stdlib.h"
#include "stdio.h"
#include "drv/serial/ipOctalSerial.h"
                          
struct HISTORY_POOL {
	char outchar;
	FILE *stream;
};
	struct HISTORY_POOL ip_history_pool[1024];
	struct HISTORY_POOL *ip_current_history=&ip_history_pool[0];

LOCAL int octSerDrvNum;		/* Driver number assigned to this driver */

/*
 * baudTable is a table of the available baud rates, and the values to
 * write to the clock select register to get those rates
 */

LOCAL BAUD baudTable[]	=	{
	{ 75, RX_CLK_75 | TX_CLK_75},
	{ 110, RX_CLK_110 | TX_CLK_110},
	{ 150, RX_CLK_150 | TX_CLK_150},
	{ 300, RX_CLK_300 | TX_CLK_300},
	{ 600, RX_CLK_600 | TX_CLK_600},
	{ 1200, RX_CLK_1200 | TX_CLK_1200},
	{ 1800, RX_CLK_1800 | TX_CLK_1800},
	{ 2000, RX_CLK_2000 | TX_CLK_2000},
	{ 2400, RX_CLK_2400 | TX_CLK_2400},
	{ 4800, RX_CLK_4800 | TX_CLK_4800},
	{ 9600, RX_CLK_9600 | TX_CLK_9600},
	{ 19200, RX_CLK_19200 | TX_CLK_19200},
	{ 38400, RX_CLK_38400 | TX_CLK_38400},
};

/*
 * parityTable is a table of available parity settings, and the
 * corresponding bit values for mode register 1
 */

LOCAL PARITY parityTable[] = {
	 {'N', PARITY_NO},
	 {'O', PARITY_ODD},
	 {'E', PARITY_EVEN},
	 {'M', PARITY_MARK},
	 {'S', PARITY_SPACE}
};


/*
* Forward declarations
*/

LOCAL void   octSerStartup();
LOCAL int    octSerOpen();
LOCAL STATUS octSerIoctl();
LOCAL void	octSerInt(OCT_SER_DEV *pOctSerDv);

/*******************************************************************************
* octSerResetChannel - Reset a single channel
*
* DESCRIPTION
* Resets a channel and initializes it to default values:
* 9600 baud, no parity, 8 data bits, 1 stop bit, FIFO mode off.
*
* RETURNS: N/A.
*/
LOCAL void octSerResetChannel
	(
    OCT_SER_DEV *pOctSerDv      /* Pointer to device descriptor */
	)
{
	char dummy;

	/* Disables interrupts for both channels in the block ! */
/*	*pOctSerDv->intMsk = *pOctSerDv->pIntMskVal = 0;v1.0*/
	*pOctSerDv->intMsk = 0;

	*pOctSerDv->cmd = RST_TX_CMD;			/* Reset transmitter */
	*pOctSerDv->cmd = RST_RX_CMD;			/* Reset receiver */
	*pOctSerDv->cmd = RST_BRK_INT_CMD;		/* Reset break */
	*pOctSerDv->cmd = RST_ERR_STS_CMD;		/* Reset error status */
	*pOctSerDv->cmd = RST_MR_PTR_CMD;		/* Reset mode register pointer */

	/* Configure mode register 1 */
	*pOctSerDv->mode12 = pOctSerDv->mode1Val = PARITY_NO | BITS_CHAR_8;

	/* Configure mode register 2 */
	*pOctSerDv->mode12 = pOctSerDv->mode2Val = STOP_BITS_1;

	/* Set default baud rate */
	*pOctSerDv->clkSel = pOctSerDv->clkSelVal = RX_CLK_9600 | TX_CLK_9600;

	/* Select baud rate set 2, and bit clock for counter source */
	*pOctSerDv->auxCntrl = BRG_SELECT | CTR_TXC; 

    /* Set timeout delay to ~5 characters in FIFO mode */
	*pOctSerDv->cntMsb = 0;
    *pOctSerDv->cntLsb = 9*5;           

	/* Disable timeout mode */
	*pOctSerDv->cmd = TIMEOUT_MODE_DIS;

	/* Stop counter */
	dummy = *pOctSerDv->cntStop;


	/* Select RTSN for output on MPO */
	*pOctSerDv->outPortCfg = 0;

	*pOctSerDv->cmd = RST_TX_CMD;			/* Reset transmitter */
	*pOctSerDv->cmd = RST_RX_CMD;			/* Reset receiver */
}
/*******************************************************************************
* octSerHrdInit - Initialize IP hardware
*
* DESCRIPTION
* Initialize the hardware of an IP octal serial module.
*
* RETURNS: N/A.
*/
LOCAL void octSerHrdInit
    (
    OCT_SER_DEV *pOctSerDv      /* Pointer to start of device descriptor */
    )                           /*   array for this module */
{
    int   oldlevel;             /* Current interrupt level mask */
	int        i;

    oldlevel = intLock();       /* Disable interrupts during init */

	for(i=0; i<N_CHANNELS; i++)
		octSerResetChannel(pOctSerDv++);		/* Reset channel */

	intUnlock(oldlevel);
}
/*******************************************************************************
* octSerModuleInit - Initialize an IP octal serial module.
*
* DESCRIPTION
* This routine allocates and initializes 8 device descriptors, one
* for each serial channel on the IP octal serial module. It also
* calls octSerHrdInit() to initialize the IP hardware.
*
* RETURNS:
* A Pointer to the device descriptor array on success, or NULL on error.
*/
OCT_SER_DEV *octSerModuleInit
	(
	UINT8 *pIpMemBase,	/* IP memory space base address for this module */
	UINT8 *pIpIoBase,	/* IP IO space base address for this module */
	int intVecNum		/* Interrupt vector number to use */
	)
{
	int i;
	OCT_SER_DEV *pOctSerDv;
/*	UINT8 *pIntMskValStore;v1.1*/

	/*
	* Allocate device descriptor array
	*/
	if((pOctSerDv = malloc(sizeof(OCT_SER_DEV) * N_CHANNELS)) == NULL)
		return(NULL);

	/*
	* Each block (2 channels) share a common, write only, interrupt mask
	* register. The current value in the interrupt mask register is also 
	* stored in the array allocated below. Two channels in each array element.
	*/
/*	if((pIntMskValStore = malloc(sizeof(UINT8) * N_CHANNELS / 2)) == NULL)
		return(NULL);v1.1*/
	
	for(i=0; i<N_CHANNELS; i +=2)	{
		pOctSerDv[i].created = pOctSerDv[i+1].created = FALSE;
		pOctSerDv[i].index = i;
		pOctSerDv[i+1].index = i + 1;

		pOctSerDv[i].mode12 	= pIpIoBase + 0x01;
		pOctSerDv[i+1].mode12 	= pIpIoBase + 0x11;
		pOctSerDv[i].status	= pIpIoBase + 0x03;
		pOctSerDv[i+1].status 	= pIpIoBase + 0x13;
		pOctSerDv[i].clkSel 	= pIpIoBase + 0x03;
		pOctSerDv[i+1].clkSel 	= pIpIoBase + 0x13;
		pOctSerDv[i].cmd 	= pIpIoBase + 0x05;
		pOctSerDv[i+1].cmd 	= pIpIoBase + 0x15;
		pOctSerDv[i].data 	= pIpIoBase + 0x07;
		pOctSerDv[i+1].data 	= pIpIoBase + 0x17;

		pOctSerDv[i].auxCntrl = pOctSerDv[i+1].auxCntrl = pIpIoBase + 0x09;
		pOctSerDv[i].intStat = pOctSerDv[i+1].intStat = pIpIoBase + 0x0B;
		pOctSerDv[i].intMsk = pOctSerDv[i+1].intMsk = pIpIoBase + 0x0B;
		pOctSerDv[i].cntMsb = pOctSerDv[i+1].cntMsb = pIpIoBase + 0x0D;
		pOctSerDv[i].cntLsb = pOctSerDv[i+1].cntLsb = pIpIoBase + 0x0E;
		pOctSerDv[i].outPortCfg = pOctSerDv[i+1].outPortCfg = pIpIoBase + 0x1B;
		pOctSerDv[i].cntStart = pOctSerDv[i+1].cntStart = pIpIoBase + 0x1D;
		pOctSerDv[i].cntStop = pOctSerDv[i+1].cntStop = pIpIoBase + 0x1F;

/*		pOctSerDv[i].pIntMskVal = pOctSerDv[i+1].pIntMskVal =
					  &pIntMskValStore[i/2];

		pOctSerDv[i].txIntBit = TX_RDY_A_INT;
		pOctSerDv[i].rxIntBit = RX_RDY_A_INT;
		pOctSerDv[i+1].txIntBit = TX_RDY_B_INT;
		pOctSerDv[i+1].rxIntBit = RX_RDY_B_INT;v1.1*/

		pIpIoBase += 32;
	}

	/*
	* Interrupt vector must be set through memory address space, since ID
	* address space is read only (see note for Rev C. of IP-octal serial
	* address decoder PAL).
	*/ 

	*pIpMemBase = intVecNum;

	intConnect(INUM_TO_IVEC(intVecNum), octSerInt, (unsigned int) pOctSerDv);

	octSerHrdInit(pOctSerDv);

	return(pOctSerDv);
}

/*******************************************************************************
* octSerDrv - Initialize the tty driver
*
* DESCRIPTION
* This routine installs the driver for IP octal serial modules. It should
* be called once before calling octSerDevCreate(). 
*
* RETURNS:
* OK, or ERROR if the driver cannot be installed.
*/
STATUS octSerDrv(void)
{
    /* Check if driver already installed */

	if(octSerDrvNum > 0)
		return(OK);

	octSerDrvNum = iosDrvInstall(octSerOpen, (FUNCPTR) NULL, octSerOpen,
				(FUNCPTR) NULL, tyRead, tyWrite, octSerIoctl);

	return(octSerDrvNum == ERROR ? ERROR : OK);
}

/*******************************************************************************
* octSerDevCreate - Create a device for an IP octal serial channel
*
* DESCRIPTION
* This routine creates a device on a specified serial channel. Each channel
* to be used should have exactly one device associated with it by calling
* this routine.
*
* RETURNS:
* OK, or ERROR if the driver is not installed, or the device already exists.
*/
STATUS octSerDevCreate
	(
	OCT_SER_DEV *pOctSerDv,		/* Pointer to device descriptor */
	char        *name,          /* Name to use for this device */
	int         rdBufSize,      /* Read buffer size, in bytes */
	int         wrtBufSize      /* Write buffer size, in bytes */
	)
{
	char dummy;

	if(octSerDrvNum <= 0)	{
		errnoSet(S_ioLib_NO_DRIVER);
		return(ERROR);
	}

	/* if there is a device already on this channel, don't do it */

	if(pOctSerDv->created)
		return(ERROR);

	/*
	* Initialize the descriptor, enable receiver interrupts, enable
	* transmitter and receiver.
	*/

	if(tyDevInit(&pOctSerDv->tyDev, rdBufSize, wrtBufSize,
		   		 (FUNCPTR) octSerStartup) != OK)	{
		return(ERROR);
	}

    /* Enable interrupts */

/*	if(pOctSerDv->index == 0)	{
		*pOctSerDv->intMsk = *pOctSerDv->pIntMskVal |= pOctSerDv->rxIntBit |
						 CTR_RDY_INT;
		dummy = *pOctSerDv->cntStop;
	}
	else	{
		*pOctSerDv->intMsk = *pOctSerDv->pIntMskVal |= pOctSerDv->rxIntBit;
	}v1.1*/
	*pOctSerDv->intMsk = TX_RDY_A_INT | RX_RDY_A_INT | TX_RDY_B_INT |
						 RX_RDY_B_INT | CTR_RDY_INT;
	
 	dummy = *pOctSerDv->cntStop;

    /* Enable receiver */
/*	*pOctSerDv->cmd = TX_ENABLE | RX_ENABLE;v1.1*/
	*pOctSerDv->cmd = RX_ENABLE;


	/* Ready to receive ! */
	*pOctSerDv->cmd = ASSERT_RTS;

	/* Mark the device as created, and add the device to the I/O system */

	pOctSerDv->created = TRUE;

	return(iosDevAdd(&pOctSerDv->tyDev.devHdr, name, octSerDrvNum));
}

/*******************************************************************************
* octSerOpen - Open file to SCC
*
*/
LOCAL int octSerOpen
	(
    OCT_SER_DEV *pOctSerDv,     /* Pointer to device descriptor */
	char      *name,			/* Dummy arg, device name */
	int        mode				/* Dummy arg, mode */
	)
{
	return((int) pOctSerDv);
}
/*******************************************************************************
* octSerIoctl - Special device control
*
* DESCRIPTION
* This routine handles FIOBAUDRATE, FIOPARITY, FIODATABITS and FIOFMODE,
* all other requests are passed to tyIoctl().
*
* RETURNS:
* OK, or ERROR if invalid arg, or whatever tyIoctl() returns.
*/

LOCAL STATUS octSerIoctl
	(
    OCT_SER_DEV *pOctSerDv,     /* Pointer to device descriptor */
	int        request,			/* Request code */
	int        arg				/* Some argument */
	)
{
	int  	i;
	STATUS  status;
	UINT8	oldValue;

	status = ERROR;
	switch(request)	{

		case FIOBAUDRATE:
			for(i = 0; i < sizeof(baudTable)/sizeof(BAUD); i++)
				if(baudTable[i].rate == arg)	{
					status = OK;	/* baud rate is valid */

				/* Already correct baud rate ? */
				if(pOctSerDv->clkSelVal == baudTable[i].csrVal)
					break;		/* Yes, done ! */

				/* Disable transmitter and receiver */
				*pOctSerDv->cmd = TX_DISABLE | RX_DISABLE;

				/* configure transmitter and receiver */
				*pOctSerDv->clkSel = pOctSerDv->clkSelVal = baudTable[i].csrVal;

				/* Reset transmitter and receiver */
				*pOctSerDv->cmd = RST_TX_CMD;
				*pOctSerDv->cmd = RST_RX_CMD;

				/* Enable receiver */
/*				*pOctSerDv->cmd = TX_ENABLE | RX_ENABLE;v1.1*/
				*pOctSerDv->cmd = RX_ENABLE;

				break;
			}
			break;

		case FIOPARITY:
			arg = toupper((char) arg);		/* Convert to upper case */
			for(i = 0; i < sizeof(parityTable)/sizeof(PARITY); i++)	{
				if(parityTable[i].parity == (char) arg)		{
					status = OK;

					/* Already correct parity ? */
					if(parityTable[i].parVal == (pOctSerDv->mode1Val &
												 PARITY_BIT_MSK))
						break;				/* Yes, done ! */

					/* Extract none parity bits from mode register 1 value */
					oldValue = pOctSerDv->mode1Val & ~PARITY_BIT_MSK;

                    /*
					* Disable transmitter and receiver.
					* Reset mode register pointer.
					*/
					*pOctSerDv->cmd = TX_DISABLE | RX_DISABLE | RST_MR_PTR_CMD;

					/* Set new parity */
					*pOctSerDv->mode12 = pOctSerDv->mode1Val =
										 oldValue | parityTable[i].parVal;

					/* Reset transmitter and receiver */
					*pOctSerDv->cmd = RST_TX_CMD;
					*pOctSerDv->cmd = RST_RX_CMD;

					/* Enable transmitter and receiver */
/*					*pOctSerDv->cmd = TX_ENABLE | RX_ENABLE;v1.1*/
					*pOctSerDv->cmd = RX_ENABLE;

					break;
				}
			}
			break;

		case FIODATABITS:
			if(arg >= 5 && arg <= 8)	{
				status = OK;

                /* Already correct number of data bits ? */
				if(arg-5 == (pOctSerDv->mode1Val & DATABITS_BIT_MSK))
					break;	/* Yes, done ! */

				/* Extract none data bits bits from mode register 1 value */
				oldValue = pOctSerDv->mode1Val & ~DATABITS_BIT_MSK;

				/*
				* Disable transmitter and receiver.
				* Reset mode register pointer.
				*/
				*pOctSerDv->cmd = TX_DISABLE | RX_DISABLE | RST_MR_PTR_CMD;

                /* Set new number of data bits value */
				*pOctSerDv->mode12 = pOctSerDv->mode1Val =
									 oldValue | (UINT8) (arg-5);

				/* Reset transmitter and receiver */
				*pOctSerDv->cmd = RST_TX_CMD;
				*pOctSerDv->cmd = RST_RX_CMD;

				/* Enable receiver */
/*				*pOctSerDv->cmd = TX_ENABLE | RX_ENABLE;v1.1*/
				*pOctSerDv->cmd = RX_ENABLE;

			}
			break;

		case FIOFMODE:
			if(pOctSerDv->index % 2)	/* FIFO mode only supported on */
				break;					/* even channel numbers */

			status = OK;
			if(arg)	{	/* Enable FIFO mode */

				/* Reset command reg pointer */
				*pOctSerDv->cmd = RST_MR_PTR_CMD;

				/* Set receiver interrupt on FIFO full */
				*pOctSerDv->mode12 = pOctSerDv->mode1Val |= RX_INT;

				/* Enable timeout mode */
				*pOctSerDv->cmd = TIMEOUT_MODE_EN;
			}
			else	{	/* Disable FIFO mode */

				/* Reset command reg pointer */
				*pOctSerDv->cmd = RST_MR_PTR_CMD;

				/* Set receiver interrupt on RxRDY */
				*pOctSerDv->mode12 = pOctSerDv->mode1Val &= ~RX_INT;

				/* Disable timeout mode */
				*pOctSerDv->cmd = TIMEOUT_MODE_DIS;

				/* Stop counter */
				oldValue = *pOctSerDv->cntStop;
			}
			break;

		default:
			status = tyIoctl(&pOctSerDv->tyDev, request, arg);
			break;
	}
	return(status);
}


/*******************************************************************************
* octSerStartup - Transmitter startup routine
*
* DESCRIPTION
* Call interrupt level character output routine.
*
* RETURNS: N/A.
*/
LOCAL void octSerStartup
	(
    OCT_SER_DEV *pOctSerDv      /* Pointer to device descriptor */
	)
{
/*	char outChar;

	if(tyITx(&pOctSerDv->tyDev, &outChar) == OK)	{
	  *pOctSerDv->data = outChar;
	  *pOctSerDv->intMsk = *pOctSerDv->pIntMskVal |= pOctSerDv->txIntBit;
	}v1.1*/
	*pOctSerDv->cmd = TX_ENABLE;
}
/*******************************************************************************
* octSerInt - Interrupt service routine.
*
* DESCRIPTION
* This routine handles interrupts from a IP octal serial module.
* It polls each channel to determine the cause of the interrupt, since
* there is only a single interrupt vector for the module.
*
* RETURNS: N/A.
*/
LOCAL void octSerInt
	(
    OCT_SER_DEV *pOctSerDv      /* Pointer to start of device descriptor */
    )                           /*  array for this module */
{
	UINT8	intStatus;
	char	outChar;
	char    inChar;
	int	i;

	/* We need to find out which channel interrupted.  */
	for(i=0; i<N_CHANNELS; i++, pOctSerDv++)	{
		if((intStatus = *pOctSerDv->intStat) == 0)
			continue;

		if(pOctSerDv->created == FALSE)	{	/* Error, disable interrupt */
/*			*pOctSerDv->intMsk = *pOctSerDv->pIntMskVal &=
				 ~(pOctSerDv->txIntBit | pOctSerDv->rxIntBit);v1.1*/
		    	*pOctSerDv->intMsk = 0;
			continue;
		}

		/* Tx buffer empty int ? */
		if(*pOctSerDv->status & TXRDY)	{     
		  outChar=0x7F;
		  if(tyITx(&pOctSerDv->tyDev, &outChar) == OK)	{
		    ip_current_history->outchar=outChar;
	            ip_current_history->stream=(FILE *)&pOctSerDv->tyDev;
		    ip_current_history++;
		    if (ip_current_history>&ip_history_pool[1000])
		      ip_current_history=&ip_history_pool[0];
		    *pOctSerDv->data = outChar;
		  }
		  else	{
                /* No more chars to transmit. Clear transmitter interrupt */
		    *pOctSerDv->cmd = TX_DISABLE;
		  }
		}
/*		if((intStatus & *pOctSerDv->pIntMskVal) & pOctSerDv->txIntBit)	{	
		  outChar=0x7F;
		  if(tyITx(&pOctSerDv->tyDev, &outChar) == OK)	{
		    ip_current_history->outchar=outChar;
	            ip_current_history->stream=(FILE *)&pOctSerDv->tyDev;
		    ip_current_history++;
		    if (ip_current_history>&ip_history_pool[1000])
		      ip_current_history=&ip_history_pool[0];
		    *pOctSerDv->data = outChar;
		  }
		  else	{
	            *pOctSerDv->intMsk = *pOctSerDv->pIntMskVal &=
						 ~pOctSerDv->txIntBit;
		  }
		}v.1.1*/

		if(intStatus & CTR_RDY_INT)	{
			*pOctSerDv->cmd = TIMEOUT_MODE_EN;	/* Clear counter interrupt */
		}

		/*
        * Drain receiver FIFO. If we test on status-reg instead of
		* int-status-reg it will work in both character and FIFO mode.
		*/
		while(*pOctSerDv->status & RXRDY)
			tyIRd(&pOctSerDv->tyDev, inChar = *pOctSerDv->data);

	}
}
void print_ip_history(int num)
{
  struct HISTORY_POOL *h;
  FILE *stream;
  int i;

  stream = NULL;

  h=max(ip_current_history-num,&ip_history_pool[0]);
  for (i=0;i<num;i++) 
  {
    if (stream==h->stream)
    {
      if (((h->outchar&0x7F)>='0')&&((h->outchar&0x7F)<='F'))
        printf("%c ",h->outchar&0x7F);
      else
      {	
        printf("(%2x) ",h->outchar);
        if (h->outchar=='\r') printf("    \r\n");
      }
    }
    else
    {
      printf ("  TY_DEV=%p\r\n",h->stream);
      if (((h->outchar&0x7F)>='0')&&((h->outchar&0x7F)<='F'))
        printf("%c ",h->outchar&0x7F);
      else
      {	
        printf("(%2x) ",h->outchar);
        if (h->outchar=='\r') printf("    \r\n");
      }
      stream=h->stream;
    }
    h++;
  }
}
