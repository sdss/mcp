/* --------------------------------------------------------------------------
// Copyright (c) 1989-1994 S-S Technologies Inc.
//
// Module Name: std_dhp.c
// ----------------------
//
// Demo module which provides standard functions for accessing the 5136-SD
// on dh/dh+ using the S-S Tech Native mode Data Highway driver (sddh.ss1)
// or the S-S Tech Native mode Data Highway Plus driver (sddhp.ss1)
//
// This program was compiled with Borland C++ 3.1
//
// Rev 5.01 jan 94
// Rev 5.02 apr 95 - change card status timeout to 10 seconds from 2 seconds
// Rev 5.03 dec 96 - port to vxWorks, VME 162, FNAL
// --------------------------------------------------------------------------*/

#include "stdio.h"
#include "stdlib.h"
#include "vxWorks.h"
#include "taskLib.h"
#include "string.h"
#include "abdh.h"
#include "std_dhp.h"

ZIF * zp=NULL;
uchr loc_sta;
int DHP_verbose=0;
int DHP_baud=PFL_57K;
#define VME_BYTE_ACCESS 1
#ifdef VME_BYTE_ACCESS
void vmemcpy( void * dst, void * src, uint len )
{
    uint i;

    for( i = 0; i < len; ++i )
        *((uchr *)dst)++ = *((uchr *)src )++;
}
#endif

/* dh_msg packages message for Data Highway.
 *  - checks for available tx buffer
 *  - fills in new transaction number, and sends message
 *  - waits for reply
 *  - extracts msg from queue
 *  - checks if transaction number matches
 *  - places reply in buffer
 * Returns Data Highway error status. */
uint dh_msg (void * msg, void * reply, uchr *reply_len, uint tout)
{
    uint ofs;
    int i;
    uchr *rp;
    static uint tns = 1000;
    MU *mu;
    uint err;                   /* error status              */
    static uint back_off_tme=20;/*500;*//* tns error back off time   */

    if (!tx_ready(2000))        /* wait 2 secs for OK to send on tx */
    {
        printf ("Timeout waiting for Clear TX buffer\n");
        exit (1);
    }

    ofs = zp->PC_TXB;
    swab ((char *)&ofs,(char *)&ofs,2);

    mu = (MU *)((ulng)zp | ofs);

    ((MU *)msg)->tns[0] = (++tns)>>8;   /* unique transaction number */
    ((MU *)msg)->tns[1] = (tns)&0xFF;   /* unique transaction number */

#ifdef VME_BYTE_ACCESS
    vmemcpy (&mu->rem,&((MU *)msg)->rem,((MU *)msg)->len + 2);
#else
    _fmemcpy (&mu->rem,&((MU *)msg)->rem,((MU *)msg)->len + 2);
#endif

    if (DHP_verbose)
    {
      printf ("\r\n XMT: len=%x, dst=%x, src=%x, cmd=%x, sts=%x, tns=%02x%02x\r\n",
	mu->len,mu->dst,mu->src,mu->cmd,mu->sts,mu->tns[0],mu->tns[1]);
    }
    zp->PC_TXC = IFT_SM;        /* send msg                  */

    if (!rx_avail(tout))        /* wait for msg in rx queue  */
    {
        return STS_TOUT;
    }

    ofs = zp->PC_RXB;
    swab ((char *)&ofs,(char *)&ofs,2);

    mu = (MU * )((ulng)zp | ofs);


    if (((uint)(mu->tns[0])<<8)+mu->tns[1] != tns )
    {                            /* transaction number mismatch */
        while (zp->PC_RXC == IFR_MP) /* flush buffer */
        {
            zp->PC_RXC = IFR_MR;     /* acknowledge msg */
            printf ("\r\n XMT: tns=%04x\r\n",tns);
            printf ("\r\n RCV: len=%x, dst=%x, src=%x, cmd=%x, sts=%x, tns=%02x%02x\r\n",
	mu->len,mu->dst,mu->src,mu->cmd,mu->sts,mu->tns[0],mu->tns[1]);
            taskDelay (back_off_tme+=100);  /* allow z80 time to indicate another */
      if (mu->len>6)
      {
        rp = (uchr *)&mu->var;
	printf ("  VAR:  ");
        for (i=0;i<mu->len-6;i++)
	  printf (" %02x",*rp++);
	printf ("\r\n");
      }
        }                            /* if there is one                    */
        return STS_TNS_MISM;
    }
    if (DHP_verbose)
    {
      printf (" RCV: len=%x, dst=%x, src=%x, cmd=%x, sts=%x, tns=%02x%02x\r\n",
	mu->len,mu->dst,mu->src,mu->cmd,mu->sts,mu->tns[0],mu->tns[1]);
      if (mu->len>6)
      {
        rp = (uchr *)&mu->var;
	printf ("  VAR:  ");
        for (i=0;i<mu->len-6;i++)
	  printf (" %02x",*rp++);
	printf ("\r\n");
      }
    }

    if (mu->sts == 0xf0)
    {
        err = (mu->sts << 8) + mu->var[0];
    }
    else
    {
        err = mu->sts;
    }
                /* copy data part of message into reply buffer */
#ifdef VME_BYTE_ACCESS
    vmemcpy (reply,&mu->var,mu->len-6);
#else
    _fmemcpy (reply,&mu->var,mu->len-6);
#endif
    if (reply_len != NULL)
    {
      *reply_len = mu->len-6;
    }
    zp->PC_RXC = IFR_MR;        /* acknowledge msg */
    return err;
}

/*
// waits up to <tme> milliseconds for a rx message to become available
*/
int rx_avail(int tme)
{
    while ((zp->PC_RXC != IFR_MP) && tme--)
    {
        taskDelay(1);
    }
    return zp->PC_RXC == IFR_MP;       /* wait for message present */
}

/*
// waits up to <tme> milliseconds for a tx message unit to become available
*/
int tx_ready(int tme)
{
    while ((zp->PC_TXC != IFT_WT) && tme--)
    {
        taskDelay (1);
    }
    return zp->PC_TXC == IFT_WT;   /* wait for ready to transmit */
}

/* returns non-zero if node is active, 0 otherwise */
int node_active(uchr node)       /* used for DH+ only */
{
    uchr byte;

    if (node == zp->STNAD)
        return (1);
    byte = node & 0x07;
    return (zp->DHP_LIST[byte] & (1 << (node >> 3)));
}

/*
// checks SD card state, if card is on line, takes it off.
// fills in card options, and puts card on line.
// checks card state, and status, ad reports any errors.
*/
void start_card (uchr pc_ifl)
{
    int tme;    /* used for timeouts */

    if ( (zp->PC_TXC != IFT_IRS) && (zp->PC_RXC != IFR_IRS) )
    {
        for (tme = 0 ; (tme < 200) && (zp->PC_TXC != IFT_WT); tme++)
        {
            taskDelay (1);
        }
        if (zp->PC_TXC != IFT_WT)
        {
            printf("Timeout - ready for off-line command");
            zp->PC_TXC = IFT_IRS;   /* Take card off line        */
            exit(1);
        }
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
        for (tme = 0; (tme < 200) && (zp->PC_RXC != IFR_IRS) ; tme++)
        {
            taskDelay (1);
        }
        if (zp->PC_RXC != IFR_IRS)        
	{
            printf("Timeout - execute off-line command");
            exit(1);
        }
    }

    zp->STNAD  = loc_sta;           /* configure station address */
    zp->PC_IFL = pc_ifl;            /* set interface flags       */

    zp->PC_TXC = IFT_RES;           /* reset command             */

    for (tme = 0; (tme < 200) && (zp->PC_TXC != IFT_WT); tme++)
    {
        taskDelay (1);
    }
                                    /* wait while card starts    */
    if (zp->PC_TXC != IFT_WT)
    {
        printf("Timeout - execute card reset\n");
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
    }

    for (tme = 0; (tme < 1000) && (zp->status == TESTING); tme++)
    {
        taskDelay (1);
    }

    if (zp->status == CARD_OK)
    {
        return;
    }

    if (zp->status == TESTING)
    {
        printf("Timeout waiting for card status\n");
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
    }
    else if (zp->status & DUP_STATION)
    {
        printf("Duplicate station Detected\n");
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
    }
    else if (zp->status & INVALID_STATION)
    {
        printf("Invalid Station (DAI Error)\n");
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
    }
    else
    {
        printf("Unknown Card Status Error %02x\n",zp->status);
        zp->PC_TXC = IFT_IRS;       /* Take card off line        */
    }
    exit(1);
}

