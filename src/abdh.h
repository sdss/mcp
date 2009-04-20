/* --------------------------------------------------------------------------
// Copyright (c) 1989-1995 S-S Technologies Inc.
//
// Header File Name: abdh.h
// ------------------------
//
// Header file used by 5136-SD demo programs to access the card's
// Z-180 InterFace (ZIF) structure, and Message Units (MU).
//
// The structures in this file MUST be compiled using byte aligned or packed
// structure mode!
//
// --------------------------------------------------------------------------*/

/* data type definitions */

typedef unsigned char uchr;
typedef unsigned short uint;
typedef unsigned long ulng;


/* This structure describes the interface in the 5136-SD data memory    */

typedef struct zif
{
    uint    PC_RXB;         /* 00 ptr to receive buffer                 */
    uchr    PC_RXC;         /* 02 RX control                            */
    uchr    PC_IFL;         /* 03 interface flags                       */
    uint    PC_TXB;         /* 04 ptr to transmit buffer                */
    uchr    PC_TXC;         /* 06 TX control                            */
    uchr    STNAD;          /* 07 Station address                       */
    uchr    INT_EN;         /* 08 Interrupt Enable byte                 */

    uchr    MOD_ID;         /* 09 Module ID                             */

    uchr    DHP_LIST[8];    /* 0a List of active nodes on DH+           */
                            /*    DHP_LIST is valid only for            */
                            /*    Data Highway Plus                     */
    uchr    MSG_TOUT;       /* 12 msg timeout, for Data Highway only    */

    uchr    rsrvd1[4];      /*    do not use                            */
    uchr    PC_RX;          /* 17 # of msgs on PC Queue                 */
    uchr    rsrvd2;         /*    do not use                            */

    char    term_name[9];   /* 19 terminal name for node                */
    uchr    status;         /* 22 card status                           */
    uchr    ext_options;    /* 23 extended options                      */
    uint    wr_glob_data;   /* 24 global data to write from this stn    */

    uchr    rsrvd3[0x30-0x26];  /* do not use                           */

    uchr    diag_ctrs[35];  /* 30 dh+ diagnostic counters for this stn  */

    uchr    rsrvd4[0x100-0x53]; /* do not use                           */

    uint    glob_data[0100]; /* 100 global data (station 0-77 octal)    */
} ZIF;


/* definitions related to PC_RXC */

#define IFR_WT  0x04            /* no receive messages                  */
#define IFR_MP  0x08            /* message present                      */
#define IFR_MR  0x10            /* message received                     */
#define IFR_SR  0x14            /* send reply to message rcv'd          */
#define IFR_IRS 0x1c            /* interface requires reset             */


/* definitions related to PC_IFL */

#define PFL_57K      0x00       /* 57.6Kbaud DHP                        */
#define PFL_115K     0x01       /* 115.2Kbaud DHP                       */
#define PFL_230K     0x02       /* 230.4Kbaud DHP                       */
#define PFL_DUP      0x04       /* duplicate message check              */
#define PFL_XDC      0x08       /* execute diagnostic commands          */
#define PFL_RTN_MSGS 0x10       /* return msgs to offline stn's         */
#define PFL_FRCE_32K 0x20       /* force 32k map even if 16k            */
#define PFL_USE_REM  0x40       /* allow remote addressing              */
#define PFL_MULT_MSG 0x80       /* send multiple message on token       */


/* definitions related to PC_TXC */

#define IFT_WT  0x04            /* waiting for message to send          */
#define IFT_RES 0x0c            /* reset the interface                  */
#define IFT_SM  0x10            /* send message                         */
#define IFT_WB  0x14            /* waiting for buffer                   */
#define IFT_IRS 0x1c            /* interface requires reset             */


/* definitions related to INT_EN  */

#define EN_RX    0x01            /* enables receive interrupts          */
#define EN_TX    0x02            /* enables transmit interrupts         */
#define INT_PEND 0x80            /* interrupt pending                   */

/* definitions related to MOD_ID  */
/*  High nibble contains MODULE_TYPE (0xa0=dhp,0x50=dh)                 */
/*  Low nibble contains CARD_TYPE   (0=ks,1=kt/kl,2=vme,3=sd/mca,4=sd2) */

#define DHP_MODULE  0xa0
#define DH_MODULE   0x50

#define KS_CARD     0x00
#define KT_CARD     0x01
#define VME_CARD    0x02
#define SD_CARD     0x03
#define MCA_CARD    0x03
#define SD2_CARD    0x04

/*
// Data Highway Plus Active Stations List is arranged as follows:
//
//    BITS ->  0     1     2     3     4     5     6     7
//           ----  ----  ----  ----  ----  ----  ----  ----
// BYTE  0   #000  #010  #020  #030  #040  #050  #060  #070
//       1   #001  #011  #021  #031  #041  #051  #061  #071
//       2   #002  #012  #022  #032  #042  #052  #062  #072
//       3   #003  #013  #023  #033  #043  #053  #063  #073
//       4   #004  #014  #024  #034  #044  #054  #064  #074
//       5   #005  #015  #025  #035  #045  #055  #065  #075
//       6   #006  #016  #026  #036  #046  #056  #066  #076
//       7   #007  #017  #027  #037  #047  #057  #067  #077
//
// For Data Highway the DHP_LIST area is not used
*/


/* definitions related to status */

#define TESTING         0x00    /* testing, wait for non-zero           */
#define CARD_OK         0x80    /* card is on line ok                   */
#define DUP_STATION     0x01    /* duplicate station found              */
#define INVALID_STATION 0x02    /* invalid station found (DAI Error)    */


/* definitions related to ext_options */

#define SND_GLOB        0x01    /* send global data for local station   */
#define LARGE_BUFFERS   0x02    /* use 512 byte buffers                 */


/* Message Unit structure definition
 *  All Data Highway/Data Highway Plus messages follow this format.     */

typedef struct tagMU
{
    uchr len_hi;        /* hi byte of length if LARGE_BUFFERS is on */
    uchr rem;           /* used for remote addressing, DH+ only     */
    uchr len;           /* length of message data                   */

    uchr dst;           /* destination                              */
    uchr src;           /* source address                           */
    uchr cmd;           /* message command/reply                    */
    uchr sts;           /* reply status code (cmd = 0)              */
    uchr tns[2];           /* sequence number                          */
    uchr var[244];      /* variable data                            */
    uchr crc[2];        /* sdlc crc (reserved)                      */
    uchr spare;         /* spare byte                               */
} MU;

/* Remote or Off-Link Message Unit structure definition
 *  All Remote Data Highway Plus messages follow this format.       */

typedef struct tagREM_MU
{
    uchr len_hi;        /* hi byte of length if LARGE_BUFFERS is on */
    uchr rem;           /* used for remote addressing, DH+ only     */
    uchr len;           /* length of message data                   */

    uchr dst;           /* destination                              */
    uchr src;           /* source address                           */

    uchr ntwk;          /* network (usually set to 0x24)            */

    uchr did0;          /* destination link id                      */
    uchr did1;
    uchr dst0;          /* destination station                      */
    uchr dst1;

    uchr lftm;          /* lifetime (usually set to 0x80)           */

    uchr sid0;          /* source link id                           */
    uchr sid1;
    uchr src0;          /* source station                           */
    uchr src1;

    uchr nsap;          /* NSAP (usually set to 0x00)               */

    uchr cmd;           /* message command/reply                    */
    uchr sts;           /* reply status code (cmd = 0)              */
    uchr tns[2];           /* transaction number                       */
    uchr var[233];      /* variable data                            */
    uchr crc[2];        /* sdlc crc (reserved)                      */
    uchr spare;         /* spare byte                               */
} REM_MU;


/* definitions related to sts */
/* status errors generated by 5136-SD Card */
#define STS_NOMEM       0x01    /* dest could not take msg          */
#define STS_NOACK       0x02    /* dest did not send ack            */
#define STS_CONTENTION  0x03    /* unrecognized response from dest  */
#define STS_DISCON      0x04    /* local port is disconnected (dh+) */
#define STS_DUPL_STAT   0x06    /* duplicate station detected       */
#define STS_OFFLINE     0x07    /* dest is offline (PFL_RTN_MSGS=1) */
#define STS_DUPL_TNS    0x0e    /* card received duplicate TNS      */

/* status errors generated by application */
#define STS_TOUT        0x05    /* time-out waiting for reply       */
#define STS_TNS_MISM    0x0d    /* TNS mismatch                     */

#define STAT_FILE	0x84	/* status file type		    */
#define BIT_FILE	0x85	/* bit file type		    */
#define TIMR_FILE	0x86	/* timer file type		    */
#define CNTR_FILE	0x87	/* counter file type		    */
#define CTRL_FILE	0x88	/* control file type		    */
#define INT_FILE	0x89	/* integer file type		    */

/* function prototypes */
uint slc_write_blok(uchr dst, uint file_num, uint file_typ, 
		uint file_ofs, uint *data, int size);
uint slc_read_blok(uchr dst, uint file_num, uint file_typ,
		uint file_ofs, uint *data, int size);

struct SLC_MSG         /* SLC Protected, Typed Read/Write message format    */
{
    uchr rsrvd;
    uchr rem;
    uchr len;
    uchr dst;
    uchr src;
    uchr cmd;
    uchr sts;
    uchr tns[2];           /* filled in by dh_msg                          */
    uchr fnc;
    uchr num_bytes;
    uchr file;
    uchr type;
    uchr elem;
    uchr sub_elem;
    uchr data[243];
};

