/* --------------------------------------------------------------------------
// Copyright (c) 1989-1994 S-S Technologies Inc.
//
// Program Name: sbslc.c
// ---------------------
//
// This program is used to demonstrate the capabilities of the 5136-SD/SD2
// with S-S Tech DH/DH+ modules.
//
// This program does not use the queuing capabilities of DH/DHP modules. SLC
// addressing mode is used.
//
// This program writes out a block of integers to an integer file
// in the SLC then reads the integers back and checks them against
// what was sent.
//
// The MOD_ID byte of the zif structure is used to determine whether the
// network is Data Highway or Data Highway Plus.
//
// Data Mismatches show up as '*' beside the cycle #. The integer value is
// incremented after each cycle up to 0x7fff.
//
// ******* WARNING ...
// ******* Consider the Consequences of Writing to an Integer File
// ******* in the target processor.
//
// This program was compiled with Borland C++ 3.1
// This module must be linked with STD_DHP.C
//
// Rev 5.00 jan 94
// -------------------------------------------------------------------------*/

#include "vxWorks.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"
#include "abdh.h"
#include "std_dhp.h"
#include "diag.h"
#include "taskLib.h"
/* definitions */


/* function prototypes */
uint slc_write_blok(uchr dst, uint file_num, uint file_typ, 
		uint file_ofs, uint *data, int size);
uint slc_read_blok(uchr dst, uint file_num, uint file_typ, 
		uint file_ofs, uint *data, int size);
void sbslc_help();

/* global variables */


void sbslc(int dst, uint file, uint ofs, int len, 
		unsigned char loc_station, int card_adr, char *name)
{
    extern int  DHP_baud;
    int  lp;            /* loop counter                 */
    int  cycle = 0;     /* cycle counter                */
    uint err;           /* variable to hold error state */
    uint data[122];     /* data buffer (120 words max)  */
    char term_name[10]={""};
    uint card_seg = 0xd000; /* Card address                 */
extern uchr loc_sta;            /* local station address        */
extern ZIF * zp;           /* ptr to Z80 control structure */

     strcpy (term_name,name);
    loc_sta=loc_station;
    card_seg=card_adr+0x800;
    zp = (ZIF *) (((ulng)(card_seg) << 4)|0xF0000000);
/*    printf ("zp=%p, local station=%o, local name=%s, MOD_ID=%x",
	zp,loc_sta,name,zp->MOD_ID);*/

   printf("Copyright (c) 1989-1994 S-S Technologies Inc.\r\n");
    

    /* point to base card address */
    zp = (ZIF *) ((ulng)(card_seg << 4)|0xF0000000);

    if ((loc_sta > 63) || (dst > 63))
    {
        printf("Invalid Data Highway Plus Station Address\r\n");
        exit(1);
    }
    if ((zp->MOD_ID & 0xf0) != DHP_MODULE)
    {
        printf("No Card or DH+ Module not running at %x", card_seg);
        exit(1);
    }

    if (len > 120)
    {
        printf("Length too Large!\r\n");
        exit(1);
    }

    printf("Sending SLC Block Write/Read messages\r\n");

    start_card (DHP_baud | PFL_DUP | PFL_XDC);

    strncpy(zp->term_name,term_name,8);

    taskDelay(1*60);  /* allow time for card to get ON LINE */

    while (cycle++ < 0x7f) 
    {
        printf("\r\nCycle # %04x",cycle);
        for (lp = 0; lp < len; lp++)
        {
            data[lp] = cycle;
        }

        err = slc_write_blok(dst, file, INT_FILE, ofs, data, len); /* write values to SLC */

        if (err)
        {
            printf("W Err=%04x\r\n", err);
        }
        else
        {
            err = slc_read_blok(dst, file, INT_FILE, ofs, data, len); /* read values from SLC */

            if (err)
            {
                printf ("R Err=%04x\r\n",err);
            }
            else
            {                       /* check that values read back match    */
                for (lp = 0; lp < len; lp++)
                {
                    if (data[lp] != cycle)
                    {
                        printf(" Offset=%5d Wrote %04x read %04x\r\n",
				lp,cycle,data[lp]);
                    }
                }
            }
        }
    }
    zp->PC_TXC = IFT_IRS;           /* Take card off line                   */
}


/* SLC write_blok and read_blok
 * Send msg SLC format,
 * retrieve reply and place in buffer.
 * Return the error status ... STS in upper byte,
 * EXT STS in lower if there is no EXT STS, than the value in the
 * lower byte is meaningless */

#define FILE_W_CMD 0x0f
#define FILE_W_FNC 0xaa         /* protected typed write */

uint slc_write_blok(uchr dst, uint file_num, uint file_typ, uint file_ofs, uint *data, 
		int size)
{
    struct SLC_MSG msg;    /* structure to build message               */
    uchr reply[20];         /* reserve 20 bytes for reply               */
extern uchr loc_sta;            /* local station address        */

/*	printf ("\r\n Send to %d file %d offset %d length %d",
		dst,file_num,file_ofs,size);*/
    msg.rem = 0;
    msg.dst = dst;
    msg.src = loc_sta;
    msg.cmd = FILE_W_CMD;
    msg.sts = 0;
    msg.fnc = FILE_W_FNC;

    msg.num_bytes = size*2;
    msg.file = file_num;
    msg.type = file_typ;
    msg.elem = file_ofs;
    msg.sub_elem = 0;

    memcpy(msg.data, data, size * 2);
    msg.len = 12 + (size * 2);
    return dh_msg(&msg,reply,NULL,5000);
}

#define FILE_R_CMD 0x0f
#define FILE_R_FNC 0xa2         /* protected typed read */

uint slc_read_blok(uchr dst, uint file_num, uint file_typ, uint file_ofs, uint *data, int size)
{
    struct SLC_MSG msg;    /* structure to build message               */
    uint err;
    uchr rply[256];
    uchr len;
extern uchr loc_sta;            /* local station address        */

    msg.rem = 0;
    msg.dst = dst;
    msg.src = loc_sta;
    msg.cmd = FILE_R_CMD;
    msg.sts = 0;
    msg.fnc = FILE_R_FNC;

    msg.num_bytes = size*2;
    msg.file = file_num;
    msg.type = file_typ;
    msg.elem = file_ofs;
    msg.sub_elem = 0;
    msg.len = 12;

    err = dh_msg(&msg, &rply,&len, 5000);
    if (err==0)
    {
        memcpy (data,&rply[0],len);
    }
    return err;
}
void sbslc_help()
{
        printf("USE:\r\n"
               "sbslc <plc sta addr> <slc_file_num> <slc_file_ofs> <length> <card sta>\n"
               "      [<card addr>] [<term_name>]\n"
               "      the optional card address is entered in hex\n"
               "      the default card address is 0xd000\n"
               "      the local station and plc station numbers are entered in octal.\n"
               "      maximum length is 120 words\n"
               "Examples:\n"
               "   sbslc (4, 14, 0, 10, 77, 0xE000)\n"
               "   sbslc (4, 7, 150, 4, 10, 0xE000)\n"
               "   sbslc (4, 7, 150, 10, 10, 0xE000)\n"
               "   sbslc (4, 7, 144, 4, 10, 0xE000)\n");
}
