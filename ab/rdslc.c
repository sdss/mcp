/* --------------------------------------------------------------------------
// Copyright (c) 1989-1994 S-S Technologies Inc.
//
// Program Name: rdslc.c
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

void rdslc(int dst, uint file, uint ofs, int len, 
		unsigned char loc_station, int card_adr, char *name);
void slc_data_pool_init(unsigned char loc_station, int card_adr, char *name);
void slc_data_pool_set(int idx, uint freq, int dst, uint file, uint typ, 
		uint ofs, int len);
void slc_data_pool();
void rdslc_help();

/* global variables */
struct SLC_DP {
	uint freq;
	uint freq_cnt;
	uint cnt;
	int dst;
	uint file;
	uint typ;
	uint ofs;
	uint len;
	uint *data;
};
struct SLC_DP slc_dp[10];

void rdslc(int dst, uint file, uint ofs, int len, 
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

    printf("Sending SLC Block Read messages\r\n");

    start_card (DHP_baud | PFL_DUP | PFL_XDC);

    strncpy(zp->term_name,term_name,8);

    taskDelay(1*60);  /* allow time for card to get ON LINE */

    while (cycle++ < 0x7f) 
    {
        printf("\r\nCycle # %04x",cycle);
            err = slc_read_blok(dst, file, INT_FILE, ofs, data, len); /* read values from SLC */

            if (err)
            {
                printf ("R Err=%04x\r\n",err);
            }
            else
            {                       /* check that values read back match    */
                for (lp = 0; lp < len; lp++)
                {
		  if ((lp%10)==0)
		    printf ("\r\ndata %d-%d: ",lp,lp+10);
                  printf("%04x ",data[lp]);
                }
            }
    }
    zp->PC_TXC = IFT_IRS;           /* Take card off line                   */
}

void rdslc_help()
{
        printf("USE:\r\n"
               "rdslc <plc sta addr> <slc_file_num> <slc_file_ofs> <length> <card sta>\n"
               "      [<card addr>] [<term_name>]\n"
               "      the optional card address is entered in hex\n"
               "      the default card address is 0xd000\n"
               "      the local station and plc station numbers are entered in octal.\n"
               "      maximum length is 120 words\n"
               "Examples:\n"
               "   rdslc (4, 14, 0, 10, 77, 0xE000)\n"
               "   rdslc (4, 7, 0, 110, 10, 0xE000)\n"
               "   rdslc (4, 7, 110, 50, 10, 0xE000)\n"
               "   rdslc (4, 7, 144, 4, 10, 0xE000)\n");
}
