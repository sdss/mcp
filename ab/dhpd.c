/* --------------------------------------------------------------------------
// Copyright (c) 1989-1995 S-S Technologies Inc.
//
// Program Name: dhpd.c
// --------------------
//
// Demo program which uses diagnostic commands on Data Highway Plus to display
// an active station list, allow monitoring of diagnostic counters, and global
// data.
//
//
// This program was compiled with Borland C++ 3.1
// This module must be linked with STD_DHP.C and DIAG.C
//
// Rev 5.01 jan    94
// Rev 5.02 nov 24 94
//          - fixed node name for plc3 and 5/250 (truncate based on reply length)
// Rev 5.03 Tuesday -  March 28, 1995 at 0102 hrs.
//          - added support for slc 5/04
//          - fixed 1785 KA Diagnostic counter format
// Rev 5.04 - changed STD_DHP.C - increased card status time out to 10 sec's
// Rev 5.05 - port to vxWorks, VME 162, FNAL
// --------------------------------------------------------------------------*/

char VER[]={"5.05"};
#include "vxWorks.h"
#include "taskLib.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"
#include "abdh.h"
#include "std_dhp.h"
#include "diag.h"


/* global variables */
extern ZIF  * zp;          /* ptr to Z80 control structure */
extern uchr loc_sta;           /* local station address        */

void display_diag_counters (uchr node);
void display_diag_counters_all ();
void disp_ed_glob_data(void);
void dhpd_help();


void dhpd(unsigned char loc_station,int card_adr,char *name)
{
    extern int  DHP_baud;
    int  node;
    uint err;
    char term_name[10]={""};
    char node_name[9];
    TYPE_INFO_STUC * type_ptr;
    uint card_seg = 0xd000; /* card memory address          */
extern ZIF  * zp;          /* ptr to Z80 control structure */
extern uchr loc_sta;           /* local station address        */

    strcpy (term_name,name);
    loc_sta=loc_station;
    card_seg=card_adr+0x800;
    zp = (ZIF *) (((ulng)(card_seg) << 4)|0xF0000000);
/*    printf ("zp=%p, local station=%o, local name=%s, MOD_ID=%x",
	zp,loc_sta,name,zp->MOD_ID);*/

    if ((zp->MOD_ID & 0xf0) != DHP_MODULE)
    {
        printf("No Card or DH+ Module not running at %x", card_seg);
        exit(1);
    }

    if (loc_sta > 077)
    {
        printf("Invalid Data Highway Plus Station Address\r\n");
        exit(1);
    }

    start_card (DHP_baud | PFL_DUP | PFL_XDC | PFL_RTN_MSGS);

    strncpy (zp->term_name,term_name,8);

    printf("\r\nData Highway Plus Diagnostic Utility Version %s\r\n"
            "   Copyright (c) 1989-1995 S-S Technologies Inc.\r\n",VER);

    taskDelay(1*60);  /* allow time for card to get ON LINE */

    printf("Active Station List (Who's on)\r\n");

        for (node = 0; node <= 077; node++)
        {
            if (node_active(node))
            {
                printf ("  Node=%02o(Octal) ", node);
                err = get_sta_type (node, &type_ptr);
                if (err == 0)
                {
                    printf("%s", type_ptr->name);
                    get_sta_name (node_name);
                    printf(" %s\r\n", node_name);
/*		    display_diag_counters ((uchr)node);*/
                }
                else
                {
                    printf("****No type/name ");
                }
            }
#ifdef VERBOSE
            else
            {
                    printf("****Not Active   ");
            }
#endif
        }
}

void display_diag_counters_all ()
{
    uchr node;

        for (node = 0; node <= 077; node++)
        {
            if (node_active((uint)node))
            {
		    display_diag_counters (node);
	    }
	}
}
void display_diag_counters (uchr node)
{
    int lp;
    uint err;
    uchr dg_ctr[244];
    char ctr_str[10];
    char node_name[9];
    TYPE_INFO_STUC * type_ptr;

    printf ("Station Diagnostic Counters for Station #%o\r\n",node);

    err = get_sta_type (node, &type_ptr);
    if (err)
    {
        printf ("Error %04x while getting station type\r\n",err);
        return;
    }

    if (type_ptr->name[0] == '?')
    {
        printf ("Unknown station type\r\n");
        return;
    }

    printf(" %s", type_ptr->name);
    get_sta_name (node_name);
    printf(" %s\r\n", node_name);

        err = get_diag_counters (dg_ctr);
        if (err)
        {
            printf ("Error %04x while getting diagnostic counters\r\n",err);
            return;
        }

        for (lp = type_ptr->dta_fmt; DIAG_CTR_INFO[lp].fmt != _END; lp++)
        {
            get_ctr_str (lp,ctr_str,dg_ctr);
            printf ("%s",ctr_str);
        }
}
void dhpd_help()
{
        printf("Copyright (c) 1989-1995 S-S Technologies Inc.\r\n"
                "Data Highway Plus Diagnostic Utility\r\n"
                "Version %s\r\n\n",VER);
        printf("Use : dhpd <loc_station> <card_mem> [<terminal_name>]\r\n"
                "      <card_mem> is entered in hex. Default is D000\r\n"
                "      <loc_station> is entered in octal (0-77)\r\n"
                "Examples:\r\n"
                "DHPD 32, e000,\"name\"\r\n");
}
