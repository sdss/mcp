/* --------------------------------------------------------------------------
// Copyright (c) 1989-1994 S-S Technologies Inc.
//
// Header File Name: diag.h
// ------------------------
//
// Header file used by 5136-SD demo programs which use the diag.c module.
//
// The structures in this file MUST be compiled using byte aligned, or packed
// structure mode!
//
// --------------------------------------------------------------------------*/

/* structure of reply to "read status" diagnostic command */
typedef struct
{
    uchr    op_stat;
    uchr    mod_id;
    uchr    ext_mod_id1;
    uchr    ext_mod_id2;
    uint    mem_size;
    uint    ctr_addr;
    uchr    ser_rev;
    uchr    op_set;
    char    name[8]; /* node name for terminal                          */
    uchr    pad[40]; /* pad in case node sends back more than we expect */
} DIAG_STS_REPLY;


/* structure for storage of station names, diag counter format etc.     */
typedef struct
{
    char name[7];   /* Name of this station type                        */
    uchr num_ctrs;  /* Number of bytes to read in diag_read command     */
    uchr txt_fmt;   /* Text format number                               */
    uint dta_fmt;   /* Data format number                               */
    uchr nme_prc;   /* Name acquisition procedure                       */
} TYPE_INFO_STUC;

/* diag counter data formats, _WORD is lo/hi, _REVWORD is hi/lo         */
typedef enum {_END,_BYTE,_WORD,_REVWORD} CFMT;
typedef struct
{
    CFMT fmt;       /* data format                                      */
    uchr x;         /* screen location X offset                         */
    uchr y;         /* screen location Y offset                         */
    uchr ofs;       /* offset of data in diag ctr reply                 */
} DIAG_CTR_INFO_STUC;

/* funtion prototypes */
uint get_sta_type (uchr dst, TYPE_INFO_STUC **t);
uint get_sta_name (char * name);
uint get_diag_counters (uchr *dg_ctrs);
uint reset_diag_counters (void);
void get_ctr_str (uint ctr, char *ctr_str, uchr * data);

#ifndef _DIAG
extern char CTR_TEXT[][1215];
extern DIAG_CTR_INFO_STUC DIAG_CTR_INFO[];
#endif
