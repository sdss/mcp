/* --------------------------------------------------------------------------
// Copyright (c) 1989-1994 S-S Technologies Inc.
//
// Header File Name: std_dhp.h
// ---------------------------
//
// Header file used by 5136-SD demo programs to access functions in the
// standard dh+ module std_dhp.c
//
// --------------------------------------------------------------------------*/

uint dh_msg(void * msg, void * reply, uchr * reply_len , uint tout);
int  rx_avail(int tme);
int  tx_ready(int tme);
int  node_active(uchr node);
void start_card(uchr pc_ifl);
int  get_key(void);
void store_cursor (void);
void cursor_off (void);
void cursor_on (void);
