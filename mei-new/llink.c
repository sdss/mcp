/*
	llink.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include "idsp.h"


typedef struct
{	DSP_DM		frac ;
	DSP_DM		whole ;
} LIM32 ;


int16 FNTYPE pcdsp_set_link(PDSP dsp, int16 axis, P_DSP_DM master_addr, PFIXED ratio)
{
	P_DSP_DM
		ds = dsp->data_struct + DS(axis) ;

    DSP_DM ms_control_block[3];
    
    ms_control_block[0] = master_addr;
    ms_control_block[1] = (DSP_DM) HIGH_16(ratio->frac) ;
    ms_control_block[2] = (DSP_DM) LOW_16(ratio->whole) ;

	pcdsp_transfer_block(dsp, FALSE, FALSE, (P_DSP_DM)(ds + DS_MASTER_POS),
        3, ms_control_block);

	return dsp_error ;
}

int16 FNTYPE pcdsp_endlink(PDSP pdsp, int16 slave)
{
   int16         i;
   DSP_DM      buffer[10], control_buffer[3], command_buffer[4];
   P_DSP_DM    control_addr = pdsp->data_struct + DS(slave) + DS_RATIO + 2;
   P_DSP_DM    command_addr = pdsp->data_struct + DS(slave) + DS_POSITION;
   LFIXED      command, control;

   pcdsp_transfer_block(pdsp, TRUE, FALSE, control_addr, 3, control_buffer);
   pcdsp_transfer_block(pdsp, TRUE, FALSE, command_addr, 4, command_buffer);

#ifdef MEI_O_BENDIAN
   control.frac = ((unsigned32)control_buffer[0]) & 0x0000FFFFL; 
   control.whole = (((unsigned32)control_buffer[1]) << 16) & 0xFFFF0000L;
   control.whole |= ((unsigned32)control_buffer[2] & 0x0000FFFFL);
   command.frac = ((unsigned32)command_buffer[1]) & 0x0000FFFFL; 
   command.frac |= ((((unsigned32)command_buffer[0]) << 16) & 0xFFFF0000L); 
   command.whole = (((unsigned32)command_buffer[2]) << 16) & 0xFFFF0000L;
   command.whole |= ((unsigned32)command_buffer[3] & 0x0000FFFFL);
#else
   control.frac = (((unsigned32)control_buffer[0]) << 16) & 0xFFFF0000L; 
   control.whole = (((unsigned32)control_buffer[2]) << 16) & 0xFFFF0000L;
   control.whole |= ((unsigned32)control_buffer[1] & 0x0000FFFFL);
   command.frac = ((unsigned32)command_buffer[0]) & 0x0000FFFFL; 
   command.frac |= ((((unsigned32)command_buffer[1]) << 16) & 0xFFFF0000L); 
   command.whole = (((unsigned32)command_buffer[3]) << 16) & 0xFFFF0000L;
   command.whole |= ((unsigned32)command_buffer[2] & 0x0000FFFFL);
#endif

   command.frac += control.frac;
   command.whole += control.whole;

#ifdef MEI_O_BENDIAN
   buffer[1] = (DSP_DM)(command.frac & 0x0000FFFFL);
   buffer[0] = (DSP_DM)((command.frac & 0xFFFF0000L) >> 16);
   buffer[3] = (DSP_DM)(command.whole & 0x0000FFFFL);
   buffer[2] = (DSP_DM)((command.whole & 0xFFFF0000L) >> 16);
#else
   buffer[0] = (DSP_DM)(command.frac & 0x0000FFFFL);
   buffer[1] = (DSP_DM)((command.frac & 0xFFFF0000L) >> 16);
   buffer[2] = (DSP_DM)(command.whole & 0x0000FFFFL);
   buffer[3] = (DSP_DM)((command.whole & 0xFFFF0000L) >> 16);
#endif
   for(i = 4; i < 10; i++)
      buffer[i] = 0;
   return pcdsp_transfer_block(pdsp, FALSE, FALSE, command_addr, 10, buffer);
}


