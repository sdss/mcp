/*
	llbm.c
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

#  ifdef __TURBOC__
#	include <dos.h>
#	endif


static int16 LOCAL_FN write_bm(PDSP pdsp, P_DSP_DM addr, DSP_DM byte, int16 crit)
{
	if (crit)
	{	CRITICAL;
	}

	DSP_OUT(pdsp->address, (unsigned16)(addr | PCDSP_BM));
	DSP_OUT(pdsp->data, (unsigned16)(byte & 0xFF));

	if (crit)
	{	ENDCRITICAL;
	}
	return DSP_OK ;
}

static DSP_DM LOCAL_FN read_bm(PDSP pdsp, P_DSP_DM addr, int16 crit)
{
	DSP_DM r;

	if (crit)
	{	CRITICAL;
	}

	DSP_OUT(pdsp->address, (unsigned16)(addr | PCDSP_BM)) ;
	r = DSP_IN(pdsp->data) & 0xFF ;

	if (crit)
	{	ENDCRITICAL;
	}
	return r;
}

int16 FNTYPE pcdsp_read_bm(PDSP dsp, P_DSP_DM addr, unsigned len, DSP_DM PTRTYPE * dest)
{	unsigned i;
	int16 r = DSP_OK ;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	addr *= 4;			/* bytes to 24-bit words. */

	for (i = 0; i < len; i++)
	{	r = read_bm(dsp, addr++, 1) << 8;
		r |= read_bm(dsp, addr++, 1) ;
		addr += 2;		/* skip the two extra bytes. */

		if (dest)
			dest[i] = r ;
	}

	if ((len != 1) || (dest))
		r = DSP_OK ;

	return r ;
}

static int16 LOCAL_FN ncw_bm(PDSP pdsp, P_DSP_DM addr, DSP_DM byte, int16 task)
{
   switch(task)
   {
      case 0:
         write_bm(pdsp, addr, (DSP_DM)((byte >> 8) & 0xFF), 0);
         break;

      case 1:
    	   write_bm(pdsp, addr, (DSP_DM)(byte & 0xFF), 0);
         break;

      case 2:
	      write_bm(pdsp, addr, 0, 0);
         break;

      case 3:
         break;   /* skip the extra byte. */
   }

   return DSP_OK;
}

int16 FNTYPE pcdsp_write_bm(PDSP dsp, P_DSP_DM addr, unsigned len, DSP_DM PTRTYPE * src)
{	unsigned16 i;
	int16 j, byte, bytelimit, last_byte_written;
	P_DSP_DM a;
	char s;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	CRITICAL;

   	addr *= 4;			/* bytes to 24-bit words. */

	   i = 0;
      while(i < len)
      {
      /* if not on a 32 byte boundary then write the data up to next boundary.*/

         bytelimit = 32 - (addr % 32);
         byte = 0;
         while((byte < bytelimit) && (i < len))
         {
            ncw_bm(dsp, addr, src[i], (int16)(byte % 4));
            if((byte % 4) == 3)
               i++;
            byte++;
            addr++;
         }
         last_byte_written = (byte - 1) % 4;

         if(last_byte_written == 3)
            j = i - 1;
         else
            j = i;
			a = addr - last_byte_written - 1;
			s = (src[j] >> 8) & 0xFF;
         while((char)read_bm(dsp, a, 0) != s);
      }

	ENDCRITICAL;

	return DSP_OK ;
}

int16 FNTYPE dsp_read_bm(PDSP dsp, P_DSP_DM addr, unsigned len, P_CHAR dest)
{
	unsigned i ;

	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	for (i = 0; i < len; i++)
		dest[i] = (char)(read_bm(dsp, (P_DSP_DM)(addr + i), 1) & 0x00FF);

	return dsp_error ;
}

int16 FNTYPE dsp_write_bm(PDSP dsp, P_DSP_DM addr, unsigned len, P_CHAR src)
{	unsigned16 i;
	int16 byte, bytelimit;
	P_DSP_DM a;
	char s;

  	if (! dsp)
   		return (dsp_error = DSP_NOT_INITIALIZED) ;

	CRITICAL;

      i = 0;
   	while(i < len)
      {
      /* if not on a 32 byte boundary then write the data up to next boundary.*/

         bytelimit = 32 - ((addr + i) % 32);
         byte = 0;
         while((byte < bytelimit) && (i < len))
         {
   		   write_bm(dsp, (P_DSP_DM)(addr + i), (DSP_DM) (src[i]), 0);
            i++;
            byte++;
         }
			a = addr + i - 1;
			s = src[i - 1];
         while((char)read_bm(dsp, a, 0) != s);
      }

	ENDCRITICAL;

	return DSP_OK ;
}

unsigned16 FNTYPE pcdsp_get_checksum(PDSP pdsp)
{	unsigned16
		sum = 0;

	sum = (unsigned16) read_bm(pdsp, BM_CHECKSUM, 1);
	sum |= ((unsigned16) read_bm(pdsp, BM_CHECKSUM + 1, 1) << 8);

	return sum;
}

unsigned16 FNTYPE pcdsp_checksum(PDSP pdsp)
{	P_DSP_DM i;
	unsigned16 sum = 0;

	for(i = BM_PAGE_0; i < (BM_PAGE+BM_PAGE_0); i++)
		sum += read_bm(pdsp, i, 1);
	for(i = BM_PAGE_1; i < BM_CHECKSUM; i++)
		sum += read_bm(pdsp, i, 1);
	return sum;
}

int16 FNTYPE pcdsp_write_checksum(PDSP pdsp, unsigned16 sum, int16 save)
{	if(!pdsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	write_bm(pdsp, BM_CHECKSUM, (DSP_DM)(sum & 0xFF), 1);
	write_bm(pdsp, BM_CHECKSUM + 1, (DSP_DM)((sum >> 8) & 0xFF), 1);

	if(save)
		return pcdsp_save(pdsp);
	else
		return DSP_OK;
}

int16 FNTYPE pcdsp_save(PDSP pdsp)
{
	int16 i, timeout = 1 ;
	DSP_DM zero = 0;

	if (! pdsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	/* non-volatile memory save */
	read_bm(pdsp, 0x0000, 1);
	read_bm(pdsp, 0x2555, 1);
	read_bm(pdsp, 0x0AAA, 1);
	read_bm(pdsp, 0x2FFF, 1);
	read_bm(pdsp, 0x20F0, 1);
	read_bm(pdsp, 0x0F0F, 1);

	for (i = 0; timeout && (i < 10); i++)		/* delay for NVRAM store */
	{	idsp_write_dm(pdsp, 0x100, zero);
		timeout = TIMEOUT ;
	 	while (timeout && (idsp_read_dm(pdsp, 0x100) != 0x641C))
	 		timeout-- ;
	}

	return DSP_OK ;
}

int16 FNTYPE pcdsp_check(PDSP pdsp)
{
	if (pcdsp_get_checksum(pdsp) != pcdsp_checksum(pdsp))
		return (dsp_error = DSP_CHECKSUM) ;

	return DSP_OK ;
}


