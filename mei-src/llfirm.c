/*
	llfirm.c - supplement to LLBM.C.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

# include <stdio.h>
# include "idsp.h"

#ifdef __TURBOC__
#  include <conio.h>
#  define   MEI_STD_FILE
#endif

#ifdef __SC__
#  include <dos.h>
#  define   MEI_STD_FILE
#endif

#ifdef __WATCOMC__
#  include <conio.h>
#  define   MEI_STD_FILE
#endif

/* Microsoft under DOS */
#ifndef MEI_WINNT
#  ifdef _MSC_VER
#     include <dos.h>
#     define   MEI_STD_FILE
#  endif
#endif

#ifdef MEI_WIN95
#	define MEI_STD_FILE
#	define HUGE
#endif

#ifdef MEI_WINNT
#  define   MEI_STD_FILE
#  define	HUGE
#endif

#ifdef __QNX__
#  include <conio.h>
#  define   MEI_STD_FILE
#  define	HUGE
#endif

#ifdef MEI_OS2
#  define   MEI_STD_FILE
#  define   HUGE
#endif

#ifdef MEI_OS2WARP
#  define   MEI_STD_FILE
#  define   HUGE
#endif

#ifdef MEI_VW
#  define   MEI_STD_FILE
#  define	HUGE
#endif

#ifdef MEI_OS9
# define MEI_STD_FILE
# define HUGE
#endif

#ifdef MEI_LYNX
# include <stdio.h>
# define MEI_STD_FILE
# define HUGE
#endif

#ifdef MEI_VRTXOS
#	define MEI_STDFILE
#	define HUGE
	extern int16 * mei_dsp_base ;
#	define OUTB(p, b)      ((((int8*) mei_dsp_base)[p]) = (b))
#endif

#ifdef MEI_STD_FILE
#  define OPEN(f, arg)                     fopen(f, arg)
#  define MEIREAD(buf, nelem, size, fp)    fread(buf, nelem, size, fp)
#  define MEIWRITE(buf, nelem, size, fp)   fwrite(buf, nelem, size, fp)
#  define CLOSE(fp)                        fclose(fp)
#endif

#ifndef HUGE
#  define HUGE		huge
#endif

int16 FNTYPE check_firmware(char *buf)
{
	/*
	 *	do a checksum16 check with checksum field 
	 *	in firmware file buffer.
	 */
	unsigned16	sum=0, checkSum;
	int i;


	/* summation of buffer as 16bit integers */
	for(i = BM_PAGE_0; i < (BM_PAGE + BM_PAGE_0); i++) {
		sum += (unsigned16)(buf[i]) & 0xFF;
	}
	/* skip the checksum page */
	for(i = BM_PAGE_1; i < BM_CHECKSUM; i++) {
		sum += (unsigned16)(buf[i]) & 0xFF;
	}
	/* get the checksum value in buffer */
	checkSum = ((unsigned16)buf[BM_CHECKSUM]) & 0xFF;
	checkSum |= ((unsigned16)buf[BM_CHECKSUM + 1]  & 0xFF) << 8;

	/* compair */
	if(sum != checkSum)
		return(dsp_error = DSP_FIRMWARE_CHECKSUM);

	return dsp_error;
}

int16 FNTYPE upload_firmware_file(char *file)
{	
	FILE *fp;

	/* define buffer size */
	#ifdef  MEI_QC25
		static unsigned char _huge    *bm_buf;
		bm_buf = halloc(BM_SIZE, 1);
	#else
		static unsigned char HUGE     bm_buf[BM_SIZE];
	#endif
	
	/* upload firmware from controller */
	if(pcdsp_upload_firmware((char*)bm_buf))
		return (dsp_error);

	/* write buffer to file */
	if((fp = OPEN(file,"wb")) == NULL)
		return (dsp_error = DSP_BAD_FIRMWARE_FILE);
	MEIWRITE(bm_buf,1,BM_SIZE,fp);
	CLOSE(fp);

	return dsp_error;
}

int16 FNTYPE pcdsp_upload_firmware(char *bm_buf)
{

	/* load firmware on controller to buffer */
	if(dsp_read_bm(dspPtr,BM_PAGE_0,BM_SIZE,bm_buf)) 
		return(dsp_error);
	
	/* perform Checksum16 on firmware in buffer */
	check_firmware(bm_buf);
	
	return dsp_error;
}

int16 FNTYPE download_firmware_file(char *file)
{
	FILE *fp;

/* define buffer size */
	#ifdef  MEI_QC25
		static unsigned char _huge    *bm_buf;
		bm_buf = halloc(BM_SIZE, 1);
	#else
		static unsigned char HUGE     bm_buf[BM_SIZE];
	#endif

	/* open firmware file */
	if((fp = OPEN(file,"rb")) == NULL)
		return (dsp_error = DSP_BAD_FIRMWARE_FILE);
	/* read the '.abs' file to buffer */ 
	if(MEIREAD(bm_buf, 1, BM_SIZE, fp) != BM_SIZE)		
		return (dsp_error = DSP_BAD_FIRMWARE_FILE);
	/* close firmware file */
	CLOSE(fp);

	/* download buffer to controller */
	pcdsp_load_firmware((char *)bm_buf);
		
	return dsp_error;
}

int16 FNTYPE pcdsp_load_firmware(char *bm_buf)
{
	dsp_error = DSP_OK;

	/* perform Checksum16 before writing */
	if(check_firmware(bm_buf))
		return(dsp_error);

	/* put DSP into a defined state */
	DSP_OUTB(dspPtr->reset,0x0);	

	/* write the buffer to boot memory */
	dsp_write_bm(dspPtr,BM_PAGE_0,BM_PAGE,&bm_buf[BM_PAGE_0]);
	dsp_write_bm(dspPtr,BM_PAGE_1,BM_PAGE,&bm_buf[BM_PAGE_1]);

	/* reboot controller with new firmware */
	dsp_rset();
			
	return(dsp_error);
	
}


