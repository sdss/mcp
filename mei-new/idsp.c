/*
	idsp.c
*/

static char
	COPYRIGHT[] = "Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.  This software  contains proprietary and confidential information  of Motion Engineering Inc., and its suppliers.  Except as may be set forth in the license agreement under which  this software is supplied, use, disclosure, or reproduction is prohibited without the prior express written	consent of Motion Engineering, Inc."; 

#include "idsp.h"
#include "sercrset.h"

/* Section 1: Global variables */
int16 MEIKEY;
int16 DATATYPE
	dsp_error ;

static int16
	_dsp_initd = FALSE ;

LFIXED DATATYPE
	fixed_zero ;
/* End of Section 1 */


/* Section 2: Defines and functions for basic I/O operations.  This section is 
entered unless one of the following MEI_xxx is defined specifying either an odd
Operating System or a specific Hardware Platform. */
#ifndef MEI_ASM      /* Assembly language module used for I/O */
#ifndef MEI_EPC0     /* Uses EPCONNECT for Radisys platform */
#ifndef MEI_EPC1     /* Uses EPCONNECT for Radisys platform */
#ifndef MEI_XVME     /* Uses Xycom platform */
#ifndef MEI_WINNT    /* Uses Windows NT calls to DeviceIoControl to access DD */
#ifndef MEI_OS9		/* OS9 */
#ifndef MEI_OS9000	/* OS9000 */
#ifndef MEI_VW       /* VxWorks */
#ifndef MEI_OS2      /* OS/2 */
#ifndef MEI_OS2WARP	/* OS/2 Warp*/
#ifndef MEI_VRTXOS	/* VRTX/OS */
#ifndef DSP_MACROS   /* Macros for DSP_IN etc... defined as inport() etc... */
#ifndef MEI_VXI		/* VXI/VME */

#  ifdef __TURBOC__
#     include <conio.h>
#  endif

#  ifdef __WATCOMC__
#  	include <conio.h>
#	endif

#  ifdef __SC__
#	   include <dos.h>
#	endif

/* Microsoft under DOS */
#	ifndef MEI_MSOS
#		ifdef _MSC_VER
#			include <dos.h>
#		endif
#	else
#		ifdef MEI_WIN95
#			include <conio.h>
#		endif
#	endif

#  ifdef __QNX__
#  	include <conio.h>
#	endif

unsigned16 FNTYPE DSP_IN(int16 pp)
{  return inpw(pp);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  outpw(pp, vv) ;
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  return inp(pp);
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  outp(pp, vv);
}
int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{  return pcdsp_set_address(pdsp, iobase);
}
#endif /* !MEI_VXI */
#endif /* !DSP_MACROS */
#endif /* !MEI_VRTXOS */
#endif /* !MEI_OS2WARP */
#endif /* !MEI_OS2 */
#endif /* !MEI_VW */
#endif /* !MEI_OS9000 */
#endif /* !MEI_OS9 */
#endif /* !MEI_WINNT */
#endif /* !MEI_XVME */
#endif /* !MEI_EPC1 */
#endif /* !MEI_EPC0 */
#endif /* !MEI_ASM */
/* End of section 2 */


/* Section 3: I/O routines for specific Op Sys' or Hardware platforms */
/* Section 3a: Windows NT */
#ifndef MEI_VMIC
#	ifdef MEI_WINNT
#	include <stdarg.h>
#	include <winbase.h>
#	include <winioctl.h>
#	include <winreg.h>
#	include <stdlib.h>

int16 dsp_io_problem = TRUE ;

#ifndef MEI_EPC1

LPCSTR device_name[MaxBoards] = {	"\\\\.\\dspio0",
									"\\\\.\\dspio1",
									"\\\\.\\dspio2",
									"\\\\.\\dspio3",
									"\\\\.\\dspio4",
									"\\\\.\\dspio5",
									"\\\\.\\dspio6",
									"\\\\.\\dspio7" };


int16 FNTYPE dsp_open(void)
{  int16 b = 0;
   dsp_io_problem = 0;
   dspPtr->dsp_file = CreateFile(device_name[dspPtr->board], GENERIC_WRITE|GENERIC_READ,
					FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

   if (dspPtr->dsp_file == INVALID_HANDLE_VALUE)        /* Was the device opened? */
      dsp_io_problem = 1;
   else
      dd_call(IOCTL_PCDSP_INIT, &b);
   return dsp_io_problem;
}

unsigned16 FNTYPE DSP_IN(int16 pp)
{	DWORD rl;
	USHORT r;
	DeviceIoControl(dspPtr->dsp_file, (ULONG)IOCTL_PCDSP_READ_PORT_USHORT,
      &pp, (ULONG) sizeof(pp), &r, (ULONG) sizeof(r), &rl, NULL);
	return r;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	ULONG rl;
	WRITE_INPUT iobuffer;
	iobuffer.PortNumber = pp;
	iobuffer.ShortData = vv;
   DeviceIoControl(dspPtr->dsp_file, (ULONG)IOCTL_PCDSP_WRITE_PORT_USHORT,
      &iobuffer, (ULONG) sizeof(iobuffer), NULL, 0, &rl, NULL);
}

unsigned char FNTYPE DSP_INB(int16 port)
{	DWORD rl;
	UCHAR r;
	DeviceIoControl(dspPtr->dsp_file, (ULONG)IOCTL_PCDSP_READ_PORT_UCHAR,
      &port, (ULONG) sizeof(port), &r, (ULONG) sizeof(r), &rl, NULL);
	return r;
}

void FNTYPE DSP_OUTB(int16 port, unsigned char datum)
{  ULONG rl;
   WRITE_INPUT_CHAR iobuffer ;
   iobuffer.PortNumber = port ;
   iobuffer.CharData = datum ;
   DeviceIoControl(dspPtr->dsp_file, (ULONG)IOCTL_PCDSP_WRITE_PORT_UCHAR,
      &iobuffer, (ULONG) sizeof(iobuffer), NULL, 0, &rl, NULL);
}

char *sPortAddress[8] = {"IoPortAddress0",
						"IoPortAddress1",
						"IoPortAddress2",
						"IoPortAddress3",
						"IoPortAddress4",
						"IoPortAddress5",
						"IoPortAddress6",
						"IoPortAddress7"};


int16 LOCAL_FN check_registry(PDSP pdsp, unsigned16 iobase)
{	HKEY hKey;
	DWORD type, size = 4;
	union {
		long l;
		char b[4];
	} data;
	long IoNumberOfBoards, board;

	pdsp->board = -1;

	if(RegOpenKeyEx(HKEY_LOCAL_MACHINE,"SYSTEM\\CurrentControlSet\\Services\\DSPIO\\Parameters", 0, KEY_QUERY_VALUE, &hKey))
	{	if(hKey != NULL)
			CloseHandle(hKey);
		return (dsp_error = DSP_NT_DRIVER);
	}


	if(RegQueryValueEx(hKey, "IoNumberOfBoards", NULL, &type, data.b, &size))
	{	CloseHandle(hKey);
		return (dsp_error = DSP_NT_DRIVER);
	}
		
	IoNumberOfBoards = data.l;

	for(board = 0; board < IoNumberOfBoards; board++)
	{	size = 4;
		RegQueryValueEx(hKey, sPortAddress[board], NULL, &type, data.b, &size);
		if((unsigned16)(data.l) == iobase)
			pdsp->board = (int16)board;
	}
	if(pdsp->board == -1)
		dsp_error = DSP_NT_DRIVER;
	else
		dsp_error = DSP_OK;

	CloseHandle(hKey);

	return dsp_error;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
	int16 e;

	if(check_registry(pdsp, iobase))
		return dsp_error;

	dsp_open();

	if(dsp_io_problem)
		return (dsp_error = DSP_NT_DRIVER);

/*	atexit(dsp_close); */

	e = pcdsp_set_address(pdsp, 0x0);	/* address is set to 0x0 */
	pdsp->iobase = iobase;				/* iobase is set to iobase */
	return e;
}

int16 FNTYPE mei_sleep_until_interrupt(HANDLE file_handle, USHORT Axis, USHORT InterruptType)
{
	INTERRUPT_DATA	Data;
	ULONG			rl;

	if(pcdsp_sick(dspPtr, (int16)Axis))
		return dsp_error;

	Data.InterruptType = InterruptType;
	Data.Axis = Axis;
	return DeviceIoControl(file_handle, (ULONG)IOCTL_PCDSP_WAIT,
		&Data, (ULONG) sizeof(Data), NULL, 0, &rl, NULL);
} 

int16 FNTYPE get_data_from_driver(int16 address, int16 data_offset, int16 datum_len, 
	int16 pieces_of_data, int16 *data)
{	DWORD rl;
	GET_DATA_DATA gdd;
	gdd.addr = address;
	gdd.len = pieces_of_data;
	gdd.datum_len = datum_len;
	gdd.offset = data_offset;
	return DeviceIoControl(dspPtr->dsp_file, (ULONG)IOCTL_GET_DATA,
		&gdd, sizeof(GET_DATA_DATA), data, (pieces_of_data*datum_len)*sizeof(USHORT), &rl, NULL);
}

#	endif	/* !MEI_EPC1 */

void __cdecl dsp_close(void)
{
	CloseHandle(dspPtr->dsp_file);
}

#	else        
int16 dsp_io_problem = FALSE ;
#	endif /* MEI_WINNT */
#endif /* MEI_VMIC */
/* End of setion 3a: Windows NT */


/* Section 3b: Xycom */
#ifdef MEI_XVME
#  include "xvmedefs.h"
#  include "vmeext.h"
#  define MEI_SEP_RESET

unsigned16 FNTYPE DSP_IN(int16 pp)
{  unsigned16 value;

   ReadVMEBusMemoryRM((char *)&value, TRANSFER16, LITTLEENDIAN, 2, SHORT_IO_ACCESS,
      (long)pp, !FALSE);
   return(value);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  WriteVMEBusMemoryRM((char *)&vv, TRANSFER16, LITTLEENDIAN, 2, SHORT_IO_ACCESS,
      (long)pp, !FALSE);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  unsigned8   value;

   ReadVMEBusMemoryRM((char *)&value, TRANSFER8, LITTLEENDIAN, 1, SHORT_IO_ACCESS,
      (long)pp, !FALSE);
   return(value);
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  WriteVMEBusMemoryRM((char *)&vv, TRANSFER8, LITTLEENDIAN, 1, SHORT_IO_ACCESS,
      (long)pp, !FALSE);
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;

   AutoInitLib();    /* Initialize XVME library */
   Set_RM_Window(SHORT_IO_ACCESS, 0);     /* Map Real Mode memory window */

	if(dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#endif /* MEI_XVME */
/* End of Section 3b: Xycom */


/* Section 3c: EPCONNECT  Type 0 */
#ifdef MEI_EPC0
#  include "busmgr.h"
#  define MEI_SEP_RESET

unsigned16 FNTYPE DSP_IN(int16 pp)
{
   unsigned16 value;

   EpcFromVmeAm(A16S | BM_MBO, BM_W16 | BM_FASTCOPY, (long)pp, (char *)&value, 2);
   return(value);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  EpcToVmeAm(A16S | BM_MBO, BM_W16 | BM_FASTCOPY, (char *)&vv, (long)pp, 2);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  unsigned8   value;

   EpcFromVmeAm(A16S | BM_MBO, BM_W8 | BM_FASTCOPY, (long)pp, (char *)&value, 1);
   return(value);
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  EpcToVmeAm(A16S | BM_MBO, BM_W8 | BM_FASTCOPY, (char *)&vv, (long)pp, 1);
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;

   if (EpcCkBm() < 0)
   {
	   return (dsp_error = DSP_NOT_FOUND);
	}
   else    /* test access mode */
   {
      int16 saved_mode, r;
	   saved_mode = EpcGetAccMode();
	   r = EpcSetAccMode(A16S | BM_MBO) ;
      if(r != EPC_SUCCESS)
      {
		   EpcSetAccMode(saved_mode);
     	   return (dsp_error = DSP_NOT_FOUND);
      }
		EpcSetAccMode(saved_mode);
   }

	if (dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#  endif   /* MEI_EPC0 */
/* End of Section 3c: EPCONNECT Type 0 */


/* Section 3d: EPCONNECT Type 1 */
#ifdef MEI_EPC1
#	ifndef MEI_WINNT
#  	include "busmgr.h"
#  	define MEI_SEP_RESET
unsigned long mei_session_id;
int16 mei_session_opened = 0;

void mei_close_session(void)
{
	if (mei_session_opened)
		EpcCloseSession(mei_session_id);
	mei_session_opened = 0;
}

unsigned16 FNTYPE DSP_IN(int16 pp)
{  unsigned16 value;
   pp /= 2;
	value = (unsigned16) (dspPtr->mei_dsp_base[pp]);
   return(value);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	pp /= 2;
	dspPtr->mei_dsp_base[pp] = (int16) vv;
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  unsigned char value;
   value = (unsigned char) (dspPtr->mei_dsp_base[pp]);
   return value;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  char huge volatile * c = (char huge volatile *) dspPtr->mei_dsp_base;
   c[pp] = (char)vv;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;
	struct EpcEnvironment environment ;

	/* Check to make sure that the VMEbus environment is loaded... */
	if (EpcVerifyEnvironment(&environment) != EPC_SUCCESS)
		return (dsp_error = DSP_NOT_FOUND);
	if (!mei_session_opened)
	{	if (EpcOpenSession(&mei_session_id) != EPC_SUCCESS)
			return (dsp_error = DSP_NOT_FOUND);
		mei_session_opened = 1 ;
	}
	atexit(mei_close_session) ;
	/* get a pointer to VME A16 bus space. */
	{	void volatile huge * p ;
		if (EpcMapBusMemory(mei_session_id,
								EPC_A16S,
								EPC_MBO,
								0x00000000,
								0x00010000, &p) != EPC_SUCCESS)
			return (dsp_error = DSP_NOT_FOUND) ;
		pdsp->mei_dsp_base = (int16 volatile huge *) p;
	}
	/* So now mei_dsp_base points to the beginning of A16 space. */

	if (dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#	else	/* MEI_WINNT */
/* Windows NT for VME Bus */
#include "vmemap.h"
#  define MEI_SEP_RESET
int16 mei_session_opened = 0;
char * mem_base;

int16 FNTYPE dsp_open(void)
{	int16 b = 0;
	if(mei_session_opened == 0)
	{
		dsp_io_problem = 0;
		dspPtr->dsp_file = CreateFile("\\\\.\\VMEMAP", GENERIC_WRITE|GENERIC_READ,
						0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

		if(dspPtr->dsp_file == INVALID_HANDLE_VALUE)        /* Was the device opened? */
			dsp_io_problem = 1;
		if(!dsp_io_problem)
		{
			PHYSICAL_MEMORY_INFO pmi;
			DWORD cbReturned;

		  	mei_session_opened = 1;
/*			atexit(dsp_close); */
			pmi.InterfaceType = 1;
			pmi.BusNumber = 0;
#ifndef MEI_WINNT_BC
			pmi.BusAddress.HighPart = 0;
			pmi.BusAddress.LowPart = 0xC0000000;	/* A16 Motorola */
#else
			pmi.BusAddress.u.HighPart = 0;
			pmi.BusAddress.u.LowPart = 0xC0000000;
#endif
			pmi.AddressSpace = 0;
			pmi.Length = 0xFFFF;					/* all 64K */
			
			if(!DeviceIoControl (dspPtr->dsp_file,
					(DWORD) IOCTL_VMEMAP_MAP_USER_PHYSICAL_MEMORY,
					&pmi,
					sizeof(PHYSICAL_MEMORY_INFO),
					&((PVOID)mem_base),
					sizeof(PVOID),
					&cbReturned,
					0))
				dsp_io_problem = 1;
			if(!mem_base)
				dsp_io_problem = 1;
		}
	}
			
	return dsp_io_problem ;
}

unsigned16 FNTYPE DSP_IN(int16 pp)
{  unsigned16 value;
   pp /= 2;
	value = (unsigned16) (dspPtr->mei_dsp_base[pp]);
   return(value);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	pp /= 2;
	dspPtr->mei_dsp_base[pp] = (int16) vv;
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  char  * c = (char  *) dspPtr->mei_dsp_base;
   unsigned char value;
   value = c[pp];
   return value;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  char  * c = (char  *) dspPtr->mei_dsp_base;
   c[pp] = (char)vv;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
	dsp_open();

	if(dsp_io_problem)
	{	mem_base = NULL;
		dspPtr->mei_dsp_base = NULL;
		return (dsp_error = DSP_NOT_INITIALIZED);
	}

	dspPtr->mei_dsp_base = (int16 *)mem_base;
	return pcdsp_set_address(pdsp, iobase);
}
#	endif	/* MEI_WINNT */
#endif   /* MEI_EPC1 */
/* End of Section 3d: EPCONNECT Type 1 */


/* Section 3e: */
#ifdef MEI_VW
#	ifdef MEI_IOMAPPED
#		define MEI_SEP_RESET

extern unsigned16		sysInWord(int16 pp);
extern void				sysOutWord(int16 pp, unsigned16 vv);
extern unsigned char	sysInByte(int16 pp);
extern void				sysOutByte(int16 pp, unsigned char vv);


unsigned16 FNTYPE DSP_IN(int16 pp)
{	return sysInWord(pp);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	sysOutWord(pp, vv);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{	return sysInByte(pp);
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{	sysOutByte(pp, vv) ;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{  return pcdsp_set_address(pdsp, iobase);
}

#  else
#  define	MEI_SEP_RESET
#  ifndef A16_BASE
/* Motorola MVME 162 CPU */
#     define	A16_BASE			(0xFFFF0000)
/* GMS CPU */
/* #     define	A16_BASE			(0xF1000000) */
/* Synergy CPU 68080 */
/* #     define	A16_BASE			(0xFF800000) */
#  endif

unsigned16 FNTYPE DSP_IN(int16 pp)
{  return (unsigned16)(dspPtr->mei_dsp_base[pp/2]);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  unsigned16 * c = (unsigned16 *) dspPtr->mei_dsp_base;
   c[pp/2] = vv;
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  return (unsigned char)(dspPtr->mei_dsp_base[pp]);
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  unsigned char * c = (unsigned char *) dspPtr->mei_dsp_base;
   c[pp] = vv;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;
	pdsp->mei_dsp_base = (int16*) (A16_BASE);

	if (dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#  endif /* MEI_IOMAPPED */
#endif /*MEI_VW */
/* End of Section 3e: */


/* Section 3f: OS9 */
#ifdef MEI_OS9
#  define	MEI_SEP_RESET
#  ifndef A16_BASE
#     define	A16_BASE 0xFFFF0000
#  endif
#  define	PM_READ				1
#  define	PM_WRITE          2
#  define	PM_EXECUTE			4
#  define	PM_ACCESS			(PM_READ | PM_WRITE)
#  include <process.h>
#  include <stdlib.h>

int16 getpid(void);

int16 * MeiMemoryPointer(int16 A16Start, int16 * ioproblem)
{	int16 pid ;
	int16 * p = (int16*) (A16_BASE | A16Start);
	pid = getpid() ;
	_os_permit(p, 16, PM_ACCESS, pid);
	return p;
}

unsigned16 FNTYPE DSP_IN(int16 pp)
{  return (unsigned16)((dspPtr->mei_dsp_base)[pp/2]);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  unsigned16 * c = (unsigned16 *) (dspPtr->mei_dsp_base);
   c[pp/2] = vv;
}

/* NOTE: DSP_INB can only be used to access 8 bit port 2,
otherwise, a BUS FAULT will occur.  It is probably better to use DSP_IN()
and mask the bits to get the 8 bit value. */
unsigned char FNTYPE DSP_INB(int16 pp)
{  unsigned char x;
   int16 y;
   y = (dspPtr->mei_dsp_base)[pp];
   x = (unsigned char)(y & 0x00FF);
   return x;
}

/* NOTE: use only to write to reset ports (addr + 0x4 and addr + 0x5) */
void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  unsigned char * c = (unsigned char *) (dspPtr->mei_dsp_base);
   c[pp] = vv;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
	int16 e;
	pdsp->mei_dsp_base = MeiMemoryPointer(iobase, &dsp_io_problem);

	if (dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, 0);
	pdsp->iobase = iobase;
	return e;
}
#endif
/* End of Section 3f: OS9 */


/* Section 3g: OS/2 */
#ifdef MEI_OS2
   extern int16 _Far16 plugh1(int16);
   extern void _Far16 plugh2(int16, int16);

unsigned16 FNTYPE DSP_IN(int16 pp)
{  unsigned16 r;
	r = plugh1(pp);
	r |= (plugh1(pp + 1) << 8);
	return r;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  plugh2(pp, vv);
	plugh2(pp + 1, vv >> 8);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  unsigned char r;
   r = plugh1(pp);
   return r;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  plugh2(pp, vv);
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;

	if(dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#endif
/* End of Section 3g: OS/2 */


/* Section 3h: OS/2 Warp 	
	NOTE: uses generic I/O device driver TESTCFG.SYS distributed with 
	operating system. */
#ifdef MEI_OS2WARP
#define INCL_NOPMAPI		/* to avoid multiple declatations of PFIXED */
#define INCL_DOSFILEMGR
#define INCL_DOSDEVICES
#include <os2.h>
#include <stdio.h>
#include <stdlib.h>
typedef struct{
  unsigned short address;
  unsigned short width;
  unsigned long IOData;
} ODescrpt_s;

typedef struct{
  unsigned short address;
  unsigned short width;
}IODescrpt_s;

#define TESTCFG_CAT     0x80
#define TESTCFG_INIO    0x41
#define TESTCFG_OUTIO   0x42

#define IOWIDTH_BYTE    1
#define IOWIDTH_WORD    2
#define IOWIDTH_DWORD   4

static HFILE   dsp_file;

int16 FNTYPE dsp_open(void)
{  ULONG action;

   dsp_io_problem = 0;
   if(DosOpen("TESTCFG$", &dsp_file, &action, 0L, 0, FILE_OPEN,
	OPEN_ACCESS_READONLY | OPEN_FLAGS_NO_CACHE | OPEN_SHARE_DENYNONE,
	0L))
      dsp_io_problem = 1;

   return dsp_io_problem;
}

void FNTYPE dsp_close(void)
{  if(DosClose(dsp_file))
   {  dsp_io_problem = 1;
      dsp_file = 0;
   }
}

unsigned16 FNTYPE DSP_IN(int16 pp)
{  IODescrpt_s IODescrpt;
   ULONG   ParmLengthInOut;
   ULONG   DataLengthInOut;
   unsigned16 IOData;

   IODescrpt.address = pp;
   IODescrpt.width   = IOWIDTH_WORD;

   ParmLengthInOut = sizeof(IODescrpt);
   DataLengthInOut = sizeof(IOData);

   DosDevIOCtl(dsp_file, TESTCFG_CAT, TESTCFG_INIO, (void *)&IODescrpt,
      sizeof(IODescrpt), &ParmLengthInOut, (void *)&IOData,
      sizeof(IOData), &DataLengthInOut);

   return IOData;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  ODescrpt_s IODescrpt;
   ULONG   ParmLengthInOut;

   IODescrpt.address = pp;
   IODescrpt.width   = IOWIDTH_WORD;
   IODescrpt.IOData = vv;

   ParmLengthInOut = sizeof(IODescrpt);

   DosDevIOCtl(dsp_file, TESTCFG_CAT, TESTCFG_OUTIO, (void *)&IODescrpt,
      sizeof(IODescrpt), &ParmLengthInOut, NULL, 0, NULL);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{  IODescrpt_s IODescrpt;
   ULONG   ParmLengthInOut;
   ULONG   DataLengthInOut;
   unsigned char IOData;

   IODescrpt.address = pp;
   IODescrpt.width   = IOWIDTH_BYTE;

   ParmLengthInOut = sizeof(IODescrpt);
   DataLengthInOut = sizeof(IOData);

   DosDevIOCtl(dsp_file, TESTCFG_CAT, TESTCFG_INIO, (void *)&IODescrpt,
      sizeof(IODescrpt), &ParmLengthInOut, (void *)&IOData,
      sizeof(IOData), &DataLengthInOut);

   return IOData;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  ODescrpt_s IODescrpt;
   ULONG   ParmLengthInOut;

   IODescrpt.address = pp;
   IODescrpt.width   = IOWIDTH_BYTE;
   IODescrpt.IOData = vv;

   ParmLengthInOut = sizeof(IODescrpt);

   DosDevIOCtl(dsp_file, TESTCFG_CAT, TESTCFG_OUTIO, (void *)&IODescrpt,
      sizeof(IODescrpt), &ParmLengthInOut, NULL, 0, NULL);
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
   int16 e;

	dsp_open();

	if(dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	atexit(dsp_close);
	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#endif
/* End of Section 3h: OS2 Warp */


/* Section 3i: LynxOS */
/* NOTE: LynxOS uses assembly language code for I/O.  The code is found
in Lynxasm.c */
#ifdef MEI_LYNX
int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{	return pcdsp_set_address(pdsp, iobase);
}
#endif
/* End of section 3i: LynxOS */

/* Section 3j: VRTX/OS */ 	
/* NOTE: For the VRTX operating system, the memory must be mapped in the mmu 
so that the 0xFFFF0000 page is set to BOOTOS_MEMORY_IO|BOOTOS_MEMORY_SERIALIZED.
This is usually set in bootcnfg.c. */
#ifdef MEI_VRTXOS
#  ifndef A16_BASE
#     define	A16_BASE			(0xFFFF0000)
#  endif

unsigned16 FNTYPE DSP_IN(int16 pp)
{  return (unsigned16)(dspPtr->mei_dsp_base[pp/2]);
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{  unsigned16 * c = (unsigned16 *) dspPtr->mei_dsp_base;
   c[pp/2] = vv;
}

unsigned char FNTYPE DSP_INB(int16 pp)
{	unsigned char * c = (unsigned char *) dspPtr->mei_dsp_base;
	return c[pp];
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{  unsigned char * c = (unsigned char *) dspPtr->mei_dsp_base;
   c[pp] = vv;
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{
	int16 e;
	mo16xio_enable_vme16();  /*  BS 2/4 */
	
	pdsp->mei_dsp_base = (int16*) (A16_BASE);
	
	if (dsp_io_problem)
		return (dsp_error = DSP_NOT_INITIALIZED);

	e = pcdsp_set_address(pdsp, iobase);
	return e;
}
#endif
/* End of section 3j: VRTX/OS */


/* Section 3k: VXI */
#ifdef MEI_VXI
#include <nivxi.h>
#include <stdlib.h>

unsigned16 FNTYPE DSP_IN(int16 pp)
{	unsigned16 vv;
	VXIin(0x1, pp, 2, &vv);
	return vv;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	VXIout(0x1, pp, 2, vv);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{	unsigned char vv;
	VXIin(0x1, pp, 1, &vv);
	return vv;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{	VXIout(0x1, pp, 1, vv);
}

void dsp_close(void)
{	CloseVXIlibrary();
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{	int16 ret;
	
	ret = InitVXIlibrary();

	if(ret < 0)	/* initialization of RM library failed */
		return (dsp_error = DSP_NOT_INITIALIZED);

	atexit(dsp_close);

	return pcdsp_set_address(pdsp, iobase);
}

#endif 
/* End of section 3k: VXI */


/* Section 3l: OS9000 */
#ifdef MEI_OS9000

int FNTYPE pcdsp_init_board_comm(PDSP pdsp, int iobase)
{  return pcdsp_set_address(pdsp, iobase);
}
  
unsigned16 FNTYPE DSP_IN(unsigned32 pp)
{  return inw(pp);
}
 
void FNTYPE DSP_OUT(unsigned32 pp, unsigned16 vv)
{  outw(pp, vv) ;
}
 
unsigned char FNTYPE DSP_INB(unsigned32 pp)
{  return inc(pp);
}
 
void FNTYPE DSP_OUTB(unsigned32 pp, unsigned char vv)
{  outc(pp, vv);
} 
#endif
/* End of Section 3l: OS9000 */


/* Section 3m: VMIC for WINNT */
#ifdef MEI_VMIC
#	ifdef MEI_WINNT
unsigned16 FNTYPE DSP_IN(int16 pp)
{	V_DATA16 vv;
	vmeReadVmeEx(0, V_DATA16, pp, &vv, 1, V_A16SD, V_LITTLE_ENDIAN, 
		V_LEVEL_BR0, 512, 0);
	return (unsigned16)vv;
}

void FNTYPE DSP_OUT(int16 pp, unsigned16 vv)
{	vmeWriteVmeEx(0, V_DATA16, pp, &vv, 1, V_A16SD, V_LITTLE_ENDIAN,
		V_LEVEL_BR0, 512, 0);
}

unsigned char FNTYPE DSP_INB(int16 pp)
{	V_DATA8 vv;
	vmeReadVmeEx(0, V_DATA8, pp, &vv, 1, V_A16SD, V_LITTLE_ENDIAN, 
		V_LEVEL_BR0, 512, 0);
	return (unsigned char)vv;
}

void FNTYPE DSP_OUTB(int16 pp, unsigned char vv)
{	vmeWriteVmeEx(0, V_DATA8, pp, &vv, 1, V_A16SD, V_LITTLE_ENDIAN,
		V_LEVEL_BR0, 512, 0);
}

void __cdecl dsp_close(void)
{	vmeTerm(0);
}

int16 FNTYPE pcdsp_init_board_comm(PDSP pdsp, unsigned16 iobase)
{	if(vmeInit(0, 0) != ERROR_SUCCESS)
		return (dsp_error = DSP_NOT_INITIALIZED);

/*	atexit(dsp_close); */

	return pcdsp_set_address(pdsp, iobase);
}
#	endif	/* MEI_WINNT */
#endif /* MEI_VMIC */
/* End of section 3m: VMIC for WINNT */


/* Section 4: Device Driver calls */
#ifdef MEI_DD
#  ifndef MEI_WINNT
int dd_call(int function, int16 * ptr)
{
	_AX = function ;
	_ES = FP_SEG(ptr) ;
	_BX = FP_OFF(ptr);
	asm int DD_INTERRUPT ;
	return 0;
}

#  else	/* MEI_WINNT is defined. */
static int16 dummy[128] ;

int dd_call(int function, int16 * ptr)
{
	ULONG length = 0, rl, i ;

	switch (function)
	{
		case IOCTL_PCDSP_INIT:
			length = 1;
			break;
		case IOCTL_PCDSP_OUT:
			length = (ULONG) ptr[0] + 2 ;
			break;
		case IOCTL_PCDSP_IN:
			length = (ULONG) ptr[0] + 2 ;
			break;
		case IOCTL_PCDSP_ALLOCATE:
			length = 1;
			break;
		case IOCTL_PCDSP_DOWNLOAD:
			length = 22 ;
			break;
		case IOCTL_PCDSP_BLOCK:
			length = (ULONG) ptr[0] + 3 ;
			break;
	}

	length *= sizeof(USHORT) ;

   DeviceIoControl(dspPtr->dsp_file, function,
			ptr, length,
			dummy, length,
			&rl, NULL);
	rl /= sizeof(int16) ;
	for (i = 0; i < rl; i++)
		ptr[i] = dummy[i] ;
	return 0;
}
#  endif	/* MEI_WINNT */
#endif		/* MEI_DD */
/* End of Section 4: Device Driver calls */


/* Section 5: General.  Used by all operating systems */



int16 FNTYPE pcdsp_reset(PDSP dsp)
{
	if(!dsp || !_dsp_initd)
		return (dsp_error = DSP_NOT_INITIALIZED);
#ifdef MEI_SEP_RESET
	DSP_OUTB(dsp->reset, 0);
	DSP_OUTB((P_DSP_DM)(dsp->reset + 1), 0);
#else
	DSP_OUT(dsp->reset, 0xFFFF);
#endif
	return (dsp_error = DSP_OK);
}

int16 FNTYPE pcdsp_sick(PDSP dsp, int16 axis)
{	DSP_DM current;
	if(!_dsp_initd || !dsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	if(!(dsp->ok))
		return (dsp_error = DSP_NOT_INITIALIZED);

	if((axis < 0) || (axis >= ((int16) dsp->axes)))
		return (dsp_error = DSP_INVALID_AXIS);

	pcdsp_get_config_struct(dsp, axis, CL_CONFIG_DATA, NULL, &current);
	if(current & CD_KILL_AXIS)
		return (dsp_error = DSP_INVALID_AXIS);

	if((dspPtr->sercos) && loop_open())
		return dsp_error;

	return (dsp_error = DSP_OK);
}

int16 FNTYPE pcdsp_init_axis_check(PDSP dsp, int16 axis)
{	
	if(!_dsp_initd || !dsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	if(!(dsp->ok))
		return (dsp_error = DSP_NOT_INITIALIZED);

	if((axis < 0) || (axis >= ((int16) dsp->axes)))
		return (dsp_error = DSP_INVALID_AXIS);

	return (dsp_error = DSP_OK);
}

int16 FNTYPE pcdsp_init_check(PDSP dsp)
{
	if(!_dsp_initd || !dsp)
		return (dsp_error = DSP_NOT_INITIALIZED);

	if(!(dsp->ok))
		return (dsp_error = DSP_NOT_INITIALIZED);

	return (dsp_error = DSP_OK);
}

int16 FNTYPE pcdsp_set_address(PDSP dsp, unsigned iobase)
{
	if (! dsp)
		return (dsp_error = DSP_NOT_INITIALIZED) ;

	dsp->iobase = iobase;
	dsp->address = iobase + PCDSP_ADDRESS ;
	dsp->data = iobase + PCDSP_DATA ;
	dsp->reset = iobase + PCDSP_RESET ;
	dsp->ok = FALSE ;

	if (! _dsp_initd)
	{	fixed_zero.whole = 0 ;
		fixed_zero.frac = 0;
	}

	_dsp_initd = TRUE ;

	return DSP_OK ;
}

int16 FNTYPE pcdsp_axes(PDSP pdsp)
{	int16 axes = 0, axis;
	for(axis = 0; axis < pdsp->axes; axis++)
		if(!(dsp_read_dm((int16)(pdsp->e_data+ED(axis)))&CD_KILL_AXIS))
			axes++;
	return axes;
}

int16 FNTYPE pcdsp_read_pointers(PDSP pdsp)
{
#ifndef MEI_VW
	int16 clock_id;
#endif

	pdsp->signature = idsp_read_dm(pdsp, DM_SIGNATURE);
	pdsp->version = idsp_read_dm(pdsp, DM_FIRMWARE_VERSION);
	pdsp->option = idsp_read_dm(pdsp, DM_OPTION);

	if (pdsp->signature != PCDSP_SIGNATURE)
		return (dsp_error = DSP_NOT_FOUND) ;

	if ((pdsp->version >> 4) != PCDSP_VERSION)
		return (dsp_error = DSP_FIRMWARE_VERSION) ;

	pdsp->ok = TRUE ;

	pdsp->axes = idsp_read_dm(pdsp, DM_AXES);
	pdsp->pc_event = idsp_read_dm(pdsp, DM_PC_EVENT);
	pdsp->pc_status = idsp_read_dm(pdsp, DM_PC_STATUS);
	pdsp->inptr = idsp_read_dm(pdsp, DM_INPTR);
	pdsp->outptr = idsp_read_dm(pdsp, DM_OUTPTR);

	pdsp->infree = pdsp->inptr + pdsp->axes;
	pdsp->outfree = pdsp->outptr + pdsp->axes;
	
	pdsp->transfer = idsp_read_dm(pdsp, DM_TRANSFER_BLOCK);
	pdsp->data_struct = idsp_read_dm(pdsp, DM_DATA_STRUCT);
	pdsp->e_data = idsp_read_dm(pdsp, DM_E_DATA);
	pdsp->global_data = idsp_read_dm(pdsp, DM_GLOBAL_DATA);

	/* read the sample clock. */
	dsp_error = DSP_OK ;
	dsp_error = pcdsp_transfer_block(pdsp, TRUE, FALSE, PCDSP_TIMER, 1, &(pdsp->sample_clock)) ;

#ifdef MEI_VW
		pdsp->timer_scale = 10000000L;	/* clock is running at 10MHz (default) */
#else
	/* check to see if the board is running at 20 MHz, by writing the address
		of the DSP's signature word into the ID address.  If the controller is
		operating at 20MHz, the ID low byte (0x8) will be aliased to the
		address low byte (0x0) and the ID high byte (0x9) will be aliased to
		the address high byte (0x1).  */
	DSP_OUT((int16)(pdsp->iobase + 8), (unsigned16)(PCDSP_DM | DM_SIGNATURE));
	clock_id = DSP_IN((int16)(pdsp->iobase + PCDSP_DATA));

	if (clock_id == PCDSP_SIGNATURE)
	{
		pdsp->timer_scale = 20000000L;	/* clock is running at 20MHz */
	}
	else
	{
		pdsp->timer_scale = 10000000L;	/* clock is running at 10MHz (default) */
	}
#endif
	return dsp_error;
}

int16 FNTYPE pcdsp_interrupt_enable(PDSP pdsp, int16  enable)
{	P_DSP_DM ip = PC_INTERRUPT_ENABLE ;
	DSP_DM p = idsp_read_dm(pdsp, ip) ;
	if (enable)
		p &= ~DSP_INTERRUPT_BIT ;
	else
		p |= DSP_INTERRUPT_BIT ;
	return idsp_write_dm(pdsp, ip, p) ;
}

int16 FNTYPE pcdsp_arm_latch(PDSP pdsp, int16 enable)
{
	DSP_DM gd_config ;
	P_DSP_DM gd = pdsp->global_data + GD_CONFIG ;

	if (pcdsp_init_check(pdsp))
		return dsp_error ;

	gd_config = idsp_read_dm(pdsp, gd) ;
	if (enable)
	{	gd_config |= GDC_LATCH ;
		idsp_write_dm(pdsp, gd, gd_config) ;
		pcdsp_interrupt_enable(pdsp, enable) ;
	}
	else
	{	gd_config &= ~GDC_LATCH ;
		pcdsp_interrupt_enable(pdsp, enable) ;
		idsp_write_dm(pdsp, gd, gd_config) ;
	}
    return dsp_error;
}

int16 FNTYPE pcdsp_latch_status(PDSP pdsp)
{	DSP_DM i ;
	if (pcdsp_init_check(pdsp))
		return 0 ;
	i = idsp_read_dm(pdsp, (P_DSP_DM)(pdsp->global_data + GD_CONFIG));
	return (i & (GDC_LATCH_STATUS | GDC_LATCH)) == 1;
}

int16 FNTYPE pcdsp_jog_axis(PDSP pdsp, int16 axis, int16 jog_channel, DSP_DM c, DSP_DM d, DSP_DM m1, DSP_DM m2)
{
	P_DSP_DM a = pdsp->global_data + GD_JOG ;
   DSP_DM ax = (DSP_DM)axis;
   a += (jog_channel * 5);
	idsp_write_dm(pdsp, a++, ax);
	idsp_write_dm(pdsp, a++, c);
	idsp_write_dm(pdsp, a++, d);
	idsp_write_dm(pdsp, a++, m2);
	idsp_write_dm(pdsp, a++, m1);
	return DSP_OK ;
}

int16 FNTYPE pcdsp_jog_enable(PDSP pdsp, int16 axis)
{	DSP_DM config ;
	pcdsp_get_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &config) ;
	config |= CD_JOG ;
	pcdsp_set_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &config) ;
	return 0 ;
}

int16 FNTYPE pcdsp_jog_disable(PDSP pdsp, int16 axis)
{	DSP_DM config ;
	pcdsp_get_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &config) ;
	config &= ~CD_JOG ;
	pcdsp_set_config_struct(pdsp, axis, CL_CONFIG_DATA, NULL, &config) ;
	return 0;
}

int16 FNTYPE pcdsp_set_integrator(PDSP pdsp, int16 axis, int16 value)
{
    DSP_DM addr;
    DSP_DM data;

    addr = pdsp->data_struct + DS(axis) + DS_D(2) ;
    data = value ;

	pcdsp_transfer_block(pdsp, FALSE, FALSE, addr ,1, &data) ;

	return dsp_error ;
}
/* End of section 5 */

int16 LOCAL_FN check_copyright(void)
{
	/* This function is not very useful.  It is used to prevent compiler
		warning messages for COPYRIGHT being defined but not used. */
	
	if (COPYRIGHT == NULL)
		return dsp_error;

	return dsp_error;
}


