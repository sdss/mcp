/*
	mlerror.c
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
#	include <stddef.h>
#	include <string.h>


typedef struct
{	int16			code ;
	char	*	msg ;
} ERROR_MSG ;


static ERROR_MSG
	__error_message[] = {
		{ DSP_OK,				"No Error" },
		{ DSP_NOT_INITIALIZED,	"DSP Not Initialized" },
		{ DSP_NOT_FOUND,		"DSP Not found at the given I/O address" },
		{ DSP_INVALID_AXIS,		"Illegal axis specified" },
		{ DSP_ILLEGAL_ANALOG,	"Illegal analog channel specified" },
		{ DSP_ILLEGAL_IO,		"get_io/set_io from illegal port" },
		{ DSP_OUT_OF_MEMORY,	"Out of DSP internal memory" },
		{ DSP_FRAME_UNALLOCATED, "Download of Unallocated Frame" },
		{ DSP_ILLEGAL_PARAMETER, "Illegal Accel, Velocity, or Jerk" },
		{ DSP_ILLEGAL_CONVERSION, "CountsPerDistance or SecondsPerPeriod are zero"},
		{ DSP_FRAME_NOT_CONNECTED, "Unload of unconnected frame" },
		{ DSP_FIRMWARE_VERSION,	"PC software v" PCDSP_ASCII_VERSION "/DSP firmware version incompatible" },
		{ DSP_ILLEGAL_TIMER,	"Illegal timer" },
		{ DSP_STRUCTURE_SIZE,	"FRAME Structure size is incorrect; a porting problem is suspected" },
		{ DSP_TIMEOUT_ERROR,	"DSP Timeout error" },
		{ DSP_RESOURCE_IN_USE,	"Libraries' Global Resource is in use" },
		{ DSP_CHECKSUM,			"Boot memory checksum error" },
		{ DSP_CLEAR_STATUS,		"Can't clear status" },
		{ DSP_NO_MAP,			"Coordinated motion not initialized" },
		{ DSP_NO_ROOM,			"Out of DSP FIFO buffer space" },
		{ DSP_BAD_FIRMWARE_FILE,   "Specified firmware file is currupt or nonexistant" },
		{ DSP_ILLEGAL_ENCODER_CHANNEL,   "Illegal encoder channel specified" },
		{ DSP_FUNCTION_NOT_AVAILABLE,   "Function is no longer available" },
		{ DSP_NO_EXTERNAL_BUFFER,   "External buffer is full or does not exist" },
		{ DSP_NT_DRIVER,   "Windows NT DSPIO Driver not found" },
		{ DSP_FUNCTION_NOT_APPLICABLE, "Function is not applicable"},
		{ DSP_NO_DISTANCE, "Move distance is zero"},
		{ DSP_FIRMWARE_CHECKSUM, "Firmware buffer checksum error"},
		{ DSP_SERCOS_SLAVE_ERROR, "SERCOS drive is not responding"}, 
		{ DSP_SERCOS_INVALID_PARAM, "Invalid parameter passed to serc_reset()"},
		{ DSP_SERCOS_DISTORTED, "SERCOS data is distorted"},
		{ DSP_SERCOS_LOOP_OPEN, "SERCOS ring is open"},
		{ DSP_SERCOS_EARLY, "SERCOS MST was sent too early"},
		{ DSP_SERCOS_LATE, "SERCOS MST was sent too late"},
		{ DSP_SERCOS_MST_MISSING, "SERCOS MST is missing"},
		{ DSP_SERCOS_DRIVE_INIT, "SERCOS drive(s) were not found"},
		{ DSP_SERCOS_INVALID_DRIVE_TYPE, "SERCOS invalid drive mode"},
		{ DSP_SERCOS_INVALID_DRIVE_NUMBER, "Invalid number of SERCOS drives"},
		{ DSP_SERCOS_INVALID_DRIVE_ADDR, "Invalid SERCOS drive address"},
		{ DSP_SERCOS_DUPLICATE_DRIVE_ADDR, "Duplicate SERCOS drive address"},
		{ DSP_SERCOS_PROC_FAILURE, "SERCOS Procedure failed"},
		{ DSP_SERCOS_AXIS_ASSIGNMENT, "Axis not assigned to a SERCOS drive"},
		{ DSP_SERCOS_RESET, "Use serc_reset(...) instead of dsp_reset(...) with SERCOS controllers"},
		{ DSP_SERCOS_VARIABLE_READ, "Use get_idn_string(...) to read variable length strings"},
		{ DSP_SERCOS_INVALID_IDN_AT, "Invalid IDN in the SERCOS AT cyclic data"},
		{ DSP_SERCOS_INVALID_IDN_MDT, "Invalid IDN in the SERCOS MDT cyclic data"},
		{ DSP_SERCOS_127_FAILURE, "SERCOS drive unable to enter phase 3"},
		{ DSP_SERCOS_128_FAILURE, "SERCOS drive unable to enter phase 4"},
		{ DSP_SERCOS_IDN_NOT_AVAILABLE, "SERCOS IDN not supported in drive"},
		{ DSP_SERCOS_NO_CHANNEL, "SERCOS Service channel not open"},
		{ DSP_SERCOS_ELEMENT_MISSING, "SERCOS IDN Element not supported in drive"},
		{ DSP_SERCOS_SHORT_TRANS, "SERCOS Transmission is too short"},
		{ DSP_SERCOS_LONG_TRANS, "SERCOS Transmission is too long"},
		{ DSP_SERCOS_STATIC_VAL, "SERCOS Value cannot be changed"},
		{ DSP_SERCOS_WRITE_PROTECT, "SERCOS Value is write protected"},
		{ DSP_SERCOS_MIN, "SERCOS Data is less than allowable range"},
		{ DSP_SERCOS_MAX, "SERCOS Data is greater than allowable range"},
		{ DSP_SERCOS_INVALID_DATA, "SERCOS Data is not valid"},
		{ DSP_SERCOS_PROTOCOL, "SERCOS Protocol error in slave"},
		{ DSP_SERCOS_HS_TIMEOUT, "SERCOS Service container handshake timeout"},
		{ DSP_SERCOS_BUSY, "SERCOS drive's BUSY_AT bit is on"},
		{ DSP_SERCOS_CMD, "SERCOS drive set the command modification bit"},
		{ DSP_SERCOS_M_NOT_READY, "SERCOS M_BUSY bit is off"},
		{ DSP_SERCOS_SC_TIMEOUT, "SERCOS Service Container timeout error"},
		{ DSP_SERCOS_REC_ERR, "SERCOS Invalid Service Container transmission"},
		{ DSP_SERCOS_INVALID_CYCLE_TIME, "SERCOS Cycle Time is too short"},
		{ DSP_SERCOS_USER_AT, "SERCOS user configurable AT maximum length exceeded"},
		{ DSP_SERCOS_USER_MDT, "SERCOS user configurable MDT maximum length exceeded"}
			} ;

static unsigned
	__error_messages_length = sizeof(__error_message) / sizeof(__error_message[0]);



P_CHAR FNTYPE _error_msg(int16 code)
{
	unsigned
		u;

	char
		* r = NULL ;

	for (u = 0; !r && (u < __error_messages_length); u++)
	{	if (__error_message[u].code == code)
			r = __error_message[u].msg ;
	}

	return r;
}


/*
	dst must be at least MAX_ERROR_LEN characters long.
*/
int16 FNTYPE error_msg(int16 code, P_CHAR dst)
{
	char * s = _error_msg(code) ;

	if (s)
		strcpy(dst, s);
	else
		dst[0] = 0;

	return dsp_error ;
}


/*
	provide read-only access to dsp_error.
*/
int16 FNTYPE _dsp_error(void)
{	return dsp_error ;
}



