/*
  	SERCDIAG.C - Sercos diagnostic routines
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */
 
# include <string.h>
# include "idsp.h"
# include "sercos.h"

typedef struct
{	int16 code;
	char *msg;
} DIAG_MSG ;

static DIAG_MSG
	__c1diag_message[] = {
		{ 0x0, "No Error" },
		{ 0x1, "Overload shutdown"},
		{ 0x2, "Amplifier over temperature shutdown"},
		{ 0x4, "Motor over temperature shutdown"},
		{ 0x8, "Cooling error shutdown"},
		{ 0x10, "Control voltage error"},
		{ 0x20, "Feedback error"},
		{ 0x40, "Error in the commutation system"},
		{ 0x80, "Overcurrent error"},
		{ 0x100, "Overvoltage error"},
		{ 0x200, "Undervoltage error"},
		{ 0x400, "Power supply phase error"},
		{ 0x800, "Excessive position deviation"},
		{ 0x1000, "Communication error"},
		{ 0x2000, "Overtravel limit is exceeded"},
		{ 0x4000, "(reserved)"},
		{ (int16)0x8000, "Manufacturer-specific error"}
			} ;

static unsigned
	__c1diag_message_length = sizeof(__c1diag_message) / sizeof(__c1diag_message[0]);

P_CHAR FNTYPE _c1diag_msg(int16 code)
{	unsigned u;
	char * r = NULL;

	for(u = 0; !r && (u < __c1diag_message_length); u++)
	{	if (__c1diag_message[u].code == code)
			r = __c1diag_message[u].msg;
	}

	return r;
}

/* msg must be at least MAX_ERROR_LEN characters long.*/
int16 FNTYPE get_class_1_diag(int16 axis, int16 *code, char *msg)
{	char *s;

	get_idn(axis, (int16)11, (long*)code);
	s = _c1diag_msg(*code);

	if(s)
		strcpy(msg, s);
	else
		msg[0] = 0;

	return dsp_error;
}

static DIAG_MSG
	__c2diag_message[] = {
		{ 0x0, "No Error" },
		{ 0x1, "Overload warning"},
		{ 0x2, "Amplifier over temperature warning"},
		{ 0x4, "Motor over temperature warning"},
		{ 0x8, "Cooling error warning"},
		{ 0x10, "(reserved)"},
		{ 0x20, "(reserved)"},
		{ 0x40, "(reserved)"},
		{ 0x80, "(reserved)"},
		{ 0x100, "(reserved)"},
		{ 0x200, "(reserved)"},
		{ 0x400, "(reserved)"},
		{ 0x800, "(reserved)"},
		{ 0x1000, "(reserved)"},
		{ 0x2000, "(reserved)"},
		{ 0x4000, "(reserved)"},
		{ (int16)0x8000, "Manufacturer-specific warning"}
			} ;

static unsigned
	__c2diag_message_length = sizeof(__c2diag_message) / sizeof(__c2diag_message[0]);

P_CHAR FNTYPE _c2diag_msg(int16 code)
{	unsigned u;
	char * r = NULL;

	for(u = 0; !r && (u < __c2diag_message_length); u++)
	{	if (__c2diag_message[u].code == code)
			r = __c2diag_message[u].msg;
	}

	return r;
}

/* msg must be at least MAX_ERROR_LEN characters long.*/
int16 FNTYPE get_class_2_diag(int16 axis, int16 * code, char *msg)
{	char *s;

	get_idn(axis, (int16)12, (long*)code);
	s = _c2diag_msg(*code);

	if(s)
		strcpy(msg, s);
	else
		msg[0] = 0;

	return dsp_error;
}
