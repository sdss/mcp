/*
	envutil.c - understand the DSP environment variable.
	
-----------------------------------------------------------------------------
	understand_dsp(char * str) - understand the given string, or if str
			is NULL, then use getenv("DSP").

		The string should follow this format:

			VAR:value;VAR:value...

		The supported VARs and values supported are:

		VAR			Values
		----------	-----------------------------------------------------
		BASE		0xHHH, where HHH is the base I/O address of the card.
					'dsp_base' is set to the value given.

		understand_dsp doesn't return an error code if a variable isn't
			understood.  VAR is case-insensitive.  I wanted to use the
			format 'var=somevalue', but you can't SET variables with
			those sorts of values (DOS yaks on the '=').
-----------------------------------------------------------------------------
	do_dsp() - execute understand_dsp(NULL), and call dsp_init.  The
			value in 'dsp_base' is used as the base address, which unless
			otherwise specified is PCDSP_BASE.  This function returns the
			error code given by dsp_init.
-----------------------------------------------------------------------------
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include "pcdsp.h"
#	include <stdlib.h>
#	include <string.h>
#	include <stddef.h>

#	ifdef	MEI_LYNX
#		define	MEI_STRCMP			strcmp
# 	endif

#	ifdef MEI_VW
#		define	MEI_STRCMP			strcmp
# 	endif

#	ifdef MEI_VRTXOS
#		define	MEI_STRCMP			strcmp
# 	endif

#	ifdef MEI_OS9000
#		define	MEI_STRCMP			strcmp
# 	endif

#	ifdef MEI_OS9
#		define	MEI_STRCMP			strcmp
# 	endif

#	ifndef MEI_STRCMP
#		define	MEI_STRCMP			stricmp
# 	endif

int16 DATATYPE
	dsp_base = PCDSP_BASE ;


int16 LOCAL_FN unknown_seg(P_CHAR parm, P_CHAR value)
{	parm = parm ;
	value = value ;
	return DSP_OK ;
}

USER_SEG user_seg = unknown_seg ;

static int16 understand_seg(char * seg)
{
	char
		*	word = seg,
		*	value = strchr(seg, ':');

	if (value)		/* skip the ':'. */
	{	*value = 0;
		value++ ;
	}

	if (! strlen(word))
		return 0;

	if (!MEI_STRCMP("base", word) && value)
	{	dsp_base = (int16) strtol(value, NULL, 0) ;
		return 0;
	}

	return user_seg(word, value) ;
}



int16 FNTYPE understand_dsp(P_CHAR var)
{
	char *	seg ;
	int16		r = DSP_OK ;
	char	buffer[80] ;

	if (! var)
	{   var = getenv("DSP") ;
		if (var)
			strcpy(buffer, var) ;
		else 
			strcpy(buffer, "") ;
		var = buffer ;
	}

	for (seg = strchr(var, ';'); seg; seg = strchr(var, ';'))
	{	*seg = 0;
		understand_seg(var) ;
		var = seg + 1;
	}

	if (strlen(var))
		understand_seg(var) ;

	return r;
}


int16 FNTYPE do_dsp(void)
{	understand_dsp(NULL) ;

	return dsp_init(dsp_base) ;
}
