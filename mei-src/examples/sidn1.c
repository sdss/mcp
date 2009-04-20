/* SIDN1.C

:SERCOS IDN value/string read.

This sample program demonstrates how to read an IDN value or IDN string
 from a SERCOS node.

Be sure to initialize the SERCOS ring with serc_reset(...) before
 reading/writing to an IDN.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5  
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "idsp.h"
#include "sercos.h"
#include "sercrset.h"

#define AXIS	0
#define ADDR	1
#define IDN		30	/* Manufacturer Version (string) */


void error(int16 error_code)
{   
	char buffer[MAX_ERROR_LEN];

	switch (error_code)
	{
		case DSP_OK:
			/* No error, so we can ignore it. */
			break ;

		default:
			error_msg(error_code, buffer) ;
			fprintf(stderr, "ERROR: %s (%d).\n", buffer, error_code) ;
			exit(1);
			break;
	}
}

int main(void)
{	
	int16 error_code, size;
	int32 value;
	char string[MAX_ERROR_LEN];
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(get_idn_size(AXIS, IDN, &size));	/* determine data size */

	if (size)
	{ 	error(get_idn(AXIS, IDN, &value)); 	
		printf("\nData length(words) is: %d\n", size);
		printf("\nValue: %ld", value);
	}
	else	/* variable length string */
	{	error(get_idn_string(AXIS, 1, IDN, string)); 
		printf("\nData length is variable\n");
		printf("\n%s", string);
	}
	
	return 0;
}
