/* SNFIND.C

:SERCOS Node find.

This sample program demonstrates how to find the addresses of Nodes connected
 to a SERCOS ring.

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
	int16 error_code, i, nodes;
	unsigned16 addrs[8];

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(get_drive_addresses(BIT_RATE2, &nodes, addrs));
	for(i = 0; i < nodes; i++)
		printf("SERCOS Node found at address %d\n", addrs[i]);

	return 0;
}
