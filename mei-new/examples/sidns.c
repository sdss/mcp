/* SIDNS.C

:Read/Write/Copy a group of SERCOS IDNs.

This sample code demonstrates how to read/write a list of sequential
 SERCOS IDN data values.

Be sure to initialize the SERCOS ring with serc_reset(...) before
 reading/writing to the IDNs. 

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5
*/

	
#include <stdio.h>
#include <stdlib.h>
#include "pcdsp.h"
#include "sercos.h"

#define SOURCE_AXIS			0
#define SOURCE_ADDR			1	/* SERCOS Node Address */

#define TARGET_AXIS			1
#define TARGET_ADDR			2	/* SERCOS Node Address */

#define	FIRST_S_IDN			30	/* first Standard IDN */
#define FIRST_P_IDN			0	/* first Product Specific IDN */

#define	S_IDNS				200
#define P_IDNS				100

IDNS s_buffer[S_IDNS];
IDNS p_buffer[P_IDNS];


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

int	main()
{
	int16 error_code, i;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	printf("\nReading Standard IDNs...");
	error(get_idns(SOURCE_AXIS, SOURCE_ADDR, FIRST_S_IDN, S_IDNS, s_buffer));
	printf("done.\n");
	for(i = 0; i < S_IDNS; i++)
		if (!s_buffer[i].error)	
			printf("S-0-%4d:\t%ld\n", s_buffer[i].idn, s_buffer[i].value);

	printf("\nReading Drive Specific IDNs...");
	error(get_idns(SOURCE_AXIS, SOURCE_ADDR, (0x8000 + FIRST_P_IDN), P_IDNS, p_buffer));
	printf("done.\n");
	for(i = 0; i < P_IDNS; i++)
		if (!p_buffer[i].error)	
			printf("P-0-%4d:\t%ld\n", (p_buffer[i].idn - 0x8000), p_buffer[i].value);

	printf("\nWriting Standard IDNs...");
	error(set_idns(TARGET_AXIS, TARGET_ADDR, S_IDNS, s_buffer));
	printf("done.\n");
	for(i = 0; i < S_IDNS; i++)
		if (!s_buffer[i].error)	
			printf("P-0-%4d:\t%ld\n", s_buffer[i].idn, s_buffer[i].value);		

	printf("\nWriting Drive Specific IDNs...");
	error(set_idns(TARGET_AXIS, TARGET_ADDR, P_IDNS, p_buffer));
	printf("done.\n");
	for(i = 0; i < P_IDNS; i++)
		if (!p_buffer[i].error)	
			printf("P-0-%4d:\t%ld\n", (p_buffer[i].idn - 0x8000), p_buffer[i].value);

	return 0;
}
