/* SIDNL.C

:SERCOS sample to read an "IDN-List" from an IDN in a specific drive.

The "Operation Data" for several IDNs are actually a variable length list of 
 IDNs.  This sample code demonstrates how to read an "IDN-List" from an IDN.

Here are some useful IDNs (with IDN-Lists):

IDN		Description

17		List of all operation data.
25		List of all procedure commands.
192		List of backup operation data.
187		List of configurable data in the AT
188		List of configurable data in the MDT
21		List of invalid operation data for CP2
22		List of invalid operation data for CP3
23		List of invalid operation data for CP4
18		List of operation data for CP2
19		List of operation data for CP3
20		List of operation data for CP4

Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before reading the list of supported IDNs.

Please consult the drive manufacturer's documentation or SERCOS specification
 for more information concerning "IDN-List".

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5
*/

	
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <string.h>
#include "idsp.h"
#include "sercos.h"

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define BUFFER_SIZE		1024	
#define AXIS			0
#define IDN				17


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
	int16 error_code, len, value, p, a=0;
	unsigned16 i, *buffer, channel, str[MAX_E2_INTS];
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	buffer = (unsigned16*) calloc(BUFFER_SIZE, sizeof(unsigned16));
	
	error(get_sercos_channel(AXIS, &channel));	/* read channel assigned to axis */
	error(read_idn(channel, IDN, &len, buffer, TRUE));	/* read list of idns */

	printf("List of IDN Values for axis: %d\n", AXIS);

	for(i = 0; i < len; i++)
	{	if(get_element_2(channel, buffer[i], str))
			memcpy(str, "No text available", 20);
		p = buffer[i] & 0x8000;
		if(p)
			buffer[i] -= (int16)0x8000;
		printf("%s-%1d-%4d   %s\n",p?"P":"S", a, buffer[i], str);
	}

    return 0;
}
