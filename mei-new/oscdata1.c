/* OSCDATA1.C

:Oscilloscope data acquisition with a SERCOS Indramat drive.

This sample demonstrates how to configure a SERCOS Indramat drive to acquire
 data samples with its oscilloscope functions.  The Indramat drive can be
 configured to capture a specified buffer of data.  Data can be position,
 velocity, or torque information.  Data collection can be triggered externally
 or internally.  Also, trigger offsets and delays can be specified.  For more
 information regarding the oscilloscope functions please consult the Indramat
 drive documenation. 

Be sure to initialize the SERCOS ring with serc_reset(...) before
 reading/writing to the IDNs. 

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Motion Library Version 2.5  
*/

	
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "idsp.h"
#include "sercos.h"

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define PIDN	32768	/* drive specific IDNs */

unsigned16 data[2048];


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
	int16 i, error_code, n_words;
	long value;
	unsigned16 channel;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	/* Configure Oscilloscope data logging:
		0 : no signal feedback
		1 : position feedback
		2 : velocity feedback
		3 : velocity deviation
		4 : position deviation
		5 : torque command value
	*/	
	error(set_idn(0, 23 + PIDN, 1));
	
	error(set_idn(0, 25 + PIDN, 2));	/* use internal triggering */
	error(set_idn(0, 26 + PIDN, 1));	/* trigger with position feedback */
	error(set_idn(0, 27 + PIDN, 0L));	/* trigger at position zero */
	error(set_idn(0, 30 + PIDN, 3));	/* trigger in pos/neg direction */

	error(set_idn(0, 31 + PIDN, 250));	/* acquire at 250 microsec intervals */	

	error(set_idn(0, 32 + PIDN, 512));	/* memory allocation */
	error(set_idn(0, 33 + PIDN, 2));	/* trigger sample delay */	
	
	error(set_idn(0, 36 + PIDN, 0x7));	/* start */

	while (!kbhit())
	{ 	error(get_idn(0, 37 + PIDN, &value));	/* read status */
		printf("\rValue:0x%x", value);
	}
	getch();

	channel=dspPtr->sercdata[0].channel;
	read_idn(channel, 21 + PIDN, &n_words, data, TRUE); /* read data */

	printf("\nn_words:%d", n_words);
	for (i = 0; i < n_words; i++)
		printf("\nSample:%4d Value:%u", i, data[i]);
	
    return 0;
}
