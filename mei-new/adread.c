/* ADREAD.C

:Read the Analog to Digital converter.

This sample code demonstrates how to read digital values from the A/D (analog
 to digital) converter.  The values are stored into a buffer (host) and later
 displayed.  There are several command line switches to configure the A/D, 
 the number of reads, and the display:

	diff - read a differential channel
	bi - read a bipolar channel
	dsp - read the A/D with the DSP
	debug - display the buffer 

For example, to read a single ended A/D input with the DSP 5000 times, and 
 redirect the output to a file:

 	adread 5000 dsp debug > results.txt

Warning!  The DSP and host CPU must NOT be configured to read the A/D converter
 at the same time.
 
Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/

	
# include <stdio.h>
# include <stdlib.h>
# include <conio.h>
# include <dos.h>
# include <string.h>
# include "pcdsp.h"


# define MAX_BUFFER_SIZE	10000
# define AXIS				0
# define AD_CHANNEL			0		/* valid range 0 to 7 */

int16 atod[MAX_BUFFER_SIZE];
int16 buffer_size = 0;

int16 ad_bi = FALSE;			/* bipolar? */
int16 ad_diff = FALSE;		/* differential? */
int16 ad_dsp = FALSE;			/* read with the DSP? */
int16 debug = FALSE;			/* display the buffer? */


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

void calc_values(int16 num, int16 * value)
{
	double i, avg, max, min, total = 0.0;

	for (i = 0; i < num; i++)
		total += value[i];

	avg = total / num;
	max = avg;
	min = avg;
		
	for (i = 0; i < num; i++)
	{
		if (value[i] > max)
		{ 	max = value[i];
			if (debug)
				printf("Max: ");
		}	

		if (value[i] < min)
		{	min = value[i];
			if (debug)
				printf("Min: ");
		}	
		if (debug)
			printf("%4d\n", value[i]);	
	}
	printf("\nDiff: %d Bi: %d DSP: %d", ad_diff, ad_bi, ad_dsp);	
	printf("\nAvg: %6.2lf Max: %6.2lf Min: %6.2lf Range: %6.2lf", avg, max, min, max - min);

}

void arguments(int16 argc, char * argv[])
{
	int16 i;
	char * s;
	
    for (i = 1; i < argc; i++)
   	{	s = argv[i]; 
     	if (atoi(s))
     	{	buffer_size = atoi(s);
     		continue;
		}		
		if (!strcmpi(s, "diff"))
		{	ad_diff = TRUE;		
			continue;
		}
		if (!strcmpi(s, "bi"))
		{	ad_bi = TRUE;
			continue;
		}	
		if (!strcmpi(s, "dsp"))
		{	ad_dsp = TRUE;
			continue;
		}		
		if (!strcmpi(s, "debug"))
		{	debug = TRUE;
			continue;
		}		
		if (!strcmpi(s, "?"))
		{	printf("\nUsage: [diff] [bi] [dsp] [debug]");
			exit(0);
		}	
	}	
	if (buffer_size > MAX_BUFFER_SIZE)
		buffer_size = MAX_BUFFER_SIZE;
	if (buffer_size < 1) 
		buffer_size = 1;
}

int16	main(int16 argc, char * argv[])
{
	int16 error_code, i, dsp_time;

	arguments(argc, argv);
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	
	printf("\nCollecting %d A/D values\n", buffer_size);
	set_axis_analog(AXIS, ad_dsp);
	
	if (ad_dsp)
	{ 	set_feedback(AXIS, FB_ANALOG);
		set_analog_channel(AXIS, AD_CHANNEL, ad_diff, ad_bi);
	}
	else
		init_analog(AD_CHANNEL, ad_diff, ad_bi);	
	
	for (i = 0; i < buffer_size; i++)
	{ 	
		if (ad_dsp)
			read_axis_analog(AXIS, &atod[i]);
		else
			start_analog(AD_CHANNEL);
			
		delay(1);		/* wait for the A/D conversion (at least 15 microsec) */	
			
		if (!ad_dsp)
			read_analog(&atod[i]);
	}	
	calc_values(buffer_size, atod);
	
    return 0;
}
