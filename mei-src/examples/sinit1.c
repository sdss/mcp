/* SINIT1.C

:SERCOS initialization with Indramat drive.

This sample program demonstrates SERCOS ring initialization with an
 Indramat servo drive.

The following functions can be used for debugging the ring initialization:

 print_drive_assignment(...) - displays the axis and address assignments for
 all of the possible axes.

 print_log(...) - displays any drive messages.

 print_phase2idns(...) - displays any phase 2 IDNs that were set improperly
 and are required for loop initialization.

 print_phase3idns(...) - displays any phase 3 IDNs that were set improperly
 and are required for loop initialization.

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

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define NODES			1	/* Number of nodes on Sercos ring */
#define	AXIS			0
#define NODE_ADDR		1

/* Drive Configuration Data */
DRIVE_INFO drive_info[NODES] = {
/*	{Axis, Drive Address, Operation Mode, Manufacturer} */
	{AXIS, NODE_ADDR, POSMODE, INDRAMAT}
};


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

void print_drive_assignment(void)
{	int16 i;
	for(i = 0; i < PCDSP_MAX_AXES; i++)
	{	if(assignedDrive[i] > 0)
			printf("Axis %d assigned to Drive %d.\n", i, assignedDrive[i]);
		else
		{	if(assignedDrive[i] == -1)
				printf("Axis %d Drive could not be found.\n", i);
			else
				printf("Axis %d not assigned to any drive.\n", i);
		}
	}
	printf("\n");
}

void print_log(void)
{	int16 i, j;

	for(i = 0; i < dspPtr->axes; i++)
	{	for(j = 0; j < logCount[i]; j++)
			printf("Axis: %d -- %s\n", i, driveMsgs[i][j].msgstr);
	}
}

void print_phase2idns(void)
{	int16 axis, j;

	for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	if(phase2_idncount[axis])
		{	printf("Phase 2 IDNs not set for axis %d:\n", axis);
			for(j = 0; j < phase2_idncount[axis]; j++)
				printf("%d (0x%x)\n", phase2_idnlist[axis][j], phase2_idnlist[axis][j]);
			printf("\n");
		}
	}
}

void print_phase3idns(void)
{	int16 axis, j;

	for(axis = 0; axis < PCDSP_MAX_AXES; axis++)
	{	if(phase3_idncount[axis])
		{	printf("Phase 3 IDNs not set for axis %d:\n", axis);
			for(j = 0; j < phase3_idncount[axis]; j++)
				printf("%d (0x%x)\n", phase3_idnlist[axis][j], phase3_idnlist[axis][j]);
			printf("\n");
		}
	}
}

int main(void)
{	
	int16 error_code;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error_code = serc_reset(BIT_RATE2, NODES, drive_info);

	print_drive_assignment(); 
	print_log();

	if(error_code == DSP_SERCOS_DRIVE_INIT)
	{	int16 drives;
		unsigned16 addrs[8], i;
		get_drive_addresses(BIT_RATE2, &drives, addrs);
		for(i = 0; i < drives; i++)
			printf("Drive found at address %d\n", addrs[i]);
	}
	if(error_code == DSP_SERCOS_127_FAILURE)
		print_phase2idns();
	if(error_code == DSP_SERCOS_128_FAILURE)
		print_phase3idns();
	error(error_code);

	enable_amplifier(AXIS);			/* enable servo drive */

	return 0;
}
