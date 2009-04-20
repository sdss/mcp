/* SINIT4.C

:SERCOS loop initialization with Lutze ComCon, 16bit input and output modules.

This sample program demonstrates SERCOS loop initialization with a Lutze
 ComCon slave module.  Additionally, a 16bit input and a 16bit output module
 are mounted in the Lutze carrier unit (backplane).

The I/O modules are mapped into the cyclic data and updated by the DSP every
 SERCOS loop cycle.  The Inputs are mapped into the "AT" and the Outputs are
 mapped into the "MDT".  The cyclic data can be accessed from the host with
 the functions read_cyclic_at_data(...) and read/write_cyclic_mdt_data(...).


Here are the configurations:

SERCOS Baud Rate = 2Mbit

Lutze ComCon:  Axis = 0, SERCOS Address = 10
Lutze Input Module:  Address = 2066
Lutze Output Module:  Address = 2086

Note:  The Lutze I/O module addresses (IDNs in cyclic data) are switch selectable.
 Please consult the Lutze documentation for more details.

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
#include "sercrset.h"

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define NODES			1	/* Number of nodes on Sercos ring */
#define	AXIS			0
#define NODE_ADDR		10

#define AT_IDNS			1	/* Number of IDNs for the AT */
#define MDT_IDNS		1	/* Number of IDNs for the MDT */

/* Drive Configuration Data */
DRIVE_INFO drive_info[NODES] = {
/*	{axis, Sercos drive address, drive type, drive manufacturer} */
	{AXIS, NODE_ADDR, USERMAP, LUTZE}
};

/* AT Configuration data */
CYCLIC_DATA at_idns[AT_IDNS] = {
/* {idn, Sercos drive address} */
	{2066, NODE_ADDR}	/* IDN is the address of the Input module */
};

/* MDT Configuration Data */
CYCLIC_DATA mdt_idns[MDT_IDNS] = {
/* {idn, Sercos drive address} */
	{2086, NODE_ADDR}	/* IDN is the address of the Output module */
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
	int16 error_code, lutze_in, lutze_out;
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(configure_at_data(AT_IDNS, at_idns));		/* input data */
	error(configure_mdt_data(MDT_IDNS, mdt_idns));	/* output data */
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

	enable_amplifier(AXIS);					/* enable Lutze block */
	write_cyclic_mdt_data(AXIS, 0, 0x5555);	/* write the outputs */

	printf("Lutze I/O.\n");
	while (!kbhit())
	{	lutze_in = read_cyclic_at_data(AXIS, 0);	/* read inputs */
		lutze_out = read_cyclic_mdt_data(AXIS, 0);	/* read outputs */
		printf("\rIn:0x%4x Out:0x%4x", lutze_in, lutze_out);
	}
	getch();

	return 0;
}
