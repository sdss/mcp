/* SDIAG.C

:SERCOS drive fault diagnostics/recovery.

This sample program demonstrates how to read a SERCOS drive's Drive Status
 and diagnostic messages.  If the drive is shutdown due to an error in the
 Class 1 Diagnostic, then a drive reset/enable is attempted.

Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before reading the drive diagnostic information.

Written for Motion Library Version 2.5  
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "sercos.h"

#define AXIS			0	/* Controller Axis */
#define DRIVE_ADDRESS	1	/* SERCOS Node Address */

#define CLASS_1_ERROR	0x2000	/* bit 13 */
#define CLASS_2_ERROR	0x1000	/* bit 12 */

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
	int16 error_code, code, status;
	char message[MAX_ERROR_LEN];
	
	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(get_drive_status(AXIS, &status));
	printf("\nDrive Status:0x%x", status);
	
	/* Operating Status Drive Message */
	error(get_idn_string(AXIS, DRIVE_ADDRESS, 95, message));
	printf("\nDrive Msg: %s", message);

	if (status & CLASS_1_ERROR) 
	{	error(disable_amplifier(AXIS));		/* disable SERCOS drive */
		error(get_class_1_diag(AXIS, &code, message));
		printf("\nClass 1:(0x%x) %s", code, message);

		error(reset_sercos_drive(AXIS));	/* clear Class 1 Diagnostic Error */
		error(controller_run(AXIS));		/* clear controller Abort Event */
		error(enable_amplifier(AXIS));		/* enable SERCOS drive */
		printf("\nDrive Re-enabled");
	}	
	if (status & CLASS_2_ERROR) 
	{ 	error(get_class_2_diag(AXIS, &code, message));
		printf("\nClass 2:(0x%x) %s", code, message);
	}	

	/* Operating Status Drive Message */
	error(get_idn_string(AXIS, DRIVE_ADDRESS, 95, message));
	printf("\nDrive Msg: %s", message);

	return 0;
}
