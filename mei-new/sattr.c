/* SATTR.C

:SERCOS IDN attributes.

This sample program demonstrates how to read an IDN's attributes from a 
 SERCOS node.  The attributes consist of 7 elements defined in the SERCOS
 specification:

Element	1:
 "ID Number"
 size is 16 bits, binary
 manadatory 

Element 2:
 "Name of operation data"
 maximum size is 64 bytes
 upper 4 bytes specify the string size
 lower 60 bytes are the string
 optional

Element 3:
 "Attribute of operation data"
 size is 4 bytes, binary
 bits 0-15 represent the converion factor
 bits 16-18 represent the data length
 bit 19 represents the function of operation data
 bits 20-22 represent the data type and display format
 bits 24-27 represent places after the decimal point
 bits 28-31 reserved
 mandatory

Element 4:
 "Operation data unit"
 maximum size is 16 bytes, binary/string
 bytes 1-2 specify length of programmed text in drive
 bytes 3-4 indicate maximum text length (in bytes) in the drive
 bytes 4-12 variable length character text string
 optional

Element 5:
 "Minimum input value of operation data"
 size is the same as Element 7 (Operation Data), binary
 optional

Element 6:
 "Maximum input value fo operation data"
 size is the same as Element 7 (Operation Data), binary
 optional

Element 7:
 "Operation data"
 maximum size is 65532 bytes, binary and/or string
 Fixed length: 
 	bytes 1-2, operation data, 2 bytes
	bytes 1-4, operation data, 4 bytes
 Variable length:
	bytes 1-2, specify length of programmed data (bytes) in drive
	bytes 3-4, indicate maximum data length available (bytes) in drive
	bytes 5-length, operation data
   
Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before reading an IDN's attributes.

Written for Motion Library Version 2.5  
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "pcdsp.h"
#include "sercos.h"

#define AXIS			0	/* Controller Axis */
#define DRIVE_ADDRESS	1	/* SERCOS Node Address */
#define IDN				79	/* Rotational Position Resolution */


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
	int16 error_code, size = 0;
	char * endp;
	long *buff, exp, proc, type;
	IDN_ATTRIBUTES attr;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	error(get_idn_attributes(AXIS, IDN, &attr));

	printf("\nidn: %u (0x%X)\n", IDN, IDN);
	printf("%s-%d-%d\n", attr.elem_1 & 0x8000 ? "P":"S",
		(attr.elem_1 & 0x7000)>>12, attr.elem_1 & 0xFFF);

	printf("%s\n", attr.elem_2);

	if(((attr.elem_3>>16) & 7) == 1)
		size = 2;
	if(((attr.elem_3>>16) & 7) == 2)
		size = 4;
	if(size)
		printf("Size: %d bytes\n", size);
	else
		printf("Size: Variable length string\n");

	proc = attr.elem_3 & 0x80000;
	printf("Data Type: %s\n", proc?"Procedure Command":"Operation Data or Parameter");
	if(!proc)
	{	exp = (attr.elem_3>>24) & 0xF;
		printf("Conversion Factor: %ldx10e%ld\n", (attr.elem_3&0xFFFF), exp);
		printf("Units: %s\n", attr.elem_4);
		printf("Minumum Value: %ld\n", attr.elem_5);
		printf("Maximum Value: %ld\n", attr.elem_6);
	}

	return 0;
}

