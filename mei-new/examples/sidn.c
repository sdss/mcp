/* SIDN.C

:SERCOS sample to decode an IDN's value(s) based on its attributes.

This sample demonstrates how to read the values for an IDN.  Each
 IDN contains 7 elements (defined in the SERCOS specification):

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

For more detailed information regarding the IDN elements, please consult
 the SERCOS specification.

Be sure to initialize the SERCOS communication ring with serc_reset(...)
 before reading an IDN.

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

//#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
//#endif

#define BUFFER_SIZE		1024


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

int16 print_list(int16 axis, unsigned16 idn)
{	int16 i, len, p, a=0;
	unsigned16 *buffer, channel, str[MAX_E2_INTS];

	if(get_sercos_channel(axis, &channel))
		return dsp_error;	

	buffer = (unsigned16*) calloc(BUFFER_SIZE, sizeof(unsigned16));
	if(read_idn(channel, idn, &len, buffer, TRUE))
		return dsp_error;

	printf("IDN-List for axis %d, IDN %d\n", axis, idn);
	for(i = 0; i < len; i++)
	{	if(get_element_2(channel, buffer[i], str))
			memcpy(str, "No text available", 20);
		p = buffer[i] & 0x8000;
		if(p)
			buffer[i] -= (int16)0x8000;
		printf("%s-%1d-%4d\t%s\n",p?"P":"S", a, buffer[i], str);
	}
	return DSP_OK;
}

int16 print_long_list(int16 axis, unsigned16 idn)
{	int16 i, len;
	unsigned16 channel;
	unsigned32 *buffer;

	if(get_sercos_channel(axis, &channel))
		return dsp_error;	

	buffer = (unsigned32*) calloc(BUFFER_SIZE, sizeof(unsigned32));
	if(read_idn(channel, idn, &len, (unsigned16*)buffer, TRUE))
		return dsp_error;

	for(i = 0; i < len/2; i++)
		printf("%8ld\n",buffer[i]);

	return DSP_OK;
}

int16 print_data_type(IDN_ATTRIBUTES * attr)
{	long proc, type;

	proc = (attr->elem_3 & 0x80000);		/* bit 19 indicates IDN is a Procedure */
	printf("Data function: %s\n", proc?"Procedure Command":"Operation Data or Parameter");
	type = (attr->elem_3>>20) & 0x7L;	/* bit 20-22 represent data type */
		
	switch (type)
	{
		case 0L:
			printf("Data type: binary\n");
			break;

		case 1L:
		case 3L:
			printf("Data type: unsigned integer\n");
			break;

		case 2L:
			printf("Data type: signed integer\n");
			break;

		case 4L:
			printf("Data type: text\n");
			break;
			
		case 5L:
			printf("Data type: ID Number (IDN)\n");
			break;

		default:
			printf("Data type: undefined\n");
	}
	return DSP_OK;
}

int16 print_element_data(int16 axis, unsigned16 idn, int16 size, IDN_ATTRIBUTES * attr)
{	long lval, exp;
	unsigned16 channel, dr_addr;
	char str[200];

	switch (size)
	{
		case 1:			/* fixed data length (2 or 4 bytes) */
		case 2:
			printf("Fixed data length is: %d bytes\n", (size * 2));
			print_data_type(attr);
			exp = (attr->elem_3>>24) & 0xF;	/* bits 24-27 represent decimal places */
			printf("Conversion Factor: %ldx10e-%ld\n", (attr->elem_3 & 0xFFFF), exp);
			printf("Units: %s\n", attr->elem_4);
			printf("Minumum Value: %ld\n", attr->elem_5);
			printf("Maximum Value: %ld\n", attr->elem_6);
			while(!kbhit())
			{	error(get_idn(axis, idn, &lval));
				printf("Value: %10ld (0x%lX)        \r", lval, lval);
			}
			getch();
			break;

		case 4:			/* String */
			printf("Variable data length with 1-byte data strings.\n");
			if(get_sercos_channel(axis, &channel))
				return dsp_error;
			dr_addr = dspPtr->sercdata[channel].drive_addr;
			if(get_idn_string(axis, dr_addr, idn, str))
				return dsp_error;
			printf("String: %s\n", str);
			break;

		case 5:			/* IDN String */
			printf("Variable data length with 2-byte data strings.\n");
			error(print_list(axis, idn));
			break;

		case 6:			/* String (long) */
			printf("Variable data length with 4-byte data strings.\n");
			error(print_long_list(axis, idn));
			break;

		default:
			printf("Data length is undefined\n");
	}
	return DSP_OK;
}

int main(int argc, char* argv[])
{	int16 axis, error_code, size = 0;
	unsigned16 idn;
	char *endp;
	IDN_ATTRIBUTES attr;

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */

	if(argc < 3)
	{	printf("USAGE: SIDN [axis] [idn]");
		return 1;
	}
	axis = (int16) strtol(argv[1], &endp, 10);
	if(!strnicmp("0x", argv[2], 2))
		idn = (unsigned16) strtol(argv[2], &endp, 16);
	else
		idn = (unsigned16) strtol(argv[2], &endp, 10);

	printf("IDN: %u (0x%X)\n", idn, idn);
	error(get_idn_attributes(axis, idn, &attr));	/* read the elements */

	/* Element 1 specifies the ID number.  "P" indicates product specific,
		"S" indicates standard. */		
	printf("%s-%d-%d\n", attr.elem_1 & 0x8000 ? "P":"S",
		(attr.elem_1 & 0x7000) >> 12, attr.elem_1 & 0xFFF);
	printf("Name: %s\n", attr.elem_2);		/* Name of operation data */

	size = ((attr.elem_3 >> 16) & 0x7);	/* bits 16-18 represent data length */
	error(print_element_data(axis, idn, size, &attr));

	return DSP_OK;
}
