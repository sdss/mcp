/* SPEED.C

:Test function execution time.

This code uses the DSP's sample clock to calculate the execution time for
  several library functions.

Warning!  This is a sample program to assist in the integration of the
  DSP-Series controller with your application.  It may not contain all 
  of the logic and safety features that your application requires.
  
Written for Version 2.5  
*/

/*	Revision Control System Information
	$Source$ 
	$Revision$
	$Date$

	$Log$
	Revision 1.1  1999/08/31 16:43:57  briegel
	source for MEI library

*/

	
#	include <stdio.h>
#	include <stdlib.h>
#	include <dos.h>
#	include "pcdsp.h"


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

void calculate_time(int16 t1, int16 t2, int16 cycles)
{
	int16 delta, sample_rate;
	double e_time;

	sample_rate = dsp_sample_rate();
    delta = t2 - t1;
    e_time = 1.e6 * (delta)/((double)cycles * (double)sample_rate);
    printf("%d cycles in %d samples (%lf microseconds/cycle).\n\n",
    		cycles, delta, e_time);
}

void test_no_function(int16 cycles)
{
	int16 i, t1, t2;
	
    printf("\nBasic Cycle (no function called)\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
    	;
    	
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}	

void test_dsp_read_dm(int16 cycles)
{
	int16 i, t1, t2;
	
    printf("dsp_read_dm() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        dsp_read_dm(0x11F);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}	

void test_dsp_write_dm(int16 cycles)
{
	int16 i, t1, t2;
	
    printf("dsp_write_dm() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        dsp_write_dm(0x100,0);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}	

void test_get_io(int16 cycles)
{
	int16 i, t1, t2, value;
	
    printf("get_io() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        get_io(0, &value);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}	

void test_set_io(int16 cycles)
{
	int16 i, t1, t2;
	
    printf("set_io() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        set_io(0, 0);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}	

void test_get_command(int16 cycles)
{
	int16 i, t1, t2;
	double value;
	
    printf("get_command() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        get_command(0, &value);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    

void test_set_command(int16 cycles)
{
	int16 i, t1, t2;
	double value = 0.0;
	
    printf("set_command() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
    { 	set_command(0, value);
        value += 1.0;
	}        
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    

void test_get_position(int16 cycles)
{
	int16 i, t1, t2;
	double value;
	
    printf("get_position() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
        get_position(0, &value);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    

void test_set_position(int16 cycles)
{
	int16 i, t1, t2;
	double value = 0.0;
	
    printf("set_position() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
    { 	set_position(0, value);
        value += 1.0;
    }    
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    

void test_get_analog(int16 cycles)
{
	int16 i, t1, t2, value;
	
    printf("get_analog() function\n");

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
     	get_analog(0, &value);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    
        
void test_start_r_move(int16 cycles)
{
	int16 i, t1, t2;
	
    printf("start_r_move() function\n");
    error(dsp_reset());

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
     	start_r_move(0, 1000.0, 10000.0, 100000.0);
        
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    

void test_start_move(int16 cycles)
{
	int16 i, t1, t2;
	double value = 0.0;
	
    printf("start_move() function\n");
    error(dsp_reset());

    disable();
    t1 = dsp_read_dm(0x11F);
    
    for(i = 0; i < cycles; i++)
    { 	start_move(0, value, 10000.0, 100000.0);
        value += 1000.0;
	}
    
    t2 = dsp_read_dm(0x11F);
    enable();
    calculate_time(t1, t2, cycles);
}    


int16	main()
{
    int16 error_code;	
	
	error_code = do_dsp();		/* initialize communication with the controller */
	error(error_code);			/* any problems initializing? */
	error(dsp_reset());			/* hardware reset */

    printf("\nSpeeds measured with the DSP sample timer (%d Hz)\n", dsp_sample_rate());

	test_no_function(30000);
	
	test_dsp_read_dm(30000);
	test_dsp_write_dm(30000);
	
	test_get_io(30000);
	test_set_io(30000);
	
	test_get_command(500);
	test_set_command(500);
	
	test_get_position(500);
	test_set_position(500);
	
	test_get_analog(10000);
	
	test_start_r_move(100);
	test_start_move(100);

    return 0;
}
