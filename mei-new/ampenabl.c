/* AMPENABL.C

:Demonstrates how to configure the dedicated amp enable outputs.

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
	Revision 1.1  1999/08/31 16:42:52  briegel
	source for MEI library

*/

# include <stdio.h>
# include <stdlib.h>
# include <conio.h>	  
# include "pcdsp.h"


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

int16 get_info(void)		  
{
   char  buffer[80];
   int16   generic;

   gets(buffer);  
   sscanf(buffer,"%d",&generic);
   return generic;
}

int16 enable_axis(int16 axis)
{
   int16      level;

   printf("\nEnter run time amp enable level for axis %d (1 = HIGH, 0 = LOW): ", axis);
   level = get_info();
   set_amp_enable_level(axis, level);
   printf("\nSet amp enable to disabled state for boot up? (y/n) ");
   if(getch() == 'y')
   {
      set_boot_amp_enable_level(axis, level);
      // NOTE: the call to set_boot_amp_enable() is not really needed since
      // set_boot_amp_enable_level() calls set_boot_amp_enable() with !level.
      set_boot_amp_enable(axis, !level);
      mei_checksum();
   }
   enable_amplifier(axis);
      
   return dsp_error;
}

int16 main()
{
   int16      i, level, bootlevel, state, bootstate;
   int16 error_code;
	
   error_code = do_dsp();	/* initialize communication with the controller */
   error(error_code);		/* any problems initializing? */

   for(i = 0; i < dsp_axes(); i++)
      if(enable_axis(i))
         printf("\nCould configure amp enable for axis %d.\n", i);
      else
      {
         printf("\nAmp enable configured for axis %d.\n", i);
         get_amp_enable_level(i, &level);
         printf("Runtime amp enable level is %s\n", level ? "HIGH" : "LOW");
         get_amp_enable(i, &state);
         printf("Runtime amp enable state is %s\n", state ? "HIGH" : "LOW");
         get_boot_amp_enable_level(i, &bootlevel);
         printf("Boot amp enable level is %s\n", bootlevel ? "HIGH" : "LOW");
         get_boot_amp_enable(i, &bootstate);
         printf("Boot amp enable state is %s\n", bootstate ? "HIGH" : "LOW");
      }

   return 0;
}
