#include <stdio.h>
#include "ip480.h"


/*
    DECLARE MODULES CALLED:
*/

    void trig480();     /* routine to software trigger the IP480 */
    void setconf();	/* routine to set up the configuration param. block */
    void selectcnt();   /* routine to examine/change the current counter */
    void selectcon();   /* routine to examine/change the counter constant*/
    void readstat();	/* routine which calls the Read Status Command */
    void clear_screen(); /* utility to clear the screen */
void c_isr(struct conf_blk *cblk);


/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       drvr480.c (generic version)

    MODULE NAME:    main - main routine of example software.

    VERSION:	    REV A

    CREATION DATE:  07/08/96

    DESIGNED BY:    R.H.

    CODED BY:       R.H.

    ABSTRACT:	    This module is the main routine for the example program
                    which demonstrates how the IP480 Library is used.

    CALLING
	SEQUENCE:

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:

	This module is the main routine for the example program
        which demonstrates how the IP480 Library is used.
*/

void ip480_test()
{

/*
    DECLARE EXTERNAL DATA AREAS:
*/

/*
    DECLARE LOCAL DATA AREAS:
*/

    char cmd_buff[32];		/* command line input buffer */
    unsigned finished;		/* flag to exit program */
    UWORD cont_reg;
    int item;                   /* menu item selection variable */
    int status;			/* returned status of driver routines */
    unsigned finished2;		/* flag to exit a loop */
    int addr;			/* integer to hold board address */
    int flag;			/* general flag for exiting loops */
    int i, j;                   /* loop index */
    int hflag;                  /* interrupt handler installed flag */
    ULONG ulong_value;
    struct conf_blk c_block;    /* configuration block */


/*
    ENTRY POINT OF ROUTINE:
    INITIALIZATION
*/

    flag = 0;		/* indicate board address not yet assigned */
    finished = 0;	/* indicate not finished with program */
    hflag = 0;          /* indicate interrupt handler not installed yet */
    
/*
    Initialize the Configuration Parameter Block to default values.
*/




/*
    Enter main loop
*/

    while(!finished)
    {
      printf("\n\nIP480 Library Demonstration  Rev. A\n\n");
      printf(" 1. Exit this Program\n");
      printf(" 2. Set Board Base Address\n");
      printf(" 3. Issue Software Reset to Board\n");
      printf(" 4. Read Module I.D./Display Event Status\n");
      printf(" 5. Examine/Change Current Counter\n");
      printf(" 6. Set Up Configuration Parameters\n");
      printf(" 7. Configure Control Register\n");
      printf("10. Software Trigger Control\n");
      printf("11. Display Counter Control Registers\n");
      printf("12. Display Read Back Registers\n");
      printf("13. Examine/Change Counter Constant\n");
      printf("14. Write Counter Constant Register\n");
      printf("15. Stop Counter\n");
      printf("16. Disable Interrupt\n");
      printf("\nSelect: ");
      rewind(stdin);
      scanf("%d",&item);

/*
    perform the menu item selected.
*/

      switch(item)
      {
	case 1: /* exit program command */
	    printf("Exit program(y/n)?: ");
	    scanf("%s",cmd_buff);
	    if( cmd_buff[0] == 'y' || cmd_buff[0] == 'Y' )
		finished++;
        break;

	case 2: /* set board address command */
            do {
                if(flag == 0)
		{
                    printf("\n\nenter base address of board in hex: ");
                    scanf("%x",&addr);
                }
                printf("address: %x\n",addr);
                printf("is this value correct(y/n)?: ");
                scanf("%s",cmd_buff);
                if( cmd_buff[0] == 'y' || cmd_buff[0] == 'Y' )
		{
/*
    Now store this address in the Configuration Block and Interrupt Handler
    Data structures.  This address will now be universally used when reading
    and writing the board's registers.
*/

                    c_block.brd_ptr = (BYTE*)addr;
                    
                    i = ((addr >> 8) & 3);	/* form IP position */
		    switch(i)		/* convert to IP under service code */
		    {
			case 1:
                                c_block.ip_pos = 4;   /* IPB0 under service */
			break;

			case 2:
                                c_block.ip_pos = 0x10; /* IPC0 under service */
			break;

			case 3:
                                c_block.ip_pos = 0x40; /* IPD0 under service */
			break;

			default:
                                c_block.ip_pos = 1;    /* IPA0 under service */
			break;
		    }
                    flag = 1;
                }
		else
                    flag = 0;
                
            }while( cmd_buff[0] != 'y' && cmd_buff[0] != 'Y' );
        break;


        case 3: /* Reset board */
            outpw(c_block.brd_ptr + ResetRegister, Reset);
        break;


        case 4:     /* read board I.D. command */
	    if(flag == 0)
		printf("\n>>> ERROR: BOARD ADDRESS NOT SET <<<\n");
	    else
                readstat(&c_block); /* read board status */
        break;


        case 5: /* counter number */
            selectcnt(&c_block);
        break;


        case 6: /* set up configuration block parameters */
	    setconf(&c_block);
        break;


        case 7: /* configure */
            ConfigureCounterTimer(&c_block, c_block.counter_num);
/*            printf("\n\nControl Word: %03x\n",build_control(&c_block)); */
        break;


        case 10: /* Software trigger control */
            trig480(&c_block);
        break;

        case 11: /* Read Counter Control registers */
            printf("\nCounter Control Registers = ");
               cont_reg = inpw(c_block.brd_ptr + CounterControl1);
               printf("%04X ",cont_reg);
               cont_reg = inpw(c_block.brd_ptr + CounterControl2);
               printf("%04X ",cont_reg);
               cont_reg = inpw(c_block.brd_ptr + CounterControl3);
               printf("%04X ",cont_reg);
               cont_reg = inpw(c_block.brd_ptr + CounterControl4);
               printf("%04X ",cont_reg);
               cont_reg = inpw(c_block.brd_ptr + CounterControl5);
               printf("%04X ",cont_reg);
               cont_reg = inpw(c_block.brd_ptr + CounterControl6);
               printf("%04X ",cont_reg);

            printf("\n");
        break;

        case 12: /* Read Read Back registers */
            printf("\nRead Back Registers = ");
            for(i = 1; i < 7; i++)
              {
               if(c_block.m_CounterSize[i] && (i == 1 || i == 3 || i == 5))
               {
               }
               else
               {
               ReadCounter(&c_block, i, &ulong_value);
               printf("%04X ",ulong_value);
               }
              }

            printf("\n");
        break;
  
        case 13: /* select counter constant */
            selectcon(&c_block);
        break;

        case 14: /* Write counter constant */
            WriteCounterConstant(&c_block, c_block.counter_num);
        break;

        case 15: /* Disable the current channel */
            StopCounter(&c_block, c_block.counter_num);
        break;

        case 16: /* Disable the interrupt on the current channel */
            DisableInterrupt(&c_block, c_block.counter_num);
        break;

        }   /* end of switch */
    }	/* end of while */

/*
    Reset board to disable interrupts from all counters on this IP module
*/
    if(flag != 0)		  /* module address was set */
      c_block.brd_ptr[ResetRegister] = 1;      /* reset board */

    printf("\nEXIT PROGRAM\n");
}   /* end of main */




/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       drvr480.c

    MODULE NAME:    get_param - get a parameter from the console

    VERSION:	    V1.0

    CREATION DATE:  3/24/89

    DESIGNED BY:    RTL

    CODED BY:	    RTL

    ABSTRACT:	    Routine which is used to get parameters

    CALLING
	SEQUENCE:   get_param();

    MODULE TYPE:    long

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


long get_param()
{

/*
    declare local storage.
*/

    long temp;

/*
    print prompt string
*/
    printf("enter hex parameter: ");

/*
    get input
*/
    scanf("%x",&temp);

/*
    output a linefeed
*/
    printf("\n");

    return(temp);
}

/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       ip480.c

    MODULE NAME:    setconf - set configuration block contents.

    VERSION:	    A

    CREATION DATE:  07/11/96

    DESIGNED BY:    FJM

    CODED BY:	    FJM

    ABSTRACT:	    Routine which is used to enter parameters into
		    the Configuration Block.

    CALLING
	SEQUENCE:   setconf(c_block)
		    where:
			c_block (structure pointer)
			  The address of the configuration param. block

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:     get_param()     input a parameter from console

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:
*/

void setconf(c_blk)
struct conf_blk *c_blk;
{

/*
    DEFINITIONS:
*/


/*
    DECLARE EXTERNAL DATA AREAS:
*/



/*
    DECLARE LOCAL DATA AREAS:
*/
    int status;
    int item;			/* menu item variable */
    int counter;              /* counter number */
    unsigned finished;        /* flag to exit loop */
    BYTE byte_value;          /* storage for retieved data */
    BOOL bool_value;          /* storage for retreived data */
/*
    DECLARE MODULES CALLED
*/
    long get_param();	/* input a parameter */

/*
    ENTRY POINT OF ROUTINE:
*/
    finished = 0;
    while(!finished)
    {
        counter = c_blk->counter_num;
        printf("\n\n\n\n");
        printf("\n\nCurrent Counter Configuration Parameters %x\n\n",counter);
	printf(" 1. Return to Previous Menu\n");
        printf(" 2. Board Pointer:         %x\n",c_blk->brd_ptr);
        status = GetMode(c_blk, counter, &byte_value);
        printf(" 3. Counter Mode:          %x\n",byte_value);
        status = GetOutputPolarity(c_blk, counter, &bool_value);
        printf(" 4. Output Polarity:       %x\n",bool_value);
        status = GetInputPolarity(c_blk, counter, &bool_value);
        printf(" 5. Input/Event Polarity:  %x\n",bool_value);
        status = GetTriggerSource(c_blk, counter, &byte_value);
        printf(" 6. Trigger Source:        %x\n",byte_value);
        status = GetCounterSize(c_blk, counter, &bool_value);
        printf(" 7. Counter Size:          %x\n",bool_value);
        status = GetClockSource(c_blk, counter, &byte_value);
        printf(" 8. Clock Source:          %x\n",byte_value);
        status = GetWatchdogLoad(c_blk, counter, &bool_value);
        printf(" 9. Watchdog Load:         %x\n",bool_value);
        status = GetInterruptVector(c_blk, &byte_value);
        printf("10. Interrupt Vector:      %x\n",byte_value);
        status = GetInterruptEnable(c_blk, counter, &bool_value);
        printf("11. Interrupt Enable:      %x\n",bool_value);
        status = GetDebounce(c_blk, counter, &bool_value);
        printf("12. Debounce:              %x\n",bool_value);
        printf("\nselect: ");
        rewind(stdin);
        scanf("%d",&item);
	switch(item)
	{
	case 1: /* return to previous menu */
	    finished++;
	    break;

	case 2: /* board address */
	    printf("ADDRESS CAN BE CHANGED ONLY IN THE MAIN MENU\n");
	    break;
 
        case 3: /* mode */
            clear_screen();
            printf("0 - Disabled\n");
            printf("1 - Output Pulse\n");
            printf("2 - Output Squarewave\n");
            printf("3 - Watchdog Timer\n");
            printf("4 - Input Event Counter\n");
            printf("5 - Input Pulse Width Measurement\n");
            printf("6 - Input Period Measurement\n");
            printf("7 - One-shot Output Pulse\n\n");
            byte_value = (BYTE)get_param();
            SetMode(c_blk, counter, byte_value);
	    break;

        case 4: /* output polarity */
            clear_screen();
            printf("0 - Output Polarity Low\n");
            printf("1 - Output Polarity High\n\n");
            bool_value = (BOOL)get_param();
            SetOutputPolarity(c_blk, counter, bool_value);
	    break;

        case 5: /* input polarity */
            clear_screen();
            printf("0 - Input Polarity Low\n");
            printf("1 - Input Polarity High\n\n");
            bool_value = (BOOL)get_param();
            SetInputPolarity(c_blk, counter, bool_value);
	    break;


        case 6: /* trigger source*/
            clear_screen();
            printf("0 - Internal Trigger\n");
            printf("1 - External Trigger, Polarity Low\n");
            printf("2 - External Trigger, Polarity High\n\n");
            byte_value = (BYTE)get_param();
            SetTriggerSource(c_blk, counter, byte_value);
            break;

        case 7: /* counter size */
            clear_screen();
            printf("0 - 16-bit Counter\n");
            printf("1 - 32-bit Counter\n\n");
            bool_value = (BOOL)get_param();
            SetCounterSize(c_blk, counter, bool_value);
	    break;

        case 8: /* clock source */
            clear_screen();
            printf("0 - Internal 1MHz Clock\n");
            printf("1 - Internal 4MHz Clock\n");
            printf("2 - Internal 8MHz Clock\n");
            printf("3 - External Clock\n\n");
            byte_value = (BYTE)get_param();
            SetClockSource(c_blk, counter, byte_value);
	    break;

        case 9: /* watchdog int./ext load */
            clear_screen();
            printf("0 - Internal (Auto) Load of Counter\n");
            printf("1 - External Triggered Load of Counter\n\n");
            bool_value = (BOOL)get_param();
            SetWatchdogLoad(c_blk, counter, bool_value);
	    break;

        case 10: /* interrupt vector */
            clear_screen();
            byte_value = (BYTE)get_param();
            SetInterruptVector(c_blk, byte_value);
	    break;

        case 11: /* interrupt enable */
            clear_screen();
            printf("0 - Interrupts Disabled\n");
            printf("1 - Interrupts Enabled\n\n");
            bool_value = (BOOL)get_param();
            SetInterruptEnable(c_blk, counter, bool_value);
            break;


        case 12: /* debounce */
            clear_screen();
            printf("0 - Debounce Disabled\n");
            printf("1 - Debounce Enabled\n\n");
            bool_value = (BOOL)get_param();
            SetDebounce(c_blk, counter, bool_value);
            break;

         }
    }
}



/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       ip480.c

    MODULE NAME:    readstat - read board status by using rmid480().

    VERSION:	    A

    CREATION DATE:  11/07/95

    DESIGNED BY:    FJM

    CODED BY:	    FJM

    ABSTRACT:	    Routine which is used to "Read Board Status" and to
		    print the results to the console.

    CALLING
        SEQUENCE:   readstat(&c_block)
		    where:
                        c_block (structure pointer)
                          The address of the configuration param. block

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:     get_param()     input a parameter from console.
                    rmid480()       Read module I.D..

   REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:
*/

void readstat(c_blk)
struct conf_blk *c_blk;
{

/*
    DEFINITIONS:
*/


/*
    DECLARE EXTERNAL DATA AREAS:
*/



/*
    DECLARE LOCAL DATA AREAS:
*/
    int item;		/* menu item variable */
    int i;		/* loop index */
    unsigned finished;  /* flags to exit loops */

/*
    DECLARE MODULES CALLED
*/
    void rmid480();     /* reads and stores the contents of the id prom */
    long get_param();	/* input a parameter */

/*
    ENTRY POINT OF ROUTINE:
*/

    rmid480(c_blk);            /* read the modules id prom */
    if(c_blk->id_prom[5] == 0x16)
     c_blk->num_chan = 6;
    else
     c_blk->num_chan = 2;

    finished = 0;
    while(!finished)
    {
        printf("Identification:         ");
	for(i = 0; i < 4; i++)		/* identification */
           printf("%c",c_blk->id_prom[i]);
        printf("\nManufacturer's I.D.:    %x",(BYTE)c_blk->id_prom[4]);
        printf("\nIP Model Number:        %x",(BYTE)c_blk->id_prom[5]);
        printf("\nRevision:               %x",(BYTE)c_blk->id_prom[6]);
        printf("\nReserved:               %x",(BYTE)c_blk->id_prom[7]);
        printf("\nDriver I.D. (low):      %x",(BYTE)c_blk->id_prom[8]);
        printf("\nDriver I.D. (high):     %x",(BYTE)c_blk->id_prom[9]);
        printf("\nTotal I.D. Bytes:       %x",(BYTE)c_blk->id_prom[10]);
        printf("\nCRC:                    %x",(BYTE)c_blk->id_prom[11]);
        printf("\nEvent Status:           %x",(BYTE)c_blk->event_status);


	printf("\n\n1 Return to Previous Menu\n");
        printf("2 Read Again\n");
        printf("3 Clear Event Status\n");
	printf("\nselect: ");
	scanf("%d",&item);

	switch(item){

	case 1: /* return to previous menu */
	    finished++;
	    break;

        case 3: /* clear out event status */
             c_blk->event_status = 0;
             break;

	}
    }
}


/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       ip480.c

    MODULE NAME:    trig480 - Software trigger for IP480.

    VERSION:	    A

    CREATION DATE:  07/11/96

    DESIGNED BY:    FJM

    CODED BY:	    FJM

    ABSTRACT:       Routine triggers counter/timers on the IP480.

    CALLING
        SEQUENCE:   trig480(c_block)
		    where:
			c_block (structure pointer)
			  The address of the configuration param. block

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:     get_param()     input a parameter from console

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:
*/

void trig480(c_blk)
struct conf_blk *c_blk;
{

/*
    DEFINITIONS:
*/


/*
    DECLARE EXTERNAL DATA AREAS:
*/



/*
    DECLARE LOCAL DATA AREAS:
*/
    int item, trig;             /* menu item variable */
    unsigned finished;          /* flag to exit loop */

/*
    DECLARE MODULES CALLED
*/
    long get_param();	/* input a parameter */

/*
    ENTRY POINT OF ROUTINE:
*/
    finished = 0;
    while(!finished)
    {
        printf("\n\nSoftware Triggering Options\n\n");
	printf(" 1. Return to Previous Menu\n");
        printf(" 2. Trigger Single Counter/Timer\n");
        printf(" 3. Trigger Multiple Counter/Timers\n");
        printf("\nselect: ");
        rewind(stdin);
        scanf("%d",&item);
	switch(item)
	{
	case 1: /* return to previous menu */
	    finished++;
	    break;

        case 2: /* Single trigger */
            printf("Enter Counter Number to Trigger (1 - 6): ");
            rewind(stdin);
            scanf("%d",&trig);
          StartCounter(c_blk, trig);
	    break;

        case 3: /* Multiple trigger */
            printf("Enter Hex Mask for Multiple Trigger: ");
            rewind(stdin);
            scanf("%x",&trig);
            StartSimultaneousCounters(c_blk, (BYTE)trig);
	    break;
         }
    }
}


/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       ip480.c

    MODULE NAME:    selectcnt - Select counter for IP480.

    VERSION:	    A

    CREATION DATE:  07/11/96

    DESIGNED BY:    FJM

    CODED BY:	    FJM

    ABSTRACT:       Routine selects counter/timers on the IP480.

    CALLING
        SEQUENCE:   selectcnt(c_block)
		    where:
			c_block (structure pointer)
			  The address of the configuration param. block

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:     get_param()     input a parameter from console

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:
*/

void selectcnt(c_blk)
struct conf_blk *c_blk;
{

/*
    DEFINITIONS:
*/


/*
    DECLARE EXTERNAL DATA AREAS:
*/



/*
    DECLARE LOCAL DATA AREAS:
*/
    int item, cntr;             /* menu item variable */
    unsigned finished;          /* flag to exit loop */

/*
    DECLARE MODULES CALLED
*/
    long get_param();	/* input a parameter */

/*
    ENTRY POINT OF ROUTINE:
*/
    finished = 0;
    while(!finished)
    {
        printf("\n\nCurrent Counter: %x\n\n",c_blk->counter_num);
	printf(" 1. Return to Previous Menu\n");
        printf(" 2. Change Counter\n");
        printf("\nselect: ");
        rewind(stdin);
        scanf("%d",&item);
	switch(item)
	{
	case 1: /* return to previous menu */
	    finished++;
	    break;

        case 2: /* Select counter */
            printf("Enter New Counter Number (1 - 6): ");
            rewind(stdin);
            scanf("%d",&cntr);
            c_blk->counter_num = (BYTE)(cntr);
	    break;
         }
    }
}


/*
{+D}
    SYSTEM:         IP480 Software

    FILENAME:       ip480.c

    MODULE NAME:    selectcon - Select counter constant for IP480.

    VERSION:	    A

    CREATION DATE:  07/11/96

    DESIGNED BY:    FJM

    CODED BY:	    FJM

    ABSTRACT:       Routine selects counter constants for the IP480.

    CALLING
        SEQUENCE:   selectcon(c_block)
		    where:
			c_block (structure pointer)
			  The address of the configuration param. block

    MODULE TYPE:    void

    I/O RESOURCES:

    SYSTEM
	RESOURCES:

    MODULES
	CALLED:     get_param()     input a parameter from console

    REVISIONS:

  DATE	  BY	    PURPOSE
-------  ----	------------------------------------------------

{-D}
*/


/*
    MODULES FUNCTIONAL DETAILS:
*/

void selectcon(c_blk)
struct conf_blk *c_blk;
{

/*
    DEFINITIONS:
*/


/*
    DECLARE EXTERNAL DATA AREAS:
*/



/*
    DECLARE LOCAL DATA AREAS:
*/
    int item;                   /* menu item variable */
    int counter;                /* counter number stroage */
    unsigned finished;          /* flag to exit loop */
    unsigned long con;
    ULONG value;
/*
    DECLARE MODULES CALLED
*/
    long get_param();	/* input a parameter */

/*
    ENTRY POINT OF ROUTINE:
*/
    finished = 0;
    while(!finished)
    {
      counter = c_blk->counter_num;
      printf("\n\nCurrent Counter:          %x\n",counter);
      GetCounterConstant(c_blk, counter, &value);
      printf("Current Contents of Counter Constant Register: %x\n\n", value);

      printf(" 1. Return to Previous Menu\n");
      printf(" 2. Change Constant\n");
      printf("\nselect: ");
      rewind(stdin);
      scanf("%d",&item);
      switch(item)
      {
      case 1: /* return to previous menu */
          finished++;
      break;

      case 2: /* Select constant */
      clear_screen();
      value = (ULONG)get_param();
      SetCounterConstant(c_blk, counter, value);
      break;
      }
    }
}



/*
{+D}
        SYSTEM:         VME TEST FIXTURE

        MODULE NAME:    clear_screen - clear the CRT screen

        VERSION:        U1.0

        CREATION DATE:  01/13/89

        DESIGNED BY:    Frank Musatics

        CODED BY:       Frank Musatics

        ABSTRACT:       Clears the CRT screen by sending 24 line feeds

        CALLING
          SEQUENCE:     void = clear_screen()

        MODULE TYPE:    void

        I/O
          RESOURCES:

        SYSTEM
          RESOURCES:

        MODULES
          CALLED:

        REVISIONS:

  DATE      BY     PURPOSE
---------  ----   -------------------------------------------------------


{-D}
*/

/*
        MODULES FUNCTIONAL DETAILS:

        Clears the CRT screen by sending 24 line feeds
*/


void clear_screen()


  {



/*
        Declare external data areas
*/


/*
        Declare local data areas
*/


/*
        Declare modules called
*/


/*
        Entry point
*/

   printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

}
struct conf_blk cbrd;

test_interrupt (char *addr, int counter)
{
  cbrd.brd_ptr=(BYTE *)addr;
  SetInterruptVector (&cbrd,0xB4);
  attach_ihandler (0,cbrd.m_InterruptVector,0,c_isr,(struct handler_data *)&cbrd);
  SetCounterConstant (&cbrd,counter,16667*2);
  SetMode (&cbrd,counter,Watchdog);
  SetDebounce (&cbrd,counter,DebounceOff);
  SetInterruptEnable(&cbrd,counter,IntEnable);
  SetCounterSize (&cbrd,counter,CtrSize16);
  SetClockSource (&cbrd,counter,InC1Mhz);
  SetTriggerSource (&cbrd,counter,InTrig);
  SetWatchdogLoad (&cbrd,counter,WDIntLd);
  SetOutputPolarity (&cbrd,counter,InPolLow);
  ConfigureCounterTimer(&cbrd,counter);
 /* VIPC610_IP_Interrupt_Enable (addr, 0, 3);*/
/*  VMESC5_IP_Interrupt_Enable (addr, 0, 3);*/
  VME162_IP_Interrupt_Enable (addr, 2, 3);
  sysIntEnable (3);
}
int wdog=0;
void c_isr(struct conf_blk *cblk)
{
struct map96x0 *carrier;        /* pointer to carrier base address */
int i,j;
UWORD i_stat;
UWORD i_stat_overall;

 i_stat = inpw(cblk->brd_ptr + InterruptPending);
  if(cblk->num_chan == 2)   /* check if it's a 2 or 6 channel bo */
   {
    i_stat &= 0x0300;       /* and off the unused upper bits */
    j = 2;
   }
  else
   {
    i_stat &= 0x3F00;        /* and off the unused bits and save the */
    j = 6;
   }


  if( i_stat )               /* any */
  {
           cblk->event_status |= (BYTE)(i_stat >> 8 );   /* update event */

        /* service the hardware */
    logMsg ("\r\nInterrupt Watchdog %d",wdog++);
        /* check each bit for an interrupt pending state */
    for( i = 0; i < j; i++ )   /* check each c */
    {
          if( i_stat & (1 << (i + 8)) )        /* build interr */
                  i_stat &= (~(1 << (i + 8))); /* clear interr */
          else
                  i_stat |= (1 << (i + 8));    /* set */
    }
    outpw(cblk->brd_ptr + InterruptPending, i_stat);  /* write interrupt pe */
  }
}
