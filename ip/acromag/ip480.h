/*
{+D}
    SYSTEM:         Software for IP480
    
    FILE NAME:      ip480.h

    VERSION:	    A

    CREATION DATE:  07/08/96

    DESIGNED BY:    F.J.M.

    CODED BY:	    F.J.M.

    ABSTRACT:	    This module contains the definitions and structures
                    used by the IP480 library.

    CALLING
	SEQUENCE:

    MODULE TYPE:    header file

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

    This module contains the definitions and structures used by
    the IP480 library.
*/

/*
    DEFINITIONS:
*/

#define TRUE 1
#define FALSE 0

typedef unsigned short UWORD;
typedef unsigned char BYTE;
/*
typedef int BOOL;
typedef unsigned long ULONG;
*/


typedef enum
{
	Success = 0,
	ParameterOutOfRange = 1,	/* Parameter in error */
	InvalidPointer = 2,			/* Flag NULL pointers */
	DataDirection = 3,			/* data direction error */
	TimeOut = 4					/* time-out error */
} IPSTAT;

/*
	Mode and option selection enumerations
*/

enum
	{
		None	  = 0,          /* disable counter */
		OutPulse  = 1,          /* output pulse waveform */
		OutSquare = 2,          /* output square waveform */
		Watchdog  = 3,          /* watchdog function */
		InEvent   = 4,          /* input event */
		InWidth   = 5,          /* input width */
		InPeriod  = 6,          /* input period */
		OneShot   = 7,          /* One-Shot output pluse */

		OutPolLow = 0,          /* output polarity active low */
		OutPolHi  = 1,          /* output polarity active high */

		InPolLow  = 0,          /* input polarity active low */
		InPolHi   = 1,          /* input polarity active high */

		InTrig    = 0,          /* internal trigger */
		ExtPolLow = 1,          /* external trigger polarity */
		ExtPolHi  = 2,          /* external trigger polarity */

		IntDisable= 0,          /* disable interrupt */
		IntEnable = 1,          /* interrupt enabled */

		CtrSize16 = 0,          /* counter 16 bit */
		CtrSize32 = 1,          /* counter 32 bit */

		InC1Mhz   = 0,          /* internal 1MHZ clock */
		InC4Mhz   = 1,          /* internal 4MHZ clock */
		InC8Mhz   = 2,          /* internal 8MHZ clock */
		ExClock   = 3,          /* external clock */

		WDIntLd   = 0,          /* watchdog timer autoloaded (internal ) */
		WDExtLd   = 1,          /* watchdog loaded when externally triggered */

		Latch32   = 1,          /* latch lower 16 bits on read of 32 bit reg */

		Reset	  = 1,

		LatchOn	  = 1,			/* 32 bit data latch enabled */
		LatchOff  = 0,			/* 32 bit data latch disabled */

		DebounceOn	= 1,		/* Debounce enabled */
		DebounceOff = 0			/* Debounce disabled */
	};





enum	/* IP480 direct register offsets in bytes */
	{
		CounterControl1  = 0,
		CounterControl2  = 2,
		CounterControl3  = 4,
		CounterControl4  = 6,
		CounterControl5  = 8,
		CounterControl6  = 10,
		
		CounterReadBack1 = 12,
		CounterReadBack2 = 14,
		CounterReadBack3 = 16,
		CounterReadBack4 = 18,
		CounterReadBack5 = 20,
		CounterReadBack6 = 22,

		CounterConstant1 = 24,
		CounterConstant2 = 26,
		CounterConstant3 = 28,
		CounterConstant4 = 30,
		CounterConstant5 = 32,
		CounterConstant6 = 34,

		TriggerControl   = 36,	/* bits 05...00 */
		ResetRegister	 = 38,	/* bit 0 */
		ReadBackLatch    = 40,
		InterruptPending = 42,	/* bits13...08 */
		InterruptVectorReg = 44
	};

struct conf_blk
 {
    BYTE *brd_ptr;               /* base address of board */
    UWORD m_CounterConstant[7];/* constant registers are write only copies are here */
    BYTE m_Mode[7];					/* the counter mode */
    BYTE m_Debounce[7];				/* only 1 & 2 have debounce */
    BOOL m_OutputPolarity[7];		/* output polarity */
    BOOL m_InputPolarity[7];		/* input polarity */
    BYTE m_Trigger[7];				/* triggering internal/external-polarity */
    BOOL m_CounterSize[7];			/* 16/32 bit counter size flags */
    BOOL m_ClockSource[7];			/* clock source */
    BOOL m_WDLoad[7];			    /* watchdog timer internal/external loaded */
    BOOL m_InterruptEnable[7];		/* interrupt enable */
    BYTE m_InterruptVector;			/* interrupt vector register */
    BYTE event_status;  /* interrupt event status */
    int num_chan;       /* is this a 2 or 6 channel board */
    BYTE id_prom[33];	/* id prom contents used by read status */
    BYTE ip_pos;        /* IP under service position */
    BYTE counter_num;   /* counter being serviced */
};

struct handler_data {
   int h_pid;
   char* hd_ptr;
   };

/* function protypes - FNAL */
IPSTAT GetInterruptVector(struct conf_blk *c_blk, BYTE *vector);
IPSTAT SetInterruptVector(struct conf_blk *c_blk, BYTE *vector);
IPSTAT ReadCounter(struct conf_blk *c_blk, int counter, ULONG val);
IPSTAT GetCounterConstant(struct conf_blk *c_blk, int counter, ULONG val);
IPSTAT SetCounterConstant(struct conf_blk *c_blk, int counter, ULONG val);
IPSTAT WriteCounterConstant(struct conf_blk *c_blk, int counter);
IPSTAT GetMode(struct conf_blk *c_blk, int counter, BYTE *mode);
IPSTAT SetMode(struct conf_blk *c_blk, int counter,  BYTE mode);
IPSTAT GetDebounce(struct conf_blk *c_blk, int counter, BOOL *debounce);
IPSTAT SetDebounce(struct conf_blk *c_blk, int counter, BOOL debounce);
IPSTAT GetInterruptEnable(struct conf_blk *c_blk, int counter, BOOL *enable);
IPSTAT SetInterruptEnable(struct conf_blk *c_blk, int counter, BOOL enable);
IPSTAT GetCounterSize(struct conf_blk *c_blk, int counter, BOOL *size);
IPSTAT SetCounterSize(struct conf_blk *c_blk, int counter, BOOL size);
IPSTAT GetClockSource(struct conf_blk *c_blk, int counter, BYTE *source);
IPSTAT SetClockSource(struct conf_blk *c_blk, int counter, BYTE source);
IPSTAT GetTriggerSource(struct conf_blk *c_blk, int counter, BYTE *trigger);
IPSTAT SetTriggerSource(struct conf_blk *c_blk, int counter, BYTE trigger);
IPSTAT GetWatchdogLoad(struct conf_blk *c_blk, int counter, BOOL *load);
IPSTAT SetWatchdogLoad(struct conf_blk *c_blk, int counter, BOOL load);
IPSTAT GetOutputPolarity(struct conf_blk *c_blk, int counter, BYTE *polarity);
IPSTAT SetOutputPolarity(struct conf_blk *c_blk, int counter, BYTE polarity);
IPSTAT GetInputPolarity(struct conf_blk *c_blk, int counter, BYTE *polarity);
IPSTAT SetInputPolarity(struct conf_blk *c_blk, int counter, BYTE polarity);
IPSTAT StopCounter(struct conf_blk *c_blk, int counter);
IPSTAT DisableInterrupt(struct conf_blk *c_blk, int counter);
IPSTAT StartCounter(struct conf_blk *c_blk, int counter);
IPSTAT StopCounter(struct conf_blk *c_blk, int counter);
IPSTAT StartSimultaneousCounters(struct conf_blk *c_blk, BYTE mask);
IPSTAT ConfigureCounterTimer(struct conf_blk *c_blk, int counter);
