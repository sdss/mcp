/* SEM.C

:Multi-tasking and interrupts under Windows NT.

This sample demonstrates how to create a separate thread for each axis.
 The number	of axes is specified by the define AXES.  Each thread will
 enter an efficient wait state until an interrupt that is generated from
 an event occurs.  The thread will then clear the event by calling
 controller_run(...) and then go back to sleep until the next interrupt
 occurs.

In order for threads to use the mei_sleep_until_interrupt(...) function
 without error, each thread MUST use its own HANDLE to the DSPIO driver
 in the function call.  The HANDLE is created with a call to CreateFile(...).

Like mei_sleep_until_interrupt(), all other library functions must have a
 HANDLE to the DSPIO driver.  This HANDLE is created when either dsp_init(...)
 or do_dsp(...) are called.  This HANDLE is held by the library and is used
 for all library access to the MEI card.

This code uses a global semaphore (MEI_SEMAPHORE) to gate access to 
 the library so that only one thread is allowed to communicate with the 
 MEI card at a time.  The semaphre is created by a call to
 create_MEI_semaphore(...) and destroyed by a call to kill_MEI_semaphore(...).

All calls to the MEI library are bracketed by calls	to get_MEI_semaphore(...)
 and release_MEI_semaphore(...).

When using interrupts, be sure to set the appropriate IRQ switch on the
 DSP-Series controller.  Also, make sure the device driver "DSPIO" is 
 configured for the same IRQ.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/

#include <windows.h>
#include <winioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "idsp.h"

#ifdef MEI_MSVC20		/* support for Microsoft Visual C/C++ ver 2.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#ifdef MEI_MSVC40		/* support for Microsoft Visual C/C++ ver 4.0 */
#	include "medexp.h"	/* prototypes for access to DLL's internal data */
#endif

#define AXES	(int16)4			/* This also defines the number of
										threads to be created */
#define	SEM_MAXIMUM_USAGE				1L
#define	SEM_INITIAL_SIGNALED			1L
#define	SEM_INITIAL_NON_SIGNALED		0L
#define	SEM_INCREMENT					1L
#define	THREAD_STACK_SIZE				0
#define	THREAD_CREATION_FLAGS			0
#define	EXIT_CODE_OK					0

/* Globals */
HANDLE			MEI_SEMAPHORE;		/* Handle to library semaphore */
HANDLE			hThreads[AXES];		/* Handles to each created task */

typedef struct interrupt_thread_args{
	int16		axis;		 
	HANDLE		semaphore;		 
}InterruptThreadArgs;
/* create one InterruptThreadArgs structure for each axis */
InterruptThreadArgs	ThreadArgs[AXES];


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

/* Create the semaphore object.  This semaphore will allow access to the
	MEI library for one task at time.  All other tasks will wait on the
	semaphore until it is available.  The initial state of the semaphore
	is signaled. */
HANDLE create_MEI_semaphore(void)
{	return CreateSemaphore(NULL, SEM_INITIAL_SIGNALED, 
		SEM_MAXIMUM_USAGE, "MEI_semaphore");
}

/* Terminate the semaphore object */
int16 kill_MEI_semaphore(HANDLE sem)
{	return (int16)CloseHandle(sem);
}

/* Wait for the semaphore to be signaled and set the semaphore to a 
	non-signaled state */
int16 get_MEI_semaphore(HANDLE sem)
{	return (int16)WaitForSingleObject(sem, INFINITE);
}

/* Release the semaphore by incrementing it to a signaled state */
int16 release_MEI_semaphore(HANDLE sem)
{	return ReleaseSemaphore(sem, SEM_INCREMENT, NULL);
}

void MEI_InterruptThread(void *args)
{	InterruptThreadArgs	*pArgs;
	HANDLE					thread_semaphore;
	int16						axis;
	HANDLE					file_ptr;

	pArgs = (InterruptThreadArgs *)args;
	thread_semaphore = pArgs->semaphore;
	axis = pArgs->axis;
	
	/* create a separate file pointer to the driver for each task */
	file_ptr = CreateFile("\\\\.\\dspio0", GENERIC_WRITE|GENERIC_READ,
					FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

	/* set thread_semaphore to a signaled state so that the rest of 
		the program can resume */
	ReleaseSemaphore(thread_semaphore, SEM_INCREMENT, NULL);

	while(1)
 	{	mei_sleep_until_interrupt(file_ptr, axis, MEI_EVENT);
		get_MEI_semaphore(MEI_SEMAPHORE);
			while(controller_run(axis))
				;
		release_MEI_semaphore(MEI_SEMAPHORE);
	}
}
	 
void setup(int16 axis)
{	unsigned long ThreadId;
	
	get_MEI_semaphore(MEI_SEMAPHORE);
		controller_run(axis);	
		set_home_level(axis, FALSE);
		set_home(axis, ABORT_EVENT);
		clear_status(axis);
		interrupt_on_event(axis, TRUE);
	release_MEI_semaphore(MEI_SEMAPHORE);


	ThreadArgs[axis].axis = axis;
	/* create a semaphore to wait for created thread to finish initializing */
	ThreadArgs[axis].semaphore = CreateSemaphore(NULL, SEM_INITIAL_NON_SIGNALED, 
		SEM_MAXIMUM_USAGE, NULL); 

	/* create thread
		default security descriptor, default stack size (same as creating thread)*/
	hThreads[axis] = CreateThread(NULL, THREAD_STACK_SIZE, 
		(LPTHREAD_START_ROUTINE)MEI_InterruptThread, &ThreadArgs[axis], 
		THREAD_CREATION_FLAGS, &ThreadId);

	/* wait for created thread to finish initializing.  This occurs when
		MEI_InterruptThread releases the thread_semaphore. */
	WaitForSingleObject(ThreadArgs[axis].semaphore, INFINITE);

	/* semaphore to created thread is no longer needed */
	CloseHandle(ThreadArgs[axis].semaphore);
}

int main()
{	int16 error_code, axis, state[AXES], source[AXES];

	MEI_SEMAPHORE = create_MEI_semaphore();

	get_MEI_semaphore(MEI_SEMAPHORE);

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	init_io(PORT_A, IO_INPUT);	/* initialize I/O port for inputs */
	init_io(PORT_B, IO_INPUT);
	init_io(PORT_C, IO_OUTPUT);	/* initialize I/O port for outputs */

	reset_bit(23);				/* enable interrupt generation */
	release_MEI_semaphore(MEI_SEMAPHORE);

	for(axis = 0; axis < (int16)AXES; axis++)
	  	setup(axis);

	while(!kbhit())
	{	for(axis = 0; axis < AXES; axis++)
		{	get_MEI_semaphore(MEI_SEMAPHORE);
			state[axis] = axis_state(axis);
			source[axis] = axis_source(axis);
			release_MEI_semaphore(MEI_SEMAPHORE);
			printf("Axis: %d state<%d> source<%d>\n",
				axis, state[axis], source[axis]);
		}
	}
	getch();
	kill_MEI_semaphore(MEI_SEMAPHORE);
	for(axis = 0; axis < AXES; axis++)
	{	CloseHandle(hThreads[axis]);
		TerminateThread(hThreads[axis], EXIT_CODE_OK);
	}
	return 0;
}
