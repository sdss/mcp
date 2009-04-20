/* MTASK.C

:Multi-tasking under Windows NT.

This sample demonstrates how to create multiple threads under Windows NT.

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

#define	SEM_MAXIMUM_USAGE				1L
#define	SEM_INITIAL_SIGNALED			1L
#define	SEM_INITIAL_NON_SIGNALED		0L
#define	SEM_INCREMENT					1L
#define	THREAD_STACK_SIZE				0
#define	THREAD_CREATION_FLAGS			0
#define	EXIT_CODE_OK					0

/* Globals */
HANDLE			MEI_SEMAPHORE;		/* Handle to library semaphore */
HANDLE			hTasks[2];			/* Handles to each created task */
int16			port = 0, io;


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

void MEI_motion_task(void *args)
{	int16 axis = 0;
	while(1)
 	{	get_MEI_semaphore(MEI_SEMAPHORE);
			if(motion_done(axis))
				start_move(axis, 10000.0, 10000.0, 100000.0);
			if(motion_done(axis))
				start_move(axis, 0.0, 10000.0, 100000.0);
		release_MEI_semaphore(MEI_SEMAPHORE);
	}
}

void MEI_io_task(void *args)
{	
	while(1)
 	{	get_MEI_semaphore(MEI_SEMAPHORE);
			get_io(port, &io);
		release_MEI_semaphore(MEI_SEMAPHORE);
	}
}
	 
int main()
{	unsigned long ThreadId[2], i;
	int16 error_code;
	
	MEI_SEMAPHORE = create_MEI_semaphore();

	error_code = do_dsp();	/* initialize communication with the controller */
	error(error_code);		/* any problems initializing? */
	error(dsp_reset());		/* hardware reset */

	init_io(PORT_A, IO_INPUT);	/* initialize I/O port for inputs */
	
	/* create thread
		default security descriptor, default stack size (same as creating thread)*/
	hTasks[0] = CreateThread(NULL, THREAD_STACK_SIZE, 
		(LPTHREAD_START_ROUTINE)MEI_motion_task, NULL, 
		THREAD_CREATION_FLAGS, &ThreadId[0]);

	hTasks[1] = CreateThread(NULL, THREAD_STACK_SIZE, 
		(LPTHREAD_START_ROUTINE)MEI_io_task, NULL, 
		THREAD_CREATION_FLAGS, &ThreadId[1]);

	while(!kbhit())
		printf("Port %d\t value: %4d\r", port, io);
	getch();
	for(i = 0; i < 2; i++)
	{	TerminateThread(hTasks[i], EXIT_CODE_OK);
		CloseHandle(hTasks[i]);
	}
	kill_MEI_semaphore(MEI_SEMAPHORE);
	return 0;
}


