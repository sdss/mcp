#include <taskLib.h>
#include <taskHookLib.h>
#include "dscTrace.h"
/*
 * A couple of tasks designed to be added as task{Create,Delete}Hooks
 */
void
taskCheckOnCreate(WIND_TCB *tcb)	/* the task that's being created */
{
   TRACEPROC("createTaskCheck");

   TRACEP(31, "Creating task cccccccc == 0x%08x%08x",
	  ((int *)tcb->name)[0], ((int *)tcb->name)[1]);
}


void
taskCheckOnDelete(WIND_TCB *tcb)	/* the task that's dying */
{
   TASK_DESC desc;			/* task descriptor */

   TRACEPROC("deleteTaskCheck");

   TRACEP(31, "Destroying task cccccccc == 0x%08x%08x",
	  ((int *)tcb->name)[0], ((int *)tcb->name)[1]);

   if(taskInfoGet((int)tcb, &desc) != OK) {
      return;
   }

   if(desc.td_stackMargin > 0) {
      TRACEP(31, "                Margin: %d", desc.td_stackMargin, 0);
   } else {
      TRACEP(0, "Stack overrun for task cccccccc == 0x%08x%08x",
	     ((int *)tcb->name)[0], ((int *)tcb->name)[1]);
   }
}

/*****************************************************************************/
/*
 * Here's an idle task to run during taskDelay's
 */
int idle_counter = 0;

int
idle(void)
{
   for(;;) {
      idle_counter++;
   }
}

/*****************************************************************************/
/*
 * Print a task name given a number of byte values from a trace output,
 * e.g. printf("0x%08x", *(int *)name);
 *
 * See also trace.el in ../etc
 */
void
cccc2name(const char *str)
{
   int i = 0;

   if(strncmp(str, "0x", 2) == 0) {
      str += 2;
   }

   while(*str != '\0') {
      if(sscanf(str, "%2x", &i) == 1) {
	 printf("%c", i);
      }
      str++; if(*str != '\0') str++;	/* skip two characters */
   }
   putchar('\n');
}
