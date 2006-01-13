#include <stdio.h>
#include <assert.h>
#include <string.h>

/*****************************************************************************/
/*
 * Dummy routine to report status of non-existant umbilical tower
 */
int
get_umbilstatus(char *status_ans,
		int size)			/* dimen of status_ans */
{
   int len;

   sprintf(status_ans, "Umbil: %d  %d %2.2f  %d %d\n",
	   0, 0, 0.0, 0, 0);
   len = strlen(status_ans);
   assert(len < size);
   
   return(len);
}
