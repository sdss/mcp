#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "as2.h"

/*
 * This function is provided so that dollar-HeadURL-dollar only
 * needs to be expanded in one file
 */
const char *
getCvsTagname(void)
{
   static char stag[] = "$HeadURL$";
   static char *tag = NULL;		/* the actual tag */
   
   if (tag == NULL) {
      char const *mcp_tags = "/mcp/tags/";
      char *ptr = strstr(stag, mcp_tags);

      if(ptr == NULL) {
	 tag = "NOSVN";
      } else {
	 ptr += strlen(mcp_tags);
	 tag = ptr;
	 ptr = strchr(tag, '/');
	 if (ptr != NULL) {
	    *ptr = '\0';
	 }
      }
   }

   return(tag);
}
/*
 * Return the current version (and time of compilation)
 *
 * N.b. we fake SVN's HeadURL so that it appears to be an expansion of CVS's Name,
 * as the mcpMenu relies on this and it doesn't seem worth rewriting
 */
char *
version_cmd(int uid, unsigned long cid,
	    char *cmd)			/* NULL to not print any status keywords */
{
   static char version[100];
   sprintf(version, "mcpVersion=\"$Name" ": %s$|" __DATE__ "|" __TIME__ "\"", getCvsTagname());
   assert(strlen(version) < sizeof(version));

   if (cmd != NULL) {
      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "mcpVersion", getCvsTagname());
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "version");
   }

   return version;
}
