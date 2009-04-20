#include "as2.h"
/*
 * This function is provided so that the dollar-Name:-dollar only
 * needs to be expanded in one file
 */
const char *
getCvsTagname(void)
{
   return "HeadURL$";
}

char *
version_cmd(int uid, unsigned long cid,
	    char *cmd)			/* NULL to not print any status keywords */
{
   if (cmd != NULL) {
      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "mcpVersion", "$Name$|"  __DATE__ "|" __TIME__);
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "version");
   }

   return "mcpVersion=\"HeadURL$|"  __DATE__ "|" __TIME__ "\"\n";
}
