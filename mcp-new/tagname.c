#include "as2.h"
/*
 * This function is provided so that the dollar-Name:-dollar only
 * needs to be expanded in one file
 */
const char *
getCvsTagname(void)
{
   return "$Name$";
}

char *
version_cmd(int uid, unsigned long cid,
	    char *cmd)			/* NOTUSED */
{
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "mcpVersion", "$Name$|"  __DATE__ "|" __TIME__);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "version");

   return "mcpVersion=\"$Name$|"  __DATE__ "|" __TIME__ "\"\n";
}
