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
   return "mcpVersion=\"$Name$|"  __DATE__ "|" __TIME__ "\"\n";
}
