#if !defined(MCPNTP_H)
#define MCPNTP_H
/*
 * The MCP's NTP command
 */
int setTimeFromNTP(const char *NTPserver_name, int retryDelay,		
		   unsigned long retryCnt, int forceStep, struct timeval *t);

#endif
