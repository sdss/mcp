#if !defined(MCPUTILS_H)
#define MCPUTILS_H

#include <stdio.h>
#include <semLib.h>

void sysReset(int);			/* really in vx_dsc */

char *get_date(void);
int get_mjd(void);
FILE *fopen_logfile(const char *file, const char *mode);
unsigned long timer_read(int timer);
unsigned long timer_start(int timer);
void VME2_pre_scaler(unsigned long adjust);
char *mcpVersion(char *ver, int len);
char *version_cmd(int uid, unsigned long cid, char *cmd);
long getSemTaskId(SEM_ID sem);
long phCrcCalc(long crc, const char *buff, int n);

#endif
