#if !defined(MCPUTILS_H)
#define MCPUTILS_H

#include <stdio.h>
#include <semLib.h>

char *get_date(void);
int mjd(void);
FILE *fopen_logfile(const char *file, const char *mode);
unsigned long timer_read(int timer);
unsigned long timer_start(int timer);
void VME2_pre_scaler(unsigned long adjust);
char *mcpVersion(char *ver, int len);
char *version_cmd(char *cmd);
long getSemTaskId(SEM_ID sem);

#endif
