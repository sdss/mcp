#if !defined(MCPUTILS_H)
#define MCPUTILS_H

long date(void);
char *get_date(void);
unsigned long timer_read(int timer);
unsigned long timer_start(int timer);
void VME2_pre_scaler(unsigned long adjust);

#endif
