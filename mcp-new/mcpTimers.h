#if !defined(MCP_TIMERS_H)
#define MCP_TIMERS_H 1

#include "mcpUtils.h"
#include "dio316dr.h"
#include "dio316lb.h"
#include "did48lb.h"
#include "mv162IndPackInit.h"

#define ONE_DAY	86400			/* number of seconds in a day */

#define DID48_VECTOR		29
#define DID48_IRQ		5
#define DID48_TYPE		3
	
#define DIO316_VECTOR		27
#define DIO316_IRQ		5
#define DIO316_TYPE		2

#define NIST_INT		0x1
#define NBS_INT			0x1
#define AZIMUTH_INT		0x2
#define ALTITUDE_INT		0x4
#define INSTRUMENT_INT		0x8

int DIO316_initialize(unsigned char *addr, unsigned short vecnum);
void DIO316_interrupt(int type);
void axis_DIO316_shutdown(int type);
double sdss_delta_time(double t2, double t1);

extern long SDSStime;

extern int tm_DIO316;
extern int DIO316_Init;
extern unsigned char dio316int_bit;
extern SEM_ID semLATCH;

#endif

