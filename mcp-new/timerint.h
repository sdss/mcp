/*============================================================================
//
//	T I M E R I N T . H
//
//	This header file declares the routines that interface with the
//	'162's 4th timer.
//
//	--MOD_WARNING--
//
//  $Id$
============================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void timerStart(unsigned long freq, unsigned char level, void (*func)());
void timerStat();
void timerStop();

#ifdef __cplusplus
};
#endif
