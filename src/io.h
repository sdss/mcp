#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/************************************************************************/

#ifndef __IO_H__
/* DIO316 */
#define AMP_RESET	2
#define AMP_AZ1		0x1		/* 24 */
#define AMP_AZ2		0x2		/* 23 */
#define AMP_AL1		0x4		/* 22 */
#define AMP_AL2		0x8		/* 21 */
#define AMP_ROT		0x10		/* 20 */

/*...............................C1 BURNDY # pin.DIO316 pin....*/
#define IL_ENABLE	3
#define IL_ENABLED	0x20		/* 5E		27 */
#define IL_DISABLED	0xDF

#define CW_SELECT       3
#define CW_SELECT_0     0x0
#define CW_SELECT_1     0x1
#define CW_SELECT_2     0x2
#define CW_SELECT_3     0x3

#define CW_DIRECTION            3
#define CW_POS_DIRECTION        0x4
#define CW_NEG_DIRECTION        0

#define CW_POWER                3
#define CW_POWER_ON     0x10
#define CW_POWER_OFF    0xEF

#define DC_INTERRUPT            3
#define DC_INTPULSE     0x80

#define CW_LIMIT_INT            4
#define CW_LIMIT        0x1

#define CW_INTERLOCK            4
#define CW_INTERLOCK_OK 0x10
#define CW_INTERLOCK_BAD 0xEF
#define CW_LCLRMT               4
#define CW_LOCAL        0x20
#define CW_REMOTE       0xDF

#define CW_LIMIT_STATUS         5
#define CW_UPPER_LIMIT_0        0x01
#define CW_LOWER_LIMIT_0        0x02
#define CW_UPPER_LIMIT_1        0x04
#define CW_LOWER_LIMIT_1        0x08
#define CW_UPPER_LIMIT_2        0x10
#define CW_LOWER_LIMIT_2        0x20
#define CW_UPPER_LIMIT_3        0x40
#define CW_LOWER_LIMIT_3        0x80

/* IP480 */
#define IL_WD		2		/* WD channel    11 */
#define TM_WD           4               /* WD channel    20 */
					/* relay 41(NO),42,43(NC) */
#define CW_WD           6               /* WD channel    29 */
/* DAC */
#define CW_MOTOR        0
/* ADC128F1 */
#define CW_POS          0
#define CW_POS_0        0
#define CW_POS_1        1
#define CW_POS_2        2
#define CW_POS_3        3
#define	IL_POSITION	4		/*	      9,10 */
#define	IL_STRAIN_GAGE	5		/*	     11,12 */
#define IL_FAKE_POS	5
#define IL_FAKE_STRAIN	6

#define UP_DIRECTION	TRUE
#define DOWN_DIRECTION	FALSE
#define DEFAULT		0

#define __IO_H__             /* do only once */

#endif	/* End __IO_H__ */
