#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		Magnet Sensor control						*/
/*   File:	frame.h							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/

#ifndef __FRAME_H__

#define POSITIVE_DIRECTION	0
#define NEGATIVE_DIRECTION	1

struct FRAME {
	struct FRAME *nxt;
	double position;
	double velocity;
	double acceleration;
	double deceleration;
	double jerk;
	double error[2];
	unsigned long status[2];
	int direction;
	double start_time;
	double end_time;
};

struct FRAME_QUEUE {
#define MAX_FRAME_CNT	100
	int cnt;
	struct FRAME *top;
	struct FRAME *end;
	struct FRAME *active;
};

#define __FRAME_H__             /* do only once */

#endif	/* End __FRAME_H__ */
