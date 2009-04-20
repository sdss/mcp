#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		Magnet Sensor control						*/
/*   File:	ms.h							*/
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

#ifndef __MS_H__

#define POSITIVE_DIRECTION	0
#define NEGATIVE_DIRECTION	1

#define MAP_VEL			10000.0
#define MAP_ACCEL		50000.0

#define MAGNESWITCH_ERROR_MASK	FF000000L

struct MAG_SENSOR {
	int region;
	double position[2];
	double encoder[2];
	double error[2];
	unsigned long status[2];
	int direction;
	float time[2];
};

#define __MS_H__             /* do only once */

#endif	/* End __MS_H__ */
