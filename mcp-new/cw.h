#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	cw.h							*/
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

#ifndef __CW_H__

#define DIO316_VECTOR		27
#define DIO316_IRQ		5
#define DIO316_TYPE		2

#define DAC128V_CHANS		8

#define INST_DEFAULT		16
#define NUMBER_INST (INST_DEFAULT + 1)

#define ALL_CW -999			/* move all counter weights */
#define NUMBER_CW 4			/* number of counter weights */

void tMoveCWInit(unsigned char *addr, unsigned short vecnum);

int get_cwstatus(char *cwstatus_ans, int size);

int cw_abort(void);
void cw_data_collection(void);
int cw_get_inst(char *cmd);
void set_counterweight(int inst, int cw, short pos);

/*
 * Umbilical
 */
int get_umbilstatus(char *status_ans, int size);

/*
 * global variables
 */
extern unsigned char cwLimit;
extern int cw_DIO316;
extern int cw_ADC128F1;
extern int cw_DAC128V;
extern char *inst_name[];

#define __CW_H__             /* do only once */

#endif	/* End __CW_H__ */
