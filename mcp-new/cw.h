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

char *balance_weight(int inst);
char *balance_cmd(char *cmd);
void balance (int cw, int inst);
int cw_abort(void);
void cw_set_positionv(int inst, short p1, short p2, short p3, short p4);
void cw_set_posv(int inst, short *p1, short *p2, short *p3, short *p4);
void cw_pos(int cw, float *pos);
void cw_posv(int cw, short *pos);
void cw_position(int cw, double pos);
void cw_positionv(int cw, short pos);
int cw_get_inst(char *cmd);
int cw_select(int cw);
int cw_rdselect();
int cw_status();
void cw_data_collection(void);

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
