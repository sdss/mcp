#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/*   File:	cmd.h							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	cmd_handler : VxWorks					*/
/*   Modules:	cmd_handler : command parser				*/
/*		cmd_init    : initialization				*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
	Utilizes cmd.h which specifies the commands.
--*/
/************************************************************************/

#ifndef __CMD_H__
/* function prototypes */
int cmd_init();
char *cmd_handler(char *cmd);
void get_ctrl(SEM_ID sem), rtn_ctrl(SEM_ID sem);

struct COMMANDS {
	char *command;
	char *(*function)();
};
struct COMMANDS axis_cmds[] = {
	{"\033",reboot_cmd},
	{"CORRECT",correct_cmd},
	{"DRIFT",drift_cmd},
	{"ENCL",encl_cmd},
	{"ID",id_cmd},
	{"INIT",init_cmd},
	{"MAXACC",maxacc_cmd},
	{"MAXVEL",maxvel_cmd},
	{"MC.DUMP",mc_dump_cmd},
	{"MC.MAXACC",mc_maxacc_cmd},
	{"MC.MAXPOS",mc_maxpos_cmd},
	{"MC.MAXVEL",mc_maxvel_cmd},
	{"MC.MINPOS",mc_minpos_cmd},
	{"MOVE",move_cmd},
	{"+MOVE",plus_move_cmd},
	{"MR.DUMP",mr_dump_cmd},
	{"MS.DUMP",ms_dump_cmd},
	{"MS.MAP.DUMP",ms_map_dump_cmd},
	{"MS.MAP.LOAD",ms_map_load_cmd},
	{"MS.OFF",ms_off_cmd},
	{"MS.ON",ms_on_cmd},
	{"MS.POS.DUMP",ms_pos_dump_cmd},
	{"REMAP",remap_cmd},
	{"IR",rot_cmd},
	{"SET.LIMITS",set_limits_cmd},
	{"SET.POSITION",set_position_cmd},
	{"SET.TIME",set_time_cmd},
	{"STATS",stats_cmd},
	{"STATUS.LONG",status_long_cmd},
	{"STATUS",status_cmd},
	{"TEL1",tel1_cmd},
	{"TEL2",tel2_cmd},
	{"TICKLOST @ .",ticklost_cmd},
	{"TIME?",time_cmd},
	{"BRAKE.ON",brakeon_cmd},
	{"BRAKE.OFF",brakeoff_cmd},
	{"CLAMP.ON",clampon_cmd},
	{"CLAMP.OFF",clampoff_cmd},
	{"CWMOV",cwmov_cmd},
	{"CWPOS",cwpos_cmd},
	{"CWINST",cwinst_cmd},
	{"CWABORT",cwabort_cmd},
	{"CWSTATUS",cwstatus_cmd},
	{"SP1",sp1_cmd},
	{"SP2",sp2_cmd},
	{"SLIT.CLOSE",slitclose_cmd},
	{"SLIT.OPEN",slitopen_cmd},
	{"CART.LATCH",cartlatch_cmd},
	{"CART.UNLATCH",cartunlatch_cmd},
	{"SLIT.STATUS",slitstatus_cmd},
	{"FFS.CLOSE",ffsclose_cmd},
	{"FFS.OPEN",ffsopen_cmd},
	{"FFL.ON",fflon_cmd},
	{"FFL.OFF",ffloff_cmd},
	{"NE.ON",neon_cmd},
	{"NE.OFF",neoff_cmd},
	{"HGCD.ON",hgcdon_cmd},
	{"HGCD.OFF",hgcdoff_cmd},
	{"FF.STATUS",ffstatus_cmd},
	{"AB.STATUS",abstatus_cmd},
	{NULL,dummy_cmd}		/* termination */
};
#define __CMD_H__             /* do only once */

#endif	/* End __CMD_H__ */
