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

extern char *correct_cmd(char *cmd);
extern char *drift_cmd(char *cmd);
extern char *gp1_cmd(char *cmd);
extern char *gp2_cmd(char *cmd);
extern char *id_cmd(char *cmd);
extern char *init_cmd(char *cmd);
extern char *maxacc_cmd(char *cmd);
extern char *maxvel_cmd(char *cmd);
extern char *mc_dump_cmd(char *cmd);
extern char *mc_maxacc_cmd(char *cmd);
extern char *mc_maxpos_cmd(char *cmd);
extern char *mc_maxvel_cmd(char *cmd);
extern char *mc_minpos_cmd(char *cmd);
extern char *move_cmd(char *cmd);
extern char *plus_move_cmd(char *cmd);
extern char *mr_dump_cmd(char *cmd);
extern char *ms_dump_cmd(char *cmd);
extern char *ms_map_dump_cmd(char *cmd);
extern char *ms_map_load_cmd(char *cmd);
extern char *ms_off_cmd(char *cmd);
extern char *ms_on_cmd(char *cmd);
extern char *ms_pos_dump_cmd(char *cmd);
extern char *remap_cmd(char *cmd);
extern char *rot_cmd(char *cmd);
extern char *set_limits_cmd(char *cmd);
extern char *set_position_cmd(char *cmd);
extern char *set_time_cmd(char *cmd);
extern char *stats_cmd(char *cmd);
extern char *status_cmd(char *cmd);
extern char *status_long_cmd(char *cmd);
extern char *tel1_cmd(char *cmd);
extern char *tel2_cmd(char *cmd);
extern char *ticklost_cmd(char *cmd);
extern char *time_cmd(char *cmd);
extern char *dummy_cmd(char *cmd);

struct COMMANDS {
	char *command;
	char *(*function)();
};
struct COMMANDS axis_cmds[] = {
	{"\033",init_cmd},
	{"CORRECT",correct_cmd},
	{"DRIFT",drift_cmd},
	{"GP1",gp1_cmd},
	{"GP2",gp2_cmd},
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
	{NULL,dummy_cmd}		/* termination */
};
#define __CMD_H__             /* do only once */

#endif	/* End __CMD_H__ */
