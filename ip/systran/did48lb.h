/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Procedure Name: did48lb.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  DID48 library routines.                                                */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-28-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

#include "types.h"
#include "did48ld.h"

/* hardware dependent routines */
int DID48_Read_Reg(int, int, u_int16 *);
int DID48_Read_ID_PROM(int, u_int8 *);
int DID48_Read_Port(int, int, u_int8 *);
int DID48_Write_Reg(int, int, u_int16);
int DID48_Write_Port(int, int, u_int8);

/* operation routines */
int DID48_Configuration(int, int,
	enum DID48_Config_Cmd_Type);
int DID48_Interrupt_Enable_Control(int, int,
	enum DID48_Interrupt_Ena_Dis_Type);
