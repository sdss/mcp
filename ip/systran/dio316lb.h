/****************************************************************************/
/*                                                                          */
/*   DDDDD      IIIIIIIII      OOOO       333333       1         66666      */
/*   D    D         I         O    O     3      3     11        6     6     */
/*   D     D        I        O      O           3      1       6            */
/*   D      D       I        O      O           3      1       6            */
/*   D      D       I        O      O         33       1       6            */
/*   D      D       I        O      O           3      1       6  6666      */
/*   D      D       I        O      O           3      1       6 6    6     */
/*   D     D        I        O      O           3      1       6       6    */
/*   D    D         I         O    O     3      3      1        6     6     */
/*   DDDDD      IIIIIIIII      OOOO       333333     11111       66666      */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                     This software was developed by:                      */
/*                                                                          */
/*                              SYSTRAN Corp.                               */
/*                            4126 Linden Ave.                              */
/*                         Dayton, Ohio 45432-3066                          */
/*                             (513) 252-5601                               */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  Procedure Name: dio316lb.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  DIO316 library routines.                                                */
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
#include "dio316ld.h"

/* hardware dependent routines */
int DIO316_Read_Reg(int, int, u_int16 *);
int DIO316_Read_ID_PROM(int, u_int8 *);
int DIO316_Read_Port(int, int, u_int8 *);
int DIO316_Write_Reg(int, int, u_int16);
int DIO316_Write_Port(int, int, u_int8);

/* operation routines */
int DIO316_OE_Control(int, int, enum DIO316_OE_Cmd_Type);
int DIO316_Port_Direction_Control(int, int, enum DIO316_Direction_Cmd_Type);
int DIO316_Interrupt_Configuration(int, int,
	enum DIO316_Interrupt_Config_Cmd_Type);
int DIO316_Interrupt_Enable_Control(int, int,
	enum DIO316_Interrupt_Ena_Dis_Type);
