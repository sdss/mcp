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
/*  Module Name: dio316ld.h                                                 */
/*                                                                          */
/*  Description: This include file contains type definitions for the        */
/*  DIO316 library routines.                                                */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-24-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

#include "types.h"

/* type definitions */

/* Output Enable Command enumerated type */
enum DIO316_OE_Cmd_Type {DIO316_OE_DIS, DIO316_OE_ENA};

/* Port Direction Command enumerated type */
enum DIO316_Direction_Cmd_Type {DIO316_DIR_IN, DIO316_DIR_OUT};

/* Interrupt Configuration Command enumerated type */
enum DIO316_Interrupt_Config_Cmd_Type {DIO316_INT_LOW_LVL, DIO316_INT_HIGH_LVL,
DIO316_INT_FALL_EDGE, DIO316_INT_RISE_EDGE, DIO316_INT_ON_CHANGE};

/* Interrupt Enable Command enumerated type */
enum DIO316_Interrupt_Ena_Dis_Type {DIO316_INT_DIS, DIO316_INT_ENA};
