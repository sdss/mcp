/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48ld.h                                                 */
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

/* Interrupt Configuration Command enumerated type */
enum DID48_Config_Cmd_Type {DID48_DEBOUNCE, DID48_BOUNCE};

/* Interrupt Enable Command enumerated type */
enum DID48_Interrupt_Ena_Dis_Type {DID48_INT_DIS, DID48_INT_ENA};
