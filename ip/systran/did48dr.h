/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Procedure Name: did48dr.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  DID48 driver routines.                                                 */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*                                                                          */
/****************************************************************************/

#include "types.h"

int DID48ReadReg16(int, int, u_int16 *);
int DID48ReadID(int, int, u_int16 *);
int DID48ReadISR(int, u_int8 *);
int DID48WriteReg8(int, int, u_int8);
int DID48WriteReg16(int, int, u_int16);
int DID48ClearISR(int, u_int8);
