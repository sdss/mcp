/****************************************************************************/
/*                                                                          */
/*       A     DDDD       CCC      1      222      8888    FFFFFFF   1      */
/*      A A    D   D     C   C    11     2   2    8    8   F        11      */
/*     A   A   D    D   C     C    1    2     2  8      8  F         1      */
/*    A     A  D     D  C          1          2   8    8   F         1      */
/*    A     A  D     D  C          1         2     8888    FFFFF     1      */
/*    AAAAAAA  D     D  C          1        2     8    8   F         1      */
/*    A     A  D     D  C          1       2     8      8  F         1      */
/*    A     A  D    D   C     C    1      2      8      8  F         1      */
/*    A     A  D   D     C   C     1     2        8    8   F         1      */
/*    A     A  DDDD       CCC     111   2222222    8888    F        111     */
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
/*  Procedure Name: ad12f1dr.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  ADC128F1 driver routines.                                               */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  11-3-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

#include <types.h>

int ADC128F1ReadReg16(int, int, u_int16 *);
int ADC128F1ReadID(int, int, u_int16 *);
int ADC128F1WriteReg16(int, int, u_int16);
