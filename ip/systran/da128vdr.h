/****************************************************************************/
/*                                                                          */
/*   DDDD         A        CCC      1      222       8888      V         V  */
/*   D   D       A A      C   C    11     2   2     8    8     V         V  */
/*   D    D     A   A    C     C    1    2     2   8      8     V       V   */
/*   D     D   A     A   C          1          2    8    8      V       V   */
/*   D     D   A     A   C          1         2      8888        V     V    */
/*   D     D   AAAAAAA   C          1        2      8    8       V     V    */
/*   D     D   A     A   C          1       2      8      8       V   V     */
/*   D    D    A     A   C     C    1      2       8      8        V V      */
/*   D   D     A     A    C   C     1     2         8    8         V V      */
/*   DDDD      A     A     CCC     111   2222222     8888           V       */
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
/*  Procedure Name: da128vdr.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  DAC128V driver routines.                                                */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  10-18-94  Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

#include <types.h>

int DAC128VReadReg16(int, int, u_int16 *);
int DAC128VReadID(int, int, u_int16 *);
int DAC128VWriteReg16(int, int, u_int16);
