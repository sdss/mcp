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
/*  Procedure Name: ad12f1lb.h                                              */
/*                                                                          */
/*  Description: This include file contains function prototypes for the     */
/*  ADC128F1 library routines.                                              */
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

#include "ad12f1ld.h"

/* initialization - FNAL */
int ADC128F1Init(struct ADC128F1 *addr);

/* hardware dependent routines */
int ADC128F1_Read_Reg(int, int, unsigned short *);
int ADC128F1_Read_ID_PROM(int, unsigned char *);
int ADC128F1_Write_Reg(int, int, unsigned short);

/* operation routines */
int ADC128F1_Read_Volts(int, int, float *);
int ADC128F1_CVT_Update_Control(int, enum Enable_Disable_Type);
