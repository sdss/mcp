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
/*  Module Name: ad12f1rg                                                   */
/*                                                                          */
/*  Description: This include file contains ADC128F1 register definitions.  */
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

/* data register offsets */
#define ADC128F1_CHAN_1 0
#define ADC128F1_CHAN_2 1
#define ADC128F1_CHAN_3 2
#define ADC128F1_CHAN_4 3
#define ADC128F1_CHAN_5 4
#define ADC128F1_CHAN_6 5
#define ADC128F1_CHAN_7 6
#define ADC128F1_CHAN_8 7
#define ADC128F1_CTL	8
#define ADC128F1_STAT	9

/* control register bit definitions */
#define ADC128F1_CSR8_N_SUPDIS 0x01
