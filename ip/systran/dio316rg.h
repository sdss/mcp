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
/*  Module Name: dio316rg                                                   */
/*                                                                          */
/*  Description: This include file contains DIO316 register definitions.    */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  6-27-94   Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

/* data port offsets */
#define DIO316_PORT0 0
#define DIO316_PORT1 1
#define DIO316_PORT2 2
#define DIO316_PORT3 3
#define DIO316_PORT4 4
#define DIO316_PORT5 5

/* register offsets */
#define DIO316_CSR_DAT_0	0
#define DIO316_CSR_DAT_1	1
#define DIO316_CSR_DAT_2	2
#define DIO316_CSR_OE_CTL	3
#define DIO316_CSR_DIR_CTL	4
#define DIO316_CSR_INT_ENA	5
#define DIO316_CSR_INT_STA	6
#define DIO316_CSR_INT_EL	7
#define DIO316_CSR_INT_HL	8
#define DIO316_CSR_INT_OC	9
#define DIO316_CSR_INT_VEC	10

/* number of 16 and 8 bit output data ports */
#define DIO316_NUM_OPORT_16	2
#define DIO316_NUM_OPORT_8	4
/* number of 16 and 8 bit bi-directional data ports */
#define DIO316_NUM_BPORT_16	1
#define DIO316_NUM_BPORT_8	2

/* define control register bits */
/* Output Enable Control */
#define DIO316_CSR3_OE_DIS		0x0
#define DIO316_CSR3_OE_ENA		0x1
/* Port Direction Control */
#define DIO316_CSR4_DIR_IN		0x0
#define DIO316_CSR4_DIR_OUT		0x1
/* Interrupt Enable Control */
#define DIO316_CSR5_IEN_DIS		0x0
#define DIO316_CSR5_IEN_ENA		0x1
/* Interrupt Edge Level Configuration */
#define DIO316_CSR7_IEL_LVL		0x0
#define DIO316_CSR7_IEL_EDG		0x1
/* Interrupt High Low Configuration */
#define DIO316_CSR8_IHL_LOW		0x0
#define DIO316_CSR8_IHL_HIGH	0x1
/* Interrupt On Change Configuration */
#define DIO316_CSR9_IOC_DIS		0x0
#define DIO316_CSR9_IOC_ENA		0x1
