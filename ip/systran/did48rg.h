/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*  Module Name: did48rg                                                   */
/*                                                                          */
/*  Description: This include file contains DID48 register definitions.    */
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
#define DID48_PORT0 0
#define DID48_PORT1 1
#define DID48_PORT2 2
#define DID48_PORT3 3
#define DID48_PORT4 4
#define DID48_PORT5 5

/* register offsets */
#define DID48_CSR_DAT_0	0
#define DID48_CSR_DAT_1	1
#define DID48_CSR_DAT_2	2
#define DID48_CSR_CTL		3
#define DID48_CSR_INT_STA	4
#define DID48_CSR_INT_ENA	5
#define DID48_CSR_INT_VEC	6

/* number of 16 and 8 bit output data ports */
#define DID48_NUM_IPORT_16	3
#define DID48_NUM_IPORT_8	6

/* define control register bits */
/* Output Enable Control */
#define DID48_CSR3_BOUNCE		0x0
#define DID48_CSR3_DEBOUNCE		0x1
/* Interrupt Enable Control */
#define DID48_CSR4_IEN_DIS		0x0
#define DID48_CSR4_IEN_ENA		0x1
