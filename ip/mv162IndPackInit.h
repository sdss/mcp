/* mv162IndPackInit.h - Motorola 162 IndustryPack initialization header.
*
* Version: "@(#)mv162IndPackInit.h	1.1    23 May 1994 TSL"
*
* Copyright (c) 1994 The Svedberg Laboratory.
*/

#pragma ident "@(#)mv162IndPackInit.h	1.1    23 May 1994 TSL"

/*
modification history
--------------------
940420,LT	written.
*/

#ifndef __INCmv162IndPackInith
#define __INCmv162IndPackInith

#ifdef __cplusplus
/*extern "C" {*/
#endif

#define ID_OFFSET		0x0080
#define ID_MANUFACTURER		0x0089
#define ID_MODEL		0x008B
#define ID_REVISION		0x008D
#define USER_DATA		0x0099
#define IP_SIZE			0x0100

#define VMESC5_Reset		0x501
#define VMESC5_Status		0x503
#define VMESC5_Interrupt	0x581
#define VMESC5_General_Purpose	0x601
	#define VMESC5_GP_InUse		0x01

#define MAX_SLOTS			5
#define VME162_Slots			4
#define GREENSPRING_VIPC610_Slots	4
#define SYSTRAN_VMESC5_Slots		5

#define	VME162			0x0162
#define	GreenSpring_VIPC610	0x00F0
#define	Systran_VMESC5		0x0045

#define FNAL_MANUFACTURER	0xBB
struct IPACK {
	unsigned short carrier;
	unsigned char *adr[MAX_SLOTS];
};

#define NUM_IP_SLOT				4			/* Number of IP slots on MVME-162 */

#define IP_INT_VEC_START		0xB0		/* Start using interrupt vectors */
											/*  from this number */

#define IP_ID_AREA_BASE			0xfff58080	/* ID area base address */
#define IP_IO_AREA_BASE			0xfff58000	/* IO area base address */
#define IP_SLOT_INC				0x100		/* Address increment to next slot */

#define IP_INT_CNTRL_BASE		0xfffbc010	/* Interrupt control register */

#define IPC_INT_EN				0x10		/* IPC interrupt enable bit */
		
#define IPC_INT_LEVEL_1			0x01		/* IPC interrupt level codes */
#define IPC_INT_LEVEL_2			0x02
#define IPC_INT_LEVEL_3			0x03
#define IPC_INT_LEVEL_4			0x04
#define IPC_INT_LEVEL_5			0x05
#define IPC_INT_LEVEL_6			0x06
#define IPC_INT_LEVEL_7			0x07


#define IP_MEM_BASE_BASE	0xFFFBC004	/* IP mem base reg. base */
#define IP_MEM_SIZE_BASE	0xFFFBC00C	/* IP mem size reg. base */
#define IP_GEN_CNTRL_REG_BASE	0xFFFBC018	/* IP control reg. base */

/*
* IP general control register codes
*/
#define IP_RT_0					0x00		/* Recovery time 0 uSec */
#define IP_RT_2					0x10		/* Recovery time 2 uSec */
#define IP_RT_4					0x20		/* Recovery time 4 uSec */
#define IP_RT_8					0x30		/* Recovery time 8 uSec */

#define IP_MEM_WIDTH_BYTE		0x04		/* IP memory width codes */	
#define IP_MEM_WIDTH_WORD		0x08
#define IP_MEM_WIDTH_LWORD		0x00

#define IP_MEM_ENABLE			0x01		/* Enable IP memory space */

/*
* The IP octal serial needs a temporary memory page to be map in during
* initialization since the interrupt vector must be written through 
* memory address space. 
*/
#define IP_TMP_MEM_PAGE			0xfffd0000	/* Temp. memory page base address */

/*
* GreenSpring IP model numbers
*/
#define MODEL_IP_OCTAL_SERIAL		0xF022	
#define MODEL_IP_DUAL_P_T		0xF023	
#define MODEL_IP_DIGIT_48		0xF024	
#define MODEL_IP_ADC_16			0xF036
#define MODEL_IP_ETHERNET_LANCE		0xF035
#define MODEL_IP_488		        0xF014
#define MODEL_IP_WATCHDOG               0xF054

/*
* SYSTRAN IP model numbers
*/
#define MODEL_IP_TRIDO_48		0x4562
#define SYSTRAN_TRIDO48		0x4562
#define MODEL_IP_DIO_316		0x4563
#define SYSTRAN_DIO316		0x4563
#define MODEL_IP_DIO_316I		0x4563
#define SYSTRAN_DIO316I		0x4563
#define MODEL_IP_DUAL_32CT		0x4566
#define SYSTRAN_DUAL32CT	0x4566
#define MODEL_IP_DID_48			0x4568
#define SYSTRAN_DID48		0x4568
#define MODEL_IP_DAC_128V		0x4569
#define SYSTRAN_DAC128V		0x4569
#define MODEL_IP_ADC_128F1		0x456A
#define SYSTRAN_ADC128F1	0x456A
/*
* ACROMAG IP model numbers
*/
#define MODEL_IP_320			0xA332
#define MODEL_IP_330			0xA311
#define MODEL_IP_470			0xA308
#define MODEL_IP_480_6			0xA316
#define MODEL_IP_480_2			0xA317
#define MODEL_IP_502			0xA306
/*
* FNAL IP model numbers
*/
#define MODEL_IP_FNAL			0xBB00
#define MODEL_IP_UCD			0xBB01
#define MODEL_IP_UCD5			0xBB02
#define MODEL_IP_TSLATMR8		0xBB03
#define MODEL_IP_MDATRX			0xBB10
#define MODEL_IP_177			0xBB11
#define MODEL_IP_TWTPM			0xBB0E
#define MODEL_IP_MDATUCD		0xBB15

/*
* Function prototypes
*/
int mv162IndPackInit(void);
int Industry_Pack(unsigned char *ip_base, unsigned short model, struct IPACK *ip);
void Industry_Pack_List(unsigned char *ip_base);
void Industry_Pack_Array(unsigned short *modules_return);
int IP_Interrupt_Enable (struct IPACK *ip, int irq);
int VME162_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq);
int VMESC5_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq);
int VIPC610_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq);

#ifdef __cplusplus
/*}*/
#endif

#endif /* __INCmv162IndPackInith */
