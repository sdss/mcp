/* FINDPCI.C

:Utility to locate and read information from CPCI/DSP boards

For use with Microsoft:
		DOS
		Windows 3.X
		Windows 95

If a CPCI/DSP board is located, its interrupt logic will be enabled.
 Further documentation regarding interrupt support and MEI's 
 DSP-series controllers is located in the 'C' Programming Manual.

Warning!  This is a sample program to assist in the integration of the
 DSP-Series controller with your application.  It may not contain all 
 of the logic and safety features that your application requires.
  
Written for Version 2.5 
*/

/*	Revision Control System Information
	$Source$ 
	$Revision$
	$Date$

	$Log$
	Revision 1.1  1999/08/31 16:43:03  briegel
	source for MEI library

*/

	
# include <stdio.h>
# include <stdlib.h>
# include "idsp.h"


#define DM_LIRQ_REG				(DSP_DM)	0x72
#define PCI_DEVICE_ID    		0C0FEh
#define PCI_VENDOR_ID    		0C0FEh
#define PCI_BIOS_PRESENT 		0b101h
#define PCI_FIND_DEVICE  		0b102h
#define PCI_READ_CONFIG_BYTE  	0b108h
#define PCI_READ_CONFIG_WORD  	0b109h
#define PCI_LOCAL_CONFIG		00014h
#define PCI_IO_ADDRESS			00018h
#define PCI_INT_LINE			0003ch
#define PCI_INT_ENABLE          0x41
#define PCI_INT_ENABLE_PORT     0x4c
#define PCI_ACK_INT             0x30
#define MAX_BOARDS				16

int16 find_cpci_dsp(int16 * boardsfound, 
                        int16 * pci_addresses, 
                        int16 * pci_interrupt)
	{
	int16 index, address, config_address, cardcount = 0;
	char  pci_byteret;	
	unsigned char interrupt_num;
	int16 pci_wordret;

	_asm						/* check to see if PCI BIOS installed */
		{
		mov	ax,PCI_BIOS_PRESENT
		mov	dx,0
		int	1ah
		mov	pci_wordret,dx
		}

	if (pci_wordret != 'CP')	/* PCI signature in DX? */
		{
		for (index=0; index < MAX_BOARDS; index++)
			{
			_asm
				{
				mov		ax, PCI_FIND_DEVICE /* try to find a board */
				mov		cx, PCI_DEVICE_ID
				mov		dx, PCI_VENDOR_ID
				mov		si, index
		 		int 	1ah
				mov		pci_byteret,ah			
			
				cmp		ah,0				/* found one? */
				jnz		dev_not_found

				 mov	ax,PCI_READ_CONFIG_WORD
				 mov	di,PCI_IO_ADDRESS
				 int	1ah					/* get io address of this board */
				 mov	address,cx

				 mov	ax,PCI_READ_CONFIG_WORD
				 mov	di,PCI_LOCAL_CONFIG
				 int	1ah					/* get configuratio address of this board */
				 mov	config_address,cx

				 mov	ax,PCI_READ_CONFIG_BYTE
				 mov	di,PCI_INT_LINE
				 int	1ah					/* get interrupt for this board */
				 mov	interrupt_num,cl
				 
				 mov	pci_byteret,ah
dev_not_found:
				 }
		
			if (pci_byteret == 0)
				{
				cardcount++;
				pci_addresses[index] = address & 0xfffe;
				pci_interrupt[index] = (int16) interrupt_num;
				config_address &= 0xfffe;

				/* enable interrupts in pci interface and fpga */

				DSP_OUT(pci_addresses[index],   DM_LIRQ_REG | PCDSP_DM);
				DSP_OUT(pci_addresses[index]+2, PCI_ACK_INT);
				
				DSP_OUTB(config_address+PCI_INT_ENABLE_PORT, PCI_INT_ENABLE);
				}
			}
		}

	*boardsfound = cardcount;

	if (!cardcount)
		return(DSP_NOT_FOUND);
	else
		return(DSP_OK);
	}


void error(int error_code)
{   
	char buffer[MAX_ERROR_LEN];

	switch (error_code)
	{
		case DSP_OK:
			/* No error, so we can ignore it. */
			break ;

		default:
			error_msg(error_code, buffer) ;
			fprintf(stderr, "ERROR: %s (%d).\n", buffer, error_code) ;
			exit(1);
			break;
	}
}


int	main(void)
{
	int16 boardsfound, addrs[MAX_BOARDS], irqs[MAX_BOARDS],
			count, error_code;
	char * endp;
	long boardstofind;

	printf("Searching for CPCI/DSP boards...\n");
	find_cpci_dsp(&boardsfound, addrs, irqs);

	if(boardsfound)
	{
		for(count = 0; count < boardsfound ; count++)
		{
			printf("Board %d at I/O address: 0x%x  using IRQ: %d\n",
		 				count, addrs[count], irqs[count]);
		}	
	}
	else
	{
		printf("Unable to find any CPCI/DSP boards. \n");
	}	
		
	return 0;
}












