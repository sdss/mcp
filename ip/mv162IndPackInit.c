/* mv162IndPackInit.c - Motorola 162 IndustryPack initialization
*
* Version: "@(#)mv162IndPackInit.c	1.1    23 May 1994 TSL"
*
* Copyright (c) 1994 The Svedberg Laboratory.
*/
static const char scm[] = "@(#)mv162IndPackInit.c	1.5 12/21/98 %TUT%";  

/* old identifier stuff. */
/* #pragma ident "@(#)mv162IndPackInit.c	1.1    23 May 1994 TSL"*/


/*
modification history             
--------------------
940420,LT	written.
*/

/*
DESCRIPTION
This is an initialization routine for IndustryPack's on a MVME-162 CPU.
It searches the IP ID address space and try's to identify any pack's found.
If a suitable driver is available it is installed.

Note !
The #ifdef INDUSTRY_PACK_C_FILE entry in mv162/sysLib.c - sysHwInit() can't be
used to call mv162IndPackInit() since the I/O system isn't initialized at that
point. usrConfig.c - usrRoot() works better.
*/

#include "vxWorks.h"
#include "ioLib.h"
#include "taskLib.h"
#include "iv.h"
#include "intLib.h"
#include "stdio.h"
#include "vxLib.h"
#include "mv162IndPackInit.h"

UINT8 strId[4] = {'I', 'P', 'A', 'C'};


/*******************************************************************************
* Industry_Pack - given an IndustryPack base address return specifications.
*
* DESCRIPTION
* Searches the processor or carrier's IP module ID area for installed IP 
* modules. When an
* IP is found it's model number is compared to the specified model numbers
* for identification. If a matching model is available, the address of the
* module is returned and the model can be retrieved from the IP
* Zero in the model specification implies all responding IPs address will be
* returned.
* RETURNS
* static structure of IP modules found and the carrier board which means it
* could be clobbered, but this is useful at boot time.
*/
	static char *FNAL_Model_name[]={
		"CLKACNET",
		"IRMIPDG6",
		"IRMIPDG5",
		"TSLATMR8"
};
int Industry_Pack(unsigned char *ip_base, unsigned short model, 
		struct IPACK *ip)
{
	int slots;
	int i, j, jj;
	unsigned int ipModel;
	unsigned char *pId;
	unsigned char byte;
	int module_cnt;
	unsigned char *model_name;
	unsigned char *user_data;

 	for(i=0; i<MAX_SLOTS; i++)	
	  ip->adr[i]=NULL;
	if (((unsigned long)ip_base&0xFFFF0000)!=0xFFFF0000) 
	{
          ip->carrier=VME162;
	  slots = VME162_Slots;
	}
	else 
	{
          if (vxMemProbe((char *) (ip_base+VMESC5_Reset), READ, 1, 
	    (char *) &byte) == ERROR) 
          {
            ip->carrier=GreenSpring_VIPC610;
	    slots = GREENSPRING_VIPC610_Slots;
          }
	  else
	  {
	    ip->carrier=Systran_VMESC5;
	    slots = SYSTRAN_VMESC5_Slots;
          }
	}

	module_cnt=0;
 	for(i=0; i<slots; i++)	
	{
/*
* Check if any IP module installed in this slot.
*/
	  pId = ip_base+(IP_SIZE*i)+ID_OFFSET+1; 	/* Uses only odd addresses */
	  for(j=0; j<sizeof(strId); j++, pId +=2)	
	  {
	    if(vxMemProbe((char *) pId, READ, 1, (char *) &byte) == ERROR)
	      break;
	    if(byte != strId[j])
	      break;
	  }
	  if(j < sizeof(strId))
	    continue;		/* Didn't pass ID string test */
	  if (model==0)
	  {
	    ip->adr[i]=ip_base + (IP_SIZE*i);	/* Uses only odd addresses */
	    module_cnt++;
	  }
	  else
	  {
	    if ((*pId==FNAL_MANUFACTURER)&&(*(pId+2)==0))
	    {
	      for (j=0;j<sizeof(FNAL_Model_name)/sizeof(unsigned char *);j++)
	      {
	        model_name = FNAL_Model_name[j];
		user_data = pId+(8*2);
	        for (jj=0;jj<8;jj++)
		{
		  if (*model_name++ != *user_data) break;
		  user_data +=2;
		}
	        if ((jj>=8) && (j==(model&0xFF)))
		{
	          ip->adr[i]=ip_base + (IP_SIZE*i);
	          module_cnt++;
	        }
	      }	      
	    }
	    else
	    {
	      ipModel = *pId << 8;	/* Read manufacturer ID */
	      pId += 2;
	      ipModel |= *pId;	/* Read IP model number */
	      if (ipModel==model)	
	      {
	        ip->adr[i]=ip_base + (IP_SIZE*i);
	        module_cnt++;
	      }
	    }
	  }
	}
	return module_cnt;
}

	static unsigned short carrier_type[]={
		VME162,
		GreenSpring_VIPC610,
		Systran_VMESC5
};
	static const char *carrier_name[]={
		"VME162",
		"GreenSpring_VIPC610",
		"Systran_VMEC5",
		"NONE FOUND"
};
	static unsigned short module_type[]={
		MODEL_IP_ETHERNET_LANCE,
		MODEL_IP_488,
		MODEL_IP_WATCHDOG,
		MODEL_IP_OCTAL_SERIAL,
		MODEL_IP_ADC_16,
		MODEL_IP_TRIDO_48,
		MODEL_IP_DIO_316,
		MODEL_IP_DUAL_32CT,
		MODEL_IP_DID_48,
		MODEL_IP_DAC_128V,
		MODEL_IP_ADC_128F1,
		MODEL_IP_FNAL,
		MODEL_IP_UCD,
		MODEL_IP_UCD5,
		MODEL_IP_TSLATMR8,
		MODEL_IP_MDATRX,
		MODEL_IP_177,
		MODEL_IP_TWTPM,
		MODEL_IP_MDATUCD,
		MODEL_IP_320,
		MODEL_IP_330,
		MODEL_IP_470,
		MODEL_IP_480_6,
		MODEL_IP_480_2,
		MODEL_IP_502
};
	static const char *module_name[]={
		"Green Spring IP_Ethernet Lance",
		"Green Spring IP-488",
		"Green Spring IP Watchdog",
		"Green Spring IP-Octal 232",
		"Green Spring IP-16ADC",
		"Systran TRIO48",
		"Systran DIO316I",
		"Systran DUAL32CT",
		"Systran DID48",
		"Systran DAC128V",
		"Systran ADC128F1",
		"FNAL UNSPECIFIED MODULE",
		"FNAL UCD",
		"FNAL UCD5",
		"FNAL TSLATMR8",
		"FNAL MDATRX UCD",
		"FNAL IP177",
		"FNAL IPTWTPM",
		"FNAL IPMDATUCD",
		"Acromag IP320",
		"Acromag IP330",
		"Acromag IP470",
		"Acromag IP480",
		"Acromag IP480_2",
		"Acromag IP502",
		"NOT IN TABLE"
};
/**************************************************************/
/* This function  prints out info about what IP modules it finds attached.
   The typical value of the ip_base address argument is 0xfff58000
   which is has defined in the .h file.
   DJN 8/11/98  */
/**************************************************************/
void Industry_Pack_List(unsigned char *ip_base)
{
	struct IPACK ip;
	int i, ii, j, jj;
	unsigned short module;
	unsigned char *model_name;
	unsigned char *user_data;

	if (Industry_Pack(ip_base, 0, &ip)==ERROR) 
	{
	  printf ("\r\n *** NOT A VALID CARRIER *** address at %p\r\n",ip_base);
	  return;
	}

	for (i=0;i<sizeof(carrier_type)/2;i++)
	  if (carrier_type[i]==ip.carrier) break;
	printf ("\r\nIndustry Pack at address %p; Carrier type is %s",
		ip_base,carrier_name[i]);
	for (i=0;i<MAX_SLOTS;i++)
	{
	  if (ip.adr[i]!=NULL)
	  {
	    printf ("\r\nIP Slot %d at address %p; Manufacturer=%x Module=%x Revision=%x",
	      i,
	      ip.adr[i],*(ip.adr[i]+ID_MANUFACTURER),
	      *(ip.adr[i]+ID_MODEL),
 	      *(ip.adr[i]+ID_REVISION));
	    module = (*(ip.adr[i]+ID_MANUFACTURER)<<8)+
		*(ip.adr[i]+ID_MODEL);
            for (ii=0;ii<sizeof(module_type)/2;ii++)
               if (module_type[ii]==module) break;
	    printf ("\r\n  %s ",module_name[ii]);
	    user_data = ip.adr[i]+USER_DATA;
	    if (*(ip.adr[i]+ID_MANUFACTURER)==FNAL_MANUFACTURER)
	    {
	      for (ii=0;ii<16;ii=ii+2)
		printf ("%c",user_data[ii]);
	      for (j=0;j<sizeof(FNAL_Model_name)/sizeof(unsigned char *);j++)
	      {
	        model_name = FNAL_Model_name[j];
	        user_data = ip.adr[i]+USER_DATA;
	        for (jj=0;jj<8;jj++)
		{
		  if (*model_name++ != *user_data) break;
		  user_data +=2;
		}
	        if (jj>=8)
	          printf (" Pseudo Module #=0x%x",j);
	      }	      
	    }
	  }
	}
	printf ("\r\n");
}
/**************************************************************/
/* Industry_Pack_Array().  Added Aug. 11, 1998 Dennis Nicklaus.
   This function is basically just like Industry_Pack_List,
   with two exceptions:
   1: You don't pass it the base IP ID area address, it uses the default
   from the include file.
   2: It's argument is a pointer to an array of shorts. The array
   is filled in with the module numbers found at each slot.
   The array is treated as an array of length MAX_SLOTS. 

   The same info is printed out as in  Industry_Pack_List.
*/
/**************************************************************/

void Industry_Pack_Array(unsigned short *modules_return)
{
	struct IPACK ip;
	int i, ii, j, jj;
	unsigned short module;
	unsigned char *model_name;
	unsigned char *user_data;

	if (Industry_Pack((unsigned char *)IP_IO_AREA_BASE, 0, &ip)==ERROR) 
	{
	  printf ("\r\n *** NOT A VALID CARRIER *** address at %p\r\n",IP_IO_AREA_BASE);
	  return;
	}

	for (i=0;i<sizeof(carrier_type)/2;i++)
	  if (carrier_type[i]==ip.carrier) break;
	printf ("\r\nIndustry Pack at address %p; Carrier type is %s",
		IP_IO_AREA_BASE,carrier_name[i]);
	for (i=0;i<MAX_SLOTS;i++)
	{
	  if (ip.adr[i]!=NULL)
	  {
	    printf ("\r\nIP Slot %d at address %p; Manufacturer=%x Module=%x Revision=%x",
	      i,
	      ip.adr[i],*(ip.adr[i]+ID_MANUFACTURER),
	      *(ip.adr[i]+ID_MODEL),
 	      *(ip.adr[i]+ID_REVISION));
	    module = (*(ip.adr[i]+ID_MANUFACTURER)<<8)+
		*(ip.adr[i]+ID_MODEL);

	    /* note this line provides the returned values.  DJN 8/11/998 */
	    if (modules_return != NULL) modules_return[i] = module;
	      
            for (ii=0;ii<sizeof(module_type)/2;ii++)
               if (module_type[ii]==module) break;
	    printf ("\r\n  %s ",module_name[ii]);
	    user_data = ip.adr[i]+USER_DATA;
	    if (*(ip.adr[i]+ID_MANUFACTURER)==FNAL_MANUFACTURER)
	    {
	      for (ii=0;ii<16;ii=ii+2)
		printf ("%c",user_data[ii]);
	      for (j=0;j<sizeof(FNAL_Model_name)/sizeof(unsigned char *);j++)
	      {
	        model_name = FNAL_Model_name[j];
	        user_data = ip.adr[i]+USER_DATA;
	        for (jj=0;jj<8;jj++)
		{
		  if (*model_name++ != *user_data) break;
		  user_data +=2;
		}
	        if (jj>=8)
	          printf (" Pseudo Module #=0x%x",j);
	      }	      
	    }
	  }
	  else {
	    /* note this line provides the returned values.  DJN 8/11/998 */
	    if (modules_return != NULL) modules_return[i] = 0;
	  }
	}
	printf ("\r\n");
}
int IP_Interrupt_Enable (struct IPACK *ip, int irq)
{
	int i;

/*	printf ("\r\nIE irq=%d\r\n",irq);*/
	switch (ip->carrier) 
	{
          case VME162:
	  for (i=0;i<VME162_Slots;i++) 
	    if (ip->adr[i]!=NULL)
	      VME162_IP_Interrupt_Enable(ip->adr[i],i,irq);
	  break;
          case GreenSpring_VIPC610:
/*
	  for (i=0;i<GREENSPRING_VIPC610_Slots;i++) 
	    if (ip->adr[i]!=NULL)
	      VIPC610_IP_Interrupt_Enable(ip->adr[i],i,irq);
*/
	  break;
	  case Systran_VMESC5:
	  for (i=0;i<SYSTRAN_VMESC5_Slots;i++) 
	    if (ip->adr[i]!=NULL)
	      VMESC5_IP_Interrupt_Enable(ip->adr[i],i,irq);
	  break;
	  default:
	    printf ("\r\nNO CARRIER FOUND - IE\r\n");

	}
}
int VME_IP_Interrupt_Enable (unsigned char *addr, int irq)
{
	int i;
	struct IPACK ip;

/*	printf ("\r\nIE irq=%d\r\n",irq);*/
	Industry_Pack((unsigned char *)((unsigned long)addr&0xFFFFF000), 0, &ip);
	switch (ip.carrier) 
	{
          case VME162:
	  for (i=0;i<VME162_Slots;i++) 
	    if (ip.adr[i]==addr)
	      VME162_IP_Interrupt_Enable(ip.adr[i],i,irq);
	  break;
          case GreenSpring_VIPC610:
/*
	  for (i=0;i<GREENSPRING_VIPC610_Slots;i++) 
	    if (ip.adr[i]==addr)
	      VIPC610_IP_Interrupt_Enable(ip.adr[i],i,irq);
*/
	  break;
	  case Systran_VMESC5:
	  for (i=0;i<SYSTRAN_VMESC5_Slots;i++) 
	    if (ip.adr[i]==addr)
	      VMESC5_IP_Interrupt_Enable(ip.adr[i],i,irq);
	  break;
	  default:
	    printf ("\r\nNO CARRIER FOUND - IE\r\n");

	}
}
int VME162_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq)
{
/* Clear memory page. */
/*	*(unsigned char *)(IP_GEN_CNTRL_REG_BASE+slot) = 0;*/
/* Enable IPC int0 */
	*(unsigned char *)(IP_INT_CNTRL_BASE+(slot*2)) = IPC_INT_EN | irq;
/* Enable IPC int1 */
	*(unsigned char *)(IP_INT_CNTRL_BASE+(slot*2)+1) = IPC_INT_EN | irq;
	return 0;
}
int VME162_IP_Memory_Enable (unsigned char *addr, int slot, char *mem)
{
/* Set up the IPIC for IP memory at 72000000 */
  *(unsigned char *)(IP_GEN_CNTRL_REG_BASE+slot) = 0x0;

/* size of IP memory is 512K bytes */
  *(unsigned char *)(IP_MEM_SIZE_BASE+slot) = 0x07; 

  *(unsigned short *)(IP_MEM_BASE_BASE+(slot*2)) = 
  		(unsigned short)((unsigned long)mem>>16);

/* Set up memory access widths */

  *(unsigned char *)(IP_GEN_CNTRL_REG_BASE+slot) = 0x39;
  return 0;  
}
int VMESC5_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq)
{
/* mask off slot specific address lines for module address */
	(unsigned long)addr &= 0xFFFFF000;
/*	if (*(addr+VMESC5_General_Purpose+(slot*2)) == VMESC5_GP_InUse)
	  return -1;*/
/* Reset slot - 200 ms reset pulse to IP module */
/*	*(addr+VMESC5_Reset) = 1<<slot;
	taskDelay (20);*/
/* Enable IPC int0 */
/* Enable IPC int1 */
	*(addr+VMESC5_Interrupt+(slot*2)) = (irq<<4)+irq;
/* Mark slot in use */
/*	*(addr+VMESC5_General_Purpose+(slot*2)) = VMESC5_GP_InUse;*/
	return 0;
}
int VIPC610_IP_Interrupt_Enable (unsigned char *addr, int slot, int irq)
{
/* Its all done with jumpers, check your board */
	return 0;
}
