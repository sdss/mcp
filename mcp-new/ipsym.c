#include "copyright.h"
/************************************************************************/
/*   File:	bsym.c							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	Broadsym info (V1.00) : vxWorks			*/
/*   Modules:	*/
/*++ Version:
  1.00 - initial version
--*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/
/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"
#include "sysLib.h"
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "taskLib.h"
#include "sigLib.h"
#include "etherLib.h"
#include "inetLib.h"
#include "sockLib.h"
#include "symLib.h"
#include "symbol.h"
#include "ioLib.h"
#include "stdio.h"
#include "rebootLib.h"
#include "timers.h"
#include "time.h"
#include "timerint.h"

SEM_ID semsym_1Hz=0;
struct sockaddr_in sym_sockaddr;
int sym_s;
char sym_adr[12];
char *datatypes[]={"uchar","char","ushort","short","ulong","long","float","double"};

struct SYM_HEADER {
	unsigned char vers;
#define VERSION		1
	unsigned char type;
#define UNUSED_TYPE	0
#define CANCEL_TYPE	1
#define SET	2
#define GET	3
#define GETDATA_TYPE	4
#define ENABLE_TYPE	5
#define DISABLE_TYPE	6
	unsigned char req;
	unsigned char err;
#define SUCCESS				0
#define SYMBOL_NOT_FOUND		1
#define VERSION_NOT_SUPPORTED		2	
#define TYPE_NOT_SUPPORTED		3
#define TRIGGER_NOT_IMPLEMENTED		4
#define BUFFER_TOO_LARGE		5
#define FREQ_NOT_SUPPORTED		6
#define DATA_NOT_COLLECTED		7
	unsigned short buffer1KB;
	unsigned short trigger_type;
#define CONTINUOUS_TRIGGER		1
#define MANUAL_TRIGGER			2
#define MOTION_TRIGGER			3
	unsigned char trigger_data;
	unsigned char frequency;
	unsigned char cnt;
};
struct SYM_FRAME {
	struct SYM_HEADER sym_hdr;
	unsigned char symbols[1024];
};
struct SYM_FRAME
sf={{VERSION,UNUSED_TYPE,1,SUCCESS,50,MANUAL_TRIGGER,0,1,7},
	'_','f','r','e','q','t','i','c','k',0,
		'l','o','n','g',0,
/*
	'_','r','a','w','t','i','c','k',0,
		'l','o','n','g',0,
	'_','r','a','w','d','a','t','a','s','i','z','e',0,
		'l','o','n','g',0,
		'd','o','u','b','l','e',0,
*/
	'*','_','a','x','i','s','4','p','o','s',0,
		'l','o','n','g',0,
	'*','_','a','x','i','s','4','c','m','d',0,
		'l','o','n','g',0,
	'*','_','a','x','i','s','5','p','o','s',0,
		'l','o','n','g',0,
	'*','_','a','x','i','s','4','e','r','r',0,
		's','h','o','r','t',0,
	'*','_','a','x','i','s','4','t','i','m',0,
		'l','o','n','g',0,
/*
	'*','_','a','x','i','s','4','a','c','c','e','l',0,
		'l','o','n','g',0,
	'*','_','a','x','i','s','4','v','e','l',0,
		'l','o','n','g',0,
*/
	'*','_','d','a','c','4','o','u','t',0,
		's','h','o','r','t',0,
	' '};
struct SYM_HEADER dh={VERSION,UNUSED_TYPE,1,0,0,0,0,0,0};
int *rawdata=NULL;
int *rawdataptr=NULL;
int rawdatasize=0;
int freqtick=0;

char *sym_data=NULL;
int sym_enable=FALSE;
int trig_enable=FALSE;
int sym_size=0;
int sym_bufsize=0;
char *sym_addr[40];
char sym_type[40];
char sym_len[40];
void serverData(int hz, void (*data_routine()));
void serverDataCollection();
int 	serverDummy();
int	serverCancel();
int	serverSetSym();
int	serverGetSym();
int	serverGetData();
int	serverEnable();
int	serverDisable();
void manTrg();

int (*serverType[])() = {
		serverDummy,
		serverCancel,
		serverSetSym,
		serverGetSym,
		serverGetData,
		serverEnable,
		serverDisable
};

void symServe ( int symFd, char *addr, unsigned short port)
{
  int nrd=0,nsym;
  char *brd, *bsym;
  int i,n;
  
  
  brd=(char *)&sf;
  FOREVER
  {
  while (nrd<sizeof(struct SYM_HEADER))
  {
    if (n = read(symFd,brd,sizeof(struct SYM_FRAME)-nrd)<0)
    {
      close(symFd);
      return;
    }
    brd +=n;
    nrd +=n;
  }
  nsym = nrd-sizeof(struct SYM_HEADER);
  bsym = &sf.symbols[0];
  for (i=sf.sym_hdr.cnt;i<0;i--)
  {
    while (*bsym++!=EOS) 
    {
      if(nsym--<=0)
      {
        if (n = read(symFd,brd,sizeof(struct SYM_FRAME)-nrd)<0)
        {
          close(symFd);
          return;
        }
        brd +=n;
        nrd +=n;
        nsym +=n;
      }
    }
  }
  sym_size=nrd;
  printf ("\r\n Server request from %s; port %d of type %d",
  		addr,port,sf.sym_hdr.type);
  n=serverType [sf.sym_hdr.type]();
  if (n>0) write(sym_s, (char *)&sf,n);
  }
}
int 	serverDummy()
{
  return 0;
}
int	serverCancel()
{
  sf.sym_hdr.cnt=0;
  return serverDisable();
}
int	serverSetSym()
{
  int i,ii;
  char symtype;
  SYM_TYPE *bsym;
  extern SYMTAB_ID sysSymTbl;
  STATUS stat;
    
  serverDisable();
  if ((rawdata!=NULL)&&(sym_size!=sf.sym_hdr.buffer1KB)) 
  {
    free(rawdata);
    rawdata=NULL;
  }
  if (sym_size!=sf.sym_hdr.buffer1KB)
  { 
    rawdata=(int *)malloc (sf.sym_hdr.buffer1KB*1024*sf.sym_hdr.cnt);
  }
  rawdataptr = rawdata;
  rawdatasize = 0;
  sym_size=sf.sym_hdr.buffer1KB;
  bsym=(char *)&sf.symbols[0];
  sf.sym_hdr.err=SUCCESS;
  for (i=0;i<sf.sym_hdr.cnt;i++)
  {
    if (*bsym=='*') 
    {
      bsym++;
      stat=symFindByName(sysSymTbl,bsym,(char **)&sym_addr[i],
  			(SYM_TYPE *)&symtype);
      sym_addr[i]=(char *)*(char **)sym_addr[i];
    }
    else
    {
      stat=symFindByName(sysSymTbl,bsym,(char **)&sym_addr[i],
  			(SYM_TYPE *)&symtype);
    }
    printf ("\r\n %s: val=%p, typ=%d stat=%d",
    	bsym,sym_addr[i],symtype,stat);
    if (symtype==0) sf.sym_hdr.err = SYMBOL_NOT_FOUND;
    while (*bsym++!=EOS); 
    for (ii=0;ii<sizeof(datatypes)/sizeof(char *);ii++) 
      if (strcmp (bsym,datatypes[ii])==0)break;
    sym_type[i]=ii;
    switch (sym_type[i])
    {
	case 0:
	case 1:
	  sym_len[i]=1;
	  break;
	case 2:
	case 3:
	  sym_len[i]=2;
	  break;
	case 4:
	case 5:
	case 6:
	  sym_len[i]=4;
	  break;
	case 7:
	  sym_len[i]=8;
	  break;
      }

    printf ("\r\n  %s is the data type=%d",datatypes[ii],ii);
    while (*bsym++!=EOS); 
  }
  sym_bufsize=(int)bsym-(int)&sf;
  dh.req=sf.sym_hdr.req+1;
  dh.buffer1KB =sf.sym_hdr.buffer1KB;
  dh.trigger_type =sf.sym_hdr.trigger_type;
  if (dh.trigger_type==CONTINUOUS_TRIGGER) trig_enable=TRUE;
  else trig_enable=FALSE;
  dh.trigger_data =sf.sym_hdr.trigger_data;
  dh.frequency=sf.sym_hdr.frequency;
  dh.cnt=sf.sym_hdr.cnt;
  dh.err=sf.sym_hdr.err;
  serverEnable();
  return sizeof (struct SYM_HEADER);
}
int	serverGetSym()
{
  return sym_bufsize;
}
int	serverGetData()
{
  write(sym_s, (char *)&dh,sizeof(struct SYM_HEADER));
  if (rawdata!=NULL)
    write(sym_s, (char *)rawdata,rawdatasize);
  if (dh.trigger_type==CONTINUOUS_TRIGGER) 
    manTrg();
  return 0;
}
int	serverEnable()
{
  sym_enable=TRUE;
  return sizeof (struct SYM_HEADER);
}
int	serverDisable()
{
  sym_enable=FALSE;
  return sizeof (struct SYM_HEADER);
}
SEM_ID semDC=NULL;
SEM_ID semTRG=NULL;
void serverDCStart()
{
  freqtick++;
  if ((dh.frequency==0) || ((freqtick%dh.frequency)==0))
    if (semDC!=NULL) semGive (semDC);
}
void server_shutdown(int type)
{
    printf("SERVER Timer4Stop shutdown: \r\n");
/*    Timer4Stop ();*/
    sysIntDisable (5);
    taskDelay(30);
}
void serverData(int hz, void (*data_routine()))
{
  int i;
  char *chardataptr;
  short *shortdataptr;
  double *doubledataptr;
  extern void serverDataCollection();

  rebootHookAdd((FUNCPTR)server_shutdown);
  semDC = semBCreate (SEM_Q_FIFO,SEM_EMPTY);
/*  Timer4Start (hz,5,serverDCStart); */
  for (;;)
  {
    if (semTake (semDC,WAIT_FOREVER)!=ERROR)
    {
      if (data_routine!=NULL) data_routine();
      if ((sym_enable)&&(trig_enable))
      {
       if ( (rawdatasize<(dh.buffer1KB*1024)) && (rawdataptr!=NULL))
         for (i=0;i<dh.cnt;i++)
         {
           switch (sym_len[i])
           {
             case 1:
	       chardataptr = (char *)rawdataptr;
               *chardataptr++ = *(char *)sym_addr[i];
	       rawdataptr = (int *) chardataptr;
	       break;
             case 2:
	       shortdataptr = (short *)rawdataptr;
               *shortdataptr++ = *(short *)sym_addr[i];
	       rawdataptr = (int *) shortdataptr;
	       break;
             case 4:
               *rawdataptr++ = *(int *)sym_addr[i];
	       break;
             case 8:
	       doubledataptr = (double *)rawdataptr;
               *doubledataptr++ = *(double *)sym_addr[i];
	       rawdataptr = (int *) doubledataptr;
	       break;
           }
           rawdatasize +=sym_len[i];
        }
        else
          trig_enable=FALSE;
      }  
    }
  }
}
int     serverFile(char *name)
{
  int fd;

  fd=open (name,O_RDWR|O_CREAT,0x1B6);  /* 0666 */
  if (fd==ERROR) printf ("\r\nOpen file error: %s",name);
  write(fd, (char *)&dh,sizeof(struct SYM_HEADER));
  write(fd, (char *)rawdata,rawdatasize);
  if (dh.trigger_type==CONTINUOUS_TRIGGER)
    manTrg();
  close (fd);
  return 0;
}
int     serverJFile(char *name, char *text)
{
  FILE *fp;
  int i,ii;
  long *rd;
  unsigned char *ucrd;
  char *crd;
  unsigned short *usrd;
  short *srd;
  float *frd;
  double *drd;
  int samplesize;

  int rds;
  char *bsym;

  if (dh.cnt==0) 
  {
    printf ("No symbols are specified or set\r\n");
    return -1;
  }
  fp=fopen (name,"w");
  if (fp==NULL) printf ("\r\nOpen file error: %s",name);
  rds=rawdatasize;
  samplesize=0;
  for (i=0;i<dh.cnt;i++) samplesize += sym_len[i];
  fprintf(fp,"ARRAY '%s' %d %d T\r\n",text,dh.cnt-1,rds/samplesize);
  bsym = &sf.symbols[0];
  for (i=0;i<dh.cnt;i++)
  {
    if (*bsym=='*') bsym++;
    fprintf(fp,"'%s ",bsym+1);
    bsym += (strlen(bsym)+1);
    fprintf(fp,"%s' ",bsym);
    bsym += (strlen(bsym)+1);
  }
  fprintf(fp,"\r\n");
  if (rawdata!=NULL)
  {
  rd = (long *)rawdata;
  for (i=0;i<rds/(samplesize);i++)
  {
    for (ii=0;ii<dh.cnt;ii++)
    {
      switch (sym_type[ii])
      {
	case 0:
	  ucrd = (unsigned char *)rd;
          fprintf(fp,"%d ",(unsigned long)*ucrd++);
	  rd = (long *)ucrd;
	  break;
	case 1:
	  crd = (unsigned char *)rd;
          fprintf(fp,"%d ",(long)*crd++);
	  rd = (long *)crd;
	  break;
	case 2:
	  usrd = (unsigned short *)rd;
          fprintf(fp,"%hd ",*usrd++);
	  rd = (long *)usrd;
	  break;
	case 3:
	  srd = (short *)rd;
          fprintf(fp,"%hd ",*srd++);
	  rd = (long *)srd;
	  break;
	case 4:
          fprintf(fp,"%u ",*rd++);
	  break;
	case 5:
          fprintf(fp,"%ld ",*rd++);
	  break;
	case 6:
	  frd = (float *)rd;
          fprintf(fp,"%f ",(double)*frd++);
	  rd = (long *)frd;
	  break;
	case 7:
	  drd = (double *)rd;
          fprintf(fp,"%f ",*drd++);
	  rd = (long *)drd;
	  break;
      }
    }
    fprintf (fp,"\r\n");
  }
  }
  if (dh.trigger_type==CONTINUOUS_TRIGGER)
    manTrg();
  fclose (fp);
  return 0;
}

int serverSDDSFile(char *name, char *text)
{
  FILE *fp;
  int i,ii;
  long *rd;
  unsigned char *ucrd;
  char *crd;
  unsigned short *usrd;
  short *srd;
  float *frd;
  double *drd;
  int samplesize;

  int rds;
  char *bsym;

  if (dh.cnt==0) 
  {
    printf ("No symbols are specified or set\r\n");
    return -1;
  }
  fp=fopen (name,"w");
  if (fp==NULL) printf ("\r\nOpen file error: %s",name);
  rds=rawdatasize;
  samplesize=0;
  for (i=0;i<dh.cnt;i++) samplesize += sym_len[i];
  fprintf(fp,"SDDS1\r\n");
  fprintf(fp,"&description text=\"%s\", contents=\"VxWorks Symbols\" &end\r\n",
			text);
  bsym = &sf.symbols[0];
  for (i=0;i<dh.cnt;i++)
  {
    if (*bsym=='*') bsym++;
    fprintf(fp,"&column name=%s, ",bsym+1);
    bsym += (strlen(bsym)+1);
    fprintf(fp,"type=double, units=%s &end\r\n",bsym);
    bsym += (strlen(bsym)+1);
  }
  fprintf(fp,"&data mode=ascii &end\r\n");
  fprintf(fp,"%d\r\n",rds/samplesize);
  if (rawdata!=NULL)
  {
  rd = (long *)rawdata;
  for (i=0;i<rds/(samplesize);i++)
  {
    for (ii=0;ii<dh.cnt;ii++)
    {
      switch (sym_type[ii])
      {
	case 0:
	  ucrd = (unsigned char *)rd;
          fprintf(fp,"%d ",(unsigned long)*ucrd++);
	  rd = (long *)ucrd;
	  break;
	case 1:
	  crd = (unsigned char *)rd;
          fprintf(fp,"%d ",(long)*crd++);
	  rd = (long *)crd;
	  break;
	case 2:
	  usrd = (unsigned short *)rd;
          fprintf(fp,"%hd ",*usrd++);
	  rd = (long *)usrd;
	  break;
	case 3:
	  srd = (short *)rd;
          fprintf(fp,"%hd ",*srd++);
	  rd = (long *)srd;
	  break;
	case 4:
          fprintf(fp,"%u ",*rd++);
	  break;
	case 5:
          fprintf(fp,"%ld ",*rd++);
	  break;
	case 6:
	  frd = (float *)rd;
          fprintf(fp,"%f ",(double)*frd++);
	  rd = (long *)frd;
	  break;
	case 7:
	  drd = (double *)rd;
          fprintf(fp,"%f ",*drd++);
	  rd = (long *)drd;
	  break;
      }
    }
    fprintf (fp,"\r\n");
  }
  }
  if (dh.trigger_type==CONTINUOUS_TRIGGER)
    manTrg();
  fclose (fp);
  return 0;
}

int serverMATLABFile(char *name, char *text)
{
  FILE *fp;
  int i,ii;
  long *rd;
  unsigned char *ucrd;
  char *crd;
  unsigned short *usrd;
  short *srd;
  float *frd;
  double *drd;
  int samplesize;

  int rds;
  char *bsym;

  if (dh.cnt==0) 
  {
    printf ("No symbols are specified or set\r\n");
    return -1;
  }
  fp=fopen (name,"w");
  if (fp==NULL) printf ("\r\nOpen file error: %s",name);
  rds=rawdatasize;
  samplesize=0;
  for (i=0;i<dh.cnt;i++) samplesize += sym_len[i];
  fprintf(fp,"%%SDDS1 - MATLAB format\r\n");
  fprintf(fp,"%%&description text=\"%s%%\", contents=\"VxWorks Symbols\" &end\r\n",
			text);
  bsym = &sf.symbols[0];
  for (i=0;i<dh.cnt;i++)
  {
    if (*bsym=='*') bsym++;
    fprintf(fp,"%%&column name=%s, ",bsym+1);
    bsym += (strlen(bsym)+1);
    fprintf(fp,"type=double, units=%s &end\r\n",bsym);
    bsym += (strlen(bsym)+1);
  }
  fprintf(fp,"%%&data mode=ascii &end\r\n");
  fprintf(fp,"%%%d\r\n",rds/samplesize);
  if (rawdata!=NULL)
  {
  rd = (long *)rawdata;
  for (i=0;i<rds/(samplesize);i++)
  {
    for (ii=0;ii<dh.cnt;ii++)
    {
      switch (sym_type[ii])
      {
	case 0:
	  ucrd = (unsigned char *)rd;
          fprintf(fp,"%d ",(unsigned long)*ucrd++);
	  rd = (long *)ucrd;
	  break;
	case 1:
	  crd = (unsigned char *)rd;
          fprintf(fp,"%d ",(long)*crd++);
	  rd = (long *)crd;
	  break;
	case 2:
	  usrd = (unsigned short *)rd;
          fprintf(fp,"%hd ",*usrd++);
	  rd = (long *)usrd;
	  break;
	case 3:
	  srd = (short *)rd;
          fprintf(fp,"%hd ",*srd++);
	  rd = (long *)srd;
	  break;
	case 4:
          fprintf(fp,"%u ",*rd++);
	  break;
	case 5:
          fprintf(fp,"%ld ",*rd++);
	  break;
	case 6:
	  frd = (float *)rd;
          fprintf(fp,"%f ",(double)*frd++);
	  rd = (long *)frd;
	  break;
	case 7:
	  drd = (double *)rd;
          fprintf(fp,"%f ",*drd++);
	  rd = (long *)drd;
	  break;
      }
    }
    fprintf (fp,"\r\n");
  }
  }
  if (dh.trigger_type==CONTINUOUS_TRIGGER)
    manTrg();
  fclose (fp);
  return 0;
}
#ifdef NOTDEFINED
int sym_ini ()
{
    int optval;
    int sym_port=0x6806;

    sym_s = socket (AF_INET, SOCK_STREAM, 0);	/* get a tcp socket */
    if (sym_s < 0)
    {
      printf ("symbol socket error (errno = %#x)\r\n", errnoGet ());
      return;
    }
    bzero (&sym_sockaddr, sizeof (sym_sockaddr));
    sym_sockaddr.sin_family      = AF_INET;
    sym_sockaddr.sin_port        = htons(sym_port);
    if (bind (sym_s, (struct sockaddr *)&sym_sockaddr, sizeof (sym_sockaddr)) == ERROR)
    {
      printf ("symbol bind error (errno = %#x)\r\n", errnoGet ());
      return;
    }
    if (listen (sym_s, 10)==ERROR)
    {
      printf ("symbol listen error (errno = %#x)\r\n", errnoGet ());
      return;
    }

}
void symbol_scope()
{
  int status;
  int newFd,ix=0;
  struct sockaddr_in newclient;
  int socksize;
  char symName[16];
  
  socksize=sizeof(struct sockaddr_in);
  sym_ini();

  FOREVER 
  {
    
    if ((newFd = accept (sym_s, (struct sockaddr *)&newclient,
    &socksize))==ERROR)
    {
      printf ("symbol accept error (errno = %#x)\r\n", errnoGet ());
      close (newFd);
      return;
    }
    sprintf (symName,"SymServer%d",ix++);
    if (taskSpawn (symName,100,VX_FP_TASK,1000,symServe,newFd,
    	(int)inet_ntoa(newclient.sin_addr),
    	ntohs(newclient.sin_port),0,0,0,0,0,0,0)== ERROR)
    {
      printf ("symbol spawn error (errno = %#x)\r\n", errnoGet ());
      close (newFd);
      return;    
    }
  }
}
#endif
void clrSym()
{
  sf.sym_hdr.cnt=0;
}
void addSym(char *name,char *type)
{
  int cnt,i;
  
  cnt=0;
  for (i=0;i<sf.sym_hdr.cnt*2;i++)
    cnt += strlen(&sf.symbols[i+cnt]);	/* end of symbols */
  if (name[0]=='*') 			/* check for indirection */
  {
    strcpy (&sf.symbols[i+cnt],"*");
    strcpy (&sf.symbols[i+cnt+1],"_");	/* add for name resolution */
    strcpy (&sf.symbols[i+cnt+2],&name[1]);	/* name specified */
  }
  else
  {
    strcpy (&sf.symbols[i+cnt],"_");	/* add for name resolution */
    strcpy (&sf.symbols[i+cnt+1],name);	/* name specified */
  }
  strcpy (&sf.symbols[i+cnt+2+strlen(name)],type);
  sf.sym_hdr.cnt++;
}
void sizSym(unsigned short size)
{
  sf.sym_hdr.buffer1KB=size;
}
void trgSym (char mode)
{
  sf.sym_hdr.trigger_type=mode;
}
void frqSym (char frequency)
{
  sf.sym_hdr.frequency=frequency;
}
void setSym ()
{
  serverSetSym();
}
void manTrg()
{
  rawdataptr = rawdata;
  rawdatasize = 0;
  freqtick=0;
  trig_enable=TRUE;
  if (semTRG!=NULL) semGive (semTRG);
}
void stpTrg()
{
  trig_enable=FALSE;
}
int stsTrg()
{
  if ((rawdatasize<(dh.buffer1KB*1024))&&(trig_enable))
  { 
    printf ("\r\n...Collecting rawdatasize=%d",rawdatasize);
    return FALSE;
  }
  else
  {  
    printf ("\r\n...Trigger Complete");
    return TRUE;
  }
}
void pollTrg()
{
  extern char *get_date();

  while (!stsTrg()) taskDelay (60);
  serverSDDSFile("sdds.dat",(char *)get_date());
/*  serverMATLABFile("matlab.dat",(char *)get_date());*/
}
void taskTrg()
{
  if (semTRG==NULL) semTRG = semBCreate (SEM_Q_FIFO,SEM_EMPTY);
  for (;;)
  {
    if (semTake (semTRG,WAIT_FOREVER)!=ERROR)
    taskDelay (4*60);
    pollTrg();
  }
}
