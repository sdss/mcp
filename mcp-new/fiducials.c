/*
 * Code to handle the fiducials on all three axes
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <vxWorks.h>
#include <semLib.h>
#include <taskLib.h>
#include "pcdsp.h"
#include "tm.h"
#include "gendefs.h"
#include "mcpMsgQ.h"
#include "mcpFiducials.h"
#include "mcpTimers.h"
#include "axis.h"
#include "data_collection.h"
#include "dscTrace.h"
#include "serial.h"

/*
 * Data from reading a latch
 */
struct LATCH_POS {
   int axis;				/* which axis */
   double pos1;				/* first encoder position */
   double pos2;				/* second encoder position */
};

void tLatch(const char *name);
static void DIO316ClearISR_delay(void);
static void restore_fiducials(int axis);
static void tm_print_fiducial(int axis);

SEM_ID semLATCH = NULL;
static FILE *fidfp = NULL;
static int latchidx = 0;

#define MAX_LATCHED	2000
struct LATCH_POS latchpos[MAX_LATCHED];

/*
 * axis fiducial max error to post msg
 */
static int errmsg_max[3] = {400, 200, 100};

/*
 * structure for the primary set of fiducials - one per axis
 */
struct FIDUCIARY fiducial[3] = {
   {FALSE, 0, 33, 31016188},		/* AZ:  120:45:44.9 */
   {FALSE, 0, 1,  3766415+58807},	/* ALT: 14:39:23:286 */
   {FALSE, 0, 83, ROT_FID_BIAS + 336370/2}/* ROT: 000:59:45.00 */
};

/*
 * structure for all the fiducials
 */
struct FIDUCIALS az_fiducial[N_AZ_FIDUCIALS];
struct FIDUCIALS alt_fiducial[N_ALT_FIDUCIALS];
struct FIDUCIALS rot_fiducial[N_ROT_FIDUCIALS];
long az_fiducial_position[N_AZ_FIDUCIALS];
long alt_fiducial_position[N_ALT_FIDUCIALS];
long rot_fiducial_position[N_ROT_FIDUCIALS];
int fiducialidx[NAXIS] = {-1, -1, -1};	/* last fiducial crossed */

static char const *bldFileName(char const* newName);
static void invalidate_fiducial_errors(int axis);

/*=========================================================================
**=========================================================================
**
**	Initialize the fiducials marking all invalid and restoring known
**	positions from shared memory.  Always update the fixed fiducial
**	for each axis to a known point.
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
void
tLatchInit(void)
{
   time_t fidtim;
   const char *name = "latch.dat";	/* name of logfile */
   int i;

   fidfp = fopen(bldFileName(name),"a");
   if(fidfp == NULL) {
      printf ("\r\nOpen file error: %s",name);
   } else {
      printf ("\r\nOpen file %s; %p",name,fidfp);

#if 0
      setvbuf(fidfp,NULL,_IOLBF,0);
      rebootHookAdd((FUNCPTR)fiducial_shutdown);
#endif
      
      time(&fidtim);
      fprintf(fidfp,"#RESTART......... %s %.24s\n",
	       bldFileName(name),ctime(&fidtim));
      fprintf(fidfp,
	      "#Axis\tIndex\tDate & Time:SDSStime\tPosition1\tPosition2\n");
#if 1
      fflush(fidfp);
#else
      fclose(fidfp);
      fidfp = fopen(bldFileName(name),"a");
#endif
   }
/*
 * Create semaphores and message queues
 */
   if(semLATCH == NULL) {
      semLATCH = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
   }

   if(msgDIO316ClearISR == NULL) {
      msgDIO316ClearISR = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgDIO316ClearISR != NULL);
	 
      taskSpawn("tm_ClrInt", 30, 8, 4000, \
		(FUNCPTR)DIO316ClearISR_delay, 120, dio316int_bit,
		0,0,0,0,0,0,0,0);
   }
/*
 * Initialise arrays
 */
   for(i = 0; i < sizeof(az_fiducial)/sizeof(struct FIDUCIALS); i++) {
      az_fiducial[i].markvalid=FALSE;
      az_fiducial[i].last = 0;
      az_fiducial[i].err=0;
      az_fiducial[i].poserr=0;
      az_fiducial[i].mark=0;
      az_fiducial_position[i]=0;
   }
   
   restore_fiducials(AZIMUTH);
   az_fiducial_position[fiducial[AZIMUTH].index] =
					      fiducial[AZIMUTH].known_position;
   
   for(i = 0; i < sizeof(alt_fiducial)/sizeof(struct FIDUCIALS); i++) {
      alt_fiducial[i].markvalid=FALSE;
      alt_fiducial[i].last=0;
      alt_fiducial[i].err=0;
      alt_fiducial[i].poserr=0;
      alt_fiducial[i].mark=0;
      alt_fiducial_position[i]=0;
   }
   
   restore_fiducials(ALTITUDE);
   alt_fiducial_position[fiducial[1].index] =
					     fiducial[ALTITUDE].known_position;
   alt_fiducial_position[0]=0x0;	/* 00:00:00:00 */
   alt_fiducial_position[6]=0x0160E6C6;	/* 090:00:00:00 */
   
   for(i = 0; i < sizeof(rot_fiducial)/sizeof(struct FIDUCIALS); i++) {
      rot_fiducial[i].markvalid=FALSE;
      rot_fiducial[i].last=0;
      rot_fiducial[i].err=0;
      rot_fiducial[i].poserr=0;
      rot_fiducial[i].mark=0;
      rot_fiducial_position[i]=0;
   }
   
   restore_fiducials(INSTRUMENT);
   rot_fiducial_position[fiducial[INSTRUMENT].index] =
					   fiducial[INSTRUMENT].known_position;
   
   taskSpawn("tLatch",49,VX_FP_TASK,10000,(FUNCPTR)tLatch,
	     0,0,0,0,0,0,0,0,0,0);
}

/*****************************************************************************/
/*
 * Set a fiducial
 */
int
mcp_set_fiducial(int axis)
{
   int correction;			/* how much to correct position */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"mcp_set_fiducial: illegal axis %d\n", axis);

      return(-1);
   }

#ifdef ROT_ROTARY_ENCODER
#  error I do not know how to read rotary encode fiducial
#endif
   
   if(!fiducial[axis].markvalid) {
      fprintf(stderr,"fiducial for axis %s not crossed", axis_name(axis));
      return(-1);
   }

   correction = fiducial[axis].known_position - fiducial[axis].mark;

   TRACE(1, "Setting %s fiducial; offsetting by %d ticks",
	 axis_name(axis), correction);

   if(tm_adjust_pos(axis, correction) < 0) {
      TRACE(0 ,"Failed to adjust position for axis %s", axis_name(axis), 0);
      fprintf(stderr,"Failed to adjust position for axis %s", axis_name(axis));
      return(-1);
   }

   if (fidfp != NULL) {
      time_t fidtim;
      
      time (&fidtim);
      fprintf(fidfp, "%s\t%d\t%.25s:%f\t%ld\t%ld\n",
	      axis_name(axis), fiducial[axis].index,
	      ctime(&fidtim), sdss_get_time(), correction,
	      (long)(*tmaxis[axis]).actual_position);
   }

   invalidate_fiducial_errors(axis);
  
   fiducial[axis].mark = fiducial[axis].known_position;

   return(0);
}

/*****************************************************************************/
/*
 * Return a logfile name. This will be replaced by open_log() as soon as
 * the MJD problems are sorted out
 */
static char const*
bldFileName(char const* newName)
{
   static char buffer[1024] = "";
/*
 * If the new name isn't NULL, then we'll be modifying the static
 * value. If it is NULL, we'll just return the current contents
 */
   if(newName == NULL) {
      return(buffer);
   }
/*
 * If the string isn't empty, then build up a path;
 * If the string is empty, we clear out the static value.
 */
   if(*newName != '\0') {
      strcpy(buffer, "/mcptpm/");
      strncat(buffer, "newName", sizeof(buffer));
   } else {
      buffer[0] = '\0';
   }

   return buffer;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: fiducial_shutdown
**
** DESCRIPTION:
**	Flush the file and close it.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
fiducial_shutdown(int type)
{
  printf("fiducial file shutdown: FP=%p\r\n",fidfp);
  
  fclose (fidfp);
#if defined(CALL_NSF)
  destroyNfsConnection();
#endif
}

/*****************************************************************************/
/*
 * Convert a LATCH_POS and the previous value of the latch to a fiducial index
 */
static int
get_rot_fididx(const struct LATCH_POS *latchpos, /* the most recent pos. */
	       int last_latch_pos,	/* previous latch pos */
	       int *big)		/* is this a big interval? */
{
   int fididx;				/* the desired index */
   
   if(abs((long)latchpos->pos1 - last_latch_pos) < 250000) {
      return(0);			/* we just crossed the same fiducial
					   twice */
   }
/*
 * Find the index from the Heidenhain
 */
   fididx = abs(iround((latchpos->pos1 - last_latch_pos)/800.));
   fididx -= 500;
   
   if(fididx > 0) {
      *big = 1;
   } else {
      *big = 0;
      fididx = -fididx;
   }
/*
 * Convert that to the SDSS numbering system, putting fiducial 84 at ~ -1deg
 * 44 at ~ 185deg, 46 at ~ 176deg
 */
   if(fididx > 45 && latchpos->pos1 > ROT_FID_BIAS) {
      fididx -= 76;
   } else {
      if(fididx < 35 && latchpos->pos1 < ROT_FID_BIAS) {
	 fididx += 76;
      }
   }
   fididx += 45;
 	       
   if(fididx <= 0 || fididx >= N_ROT_FIDUCIALS) {
      fprintf(stderr,"Illegal fididx = %d\n", fididx);
      return(-1);
   }

   return(fididx);
}

/*=========================================================================
**=========================================================================
**
**	The azimuth has 48 entries for +-360 degrees.  The azimuth uses small
**	segments of optical tape with reference marks to interrupt the MCP.
**	A barcode reader is triggered to read a corresponding UPC which 
**	contains an absolute index.  The position must be aligned within
**	approximately 120 deg for this mechanism to work since the code
**	must determine if the azimuth is wrapped or not.
**
**	The altitude has 7 entries for 0-90 degrees.  The altitude also uses
**	segments of optical tape with reference marks to interrupt the MCP, 
**	but the barcode reader is disabled (failed).  The clinometer is used
**	to judge the index for the reference crossing.  The clinometer must
**	be accurate to with +-7 degrees for this to work correctly.
**
**	The rotator has 156 entries to exceed +-360 degrees.  The rotator
**	uses the same optical tape but it is continuous and provides an
**	absolute unique value between any two reference marks.  This value
**	is adjusted as an index into the table.  We only use the evenly
**      spaced marks (every ~ 5 degrees) at fiducials -- the "dithered"
**      intermediate marks are only used to decide which fiducial we've
**      seen.  We could use all of them, but there's no real need.
**
**	tm_latch is the routine triggered by the semaphore and processes
**	the reference corresponding to the axis causing the trigger.  Note:
**	when a latch occurs in the MEI, all axis are latched.  Also, if
**	numerous interrupts occur due to setting on the reference mark, then
**	the next interrupt is delayed by disabling the interrupt and not
**	enabling until a delay is expired.
**
** CALLS TO:
**	barcode_serial
**
** GLOBALS REFERENCED:
**	semMEI
**	sdssdc
**	latchpos
**	fiducial
**	fiducialidx
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**	altclino_off, altclino_sf
**
**=========================================================================
*/
void
tLatch(const char *name)
{
   int i;
   int fididx;
   int fididx1;
   int status;
   time_t fidtim;
   int ret;				/* return code from semTake() */
   int pos_is_mark;			/* is this a "real" mark on
					   the rotator, not one of the dithered
					   ones? */
   long rot_latch = 0;			/* position of last rotary latch seen*/
   
   for(latchidx = -1;; latchidx = (latchidx + 1)%MAX_LATCHED) {
      if(latchidx < 0) {
	 latchidx = 0;
      } else {
/*
 * send message requesting the latches to rearm, reenabling interrupts
 */
	 MCP_MSG msg;
	 STATUS stat;

	 msg.type = DIO316ClearISR_type;
	 msg.u.DIO316ClearISR.timeout = 120;	
	 msg.u.DIO316ClearISR.dio316int_bit = dio316int_bit;
 
	 stat = msgQSend(msgDIO316ClearISR, (char *)&msg, sizeof(msg),
			 NO_WAIT, MSG_PRI_NORMAL);
	 assert(stat == OK);
      }
/*
 * The interrupt routine DIO316_interrupt gives semLATCH, so wait for it
 */
      ret = semTake(semLATCH, WAIT_FOREVER);
      assert(ret != ERROR);
      TRACE(8, "Took semLATCH", 0, 0);

      for(i = 15, status = FALSE; status == FALSE && i > 0; i--) {
	 ret = semTake(semMEI, WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 status = (int)latch_status();
	 semGive(semMEI);
	 
	 taskDelay(1);
      }
      
      if(status == FALSE) {		/* we didn't see anything */
	 latchpos[latchidx].axis = -9;
	 
	 if(dio316int_bit & AZIMUTH_INT) {
	    latchpos[latchidx].axis = -(AZIMUTH+1);
	 }
	 if(dio316int_bit & ALTITUDE_INT) {
	    latchpos[latchidx].axis = -(ALTITUDE+1);
	 }
	 if (dio316int_bit & INSTRUMENT_INT) {
	    latchpos[latchidx].axis = -(INSTRUMENT+1);
	 }

	 TRACE(4, "BAD LATCH: latchidx = %d", latchidx, 0);

	 continue;
      }
/*
 * OK, we read a latch position so do something with it
 */
      fididx = -1;
      assert(latchidx >= 0 && latchidx < MAX_LATCHED); 
      latchpos[latchidx].axis = -9;

      if(dio316int_bit & AZIMUTH_INT) {
	 latchpos[latchidx].axis = AZIMUTH;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 get_latched_position(0, &latchpos[latchidx].pos1);
	 get_latched_position(1, &latchpos[latchidx].pos2);
	 semGive (semMEI);

	 fididx1 = barcode_serial(3); /* backwards from what you'd think */
	 fididx = barcode_serial(3);	/* read twice...not reliable */
	 if(fididx > 0 && fididx <= 24) {
	    if(latchpos[latchidx].pos1 > 0) {
	       fididx += 24;
	    }
	    
	    if(fidfp != NULL) {
	       fidtim = time(&fidtim);
	       time(&fidtim);
	       
	       fprintf(fidfp, "%d\t%d\t%.24s:%f\t%ld\t%ld\n",
		       latchpos[latchidx].axis,fididx,
		       ctime(&fidtim),sdss_get_time(),
		       (long)latchpos[latchidx].pos1,
		       (long)latchpos[latchidx].pos2);
	       fprintf(fidfp, "#first barcode reading=%d\n",fididx1);
#if 1
	       fflush(fidfp);
#else
	       fclose(fidfp);
	       fidfp = fopen(bldFileName(name),"a");
#endif
	    }
	    
	    if(fididx > 0 && fididx < N_AZ_FIDUCIALS) {
	       az_fiducial[fididx].last=az_fiducial[fididx].mark;
	       az_fiducial[fididx].mark=latchpos[latchidx].pos1;
	       az_fiducial[fididx].err=az_fiducial[fididx].mark-
		 az_fiducial[fididx].last;
	       az_fiducial[fididx].poserr=az_fiducial[fididx].mark-
		 az_fiducial_position[fididx];
	       az_fiducial[fididx].markvalid=TRUE;
	       
	       if(abs(az_fiducial[fididx].poserr) > errmsg_max[AZIMUTH] &&
		  az_fiducial_position[fididx] != 0) {
		  TRACE(0, "axis %d: poserr=%ld",
			  latchpos[latchidx].axis, az_fiducial[fididx].poserr);
		  TRACE(0, "        pos = %f",
			(float)latchpos[latchidx].pos1, 0);
	       }
	       if(fididx==fiducial[AZIMUTH].index) {
		  fiducial[AZIMUTH].mark = az_fiducial[fididx].mark;
		  fiducial[AZIMUTH].markvalid = TRUE;
	       }
	       fiducialidx[AZIMUTH] = fididx;
	    }
	 }
      }
      
      if(dio316int_bit & ALTITUDE_INT) {
	 latchpos[latchidx].axis = ALTITUDE;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 get_latched_position(2,&latchpos[latchidx].pos1);
	 get_latched_position(3,&latchpos[latchidx].pos2);
	 semGive (semMEI);

/*
 * turned off (failed hardware)
 * clinometer does a better job
 */
#if 0
	 fididx = barcode_serial(2);
	 fididx = barcode_serial(2);
#endif
	 fididx=((int)(abs(sdssdc.status.i4.alt_position-altclino_off)*
		       altclino_sf)+7.5)/15;
	 fididx++;
	 if(fididx != -1) {
	    fididx--;
	    if (fidfp != NULL) {
	       fidtim = time(&fidtim);
	       time(&fidtim);
	       
	       fprintf(fidfp, "%d\t%d\t%.24s:%f\t%ld\t%ld\n",
		       latchpos[latchidx].axis,fididx,
		       ctime(&fidtim),sdss_get_time(),
		       (long)latchpos[latchidx].pos1,
		       (long)latchpos[latchidx].pos2);
	       fprintf(fidfp, "#alt_position=%d\n",
		       sdssdc.status.i4.alt_position);
#if 1
	       fflush(fidfp);
#else
	       fclose(fidfp);
	       fidfp = fopen(bldFileName(name),"a");
#endif
	    }
	    
            if(fididx >= 0 && fididx < N_ALT_FIDUCIALS) {
	       alt_fiducial[fididx].last = alt_fiducial[fididx].mark;
	       alt_fiducial[fididx].mark = latchpos[latchidx].pos1;
	       alt_fiducial[fididx].err = alt_fiducial[fididx].mark -
						     alt_fiducial[fididx].last;
	       alt_fiducial[fididx].poserr = alt_fiducial[fididx].mark -
						 alt_fiducial_position[fididx];
	       alt_fiducial[fididx].markvalid = TRUE;

	       if(abs(alt_fiducial[fididx].poserr) > errmsg_max[ALTITUDE] &&
					  alt_fiducial_position[fididx] != 0) {
		  TRACE(0, "axis %d: poserr=%ld",
			  latchpos[latchidx].axis, az_fiducial[fididx].poserr);
		  TRACE(0, "        pos = %f",
			(float)latchpos[latchidx].pos1, 0);
	       }
	       
	       if(fididx==fiducial[ALTITUDE].index) {
		  fiducial[ALTITUDE].mark = alt_fiducial[fididx].mark;
		  fiducial[ALTITUDE].markvalid = TRUE;
	       }
	       fiducialidx[ALTITUDE] = fididx;
	    }
	 }
      }
      
      if(dio316int_bit & INSTRUMENT_INT) {
	 latchpos[latchidx].axis = INSTRUMENT;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);

#ifdef ROT_ROTARY_ENCODER
	    /* switch to 5 for optical encoder, when using rotary */
	 get_latched_position(5,&latchpos[latchidx].pos1);
	 get_latched_position(4,&latchpos[latchidx].pos2);
#else
	 get_latched_position(4, &latchpos[latchidx].pos1);
	 get_latched_position(5, &latchpos[latchidx].pos2);

	 latchpos[latchidx].pos1 += ROT_FID_BIAS; /* make always +ve */
	 latchpos[latchidx].pos2 += ROT_FID_BIAS;
#endif
	 semGive (semMEI);
	 
/*
 * have we already seen a rotator latch? If so, we know which encoder
 * is which
 */
	 if(rot_latch != 0) {
	    int big = 0;		/* was this the big or small interval
					   between marks on the tape? */
	    fididx = get_rot_fididx(&latchpos[latchidx], rot_latch, &big);

	    if(fididx < 0) {
	       fprintf(stderr,"Illegal fididx = %d\n", fididx);
	       
	       rot_latch = latchpos[latchidx].pos1;
	       continue;
	    }

	    if(fididx == 0) {
	       ;			/* we just crossed the same fiducial
					   twice */
	    } else {
	       rot_fiducial[fididx].last = rot_fiducial[fididx].mark;
/*
 * Is the current or previous latch the one to use? We only use the
 * evenly-spaced marks, not the dithered ones in between
 *
 * XXX We could in fact use all the reference marks, but that wasn't the
 * way that Charlie set things up.
 */
	       if((big && latchpos[latchidx].pos1 > rot_latch) ||
		  (!big && latchpos[latchidx].pos1 < rot_latch)) {
		  pos_is_mark = 0;
		  rot_fiducial[fididx].mark = rot_latch;
	       } else {
		  pos_is_mark = 1;
		  rot_fiducial[fididx].mark = latchpos[latchidx].pos1;
	       }
	       
	       if(rot_fiducial[fididx].last != 0) {
		  rot_fiducial[fididx].err =
		    rot_fiducial[fididx].mark - rot_fiducial[fididx].last;
	       }
	    }
/*
 * And poserr, the error relative to the known positions of the fiducials
 */
	    if(rot_fiducial_position[fididx] != 0) {
	       rot_fiducial[fididx].poserr =
		 rot_fiducial[fididx].mark - rot_fiducial_position[fididx];
	    }

	    if(fididx != 0) {
	       rot_fiducial[fididx].markvalid = TRUE;
	       fiducialidx[INSTRUMENT] = fididx;
	    }

	    if(fidfp != NULL) {
	       fidtim = time(&fidtim);
	       time (&fidtim);
	       
	       fprintf(fidfp, "%d\t%d\t%.24s:%f\t%ld\t%ld\n",
		       latchpos[latchidx].axis,fididx,
		       ctime(&fidtim), sdss_get_time(),
		       (long)latchpos[latchidx].pos1,
		       (long)latchpos[latchidx].pos2);
#if 1
	       fflush(fidfp);
#else
	       fclose(fidfp);
	       fidfp = fopen(bldFileName(name),"a");
#endif
	    }
	 }

	 if(fididx > 0) {
	    if(!pos_is_mark) {
	       TRACE(4, "Intermediate rot fiducial %.2f deg",
		     (latchpos[latchidx].pos1 - ROT_FID_BIAS)/ROT_TICKS_DEG,0);
	    } else {
	       TRACE(4, "rot fiducial %.2f deg",
		     (rot_fiducial[fididx].mark - ROT_FID_BIAS)/ROT_TICKS_DEG,
									    0);
	       if(rot_fiducial[fididx].last == 0) {
		  TRACE(4, "     err = ???  poserr = ??? ticks", 0, 0);
	       } else {
		  TRACE(4, "     err = %d   poserr = %d ticks",
			rot_fiducial[fididx].err,
			rot_fiducial[fididx].poserr);
	       }
	    }
	 }
	 
	 rot_latch = latchpos[latchidx].pos1;
	 if(fididx == fiducial[INSTRUMENT].index) {
            fiducial[INSTRUMENT].mark = rot_fiducial[fididx].mark;
            fiducial[INSTRUMENT].markvalid = TRUE;
	 }
      }
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: DIO316ClearISR_delay
**
** DESCRIPTION:
**	Task is spawned to delay enabling interrupt and arming latch
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DIO316
**	semMEI
**
**=========================================================================
*/
static void
DIO316ClearISR_delay(void)
{
   unsigned char dio316int_bit;		/* value read by DIO316_interrupt */
   MCP_MSG msg;				/* message to read */
   int status;

   for(;;) {
/*
 * Wait for a message asking us to do something
 */
      status =
	msgQReceive(msgDIO316ClearISR, (char*)&msg, sizeof(msg), WAIT_FOREVER);
      assert(status != ERROR);

      TRACE(6, "DIO316ClearISR_delay: received message %d %d",
	    msg.type, msg.u.DIO316ClearISR.dio316int_bit);
      
      assert(msg.type == DIO316ClearISR_type);
      dio316int_bit = msg.u.DIO316ClearISR.dio316int_bit;
/*
 * OK, we have our orders
 */
      DIO316ClearISR(tm_DIO316);
      taskDelay(msg.u.DIO316ClearISR.timeout);
      
      status = semTake(semMEI,WAIT_FOREVER);
      assert(status == OK);

      while((status = arm_latch(TRUE)) != DSP_OK) {
	 TRACE(4, "Trying to ARM Latch; status=%d", status, 0);
      }
      semGive (semMEI);

      if(dio316int_bit & AZIMUTH_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 1, DIO316_INT_ENA);
      }
      if(dio316int_bit & ALTITUDE_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 2, DIO316_INT_ENA);
      }
      if(dio316int_bit & INSTRUMENT_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 3, DIO316_INT_ENA);
      }
   }
}	 

/*=========================================================================
**=========================================================================
**
**	Each axis has one primary fiducial and it can be adjusted or changed
**	to a new index.
**
** GLOBALS REFERENCED:
**	fiducial  - structure for the primary set of fiducials
**	fiducial_position
**
**=========================================================================
*/
void
set_primary_fiducials(int axis,
		      int fididx,
		      long pos)
{
   switch (axis) {
    case AZIMUTH:
      if(fididx >= 0 && fididx < N_AZ_FIDUCIALS) {
	 fiducial[axis].index = fididx;
	 fiducial[axis].markvalid = FALSE;
	 fiducial[axis].mark = 0;
	 fiducial[axis].known_position = pos;
      }
      az_fiducial_position[fiducial[axis].index] =
						 fiducial[axis].known_position;
      break;	  
    case ALTITUDE:
      if(fididx >= 0 && fididx < N_ALT_FIDUCIALS) {
	 fiducial[axis].index=fididx;
	 fiducial[axis].markvalid=FALSE;
	 fiducial[axis].mark = 0;
	 fiducial[axis].known_position = pos;
      }
      alt_fiducial_position[fiducial[axis].index] = 
						 fiducial[axis].known_position;
      break;
    case INSTRUMENT:
      if(fididx >= 0 && fididx < N_ROT_FIDUCIALS) {
	 fiducial[axis].index=fididx;
	 fiducial[axis].markvalid=FALSE;
	 fiducial[axis].mark = 0;
	 fiducial[axis].known_position = pos;
      }
      rot_fiducial_position[fiducial[axis].index] = 
						 fiducial[axis].known_position;
      break;
    default:
      printf("set_primary_fiducials: unknown axis %d\n", axis);
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_fiducials_all
**	    set_fiducials
**
** DESCRIPTION:
**	Set the last passed position as the "known" position for the fiducial
**	if it was marked as valid.  This does not automatically save these
**	settings to shared memory and the fiducials should be calibrated
**	before executing this function.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
static void
set_fiducials(int axis)
{
   int i;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 if(az_fiducial[i].markvalid) {
	    az_fiducial_position[i]=az_fiducial[i].mark;
	    az_fiducial[i].poserr=0;
	 }
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 if (alt_fiducial[i].markvalid) {
	    alt_fiducial_position[i]=alt_fiducial[i].mark;
	    alt_fiducial[i].poserr=0;
	 }
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 if(rot_fiducial[i].markvalid) {
	    rot_fiducial_position[i]=rot_fiducial[i].mark;
	    rot_fiducial[i].poserr=0;
	 }
      }
      break;
   }
}

void
set_fiducials_all(void)
{
   int i;
   
   for(i = 0; i < 3; i++) {
      set_fiducials(i);
   }
}

/*****************************************************************************/
/*
 * Invalidate all fiducial "last", "err", and "poserr" values
 * for the given axis; usually called after setting a fiducial
 */
static void
invalidate_fiducial_errors(int axis)
{
   int i;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 az_fiducial[i].last = 0;
	 az_fiducial[i].err = az_fiducial[i].poserr = 0;
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 alt_fiducial[i].last = 0;
	 alt_fiducial[i].err = alt_fiducial[i].poserr = 0;
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 rot_fiducial[i].last = 0;
	 rot_fiducial[i].err = rot_fiducial[i].poserr = 0;
      }
      break;
   }
}

/*=========================================================================
**
** This will set the fiducial to the nearest valid fiducial crossed (in the
** future), but currently there is one "trusted" fiducial per axis.  This
** fiducial, if valid, will adjust the axis position appropriately.
*/
char *
correct_cmd(char *cmd)			/* NOTUSED */
{
   if(mcp_set_fiducial(axis_select) < 0) {
      return "ERR: failed to set fiducial\n";
   }

   return("");
}

/*=========================================================================
**=========================================================================
**
**	Save/restore to/from shared memory the fiducials.  
**	Does nothing with the primary
**	fiducials...which is an artifact of getting fiducials to work and
**	is becoming the working scenario.
**
** GLOBALS REFERENCED:
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
#define SM_AZ_FIDUCIALS		0x02810000
#define SM_ALT_FIDUCIALS	0x02811000
#define SM_ROT_FIDUCIALS	0x02812000

static void
save_fiducials(int axis)
{
   int i;
   long *sm;
   
   switch (axis) {
    case AZIMUTH:
      sm = (long *)SM_AZ_FIDUCIALS;
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 sm[i] = az_fiducial_position[i];
      }
      break;
    case ALTITUDE:
      sm = (long *)SM_ALT_FIDUCIALS;
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 sm[i] = alt_fiducial_position[i];
      }
      break;
    case INSTRUMENT:
      sm = (long *)SM_ROT_FIDUCIALS;
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 sm[i] = rot_fiducial_position[i];
      }
      break;
   }
}

void
save_fiducials_all(void)
{
  int i;

  for(i = 0; i < 3; i++) {
     save_fiducials(i);
  }
}

/*****************************************************************************/

static void
restore_fiducials(int axis)
{
   int i;
   long *sm;
   
   switch (axis) {
    case AZIMUTH:
      sm = (long *)SM_AZ_FIDUCIALS;
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 az_fiducial_position[i] = sm[i];
      }
      break;
    case ALTITUDE:
      sm = (long *)SM_ALT_FIDUCIALS;
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 alt_fiducial_position[i] = sm[i];
      }
      break;
    case INSTRUMENT:
      sm = (long *)SM_ROT_FIDUCIALS;
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 rot_fiducial_position[i] = sm[i];
      }
      break;
   }
}

void
restore_fiducials_all ()
{
   int i;
   
   for(i = 0; i < 3; i++) {
      restore_fiducials(i);
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_fiducials
**
** DESCRIPTION:
**	Diagnostic to see the fiducial positions and errors.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	az_fiducial
**	alt_fiducial
**	rot_fiducial
**	az_fiducial_position
**	alt_fiducial_position
**	rot_fiducial_position
**
**=========================================================================
*/
void
print_fiducials(int axis,		/* which axis */
		int show_all)		/* show all fiducials, including
					   ones we haven't crossed */
{
  int i;

  switch (axis) {
   case AZIMUTH:
     for(i = 0;i < N_AZ_FIDUCIALS; i++) {
	if(fiducial[axis].index==i) {
	   printf("*");
	   if(fiducial[axis].markvalid == i) printf ("!");
	}
	if(az_fiducial[i].markvalid) {
	   printf("AZ %d %d degs:  pos= %ld, mark= %ld, last= %ld "
		  " err= %ld, poserr= %ld\n",
		  i, (int)(az_fiducial[i].mark/AZ_TICKS_DEG),
		  az_fiducial_position[i],
		  az_fiducial[i].mark, az_fiducial[i].last,
		  az_fiducial[i].err,az_fiducial[i].poserr);
	} else {
	   if(show_all) {
	      printf("AZ %d:  pos=%ld\n", i, az_fiducial_position[i]);
	   }
	}     
     }
     break;
   case ALTITUDE:
     for(i = 0;i < N_ALT_FIDUCIALS; i++) {
	if(fiducial[axis].index == i) {
	   printf("*");
	   if(fiducial[axis].markvalid == i) printf ("!");
	}
	if(alt_fiducial[i].markvalid) {
	   printf("ALT %d %d degs:  pos= %ld, mark= %ld, last= %ld "
		  " err= %ld, poserr= %ld\n",
		  i, (int)(alt_fiducial[i].mark/ALT_TICKS_DEG),
		  alt_fiducial_position[i],
		  alt_fiducial[i].mark, alt_fiducial[i].last,
		  alt_fiducial[i].err,alt_fiducial[i].poserr);
	} else {
	   if(show_all) {
	      printf("ALT %d:  pos=%ld\n",i,alt_fiducial_position[i]);
	   }
	}
     }
     break;
   case INSTRUMENT:
     for(i = 0;i < N_ROT_FIDUCIALS; i++) {
	if(fiducial[axis].index==i) {
	   printf("*");
	   if(fiducial[axis].markvalid == i) printf ("!");
	}

	if(rot_fiducial[i].markvalid) {	   
	   printf("ROT %d %d degs: pos= %ld, mark= %ld, last= %ld "
		  " err= %ld, poserr= %ld\n", i,
		  (int)((rot_fiducial[i].mark - ROT_FID_BIAS)/ROT_TICKS_DEG),
		  rot_fiducial_position[i],
		  rot_fiducial[i].mark, rot_fiducial[i].last,
		  rot_fiducial[i].err,rot_fiducial[i].poserr);
	} else {
	   if(show_all) {
	      printf("ROT %d:  pos=%ld\n",i,rot_fiducial_position[i]);
	   }
	}     
     }
     break;
  }
}

/*****************************************************************************/

void
latchexcel(int axis)
{
  int i;

  date();
  printf ("axis\tlatch pos1\tlatch pos2\n");
  for(i = 0; i < latchidx; i++) {
     if(axis == latchpos[i].axis) {
	printf("%d\t%12.0f\t%12.0f\n",latchpos[i].axis,
	       (float)latchpos[i].pos1,(float)latchpos[i].pos2);
     }
  }
}

/*=========================================================================
**
**      Print the fiducials
**
*/
static void
tm_print_fiducial(int axis)
{
   long marcs,arcs,arcm,arcd;
   double arcsec, farcsec;
   int i;
   
   i = axis/2;

   switch (i) {
    case AZIMUTH:
      printf("Axis AZ(0):\n");
      arcsec = AZ_TICK*abs(fiducial[i].known_position);
      farcsec = AZ_TICK*abs(fiducial[i].mark);
      break;
    case ALTITUDE:
      printf("Axis ALT(2):\n");
      arcsec = ALT_TICK*abs(fiducial[i].known_position);
      farcsec = ALT_TICK*abs(fiducial[i].mark);
      break;
    case INSTRUMENT:
      printf("Axis ROT(4):\n");
      arcsec = ROT_TICK*abs(fiducial[i].known_position);
      farcsec = ROT_TICK*abs(fiducial[i].mark);
      break;
    default:
      printf("Illegal axis=%d\n",axis);
      return;
   }
   
   arcd = (long)arcsec/3600;	     
   arcm = ((long)arcsec - arcd*3600)/60;	     
   arcs = (long)arcsec - arcd*3600 - arcm*60;	     
   marcs = (arcsec - (long)arcsec)*1000;
   
   printf("Fiducial Position = %c%03ld:%02ld:%02ld:%03ld\n",
	  ((fiducial[i].known_position < 0) ? '-' : ' '),
	  arcd,arcm,arcs,marcs);

   arcd=(long)(farcsec)/3600;	     
   arcm=((long)(farcsec)-(arcd*3600))/60;	     
   arcs=((long)(farcsec)-(arcd*3600)-(arcm*60));	     
   marcs = (farcsec-(long)farcsec)*1000;
   printf("Fiducial Position Mark = ");
   if (fiducial[i].markvalid) {
      if (fiducial[i].mark<0)
	printf("-%03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
      else
	printf(" %03ld:%02ld:%02ld:%03ld",arcd,arcm,arcs,marcs);
   } else {
      printf("     NOT Valid");
   }
}

void
tm_print_fiducial_all(void)
{
   int i;
   
   for(i = 0; i < 3; i++) {
      tm_print_fiducial(2*i);
   }
}
