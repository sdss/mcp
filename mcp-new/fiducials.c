/*
 * Code to handle the fiducials on all three axes
 */
#include <stdio.h>
#include <math.h>
#include <ctype.h>
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
#include "cmd.h"

/*
 * Data from reading a latch
 */
struct LATCH_POS {
   int axis;				/* which axis */
   double pos1;				/* first encoder position */
   double pos2;				/* second encoder position */
};

static void DIO316ClearISR_delay(void);
static void restore_fiducials(int axis);
static void tm_print_fiducial(int axis);

SEM_ID semLatch = NULL;			/* semaphore for updating fiducials */
MSG_Q_ID msgLatchReenable = NULL;	/* reset the DIO316 interrupt */
MSG_Q_ID msgLatched = NULL;		/* signal that we're latched */

static FILE *fidfp = NULL;
static int latchidx = 0;

#define MAX_LATCHED	2000
struct LATCH_POS latchpos[MAX_LATCHED];

/*
 * structure for the primary set of fiducials - one per axis
 */
struct FIDUCIARY fiducial[NAXIS] = {
   {FALSE, 0, 33, 31016188, 0, 0, 0},		/* AZ:  120:45:44.9 */
   {FALSE, 0, 1,  3766415+58807, 0, 0, 0},	/* ALT: 14:39:23:286 */
   {FALSE, 0, 83, ROT_FID_BIAS + 336370/2, 0, 0, 0}/* ROT: 000:59:45.00 */
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
      TRACE(0, "fiducial for axis %s not crossed", axis_name(axis), 0);
      fprintf(stderr,"fiducial for axis %s not crossed\n", axis_name(axis));
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
 * Set the maximum error that can be corrected by MS.ON
 */
int
set_max_fiducial_correction(int axis,	/* the axis in question */
			    int max_correction)	/* the max correction */
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"mcp_set_fiducial: illegal axis %d\n", axis);

      return(-1);
   }
			
   fiducial[axis].max_correction = max_correction;

   TRACE(1, "Setting maximum MS.ON correction for %s to %d",
	 axis_name(axis), max_correction);


   return(0);
}

/*****************************************************************************/
/*
 * Given an axis and an error in that axis based on the last fiducial
 * crossing consider updating the position of that axis
 */
static void
maybe_reset_axis_pos(int axis,		/* the axis */
		     int pos_is_valid,	/* is the poserr correct? */
		     int poserr,	/* poserr for the axis */
		     int all_correct)	/* allow any correction? */
{
   long correction;			/* correction to apply */
/*
 * Remember that poserr?  We could do something cleverer here, based
 * on maintaining a history of errors.  But let's not.
 */
   if(pos_is_valid) {
      fiducial[axis].error = poserr;
   }
/*
 * Actually apply the adjustment
 */
   if(fiducial[axis].ms_on) {
      correction = -fiducial[axis].error;
      			
      if(all_correct) {
	 TRACE(1, "Applying unlimited correction %ld to %s",
	       correction, axis_name(axis));
      } else {
	 if(axis_stat[axis].ms_on_correction_too_large) {
	    TRACE(0, "MS.ON is disabled for %s; not applying ",
		  axis_name(axis), correction);
	    return;
	 }
	 
	 TRACE(1, "Applying correction %ld to %s",
	       correction, axis_name(axis));
	 
	 if(abs(correction) >= fiducial[axis].max_correction) {
	    TRACE(0, "    correction %ld is too large (max %ld)",
		  correction, fiducial[axis].max_correction);
	    
	    if(semTake(semSLCDC, WAIT_FOREVER) == ERROR) {
	       TRACE(0, "couldn't take semSLCDC semahore.", 0, 0);
	    } else {
	       axis_stat[axis].ms_on_correction_too_large = 1;
	       semGive(semSLCDC);
	    }
	    
	    return;
	 }
      }
      
      tm_adjust_pos(axis, correction);
      
      invalidate_fiducial_errors(axis);
   }
}

/*****************************************************************************/
/*
 * Convert a LATCH_POS and the previous value of the latch to a fiducial index
 *
 * The rotator indices run from fiducial 6 at 365.74 deg to 123 at -188.31 deg
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
/*
 * Are we getting closeish to +360? If so we have to be careful.
 * The preceeding code sets the fiducial at
 *   ~ 54561934 (323.06deg) --> fiducial 15
 * and the following ones
 *   ~ 55362669 (327.80deg) --> fiducial 90
 *   ~ 56163422 (332.55deg) --> fiducial 89
 * We must explicitly fix them to be 14, 13, ...
 */
   if(latchpos->pos1 > ROT_FID_BIAS + 55000000) {
      fididx = (fididx - 90) + 15 - 1;
   }

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
**
**=========================================================================
*/
void
tLatch(const char *name)
{
   int i;
   int fididx;
   int fididx1;
   time_t fidtim;
   MCP_MSG msg;				/* message to pass around */
   int pos_is_mark;			/* is this a "real" mark on
					   the rotator, not one of the dithered
					   ones? */
   int ret;				/* a return code */
   long rot_latch = 0;			/* position of last rotary latch seen*/
   int status;

   
   for(latchidx = -1;; latchidx = (latchidx + 1)%MAX_LATCHED) {
      if(latchidx < 0) {
	 latchidx = 0;
      } else {
/*
 * send message requesting the latches to rearm, reenabling interrupts
 */
	 msg.type = latchReenable_type;
	 msg.u.latchReenable.timeout = 120;	
	 msg.u.latchReenable.dio316int_bit = dio316int_bit;
 
	 ret = msgQSend(msgLatchReenable, (char *)&msg, sizeof(msg),
			NO_WAIT, MSG_PRI_NORMAL);
	 assert(ret == OK);
      }
/*
 * The interrupt routine DIO316_interrupt sends a message to msgLatched
 * when any axis sees a fiducial, so wait for it
 */
      ret = msgQReceive(msgLatched, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);
      
      TRACE(8, "read msg on msgLatched, delay = %dus",
	    timer_read(2) - msg.u.latchCrossed.time, 0);
      assert(msg.type == latchCrossed_type);

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

	 TRACE(1, "failed to read latch position: status = %d axis 0x%x",
	       status, dio316int_bit);

	 continue;
      }
/*
 * OK, we read a latch position so do something with it
 */
      ret = semTake(semLatch, WAIT_FOREVER);
      assert(ret != ERROR);

      fididx = -1;
      assert(latchidx >= 0 && latchidx < MAX_LATCHED); 
      latchpos[latchidx].axis = -9;

      if(dio316int_bit & AZIMUTH_INT) {
	 latchpos[latchidx].axis = AZIMUTH;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 get_latched_position(0, &latchpos[latchidx].pos1);
	 get_latched_position(1, &latchpos[latchidx].pos2);
	 semGive(semMEI);

	 fididx1 = barcode_serial(3); /* backwards from what you'd think */
	 fididx = barcode_serial(3);	/* read twice...not reliable */
	 if(fididx <= 0 || fididx > 24) {
	    TRACE(0, "Invalid barcode in azimuth: fididx = %d, pos = %d",
		  fididx, latchpos[latchidx].pos1);
	 } else {
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
	    
	    if(fididx < 0 || fididx >= N_AZ_FIDUCIALS) {
	       TRACE(0, "Invalid azimuth fiducial %d, pos = %d",
		     fididx, latchpos[latchidx].pos1);
	    } else {
	       az_fiducial[fididx].last = az_fiducial[fididx].mark;
	       az_fiducial[fididx].mark = latchpos[latchidx].pos1;
	       az_fiducial[fididx].err =
		 az_fiducial[fididx].mark - az_fiducial[fididx].last;
	       az_fiducial[fididx].poserr =
		 az_fiducial[fididx].mark - az_fiducial_position[fididx];
	       az_fiducial[fididx].markvalid = TRUE;
	       
	       TRACE(4, "az fiducial %.2f deg",
		     az_fiducial[fididx].mark/AZ_TICKS_DEG, 0);
	       if(az_fiducial[fididx].last == 0) {
		  TRACE(4, "     err = ???  poserr = %d ticks",
			az_fiducial[fididx].poserr, 0);
	       } else {
		  TRACE(4, "     err = %d   poserr = %d ticks",
			az_fiducial[fididx].err, az_fiducial[fididx].poserr);
	       }
	       
	       if(fididx == fiducial[AZIMUTH].index) {
		  fiducial[AZIMUTH].mark = az_fiducial[fididx].mark;
		  fiducial[AZIMUTH].markvalid = TRUE;
	       }
	       fiducialidx[AZIMUTH] = fididx;
	 
	       maybe_reset_axis_pos(AZIMUTH, 1, az_fiducial[fididx].poserr, 0);
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
	 fididx = (read_clinometer() + 7.5)/15;
			
	 if(fididx < 0 || fididx >= N_ALT_FIDUCIALS) {
	    TRACE(0, "Invalid altitude fiducial %d, clino = %d",
		  fididx, (int)(read_clinometer() + 0.5));
	 } else {
	    if(fidfp != NULL) {
	       fidtim = time(&fidtim);
	       time(&fidtim);
	       
	       fprintf(fidfp, "%d\t%d\t%.24s:%f\t%ld\t%ld\n",
		       latchpos[latchidx].axis, fididx,
		       ctime(&fidtim), sdss_get_time(),
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
	    
	    alt_fiducial[fididx].last = alt_fiducial[fididx].mark;
	    alt_fiducial[fididx].mark = latchpos[latchidx].pos1;
	    alt_fiducial[fididx].err =
	      alt_fiducial[fididx].mark - alt_fiducial[fididx].last;
	    alt_fiducial[fididx].poserr =
	      alt_fiducial[fididx].mark - alt_fiducial_position[fididx];
	    alt_fiducial[fididx].markvalid = TRUE;
	    
	    TRACE(4, "alt fiducial %.2f deg",
		  alt_fiducial[fididx].mark/ALT_TICKS_DEG, 0);
	    if(alt_fiducial[fididx].last == 0) {
	       TRACE(4, "     err = ???  poserr = %d ticks",
		     alt_fiducial[fididx].poserr, 0);
	    } else {
	       TRACE(4, "     err = %d   poserr = %d ticks",
		     alt_fiducial[fididx].err, alt_fiducial[fididx].poserr);
	    }
	    
	    if(fididx==fiducial[ALTITUDE].index) {
	       fiducial[ALTITUDE].mark = alt_fiducial[fididx].mark;
	       fiducial[ALTITUDE].markvalid = TRUE;
	    }
	    fiducialidx[ALTITUDE] = fididx;
	    maybe_reset_axis_pos(ALTITUDE, 1, alt_fiducial[fididx].poserr, 0);
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
		  TRACE(4, "     err = ???  poserr = %d ticks",
			rot_fiducial[fididx].poserr, 0);
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
	 
	 maybe_reset_axis_pos(INSTRUMENT, 1, rot_fiducial[fididx].poserr, 0);
      }

      semGive(semLatch);
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
	msgQReceive(msgLatchReenable, (char*)&msg, sizeof(msg), WAIT_FOREVER);
      assert(status != ERROR);

      TRACE(6, "DIO316ClearISR_delay: received message %d %d",
	    msg.type, msg.u.latchReenable.dio316int_bit);
      
      assert(msg.type == latchReenable_type);
      dio316int_bit = msg.u.latchReenable.dio316int_bit;
/*
 * OK, we have our orders
 */
      DIO316ClearISR(tm_DIO316);
      taskDelay(msg.u.latchReenable.timeout);
      
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
**	Set the last passed position as the "known" position for the fiducial
**	if it was marked as valid.  This does not automatically save these
**	settings to shared memory and the fiducials should be calibrated
**	before executing this function.
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
int
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
    default:
      TRACE(0, "Impossible axis in set_fiducials: %d", axis, 0);
      return(-1);
   }

   return(0);
}

void
set_fiducials_all(void)
{
   int i;
   
   for(i = 0; i < 3; i++) {
      (void)set_fiducials(i);
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

/*****************************************************************************/
/*
 * MS ("MagnetoSensor") commands --- what the MCP calls fiducials, and
 * the MEI calls latches. Sigh.
 *
 * First two commands that we'll never support:
 *	MS.MAP.DUMP - Displays the last all fiducials if fed back to the 
 *	MCP will restore the positions.
 *
 *	MS.MAP.LOAD ms# val - Loads the specified fiducials.
 */
char *
ms_map_dump_cmd(char *cmd)		/* NOTUSED */
{
  return "";
}
			
char *
ms_map_load_cmd(char *cmd)		/* NOTUSED */
{
  return "";
}

/*
 * Here are two that we do use (but that the TCC is oblivious of) to set
 * the fiducials arrays to the values that we last crossed, and to save
 * the current fiducials to shared memory
 */
char *
ms_set_all_cmd(char *cmd)		/* NOTUSED */
{
   if(set_fiducials(axis_select) < 0) {
      return("ERR: invalid axis");
   }
   
   return "";
}

char *
ms_save_cmd(char *cmd)			/* NOTUSED */
{
   if(save_fiducials(axis_select) < 0) {
      return("ERR: invalid axis");
   }
   
   return "";
}

/*****************************************************************************/
/*
 * Read fiducials for a current axis from a file
 */
char *
ms_read_cmd(char *cmd)
{
   char filename[200];			/* name of file to read */

   if(sscanf(cmd, "%s", filename) != 1) {
      return("ERR: no filename supplied");
   }

   return(read_fiducials(filename, axis_select));
}

/*****************************************************************************/
/*
 * Set a fiducial, i.e. set an axis' position to a fiducial's known value
 */
char *
ms_set_axis_pos_cmd(char *cmd)		/* NOTUSED */
{
   mcp_set_fiducial(axis_select);

   return("");
}

/*=========================================================================
**=========================================================================
**
**     MS.OFF - turn off automatic setting of positions as fiducials are passed
**     MS.ON - turn on automatic setting of positions as fiducials are passed.
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
**=========================================================================
*/
char *
ms_off_cmd(char *cmd)			/* NOTUSED */
{
   int ret;
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   ret = semTake(semLatch, WAIT_FOREVER);
   assert(ret != ERROR);
			
   fiducial[axis_select].ms_on = 0;

   semGive(semLatch);

   return "";
}

char *
ms_on_cmd(char *cmd)			/* NOTUSED */
{
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   if(semTake(semLatch, 60) == ERROR) {
      return("ERR: ms_on_cmd cannot take semLatch");
   }
   
   fiducial[axis_select].ms_on = 1;

   maybe_reset_axis_pos(axis_select, 0, 0, 0);

   semGive(semLatch);

   return "";
}

/*
 * Set the maximum change in position that MS.ON is allowed to make
 */
char *
ms_max_cmd(char *cmd)			/* NOTUSED */
{
   long ms_max;				/* maximum change due to MS.ON */

   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
			
   if(sscanf(cmd, "%ld", &ms_max) != 1) {
      return("ERR: maximum offset supplied");
   }
			
   set_max_fiducial_correction(axis_select, ms_max);

   return "";
}

/*****************************************************************************/
/*
 * Correct the position of the selected axis by an unlimited amount
 */
char *
correct_cmd(char *cmd)			/* NOTUSED */
{
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   if(semTake(semLatch, 60) == ERROR) {
      return("ERR: correct_cmd cannot take semLatch");
   }

   maybe_reset_axis_pos(axis_select, 0, 0, 1);

   semGive(semLatch);

   return "";
}

/*=========================================================================
**=========================================================================
**
**	Save/restore to/from shared memory the fiducials.  
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

int
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
    default:
      TRACE(0, "Impossible axis in save_fiducials: %d", axis, 0);
      return(-1);
   }

   return(0);
}

void
save_fiducials_all(void)
{
  int i;

  for(i = 0; i < 3; i++) {
     (void)save_fiducials(i);
  }
}

/*****************************************************************************/
/*
 * Restore the fiducials from shared memory. See also read_fiducials
 */
void
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

/*****************************************************************************/
/*
 * Restore the fiducials from a file. See also restore_fiducials
 */
char *
read_fiducials(const char *file,	/* file to read from */
	       int axis)		/* which axis to read */
{
   char axis_str[40];			/* name of axis from file */
   int bias;				/* bias applied to fiducial positions*/
   float error;				/* error in mark */
   char *expected_axis_str = NULL;	/* expected name of axis from file */
   int fid;				/* fiducial number from file */
   long *fiducials;			/* array to set */
   FILE *fil;				/* F.D. for file */
   char line[200];			/* buffer to read lines of file */
   char *lptr;				/* pointer to line[] */
   float mark;				/* mark from file */
   int npt;				/* number of points used to find mark*/
   int n_fiducials;			/* max number of fiducials */

   switch (axis) {
    case AZIMUTH:
      expected_axis_str = "az";
      n_fiducials = N_AZ_FIDUCIALS;
      fiducials = az_fiducial_position;
      bias = 0;
      break;
    case ALTITUDE:
      expected_axis_str = "alt";
      n_fiducials = N_ALT_FIDUCIALS;
      fiducials = alt_fiducial_position;
      bias = 0;
      break;
    case INSTRUMENT:
      expected_axis_str = "rot";
      n_fiducials = N_ROT_FIDUCIALS;
      fiducials = rot_fiducial_position;
      bias = ROT_FID_BIAS;
      break;
    default:
      fprintf(stderr,"read_fiducials: illegal axis %d\n", axis);
      return("ERR: illegal axis");
   }
/*
 * Open file, read header and data, and set fiducials array
 */
   if((fil = fopen(file, "r")) == NULL) {
      TRACE(0, "Failed to open %s for read", file, 0);
      return("ERR: failed to open file\n");
   }
   
   while((lptr = fgets(line, 200, fil)) != NULL) {
      while(isspace(*lptr)) lptr++;

      if(*lptr == '\0') {		/* blank line */
	 continue;
      } else if(*lptr == '#') {		/* a comment */
	 lptr++;
	 while(isspace(*lptr)) lptr++;
	 if(*lptr == '\0') {		/* empty comment */
	    continue;
	 }

	 if(sscanf(lptr, "%s fiducials", axis_str) == 1) {
	    if(strcmp(axis_str, expected_axis_str) != 0) {
	       fprintf(stderr,"Expected fiducials of type %s; saw %s\n",
		       expected_axis_str, axis_str);
	    }
	 }
							  
	 continue;
      }

      if(sscanf(lptr, "%d %f +- %f %d", &fid, &mark, &error, &npt) == 4) {
	 if(fid < 0 || fid >= n_fiducials) {
	    fprintf(stderr,"Invalid fiducial %d", fid);
	    continue;
	 }
	 fiducials[fid] = mark + bias;
      } else {
	 fprintf(stderr, "Corrupt line: %s\n", line);
	 continue;
      }
   }
			
   fclose(fil);

   return("");
}

/*=========================================================================
**
**	Diagnostic to see the fiducial positions and errors.
**
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
		  (az_fiducial[i].last == 0 ? 0 : az_fiducial[i].err),
		  az_fiducial[i].poserr);
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
		  (alt_fiducial[i].last == 0 ? 0 : alt_fiducial[i].err),
		  alt_fiducial[i].poserr);
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
		  (rot_fiducial[i].last == 0 ? 0 : rot_fiducial[i].err),
		  rot_fiducial[i].poserr);
	} else {
	   if(show_all) {
	      printf("ROT %d:  pos=%ld\n",i, rot_fiducial_position[i]);
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

/*=========================================================================
 *
 * Initialize the fiducials.
 *
 * Create message queues and logfiles
 *
 * Declare fiducial commands to the command interpreter
 *
 * Mark all fiducials invalid and restore known positions from shared memory
 * (but set the fixed fiducial for each axis to a known point ignoring the
 * saved value)
 *
 * Spawn tasks
 */
void
tLatchInit(void)
{
   time_t fidtim;
   const char *name = "latch.dat";	/* name of logfile */
   int i;

   fidfp = fopen(bldFileName(name),"a");
   if(fidfp == NULL) {
      printf ("Open file error: %s\n",name);
   } else {
      printf ("Open file %s; %p\n",name,fidfp);

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
 * Create semaphore and message queues
 */
   if(semLatch == NULL) {
      semLatch = semBCreate(SEM_Q_FIFO, SEM_FULL);
   }
   
   if(msgLatched == NULL) {
      msgLatched = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgLatched != NULL);
   }

   if(msgLatchReenable == NULL) {
      msgLatchReenable = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgLatchReenable != NULL);
	 
      taskSpawn("tm_ClrInt", 30, 8, 4000, \
		(FUNCPTR)DIO316ClearISR_delay, 120, dio316int_bit,
		0,0,0,0,0,0,0,0);
   }
/*
 * Declare commands to the command interpreter
 */
   define_cmd("CORRECT",      correct_cmd,         0, 1, 1);
   define_cmd("MS.MAP.DUMP",  ms_map_dump_cmd,     0, 0, 1);
   define_cmd("MS.MAP.LOAD",  ms_map_load_cmd,     0, 1, 1);
   define_cmd("MS.OFF",       ms_off_cmd,          0, 1, 1);
   define_cmd("MS.ON",        ms_on_cmd,           0, 1, 1);
   define_cmd("MS.MAX",       ms_max_cmd,          1, 1, 1);
   define_cmd("MS.READ",      ms_read_cmd,         1, 1, 1);
   define_cmd("MS.SAVE",      ms_save_cmd,         0, 1, 1);
   define_cmd("MS.SET",       ms_set_axis_pos_cmd, 0, 1, 1);
   define_cmd("MS.SET.ALL",   ms_set_all_cmd,      0, 1, 1);
   define_cmd("SET.FIDUCIAL", ms_set_axis_pos_cmd, 0, 1, 1);
/*
 * Prepare to read fiducials
 */
   arm_latch(TRUE);
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
   
   for(i = 0; i < sizeof(alt_fiducial)/sizeof(struct FIDUCIALS); i++) {
      alt_fiducial[i].markvalid=FALSE;
      alt_fiducial[i].last=0;
      alt_fiducial[i].err=0;
      alt_fiducial[i].poserr=0;
      alt_fiducial[i].mark=0;
      alt_fiducial_position[i]=0;
   }
   
   for(i = 0; i < sizeof(rot_fiducial)/sizeof(struct FIDUCIALS); i++) {
      rot_fiducial[i].markvalid=FALSE;
      rot_fiducial[i].last=0;
      rot_fiducial[i].err=0;
      rot_fiducial[i].poserr=0;
      rot_fiducial[i].mark=0;
      rot_fiducial_position[i]=0;
   }
/*
 * Restore fiducial positions from shared memory
 */
   restore_fiducials(AZIMUTH);
   restore_fiducials(ALTITUDE);
   restore_fiducials(INSTRUMENT);
/*
 * Set the "canonical" fiducials, overriding the ones we just restored
 */
   az_fiducial_position[fiducial[AZIMUTH].index] =
					      fiducial[AZIMUTH].known_position;
   
   alt_fiducial_position[fiducial[ALTITUDE].index] =
					     fiducial[ALTITUDE].known_position;
#if 0
   alt_fiducial_position[0]=0x0;	/* 00:00:00:00 */
   alt_fiducial_position[6]=0x0160E6C6;	/* 090:00:00:00 */
#endif
			
   rot_fiducial_position[fiducial[INSTRUMENT].index] =
					   fiducial[INSTRUMENT].known_position;
/*
 * Set the maximum correction to an axis position that MS.ON can set 
 */
   for(i = 0; i < NAXIS; i++) {
      (void)set_max_fiducial_correction(i, 0);
   }
/*
 * Spawn the task that processes fiducial crossings
 */
   taskSpawn("tLatch",49,VX_FP_TASK,10000,(FUNCPTR)tLatch,
	     0,0,0,0,0,0,0,0,0,0);
}
