/*
 * Code to handle the fiducials on all three axes
 */
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <stat.h>
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
#include "mcpUtils.h"

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

static int latchidx = 0;

#define MAX_LATCHED	2000
struct LATCH_POS latchpos[MAX_LATCHED];

/*
 * structure for the primary set of fiducials - one per axis
 */
struct FIDUCIARY fiducial[NAXIS] = {
   {					/* Azimuth */
      FALSE, FALSE,			/* seen_fiducial, seen_index */
      0,				/* mark */
      33,				/* index */
      31016188,				/* known_position */
      0, 0, 0,				/* ms_on, error, max_corr */
      0					/* last_latch */
   },
   {					/* Altitude */
      FALSE, FALSE,			/* seen_fiducial, seen_index */
      0,				/* mark */
      1,				/* index */
      3825222,				/* known position */
      0, 0, 0,				/* ms_on, error, max_corr */
      0					/* last_latch */
   },
   {					/* Rotator */
      FALSE, FALSE,			/* seen_fiducial, seen_index */
      0,				/* mark */
      83,				/* index */
      ROT_FID_BIAS + 168185,		/* known position */
      0, 0, 0,				/* ms_on, error, max_corr */
      0					/* last_latch */
   }
};

/*
 * What we know about the fiducials we have crossed
 */
struct FIDUCIALS az_fiducial[N_AZ_FIDUCIALS];
struct FIDUCIALS alt_fiducial[N_ALT_FIDUCIALS];
struct FIDUCIALS rot_fiducial[N_ROT_FIDUCIALS];
      
/*
 * "true" position of all fiducials. These aren't part of struct FIDUCIALS as
 * they are saved to shared memory, and it's easier to do if they are arrays
 */
long az_fiducial_position[N_AZ_FIDUCIALS];
long alt_fiducial_position[N_ALT_FIDUCIALS];
long rot_fiducial_position[N_ROT_FIDUCIALS];
/*
 * Lat fiducial crossed
 */
int fiducialidx[NAXIS] = {-1, -1, -1};

static void update_fiducial_errors(int axis, int corr);

/*****************************************************************************/
/*
 * initialise the fiducials log file
 */
static void
init_fiducial_log(FILE *fd)		/* file descriptor for file */
{
   char version[100];			/* buffer for MCP version */
   
   fprintf(fd, "version       %s  # MCP version\n", mcpVersion(version, 100));
   fprintf(fd, "mjd           %d\n", mjd());
   fprintf(fd, "\n");
   fprintf(fd, "initialTime   %d\n", time(NULL));
   fprintf(fd, "\n");
   
   fprintf(fd, "typedef enum {\n");
   fprintf(fd, "   AZIMUTH = 0,\n");
   fprintf(fd, "   ALTITUDE = 1,\n");
   fprintf(fd, "   INSTRUMENT = 2,\n");
   fprintf(fd, "} AXIS;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   float alt_pos;\n");
   fprintf(fd, "} ALT_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "} AZ_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   int latch;\n");
   fprintf(fd, "} ROT_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "} DEFINE_FIDUCIALS;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true;\n");
   fprintf(fd, "   int pos;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   int correction;\n");
   fprintf(fd, "} SET_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "   int error;\n");
   fprintf(fd, "} SET_FIDUCIAL_ERROR;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "} START_FIDUCIAL;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "   int error;\n");
   fprintf(fd, "} UPDATE_ENCODER;\n");
   fprintf(fd, "\n");
}

/*****************************************************************************/
/*
 * Open the fiducials log file, and write an entry of the specified type
 *
 * Not all arguments are used for all types
 */
void
write_fiducial_log(const char *type,	/* type of entry */
		   int axis,		/* the axis in question */
		   int fididx,		/* the fiducial in question */
		   int true,		/* the "true" position of the fid. */
		   int pos1,		/* position of axis */
		   int pos2,		/* second position, if available */
		   float arg0)		/* something else to write */
{
   float deg;				/* position in degrees */
   FILE *fd;				/* fd for logfile */
   char filename[100];
   struct stat status;			/* information about the directory */

   sprintf(filename, "mcpFiducials-%d.dat", mjd());
   if((fd = fopen_logfile(filename, "a")) == NULL) {
      TRACE(0, "Cannot open %s: %s", filename, strerror(errno));
      return;
   }

   if(fstat(fileno(fd), &status) == ERROR) {
      TRACE(0, "Cannot stat %s: %s", filename, strerror(errno));
   } else {
      if(status.st_size == 0) {
	 init_fiducial_log(fd);
      }
   }
/*
 * Find the angular position of the axis
 */
   switch (axis) {
    case AZIMUTH:    deg = pos1/AZ_TICKS_DEG; break;
    case ALTITUDE:   deg = pos1/ALT_TICKS_DEG; break;
    case INSTRUMENT: deg = (pos1 - ROT_FID_BIAS)/ROT_TICKS_DEG; break;
    case NAXIS: deg = 0; break;		/* i.e. all axes */
    default:
      TRACE(0 ,"write_fiducial_log: illegal axis %d", axis, 0);
      return;
   }
/*
 * Write the entry
 */
   if(strcmp(type, "ALT_FIDUCIAL") == 0) {
      int alt_pos = arg0;
      
      fprintf(fd, "%s %d %4d %9d  %9d %9d %9.3f  %9d\n", type, time(NULL),
	      fididx, true, pos1, pos2, deg, alt_pos);
   } else if(strcmp(type, "AZ_FIDUCIAL") == 0) {
      fprintf(fd, "%s %d %4d %9d  %9d %9d %9.3f\n", type, time(NULL),
	      fididx, true, pos1, pos2, deg);
   } else if(strcmp(type, "ROT_FIDUCIAL") == 0) {
      int rot_latch = arg0;
      
      fprintf(fd, "%s %d %4d %9d  %9d %9d %9.3f  %9d\n", type, time(NULL),
	      fididx, true, pos1, pos2, deg, rot_latch);
   } else if(strcmp(type, "DEFINE_FIDUCIALS") == 0) {
      fprintf(fd, "%s %d %s\n", type, time(NULL), axis_name(axis));
   } else if(strcmp(type, "START_FIDUCIAL") == 0) {
      fprintf(fd, "%s %d\n", type, time(NULL));
   } else if(strcmp(type, "SET_FIDUCIAL") == 0) {
      int correction = arg0;

      fprintf(fd, "%s %d %s  %3d %9d  %9d %9.3f  %d\n", type, time(NULL),
	      axis_name(axis), fididx, true, pos1, deg, correction);
   } else if(strcmp(type, "SET_FIDUCIAL_ERROR") == 0) {
      int error = arg0;

      fprintf(fd, "%s %d %s  %d\n", type, time(NULL), axis_name(axis), error);
   } else if(strcmp(type, "UPDATE_ENCODER") == 0) {
      int offset = arg0;
      
      fprintf(fd, "%s %d %s  %d  %d\n", type, time(NULL),
	      axis_name(axis), pos1, offset);
   } else {
      TRACE(0, "Unknown entry type for fiducial log: %s", type, 0);
   }

   fclose(fd);
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
   
   if(!fiducial[axis].seen_index) {
      TRACE(0, "fiducial for axis %s not crossed", axis_name(axis), 0);
      fprintf(stderr,"fiducial for axis %s not crossed\n", axis_name(axis));
      return(-1);
   }

   correction = fiducial[axis].known_position - fiducial[axis].mark;

   TRACE(3, "Setting %s fiducial; adjusting encoders by %d ticks",
	 axis_name(axis), correction);

   if(tm_adjust_position(axis, correction) < 0) {
      TRACE(0 ,"Failed to adjust position for axis %s", axis_name(axis), 0);
      fprintf(stderr,"Failed to adjust position for axis %s", axis_name(axis));
      return(-1);
   }
   update_fiducial_errors(axis, correction);

   write_fiducial_log("SET_FIDUCIAL", axis, fiducial[axis].index,
		      fiducial[axis].known_position,
		      tmaxis[axis]->actual_position, 0, correction);
  
   fiducial[axis].mark = fiducial[axis].known_position;

   return(0);
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

   TRACE(3, "Setting maximum MS.ON correction for %s to %d",
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
   if(fiducial[axis].error == 0) {	/* nothing to do */
      return;
   }

   if(all_correct || fiducial[axis].ms_on) {
      correction = -fiducial[axis].error;
      			
      if(all_correct) {
	 TRACE(3, "Applying unlimited correction %ld to %s",
	       correction, axis_name(axis));
      } else {
	 if(axis_stat[axis].ms_on_correction_too_large) {
	    if(fiducial[axis].max_correction > 0) {
	       TRACE(0, "MS.ON is disabled for %s; not applying ",
		     axis_name(axis), correction);
	    }
	    return;
	 }
	 
	 if(fiducial[axis].max_correction > 0) {
	    TRACE(3, "Applying correction %ld to %s",
		  correction, axis_name(axis));
	 }
	 
	 if(abs(correction) >= fiducial[axis].max_correction) {
	    if(fiducial[axis].max_correction > 0) {
	       TRACE(0, "    correction for %s %ld is too large",
		     axis_name(axis), correction);
	    }
	    
	    if(semTake(semSLCDC, WAIT_FOREVER) == ERROR) {
	       TRACE(0, "couldn't take semSLCDC semahore.", 0, 0);
	    } else {
	       axis_stat[axis].ms_on_correction_too_large = 1;
	       semGive(semSLCDC);
	    }
	    
	    return;
	 }
      }
      
      set_axis_encoder_error(axis, correction);
      update_fiducial_errors(axis, correction);
      fiducial[axis].error = 0;

      if(axis == INSTRUMENT) {
	 fiducial[axis].last_latch += correction;
      }
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
#if 1
/*
 * Convert that to the SDSS numbering system, putting fiducial 84 at ~ -1deg
 * 6 at ~ 365deg, 123 at ~ -188deg
 *
 * Allow for the fact that the rotator can wrap; valid angles are ~ -185 to ~ +365
 * Fortunately, rot_dir_ccw is true when we are at >~ +90 degrees
 */
   fididx += 45;
   if(fididx > 80 && sdssdc.status.i8.il0.rot_dir_ccw) {
      fididx -= 76;
   } else if(fididx < 48 && !sdssdc.status.i8.il0.rot_dir_ccw) {
      fididx += 76;
   }
#else
/*
 * Convert that to the SDSS numbering system, putting fiducial 84 at ~ -1deg
 * 44 at ~ 185deg, 46 at ~ 176deg
 */
   fididx += 45;
   if(fididx > 45 && latchpos->pos1 > ROT_FID_BIAS) {
      fididx -= 76;
   } else {
      if(fididx < 35 && latchpos->pos1 < ROT_FID_BIAS) {
	 fididx += 76;
      }
   }
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
#endif

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
   int axis;				/* axis MS.ON/OFF refers to */
   int big = 0;				/* was this the big or small interval
					   between marks on the rotator tape?*/
   unsigned char dio316int_bit = 0;	/* bits set by DIO316_interrupt */
   int i;
   int fididx;
   int fididx1;
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
	 dio316int_bit = 0;
      } else if(dio316int_bit != 0) {
/*
 * send message requesting the latches to rearm, reenabling interrupts
 */
	 msg.type = latchReenable_type;
	 msg.u.latchReenable.timeout = 120;	
	 msg.u.latchReenable.dio316int_bit = dio316int_bit;
 
	 ret = msgQSend(msgLatchReenable, (char *)&msg, sizeof(msg),
			NO_WAIT, MSG_PRI_NORMAL);
	 assert(ret == OK);

	 dio316int_bit = 0;
      }
/*
 * The interrupt routine DIO316_interrupt sends a message to msgLatched
 * when any axis sees a fiducial, so wait for it.  Messages also appear
 * on this queue when an MS.ON or MS.OFF needs to be processed
 */
      ret = msgQReceive(msgLatched, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);
/*
 * What sort of message?
 *   alignClamp_type        A request from the outside world to move clamp
 *   alignClampCheck_type   A request from us to check that the clamp moved
 */
      switch (msg.type) {
       case latchCrossed_type:
	 dio316int_bit = msg.u.latchCrossed.dio316int_bit;
	 TRACE(8, "read latchCrossed on msgLatched, delay = %dus 0x%x",
	       timer_read(2) - msg.u.latchCrossed.time, dio316int_bit);
	 break;
       case ms_on_az_type:		/* by symmetry with MS.OFF */
       case ms_on_alt_type:
       case ms_on_inst_type:
	 if(semTake(semLatch, 60) == ERROR) {
	    TRACE(0, "ERR: ms_on cannot take semLatch", 0, 0);
	    continue;
	 }

	 switch (msg.type) {
	  case ms_on_az_type:   axis = AZIMUTH; break;
	  case ms_on_alt_type:  axis = ALTITUDE; break;
	  case ms_on_inst_type: axis = INSTRUMENT; break;
	  default:
	    TRACE(0, "Impossible message type %d; aborting", msg.type, 0);
	    semGive(semLatch);
	    abort(); break;
	 }
	 
	 TRACE(6, "Setting MS.ON for %s", axis_name(axis), "");
	 fiducial[axis].ms_on = 1;
	 
	 maybe_reset_axis_pos(axis, 0, 0, 0);
	 
	 semGive(semLatch);
	 continue;
       case ms_off_az_type:		/* an MS.OFF command; maybe sent by */
       case ms_off_alt_type:		/* timerTask hence the various types */
       case ms_off_inst_type:
	 ret = semTake(semLatch, WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 switch (msg.type) {
	  case ms_off_az_type:   axis = AZIMUTH; break;
	  case ms_off_alt_type:  axis = ALTITUDE; break;
	  case ms_off_inst_type: axis = INSTRUMENT; break;
	  default:
	    TRACE(0, "Impossible message type %d; aborting", msg.type, 0);
	    semGive(semLatch);
	    abort(); break;
	 }
      
	 TRACE(6, "Setting MS.OFF for %s", axis_name(axis), "");
	 fiducial[axis].ms_on = 0;
	 
	 semGive(semLatch);
	 continue;
       default:
	 TRACE(0, "Impossible message type: %d", msg.type, 0);
	 continue;	 
      }
/*
 * Time to read the latches and do the work
 */
      for(i = 15, status = FALSE; status == FALSE && i > 0; i--) {
	 ret = semTake(semMEI, WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 status = (int)latch_status();
	 semGive(semMEI);
	 
	 taskDelay(1);
      }
      
      if(status == FALSE) {		/* we didn't see anything */
	 latchpos[latchidx].axis = -(dio316int_bit & 0xe);
	 if(latchpos[latchidx].axis == 0) {
	    TRACE(0, "Impossible condition: latch interrupt but no bits set",
		  0, 0);
	    latchpos[latchidx].axis = -0x10;
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
	 
	 get_latched_position_corr(0, &latchpos[latchidx].pos1);
	 get_latched_position_corr(1, &latchpos[latchidx].pos2);
	 semGive(semMEI);

	 fididx1 = barcode_serial(3); /* backwards from what you'd think */
	 fididx = barcode_serial(3);	/* read twice...not reliable */
	 if(fididx <= 0 || fididx > 24) {
	    TRACE(0, "Invalid barcode in azimuth: fididx = %d, pos = %d",
		  fididx, latchpos[latchidx].pos1);
	 } else {
#if 1
	    if(sdssdc.status.i7.il0.az_dir_cw) {
	       fididx += 24;
	    }
#else
	    if(sdssdc.status.i7.il0.az_dir_cw) {
	       printf("New: Adding 24 to az fididx ");
	    }
	    if(latchpos[latchidx].pos1 > 0) {
	       fididx += 24;
	       printf(" Old: Adding 24 to az fididx ");
	    }
	    printf("\n");
#endif
	    
	    write_fiducial_log("AZ_FIDUCIAL", AZIMUTH, fididx,
			       az_fiducial_position[fididx],
			       latchpos[latchidx].pos1,
			       latchpos[latchidx].pos2, 0);
	    
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
	       fiducial[AZIMUTH].seen_fiducial = TRUE;
	       
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
		  fiducial[AZIMUTH].seen_index = TRUE;
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
	 
	 get_latched_position_corr(2,&latchpos[latchidx].pos1);
	 get_latched_position_corr(3,&latchpos[latchidx].pos2);
	 semGive(semMEI);

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
	    write_fiducial_log("ALT_FIDUCIAL", ALTITUDE, fididx,
			       alt_fiducial_position[fididx],
			       latchpos[latchidx].pos1,
			       latchpos[latchidx].pos2, 
			       sdssdc.status.i4.alt_position);
	    
	    alt_fiducial[fididx].last = alt_fiducial[fididx].mark;
	    alt_fiducial[fididx].mark = latchpos[latchidx].pos1;
	    alt_fiducial[fididx].err =
	      alt_fiducial[fididx].mark - alt_fiducial[fididx].last;
	    alt_fiducial[fididx].poserr =
	      alt_fiducial[fididx].mark - alt_fiducial_position[fididx];
	    alt_fiducial[fididx].markvalid = TRUE;
	    fiducial[ALTITUDE].seen_fiducial = TRUE;
	    
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
	       fiducial[ALTITUDE].seen_index = TRUE;
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
	 get_latched_position_corr(5,&latchpos[latchidx].pos1);
	 get_latched_position_corr(4,&latchpos[latchidx].pos2);
#else
	 get_latched_position_corr(4, &latchpos[latchidx].pos1);
	 get_latched_position_corr(5, &latchpos[latchidx].pos2);

	 latchpos[latchidx].pos1 += ROT_FID_BIAS; /* make always +ve */
	 latchpos[latchidx].pos2 += ROT_FID_BIAS;
#endif
	 semGive(semMEI);
	 
/*
 * have we already seen a rotator fiducial? If so, we know where we are on
 * the tape as the position of every other fiducial is dithered a little
 */
	 if(rot_latch == 0) {
	    rot_latch = latchpos[latchidx].pos1;
	    semGive(semLatch);
	    continue;
	 }
	 
	 fididx = get_rot_fididx(&latchpos[latchidx], rot_latch, &big);

	 if(fididx < 0) {
	    TRACE(0, "Failed to identify rotator fiducial at %.2f",
		  (latchpos[latchidx].pos1 - ROT_FID_BIAS)/ROT_TICKS_DEG, 0);
	    
	    rot_latch = latchpos[latchidx].pos1;
	    semGive(semLatch);
	    continue;
	 }

	 if(fididx == 0) {
	    ;			/* we just crossed the same fiducial twice */
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
	 
	 if(fididx > 0) {
	    rot_fiducial[fididx].markvalid = TRUE;
	    fiducial[INSTRUMENT].seen_fiducial = TRUE;
	    fiducialidx[INSTRUMENT] = fididx;

	    write_fiducial_log("ROT_FIDUCIAL", INSTRUMENT,
			       (pos_is_mark ? fididx : -fididx),
			       rot_fiducial_position[fididx],
			       latchpos[latchidx].
			       pos1,latchpos[latchidx].pos2, rot_latch);
	 }
      
	 if(fididx > 0) {
	    if(!pos_is_mark) {
	       TRACE(4, "Intermediate rot fiducial %.2f deg",
		     (latchpos[latchidx].pos1 - ROT_FID_BIAS)/ROT_TICKS_DEG,0);
	    } else {
	       static char pos[20];	/* won't appear properly in TRACE log*/

	       sprintf(pos, "%.2f",
		     (rot_fiducial[fididx].mark - ROT_FID_BIAS)/ROT_TICKS_DEG);

	       TRACE(4, "rot fiducial %d (%s) deg", fididx, pos);
      	       TRACE(6, "     pos = %d, rot_latch = %ld",
		     rot_fiducial[fididx].mark, rot_latch);

	       if(rot_fiducial[fididx].last == 0) {
		  TRACE(4, "     err = ???  poserr = %d ticks",
			rot_fiducial[fididx].poserr, 0);
	       } else {
		  TRACE(4, "     err = %d   poserr = %d ticks",
			rot_fiducial[fididx].err, rot_fiducial[fididx].poserr);
	       }
	    }
	    
	    if(fididx == fiducial[INSTRUMENT].index) {
	       fiducial[INSTRUMENT].mark = rot_fiducial[fididx].mark;
	       fiducial[INSTRUMENT].seen_index = TRUE;
	    }

	    if(pos_is_mark) {
	       maybe_reset_axis_pos(INSTRUMENT, 1,
				    rot_fiducial[fididx].poserr,0);
	    }
	 }
      
	 fiducial[INSTRUMENT].last_latch = rot_latch = latchpos[latchidx].pos1;
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

      TRACE(5, "arming latches", 0, 0);
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
**
** Each axis has one primary fiducial and it can be adjusted or changed
** to a new index.
**
** This is not a good idea for the long term. We should be able to use
** the MS.ON and CORRECT commands to accomplish setting the encoders from
** _any_ fiducial. RHL
*/
void
set_primary_fiducials(int axis,
		      int fididx,
		      long pos)
{
   int n_fiducials;			/* == N_axis_FIDUCUIALS */
   long *fiducial_position;		/* == AXIS_fiducial_position */

   switch (axis) {
    case AZIMUTH:
      n_fiducials = N_AZ_FIDUCIALS;
      fiducial_position = az_fiducial_position;
      break;	  
    case ALTITUDE:
      n_fiducials = N_ALT_FIDUCIALS;
      fiducial_position = alt_fiducial_position;
      break;
    case INSTRUMENT:
      n_fiducials = N_ROT_FIDUCIALS;
      fiducial_position = rot_fiducial_position;
      break;
    default:
      printf("set_primary_fiducials: unknown axis %d\n", axis);
      return;
   }

   if(fididx >= 0 && fididx < n_fiducials) {
      fiducial[axis].index = fididx;
      fiducial[axis].seen_fiducial = FALSE;
      fiducial[axis].seen_index = FALSE;
      fiducial[axis].mark = 0;
      fiducial[axis].known_position = pos;
   }
      
   fiducial_position[fiducial[axis].index] = fiducial[axis].known_position;
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
define_fiducials(int axis)
{
   int i;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 if(az_fiducial[i].markvalid) {
	    az_fiducial_position[i] = az_fiducial[i].mark;
	    az_fiducial[i].poserr = 0;
	 }
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 if (alt_fiducial[i].markvalid) {
	    alt_fiducial_position[i] = alt_fiducial[i].mark;
	    alt_fiducial[i].poserr=0;
	 }
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 if(rot_fiducial[i].markvalid) {
	    rot_fiducial_position[i] = rot_fiducial[i].mark;
	    rot_fiducial[i].poserr=0;
	 }
      }
      break;
    default:
      TRACE(0, "Impossible axis in define_fiducials: %d", axis, 0);
      return(-1);
   }

   write_fiducial_log("DEFINE_FIDUCIALS", axis, 0, 0, 0, 0, 0);
      
   return(0);
}

void
define_fiducials_all(void)
{
   int i;
   
   for(i = 0; i < 3; i++) {
      (void)define_fiducials(i);
   }
}

/*****************************************************************************/
/*
 * Update all fiducial "mark" and "last" values for the given axis;
 * usually called after setting a fiducial
 */
static void
update_fiducial_errors(int axis,	/* the axis */
		       int corr)	/* correction to apply */
{
   int i;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 if(az_fiducial[i].markvalid) {
	    az_fiducial[i].mark += corr;
			
	    if(az_fiducial[i].last != 0) {
	       az_fiducial[i].last += corr;
	    }
	 }
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 if(alt_fiducial[i].markvalid) {
	    alt_fiducial[i].mark += corr;
			
	    if(alt_fiducial[i].last != 0) {
	       alt_fiducial[i].last += corr;
	    }
	 }
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 if(rot_fiducial[i].markvalid) {
	    rot_fiducial[i].mark += corr;
			
	    if(rot_fiducial[i].last != 0) {
	       rot_fiducial[i].last += corr;
	    }
	 }
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
ms_define_cmd(char *cmd)		/* NOTUSED */
{
   if(define_fiducials(axis_select) < 0) {
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
      
/*
 * Set the values that we last crossed from the fiducials arrays.  Why would
 * you want to do that? So as to be able to write them to a file with MS.WRITE
 */
char *
ms_get_cmd(char *cmd)			/* NOTUSED */
{
   int axis = axis_select;
   int i;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 az_fiducial[i].mark = az_fiducial_position[i];
	 az_fiducial[i].markvalid = 1;
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 alt_fiducial[i].mark = alt_fiducial_position[i];
	 alt_fiducial[i].markvalid = 1;
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 rot_fiducial[i].mark = rot_fiducial_position[i];
	 rot_fiducial[i].markvalid = (i >= 6 && i <= 123) ? 1 : 0; /* real fiducials */
      }
      break;
    default:
      TRACE(0, "Impossible axis in ms_get: %d", axis, 0);
      return("Impossible axis in ms_get");
   }

   return("");
}
      
/*****************************************************************************/
/*
 * Read/write fiducials for a current axis from/to a file
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

char *
ms_write_cmd(char *cmd)
{
   char filename[200];			/* name of file to write */

   if(sscanf(cmd, "%s", filename) != 1) {
      return("ERR: no filename supplied");
   }

   return(write_fiducials(filename, axis_select));
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
**
** These commands work by sending messages to the tLatch task.  The axis in
** question is encoded in the _type_ of the message. Why? -- because the MS.OFF
** may be sent via the timer task, and there is no convenient way to pass an
** parameter as part of the union.
*/
int
set_ms_on(int axis)			/* the axis in question */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
/*
 * abort any pending MS.OFFs 
 */
   (void)timerSend(ms_off_az_type, tmr_e_abort_ns, 0, 0, 0);
   (void)timerSend(ms_off_alt_type, tmr_e_abort_ns, 0, 0, 0);
   (void)timerSend(ms_off_inst_type, tmr_e_abort_ns, 0, 0, 0);
/*
 * and request the MS.ON
 */
   switch (axis_select) {
    case AZIMUTH:    msg.type = ms_on_az_type; break;
    case ALTITUDE:   msg.type = ms_on_alt_type; break;
    case INSTRUMENT: msg.type = ms_on_inst_type; break;
   }
   
   ret = msgQSend(msgLatched, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   if(ret != OK) {
      TRACE(0, "Failed to send MS.ON message: %d %s", errno, strerror(errno));
      return(-1);
   }

   return(0);
}

int
set_ms_off(int axis,			/* the desired axis */
	   float delay)			/* delay until MS.OFF takes effect, s*/
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* a return code */
/*
 * abort any pending MS.OFFs 
 */
   TRACE(6, "Aborting old MS.OFFs", 0, 0);
   (void)timerSend(ms_off_az_type, tmr_e_abort_ns, 0, 0, 0);
   (void)timerSend(ms_off_alt_type, tmr_e_abort_ns, 0, 0, 0);
   (void)timerSend(ms_off_inst_type, tmr_e_abort_ns, 0, 0, 0);
   taskDelay(1);			/* give the timerTask a chance */
/*
 * and send one that's active _now_, if MS.ON is currently true
 */
   if(semTake(semLatch, 60) != ERROR) {
      int ms_is_on = fiducial[axis].ms_on;
      semGive(semLatch);
      
      if(!ms_is_on) {			/* nothing to do */
	 TRACE(6, "Not sending MS.OFF to %s as it's already off",
	       axis_name(axis), 0);
	 return(0);
      }
   }
/*
 * Time to actually set MS.OFF
 */
   switch (axis_select) {
    case AZIMUTH:    msg.type = ms_off_az_type; break;
    case ALTITUDE:   msg.type = ms_off_alt_type; break;
    case INSTRUMENT: msg.type = ms_off_inst_type; break;
   }
      
   if(delay == 0) {			/* no time specified */
      TRACE(5, "Sending message to msgLatched: type %d", msg.type, 0);
      
      ret = msgQSend(msgLatched, (char *)&msg, sizeof(msg),
		     NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
    } else {				/* wait a while and send MS.OFF */
      TRACE(10, "Sending msg to tTimerTask/msgLatched: type %d delay %d ticks",
	    msg.type, (int)(delay*60));
      if(timerSend(msg.type, tmr_e_add,
		   delay*60, 0, msgLatched) == ERROR) {
	 TRACE(0, "Failed to send ms_off message to timer task: %s (%d)",
	       strerror(errno), errno);
	 return(-1);
      }
   }
      
   return(0);
}

/*****************************************************************************/
/*
 * And the interfaces to those commands
 *
 * set MS.ON
 */
char *
ms_on_cmd(char *cmd)			/* NOTUSED */
{
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(set_ms_on(axis_select) < 0) {
      return("ERR: failed to set MS.ON");
   }

   return "";
}

/*
 * set MS.OFF
 */
char *
ms_off_cmd(char *cmd)			/* NOTUSED */
{
   float delay;				/* delay until MS.OFF takes effect, s*/
   float time;				/* when MS.OFF should take effect */
   
   if(axis_select != AZIMUTH && axis_select != ALTITUDE &&
						   axis_select != INSTRUMENT) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(sscanf(cmd, "%f", &time) == 0) {	/* no time specified */
      delay = 0;
   } else {
      delay = sdss_delta_time(time, sdss_get_time()); /* how long to wait */

      if(delay < 0) {
	 delay = 0;
      }
   }

   if(set_ms_off(axis_select, delay) < 0) {
      return("ERR: failed to set MS.OFF");
   }

   return("");
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

   if(!fiducial[axis_select].seen_fiducial) {
      semGive(semLatch);

      return("ERR: no fiducials have been crossed");
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
 * Restore the fiducials from a file. Note that this doesn't make them
 * current -- use MS.DEFINE (ms_define_cmd()) for that.
 *
 * See also restore_fiducials and write_fiducials
 */
char *
read_fiducials(const char *file,	/* file to read from */
	       int axis)		/* which axis to read */
{
   char axis_str[40];			/* name of axis from file */
   int bias;				/* bias applied to fiducial positions*/
   float error;				/* error in mark */
   int fid;				/* fiducial number from file */
   char fid_str[40];			/* buffer to read string "fiducials" */
   long *fiducials;			/* array to set */
   FILE *fil;				/* F.D. for file */
   char line[200];			/* buffer to read lines of file */
   char *lptr;				/* pointer to line[] */
   float mark;				/* mark from file */
   int npt;				/* number of points used to find mark*/
   int n_fiducials;			/* max number of fiducials */

   switch (axis) {
    case AZIMUTH:
      n_fiducials = N_AZ_FIDUCIALS;
      fiducials = az_fiducial_position;
      bias = 0;
      break;
    case ALTITUDE:
      n_fiducials = N_ALT_FIDUCIALS;
      fiducials = alt_fiducial_position;
      bias = 0;
      break;
    case INSTRUMENT:
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
   TRACE(1, "Reading %s fidcls from %s", axis_name(axis), file);
   
   if((fil = fopen(file, "r")) == NULL) {
      TRACE(0, "Failed to open %s for read", file, 0);
      TRACE(0, "  %d : %s", errno, strerror(errno));
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

	 if(sscanf(lptr, "%s %s", axis_str, fid_str) == 2 &&
	    strcmp(fid_str, "fiducials") == 0) {
	    if(strcmp(axis_str, axis_name(axis)) != 0) {
	       TRACE(0, "Expected fiducials of type %s; saw %s; proceeding",
		     axis_name(axis), axis_str);
	    }
	 }
							  
	 continue;
      }

      if(sscanf(lptr, "%d %f +- %f %d", &fid, &mark, &error, &npt) == 4) {
	 if(fid < 0 || fid >= n_fiducials) {
	    TRACE(0, "Invalid fiducial %d in file %s", fid, file);
	    continue;
	 }

	 if(error < 0) {		/* invalid position */
	    fiducials[fid] = mark + bias;
	 }
      } else {
	 TRACE(0, "Corrupt line: %s", line, 0);
	 continue;
      }
   }
			
   fclose(fil);

   return("");
}

/*****************************************************************************/
/*
 * Write the current fiducials to a file.
 *
 * See also read_fiducials and save_fiducials
 */
char *
write_fiducials(const char *file,	/* file to write to */
	       int axis)		/* which axis to write */
{
   float err;				/* error in mark */
   struct FIDUCIALS *fiducials;		/* the desired information */
   FILE *fil;				/* F.D. for file */
   int i;
   int npt = 1;				/* number of points used to find mark*/
   int n_fiducials;			/* max number of fiducials */

   switch (axis) {
    case AZIMUTH:
      n_fiducials = N_AZ_FIDUCIALS;
      fiducials = az_fiducial;
      break;
    case ALTITUDE:
      n_fiducials = N_ALT_FIDUCIALS;
      fiducials = alt_fiducial;
      break;
    case INSTRUMENT:
      n_fiducials = N_ROT_FIDUCIALS;
      fiducials = rot_fiducial;
      break;
    default:
      fprintf(stderr,"write_fiducials: illegal axis %d\n", axis);
      return("ERR: illegal axis");
   }
/*
 * Open file and write header
 */
   TRACE(1, "Writing %s fidls to %s", axis_name(axis), file);
   
   if((fil = fopen(file, "w")) == NULL) {
      TRACE(0, "Failed to open %s for write", file, 0);
      TRACE(0, "  %d : %s", errno, strerror(errno));
      return("ERR: failed to open file\n");
   }

   fprintf(fil, "#\n");
   fprintf(fil, "# %s fiducials\n", axis_name(axis));
   fprintf(fil, "# ?????\n");
   fprintf(fil, "#\n");
   fprintf(fil, "# Fiducial Mark +- error  npoint\n");
   fprintf(fil, "#\n");
/*
 * write data
 */
   for(i = 0;i < n_fiducials; i++) {
      if(fiducials[i].markvalid) {
	 err = 0;
	 npt = 1;
      } else {
	 err = -9999;
	 npt = 0;
      }
	 
      fprintf(fil, "%3d  %10ld +- %7f %d\n", i, fiducials[i].mark, err, npt);
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
	if(fiducial[axis].index == i) {
	   printf("*");
	   if(fiducial[axis].seen_index) printf ("!");
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
	   if(fiducial[axis].seen_index) printf ("!");
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
	   if(fiducial[axis].seen_index) printf ("!");
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
   if (fiducial[i].seen_index) {
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
   int i;

   write_fiducial_log("START_FIDUCIAL", NAXIS, 0, 0, 0, 0, 0);
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
		(FUNCPTR)DIO316ClearISR_delay,
		0,0,0,0,0,0,0,0,0,0);
   }
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
 * Restore the fiducial positions from shared memory
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
/*
 * Declare commands to the command interpreter
 */
   define_cmd("CORRECT",      correct_cmd,         0, 1, 1);
   define_cmd("MS.MAP.DUMP",  ms_map_dump_cmd,     0, 0, 1);
   define_cmd("MS.MAP.LOAD",  ms_map_load_cmd,     0, 1, 1);
   define_cmd("MS.OFF",       ms_off_cmd,         -1, 1, 1);
   define_cmd("MS.ON",        ms_on_cmd,           0, 1, 1);
   define_cmd("MS.MAX",       ms_max_cmd,          1, 1, 1);
   define_cmd("MS.READ",      ms_read_cmd,         1, 1, 1);
   define_cmd("MS.SAVE",      ms_save_cmd,         0, 1, 1);
   define_cmd("MS.SET",       ms_set_axis_pos_cmd, 0, 1, 1);
   define_cmd("MS.GET",       ms_get_cmd,          0, 1, 1);
   define_cmd("MS.DEFINE",    ms_define_cmd,       0, 1, 1);
   define_cmd("MS.WRITE",     ms_write_cmd,        1, 1, 1);
   define_cmd("SET.FIDUCIAL", ms_set_axis_pos_cmd, 0, 1, 1);
}
