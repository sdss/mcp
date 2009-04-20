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
#include "tod_prototypes.h"
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
#include "as2.h"

char fiducialVersion[3][FIDVERLEN] = {"undefined", "undefined", "undefined"};	/* Fiducial table CVS versions. */

/*
 * Data from reading a latch
 */
struct LATCH_POS {
   int axis;				/* which axis */
   double pos[3];			/* encoder positions (0 is unused) */
};

void DIO316ClearISR_delay(void);
static void maybe_reset_axis_pos(int axis, long *poserr, int all_correct);

SEM_ID semLatch = NULL;			/* semaphore for updating fiducials */
MSG_Q_ID msgLatchReenable = NULL;	/* reset the DIO316 interrupt */
MSG_Q_ID msgLatched = NULL;		/* signal that we're latched */

static int latchidx = 0;

#define MAX_LATCHED	2000
struct LATCH_POS latchpos[MAX_LATCHED];

/*
 * structure for the primary set of fiducials - one per axis
 */
struct FIDUCIARY fiducial[NAXIS];

/*
 * What we know about the fiducials we have crossed
 */
struct FIDUCIALS az_fiducial[N_AZ_FIDUCIALS];
struct FIDUCIALS alt_fiducial[N_ALT_FIDUCIALS];
struct FIDUCIALS rot_fiducial[N_ROT_FIDUCIALS];
/*
 * Last fiducial crossed
 */
int fiducialidx[NAXIS] = {-1, -1, -1};

static void update_fiducial_errors(int axis, long *corr);

/*****************************************************************************/
/*
 * initialise the fiducials log file
 */
static void
init_fiducial_log(FILE *fd)		/* file descriptor for file */
{
   char version[100];			/* buffer for MCP version */
   
   fprintf(fd, "version       %s  # MCP version\n", mcpVersion(version, 100));
   fprintf(fd, "mjd           %d\n", get_mjd());
   fprintf(fd, "\n");
   fprintf(fd, "initialTime   %ld\n", time(NULL));
   fprintf(fd, "\n");
   
   fprintf(fd, "typedef enum {\n");
   fprintf(fd, "   azimuth = 0,\n");
   fprintf(fd, "   altitude = 1,\n");
   fprintf(fd, "   rotator = 2,\n");
   fprintf(fd, "} AXIS;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true1;\n");
   fprintf(fd, "   int true2;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   float alt_pos;\n");
   fprintf(fd, "   float velocity;\n");
   fprintf(fd, "   int encoder_error1;\n");
   fprintf(fd, "   int encoder_error2;\n");
   fprintf(fd, "} ALT_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true1;\n");
   fprintf(fd, "   int true2;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   float velocity;\n");
   fprintf(fd, "   int encoder_error1;\n");
   fprintf(fd, "   int encoder_error2;\n");
   fprintf(fd, "} AZ_FIDUCIAL;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int fididx;\n");
   fprintf(fd, "   int true1;\n");
   fprintf(fd, "   int true2;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int pos2;\n");
   fprintf(fd, "   float deg;\n");
   fprintf(fd, "   int latch;\n");
   fprintf(fd, "   float velocity;\n");
   fprintf(fd, "   int encoder_error1;\n");
   fprintf(fd, "   int encoder_error2;\n");
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
   fprintf(fd, "   int encoder;\n");
   fprintf(fd, "   int error;\n");
   fprintf(fd, "} SET_FIDUCIAL_ERROR;\n");
   fprintf(fd, "\n");
      
   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   char mcpVersion[100];\n");
   fprintf(fd, "} START_FIDUCIAL;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "   int encoder;\n");
   fprintf(fd, "   int pos1;\n");
   fprintf(fd, "   int error;\n");
   fprintf(fd, "   float velocity;\n");
   fprintf(fd, "} UPDATE_ENCODER;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "   int correction;\n");
   fprintf(fd, "} DISABLE_MS_CORRECTION;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "} MS_ON;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   AXIS axis;\n");
   fprintf(fd, "} MS_OFF;\n");
   fprintf(fd, "\n");

   fprintf(fd, "typedef struct {\n");
   fprintf(fd, "   int time;\n");
   fprintf(fd, "   int dio316;\n");
   fprintf(fd, "} BAD_DIO316;\n");
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
		   int true1, int true2, /* the true position of the fiducial;
					    encoders 1 and 2 */
		   int pos1,		/* position of axis */
		   int pos2,		/* second position, if available */
		   double arg0,		/* something else to write */
		   double arg1,		/* yet something else to write */
		   long iarg0,		/* an extra int to write */
		   long iarg1)		/* yet another int to write */
{
   int uid = 0, cid = 0;
   const char *aname = NULL;		/* upper case name of axis */
   float deg;				/* position in degrees */
   FILE *fd;				/* fd for logfile */
   char filename[100];
   struct stat status;			/* information about the directory */

   sprintf(filename, "mcpFiducials-%d.dat", get_mjd());
   if((fd = fopen_logfile(filename, "a")) == NULL) {
      NTRACE_2(0, uid, cid, "Cannot open %s: %s", filename, strerror(errno));
      return;
   }

   if(fstat(fileno(fd), &status) == ERROR) {
      NTRACE_2(0, uid, cid, "Cannot stat %s: %s", filename, strerror(errno));
   } else {
      if(status.st_size == 0) {
	 init_fiducial_log(fd);
      }
   }
/*
 * Find the angular position of the axis from the encoder
 */
   switch (axis) {
    case AZIMUTH:
      aname = "AZIMUTH";  deg = pos1/ticks_per_degree[axis];
      break;
    case ALTITUDE:
      aname = "ALTITUDE"; deg = pos1/ticks_per_degree[axis];
      break;
    case INSTRUMENT:
      aname = "ROTATOR";  deg = (pos1 - ROT_FID_BIAS)/ticks_per_degree[axis];
      break;
    case NAXIS:				/* i.e. all axes */
      aname = "ALL"; deg = 0; break;
    default:
      NTRACE_1(0, uid, cid ,"write_fiducial_log: illegal axis %d", axis);
      return;
   }
/*
 * Write the entry
 */
   if(strcmp(type, "ALT_FIDUCIAL") == 0) {
      float vel = arg0;
      int alt_pos = iarg0;
      deg = convert_clinometer(alt_pos);

      fprintf(fd, "%s %ld %4d  %9d %9d  %9d %9d %9.3f  %9d %9.0f  %d %d\n",
	      type, time(NULL), fididx, true1, true2,
	      pos1, pos2, deg, alt_pos, vel,
	      get_axis_encoder_error(axis,1), get_axis_encoder_error(axis,2));
   } else if(strcmp(type, "AZ_FIDUCIAL") == 0) {
      float vel = arg0;
      fprintf(fd, "%s %ld %4d  %9d %9d  %9d %9d %9.3f %9.0f  %d %d\n",
	      type, time(NULL), fididx, true1, true2,
	      pos1, pos2, deg, vel,
	      get_axis_encoder_error(axis,1), get_axis_encoder_error(axis,2));
   } else if(strcmp(type, "ROT_FIDUCIAL") == 0) {
      float vel = arg0;
      int rot_latch = iarg0;
      
      fprintf(fd, "%s %ld %4d  %9d %9d  %9d %9d %9.3f  %9d %9.0f  %d %d\n",
	      type,time(NULL), fididx, true1, true2,
	      pos1, pos2, deg, rot_latch, vel,
	      get_axis_encoder_error(axis,1), get_axis_encoder_error(axis,2));
   } else if(strcmp(type, "DEFINE_FIDUCIALS") == 0) {
      fprintf(fd, "%s %ld %s\n", type, time(NULL), aname);
   } else if(strcmp(type, "START_FIDUCIAL") == 0) {
      char buff[100];
      fprintf(fd, "%s %ld \"%s\"\n", type, time(NULL), mcpVersion(buff, 100));
   } else if(strcmp(type, "SET_FIDUCIAL_ERROR") == 0) {
      int error = iarg0;
      int encoder = iarg1;

      fprintf(fd, "%s %ld %s %d  %d\n", type, time(NULL),
	      aname, encoder, error);
   } else if(strcmp(type, "UPDATE_ENCODER") == 0) {
      float vel = arg0;
      int offset = iarg0;
      int encoder = iarg1;
      
      fprintf(fd, "%s %ld %s %d  %d  %d %9.0f\n", type, time(NULL),
	      aname, encoder, pos1, offset, vel);
   } else if(strcmp(type, "DISABLE_MS_CORRECTION") == 0) {
      int correction = iarg0;
      
      fprintf(fd, "%s %ld %s  %d\n", type, time(NULL),
	      aname, correction);
   } else if(strcmp(type, "MS_ON") == 0) {
      fprintf(fd, "%s %ld %s\n", type, time(NULL), aname);
   } else if(strcmp(type, "MS_OFF") == 0) {
      fprintf(fd, "%s %ld %s\n", type, time(NULL), aname);
   } else if(strcmp(type, "BAD_DIO316") == 0) {
      int dio316 = iarg0;
      fprintf(fd, "%s %ld %d\n", type, time(NULL), dio316);
   } else {
      NTRACE_1(0, uid, cid, "Unknown entry type for fiducial log: %s", type);
   }

   fclose(fd);
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

void
broadcast_fiducial_status(int uid, unsigned long cid)
{
   char buff[KEY_VALUE_LEN];

   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "azFiducialVersion", fiducialVersion[AZIMUTH]);
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "altFiducialVersion", fiducialVersion[ALTITUDE]);
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "rotFiducialVersion", fiducialVersion[INSTRUMENT]);
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "goodFiducialVersions", 
		   (strcmp(fiducialVersion[0], fiducialVersion[1]) == 0 &&
		    strcmp(fiducialVersion[0], fiducialVersion[2]) == 0));

   sprintf(buff, "%ld, %ld, %ld",
	   fiducial[AZIMUTH].max_correction,
	   fiducial[ALTITUDE].max_correction,
	   fiducial[INSTRUMENT].max_correction);
   sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "msOnMaxCorrection", buff);

   sprintf(buff, "%ld, %ld, %ld",
	   fiducial[AZIMUTH].min_encoder_mismatch,
	   fiducial[ALTITUDE].min_encoder_mismatch,
	   fiducial[INSTRUMENT].min_encoder_mismatch);
   sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "minEncoderMismatch", buff);
   
   /* PLC version, per the PLC */
   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "plcVersion", sdssdc.status.b3.w1.version_id);
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "mcpVersion", mcpVersion(NULL, -1));
}


/*****************************************************************************/
/*
 * Set a fiducial
 */
int
mcp_set_fiducial(int axis)
{
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"mcp_set_fiducial: illegal axis %d\n", axis);

      return(-1);
   }

   if(semTake(semLatch, 60) == ERROR) {
      return(-1);
   }

   if(!fiducial[axis].seen_fiducial) {
      semGive(semLatch);

      return(-1);
   }

   maybe_reset_axis_pos(axis, NULL, 1);

   semGive(semLatch);
   
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
   int old;				/* old value */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"mcp_set_fiducial: illegal axis %d\n", axis);

      return(-1);
   }
			
   old = fiducial[axis].max_correction;
   fiducial[axis].max_correction = max_correction;

   OTRACE(3, "Setting maximum MS.ON correction for %s to %d",
	 axis_name(axis), max_correction);

   return(old);
}

/*****************************************************************************/
/*
 * Set the minimum disagreement between encoders on a given axis that
 * should generate an error message
 */
int
set_min_encoder_mismatch_error(int axis, /* the axis in question */
			       int min_error) /* min error to warn about */
{
   int old;				/* old value */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"mcp_set_fiducial: illegal axis %d\n", axis);

      return(-1);
   }

   old = fiducial[axis].min_encoder_mismatch;
			
   fiducial[axis].min_encoder_mismatch = min_error;

   OTRACE(3, "Setting minimum encoder mismatch error for %s to %d",
	 axis_name(axis), min_error);

   return(old);
}

/*****************************************************************************/
/*
 * Given an axis and an error in that axis based on the last fiducial
 * crossing consider updating the position of that axis
 */
int allow_disable_ms_on = 0;		/* available from vxWorks;
					   if true disable MS.ON if
					   correction's too large. RHL XXX */

static void
maybe_reset_axis_pos(int axis,		/* the axis */
		     long *poserr,	/* poserr for the axes, or NULL */
		     int all_correct)	/* allow any correction? */
{
   int uid = 0, cid = 0;
   long correction[3];			/* corrections to apply ([0] unused) */
/*
 * Remember that poserr?  We could do something cleverer here, based
 * on maintaining a history of errors.  But let's not.
 */
   if(poserr != NULL) {
      fiducial[axis].error[1] = poserr[1];
      fiducial[axis].error[2] = poserr[2];
   }
/*
 * Actually apply the adjustment
 */
   if(fiducial[axis].error[1] == 0 && fiducial[axis].error[2] == 0) {
      return;				/* nothing to do */
   }

   if(all_correct || fiducial[axis].ms_on) {
      correction[1] = -fiducial[axis].error[1];
      correction[2] = -fiducial[axis].error[2];
      			
      if(all_correct) {
	 NTRACE_2(3, uid, cid, "Applying unlimited correction %ld to %s",
		  correction[1], axis_name(axis));
      } else {
	 if(axis_stat[axis][0].ms_on_correction_too_large) {
	    if(fiducial[axis].max_correction > 0) {
	       NTRACE_2(0, uid, cid, "MS.ON is disabled for %s; not applying %ld",
			axis_name(axis), correction[1]);
	    }
	    return;
	 }
	 
	 if(fiducial[axis].max_correction > 0) {
	    NTRACE_2(3, uid, cid, "Applying correction %ld to %s",
		  correction[1], axis_name(axis));
	 }
	 
	 if(abs(correction[1]) >= fiducial[axis].max_correction) {
	    if(fiducial[axis].max_correction > 0) {
	       NTRACE_2(0, uid, cid, "    correction for %s %ld is too large",
		     axis_name(axis), correction[1]);
	    }
	    
	    if(allow_disable_ms_on) {
	       if(semTake(semMEIUPD, WAIT_FOREVER) == ERROR) {
		  NTRACE(0, uid, cid, "couldn't take semMEIUPD semaphore.");
	       } else {			/* set it in [0] as [1]'s not cleared*/
		  axis_stat[axis][0].ms_on_correction_too_large = 1;
		  semGive(semMEIUPD);

		  write_fiducial_log("DISABLE_MS_CORRECTION", axis,
				     0, 0, 0, 0, 0,
				     0.0, 0.0, correction[1], 0);
	       }
	    } else {
	       if(fiducial[axis].max_correction > 0) {
		  NTRACE_2(3, uid, cid, "Not disabling MS.ON; error %ld (max allowed: %ld)",
			   correction[1], fiducial[axis].max_correction);
	       }
	    }
	    
	    return;
	 }
      }
      
      set_axis_encoder_error(axis, 1, correction[1], 1);
      set_axis_encoder_error(axis, 2, correction[2], 1);
      update_fiducial_errors(axis, correction);
      fiducial[axis].error[1] = fiducial[axis].error[2] = 0;

      if(axis == INSTRUMENT) {
	 fiducial[axis].last_latch += correction[1];
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
get_rot_fididx(const struct LATCH_POS *latch_pos, /* the most recent pos. */
	       int last_latch_pos,	/* previous latch pos */
	       int *big)		/* is this a big interval? */
{
   int fididx;				/* the desired index */
   
   if(abs((long)latch_pos->pos[1] - last_latch_pos) < 250000) {
      return(0);			/* we just crossed the same fiducial
					   twice */
   }
/*
 * Find the index from the Heidenhain
 */
   fididx = abs(iround((latch_pos->pos[1] - last_latch_pos)/800.));
   fididx -= 500;
   
   if(fididx > 0) {
      *big = 1;
   } else {
      *big = 0;
      fididx = -fididx;
   }
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

   if(fididx <= 0 || fididx >= N_ROT_FIDUCIALS) {
      int uid = 0, cid = 0;
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "rotBadFiducialDelta", latch_pos->pos[1] - last_latch_pos);
      return(-1);
   }

   return(fididx);
}

/*****************************************************************************/
/*
 * Remap the fiducial indices to values deemed easier for humans to comprehend;
 * PR 1525
 */
static int
remap_fiducials(int axis,		/* the axis in question */
		int rawidx)		/* raw fiducial index */
{
   int fididx;				/* remapped index */
   
   switch (axis) {
    case ALTITUDE:
      fididx = rawidx;
      break;
    case AZIMUTH:
      if(rawidx >= 39) {
	 fididx = rawidx - 38;
      } else if(rawidx >= 25) {
	 fididx = rawidx - 24;
	 fididx += 10;
      } else if(rawidx >= 10) {
	 fididx = rawidx - 9;
	 fididx += 19;
      } else {
	 fididx = rawidx;
	 fididx += 34;
      }
      break;
    case INSTRUMENT:
      fididx = rawidx - 5;
      break;
   }

   return(fididx);
}

/*****************************************************************************/
/*
 * Control the use of the az barcode reader
 */
static int use_az_barcode = 0;			/* Use the azimuth barcode reader? */

char *
az_barcode_cmd(int uid, unsigned long cid, char *cmd)
{
   sprintf(ublock->buff, "%d", use_az_barcode);
			
#if USE_BARCODE
   if(sscanf(cmd, "%d", &use_az_barcode) != 1) {
      return("ERR: AZ.BARCODE");
   }
#else
      return("ERR: AZ.BARCODE is disabled");
#endif
			
   return(ublock->buff);
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
**
**=========================================================================
*/
void
tLatch(const char *name)
{
   int uid = 0; unsigned long cid = 0;
   int axis;				/* axis MS.ON/OFF refers to */
   int big = 0;				/* was this the big or small interval
					   between marks on the rotator tape?*/
   unsigned char dio316int_bit = 0;	/* bits set by DIO316_interrupt */
   int i;
   int fididx;
#if USE_BARCODE
   int fididx1;
#endif
   MCP_MSG msg;				/* message to pass around */
   int pos_is_mark;			/* is this a "real" mark on
					   the rotator, not one of the dithered
					   ones? */
   int relatch = 1;			/* re-enable latches? */
   int ret;				/* a return code */
   long rot_latch = 0;			/* position of last rotary latch seen*/
   int status;
   double vel;				/* an axis' velocity */
   
   for(latchidx = 0;; latchidx = (latchidx + 1)%MAX_LATCHED) {
      if(relatch) {
/*
 * send message requesting the latches to rearm and to reenable interrupts
 */
	 msg.type = latchReenable_type;
	 msg.u.latchReenable.timeout = 1;	
	 msg.u.latchReenable.dio316int_bit = dio316int_bit;
	 msg.uid = uid;
	 msg.cid = cid;

	 ret = msgQSend(msgLatchReenable, (char *)&msg, sizeof(msg),
			NO_WAIT, MSG_PRI_NORMAL);
	 assert(ret == OK);
      }
/*
 * The interrupt routine DIO316_interrupt sends a message to msgLatched
 * when any axis sees a fiducial, so wait for it.  Messages also appear
 * on this queue when an MS.ON or MS.OFF needs to be processed
 */
      ret = msgQReceive(msgLatched, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      if (msg.type > 0) {
	 uid = msg.uid;
	 cid = msg.cid;
      } else {				/* came via a timerSendArg */
	 msg.type = -msg.type;
	 get_uid_cid_from_tmr_msg(&msg, &uid, &cid);
      }
/*
 * What sort of message?
 *   latchCrossed_type:          We crossed a fiducial
 *   ms_on_{az,alt,inst}_type:   We received ms.on for the specified axis
 *   ms_off_{az,alt,inst}_type:  We received ms.off for the specified axis
 */
      if (msg.type < 0) {
	 NTRACE_1(0, uid, cid, "RHL msg.type = %d\n", msg.type);
      }
      
      relatch = 0;			/* re-enable latches? */
      switch (msg.type) {
       case latchCrossed_type:
	 dio316int_bit = msg.u.latchCrossed.dio316int_bit;
	 relatch = 1;
	 NTRACE_2(8, uid, cid, "read latchCrossed on msgLatched, delay = %ldus 0x%x",
		  timer_read(2) - msg.u.latchCrossed.time, dio316int_bit);
	 break;
       case ms_on_az_type:		/* by symmetry with MS.OFF */
       case ms_on_alt_type:
       case ms_on_inst_type:
	 if(semTake(semLatch, 60) == ERROR) {
	    NTRACE(0, uid, cid, "ERR: ms_on cannot take semLatch");
	    continue;
	 }

	 switch (msg.type) {
	  case ms_on_az_type:   axis = AZIMUTH; break;
	  case ms_on_alt_type:  axis = ALTITUDE; break;
	  case ms_on_inst_type: axis = INSTRUMENT; break;
	  default:
	    NTRACE_1(0, uid, cid, "Impossible message type %d; aborting", msg.type);
	    semGive(semLatch);
	    abort(); break;
	 }
	 
	 OTRACE(3, "%s MS.ON", axis_name(axis), "");
	 fiducial[axis].ms_on = 1;

	 {
	    char key[20];
	    sprintf(key, "%sMsOn", axis_abbrev(axis));
	    sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, key, fiducial[axis].ms_on);
	 }
	 
	 maybe_reset_axis_pos(axis, NULL, 0);
	 
	 semGive(semLatch);

	 write_fiducial_log("MS_ON", axis, 0, 0, 0, 0, 0,  0.0, 0.0, 0, 0);
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
	    NTRACE_1(0, uid, cid, "Impossible message type %d; aborting", msg.type);
	    semGive(semLatch);
	    abort(); break;
	 }
      
	 OTRACE(3, "%s MS.OFF", axis_name(axis), "");
	 fiducial[axis].ms_on = 0;

	 {
	    char key[20];
	    sprintf(key, "%sMsOff", axis_abbrev(axis));
	    sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, key, fiducial[axis].ms_on);
	 }
	 
	 semGive(semLatch);
	 
	 write_fiducial_log("MS_OFF", axis, 0, 0, 0, 0, 0,  0.0, 0.0, 0, 0);
	 continue;
       default:
	 NTRACE_1(0, uid, cid, "Impossible message type on msgLatched: %d", msg.type);
	 continue;	 
      }
/*
 * Time to read the latches and do the work
 *
 * Start by checking that the MEIs are latched
 */
      assert(relatch == 1);		/* == we saw a latchCrossed_type msg */

      if(dio316int_bit == AZIMUTH_INT) {
	 axis = AZIMUTH;
      } else if(dio316int_bit == ALTITUDE_INT) {
	 axis = ALTITUDE;
      } else if(dio316int_bit == INSTRUMENT_INT) {
	 axis = INSTRUMENT;
      } else {
	 NTRACE_1(1, uid, cid, "More than one axis reports a fiducial interrupt: 0x%x", dio316int_bit);

	 write_fiducial_log("BAD_DIO316", axis,
			    0, 0, 0, 0, 0,
			    0.0, 0.0, dio316int_bit, 0);

	 continue;
      }

      for(i = 0; i < 15; i++) {
	 ret = semTake(semMEI, WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 status = (int)latch_status();
	 semGive(semMEI);

	 if(status == TRUE) {
	    break;
	 }

	 taskDelay(1);
      }
      if(status == TRUE) {		/* the MEI latched */
	 int nlatch_allowed = 1;	/* max. number of tries allowed
					   (unlimited if <= 0)*/
	 if(i > 1) {			/* but too late */
	    get_velocity(2*axis, &vel);
	    NTRACE_2(3, uid, cid, "%s: took %d attempts to read MEI latch status", axis_name(axis), i);

	    nlatch_allowed = -1;	/* allow any number */
	    if(nlatch_allowed > 0 && i > nlatch_allowed) {
	       NTRACE_2(1, uid, cid, "Ignoring delayed latch on %s (%d tries)",
		     axis_name(axis), i);
	       continue;		/* We can't trust delayed latches */
	    }
	 }
      } else {				/* The MEI didn't latch */
	 latchpos[latchidx].axis = -(dio316int_bit & 0xe);
	 if(latchpos[latchidx].axis == 0) {
	    NTRACE(0, uid, cid, "Impossible condition: latch interrupt but no bits set");
	    latchpos[latchidx].axis = -0x10;
	 }

	 NTRACE_2(1, uid, cid, "failed to read latch position: status = %d axis %s",
	       status, axis_name(axis));

	 continue;
      }
/*
 * OK, we latched exactly one fiducial, so read the positions of
 * the appropriate axis and take proper actions
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
	 
	 get_latched_position_corr(2*AZIMUTH,     &latchpos[latchidx].pos[1]);
	 get_latched_position_corr(2*AZIMUTH + 1, &latchpos[latchidx].pos[2]);
	 get_velocity(2*AZIMUTH, &vel);
	 semGive(semMEI);


#if USE_BARCODE
	 fididx1 = barcode_serial(3); /* backwards from what you'd think */
	 fididx = barcode_serial(3);	/* read twice...not reliable */
#endif
	 if(use_az_barcode && (fididx <= 0 || fididx > 24)) {
	    NTRACE_2(0, uid, cid, "Invalid barcode in azimuth: fididx = %d, pos = %g",
		     fididx, latchpos[latchidx].pos[1]);
	 } else {
	    if(use_az_barcode) {
	       if(sdssdc.status.i7.il0.az_dir_cw) {
		  fididx += 24;
	       }
	       fididx = remap_fiducials(AZIMUTH, fididx);
	    } else {
	       int dist = 0;		/* discrepancy from fiducial table */
	       int best_dist = -1;	/* Smallest discrepancy from table */
	       int pos = latchpos[latchidx].pos[1]; /* latched position */
	       
	       fididx = -1;
	       for(i = 0; i < N_AZ_FIDUCIALS; i++) {
		  dist = abs(pos - az_fiducial[i].fiducial[1]);
		  if(best_dist < 0 || dist < best_dist) {
		     best_dist = dist;
		     fididx = i;
		  }
	       }
	       NTRACE_2(5, uid, cid, "Identified az fiducial %d, delta = %d",
			fididx, best_dist);
	    }
	    
	    write_fiducial_log("AZ_FIDUCIAL", AZIMUTH, fididx,
			       az_fiducial[fididx].fiducial[1],
			       az_fiducial[fididx].fiducial[2],
			       latchpos[latchidx].pos[1],
			       latchpos[latchidx].pos[2],
			       vel, 0.0, 0, 0);
	    
	    if(fididx < 0 || fididx >= N_AZ_FIDUCIALS) {
	       NTRACE_2(0, uid, cid, "Invalid azimuth fiducial %d, pos = %g",
			fididx, latchpos[latchidx].pos[1]);
	    } else {
	       for(i = 1; i <= 2; i++) {
		  az_fiducial[fididx].last[i] = az_fiducial[fididx].mark[i];
		  az_fiducial[fididx].mark[i] = latchpos[latchidx].pos[i];
		  
		  az_fiducial[fididx].err[i] =
		    latchpos[latchidx].pos[i] - az_fiducial[fididx].last[i];
		  az_fiducial[fididx].poserr[i] =
		    latchpos[latchidx].pos[i] - az_fiducial[fididx].fiducial[i];
	       }
	       if(!az_fiducial[fididx].disabled) {
		  az_fiducial[fididx].markvalid = TRUE;
	       }
	       fiducial[AZIMUTH].seen_fiducial = TRUE;
	       
	       NTRACE_1(4, uid, cid, "az fiducial %.2f deg",
			az_fiducial[fididx].mark[1]/ticks_per_degree[AZIMUTH]);
	       if(az_fiducial[fididx].last[1] == 0) {
		  NTRACE_1(4, uid, cid, "     err = ???  poserr = %ld ticks",
			   az_fiducial[fididx].poserr[1]);
	       } else {
		  NTRACE_2(4, uid, cid, "     err = %ld   poserr = %ld ticks",
			   az_fiducial[fididx].err[1],
			   az_fiducial[fididx].poserr[1]);
	       }
	       
	       fiducialidx[AZIMUTH] = fididx;
	       if(az_fiducial[fididx].markvalid) {
		  maybe_reset_axis_pos(AZIMUTH, az_fiducial[fididx].poserr, 0);
	       }
	    }
	 }
      } else if(dio316int_bit & ALTITUDE_INT) {
	 latchpos[latchidx].axis = ALTITUDE;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);
	 
	 get_latched_position_corr(2*ALTITUDE,     &latchpos[latchidx].pos[1]);
	 get_latched_position_corr(2*ALTITUDE + 1, &latchpos[latchidx].pos[2]);
	 get_velocity(2*ALTITUDE, &vel);
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
	 fididx = remap_fiducials(ALTITUDE, fididx);
			
	 if(fididx < 0 || fididx >= N_ALT_FIDUCIALS) {
	    NTRACE_2(0, uid, cid, "Invalid altitude fiducial %d, clino = %d",
		     fididx, (int)(read_clinometer() + 0.5));
	 } else {
	    write_fiducial_log("ALT_FIDUCIAL", ALTITUDE, fididx,
			       alt_fiducial[fididx].fiducial[1],
			       alt_fiducial[fididx].fiducial[2],
			       latchpos[latchidx].pos[1],
			       latchpos[latchidx].pos[2], 
			       vel, 0.0, sdssdc.status.i4.alt_position, 0);
	    for(i = 1; i <= 2; i++) {
	       alt_fiducial[fididx].last[i] = alt_fiducial[fididx].mark[i];
	       alt_fiducial[fididx].mark[i] = latchpos[latchidx].pos[i];
	       alt_fiducial[fididx].err[i] =
		 latchpos[latchidx].pos[i] - alt_fiducial[fididx].last[i];
	       alt_fiducial[fididx].poserr[i] =
		 latchpos[latchidx].pos[i] - alt_fiducial[fididx].fiducial[i];
	    }
	    if(!alt_fiducial[fididx].disabled) {
	       alt_fiducial[fididx].markvalid = TRUE;
	    }
	    fiducial[ALTITUDE].seen_fiducial = TRUE;
	    
	    NTRACE_1(4, uid, cid, "alt fiducial %.2f deg",
		     alt_fiducial[fididx].mark[1]/ticks_per_degree[ALTITUDE]);
	    if(alt_fiducial[fididx].last[1] == 0) {
	       NTRACE_1(4, uid, cid, "     err = ???  poserr = %ld ticks",
			alt_fiducial[fididx].poserr[1]);
	    } else {
	       NTRACE_2(4, uid, cid, "     err = %ld   poserr = %ld ticks",
			alt_fiducial[fididx].err[1],
			alt_fiducial[fididx].poserr[1]);
	    }
	    
	    fiducialidx[ALTITUDE] = fididx;

	    if(alt_fiducial[fididx].markvalid) {
	       maybe_reset_axis_pos(ALTITUDE, alt_fiducial[fididx].poserr, 0);
	    }
	 }
      } else if(dio316int_bit & INSTRUMENT_INT) {
	 latchpos[latchidx].axis = INSTRUMENT;
	 ret = semTake(semMEI,WAIT_FOREVER);
	 assert(ret != ERROR);

	 get_latched_position_corr(2*INSTRUMENT,
				   &latchpos[latchidx].pos[1]);
	 get_latched_position_corr(2*INSTRUMENT + 1,
				   &latchpos[latchidx].pos[2]);
	 get_velocity(2*INSTRUMENT, &vel);

	 latchpos[latchidx].pos[1] += ROT_FID_BIAS; /* make always +ve */
	 latchpos[latchidx].pos[2] += ROT_FID_BIAS;

	 semGive(semMEI);
/*
 * have we already seen a rotator fiducial? If so, we know where we are on
 * the tape as the position of every other fiducial is dithered a little
 */
	 if(rot_latch == 0) {
	    rot_latch = latchpos[latchidx].pos[1];
	    semGive(semLatch);
	    continue;
	 }
	 
	 fididx = get_rot_fididx(&latchpos[latchidx], rot_latch, &big);
	 fididx = remap_fiducials(INSTRUMENT, fididx);

	 if(fididx < 0) {
	    NTRACE_1(0, uid, cid, "Failed to identify rotator fiducial at %.2f",
		     (latchpos[latchidx].pos[1] - ROT_FID_BIAS)/ticks_per_degree[INSTRUMENT]);
	    
	    rot_latch = latchpos[latchidx].pos[1];
	    semGive(semLatch);
	    continue;
	 }

	 if(fididx == 0) {
	    ;			/* we just crossed the same fiducial twice */
	 } else {
	    rot_fiducial[fididx].last[1] = rot_fiducial[fididx].mark[1];
/*
 * Is the current or previous latch the one to use? We only use the
 * evenly-spaced marks, not the dithered ones in between
 *
 * XXX We could in fact use all the reference marks, but that wasn't the
 * way that Charlie set things up.
 */
	    if((big && latchpos[latchidx].pos[1] > rot_latch) ||
	       (!big && latchpos[latchidx].pos[1] < rot_latch)) {
	       pos_is_mark = 0;
	       rot_fiducial[fididx].mark[1] = rot_latch;
	    } else {
	       pos_is_mark = 1;
	       rot_fiducial[fididx].mark[1] = latchpos[latchidx].pos[1];
	    }
	    
	    if(rot_fiducial[fididx].last[1] != 0) {
	       rot_fiducial[fididx].err[1] =
		 rot_fiducial[fididx].mark[1] - rot_fiducial[fididx].last[1];
	    }
	 }
/*
 * And poserr[1], the error relative to the known position of fiducial 1
 */
	 if(rot_fiducial[fididx].fiducial[1] != 0) {
	    rot_fiducial[fididx].poserr[1] =
	      rot_fiducial[fididx].mark[1] - rot_fiducial[fididx].fiducial[1];
	 }
	 
	 if(fididx > 0) {
	    if(!rot_fiducial[fididx].disabled) {
	       rot_fiducial[fididx].markvalid = TRUE;
	    }
	    fiducial[INSTRUMENT].seen_fiducial = TRUE;
	    fiducialidx[INSTRUMENT] = fididx;

	    write_fiducial_log("ROT_FIDUCIAL", INSTRUMENT,
			       (pos_is_mark ? fididx : -fididx),
			       rot_fiducial[fididx].fiducial[1],
			       rot_fiducial[fididx].fiducial[2],
			       latchpos[latchidx].pos[1],
			       latchpos[latchidx].pos[2],
			       vel, 0.0, rot_latch, 0);
	 }
      
	 if(fididx > 0) {
	    if(!pos_is_mark) {
	       NTRACE_1(4, uid, cid, "Intermediate rot fiducial %.2f deg",
			(latchpos[latchidx].pos[1] - ROT_FID_BIAS)/ticks_per_degree[INSTRUMENT]);
	    } else {
	       static char pos[20];	/* won't appear properly in TRACE log*/

	       sprintf(pos, "%.2f",
		  (rot_fiducial[fididx].mark[1] -
		   ROT_FID_BIAS)/ticks_per_degree[INSTRUMENT]);

	       NTRACE_2(4, uid, cid, "rot fiducial %d (%s) deg", fididx, pos);
      	       NTRACE_2(6, uid, cid, "     pos = %ld, rot_latch = %ld", rot_fiducial[fididx].mark[1], rot_latch);

	       if(rot_fiducial[fididx].last[1] == 0) {
		  NTRACE_1(4, uid, cid, "     err = ???  poserr = %ld ticks", rot_fiducial[fididx].poserr[1]);
	       } else {
		  NTRACE_2(4, uid, cid, "     err = %ld   poserr = %ld ticks",
			   rot_fiducial[fididx].err[1],
			   rot_fiducial[fididx].poserr[1]);
	       }
	    }
	    
	    if(pos_is_mark && rot_fiducial[fididx].markvalid) {
	       maybe_reset_axis_pos(INSTRUMENT, rot_fiducial[fididx].poserr,0);
	    }
	 }
      
	 fiducial[INSTRUMENT].last_latch = rot_latch =
						     latchpos[latchidx].pos[1];
      } else {
	 NTRACE(0, uid, cid, "More than one axis reports a fiducial interrupt");
      }

      semGive(semLatch);
   }
}

/*=========================================================================
**
**	Task is spawned to delay enabling interrupt and arming latch
**
** GLOBALS REFERENCED:
**	tm_DIO316
**	semMEI
**
**=========================================================================
*/
void
DIO316ClearISR_delay(void)
{
   int uid = 0, cid = 0;
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

      NTRACE_2(6, uid, cid, "DIO316ClearISR_delay: received message %d %d",
	       msg.type, msg.u.latchReenable.dio316int_bit);
      
      assert(msg.type == latchReenable_type);
      dio316int_bit = msg.u.latchReenable.dio316int_bit;
/*
 * OK, we have our orders.  We need to make rearming the MEI and reenabling
 * interrupts as nearly atomic as possible, hence the taskLock().
 */
      status = semTake(semMEI,WAIT_FOREVER);
      assert(status == OK);

      NTRACE(5, uid, cid, "arming latches");

      taskLock();

      while((status = arm_latch(TRUE)) != DSP_OK) {
	 NTRACE_1(0, uid, cid, "Trying to ARM Latch; status=%d", status);
      }
      semGive (semMEI);
#if 0
/*
 * Wait if so requested.  We want to be sure that there's no interrupt
 * until after the MEI has been rearmed.
 *
 * This is not a good idea; the MEI can latch, and we'll miss
 * the interrupt while delaying
 */
      taskDelay(msg.u.latchReenable.timeout);
#endif
/*
 * Reenable interrupts on all axes
 */
      DIO316ClearISR(tm_DIO316);

      DIO316_Interrupt_Enable_Control(tm_DIO316, 1, DIO316_INT_ENA);
      DIO316_Interrupt_Enable_Control(tm_DIO316, 2, DIO316_INT_ENA);
      DIO316_Interrupt_Enable_Control(tm_DIO316, 3, DIO316_INT_ENA);

      taskUnlock();
   }
}	 

/*****************************************************************************/
/*
 * Update all fiducial "mark" and "last1" values for the given axis;
 * usually called after setting a fiducial
 */
static void
update_fiducial_errors(int axis,	/* the axis */
		       long *corr)	/* correction to apply */
{
   int i, j;
   
   switch (axis) {
    case AZIMUTH:
      for(i = 0; i < N_AZ_FIDUCIALS; i++) {
	 if(az_fiducial[i].markvalid) {
	    for(j = 1; j <= 2; j++) {
	       az_fiducial[i].mark[j] += corr[j];
	       
	       if(az_fiducial[i].last[j] != 0) {
		  az_fiducial[i].last[j] += corr[j];
	       }
	    }
	 }
      }
      break;
    case ALTITUDE:
      for(i = 0; i < N_ALT_FIDUCIALS; i++) {
	 if(alt_fiducial[i].markvalid) {
	    for(j = 1; j <= 2; j++) {
	       alt_fiducial[i].mark[j] += corr[j];
	       
	       if(alt_fiducial[i].last[j] != 0) {
		  alt_fiducial[i].last[j] += corr[j];
	       }
	    }
	 }
      }
      break;
    case INSTRUMENT:
      for(i = 0; i < N_ROT_FIDUCIALS; i++) {
	 if(rot_fiducial[i].markvalid) {
	    for(j = 1; j <= 2; j++) {
	       rot_fiducial[i].mark[j] += corr[j];
	       
	       if(rot_fiducial[i].last[j] != 0) {
		  rot_fiducial[i].last[j] += corr[j];
	       }
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
ms_map_dump_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_map_dump");
   return "";
}
			
char *
ms_map_load_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_map_load");
   return "";
}
      
/*****************************************************************************/
/*
 * Read/write fiducials for a current axis from/to a file
 */
char *
ms_read_cmd(int uid, unsigned long cid, char *cmd)
{
   char filename[200];			/* name of file to read */
   char *reply = NULL;

   if(sscanf(cmd, "%s", filename) != 1) {
      return("ERR: no filename supplied");
   }

   reply = read_fiducials(filename, ublock->axis_select);

   broadcast_fiducial_status(uid, cid);
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_read");

   return reply;
}

char *
ms_write_cmd(int uid, unsigned long cid, char *cmd)
{
   char filename[200];			/* name of file to write */
   char *reply = "";

   if(sscanf(cmd, "%s", filename) != 1) {
      return("ERR: no filename supplied");
   }

   reply = write_fiducials(filename, ublock->axis_select);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_write");

   return(reply);
}

/*****************************************************************************/
/*
 * Set a fiducial, i.e. set an axis' position to a fiducial's known value
 */
char *
ms_set_axis_pos_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   mcp_set_fiducial(ublock->axis_select);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_set_axis_pos");
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
set_ms_on(int uid, unsigned long cid,
	  int axis)			/* the axis in question */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
/*
 * Abort any pending MS.OFFs and request the MS.ON
 */
   switch (axis) {
    case AZIMUTH:
      (void)timerSendArg(-ms_off_az_type, tmr_e_abort_ns, 0, uid, cid, 0);
      msg.type = ms_on_az_type;
      break;
    case ALTITUDE:
      (void)timerSendArg(-ms_off_alt_type, tmr_e_abort_ns, 0, uid, cid, 0);
      msg.type = ms_on_alt_type;
      break;
    case INSTRUMENT:
      (void)timerSendArg(-ms_off_inst_type, tmr_e_abort_ns, 0, uid, cid, 0);
      msg.type = ms_on_inst_type;
      break;
   }
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLatched, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   if(ret != OK) {
      NTRACE_2(0, uid, cid, "Failed to send MS.ON message: %d %s", errno, strerror(errno));
      return(-1);
   }

   return(0);
}

int
set_ms_off(int uid, unsigned long cid,
	   int axis,			/* the desired axis */
	   float delay)			/* delay until MS.OFF takes effect, s*/
{
   
   MCP_MSG msg;				/* message to send */
   int ret;				/* a return code */

   switch (axis) {
    case AZIMUTH:    msg.type = ms_off_az_type; break;
    case ALTITUDE:   msg.type = ms_off_alt_type; break;
    case INSTRUMENT: msg.type = ms_off_inst_type; break;
   }
   msg.uid = uid;
   msg.cid = cid;
/*
 * abort any pending MS.OFFs 
 */
   NTRACE(6, uid, cid, "Aborting old MS.OFFs");

   (void)timerSendArg(-msg.type, tmr_e_abort_ns, 0, uid, cid, 0);
   taskDelay(1);			/* give the timerTask a chance */
/*
 * actually set (or schedule) MS.OFF
 */
   NTRACE_2(3, uid, cid, "%s MS.OFF scheduled: %d\n",
	    axis_name(axis), (int)(delay + 0.5));
      
   if(delay == 0) {			/* no time specified */
      NTRACE_1(5, uid, cid, "Sending message to msgLatched: type %d", msg.type);
      
      ret = msgQSend(msgLatched, (char *)&msg, sizeof(msg),
		     NO_WAIT, MSG_PRI_NORMAL);
      assert(ret == OK);
    } else {				/* wait a while and send MS.OFF */
      NTRACE_2(1, uid, cid, "Sending msg to tTimerTask/msgLatched: type %d delay %d ticks",
	       msg.type, (int)(delay*60));
      if(timerSendArg(-msg.type, tmr_e_add, delay*60, uid, cid, msgLatched) == ERROR) {
	 NTRACE_2(0, uid, cid, "Failed to send ms_off message to timer task: %s (%d)",
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
ms_on_cmd(int uid, unsigned long cid,
	  char *cmd)			/* NOTUSED */
{
   const int axis = ublock->axis_select;
   
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_on");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(set_ms_on(uid, cid, axis) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_on");

      return "ERR: failed to set MS.ON";
   } else {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_on");
   }
   
   return "";
}

/*
 * set MS.OFF
 */
char *
ms_off_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   float delay;				/* delay until MS.OFF takes effect, s*/
   float time_off;			/* when MS.OFF should take effect */
   
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_off");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(sscanf(cmd, "%f", &time_off) != 1) { /* no time specified */
      delay = 0;
   } else {
      delay = sdss_delta_time(time_off, sdss_get_time()); /* how long to wait*/

      if(delay < 0) {
	 delay = 0;
      }
   }

   if(set_ms_off(uid, cid, axis, delay) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_off");

      return "ERR: failed to set MS.OFF";
   } else {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_off");
   }
   
   return "";
}

/*
 * Set the maximum change in position that MS.ON is allowed to make
 */
char *
ms_max_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   const int axis = ublock->axis_select;
   long ms_max;				/* maximum change due to MS.ON */
   int old;				/* old value */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_max");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(sscanf(cmd, "%ld", &ms_max) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "ms_max");
      return "ERR: maximum offset not supplied";
   }
   
   old = set_max_fiducial_correction(axis, ms_max); /* do the work */
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ms_max");
   
   sprintf(ublock->buff, "%d", old);
   
   return ublock->buff;
}

/*****************************************************************************/
/*
 * Set the minimum disagreement between encoders to report; if -ve never
 * complain about the encoders
 */
char *
min_encoder_mismatch_cmd(int uid, unsigned long cid, char *cmd)	/* NOTUSED */
{
   const int axis = ublock->axis_select;
   long min_error;			/* minimum error to report */
   int old;				/* old value */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "min_encoder_mismatch");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   if(sscanf(cmd, "%ld", &min_error) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "min_encoder_mismatch");
      return "ERR: minimum error is not supplied";
   }
   
   old = set_min_encoder_mismatch_error(axis, min_error); /* do the work */
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "min_encoder_mismatch");
   
   sprintf(ublock->buff, "%d", old);
	 
   return ublock->buff;
}

/*****************************************************************************/
/*
 * Correct the position of the selected axis by an unlimited amount
 */
char *
correct_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   const int axis = ublock->axis_select;
   
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "correct");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   
   if(semTake(semLatch, 60) == ERROR) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "correct");
      
      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "text", "correct_cmd cannot take semLatch");
      return "ERR: correct_cmd cannot take semLatch";
   }


   if(!fiducial[axis].seen_fiducial) {
      semGive(semLatch);
      
      sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "text", "no fiducials have been crossed");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "correct");

      return "ERR: no fiducials have been crossed";
   }
   
   maybe_reset_axis_pos(axis, NULL, 1);
   
   semGive(semLatch);
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "correct");

   return "";
}

/*****************************************************************************/
/*
 * Initialise an array of FIDUCIALS
 */
static void
reset_fiducials(int axis)
{
   struct FIDUCIALS *axis_fiducial; /* fiducials data structure */
   int i, j;
   int n_fiducials;			/* number of fiducials */

   switch (axis) {
    case AZIMUTH:
      n_fiducials = N_AZ_FIDUCIALS;
      axis_fiducial = az_fiducial;
      break;
    case ALTITUDE:
      n_fiducials = N_ALT_FIDUCIALS;
      axis_fiducial = alt_fiducial;
      break;
    case INSTRUMENT:
      n_fiducials = N_ROT_FIDUCIALS;
      axis_fiducial = rot_fiducial;
      break;
    default:
      fprintf(stderr,"reset_fiducials: illegal axis %d\n", axis);
      return;
   }
   
   fiducialidx[axis] = -1;
   strcpy(fiducialVersion[axis], "blank");
   for(i = 0; i < n_fiducials; i++) {
      axis_fiducial[i].markvalid = FALSE;
      axis_fiducial[i].disabled = TRUE;
      for(j = 0; j <= 2; j++) {
	 axis_fiducial[i].last[j] = 0;
	 axis_fiducial[i].fiducial[j] = 0;
	 axis_fiducial[i].err[j] = 0;
	 axis_fiducial[i].poserr[j] = 0;
	 axis_fiducial[i].mark[j] = 0;
      }
   }
}

/*****************************************************************************/
/*
 * Set the fiducials from a file.
 */
#define LSIZE 200

char *
read_fiducials(const char *file,	/* file to read from */
	       int axis)		/* which axis to read */
{
   int uid = 0, cid = 0;
   char axis_str[40];			/* name of axis from file */
   int bias;				/* bias applied to fiducial positions*/
   int canonical;			/* canonical fiducial for an axis */
   float error1, error2;		/* error in mark (encoders 1,2) */
   int fid;				/* fiducial number from file */
   char fid_str[LSIZE];			/* buffer to read string "fiducials" */
   struct FIDUCIALS *axis_fiducial;	/* fiducials data structure */
   FILE *fil;				/* F.D. for file */
   char line[LSIZE + 1];		/* buffer to read lines of file */
   char *lptr;				/* pointer to line[] */
   float mark1, mark2;			/* mark from file (encoders 1,2) */
   int npt1, npt2;			/* number of points used to find mark
					   (encoders 1,2) */
   int n_fiducials;			/* max number of fiducials */
   int nread;				/* number of values read from a line */
   double scale1, scale2;		/* scales for encoder 1 and 2 */

   switch (axis) {
    case AZIMUTH:
      n_fiducials = N_AZ_FIDUCIALS;
      axis_fiducial = az_fiducial;
      bias = 0;
      break;
    case ALTITUDE:
      n_fiducials = N_ALT_FIDUCIALS;
      axis_fiducial = alt_fiducial;
      bias = 0;
      break;
    case INSTRUMENT:
      n_fiducials = N_ROT_FIDUCIALS;
      axis_fiducial = rot_fiducial;
      bias = ROT_FID_BIAS;
      break;
    default:
      fprintf(stderr,"read_fiducials: illegal axis %d\n", axis);
      return("ERR: illegal axis");
   }
/*
 * Initialise fiducials array
 */
   reset_fiducials(axis);
/*
 * Open file, read header and data, and set fiducials array
 */
   NTRACE_2(1, uid, cid, "Reading %s fidcls from %s", axis_name(axis), file);
   
   if((fil = fopen(file, "r")) == NULL) {
      NTRACE_2(0, uid, cid, "Failed to open %s for read (%s)", file, strerror(errno));
      return("ERR: failed to open file\n");
   }
   
   line[sizeof(line)] = '\0';
   while((lptr = fgets(line, sizeof(line) - 1, fil)) != NULL) {
      while(isspace((int)*lptr)) lptr++;

      if(*lptr == '\0') {		/* blank line */
	 continue;
      } else if(*lptr == '#') {		/* a comment */
	 lptr++;
	 while(isspace((int)*lptr)) lptr++;
	 if(*lptr == '\0') {		/* empty comment */
	    continue;
	 }

	 if(sscanf(lptr, "%s %s", axis_str, fid_str) == 2 &&
	    strcmp(fid_str, "fiducials") == 0) {
	    if(strcmp(axis_str, axis_name(axis)) != 0) {
	       NTRACE_2(0, uid, cid, "Expected fiducials of type %s; saw %s; proceeding",
			axis_name(axis), axis_str);
	    }
	 } else if((nread =
		    sscanf(lptr, "Scales: %lf %lf", &scale1, &scale2)) > 1) {
	    fiducial[axis].scale[1] = scale1;
	    if(nread > 1) {
	       fiducial[axis].scale[2] = scale2;
	    }

	    if(scale2 == 0.0) {
	       fiducial[axis].scale_ratio_12 = -1.0;
	    } else {
	       fiducial[axis].scale_ratio_12 = scale1/scale2;
	    }
	 } else if(sscanf(lptr, "Canonical fiducial: %d", &canonical) == 1) {
	    if(canonical < 0 || canonical >= n_fiducials) {
	       fprintf(stderr,"Invalid canonical fiducial %d in file %s\n",
		       canonical, file);
	       NTRACE_2(0, uid, cid, "Invalid canonical fiducial %d in file %s",
			canonical, file);
	    } else {
	       fiducial[axis].canonical = canonical;
	    }
	 } else if(!strncmp(lptr, "$Name: ", 7)) {
	   char *vend;

	   NTRACE_2(2, uid, cid, "Fiducial id from %s in file %s", lptr, file);

	   /* Skip over leading "Name: " */
	   lptr += strlen("$Name: ");
	   while(isspace((int)*lptr)) lptr++;
	   if (*lptr == '\0') {
	     strcpy(fiducialVersion[axis], "NOCVS"); 
	     continue;
	   }

	   vend = lptr;
	   while (*vend && *vend != '$' && !isspace((int)*vend)) {
	      vend++;
	   }
	   if (vend-lptr >= FIDVERLEN) {
	      NTRACE_2(0, uid, cid, "Silly fiducial version length %ld in file %s", (vend - lptr), file);
	      vend = lptr+FIDVERLEN-1;
	   }
	   strncpy(fiducialVersion[axis], lptr, vend-lptr);
	   fiducialVersion[axis][vend-lptr] = '\0';
	 } 
	 continue;
      }

      nread = sscanf(lptr, "%d  %f +- %f %d  %f +- %f %d",
		     &fid, &mark1, &error1, &npt1, &mark2, &error2, &npt2);
      if(nread == 4 || nread == 7) {
	 if(fid <= 0 || fid >= n_fiducials) {
	    fprintf(stderr,"Invalid fiducial %d in file %s\n", fid, file);
	    NTRACE_2(0, uid, cid, "Invalid fiducial %d in file %s", fid, file);
	    continue;
	 }

	 axis_fiducial[fid].fiducial[1] = mark1 + bias;
	 if(error1 >= 0) {		/* valid position */
	    axis_fiducial[fid].disabled = FALSE;
	 }

	 if(nread == 4) {
	    axis_fiducial[fid].fiducial[2] = 0;
	 } else {
	    axis_fiducial[fid].fiducial[2] = mark2 + bias;
	 }
      } else {
	 NTRACE_1(0, uid, cid, "Corrupt line: %s", line);
	 continue;
      }
   }
			
   fclose(fil);
/*
 * Are any fiducials missing?  If so interpolate their positions,
 * and mark them invalid.  Why bother?  Because for AZ we use
 * the approximate positions to identify fiducials
 */
   for(fid = 0; fid < n_fiducials; fid++) {
      if(axis_fiducial[fid].disabled) {
	 int i, j, k;
	 
	 for(i = fid - 1; i >= 0; i--) { /* look backwards */
	    if(!axis_fiducial[i].disabled) {
	       break;
	    }
	 }
	 if(i < 0) {			/* cannot interpolate */
	    continue;
	 }
	 
	 for(j = fid + 1; j < n_fiducials; j++) { /* and forwards */
	    if(!axis_fiducial[j].disabled) {
	       break;
	    }
	 }
	 if(j >= n_fiducials) {		/* cannot interpolate */
	    continue;
	 }
/*
 * OK, we have fiducials to the left and right of us.  Set the
 * position but don't, of course, mark it as valid
 */
	 for(k = 1; k <= 2; k++) {
	    float pi = axis_fiducial[i].fiducial[k];
	    float pj = axis_fiducial[j].fiducial[k];
	    axis_fiducial[fid].fiducial[k] = pi + (fid - i)*(pj - pi)/(j - i);
	 }
      }
   }
/*
 * Set canonical positions etc.
 */
   fiducial[axis].canonical_position =
     axis_fiducial[fiducial[axis].canonical].fiducial[1];

   if(axis_fiducial[fiducial[axis].canonical].fiducial[2] != 0 &&
      (fiducial[axis].canonical_position !=
			axis_fiducial[fiducial[axis].canonical].fiducial[2])) {
      NTRACE_2(1, uid, cid, "%s: value of canonical fiducial %d differs for 2 encoders",
	       axis_name(axis), fiducial[axis].canonical);
   }

   return("");
}

/*****************************************************************************/
/*
 * Write the current fiducials to a file.
 *
 * See also read_fiducials
 */
char *
write_fiducials(const char *file,	/* file to write to */
	       int axis)		/* which axis to write */
{
   int uid = 0, cid = 0;
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
   NTRACE_2(1, uid, cid, "Writing %s fidls to %s", axis_name(axis), file);
   
   if((fil = fopen(file, "w")) == NULL) {
      NTRACE_2(0, uid, cid, "Failed to open %s for write (%s)", file, strerror(errno));
      return("ERR: failed to open file\n");
   }

   fprintf(fil, "#\n");
   fprintf(fil, "# %s fiducials\n", axis_name(axis));
   fprintf(fil, "# Written from MCP\n");
   fprintf(fil, "#\n");
   fprintf(fil,
	  "# Fiducial Encoder1 +- error  npoint  Encoder2 +- error  npoint\n");
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
	 
      fprintf(fil, "%-4d     %10ld +- %5.1f %3d    %10ld +- %5.1f %3d\n", i,
	      fiducials[i].fiducial[1], err, npt,
	      fiducials[i].fiducial[2], err, npt);
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

  printf("%s. MS.ON: %d (max correction: %ld)\n", axis_name(axis),
	 fiducial[axis].ms_on, fiducial[axis].max_correction);

  switch (axis) {
   case AZIMUTH:
     for(i = 0;i < N_AZ_FIDUCIALS; i++) {
	if(az_fiducial[i].markvalid) {
	   printf("AZ %d %d degs:  enabled %c pos= %ld mark= %ld last= %ld "
		  " err= %ld, poserr= %ld\n",
		  i, (int)(az_fiducial[i].mark[1]/ticks_per_degree[AZIMUTH]),
		  (az_fiducial[i].disabled ? 'N' : 'Y'),
		  az_fiducial[i].fiducial[1],
		  az_fiducial[i].mark[1], az_fiducial[i].last[1],
		  (az_fiducial[i].last[1] == 0 ? 0 : az_fiducial[i].err[1]),
		  az_fiducial[i].poserr[1]);
	} else {
	   if(show_all) {
	      printf("AZ %d:  pos=%ld\n", i, az_fiducial[i].fiducial[1]);
	   }
	}     
     }
     break;
   case ALTITUDE:
     for(i = 0;i < N_ALT_FIDUCIALS; i++) {
	if(alt_fiducial[i].markvalid) {
	   printf("ALT %d %d degs:  enabled %c pos= %ld, mark= %ld, last= %ld "
		  " err= %ld, poserr= %ld\n",
		  i, (int)(alt_fiducial[i].mark[1]/ticks_per_degree[ALTITUDE]),
		  (alt_fiducial[i].disabled ? 'N' : 'Y'),
		  alt_fiducial[i].fiducial[1],
		  alt_fiducial[i].mark[1], alt_fiducial[i].last[1],
		  (alt_fiducial[i].last[1] == 0 ? 0 : alt_fiducial[i].err[1]),
		  alt_fiducial[i].poserr[1]);
	} else {
	   if(show_all) {
	      printf("ALT %d:  pos=%ld\n",i,alt_fiducial[i].fiducial[1]);
	   }
	}
     }
     break;
   case INSTRUMENT:
     for(i = 0;i < N_ROT_FIDUCIALS; i++) {

	if(rot_fiducial[i].markvalid) {	   
	   printf("ROT %d %d degs: enabled %c pos= %ld, mark= %ld, last= %ld "
		  " err= %ld, poserr= %ld\n", i,
		  (int)((rot_fiducial[i].mark[1] -
			 ROT_FID_BIAS)/ticks_per_degree[INSTRUMENT]),
		  (rot_fiducial[i].disabled ? 'N' : 'Y'),
		  rot_fiducial[i].fiducial[1],
		  rot_fiducial[i].mark[1], rot_fiducial[i].last[1],
		  (rot_fiducial[i].last[1] == 0 ? 0 : rot_fiducial[i].err[1]),
		  rot_fiducial[i].poserr[1]);
	} else {
	   if(show_all) {
	      printf("ROT %d:  pos=%ld\n",i, rot_fiducial[i].fiducial[1]);
	   }
	}     
     }
     break;
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
 * Mark all fiducials invalid
 *
 * Spawn tasks
 */
void
tLatchInit(void)
{
   int i;

   write_fiducial_log("START_FIDUCIAL", NAXIS, 0, 0, 0, 0, 0,  0.0, 0.0, 0, 0);
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
 * Initialise arrays
 */
   reset_fiducials(AZIMUTH);
   reset_fiducials(ALTITUDE);
   reset_fiducials(INSTRUMENT);
/*
 * Initialise fiducial[] array
 */
   for(i = 0; i < NAXIS; i++) {
      fiducial[i].seen_fiducial = FALSE;
      fiducial[i].ms_on = 0;
      fiducial[i].error[0] = 0;
      fiducial[i].error[1] = fiducial[i].error[2] = 0;
      (void)set_max_fiducial_correction(i, 0);
      fiducial[i].last_latch = 0;
      fiducial[i].scale[0] = -1;	/* unused */
      fiducial[i].scale[1] = fiducial[i].scale[2] = 0.0;
      fiducial[i].scale_ratio_12 = -1;
      set_max_fiducial_correction(i, 0);
      fiducial[i].canonical = 0;
      fiducial[i].canonical_position = 0;
   }      
/*
 * Spawn the task that processes fiducial crossings
 */
   taskSpawn("tLatch",49,VX_FP_TASK,10000,(FUNCPTR)tLatch,
	     0,0,0,0,0,0,0,0,0,0);
/*
 * Declare commands to the command interpreter
 */
   define_cmd("CORRECT",      correct_cmd,         0, 1, 0, 1, "");
   define_cmd("MS_MAP_DUMP",  ms_map_dump_cmd,     0, 0, 0, 1, "");
   define_cmd("MS_MAP_LOAD",  ms_map_load_cmd,     0, 1, 0, 1, "");
   define_cmd("MS_OFF",       ms_off_cmd,         -1, 1, 0, 1, "");
   define_cmd("MS_ON",        ms_on_cmd,           0, 1, 0, 1, "");
   define_cmd("MS_MAX",       ms_max_cmd,          1, 1, 0, 1, "");
   define_cmd("MS_READ",      ms_read_cmd,         1, 1, 0, 1, "");
   define_cmd("MS_SET",       ms_set_axis_pos_cmd, 0, 1, 0, 1, "");
   define_cmd("MS_WRITE",     ms_write_cmd,        1, 0, 0, 0,
"Write the fiducial table for the current axis to the specified file;\n"
"e.g.  ROT MS.WRITE foo.rot\n"
"This command may be used to check what fiducial tables are actually in use"
	      );
   define_cmd("SET_FIDUCIAL", ms_set_axis_pos_cmd, 0, 1, 0, 1, "");
   define_cmd("AZ_BARCODE",   az_barcode_cmd,      1, 0, 0, 1,
"Tell the fiducials to use (1) or not use (0) the azimuth barcode reader\n"
"to identify fiducials.  If the MCP is totally lost, you can use the\n"
"   <axis> SET.POSITION <pos>\n"
"command to get close enough, or use\n"
"    AZ.BARCODE 1\n"
"to temporarily enable the barcode reader");
   define_cmd("MIN_ENCODER_MISMATCH", min_encoder_mismatch_cmd, 1, 1, 0, 1,
"Set the minimum reportable disagreement between encoders\n"
"If negative, never report disagreements"
	      );
}
