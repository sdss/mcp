/*
 *	Keep the umbilical cord to the mosaic camera off the floor as 
 *	determined by alt/az position.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <semLib.h>
#include <taskLib.h>
#include "instruments.h"
#include "abdh.h"
#include "data_collection.h"
#include "axis.h"
#include "tm.h"
#include "trace.h"
#include "cmd.h"

/*****************************************************************************/
/*
 * Here's JEG's umbilical tower height code, as translated from Forth
 * by poor, long suffering, RHL.
 *
 * Formulae (due to JEG):
 *
 * Make a coordinate system
 *   x, back from axis intersection, along floor
 *   y, 'south' (at stow) parallel to el axis, along floor 
 *   z, up from floor at axis intersection.
 *
 * Then the x,y,z coord of the beginning of the vertical part of the hose
 * nearest the camera  is (B,S,h), and
 *
 *  B = (D1+Rh)*cos(eps) + R*sin(eps)*cos(thet)
 *  S = -R*sin(thet)
 *  h = H + (R*cos(thet)-Rh)*cos(eps) - D1*sin(eps)
 *
 * The length of hose in the bend close to the camera is
 *  Lb = Rh*(Pi/2 - eps).
 *
 * The distance between vertical segments is
 *  l = sqrt((Bt-B)^2 + (St-S)^2),
 *
 * The length of the vertical segment closest to the camera (e) is
 *   e = h - l/2 - Cmin
 * and must be positive in order that this model be correct.
 *
 *  f = Lf - Lb - e - l*Pi/2
 *
 * if f is calculated to be negative, set it to zero, recalculate e and C:
 *  e = Lf - Lb - l*Pi/2
 *  C = h - l/2 - e
 *
 * Then
 *  T = f + l/2 + C
 *  dht = T - Tmin    <-- this is what we want to know
 *
 * if dht is negative, increment e by half of it, decrement C and f by
 * half of it and set dht to zero.
 *
 *
 * For sufficiently large l, this will not work, and we must do something else.
 * If after all of this, e is negative, we go to another model.
 * 
 * First, we make a model in which the circle which the hose is assumed to
 * follow is tangent to the vertical line from the tower attach point and
 * to a line parallel to the floor Cmin above the floor.  The angle from
 * the vertical at which the hose hangs from the camera can be estimated
 * from the formal negative value of e:
 *
 *  adec =  (-2*e/l)
 *
 * using this value, the circular arc adjacent to the camera attach point
 * swings thru Pi/2 - eps - adec, and uses Rh*(Pi/2-eps-adec) of hose.  The
 * end of this little arc is not at quite the same place as in the other
 * model because it is shorter; H is smaller by Rh*sin(adec); B by
 * Rh*(1-cos(adec)).  We will neglect the last, because adec is never very
 * big.  So l is assumed unchanged.  Thus center of the circle satisfies
 *
 *  x^2 = (x-x0)^2 + (y-y0)^2
 *
 * where x is the distance from the vertical line dropped from the tower a.p.,
 * y is the distance above Cmin above the floor, x0 = l,
 * and y0 is h - Cmin + Rh*sin(adec). The solution of this quadratic is
 *  x = x0 + y0 - sqrt(2*x0*y0).
 * 
 * x is clearly also the radius of the circle.   
 *
 * The length of the hose from the camera to the tower tangent point is
 *  L = Rh/2 + x*A   
 *
 * Where A is the total angular swing of the hose, which is less than Pi
 * by asin((x-y0)/x). Then the straight length to the tower a.p. is 
 *  f = Lf - L.
 *
 * If f<0, this model fails, and another in which there is a continuous
 * arc from the camera a.p. to the tower a.p. is necessary. In fact, it
 * appears that this is not really necessary, as f is never VERY negative,
 * and the real parameters are sufficiently accurately calculated.   
 */
/*
 * Convert inches to string-pot encoder counts, on the assumption
 * that the fully retracted position is 0 inches:tower_encoder0 counts,
 * and the fully extended one is 30 inches:30/tower_encoder_scale counts.
 */
float tower_encoder0 = 600;
float tower_encoder_scale = 33.3;

/*****************************************************************************/
/*
 * Return the desired height of the Umbilical tower for a given
 * telescope position (alt, rot).
 *
 * The result is expressed in string-pot encoder counts
 */
float
calc_tower_height(float alt,		/* altitude, degrees */
		  float rot)		/* rotator degrees, +CCW looking at
					   sky; 0.0 is instrument change pos */
{
   const float Bt = 63.;		/* distance back from elev axis to
					   tower attach pt */
   const float Cmin = 2.0;		/* desired height to be maintained
					   above floor */
   const float D1 = 42.5;		/* distance along rot axis from elev
					   axis to saddle a.p. */
   const float H = 72.1;		/* height of alt axis above floor */
   const float Lf = 124.7;		/* free length of umbilicus */
   const float R = 22.5;		/* distance from attach point on saddle
					   from rot axis */
   const float Rh = 4.0;		/* assumed bend radius of umbilical */
   const float St = 31.;		/* distance perpendicular (side) from
					   alt axis to tower a.p. */
   const float Tmin = 43.;		/* lowest point of tower elevator at
					   free length attach*/
   float B;				/* distance from beginning vert part
					   of hose to elev axis */
   float C;				/* floor clearance */
   float calt, salt;			/* cos/sin(alt) */
   float crot, srot;			/* cos/sin(rot) */
   float dht;				/* height of elevator above min. */
   float e;				/* length of vertical part of hose
					   closest to camera, if any */
   float f;				/* length of vertical part of hose
					   closest to tower. */
   float h;				/* height of beginning of vert part of
					   hose closest to cam */
   float Lb;				/* length of hose in bend close to
					   camera */
   float l;				/* distance along floor from tower a.p.
					   to beginning of vert part of hose
					   closest to camera. */
   float S;				/* perp distance from begin. vert part
					   of hose to optical axis (along y) */
   float T;				/* height of tower attach point */
   float tmp;

   alt *= M_PI/180;			/* convert to radians */
   rot *= M_PI/180;

   calt = cos(alt);
   salt = sin(alt);
   crot = cos(rot);
   srot = sin(rot);

   C = Cmin;
   Lb = (M_PI/2 - alt)*Rh;
   B = (D1 + Rh)*calt + R*salt*crot;
   S = -R*srot;
   h = H + (R*crot - Rh)*calt - D1*salt;
   l = sqrt((Bt - B)*(Bt - B) + (St - S)*(St - S));
    
   e = h - 0.5*l - C;
   tmp = Lf - Lb - M_PI/2*l;
   f = tmp - e;

   if(f < 0) {
      f = 0;
      e = tmp;
      C = h - 0.5*l - e;
   }

   T = f + 0.5*l + C;
   dht = T - Tmin;

   if(dht < 0) {
      dht = 0;
      e += 0.5*dht;
      f -= 0.5*dht;
      C -= 0.5*dht;
      T = Tmin;
   }
/*
 * If e is less than zero, we must use an alternate model
 */
   if(e < 0) {
      float A;				/* angular swing of hose arc */
      const float adec = 2*e/l;		/* estimated initial angular DEFICIT
					   of hose arc (Pi-A) */
      float x;				/* radius of hose arc */
      const float x0 = l;		/* distance from fictitious camera ap
					   to tower ap along floor */
      const float y0 = h - Cmin + Rh*sin(adec); /* height of fictitious camera
						   ap - Cmin */

      C = Cmin;
      e = 0;

      x = x0 + y0 - sqrt(2*x0*y0);
      A = M_PI + asin((y0 - x)/x);
      f = Lf - Rh*(M_PI/2 - alt - adec) - x*A;
      T = Cmin + x + f;
      dht = T - Tmin;
   }
/*
 * strictly speaking, if d<0, we must use yet another model with
 * NO vertical segments, but in practice this one is OK; it still
 * gives quite reliable dht values even for slightly negative f,
 * and f never becomes very negative.
 */
/*
 * Convert to the encoder counts returned by the PLC
 */
   return(tower_encoder0 + tower_encoder_scale*dht);
}

#if 0
int
main(int ac, char *av[])
{
   float alt, rot;
   float height;

#if 0
   while(printf("alt, rot: "), scanf("%f %f", &alt, &rot) == 2) {
      height = calc_tower_height(alt, rot);
      printf("(%f, %f) --> %.1f\n", alt, rot, height);
   }
#else
   for(rot = -180; rot <= 181; rot += 10) {
      printf("%5.0f", rot);
      for(alt = 0; alt <= 91; alt += 5) {
	 height = calc_tower_height(alt, rot);
	 printf("%5.1f", height);
      }
      printf("\n");
   }
#endif

   return(0);
}
#endif

/*****************************************************************************/
/*
 * Code to control the umbilical tower
 */
/*****************************************************************************/
/*
 * Tell the motor to start moving
 */
int
umbilical_move(int val) 
{
   int err;
   unsigned short ctrl;
   struct B10_1 il_ctrl;   
   
   if(semTake(semSLC,60) == ERROR) {
      TRACE(0, "umbilical_move: failed to get semSLC: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }
   
   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl,1);
   if(err) {
      semGive(semSLC);
      TRACE(0, "umbilical_move: error reading slc: 0x%04x", err, 0);
      return(err);
   }
   
   swab((char *)&ctrl, (char *)&il_ctrl, 2);
   il_ctrl.mcp_umbilical_up_dn = val;
   swab((char *)&il_ctrl, (char *)&ctrl, 2);

   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl,1);
   semGive (semSLC);

   if(err) {
      TRACE(0, "umbilical_move: error writing slc: 0x%04x", err, 0);
      return err;
   }

   return 0;
}

int
umbilical_move_dn() 
{
   return(umbilical_move(1));
}

int
umbilical_move_up() 
{
   return(umbilical_move(0));
}

/*****************************************************************************/
/*
 * Turn the umbilical motor on/off
 */
int
umbilical_onoff(int val) 
{
   int err;
   unsigned short ctrl;
   struct B10_1 il_ctrl;   
             
   if(semTake(semSLC,60) == ERROR) {
      TRACE(0, "umbilical: failed to get semSLC: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl,1);
   if(err) {
      semGive(semSLC);
      TRACE(0, "umbilical: error reading slc: 0x%04x", err, 0);
      return(err);
   }

   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_umbilical_on_off = val;
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl,1);
   semGive(semSLC);

   if(err) {
      TRACE(0, "umbilical: error writing slc: 0x%04x", err, 0);
      return err;
   }

   return 0;
}

int
umbilical_on(void)
{
    return(umbilical_onoff(1));
}

int
umbilical_off(void) 
{
   return(umbilical_onoff(0));
}

/*****************************************************************************/
/*
 * Return the current position of the tower, in encoder counts
 */
short
umbilical_position(void)
{
   int err;
   unsigned short pos,position;
   
   if(semTake(semSLC,60) == ERROR) {
      TRACE(0, "umbilical_position: failed to get semSLC: %s (%d)",
	    strerror(errno), errno);
      return(-1);
   }

   err = slc_read_blok(1,9,BIT_FILE,235,&pos,1);
   semGive(semSLC);
   
   if(err) {
      TRACE(0, "umbilical_position: error reading slc: 0x%04x", err, 0);
      return(err);
   }

   swab((char *)&pos, (char *)&position, 2);

   return(position);
}

void
print_umbilical_position(void)
{
   short position;
   
   position = umbilical_position();
   printf("Umbilical Position %d (%2.2f inches)\n",
	  position, (position - tower_encoder0)/tower_encoder_scale);
}

/*****************************************************************************/
/*
 * Move the tower to a specified position
 */
int
umbilical_move_pos(int pos)		/* desired tower position */
{
   int position;
   int lastpos;
   
   position = umbilical_position();
   TRACE(4, "Umbilical Position %d move to %d", position, pos);

   if(pos < position) {			/* move down */
      umbilical_move_dn();
      umbilical_on();
      taskDelay(4);
   
      lastpos = 32767;
      while(position = umbilical_position(), position - pos > 60) {
	 TRACE(4, "Umbilical Position %d moving down to %d",
	       position, pos);
	 if((lastpos - position) < 4) {
	    umbilical_off();
	    TRACE(1, "umbilical_move_pos: stopped moving down to %d (%d)",
		  position, lastpos - position)
	    return(-1);
	 }
	 lastpos = position;
	 taskDelay(1);
      }
   } else {				/* move up */
      umbilical_move_up();
      umbilical_on();
      taskDelay(4);

      lastpos = 0;
      while(position = umbilical_position(), pos - position > 30) {
	 TRACE(4, "Umbilical Position %d moving up to %d",
	       position, pos);
	 
	 if(position - lastpos < 4) {
	    umbilical_off();
	    TRACE(1, "umbilical_move_pos: stopped moving up to %d (%d)",
		  position, position - lastpos)
	    return(-1);
	 }
	 lastpos = position;
	 taskDelay(1);
      }
   }
   
   umbilical_off();

   return 0;
}

/*****************************************************************************/
/*
 * Move the tower to the correct position for the current telescope
 * altitude/instrument rotation
 */
void
umbilical_mgt()
{
   double alt, rot;			/* position in altitude/rotator */
   int umbilical_pt;			/* current position of tower */
   int umbpos;				/* desired position of tower */
   
   tm_get_position(2*ALTITUDE, &alt);
   tm_get_position(2*INSTRUMENT, &rot);

   umbpos = calc_tower_height(alt, rot);
   umbilical_pt = umbilical_position();
   if(umbpos < umbilical_pt - 60 || umbpos > umbilical_pt + 30) {
      umbilical_move_pos(umbpos);
   }
}

void
test_umbilical_mgt(int alt, int rot)
{
   int umbilical_pt;			/* current position of tower */
   int umbpos;				/* desired position of tower */
   
   umbpos = calc_tower_height(alt, rot);
   umbilical_pt = umbilical_position();
   printf("test_umbilical_mgt: move to %d from %d\n", umbpos, umbilical_pt);
   
   if(umbpos < umbilical_pt - 60 || umbpos > umbilical_pt + 30) {
      umbilical_move_pos(umbpos);
   }
}

/*****************************************************************************/
/*
 * Return the ID number of the current instrument
 */
/*
 * The camera ID switches aren't installed as of Jan 2001
 *
 * Use the state of the primary latches/spec corrector to guess what's
 * installed?
 */
#define GUESS_INSTRUMENT 1
   
#define CAMERA_ID 14

int
instrument_id(void)
{
   int inst_id1, inst_id2, inst_id3;	/* values of the inst ID switches */
#if GUESS_INSTRUMENT
   int pri_latch_opn;			/* the primary latches are open */
   int spec_lens;			/* the spec corrector in installed */
#endif
   
   if(semTake(semSDSSDC, NO_WAIT) == ERROR) {
      return(-1);			/* unknown */
   }
   
   inst_id1 = ((sdssdc.status.i1.il8.inst_id1_1 << 3) + 
	       (sdssdc.status.i1.il8.inst_id1_2 << 2) + 
	       (sdssdc.status.i1.il8.inst_id1_3 << 1) + 
	       (sdssdc.status.i1.il8.inst_id1_4 << 0));
   inst_id2 = ((sdssdc.status.i1.il8.inst_id2_1 << 3) + 
	       (sdssdc.status.i1.il8.inst_id2_2 << 2) + 
	       (sdssdc.status.i1.il8.inst_id2_3 << 1) + 
	       (sdssdc.status.i1.il8.inst_id2_4 << 0));
   inst_id3 = ((sdssdc.status.i1.il8.inst_id3_1 << 3) + 
	       (sdssdc.status.i1.il8.inst_id3_2 << 2) + 
	       (sdssdc.status.i1.il8.inst_id3_3 << 1) + 
	       (sdssdc.status.i1.il8.inst_id3_4 << 0));
#if GUESS_INSTRUMENT
   pri_latch_opn = (sdssdc.status.i1.il8.pri_latch1_opn &&
		    sdssdc.status.i1.il8.pri_latch2_opn &&
		    sdssdc.status.i1.il8.pri_latch3_opn);
   spec_lens = (!sdssdc.status.i1.il8.spec_lens1 ||
		 sdssdc.status.i1.il8.spec_lens2);
#endif

   semGive(semSDSSDC);

   if(inst_id1 != inst_id2 || inst_id1 != inst_id3) {
      static char buff[4];
      sprintf(buff,"%1d%1d%1d", inst_id1, inst_id2, inst_id3);
      TRACE(1, "Inconsistent instrument ID switches: %s", buff, 0);
      return(-1);
   }
#if GUESS_INSTRUMENT
   if(inst_id1 == 0 && !pri_latch_opn && !spec_lens) {
      return(CAMERA_ID);
   }
#endif
   
   return(inst_id1);
}

/*****************************************************************************/
/*
 * Is the imager saddle on the telescope? Note that switch 1 is deliberately
 * wired backwards
 */
int
saddle_is_mounted(void)
{
   int saddle_is_on;			/* is the saddle mounted? */
   int sad_mount1, sad_mount2;
  
   if(semTake(semSDSSDC, 10) == ERROR) {
      return(-1);			/* unknown */
   }
   
   sad_mount1 = (sdssdc.status.i1.il9.sad_mount1 == 0) ? 1 : 0; 
   sad_mount2 = (sdssdc.status.i1.il9.sad_mount2 == 0) ? 0 : 1;

   if(sad_mount1 == sad_mount2) {
      saddle_is_on = sad_mount1;
   } else {
      saddle_is_on = -1;		/* inconsistent */
   }
   
   semGive(semSDSSDC);

   return(saddle_is_on);
}

/*****************************************************************************/
/*
 * Globals to control monitoring of umbilical tower
 */
int active_umbilical_control = 1;	/* Are we controlling the tower? */
int umbilical_delay = 1;		/* time (in seconds) between checks */

/*****************************************************************************/
/*
 * A task to control the umbilical tower
 */
int
tUmbilical(void)
{
   for(;;) {
      if(active_umbilical_control && saddle_is_mounted() == 1) {
	 umbilical_mgt();
      }
   
      taskDelay(60*umbilical_delay);
   }
}

char *
umbilical_cmd(char *cmd)
{
   int on_off;				/* true to turn on active control */

   if(sscanf(cmd, "%d", &on_off) != 1) {
      return("ERR: malformed command argument");
   }

   active_umbilical_control = on_off;
   if(!active_umbilical_control) {
      umbilical_move_pos(0);		/* move tower to bottom of travel */
   }

   return("");
}

/*****************************************************************************/
/*
 * Initialise the umbilical control task
 */
int
tUmbilicalInit(int dsec)		/* interval, in seconds, between
					   checks on tower position */
{
   active_umbilical_control = 1;
   umbilical_delay = dsec;
/*
 * Spawn the task that controls the umbilical
 */
   taskSpawn("tUmbilical", 50, 0, 2000,
	     (FUNCPTR)tUmbilical,
	     0,0,0,0,0,0,0,0,0,0);
/*
 * Declare command that controls whether the umbilical tower control
 * is active
 */
   define_cmd("UMBILICAL",    umbilical_cmd,    1, 0, 1);
   
   return 0;
}
