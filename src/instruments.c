#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <semLib.h>
#include "instruments.h"
#include "abdh.h"
#include "data_collection.h"
#include "axis.h"
#include "tm.h"
#include "dscTrace.h"
#include "cw.h"
#include "cmd.h"
#include "as2.h"

/*****************************************************************************/
/*
 * Return the ID number of the current instrument
 */
int
instrument_id(int *instrument_in_place,	/* if any bits are set there's something mounted */
	      int *switches_are_inconsistent) /* are the switches consistent? */
{
   int uid = 0, cid = 0;
   int inconsistent = 0;		   /* are the switches consistend? Innocent until proven guilty */
   int inst_id = -1;		           /* The loaded instrument */
   static int notify = -1;		   /* should I notify user of bad ID? */
   int notify_rate = 60;		   /* how often should I notify? */
   int pri_latch_opn;			   /* the primary latches are open */
   
   if(semTake(semSDSSDC, 100) == ERROR) {
      return(-1);			/* unknown */
   }
	 
   if (instrument_in_place != NULL) {
      *instrument_in_place = !sdssdc.status.b3.w1.no_inst_in_place; /* is an instrument loaded? */
   }

   if (switches_are_inconsistent != NULL) {
      *switches_are_inconsistent = 0;	/* assume the PLC checked them */
   }

   pri_latch_opn = (sdssdc.status.i1.il8.pri_latch1_opn &&
                    sdssdc.status.i1.il8.pri_latch2_opn &&
                    sdssdc.status.i1.il8.pri_latch3_opn);
   
   {
      #define NINST 21
      int ECAM, IMAGER;			/* index for engineering camera and imager */
      int instrument_in_place_plc[NINST];
      int i = 0;
      instrument_in_place_plc[i++] = 0;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_1;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_2;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_3;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_4;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_5;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_6;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_7;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_8;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.cartridge_9;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_10;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_11;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_12;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_13;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_14;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_15;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_16;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_17;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w7.cartridge_18;
      ECAM = i;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.eng_cam_in_place;
      IMAGER = i;
      instrument_in_place_plc[i++] = sdssdc.status.b3.w1.img_cam_in_place;
      assert (i == NINST);

      for (i = 0; i < NINST; ++i) {
	 if (instrument_in_place_plc[i]) {
	    inst_id = i;
	    break;
	 }
      }
   }

   semGive(semSDSSDC);

   if(!inconsistent || pri_latch_opn /* not latched */) {
      if (notify >= 0) {		/* we were inconsistent */
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id, inst_id, inst_id);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
      }
      notify = -1;
   } else {
      notify = (notify + 1)%notify_rate;

      if(notify == 0) {
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id, inst_id, inst_id);
	 NTRACE_1(2, uid, cid, "Inconsistent instrument ID switches: %s", buff);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);

	 sprintf(buff,"%d, %d, %d", inst_id, inst_id, inst_id);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
	 
	 return(-1);
      }
   }
   
   return(inst_id);
}

/*****************************************************************************/
/*
 * Return the ID number of the current instrument
 */
#if 0
int
old_instrument_id(void)
{
   int uid = 0, cid = 0;
   int inst_id1, inst_id2, inst_id3;	/* values of the inst ID switches */
   static int notify = -1;		/* should I notify user of bad ID? */
   int notify_rate = 60;		/* how often should I notify? */
   
   if(semTake(semSDSSDC, 100) == ERROR) {
      return(-1);			/* unknown */
   }
   
   inst_id1 = (((sdssdc.status.i1.il8.inst_id1_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id1_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id1_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id1_4 ? 0 : 1) << 0));
   inst_id2 = (((sdssdc.status.i1.il8.inst_id2_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id2_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id2_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id2_4 ? 0 : 1) << 0));
   inst_id3 = (((sdssdc.status.i1.il8.inst_id3_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id3_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id3_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id3_4 ? 0 : 1) << 0));

   semGive(semSDSSDC);

   if((inst_id1 == inst_id2 && inst_id1 == inst_id3) /* consistent */ || pri_latch_opn /* not latched */) {
      if (notify >= 0) {		/* we were inconsistent */
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id1, inst_id2, inst_id3);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
      }
      notify = -1;
   } else {
      notify = (notify + 1)%notify_rate;

      if(notify == 0) {
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id1, inst_id2, inst_id3);
	 NTRACE_1(2, uid, cid, "Inconsistent instrument ID switches: %s", buff);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
	 
	 return(-1);
      }
   }
   
   return(inst_id1);
}
#endif

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
  
   if(semTake(semSDSSDC, 100) == ERROR) {
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
 * Report the position of the APOGEE gang connector.
 */
static int apogee_gang_position(void)
{
  int gangPosition = -1;

  if(semTake(semSDSSDC, 100) == ERROR) {
      return(-1);			/* unknown */
   }
  
  /* 0 unknown
   * 1 unplugged
   * 2 at cart
   * 4 any port in podium (always set if any of next 3 are set)
   * 8 at dense port in podium
   * 16 at sparse port in podium
   * 32 at 1m port in podium
   */
  gangPosition = ((sdssdc.status.b3.l7.apogee_gc_unplugged) |
                  (sdssdc.status.b3.l7.apogee_gc_at_cart << 1) |
                  (sdssdc.status.b3.l7.apogee_gc_at_stow << 2) |
                  (sdssdc.status.b3.l7.apogee_gc_at_dense << 3) |
                  (sdssdc.status.b3.l7.apogee_gc_at_sparse << 4) |
                  (sdssdc.status.b3.l7.apogee_gc_at_1m << 5));

  semGive(semSDSSDC);

  return(gangPosition);
}

/*****************************************************************************/
/*
 * Report the status of instruments.
 */
void
broadcast_inst_status(int uid, unsigned long cid)
{
   int inconsistent = 0;
   int inst_id = instrument_id(NULL, &inconsistent);
   int apogee_gang = apogee_gang_position();

#if 0					/* imager is no-longer used */
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "saddleIsMounted", saddle_is_mounted());
#endif

   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "instrumentNumConsistent", !inconsistent);
   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "instrumentNum", inst_id);

   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "apogeeGang", apogee_gang);
}

int
get_inststatus(char *status_ans,
	       int size)			/* dimen of status_ans */
{
   int len;

   sprintf(status_ans, "Inst: %d %d\n",
	   saddle_is_mounted(), instrument_id(NULL, NULL));

   len = strlen(status_ans);
   assert(len < size);
   
   return(len);
}
