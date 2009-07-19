/*
 *	Broadcast a sdss frame as described in data_collection.h at 1 Hz.
 */
#include "vxWorks.h"
#include <stdio.h>
#include <time.h>
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "etherLib.h"
#include "ifLib.h"
#include "string.h"
#include "inetLib.h"
#include "errnoLib.h"
#include "sockLib.h"
#include "socket.h"
#include "axis.h"
#include "data_collection.h"
#include "dscTrace.h"
#include "as2.h"

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static struct sockaddr_in cast_sockaddr;
static int cast_s=-1;
static char cast_adr[12];


/*****************************************************************************/
/*
 * Initialize socket for datagram broadcast
 *
 * Returns 0 or ERROR
 */
int
ipsdss_ini(void)
{
   int uid = 0, cid = 0;   
   int optval;
   int cast_port=0x6804;
   
   cast_s = socket(AF_INET, SOCK_DGRAM, 0);	/* get a udp socket */
   if(cast_s < 0) {
      NTRACE_2(0, uid, cid, "ipsdss_ini: creating socket: %d %s", errno, strerror(errno));
      return ERROR;
   }
   
   bzero ((char *)&cast_sockaddr, sizeof (cast_sockaddr));
   cast_sockaddr.sin_family      = AF_INET;
   cast_sockaddr.sin_port        = htons(cast_port);
   ifBroadcastGet("ei0",&cast_adr[0]);
   cast_sockaddr.sin_addr.s_addr=inet_addr(&cast_adr[0]);
   
   optval = 1;				/* turn on broadcast */
   if(setsockopt(cast_s, SOL_SOCKET, SO_BROADCAST,
		 (caddr_t)&optval, sizeof(optval)) < 0) {
      NTRACE_2(0, uid, cid, "ipsdss_ini: setsockopt: %d %s", errno, strerror(errno));
      return ERROR;
   }
   
   return 0;
}

/*****************************************************************************/
/*
 * Broadcast a message
 */
void
ipsdss_send(char *sdss_msg,		/* message to broadcast */
	    int sdss_size)		/* length of message */
{
    int uid = 0, cid = 0;   

    if(cast_s < 0) {			/* socket isn't open */
       return;
    }
    
    if(sendto(cast_s, sdss_msg, sdss_size, 0,
	      (struct sockaddr *)&cast_sockaddr, sizeof(cast_sockaddr)) < 0) {
       NTRACE_2(0, uid, cid, "couldn't broadcast sdss_msg: %d %s",
		errno, strerror(errno));
    }
}

/*****************************************************************************/
/*
 * Broadcast values extracted from SDSS_FRAME
 *
 * N.b. This code isn't machine generated from PLC's sdss.csv;  it doesn't need
 * to know about particular bitfields, so it didn't seem worth it
 */

void
init_broadcast_ipsdss() {
   declareKeyword("ab_status", integer, 0, "Status reading PLC bits");

   declareKeyword("ab_I1_L0", hex, 0, "");
   declareKeyword("ab_I1_L1", hex, 0, "");
   declareKeyword("ab_I1_L2", hex, 0, "");
   declareKeyword("ab_I1_L3", hex, 0, "");
   declareKeyword("ab_I1_L4", hex, 0, "");
   declareKeyword("ab_I1_L5", hex, 0, "");
   declareKeyword("ab_I1_L6", hex, 0, "");
   declareKeyword("ab_I1_L7", hex, 0, "");
   declareKeyword("ab_I1_L8", hex, 0, "");
   declareKeyword("ab_I1_L9", hex, 0, "");
   declareKeyword("ab_I1_L10", hex, 0, "");
   declareKeyword("ab_I1_L11", hex, 0, "");
   declareKeyword("ab_I1_L12", hex, 0, "");
   declareKeyword("ab_I1_L13", hex, 0, "");
   declareKeyword("ab_I1_L14", hex, 0, "");
   declareKeyword("ab_I1_L15", hex, 0, "");
   declareKeyword("ab_I2_L0", hex, 0, "");
   declareKeyword("ab_I6_L0", hex, 0, "");
   declareKeyword("ab_I7_L0", hex, 0, "");
   declareKeyword("ab_I8_L0", hex, 0, "");
   declareKeyword("ab_I9_L0", hex, 0, "");
   declareKeyword("ab_I10_L0", hex, 0, "");
   declareKeyword("ab_O1_L1", hex, 0, "");
   declareKeyword("ab_O1_L5", hex, 0, "");
   declareKeyword("ab_O1_L6", hex, 0, "");
   declareKeyword("ab_O1_L9", hex, 0, "");
   declareKeyword("ab_O1_L10", hex, 0, "");
   declareKeyword("ab_O1_L14", hex, 0, "");
   declareKeyword("ab_O2_L0", hex, 0, "");
   declareKeyword("ab_O11_L0", hex, 0, "");
   declareKeyword("ab_O12_L0", hex, 0, "");
}

/*
 * Actually do the broadcast
 *
 * No machine generated code here either
 */
void
broadcast_ipsdss(int uid,	   /* user ID */
		 unsigned long cid) /* command ID */
{
   struct AB_SLC500 *ab = &sdssdc.status;
   /*
    * Fillout the bitfield that gives us position-dependent booleans that the interlocks use
    */
   I10_L0 plcBools;
   const double alt_position = convert_clinometer(ab->i4.alt_position);

   *(int *)&plcBools = 0x0;
   plcBools.alt_position_lt_90_15 = (alt_position < 90.15) ? 1 : 0;
   plcBools.alt_position_gt_89_75 = (alt_position > 89.75) ? 1 : 0;
   plcBools.alt_position_lt_90_2 =  (alt_position < 90.2)  ? 1 : 0;
   plcBools.alt_position_lt_90_29 = (alt_position < 90.29) ? 1 : 0;
   plcBools.alt_position_lt_91_0 =  (alt_position < 91.0)  ? 1 : 0;
   plcBools.alt_position_gt_89_8 =  (alt_position > 89.8)  ? 1 : 0;
   plcBools.alt_position_gt_0_50 =  (alt_position > 0.50)  ? 1 : 0;
   plcBools.alt_position_gt_19_5 =  (alt_position > 19.5)  ? 1 : 0;
   plcBools.alt_position_gt_0_8 =   (alt_position > 0.8)   ? 1 : 0;
   plcBools.alt_position_lt_18_5 =  (alt_position < 18.5)  ? 1 : 0;
   plcBools.alt_position_gt_15_0 =  (alt_position > 15.0)  ? 1 : 0;
   plcBools.alt_position_gt_15_5 =  (alt_position > 15.5)  ? 1 : 0;
   plcBools.alt_position_gt_83_5 =  (alt_position > 83.5)  ? 1 : 0;
   plcBools.alt_pos_lt_0_2 =        (alt_position < 0.2)   ? 1 : 0;
   plcBools.alt_pos_gt_neg_2 =      (alt_position > -0.2)  ? 1 : 0;

   plcBools.umbilical_dn = (ab->i4.umbilical_dist < 480) ? 1 : 0;
   plcBools.lift_force_gt_f_cartridge_mount =  (ab->i4.inst_lift_force > 1400)  ? 1 : 0;
   plcBools.lift_height_gt_h_cartridge_mount = (ab->i4.inst_lift_dist >= 21.95) ? 1 : 0;
   /*
    * Send out the desired fields
    */
   {
      char buff[40];
      sprintf(buff, "%d, %ld, %ld, %ld", ab->status,
	      sdssdc.axis_state[AZIMUTH], sdssdc.axis_state[ALTITUDE], sdssdc.axis_state[INSTRUMENT]);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "ab_status", buff);
   }
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L0", *(int *)&ab->i1.il0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L1", *(int *)&ab->i1.il1);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L2", *(int *)&ab->i1.il2);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L3", *(int *)&ab->i1.il3);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L4", *(int *)&ab->i1.il4);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L5", *(int *)&ab->i1.il5);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L6", *(int *)&ab->i1.il6);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L7", *(int *)&ab->i1.il7);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L8", *(int *)&ab->i1.il8);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L9", *(int *)&ab->i1.il9);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L10", *(int *)&ab->i1.il10);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L11", *(int *)&ab->i1.il11);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L12", *(int *)&ab->i1.il12);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L13", *(int *)&ab->i1.il13);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L14", *(int *)&ab->i1.il14);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I1_L15", *(int *)&ab->i1.il15);
   
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I2_L0", *(int *)&ab->i2.il0);
   /* ab_I2_L[1-7] and ab_I3_L[0-3] are az/alt drive values (as 16-bit ints), and DCM status and spares */
   /* ab_I4_L[0-3] and ab_I5_[0-3] are rotator/umbilical/lift/counterweight values and spares */

   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I6_L0", *(int *)&ab->i6.il0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I7_L0", *(int *)&ab->i7.il0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I8_L0", *(int *)&ab->i8.il0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I9_L0", *(int *)&ab->i9.il0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_I10_L0", *(int *)&plcBools);
   /* ab_O1_L0 is unused */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L1", *(int *)&ab->o1.ol1);
   /* ab_O1_L[2-4] are all spares */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L5", *(int *)&ab->o1.ol5);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L6", *(int *)&ab->o1.ol6);
   /* ab_O1_L[7-8] are all spares */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L9", *(int *)&ab->o1.ol9);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L10", *(int *)&ab->o1.ol10);
   /* ab_O1_L1[1-3] are all spares */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O1_L14", *(int *)&ab->o1.ol14);
   /* ab_O1_L1[5] is all spares */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O2_L0", *(int *)&ab->o2.ol0);
   /* ab_O[3-5]_L[0-3] are all short int position/current/voltage/umbilical/lift/cw values */
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O11_L0", *(int *)&ab->o11.ol0);
   sendStatusMsg_X(uid, cid, INFORMATION_CODE, 1, "ab_O12_L0", *(int *)&ab->o12.ol0);
}
