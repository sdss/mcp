#if !defined(MCP_FIDUCIALS_H)
#define MCP_FIDUCIALS_H

#define ROT_FID_BIAS 40000000		/* "bias" added to rotator fiducial
					   values to make them positive */
#define N_ALT_FIDUCIALS 7		/* number of altitude fiducials + 1 */
#define N_AZ_FIDUCIALS 49		/* number of azimuth fiducials + 1 */
#define N_ROT_FIDUCIALS 156		/* number of rotator fiducials + 1 */
/*
 * What we know about an axis's fiducials.  The arrays of length 3 are for
 * encoders 1 and 2; [0] is unused
 */
struct FIDUCIARY {
   int seen_fiducial;			/* have we seen any fiducials? */
   int ms_on;				/* has the TCC asserted MS.ON for
					   this axis? */
   long error[3];			/* positional errors derived from
					   fiducial crossings */
   long max_correction;			/* |max. error| to correct */
   long last_latch;			/* last latch seen */
   double scale[3];			/* scales for encoders 1 and 2 */
   double scale_ratio_12;		/* == scale[1]/scale[2] */
   long min_encoder_mismatch;		/* max. allowed mismatch between
					   encoders 1 and 2 */
   int canonical;			/* canonical fiducial for this axis;
					   the one where the encoders agree */
   int canonical_position;		/* encoder value at canonical fiducl */
};

/*
 * N.b. there are only two fiducials, 1 and 2, so e.g. last[0] is unused
 */
struct FIDUCIALS {
   int markvalid;			/* is this valid? */
   int disabled;			/* is this fiducial disabled? */
   long mark[3];			/* positions the last time we crossed
					   a fiducial */
   long last[3];			/* previous values of mark */
   long fiducial[3];			/* fiducial positions */
   long poserr[3];			/* errors relative to knowns fiducial*/
   long err[3];				/* errors relative to last time we
					   saw this fiducial */
};

extern SEM_ID semLatch;			/* semaphore for the fiducials */
extern struct FIDUCIARY fiducial[3];
extern int fiducialidx[3];
extern struct FIDUCIALS az_fiducial[N_AZ_FIDUCIALS];
extern struct FIDUCIALS alt_fiducial[N_ALT_FIDUCIALS];
extern struct FIDUCIALS rot_fiducial[N_ROT_FIDUCIALS];

#define FIDVERLEN 20
extern char fiducialVersion[3][FIDVERLEN]; /* Fiducial table CVS versions. */

extern unsigned long check_encoder_freq; /* frequency to check encoder slip */

/*****************************************************************************/

void write_fiducial_log(const char *type, int axis, int fididx, int t1, int t2,
			int pos1, int pos2,
			double arg0, double arg1, long iarg0, long iarg1);

int set_ms_on(int axis);
int set_ms_off(int axis, float delay);

char *read_fiducials(const char *file, int axis);
char *write_fiducials(const char *file, int axis);

int set_max_fiducial_correction(int axis, int max_correction);
int set_min_encoder_slip_error(int axis, int min_error);

char *correct_cmd(char *cmd);

#endif
