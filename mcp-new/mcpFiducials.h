#if !defined(MCP_FIDUCIALS_H)
#define MCP_FIDUCIALS_H

#define ROT_FID_BIAS 40000000		/* "bias" added to rotator fiducial
					   values to make them positive */
#define N_ALT_FIDUCIALS 7		/* number of altitude fiducials */
#define N_AZ_FIDUCIALS 48		/* number of azimuth fiducials */
#define N_ROT_FIDUCIALS 156		/* number of rotator fiducials */

struct FIDUCIARY {
   int markvalid;			/* is mark valid? */
   long mark;				/* position of axis when fiducial
					   "index" was seen */
   int index;				/* the canonical index */
   long known_position;			/* correct position of fiducial */
   int ms_on;				/* has the TCC asserted MS.ON for
					   this axis? */
   long error;				/* positional error derived from
					   fiducial crossings */
   long max_correction;			/* |max. error| to correct */
};	

struct FIDUCIALS {
   int markvalid;			/* is this valid? */
   long mark;				/* position the last time we crossed
					   a fiducial */
   long last;				/* previous value of mark */
   long poserr;				/* error relative to "correct" fid. */
   long err;				/* error relative to last time we
					   saw this fiducial */
};

extern struct FIDUCIARY fiducial[3];
extern int fiducialidx[3];
extern struct FIDUCIALS az_fiducial[N_AZ_FIDUCIALS];
extern struct FIDUCIALS alt_fiducial[N_ALT_FIDUCIALS];
extern struct FIDUCIALS rot_fiducial[N_ROT_FIDUCIALS];

void latchexcel(int axis);
void set_primary_fiducials(int axis, int fididx, long pos);
int set_fiducials(int axis);
void set_fiducials_all(void);
int save_fiducials(int axis);
void save_fiducials_all(void);
void restore_fiducials_all(void);
char *read_fiducials(const char *file, int axis);
void tm_print_fiducial_all(void);
int set_max_fiducial_correction(int axis, int max_correction);

char *correct_cmd(char *cmd);

#endif
