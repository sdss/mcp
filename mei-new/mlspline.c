/*
	mlspline.c
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#include "idsp.h"
#ifdef ALLOC
#  include <alloc.h>
#else
#  include <stdlib.h>
#endif

int16 FNTYPE start_spline_motion(int16 n_axes, int16 *axis_map)
{  return reset_gates(n_axes, axis_map);
}

void LOCAL_FN frame_gen(long t, double x, double v, double a, double j, FRAME * f)
{
   LFIXED lfixed;

   frame_clear(f);

   ipcdsp_fixed(j, &lfixed);
	copy_lfixed_to_fixd(f->f.jerk, lfixed);
	ipcdsp_fixed(a, &lfixed);
	copy_lfixed_to_fixd(f->f.accel, lfixed);
	ipcdsp_fixed(v, &lfixed);
	copy_lfixed_to_fixd(f->f.velocity, lfixed);
	ipcdsp_fixed(x, &lfixed);
	copy_lfixed(f->f.position, lfixed);
	f->f.time = t;
	f->f.trig_update = FUPD_POSITION | FUPD_VELOCITY | FUPD_JERK |
                                                FUPD_ACCEL | FTRG_TIME;
   f->f.trig_action = NEW_FRAME;
}

/* input arrays have n points */
/* minimum distance between x points is 4 counts */

int16 LOCAL_FN spline(long *x, long *y, int16 n, FRAME *f[], double gamma, int16 end_f)
{
   int16      i;
   long     t;
   double   xl, xm, xr, yr, ym, yl;
   double   sig, p, y2r, y2m, y2l, um, ul, h;
   double   *u, *y2;
   double   xx, vv, aa, jj;
   FRAME    *ff;

   y2 = (double *) calloc(n, sizeof(double));
   if(y2 == NULL) return -1;

   u = (double *) calloc(n, sizeof(double));
   if(u == NULL) return -1;

   ff = *f;

   xl = x[0];
   yl = y[0];
   xm = x[1];
   ym = y[1];

   if(end_f)
   {
      y2l = -0.5;          /* v = 0 boundary conditions */
      ul = 3.0*(ym-yl)/((xm-xl)*(xm-xl));
   }
   else
   {
      y2l = 0.0;           /* "natural" boundary conditions */
      ul = 0.0;
   }

   y2[0] = y2l;
   u[0] = ul;

   for(i = 1; i < n-1; i++)
   {
      xr = x[i+1];
      yr = y[i+1];
      if(xr == xm) return -1;
      if(xl == xm) return -1;
      if(xl == xr) return -1;
      sig = (xm - xl)/(xr - xl);
      p = sig * y2l + 2.0;
      y2m = (sig - 1.0)/p;
      um = (yr - ym)/(xr - xm) -
                  (ym - yl)/(xm - xl);
      um = (6.0 * um / (xr - xl) - sig * ul) / p;
      u[i] = um;
      ul = um;
      y2[i] = y2m;
      y2l = y2m;
      xl = xm;
      xm = xr;
      yl = ym;
      ym = yr;
   }

   if(end_f)
   {
      u[n-1] = -3.0*(y[n-1] - y[n-2])/((x[n-1] - x[n-2])*(x[n-1] - x[n-2]));
      y2r = (u[n-1] - 0.5*u[n-2])/(0.5*y2[n-2] + 1.0);
   }
   else
   {
      y2r = 0.0;           /* "natural" boundary conditions */
      u[n - 1] = 0.0;
   }
   y2[n-1] = y2r;

   for(i = n-2; i >= 0; i--)
   {
      y2[i] = y2[i] * y2r + u[i];
      y2r = y2[i];
   }

   for(i = 0; i < n-1; i++)
   {
      double s;

      t = x[i+1] - x[i];
      h = t;
      if(h == 0.0) return -1;
      s = 1.0/h;

      y2l = y2[i];
      y2r = y2[i+1];

      xx = y[i];
      vv = (y[i+1] - xx) * s
                  - ((2.0 * h + 3.0 + s) * y2l - (s - h) * y2r) * gamma/6.0;
      jj = (y2r - y2l) * s * gamma;
      aa = y2l * gamma - jj;

      frame_gen(t, xx, vv, aa, jj, &ff[i]);
   }
   frame_gen(1L, y[n-1], 0.0, 0.0, 0.0, &ff[n-1]);
   free(y2);
   free(u);
   return 0;
}

/* input arrays have n points where x[n-1] - x[0] = cycle length */
/* input arrays have n points where y[n-1] ==  y[0] */
/* minimum distance between x points is 4 counts */

int16 LOCAL_FN p_spline(long * xd, long * yd, int16 n, FRAME *f[], double gamma)
{
   int16      i;
   long     t;
   double   xl, xm, xr, yr, ym, yl;
   double   sig, p, y2r, y2m, y2l, um, ul, h;
   double   *x, *y, *u, *y2;
   double   xx, vv, aa, jj;
   FRAME    *ff;

   x = (double *) calloc(n+8, sizeof(double));
   if(x == NULL) return -1;

   y = (double *) calloc(n+8, sizeof(double));
   if(y == NULL) return -1;

   y2 = (double *) calloc(n+8, sizeof(double));
   if(y2 == NULL) return -1;

   u = (double *) calloc(n+8, sizeof(double));
   if(u == NULL) return -1;
    
   ff = *f;

   for(i = 0; i < 4; i++)
   {
      x[i] = xd[0] - (xd[n-1] - xd[n-5+i]);
      x[n+4+i] = xd[n-1] + (xd[i+1] - xd[0]);
      y[i] = yd[n-5+i];
      y[n+4+i] = yd[i+1];
   }
   for(i = 0; i < n; i++)
   {
      x[i+4] = xd[i];
      y[i+4] = yd[i];
   }
   xl = x[n+7];
   yl = y[n+7];
   xm = x[0];
   ym = y[0];
   y2l = 0.0;
   ul = 0.0;

   for(i = 0; i < n+7; i++)
   {
      xr = x[i+1];
      yr = y[i+1];
      if(xr == xm) return -1;
      if(xl == xm) return -1;
      if(xl == xr) return -1;
      sig = (xm - xl)/(xr - xl);
      p = sig * y2l + 2.0;
      y2m = (sig - 1.0)/p;
      um = (yr - ym)/(xr - xm) -
            (ym - yl)/(xm - xl);
      um = (6.0 * um / (xr - xl) - sig * ul) / p;
      u[i] = um;
      ul = um;
      y2[i] = y2m;
      y2l = y2m;
      xl = xm;
      xm = xr;
      yl = ym;
      ym = yr;
   }
   y2r = 0.0;

   for(i = n+7; i >= 0; i--)
   {
      y2[i] = y2[i] * y2r + u[i];
      y2r = y2[i];
   }
   for(i = 0; i < n - 1; i++)
   {
      double s;

      t = (long)(x[i+5] - x[i+4]);
      h = t;
      if(h == 0.0) return -1;
      s = 1.0/h;
      y2l = y2[i+4];
      y2r = y2[i+5];
      xx = y[i+4];
      vv = (y[i+5] - xx) * s
            - ((2.0 * h + 3.0 + s) * y2l - (s - h) * y2r) * gamma/6.0;
      jj = (y2r - y2l) * s * gamma;
      aa = y2l * gamma - jj;
      frame_gen(t, xx, vv, aa, jj, &ff[i]);
   }
   free(x);
   free(y);
   free(y2);
   free(u);
   return 0;
}

int16 FNTYPE load_spline_motion(int16 n_axes, int16 *axis_map, int16 n_points, 
                        long *(*point_list), double *g, int16 *end_flag)
{
   int16      i, j, axis;
   FRAME    *f;

   f = (FRAME *) calloc(n_points, sizeof(FRAME));
   if(f == NULL) return -1;
   for(i = 0; i < n_axes; i++)
   {
      spline(point_list[0], point_list[i+1], n_points, &f, g[i], end_flag[i]);

      axis = axis_map[i];
      dsp_control(axis, FCTL_HOLD,TRUE);
      set_gate(axis);
      frame_allocate(&f[0], dspPtr, axis);
      frame_download(&f[0]);
      dsp_control(axis, FCTL_HOLD, FALSE);
      for(j = 1; j < n_points - 1; j++)
      {
         frame_allocate(&f[j], dspPtr, axis);
         frame_download(&f[j]);
      }
      frame_allocate(&f[n_points - 1], dspPtr, axis);
      f[n_points-1].f.trig_action = 0;
      f[n_points-1].f.trig_update = FUPD_POSITION | FUPD_VELOCITY | FUPD_JERK 
                                                                  | FUPD_ACCEL;
      frame_download(&f[n_points - 1]);
   }
   free((double *)f);
   return dsp_error;
}

int16 FNTYPE load_periodic_motion(int16 n_axes, int16 *axis_map, int16 n_points, long *(*point_list), double *g, DSP_DM *last_frame)
{
   int16      i, j, frame_start, axis;
   FRAME    *f;

   f = (FRAME *) calloc(n_points-1, sizeof(FRAME));
   if(f == NULL) return -1;
   for(i = 0; i < n_axes; i++)
   {
      p_spline(point_list[0], point_list[i+1], n_points, &f, g[i]);

      axis = axis_map[i];
      dsp_control(axis, FCTL_HOLD, TRUE);
      dsp_control(axis, FCTL_RELEASE, FALSE);
      set_gate(axis);
      frame_allocate(&f[0], dspPtr, axis);
      frame_start = f[0].current;
      frame_download(&f[0]);
      dsp_control(axis, FCTL_HOLD, FALSE);
      for(j = 1; j < n_points - 2; j++)
      {
         frame_allocate(&f[j], dspPtr, axis);
         frame_download(&f[j]);
      }
      frame_allocate(&f[n_points - 2], dspPtr, axis);
      f[n_points-2].f.next = frame_start;
      last_frame[i] = f[n_points - 2].current;
      frame_download(&f[n_points - 2]);
      dsp_control(axis, FCTL_RELEASE, TRUE);
   }
   free((double *)f);
   return dsp_error;
}

