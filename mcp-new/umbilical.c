#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      umbilical.c
**
** ABSTRACT:
**	Keep the umbilical cord to the mosaic camera off the floor as 
**	determined by alt/az position.
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
** getumbiltab		public	initialize the table from a file
** print_umbiltab()	public	diagnostic to print table
** umbil/umbilGet	public	get the umbilical position given az/alt position
** test_umbil		public	test
**
** ENVIRONMENT:
**      ANSI C.
**
** REQUIRED PRODUCTS:
**
** AUTHORS:
**      Creation date:  Aug 30, 1999
**      Charlie Briegel
**
***************************************************************************
***************************************************************************/
/*
Here are the umbilicus elevator tables (1) in inches of elevation above
the lower limit, and (2) string-pot encoder counts, assuming that fully
retracted is zero, fully extended is 30 inches and 1000 counts.  The
tables are for every 5 degrees of elevation from 0 to 90 and 10 degrees
of rotator angle from -180 to 180; bilinear interpolation in them is
more than sufficiently accurate.  If you would like tables with other
increments, I can easily (now) make them, or if you prefer, I could with
a little effort give you a C function which calculates the elevator
setting (I have code, but it is written in Forth; I would just have to
translate it.) Be careful with wrap; the tables go from -180 to 180, but
all values are modulo 360, of course; +270 would use the entry for -90
in the table. Attached at the end is a C function to interpolate in the
table. It runs successfully on both 16-bit and 32-bit machines.

        TABLE IN INCHES OF ELEVATOR HEIGHT ABOVE LOWER LIMIT

ro\el  0.0  5.0 10.0 15.0 20.0 25.0 30.0 35.0 40.0 45.0 50.0 55.0 60.0 65.0 70.0 75.0 80.0 85.0 90.0
 -180 13.8 17.1 20.1 22.6 24.7 26.3 27.4 27.9 27.8 27.0 25.7 23.8 21.6 19.0 16.1 12.9  9.5  5.8  1.9
 -170 15.4 18.7 21.6 24.0 26.0 27.6 28.6 29.1 29.1 28.3 26.9 25.1 22.8 20.2 17.2 14.0 10.5  6.9  3.0
 -160 16.2 19.4 22.3 24.7 26.7 28.2 29.2 29.8 29.8 29.2 27.9 26.1 23.9 21.3 18.4 15.3 11.8  8.2  4.4
 -150 16.1 19.4 22.2 24.7 26.7 28.2 29.3 29.9 30.0 29.6 28.6 27.0 25.0 22.5 19.7 16.7 13.3  9.8  6.0
 -140 15.2 18.5 21.4 23.9 26.0 27.6 28.8 29.5 29.8 29.6 28.9 27.7 25.9 23.6 21.0 18.1 14.9 11.5  7.9
 -130 13.5 16.9 19.9 22.5 24.7 26.5 27.8 28.7 29.2 29.2 28.8 28.0 26.6 24.7 22.3 19.7 16.7 13.5 10.0
 -120 11.1 14.5 17.7 20.4 22.8 24.8 26.4 27.6 28.3 28.6 28.5 27.9 26.9 25.5 23.5 21.2 18.5 15.5 12.3
 -110  8.0 11.6 14.9 17.9 20.5 22.8 24.6 26.1 27.1 27.7 27.9 27.7 27.1 26.0 24.6 22.7 20.3 17.7 14.7
 -100  4.5  8.3 11.8 15.0 17.9 20.4 22.5 24.3 25.7 26.7 27.2 27.4 27.1 26.5 25.4 24.0 22.1 19.8 17.2
  -90  0.7  4.7  8.4 11.8 15.0 17.8 20.3 22.4 24.1 25.5 26.4 27.0 27.1 26.8 26.2 25.1 23.7 21.9 19.6
  -80  0.0  0.9  4.8  8.5 11.9 15.0 17.9 20.3 22.4 24.2 25.5 26.5 27.0 27.1 26.9 26.2 25.2 23.8 21.9
  -70  0.0  0.0  1.1  5.0  8.7 12.2 15.3 18.1 20.6 22.7 24.5 25.8 26.8 27.3 27.4 27.2 26.5 25.5 24.0
  -60  0.0  0.0  0.0  1.5  5.4  9.2 12.6 15.8 18.6 21.1 23.3 25.0 26.3 27.3 27.8 27.9 27.6 26.9 25.8
  -50  0.0  0.0  0.0  0.0  2.1  6.0  9.8 13.3 16.4 19.3 21.8 23.9 25.7 27.0 27.9 28.4 28.4 28.1 27.3
  -40  0.0  0.0  0.0  0.0  0.0  2.8  6.8 10.5 14.0 17.2 20.1 22.6 24.7 26.4 27.6 28.4 28.8 28.8 28.3
  -30  0.0  0.0  0.0  0.0  0.0  0.0  3.8  7.7 11.5 14.9 18.1 20.9 23.3 25.3 26.9 28.1 28.8 29.0 28.8
  -20  0.0  0.0  0.0  0.0  0.0  0.0  0.8  4.9  8.8 12.5 15.9 19.0 21.7 24.0 25.8 27.2 28.2 28.7 28.8
  -10  0.0  0.0  0.0  0.0  0.0  0.0  0.0  2.3  6.3 10.1 13.7 16.9 19.8 22.3 24.4 26.0 27.2 27.9 28.1
    0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  4.1  8.0 11.6 14.9 17.9 20.5 22.7 24.5 25.8 26.6 27.0
   10  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  2.2  6.1  9.7 13.1 16.1 18.8 21.0 22.8 24.1 25.0 25.4
   20  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.8  4.7  8.2 11.5 14.5 17.1 19.3 21.0 22.3 23.2 23.6
   30  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  3.7  7.2 10.3 13.1 15.6 17.7 19.3 20.5 21.2 21.4
   40  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  3.3  6.5  9.5 12.1 14.4 16.2 17.7 18.7 19.1 19.0
   50  1.2  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  3.3  6.4  9.1 11.5 13.5 15.0 16.2 16.9 17.0 16.6
   60  1.4  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.8  3.9  6.7  9.1 11.2 12.8 14.1 14.9 15.2 14.9 14.1
   70  0.3  0.0  0.0  0.0  0.0  0.0  0.0  0.0  2.2  5.0  7.4  9.5 11.2 12.5 13.4 13.7 13.5 12.8 11.7
   80  0.0  0.0  0.0  0.0  0.0  0.0  0.0  1.2  4.0  6.5  8.6 10.3 11.6 12.5 12.8 12.6 12.0 10.9  9.5
   90  0.0  0.0  0.0  0.0  0.0  0.0  0.9  3.7  6.2  8.3 10.0 11.4 12.2 12.5 12.4 11.7 10.7  9.2  7.4
  100  0.0  0.0  0.0  0.0  0.0  1.2  4.1  6.5  8.7 10.4 11.8 12.6 12.9 12.7 12.1 11.0  9.5  7.7  5.6
  110  0.0  0.0  0.0  0.0  1.9  4.8  7.4  9.6 11.4 12.8 13.6 14.0 13.8 13.1 12.0 10.5  8.6  6.5  4.0
  120  0.0  0.0  0.0  2.9  5.9  8.5 10.8 12.7 14.2 15.1 15.5 15.3 14.7 13.6 12.1 10.2  8.0  5.5  2.7
  130  0.0  0.4  3.8  7.0  9.8 12.2 14.2 15.8 17.0 17.4 17.4 16.8 15.7 14.2 12.3 10.1  7.6  4.8  1.7
  140  0.9  4.6  7.9 10.9 13.5 15.8 17.5 18.9 19.6 19.7 19.2 18.2 16.8 14.9 12.8 10.3  7.5  4.4  1.1
  150  4.9  8.5 11.7 14.6 17.0 19.0 20.6 21.6 22.0 21.8 20.9 19.7 17.9 15.8 13.4 10.6  7.6  4.3  0.7
  160  8.4 11.9 15.1 17.8 20.1 21.9 23.3 24.1 24.2 23.7 22.6 21.1 19.1 16.8 14.1 11.2  8.0  4.5  0.8
  170 11.4 14.8 17.9 20.5 22.7 24.4 25.6 26.2 26.2 25.5 24.2 22.5 20.4 17.9 15.1 12.0  8.6  5.0  1.2
  180 13.8 17.1 20.1 22.6 24.7 26.3 27.4 27.9 27.8 27.0 25.7 23.8 21.6 19.0 16.1 12.9  9.5  5.8  1.9

To use the table below in my code, cut it out of here and make a text file
with the first line the header line which starts ro/el. Call the file
umbil.dat and put it in the same directory as the executable, or modify
the "datafile" variable in the code.

                TABLE IN STRING-POT ENCODER COUNTS 

ro\el  0   5  10  15  20  25  30  35  40  45  50  55  60  65  70  75  80  85  90
-180 458 570 668 753 822 874 910 930 926 899 854 793 718 632 535 429 315 193  64
-170 511 621 718 800 867 917 951 969 967 941 896 834 758 671 573 466 351 228  99
-160 538 646 742 823 889 939 973 990 991 971 930 870 796 710 614 508 394 272 144
-150 536 644 740 822 888 940 975 995 999 986 953 900 831 749 657 554 443 324 199
-140 506 615 712 796 865 920 959 983 992 985 963 921 861 787 700 603 497 383 263
-130 450 561 661 748 821 881 926 957 972 973 959 931 884 821 743 654 555 448 333
-120 369 483 587 680 759 826 879 917 942 952 947 929 896 848 784 705 616 517 409
-110 267 387 497 595 683 757 819 867 902 922 929 922 901 867 819 754 677 588 490
-100 151 276 392 499 594 678 750 809 855 887 906 911 903 881 846 799 736 659 572
 -90  24 155 279 393 498 592 675 745 803 848 879 898 902 894 872 837 790 728 653
 -80   0  28 159 282 396 500 594 676 747 804 849 880 898 903 895 873 838 791 730
 -70   0   0  36 167 290 404 509 603 686 756 814 859 891 909 913 904 882 847 800
 -60   0   0   0  49 180 304 420 525 620 703 774 832 877 908 925 929 919 896 860
 -50   0   0   0   0  68 200 325 441 547 642 726 797 854 898 928 944 946 934 909
 -40   0   0   0   0   0  93 226 351 467 573 668 751 821 877 919 946 959 958 943
 -30   0   0   0   0   0   0 125 257 381 497 602 695 776 843 896 934 957 966 959
 -20   0   0   0   0   0   0  27 163 294 416 529 631 721 797 859 907 939 955 957
 -10   0   0   0   0   0   0   0  76 210 337 455 563 659 743 812 866 905 928 936
   0   0   0   0   0   0   0   0   0 136 265 386 497 596 683 757 815 859 886 898
  10   0   0   0   0   0   0   0   0  74 203 324 435 536 624 699 759 803 833 846
  20   0   0   0   0   0   0   0   0  28 155 274 383 482 568 641 700 743 772 784
  30   0   0   0   0   0   0   0   0   0 123 238 343 437 519 588 642 682 706 712
  40   0   0   0   0   0   0   0   0   0 108 217 316 403 478 540 588 621 637 634
  50  40   0   0   0   0   0   0   0   0 110 212 302 381 448 500 539 562 566 552
  60  45   0   0   0   0   0   0   0  28 130 222 303 371 427 469 496 505 495 470
  70   9   0   0   0   0   0   0   0  73 165 246 316 373 416 445 456 450 427 391
  80   0   0   0   0   0   0   0  40 133 214 284 342 386 415 426 420 399 364 316
  90   0   0   0   0   0   0  31 124 206 276 333 378 407 417 411 390 354 306 247
 100   0   0   0   0   0  41 135 217 288 347 391 420 431 424 402 366 317 256 185
 110   0   0   0   0  64 160 245 318 378 424 454 464 458 435 398 349 287 215 133
 120   0   0   0  96 195 284 360 423 471 504 516 510 488 451 401 339 265 182  90
 130   0  12 127 232 325 406 474 527 564 580 578 557 522 472 410 336 252 159  57
 140  31 152 263 363 450 524 583 627 651 654 638 605 558 497 424 341 248 146  35
 150 163 281 389 484 566 633 685 720 733 724 697 654 597 526 445 353 252 143  24
 160 281 397 501 592 669 730 775 803 807 789 754 702 637 559 471 372 265 150  26
 170 380 494 595 682 755 811 851 873 872 848 806 749 678 595 501 398 286 166  39
 180 458 570 668 753 822 874 910 930 926 899 854 793 718 632 535 429 315 193  64
*/
/*
******************************************************************************
**********  INTERPOLATION AND DATA READING CODE ******************************
******************************************************************************
*/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include <stdio.h>

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
/*#define DEBUG*/
#define NROT 37
#define NEL  19
/* number of rot, elev entries in table */
#define ELINT 5
#define DELINV 0.2
#define ROTINT 10
#define DROTINV 0.1
/* intervals in degrees in table and inverses */

static char* datafile = "umbil.tab";

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
int *umbiltab[NEL];
int umbildat[NEL*NROT];
/* if you need to save space, these can be shorts; you will need to
 * modify the format in sscanf() in getumbiltab() to ("%d %hd %hd .... ) if
 * you do this
 */
int gotdata=0;

/*
	prototypes
*/
int getumbiltab();
void print_umbiltab();
int umbil(double el, double rot);
int umbilGet(int el, int rot);
int test_umbilical();
/********************** GETUMBILTAB() **************************************/
/*=========================================================================
**=========================================================================
**
** ROUTINE: getumbiltab
**
** DESCRIPTION:
**	This function reads the table file. The first line is the list of
**	elevation values; it is ignored, but must be there.
**      
**
** RETURN VALUES:
**      int 	ERROR or zero
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	umbiltab
**	umbildat
**	gotdata
**
**=========================================================================
*/
int getumbiltab()
{
    FILE *fp;
    char *res;
    int i,j;
    char line[100];
    int rot;

    /* open data file */
    fp = fopen(datafile,"r");
    if(fp == NULL){
        printf("Cannot open %s for reading\n",datafile);
        fflush(stdout);
        return -1;
    }
    
    /* set up matrix pointer array */
    umbiltab[0] = umbildat;
    for(j=1;j<NEL;j++){
        umbiltab[j] = umbiltab[j-1] + NROT;
    }

    /* get header line and throw it away */
    res = fgets(line,99,fp);
        
    /* get data and stuff matrix */
    for(i=0;i<NROT;i++){
        res = fgets(line,99,fp);
        if(res == NULL){
            printf("\nPremature end-of-file in %s",datafile);
            fclose(fp);
            fflush(stdout);
            return -1;
        }
        sscanf(line,
            "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
            &rot,
            &umbiltab[0][i], &umbiltab[1][i], &umbiltab[2][i], &umbiltab[3][i],
            &umbiltab[4][i], &umbiltab[5][i], &umbiltab[6][i], &umbiltab[7][i],
            &umbiltab[8][i], &umbiltab[9][i], &umbiltab[10][i],&umbiltab[11][i],
            &umbiltab[12][i],&umbiltab[13][i],&umbiltab[14][i],&umbiltab[15][i],
            &umbiltab[16][i],&umbiltab[17][i],&umbiltab[18][i]);
        if(rot != -180 + 10*i){
            printf("Umbilicus data file corrupted: rot, expected = %d %d",
                rot, 180 + 10*i);
            fclose(fp);
            fflush(stdout);
            return -1;
        }
#ifdef DEBUG
        printf("%4d",rot); 
        for(j=0;j<NEL;j++) printf("%4d",umbiltab[j][i]);
        printf("\n");
#endif
    }
    fclose(fp);
    gotdata = 1;
    return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_umbiltab
**
** DESCRIPTION:
**	Diagnostic to print the umbilical table.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	umbiltab
**
**=========================================================================
*/
void print_umbiltab()            
{
    int i,j;

    for(i=0;i<NROT;i++){
        printf("%4d",i);
        for(j=0;j<NEL;j++) printf("%4d",umbiltab[j][i]);
        printf("\n");
    }
}
/********************** UMBIL() ****************************************/
/*=========================================================================
**=========================================================================
**
** ROUTINE: umbil
**	    umbilGet  specifies positions as longs
**
** DESCRIPTION:
**	Given the azimuth and altitude positions, returns the umbilical value.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	umbiltab
**
**=========================================================================
*/
/* 
 * this function returns the interpolated value of the target umbilicus 
 * elevator string pot setting for a proferred pair of elevation and 
 * rotator angle values (double, degrees; the rotator is zero at the
 * instrument change position, the elevation zero at the horizon.
 * the function returns -1 if there is an error (incorrect input or
 * data matrix not set up; ie getumbiltab() has not been run correctly)
 */
 
int umbil(double el, double rot)
{
    float alpha;
    float beta;
    int indel;
    float iel;
    int indrot;
    float irot;
    float res;
    
    if(gotdata == 0 || el < 0. || el > 90.) return (-1);
    /* get rotator variable in range (-180,180) */
    do{
        if(rot < -180.) rot += 360.;
        if(rot > 180.) rot -= 360.;
    }while( rot > 180. || rot < -180.);
    /* scale up float indices by SC and round; put rotator origin at -180 */
    irot = (rot+180.)*DROTINV;  /* float 'index' for rot */
    iel =  el*DELINV;  /* same for elevation */
    /* find lower indices in table */
    indrot = irot;
    indel  = iel;
    /* and 'fractional' parts */
    alpha = iel - (float)indel;
    beta  = irot - (float)indrot;

    /* fix bookkeeping if at end */    
    if(indrot == NROT-1){
        indrot--;
        beta += 1.;
    }
    if(indel == NEL-1){
        indel--;
        alpha += 1.;
    }  
    res =  ( alpha*beta          *umbiltab[indel+1][indrot+1] +  
             alpha*(1.-beta)     *umbiltab[indel+1][indrot]   +   
             (1.-alpha)*beta     *umbiltab[indel][indrot+1]   +
             (1.-alpha)*(1.-beta)*umbiltab[indel][indrot] );

#ifdef DEBUG
    printf(
"iel,indel,alpha=%4.2f %d %4.2f irot,indrot,beta=%4.2f %d %4.2f tab,r=%d %4.1f\n",
                iel,indel,alpha,irot,indrot,beta,umbiltab[indel][indrot],res);
    fflush(stdout);
#endif

    return (int)(res + 0.5);             
}
int umbilGet(int el, int rot)
{
  return (umbil((double)el, (double)rot));
}
#if 1
/*************************** MAIN() ****************************************/
/*=========================================================================
**=========================================================================
**
** ROUTINE: test_umbil
**
** DESCRIPTION:
**	Test umbilical positions for a queried set of az/alt positions.
**
** RETURN VALUES:
**      int	ERROR or zero
**
** CALLS TO:
**	umbil
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
/* test function */

int test_umbilical()
{
    double el,rot;
    int res;
    char line[100];

    if(getumbiltab() < 0){
        printf("%s","\nError in getumbiltab\n");
        exit(-1);
    }
    printf("%s","\nEnter an elevation, rotator pair (decimal degrees)\n");
    do{
        gets(line);
        sscanf(line,"%lf %lf",&el,&rot);
        if( el < 0.) break;
        res = umbil(el,rot);
        if(res < 0){
            printf("%s","\nError return from umbil()\n");
            return(-1);
        }
        printf("%5.1f %5.1f %5d\n",el,rot,res);
    }while(1);
    return 0;
}    
#endif
