/****************************************************************************/
/*                                                                          */
/*    SSSSSS  Y       Y  SSSSSS  TTTTTTTTT RRRRRR        A      N        N  */
/*   S      S  Y     Y  S      S     T     R     R      A A     NN       N  */
/*   S          Y   Y   S            T     R      R    A   A    N N      N  */
/*    S          Y Y     S           T     R     R    A     A   N  N     N  */
/*     SSS        Y       SSS        T     R    R    A       A  N   N    N  */
/*        S       Y          S       T     RRRRR     AAAAAAAAA  N    N   N  */
/*         S      Y           S      T     R    R    A       A  N     N  N  */
/*          S     Y            S     T     R     R   A       A  N      N N  */
/*   S      S     Y     S      S     T     R      R  A       A  N       NN  */
/*    SSSSSS      Y      SSSSSS      T     R      R  A       A  N        N  */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                     This software was developed by:                      */
/*                                                                          */
/*                              SYSTRAN Corp.                               */
/*                            4126 Linden Ave.                              */
/*                         Dayton, Ohio 45432-3066                          */
/*                             (513) 252-5601                               */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  Procedure Name: gendefs                                                 */
/*                                                                          */
/*  Description: This include file contains general definitions.            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                           Revision History                               */
/*                                                                          */
/*  Date      Name            Reason                                        */
/*  --------  --------------  --------------------------------------------- */
/*  12-27-93  Mark Rowe       Original Release                              */
/*                                                                          */
/****************************************************************************/

#ifndef ERROR
#define ERROR -1
#endif
#define E_ILLARG	-2
#define E_BUSERR	-3
#if !defined(NULL)
#  define NULL 		0
#endif
typedef unsigned char u_int8;
typedef unsigned short u_int16;
/*
typedef char int8;
typedef short int16;
*/
#define MAX_IP		8
#define MAX_DIO316	8
struct DIO316 {
	union {
		unsigned short rg16[12];
		unsigned char rg8[24];
		}REG;
	unsigned short reserved;
	unsigned short unused[0x33];
	unsigned short ID[12];                                                       
};
#define MAX_DAC128V	8
struct DAC128V {
	union {
		unsigned short rg16[8];
		unsigned char rg8[16];
		}REG;
	unsigned short reserved;
	unsigned short unused[0x37];
	unsigned short ID[12];                                                       
};
#define MAX_ADC128F1	8
struct ADC128F1 {
	union {
		unsigned short rg16[10];
		unsigned char rg8[20];
		}REG;
	unsigned short reserved;
	unsigned short unused[0x35];
	unsigned short ID[12];                                                       
};
#define MAX_DID48	8
struct DID48 {
	union {
		unsigned short rg16[8];
		unsigned char rg8[16];
		}REG;
	unsigned short reserved;
	unsigned short unused[0x37];
	unsigned short ID[12];                                                       
};
